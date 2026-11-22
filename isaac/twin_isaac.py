"""PHOTOREAL digital twin in NVIDIA Isaac Sim 6.2 (RTX real-time path tracing). READ-ONLY: no physics, no ROS, no
commands - it only draws what scripts/twin_isaac_bridge.py (system ROS side) sends.

  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim_env/bin/python isaac/twin_isaac.py [--shot out.png --shot-after 40 --exit]

Robots   the twin URDF (same as RViz) converted to USD by isaac/convert_twin.py; every moving link prim is posed
         kinematically from the measured joint positions (UDP 127.0.0.1:47811): local transform = URDF joint origin x
         rotation about the joint axis - exactly what robot_state_publisher does for RViz. Physics never runs.
Scene    the reconstruction mesh (isaac/cache/mesh_static.npz, from scripts/twin_recon.py) as a smooth vertex-coloured
         mesh, live objects (mesh_live.npz) as voxel cubes, both reloaded when the files change.
Look     studio lighting (dome + key light with soft shadows + fill), ACES tone mapping, floor, fixed hero camera.
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, 'cache')
ap = argparse.ArgumentParser()
ap.add_argument('--width', type=int, default=1920)
ap.add_argument('--height', type=int, default=1080)
ap.add_argument('--headless', action='store_true')
ap.add_argument('--x', type=int, default=1920, help='window position (display 2 starts at x 1920)')
ap.add_argument('--y', type=int, default=0)
ap.add_argument('--windowed', action='store_true', help='no full screen')
ap.add_argument('--title', default='FR3 Duo Photoreal Twin')
ap.add_argument('--stream', action='store_true', help='render headless into an MJPEG stream for the browser UI')
ap.add_argument('--stream-size', default='1600x900')
ap.add_argument('--stream-fps', type=float, default=30.0)
ap.add_argument('--stream-port', type=int, default=8212)
ap.add_argument('--bind', default='127.0.0.1', help='127.0.0.1 = this computer only')
ap.add_argument('--predict-max', type=float, default=0.08,
                help='extrapolate joints by measured velocity x data age (s, capped) to the render time; 0 = off')
ap.add_argument('--ui', action='store_true', help='show the Isaac Sim editor UI')
ap.add_argument('--renderer', default='RealTimePathTracing', help='RealTimePathTracing | RaytracedLighting | PathTracing')
ap.add_argument('--eye', type=float, nargs=3, default=[2.35, -1.45, 1.45])
ap.add_argument('--target', type=float, nargs=3, default=[0.40, 0.0, 0.30])
ap.add_argument('--focal', type=float, default=28.0)
ap.add_argument('--check', action='store_true', help='print link8 / flange world positions (compare with ROS TF)')
ap.add_argument('--shot', help='save a viewport screenshot (PNG) after --shot-after seconds')
ap.add_argument('--shot-after', type=float, default=40.0)
ap.add_argument('--exit', action='store_true', help='exit after the screenshot')
a = ap.parse_args()

app = SimulationApp({'headless': a.headless, 'hide_ui': not a.ui, 'width': a.width, 'height': a.height,
                     'window_width': a.width, 'window_height': a.height, 'renderer': a.renderer,
                     'limit_cpu_threads': 8, 'anti_aliasing': 3,
                     'extra_args': [f'--/app/window/x={a.x}', f'--/app/window/y={a.y}', f'--/app/window/title={a.title}',
                                    f'--/app/window/fullscreen={str(not a.windowed).lower()}']})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)


# ------------------------------------------------------------------------------------------------ helpers
def rpy_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2),
                              math.sin(y / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])                     # w x y z


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def axis_quat(axis, q):
    s = math.sin(q / 2)
    return np.array([math.cos(q / 2), axis[0] * s, axis[1] * s, axis[2] * s])


def material(path, roughness=0.65, metallic=0.0, vertex_color=True, color=(0.7, 0.7, 0.7)):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/Surface')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    if vertex_color:
        rd = UsdShade.Shader.Define(stage, path + '/ColorReader')
        rd.CreateIdAttr('UsdPrimvarReader_float3')
        rd.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('displayColor')
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(
            rd.ConnectableAPI(), 'result')
    else:
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def vertex_normals(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def set_mesh(path, v, f, c, mat, normals=True):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(v, np.float32)))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(f), 3, np.int32)))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(np.ascontiguousarray(f.ravel(), np.int32)))
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray,
                                                  UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(c, np.float32)))
    if normals and len(f):
        mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(vertex_normals(v.astype(np.float64), f).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


# ------------------------------------------------------------------------------------------------ robots
usd = os.path.join(CACHE, 'usd', 'fr3_duo_twin', 'fr3_duo_twin.usda')
if not os.path.exists(usd):
    raise SystemExit(f'{usd} missing - run isaac/convert_twin.py first')
twin = stage.DefinePrim('/World/Twin', 'Xform')
twin.GetReferences().AddReference(usd)
app.update()
links = {p.GetName(): p for p in stage.Traverse() if str(p.GetPath()).startswith('/World/Twin/Geometry')
         and p.GetTypeName() == 'Xform'}
joints = {}             # joint name -> (orient attr, origin quat, axis)
for j in ET.parse(os.path.join(CACHE, 'fr3_duo_twin.urdf')).getroot().findall('joint'):
    if j.get('type') not in ('revolute', 'continuous'):
        continue
    child = j.find('child').get('link')
    o = j.find('origin')
    rpy = [float(x) for x in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    ax = np.array([float(x) for x in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()])
    prim = links.get(child)
    if prim is None:
        carb.log_warn(f'twin_isaac: link {child} not in USD')
        continue
    joints[j.get('name')] = (prim.GetAttribute('xformOp:orient'), rpy_quat(*rpy), ax / np.linalg.norm(ax))
print(f'twin_isaac: {len(joints)} moving joints mapped', flush=True)


def apply_joints(q):
    with Sdf.ChangeBlock():
        for name, val in q.items():
            jt = joints.get(name)
            if jt is None:
                continue
            attr, q0, ax = jt
            w, x, y, z = qmul(q0, axis_quat(ax, val))
            if attr.GetTypeName() == Sdf.ValueTypeNames.Quatd:
                attr.Set(Gf.Quatd(w, x, y, z))
            else:
                attr.Set(Gf.Quatf(w, x, y, z))


# ------------------------------------------------------------------------------------------------ studio look
dome = UsdLux.DomeLight.Define(stage, '/World/Light/Dome')
dome.CreateIntensityAttr(140.0)                      # dark room like the real lab (robots lit by key + fill)
dome.CreateColorAttr(Gf.Vec3f(0.55, 0.60, 0.68))
key = UsdLux.DistantLight.Define(stage, '/World/Light/Key')
key.CreateIntensityAttr(1500.0)
key.CreateAngleAttr(2.5)                               # soft shadow edges
key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -30.0))
fill = UsdLux.RectLight.Define(stage, '/World/Light/Fill')
fill.CreateIntensityAttr(5000.0)
fill.CreateWidthAttr(1.6)
fill.CreateHeightAttr(1.0)
fx = UsdGeom.Xformable(fill.GetPrim())
fx.AddTranslateOp().Set(Gf.Vec3d(1.6, 1.4, 1.6))
fx.AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, 135.0))
floor_mat = material('/World/Looks/Floor', roughness=0.9, vertex_color=False, color=(0.03, 0.032, 0.035))
table_mat = material('/World/Looks/Table', roughness=0.6, metallic=0.45, vertex_color=False, color=(0.16, 0.165, 0.175))
frame_mat = material('/World/Looks/TableFrame', roughness=0.5, metallic=0.6, vertex_color=False, color=(0.35, 0.36, 0.38))


def box(path, lo, hi, mat):
    """Axis-aligned box mesh (table top / legs)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    b = UsdGeom.Mesh.Define(stage, path)
    b.GetPointsAttr().Set([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])
    b.GetFaceVertexCountsAttr().Set([4] * 6)
    b.GetFaceVertexIndicesAttr().Set([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
    b.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(mat)


# the real table (aluminium profile top, mount base plane = z 0): just below the scan so it never hides it
TX, TY, TZ = (-0.30, 1.40), (-0.95, 0.95), -0.006
box('/World/Table/Top', (TX[0], TY[0], TZ - 0.03), (TX[1], TY[1], TZ - 0.0005), table_mat)


def slat_texture(path, px_per_m=2000, pitch=0.035, slot=0.008, tile=0.35):
    """Aluminium profile table top like the real one (slats along x, ~35 mm pitch, dark slots, brushed metal):
    holes in the scan then show a plausible table instead of plain grey."""
    import cv2
    n = int(tile * px_per_m)
    rng = np.random.default_rng(0)
    img = np.full((n, n), 148.0)
    img += cv2.GaussianBlur(rng.normal(0, 14, (n, n)), (1, 61), 0)              # brushed along the slats
    y = (np.arange(n) / px_per_m) % pitch
    img[(y < slot)] = 38.0                                                        # slot (dark, shadowed)
    img[(y >= slot) & (y < slot + 0.0015)] = 95.0                                 # chamfer
    img[(y >= pitch - 0.0015)] = 185.0                                            # lit edge
    rgb = np.stack([img * 0.98, img * 0.99, img * 1.02], -1)
    cv2.imwrite(path, np.clip(rgb, 0, 255).astype(np.uint8))
    return tile


tex_path = os.path.join(CACHE, 'table_slats.png')
TILE = slat_texture(tex_path)
slat_mat = UsdShade.Material.Define(stage, '/World/Looks/TableSlats')
_sh = UsdShade.Shader.Define(stage, '/World/Looks/TableSlats/Surface')
_sh.CreateIdAttr('UsdPreviewSurface')
_sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.45)
_sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.55)
_rd = UsdShade.Shader.Define(stage, '/World/Looks/TableSlats/ST')
_rd.CreateIdAttr('UsdPrimvarReader_float2')
_rd.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
_tx = UsdShade.Shader.Define(stage, '/World/Looks/TableSlats/Tex')
_tx.CreateIdAttr('UsdUVTexture')
_tx.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_path))
_tx.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
_tx.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
_tx.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
_tx.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(_rd.ConnectableAPI(), 'result')
_sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(_tx.ConnectableAPI(), 'rgb')
slat_mat.CreateSurfaceOutput().ConnectToSource(_sh.ConnectableAPI(), 'surface')
_top = UsdGeom.Mesh.Define(stage, '/World/Table/Slats')
_top.GetPointsAttr().Set([(TX[0], TY[0], TZ), (TX[1], TY[0], TZ), (TX[1], TY[1], TZ), (TX[0], TY[1], TZ)])
_top.GetFaceVertexCountsAttr().Set([4])
_top.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
_top.CreateSubdivisionSchemeAttr('none')
# st: slats run along x -> the texture's stripes (along its u axis) follow x; v follows y
UsdGeom.PrimvarsAPI(_top).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
    [(TX[0] / TILE, TY[0] / TILE), (TX[1] / TILE, TY[0] / TILE), (TX[1] / TILE, TY[1] / TILE), (TX[0] / TILE, TY[1] / TILE)])
UsdShade.MaterialBindingAPI.Apply(_top.GetPrim()).Bind(slat_mat)
for i, (x, y) in enumerate(((TX[0] + .05, TY[0] + .05), (TX[1] - .05, TY[0] + .05), (TX[1] - .05, TY[1] - .05),
                            (TX[0] + .05, TY[1] - .05))):
    box(f'/World/Table/Leg{i}', (x - .03, y - .03, -0.76),
        (x + .03, y + .03, TZ - 0.03), frame_mat)
fl = UsdGeom.Mesh.Define(stage, '/World/Floor')
s = 6.0
fl.GetPointsAttr().Set([(-s, -s, -0.76), (s, -s, -0.76), (s, s, -0.76), (-s, s, -0.76)])
fl.GetFaceVertexCountsAttr().Set([4])
fl.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
UsdShade.MaterialBindingAPI.Apply(fl.GetPrim()).Bind(floor_mat)
scene_mat = material('/World/Looks/Scan', roughness=0.7)
invisible_mat = UsdShade.Material.Define(stage, '/World/Looks/Invisible')
_ish = UsdShade.Shader.Define(stage, '/World/Looks/Invisible/Surface')
_ish.CreateIdAttr('UsdPreviewSurface')
_ish.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(0.0)
_ish.CreateInput('opacityThreshold', Sdf.ValueTypeNames.Float).Set(0.5)
invisible_mat.CreateSurfaceOutput().ConnectToSource(_ish.ConnectableAPI(), 'surface')
live_mat = material('/World/Looks/Live', roughness=0.5)

st = carb.settings.get_settings()
for k, v in (('/rtx/post/tonemap/op', 4), ('/rtx/post/tonemap/filmIso', 120.0), ('/rtx/post/tonemap/whitepoint', 6500.0),
             ('/rtx/post/tonemap/enabled', True), ('/rtx/directLighting/sampledLighting/enabled', True),
             ('/rtx/ambientOcclusion/enabled', True),
             ('/persistent/app/viewport/Viewport/Viewport0/guide/grid/visible', False),
             ('/persistent/app/viewport/Viewport/Viewport0/guide/axis/visible', False),
             ('/persistent/app/viewport/guide/selection/visible', False),
             ('/app/viewport/grid/enabled', False), ('/app/viewport/outline/enabled', False)):
    st.set(k, v)

# hero camera
cam = UsdGeom.Camera.Define(stage, '/World/HeroCamera')
cam.CreateFocalLengthAttr(a.focal)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100.0))
m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*a.eye), Gf.Vec3d(*a.target), Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(m)
try:
    from omni.kit.viewport.utility import get_active_viewport
    vp = get_active_viewport()
    vp.camera_path = '/World/HeroCamera'
except Exception as e:      # noqa: BLE001 (headless without a viewport)
    vp = None
    carb.log_warn(f'twin_isaac: no viewport ({e})')


# ------------------------------------------------------------------------------------------------ scene files
def srgb_to_linear(c):
    """Camera colours are sRGB; the RTX material expects linear values (otherwise everything looks washed out)."""
    c = np.clip(c, 0, 1)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).astype(np.float32)


def photo_material(path):
    """Material showing the ZED photo through the mesh's 'st' UVs (projective texturing)."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/Surface')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.7)
    st_rd = UsdShade.Shader.Define(stage, path + '/STReader')
    st_rd.CreateIdAttr('UsdPrimvarReader_float2')
    st_rd.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    tex = UsdShade.Shader.Define(stage, path + '/Photo')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('clamp')
    tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('clamp')
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st_rd.ConnectableAPI(), 'result')
    # the photo already contains the REAL lighting and shadows: show it as it is (emissive), do not light it again
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    sh.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat, tex.CreateInput('file', Sdf.ValueTypeNames.Asset)


photo_mat, photo_file = photo_material('/World/Looks/ZedPhoto')


def photo_projection(v):
    """Vertices -> (uv (n,2), seen (n,) bool): ZED photo coordinates and whether the ZED sees that vertex in its depth
    snapshot (within 3 cm), so hidden surfaces never get the occluder's pixels. None if no photo yet."""
    try:
        ph = np.load(os.path.join(CACHE, 'zed_photo.npz'))
    except (OSError, ValueError):
        return None
    img = os.path.join(CACHE, str(ph['image']))
    if not os.path.exists(img):
        return None
    T, (fx, fy, cx, cy), (W, H) = ph['T'], ph['K'], ph['info_wh']
    pc = (v - T[:3, 3]) @ T[:3, :3]
    z = pc[:, 2]
    zs = np.where(z > 0.05, z, 1.0)
    u, w = fx * pc[:, 0] / zs + cx, fy * pc[:, 1] / zs + cy
    dep, (dw, dh) = ph['depth'], ph['depth_wh']
    du = np.clip((u * dw / W / 4).astype(int), 0, dep.shape[1] - 1)
    dv = np.clip((w * dh / H / 4).astype(int), 0, dep.shape[0] - 1)
    d = dep[dv, du]
    seen = (z > 0.05) & (u >= 0) & (u < W) & (w >= 0) & (w < H) & np.isfinite(d) & (np.abs(d - z) < 0.03)
    return np.stack([u / W, 1.0 - w / H], 1).astype(np.float32), seen, img


sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
import texturing  # noqa: E402
VIEW_DIR = os.path.join(CACHE, 'views')
_scan_views = {'mtime': None, 'views': []}
photo_mats = {}


def scan_views():
    """Views exported by the last 3D SCAN (isaac/cache/views/views.json): photos + poses + arms + depth; the robots'
    occlusion depth image is computed once per view (ZED views: same camera, arms elsewhere)."""
    path = os.path.join(VIEW_DIR, 'views.json')
    try:
        mt = os.path.getmtime(path)
    except OSError:
        return []
    if mt == _scan_views['mtime']:
        return _scan_views['views']
    from duo_geometry import DuoGeometry
    from twin_recon_core import RobotFilter
    rf = RobotFilter(DuoGeometry(), pad=0.0, gripper_pad=0.0)
    out = []
    for v in json.load(open(path)):
        vw = {'image': os.path.join(VIEW_DIR, v['image']), 'T': np.array(v['T']), 'K': v['K'], 'wh': tuple(v['wh']),
              'depth': None, 'robot': None}
        if v.get('depth'):
            vw['depth'] = (np.load(os.path.join(VIEW_DIR, v['depth'])).astype(np.float32), tuple(v['depth_wh']))
        if v.get('q'):
            caps = rf.capsules({a: np.asarray(q, float) for a, q in v['q'].items()}, filter_margin=False)
            vw['robot'] = texturing.robot_depth(vw['T'], vw['K'], vw['wh'], caps)
        out.append(vw)
    _scan_views.update(mtime=mt, views=out)
    print(f'twin_isaac: {len(out)} scan views loaded for texturing', flush=True)
    return out


def live_view():
    try:
        ph = np.load(os.path.join(CACHE, 'zed_photo.npz'))
    except (OSError, ValueError):
        return None
    img = os.path.join(CACHE, str(ph['image']))
    if not os.path.exists(img):
        return None
    W, H = (int(x) for x in ph['info_wh'])
    return {'image': img, 'T': ph['T'], 'K': list(ph['K']), 'wh': (W, H),
            'depth': (ph['depth'].astype(np.float32), tuple(int(x) for x in ph['depth_wh'])), 'robot': None}


def load_static():
    """Reconstruction cropped to the table volume, textured face by face with the BEST photo of all views (live ZED
    photo + every photo of the last 3D SCAN: the head camera with the arms elsewhere, the wrist cameras close up);
    faces no photo shows keep the fused vertex colours."""
    z = np.load(os.path.join(CACHE, 'mesh_static.npz'))
    v, f, c = z['points'], z['faces'], z['colors']
    inside = ((v[:, 0] > TX[0]) & (v[:, 0] < TX[1]) & (v[:, 1] > TY[0]) & (v[:, 1] < TY[1]) & (v[:, 2] > -0.03)
              & (v[:, 2] < 0.6))
    f = f[inside[f].all(1)]
    path = '/World/Scene/Reconstruction'
    stage.RemovePrim(path)
    mesh = set_mesh(path, v, f, srgb_to_linear(c), scene_mat)
    views = [w for w in [live_view()] + scan_views() if w is not None]
    if not views or not len(f):
        return len(f), 0
    best, uv = texturing.choose(v.astype(np.float64), f, views)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray.FromNumpy(uv.reshape(-1, 2)))
    for i, vw in enumerate(views):
        idx = np.nonzero(best == i)[0]
        if not len(idx):
            continue
        if i not in photo_mats:
            photo_mats[i] = photo_material(f'/World/Looks/Photo_{i:03d}')
        mat, fattr = photo_mats[i]
        fattr.Set(Sdf.AssetPath(vw['image']))
        sub = UsdGeom.Subset.CreateGeomSubset(mesh, f'photo_{i:03d}', UsdGeom.Tokens.face,
                                              Vt.IntArray.FromNumpy(idx.astype(np.int32)), UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
    zc = v[f].mean(1)[:, 2]
    best[(best < 0) & (np.abs(zc) < 0.012)] = -2          # un-photographed table-level faces: hidden (slat table shows)
    hide = np.nonzero(best == -2)[0]
    if len(hide):
        UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr()   # keep the mesh visible; hide via an invisible subset
        sub = UsdGeom.Subset.CreateGeomSubset(mesh, 'hidden', UsdGeom.Tokens.face,
                                              Vt.IntArray.FromNumpy(hide.astype(np.int32)), UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(invisible_mat)
    rest = np.nonzero(best == -1)[0]
    if len(rest):
        sub = UsdGeom.Subset.CreateGeomSubset(mesh, 'fused', UsdGeom.Tokens.face,
                                              Vt.IntArray.FromNumpy(rest.astype(np.int32)), UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(scene_mat)
    return len(f), int((best >= 0).sum())


CUBE_V = np.array([[x, y, zz] for x in (-.5, .5) for y in (-.5, .5) for zz in (-.5, .5)], np.float32)
CUBE_F = np.array([[0, 2, 3], [0, 3, 1], [4, 5, 7], [4, 7, 6], [0, 1, 5], [0, 5, 4], [2, 6, 7], [2, 7, 3],
                   [0, 4, 6], [0, 6, 2], [1, 3, 7], [1, 7, 5]], np.int32)


def load_live():
    z = np.load(os.path.join(CACHE, 'mesh_live.npz'))
    c, col, size = z['centers'], z['colors'], float(z['size'])
    n = len(c)
    v = (c[:, None, :] + CUBE_V[None] * size * 0.98).reshape(-1, 3)
    f = (CUBE_F[None] + 8 * np.arange(n)[:, None, None]).reshape(-1, 3)
    set_mesh('/World/Scene/Live', v, f, srgb_to_linear(np.repeat(col, 8, 0)), live_mat, normals=False)
    return n


mtimes = {}


def changed(name):
    p = os.path.join(CACHE, name)
    try:
        t = os.path.getmtime(p)
    except OSError:
        return False
    if mtimes.get(name) == t:
        return False
    mtimes[name] = t
    return True


omni.usd.get_context().get_selection().clear_selected_prim_paths()     # no transform gizmo in the picture
# ------------------------------------------------------------------------------------------------ browser stream
VIEWS = {'hero': ([2.35, -1.45, 1.45], [0.40, 0.0, 0.30]), 'front': ([1.95, 0.0, 0.85], [0.45, 0.0, 0.20]),
         'top': ([0.62, 0.0, 2.10], [0.60, 0.0, 0.05]), 'left': ([0.70, -1.75, 0.80], [0.50, 0.0, 0.20]),
         'right': ([0.70, 1.75, 0.80], [0.50, 0.0, 0.20])}
FOCAL = {'headcam': 16.3}       # ZED Mini left camera: fx 1498 px at 1920 -> 65 deg horizontal (USD aperture 20.955 mm)


def headcam_view():
    """The CALIBRATED head camera pose (config/digital_twin.yaml cameras.zed + the ZED's own link -> left optical
    offset): the photoreal render from exactly where the real ZED looks - directly comparable with its photo."""
    import yaml
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
    from duo_geometry import rpy_matrix
    z = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'digital_twin.yaml')))['cameras']['zed']
    R = rpy_matrix(*[float(v) for v in z['rpy']])
    eye = np.array([float(v) for v in z['xyz']]) + R @ np.array([0.0, 0.032, 0.013])   # left lens (measured TF)
    return eye.tolist(), (eye + R @ np.array([1.0, 0.0, 0.0])).tolist()


try:
    VIEWS['headcam'] = headcam_view()
except Exception as e:      # noqa: BLE001
    carb.log_warn(f'twin_isaac: headcam view: {e}')


class Stream:
    """Latest rendered frame as JPEG + camera requests from the browser (applied in the render loop)."""

    def __init__(self):
        self.jpeg, self.n, self.lock, self.cond = None, 0, threading.Lock(), threading.Condition()
        self.view_req = None
        self.info = {'fps': 0.0, 'joint_age_ms': None, 'render_ms': None}
        self.orbit = {'target': np.array(a.target, float)}
        self.set_eye(np.array(a.eye, float))

    def set_eye(self, eye):
        d = eye - self.orbit['target']
        self.orbit['dist'] = float(np.linalg.norm(d))
        self.orbit['az'] = math.degrees(math.atan2(d[1], d[0]))
        self.orbit['el'] = math.degrees(math.asin(d[2] / max(self.orbit['dist'], 1e-6)))

    def eye(self):
        o = self.orbit
        az, el = math.radians(o['az']), math.radians(o['el'])
        return o['target'] + o['dist'] * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])

    def apply(self, req):
        if 'preset' in req and req['preset'] in VIEWS:
            e, t = VIEWS[req['preset']]
            cam.GetFocalLengthAttr().Set(FOCAL.get(req['preset'], a.focal))
            self.orbit['target'] = np.array(t, float)
            self.set_eye(np.array(e, float))
        o = self.orbit
        o['az'] += float(req.get('daz', 0.0))
        o['el'] = float(np.clip(o['el'] + float(req.get('del', 0.0)), -5.0, 89.0))
        o['dist'] = float(np.clip(o['dist'] * float(req.get('zoom', 1.0)), 0.25, 6.0))
        if 'pan' in req:                                   # [right, up] in metres, in the view plane
            fwd = o['target'] - self.eye()
            fwd /= np.linalg.norm(fwd)
            right = np.cross(fwd, [0, 0, 1.0])
            right /= max(np.linalg.norm(right), 1e-6)
            up = np.cross(right, fwd)
            o['target'] = o['target'] + right * float(req['pan'][0]) + up * float(req['pan'][1])
        set_camera(self.eye(), o['target'])


def set_camera(eye, target):
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam.GetPrim()).GetOrderedXformOps()[0].Set(m)


stream = Stream()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors()
        self.end_headers()

    def do_GET(self):
        if self.path.startswith('/stream.mjpg'):
            self.send_response(200)
            self.cors()
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last = -1
            try:
                while True:
                    with stream.cond:
                        stream.cond.wait_for(lambda: stream.n != last, timeout=2.0)
                        jpg, last = stream.jpeg, stream.n
                    if jpg is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path.startswith('/frame.jpg') and stream.jpeg is not None:
            self.send_response(200)
            self.cors()
            self.send_header('Content-Type', 'image/jpeg')
            self.end_headers()
            self.wfile.write(stream.jpeg)
        elif self.path.startswith('/trace'):
            body = json.dumps(list(trace)).encode()
            self.send_response(200)
            self.cors()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith('/info'):
            view = {k: (v.tolist() if hasattr(v, 'tolist') else v) for k, v in stream.orbit.items()}
            body = json.dumps(dict(stream.info, view=view)).encode()
            self.send_response(200)
            self.cors()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path.startswith('/view'):
            try:
                stream.view_req = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            except ValueError:
                self.send_error(400)
                return
            self.send_response(204)
            self.cors()
            self.end_headers()
        else:
            self.send_error(404)


annot = None
if a.stream:
    import cv2
    import omni.replicator.core as rep
    rep.orchestrator.set_capture_on_play(False)
    sw, sh = (int(x) for x in a.stream_size.split('x'))
    rp = rep.create.render_product('/World/HeroCamera', (sw, sh))
    annot = rep.AnnotatorRegistry.get_annotator('rgb')
    annot.attach([rp])
    srv = ThreadingHTTPServer((a.bind, a.stream_port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'twin_isaac: browser stream http://{a.bind}:{a.stream_port}/stream.mjpg', flush=True)
    enc_q = []
    enc_cv = threading.Condition()

    def encoder():
        while True:
            with enc_cv:
                enc_cv.wait_for(lambda: enc_q)
                img = enc_q.pop()
                enc_q.clear()
            ok, buf = cv2.imencode('.jpg', img[:, :, 2::-1], [cv2.IMWRITE_JPEG_QUALITY, 88])
            if ok:
                with stream.cond:
                    stream.jpeg, stream.n = buf.tobytes(), stream.n + 1
                    stream.cond.notify_all()
    threading.Thread(target=encoder, daemon=True).start()

# ------------------------------------------------------------------------------------------------ main loop
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 47811))
sock.setblocking(False)
t0, last_files, n_js, last_log, shot_done = time.monotonic(), 0.0, 0, time.monotonic(), False
js_stamp, last_js = None, None
from collections import deque  # noqa: E402
trace = deque(maxlen=3000)      # (wall time, rendered arm joints) - accuracy check against the measured joint states
lat = []
while app.is_running():
    msg = None
    while True:                                  # newest packet only
        try:
            msg = sock.recv(65535)
        except BlockingIOError:
            break
    if msg:
        d = json.loads(msg)
        last_js = d
        n_js += 1
        lat.append(time.time() - d['t'])
        js_stamp = d.get('stamp')
    if last_js is not None:          # pose every frame: measured q + v * age (age = render time - joint stamp, capped)
        q, v = last_js['q'], last_js.get('v') or {}
        age = min(max(time.time() - (last_js.get('stamp') or last_js['t']), 0.0), a.predict_max)
        if a.predict_max > 0 and v:
            q = {k: val + v.get(k, 0.0) * age for k, val in q.items()}
        apply_joints(q)
        trace.append((time.time(), [q.get(f'{arm}_fr3_joint{i}', 0.0) for arm in ('left', 'right') for i in range(1, 8)]))
    if stream.view_req is not None:
        req, stream.view_req = stream.view_req, None
        stream.apply(req)
    now = time.monotonic()
    if now - last_files > 5.0:          # scene reloads (USD mesh rebuild) at most every 5 s: joints have priority
        last_files = now
        try:
            if changed('mesh_static.npz') | changed('zed_photo.npz') | changed('views/views.json'):
                n_f, n_ph = load_static()
                print(f'twin_isaac: reconstruction {n_f} triangles, {n_ph} photo-textured', flush=True)
            if changed('mesh_live.npz'):
                load_live()
        except (OSError, KeyError, ValueError) as e:
            carb.log_warn(f'twin_isaac: scene file: {e}')
    if now - last_log > 5.0:
        print(f'twin_isaac: joint updates {n_js / (now - last_log):.0f} Hz, bridge->render latency median '
              f'{1e3 * float(np.median(lat)) if lat else -1:.1f} ms', flush=True)
        last_log, n_js, lat = now, 0, []
    if a.check and now - last_log < 0.02:
        xc = UsdGeom.XformCache()
        for arm in ('left', 'right'):
            p8 = links.get(f'{arm}_fr3_link8')
            if p8 is not None:
                t = xc.GetLocalToWorldTransform(p8).ExtractTranslation()
                print(f'twin_isaac: CHECK {arm}_fr3_link8 world {t[0]:.4f} {t[1]:.4f} {t[2]:.4f}', flush=True)
    if vp is not None and now - t0 < 15.0 and vp.camera_path != '/World/HeroCamera':
        vp.camera_path = '/World/HeroCamera'      # the viewport resets its camera while the stage loads
    if a.shot and not shot_done and now - t0 > a.shot_after:
        shot_done = True
        from omni.kit.viewport.utility import capture_viewport_to_file
        capture_viewport_to_file(vp, a.shot)
        for _ in range(10):
            app.update()
        print(f'twin_isaac: screenshot {a.shot}', flush=True)
        if a.exit:
            break
    t_upd = time.monotonic()
    if annot is not None:      # plain app.update() never fills the annotator (measured): a non-blocking replicator step
        rep.orchestrator.step(delta_time=0.0, rt_subframes=1, wait_for_render=False)
    else:
        app.update()
    if annot is not None:
        data = annot.get_data()
        if data is not None and getattr(data, 'size', 0) and now - getattr(stream, 't_last', 0.0) >= 1.0 / a.stream_fps:
            stream.t_last = now
            img = np.asarray(data)[:, :, :3]
            with enc_cv:
                enc_q.append(img.copy())
                enc_cv.notify()
            stream.frames = getattr(stream, 'frames', 0) + 1
            if js_stamp:
                stream.ages = (getattr(stream, 'ages', []) + [time.time() - js_stamp])[-60:]
        stream.info['render_ms'] = round((time.monotonic() - t_upd) * 1e3, 1)
        if now - getattr(stream, 't_fps', 0.0) > 2.0:
            stream.info['fps'] = round(getattr(stream, 'frames', 0) / max(now - getattr(stream, 't_fps', now - 2.0), 1e-3), 1)
            stream.t_fps, stream.frames = now, 0
            if getattr(stream, 'ages', None):
                stream.info['joint_age_ms'] = round(1e3 * float(np.median(stream.ages)), 1)
app.close()
