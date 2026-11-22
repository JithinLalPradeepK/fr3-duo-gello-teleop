#!/usr/bin/env python3
"""OFFLINE test of the twin reconstruction core (scripts/twin_recon_core.py). Synthetic scene, no ROS graph, no camera,
no robot:  python3 test/twin_recon_test.py

Scene in the twin world frame: table top z = 0, a 12 cm box, a sphere, both FR3s as their duo_geometry capsules (rendered
at 90 % radius = the real arm sits inside its capsules). Depth images are ray cast with 2 mm noise from a head camera
(ZED-like) and from a wrist camera on the left flange (FK). R1-R12 below.
"""
import os
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import numpy as np  # noqa: E402

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import READY, DuoGeometry  # noqa: E402
from twin_recon_core import (TSDF, RobotFilter, face_normals, live_points, marker_bytes,  # noqa: E402
                             surface_nets, write_ply)

VOX = 0.01
BOX_C, BOX_H = np.array([0.65, 0.0, 0.06]), np.array([0.06, 0.06, 0.06])
SPH_C, SPH_R = np.array([0.75, 0.30, 0.06]), 0.06
TABLE = (0.15, 1.05, -0.75, 0.75)            # x0 x1 y0 y1, z = 0
COL = {'table': (150, 120, 90), 'box': (220, 40, 40), 'sphere': (40, 200, 60), 'robot': (240, 240, 240),
       'obj': (30, 60, 230)}
results = []


def check(name, ok, info=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}")


def look_at(eye, target, up=(0, 0, 1)):
    """Camera optical frame (z forward, x right, y down) -> world."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], 1)
    T[:3, 3] = eye
    return T


def ray_sphere(o, d, c, r):
    oc = o - c
    b = d @ oc
    disc = b * b - (oc @ oc - r * r)
    t = -b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc > 0) & (t > 0), t, np.inf)


def ray_box(o, d, c, h):
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (c - h - o) / d, (c + h - o) / d
    tn, tf = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
    return np.where((tn < tf) & (tn > 0), tn, np.inf)


def render(T, K, wh, caps, extra=(), noise=0.002, rng=None):
    """Ray cast depth (m) + rgb + robot-pixel truth for camera pose T (optical -> world)."""
    w, h = wh
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:h, 0:w]
    dc = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    dc /= np.linalg.norm(dc, axis=1, keepdims=True)
    d = dc @ T[:3, :3].T
    o = T[:3, 3]
    best = np.full(len(d), np.inf)
    lab = np.full(len(d), -1)
    with np.errstate(divide='ignore', invalid='ignore'):
        tp = -o[2] / d[:, 2]
    p = o + tp[:, None] * d
    tp = np.where((tp > 0) & (p[:, 0] > TABLE[0]) & (p[:, 0] < TABLE[1]) & (p[:, 1] > TABLE[2]) & (p[:, 1] < TABLE[3]),
                  tp, np.inf)
    cand = [('table', tp), ('box', ray_box(o, d, BOX_C, BOX_H)), ('sphere', ray_sphere(o, d, SPH_C, SPH_R))]
    for c, r in extra:
        cand.append(('obj', ray_sphere(o, d, np.asarray(c), r)))
    A, B, R = caps
    for a, b, r in zip(A, B, R):                       # capsule = spheres along the segment (inside the capsule)
        for s in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / (0.3 * r)) + 2)):
            cand.append(('robot', ray_sphere(o, d, a + s * (b - a), 0.9 * r)))
    names = list(COL)
    for name, t in cand:
        m = t < best
        best[m], lab[m] = t[m], names.index(name)
    z = best * dc[:, 2]
    if rng is not None:
        z = z + rng.normal(0, noise, len(z))
    z[~np.isfinite(best)] = 0
    rgb = np.zeros((len(d), 3), np.uint8)
    for i, n in enumerate(names):
        rgb[lab == i] = COL[n]
    return z.reshape(h, w).astype(np.float32), rgb.reshape(h, w, 3), (lab == names.index('robot')).reshape(h, w)


def scene_sdf(p, caps, extra=()):
    """Distance to the true static surfaces (table, box, sphere [, extra spheres]) - not the robots."""
    dt = np.abs(p[:, 2])
    out_xy = np.maximum.reduce([TABLE[0] - p[:, 0], p[:, 0] - TABLE[1], TABLE[2] - p[:, 1], p[:, 1] - TABLE[3],
                                np.zeros(len(p))])
    dt = np.hypot(dt, out_xy)
    q = np.abs(p - BOX_C) - BOX_H
    db = np.abs(np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0))
    ds = np.abs(np.linalg.norm(p - SPH_C, axis=1) - SPH_R)
    d = np.minimum.reduce([dt, db, ds])
    for c, r in extra:
        d = np.minimum(d, np.abs(np.linalg.norm(p - np.asarray(c), axis=1) - r))
    return d


def integrate(vol, rf, T, K, wh, caps, rng, extra=(), cache=True, far=2.5):
    depth, rgb, _ = render(T, K, wh, caps, extra, rng=rng)
    pw = __import__('twin_recon_core').backproject(depth, K, T)
    pw[depth.ravel() <= 0] = np.nan
    robot = rf.mask(pw, caps).reshape(depth.shape)
    vol.integrate(depth, K, T, rgb, free_only=robot, far=far, cache=cache)
    return depth, rgb, pw, robot


def main():
    rng = np.random.default_rng(1)
    g = DuoGeometry()
    rf = RobotFilter(g, pad=0.02)
    ql = np.array(READY)
    qr = np.array(READY)
    caps = rf.capsules({'left': ql, 'right': qr}, filter_margin=False)
    lo, hi = (-0.3, -0.9, -0.1), (1.2, 0.9, 1.0)
    vol = TSDF(lo, hi, VOX, trunc_vox=4, max_weight=20)
    print(f'grid {vol.shape} = {np.prod(vol.shape) / 1e6:.1f} M voxels, {vol.nbytes / 1e6:.0f} MB')
    W, H = 320, 180
    K = (250.0, 250.0, W / 2 - 0.5, H / 2 - 0.5)
    T_head = look_at((1.35, 0.0, 0.80), (0.5, 0.0, 0.05))

    # R1 head camera static map
    t0 = time.perf_counter()
    for _ in range(12):
        integrate(vol, rf, T_head, K, (W, H), caps, rng)
    t_int = (time.perf_counter() - t0) / 12
    t0 = time.perf_counter()
    v, t, c = surface_nets(vol)
    t_mesh = time.perf_counter() - t0
    check('R1 mesh built from the head camera', len(t) > 5000, f'{len(v)} vertices {len(t)} triangles, '
          f'integrate {t_int * 1e3:.0f} ms/frame ({W}x{H}), mesh {t_mesh * 1e3:.0f} ms')
    # R2 accuracy
    err = scene_sdf(v, caps)
    check('R2 vertices on the true surfaces', np.median(err) < 0.25 * VOX and np.percentile(err, 95) < 0.8 * VOX,
          f'median {np.median(err) * 1e3:.1f} mm, p95 {np.percentile(err, 95) * 1e3:.1f} mm')
    # R3 no robot in the map
    A, B, R = caps
    from twin_recon_core import point_segment_dist
    dr = (point_segment_dist(v, A, B) - R[None]).min(1)
    check('R3 no robot geometry in the static mesh', (dr < -0.005).sum() == 0, f'{int((dr < -0.005).sum())} vertices '
          f'inside capsules, closest {dr.min() * 1e3:.0f} mm')
    # R4 normals face the camera (free space)
    n = face_normals(v, t)
    ctr = v[t].mean(1)
    facing = ((T_head[:3, 3] - ctr) * n).sum(1) > 0
    check('R4 triangles face free space', facing.mean() > 0.95, f'{facing.mean() * 100:.1f} %')
    # R5 colour
    on_box = (np.abs(v - BOX_C) <= BOX_H + 0.002).all(1) & (v[:, 2] > 0.02)
    cb = c[on_box].mean(0) if on_box.any() else np.zeros(3)
    on_tab = (np.abs(v[:, 2]) < 0.004) & (np.linalg.norm(v[:, :2] - BOX_C[:2], axis=1) > 0.2)
    ct = c[on_tab].mean(0)
    check('R5 colours: box red, table brown', cb[0] > 180 and cb[1] < 90 and abs(ct[0] - 150) < 15,
          f'box {cb.round()} table {ct.round()}')
    # R6 coverage of visible static surface
    depth, _, robot = render(T_head, K, (W, H), caps)
    from twin_recon_core import backproject
    pw = backproject(depth, K, T_head)[(depth.ravel() > 0) & ~robot.ravel()]
    pw = pw[np.abs(pw[:, 2]) > -1][::7]
    from scipy.spatial import cKDTree
    dd, _ = cKDTree(v).query(pw)
    check('R6 visible static surface covered', (dd < 1.5 * VOX).mean() > 0.95, f'{(dd < 1.5 * VOX).mean() * 100:.1f} %')

    # R7 moving object: live points, not in the static map at first
    obj = [((0.90, -0.30, 0.20), 0.05)]       # in free space the head camera has seen
    for _ in range(3):
        depth, rgb, pw, robot = integrate(vol, rf, T_head, K, (W, H), caps, rng, extra=obj)
    lp, lc = live_points(vol, pw[~robot.ravel()], rgb.reshape(-1, 3)[~robot.ravel()])
    d_obj = np.linalg.norm(lp - np.array(obj[0][0]), axis=1) - obj[0][1] if len(lp) else np.array([1.0])
    v2, t2, _ = surface_nets(vol)
    d2 = np.linalg.norm(v2 - np.array(obj[0][0]), axis=1)
    check('R7 moving object = live voxels, not static', len(lp) > 30 and np.percentile(np.abs(d_obj), 95) < 0.02 and
          (d2 < obj[0][1] + 0.01).sum() == 0, f'{len(lp)} live voxels (p95 {np.percentile(np.abs(d_obj), 95) * 1e3:.0f} mm off the object), '
          f'static vertices at the object '
          f'{int((d2 < obj[0][1] + 0.01).sum())}')
    # R8 object that stays becomes static; removed again -> carved away
    gap = []
    for _ in range(40):
        depth, rgb, pw, robot = integrate(vol, rf, T_head, K, (W, H), caps, rng, extra=obj)
        lp, _ = live_points(vol, pw[~robot.ravel()], rgb.reshape(-1, 3)[~robot.ravel()])
        n_live = int((np.linalg.norm(lp - np.array(obj[0][0]), axis=1) < obj[0][1] + 0.015).sum()) if len(lp) else 0
        vv, _, _ = surface_nets(vol)
        n_st = int((np.linalg.norm(vv - np.array(obj[0][0]), axis=1) < obj[0][1] + 0.01).sum())
        gap.append(n_live < 30 and n_st < 30)
    check('R8a object always shown (live or static) while the map absorbs it', not any(gap),
          f'frames shown in neither layer: {sum(gap)}/40')
    v3, _, _ = surface_nets(vol)
    n_in = int((np.linalg.norm(v3 - np.array(obj[0][0]), axis=1) < obj[0][1] + 0.01).sum())
    for _ in range(40):
        integrate(vol, rf, T_head, K, (W, H), caps, rng)
    v4, _, _ = surface_nets(vol)
    n_out = int((np.linalg.norm(v4 - np.array(obj[0][0]), axis=1) < obj[0][1] + 0.01).sum())
    check('R8 object left in place becomes static, removed -> carved', n_in > 50 and n_out == 0,
          f'after 40 frames {n_in} static vertices, after removal {n_out}')

    # R9 moving arm: no ghosts (left arm swings between two poses while the head camera watches)
    q_b = ql + np.array([0.6, 0.3, 0, 0.3, 0, 0, 0])
    for k in range(20):
        qq = ql if k % 2 else q_b
        ck = rf.capsules({'left': qq, 'right': qr}, filter_margin=False)
        integrate(vol, rf, T_head, K, (W, H), ck, rng)
    vol.clear_capsules(*rf.capsules({'left': ql, 'right': qr}))
    v5, _, _ = surface_nets(vol)
    worst = []
    for qq in (ql, q_b):
        A, B, R = rf.capsules({'left': qq, 'right': qr})
        worst.append(int(((point_segment_dist(v5, A, B) - R[None]).min(1) < -0.005).sum()))
    check('R9 swinging arm leaves no ghost', max(worst) == 0, f'vertices inside the arm at pose A/B: {worst}')

    # R10 wrist camera fusion (moving camera, pose from FK)
    T_flange = g.link_frames('left', ql)[8]
    X = np.eye(4)
    X[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])        # camera looks along the flange z (down)
    X[:3, 3] = (0.07, 0.0, 0.04)
    wv = TSDF(lo, hi, VOX, trunc_vox=4, max_weight=20)
    Kw = (200.0, 200.0, 159.5, 89.5)
    t0 = time.perf_counter()
    nfr = 0
    for k in range(16):
        qq = ql + np.array([0.25 * np.sin(k / 3), 0.15 * np.cos(k / 4), 0, 0.2 * np.sin(k / 5), 0, 0, 0])
        Tc = g.link_frames('left', qq)[8] @ X
        ck = rf.capsules({'left': qq, 'right': qr}, filter_margin=False)
        integrate(wv, rf, Tc, Kw, (W, H), ck, rng, cache=False, far=0.9)
        nfr += 1
    t_w = (time.perf_counter() - t0) / nfr
    vw, tw, _ = surface_nets(wv)
    ew = scene_sdf(vw, caps) if len(vw) else np.array([1.0])
    check('R10 wrist camera (FK pose) fuses consistently', len(tw) > 2000 and np.median(ew) < 0.3 * VOX,
          f'{len(tw)} triangles, median {np.median(ew) * 1e3:.1f} mm, {t_w * 1e3:.0f} ms/frame, flange z '
          f'{T_flange[2, 3]:.2f}')

    # R11 save / load / reset
    path = os.path.join(DEMO, 'logs', 'twin_recon_test_map.npz')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    vol.save(path)
    v_l = TSDF.load(path)
    m1, m2 = surface_nets(vol)[1], surface_nets(v_l)[1]
    write_ply(path.replace('.npz', '.ply'), *surface_nets(vol))
    vol.reset()
    check('R11 save/load round trip, reset empties', abs(len(m1) - len(m2)) <= 0.01 * len(m1) and
          len(surface_nets(vol)[1]) == 0, f'{len(m1)} vs {len(m2)} triangles, file {os.path.getsize(path) / 1e6:.1f} MB')

    # R12 Marker bytes == rclpy serialisation, and speed
    try:
        from geometry_msgs.msg import Point
        from rclpy.serialization import deserialize_message, serialize_message
        from std_msgs.msg import ColorRGBA
        from visualization_msgs.msg import Marker
        pts = np.random.rand(30, 3)
        cols = np.random.rand(30, 4).astype(np.float32)
        mk = Marker()
        mk.header.frame_id, mk.ns, mk.id, mk.type = 'twin/world', 'mesh', 7, Marker.TRIANGLE_LIST
        mk.scale.x = mk.scale.y = mk.scale.z = 1.0
        mk.text = 'tail check'
        b = marker_bytes(mk, pts, cols)
        ref = Marker()
        ref.header.frame_id, ref.ns, ref.id, ref.type = 'twin/world', 'mesh', 7, Marker.TRIANGLE_LIST
        ref.scale.x = ref.scale.y = ref.scale.z = 1.0
        ref.text = 'tail check'
        ref.points = [Point(x=float(a), y=float(bb), z=float(cc)) for a, bb, cc in pts]
        ref.colors = [ColorRGBA(r=float(r), g=float(gg), b=float(bb), a=float(aa)) for r, gg, bb, aa in cols]
        same = serialize_message(ref) == b
        back = deserialize_message(b, Marker)
        big = np.random.rand(300000, 3)
        t0 = time.perf_counter()
        marker_bytes(mk, big, np.random.rand(300000, 4).astype(np.float32))
        tb = time.perf_counter() - t0
        check('R12 Marker bytes identical to rclpy, fast', same and back.text == 'tail check' and len(back.points) == 30,
              f'100k triangles serialised in {tb * 1e3:.0f} ms')
    except ImportError as e:
        check('R12 Marker bytes (ROS not sourced)', False, str(e))

    print(f'\n{sum(results)}/{len(results)} passed')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
