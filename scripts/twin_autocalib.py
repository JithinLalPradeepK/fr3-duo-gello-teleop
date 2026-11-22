#!/usr/bin/env python3
"""AUTOMATIC ZED -> twin calibration from the camera footage (read-only for the robots: subscribes only; with --write it
edits config/digital_twin.yaml, which twin_state.py reloads live).

  python3 scripts/twin_autocalib.py [--frames 5] [--write]

The arm poses in the twin come from the robots' own joint encoders, so they are exact; what makes the twin disagree with
the camera picture is WHERE THE CAMERA SITS. This tool registers the ZED point cloud (the robots as the camera sees them)
against the surface of the twin's robot model (FR3 + Duo mount collision meshes, posed with the live twin TF) by ICP:
  DEFAULT --method plane (2026-11-22, verified on the real ZED by projecting the robot meshes into its image):
  1. the TABLE plane in the camera cloud gives roll, pitch and the camera height above the table (table at
     --table-z, default 0 = mount base plane),
  2. table points removed, x / y / yaw searched on a grid (score = points within 2 cm of the encoder-posed robot meshes,
     incl. the Robotiq grippers), 3. fine constrained search (0.5 deg, 5 mm). Free 6-DOF ICP drifted on the real scene
     (most points above the table are objects, people, background), which is why the old method gave a false fit.
  --method icp: the old multi-start ICP (accept if enough inliers and a tight RMS).
Needs: twin running (digital_twin.launch.py), cameras running, robots visible to the ZED, nothing big in front of them.
Best with the robots in a pose where much of both arms is visible (e.g. HOME).
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import rpy_matrix  # noqa: E402
from make_duo_geometry import read_stl  # noqa: E402

FD = os.path.expanduser('~/gello_deps_ws/install/franka_description/share/franka_description')


def voxel(p, size):
    """One point per voxel (first hit)."""
    if len(p) == 0:
        return p
    k = np.floor(p / size).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return p[np.sort(idx)]


def mat(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def matrix_rpy(r):
    """Inverse of duo_geometry.rpy_matrix (R = Rz(y) Ry(p) Rx(r))."""
    p = math.asin(-max(-1.0, min(1.0, r[2, 0])))
    if abs(math.cos(p)) > 1e-6:
        return (math.atan2(r[2, 1], r[2, 2]), p, math.atan2(r[1, 0], r[0, 0]))
    return (math.atan2(-r[1, 2], r[1, 1]), p, 0.0)


def kabsch(a, b):
    """Rigid T (4x4) minimising |T a - b|."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    u, _s, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1, 1, d]) @ u.T
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = cb - r @ ca
    return t


def icp(scene, tree, model, T, iters=30, max_d=(0.08, 0.04, 0.02, 0.012)):
    """Trimmed point-to-point ICP. scene: points in the camera(cloud) frame; T: world <- cloud. Returns (T, rms, inliers)."""
    rms, n = 1e9, 0
    for stage, md in enumerate(max_d):
        for _ in range(iters):
            w = scene @ T[:3, :3].T + T[:3, 3]
            d, i = tree.query(w, distance_upper_bound=md)
            ok = np.isfinite(d)
            n = int(ok.sum())
            if n < 30:
                return T, 1e9, n
            D = kabsch(w[ok], model[i[ok]])
            T = D @ T
            rms = float(np.sqrt(np.mean(d[ok] ** 2)))
            if np.linalg.norm(D[:3, 3]) < 1e-4 and abs(np.arccos(min(1.0, (np.trace(D[:3, :3]) - 1) / 2))) < 1e-4:
                break
    return T, rms, n


def register(scene, model, T0, coarse=True):
    """Best (T world<-cloud, rms, inliers) over a coarse multi-start around T0, then fine ICP."""
    tree = cKDTree(model)
    starts = [T0]
    if coarse:
        for dyaw in (-0.3, -0.15, 0.15, 0.3):            # camera turned about the vertical, same position
            s = T0.copy()
            s[:3, :3] = rpy_matrix(0, 0, dyaw) @ T0[:3, :3]
            starts.append(s)
        for dx in (-0.15, 0.15):
            for ax in range(3):
                s = T0.copy()
                s[ax, 3] += dx
                starts.append(s)
    best = None
    sub = scene[np.random.default_rng(0).permutation(len(scene))[:3000]]
    for s in starts:
        T, rms, n = icp(sub, tree, model, s, iters=15, max_d=(0.10, 0.05, 0.025))
        score = n / (1.0 + 50 * rms)
        if best is None or score > best[0]:
            best = (score, T)
    T, rms, n = icp(scene, tree, model, best[1], iters=40, max_d=(0.04, 0.02, 0.012))
    return T, rms, n


RQ = os.path.expanduser('~/gello_deps_ws/install/robotiq_description/share/robotiq_description/meshes/collision/2f_85')


def mesh_path(name):
    """Collision mesh of a twin link: mount_link, <arm>_fr3_link0..7, <arm>_robotiq_85_<part>_link."""
    if name == 'mount_link':
        return os.path.join(FD, 'meshes', 'robot_arms', 'fr3_duo', 'mount', 'collision', 'fr3_duo_mount.stl')
    if '_robotiq_85_' in name:
        part = name.split('_robotiq_85_')[1][:-len('_link')]
        return os.path.join(RQ, ('robotiq_base' if part == 'base' else part) + '.stl')
    return os.path.join(FD, 'meshes', 'robot_arms', 'fr3', 'collision', name.split('_fr3_')[1] + '.stl')


ROBOTIQ_PARTS = ('base', 'left_knuckle', 'right_knuckle', 'left_finger', 'right_finger', 'left_inner_knuckle',
                 'right_inner_knuckle', 'left_finger_tip', 'right_finger_tip')


def twin_links(with_gripper=True):
    out = ['mount_link'] + [f'{a}_fr3_link{i}' for a in ('left', 'right') for i in range(8)]
    if with_gripper:
        out += [f'{a}_robotiq_85_{p}_link' for a in ('left', 'right') for p in ROBOTIQ_PARTS]
    return out


def model_points(link_T, spacing=0.008):
    """Robot surface points in the twin world: FR3 link0-7 (both arms) + Duo mount, collision meshes (vertices + face
    centres, voxel-thinned). link_T: {'left_fr3_link3': 4x4 world<-link, ..., 'mount_link': ...}."""
    pts = []
    for name, T in link_T.items():
        v = read_stl(mesh_path(name))
        tri = v.reshape(-1, 3, 3)
        p = np.concatenate([v, tri.mean(1), (tri[:, 0] + tri[:, 1]) / 2, (tri[:, 1] + tri[:, 2]) / 2])
        pts.append(voxel(p, spacing) @ T[:3, :3].T + T[:3, 3])
    return voxel(np.concatenate(pts), spacing)


def head_starts(T_link_cloud):
    """Starting guesses for a ZED on the robot's HEAD (above the mount, looking forward (+x) and down): T world<-cloud."""
    out = []
    for x in (-0.05, 0.08, 0.2):
        for z in (0.45, 0.6, 0.75):
            for pitch in (0.35, 0.6, 0.85, 1.1):
                out.append(mat((x, 0.0, z), (0.0, pitch, 0.0)) @ T_link_cloud)
    return out


def search(scene, model, starts):
    """Best start by a short ICP on a subsample, then the full fine ICP. Returns (T, rms, n)."""
    tree = cKDTree(model)
    sub = scene[np.random.default_rng(0).permutation(len(scene))[:3000]]
    best = None
    for s0 in starts:
        T, rms, n = icp(sub, tree, model, s0, iters=15, max_d=(0.15, 0.08, 0.04, 0.02))
        score = n / (1.0 + 50 * rms)
        if best is None or score > best[0]:
            best = (score, T)
    return icp(scene, tree, model, best[1], iters=40, max_d=(0.04, 0.02, 0.012))


def fit_gripper_offset(scene_w, link_T, z_range=np.arange(0.0, 0.121, 0.005), md=0.012):
    """Gripper mount: shift the Robotiq model of BOTH arms along the flange z axis by dz (coupling height) and score
    the match with the (calibrated) scene around the grippers. Returns (best dz, inliers at best, table of (dz, n))."""
    grip = [k for k in link_T if '_robotiq_85_' in k]
    tree = cKDTree(scene_w)
    table = []
    for dz in z_range:
        pts = []
        for k in grip:
            arm = k.split('_')[0]
            ax = link_T[f'{arm}_fr3_link7'][:3, :3] @ np.array([0.0, 0.0, 1.0])      # flange z (link8 = link7 rot)
            v = voxel(read_stl(mesh_path(k)), 0.005)
            pts.append(v @ link_T[k][:3, :3].T + link_T[k][:3, 3] + ax * dz)
        d, _ = tree.query(np.concatenate(pts), distance_upper_bound=md)
        table.append((float(dz), int(np.isfinite(d).sum())))
    best = max(table, key=lambda t: t[1])
    return best[0], best[1], table


def table_plane(p, tol=0.012, iters=1500, seed=0):
    """Dominant plane of the cloud (camera link frame): (unit normal pointing up, d, inlier mask), n.p + d = 0."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        k = int((np.abs(p @ n - n @ a) < tol).sum())
        if best is None or k > best[0]:
            best = (k, n, -n @ a)
    _, n, d = best
    inl = np.abs(p @ n + d) < tol
    c0 = p[inl].mean(0)
    n = np.linalg.svd(p[inl] - c0)[2][2]
    n = n if n[2] > 0 else -n
    return n, -n @ c0, np.abs(p @ n - n @ c0) < tol


def plane_search(scene, model, T_lc, table_z=None, md=0.012):
    """--method plane: returns (T world<-cloud, inlier RMS, matched robot points, info dict). table_z None = search the
    table height too (weak - flat score; only if the mount does not stand on the table)."""
    if table_z is None:
        res = [plane_search(scene, model, T_lc, tz, md) for tz in np.arange(-0.10, 0.51, 0.10)]
        tz0 = max(res, key=lambda r: r[2])[3]['table_z']
        res = [plane_search(scene, model, T_lc, tz, md) for tz in np.arange(tz0 - 0.08, tz0 + 0.081, 0.02)]
        tz1 = max(res, key=lambda r: r[2])[3]['table_z']
        res = [plane_search(scene, model, T_lc, tz, md) for tz in np.arange(tz1 - 0.015, tz1 + 0.016, 0.005)]
        best = max(res, key=lambda r: r[2])
        best[3]['table_z_scan'] = [(round(r[3]['table_z'], 3), r[2]) for r in res]
        return best
    p = scene @ T_lc[:3, :3].T + T_lc[:3, 3]                 # camera link frame (x forward, z up)
    n, d, inl = table_plane(p)
    h = d                                                     # camera height above the table
    v = np.cross(n, [0, 0, 1.0])
    sv = np.linalg.norm(v)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R0 = np.eye(3) + K + K @ K * ((1 - n[2]) / max(sv * sv, 1e-12))          # table normal -> +z, no twist
    robot = p[(p @ n + d) > 0.025]
    tree = cKDTree(model)
    rng = np.random.default_rng(0)
    sub = robot[rng.permutation(len(robot))[:1500]] @ R0.T

    def rz(y):
        c, s_ = math.cos(y), math.sin(y)
        return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
    best = None
    for yaw in np.radians(np.arange(-180, 180, 4)):
        q = sub @ rz(yaw).T
        q[:, 2] += h + table_z
        for x in np.arange(-0.4, 0.41, 0.03):
            for y in np.arange(-0.4, 0.41, 0.03):
                k = int(np.isfinite(tree.query(q + [x, y, 0], distance_upper_bound=0.02)[0]).sum())
                if best is None or k > best[0]:
                    best = (k, yaw, x, y)
    _, yaw0, x0, y0 = best
    best = None
    for yaw in np.radians(np.arange(math.degrees(yaw0) - 8, math.degrees(yaw0) + 8.01, 0.5)):
        q = robot @ (rz(yaw) @ R0).T
        q[:, 2] += h + table_z
        for x in np.arange(x0 - 0.05, x0 + 0.051, 0.005):
            for y in np.arange(y0 - 0.06, y0 + 0.061, 0.005):
                dd = tree.query(q + [x, y, 0], distance_upper_bound=md)[0]
                k = np.isfinite(dd)
                sc = k.sum() - 0.5 * dd[k].sum() / md
                if best is None or sc > best[0]:
                    best = (sc, yaw, x, y, int(k.sum()), float(np.sqrt(np.mean(dd[k] ** 2))) if k.any() else 1.0)
    _, yaw, x, y, nm, rms = best
    T_link = np.eye(4)
    T_link[:3, :3] = rz(yaw) @ R0
    T_link[:3, 3] = (x, y, h + table_z)
    return T_link @ T_lc, rms, nm, {'height_m': float(h), 'pitch_deg': float(math.degrees(math.acos(n[2]))),
                                    'table_z': float(table_z),
                                    'table_points': int(inl.sum()), 'robot_side_points': int(len(robot))}


# ---------------------------------------------------------------------------------------------- ROS part
def capture(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import Buffer, TransformListener
    from twin_cloud import cloud_view
    from twin_state import quat_matrix
    cam = yaml.safe_load(open(a.config))['cameras']['zed']
    rclpy.init()
    node = Node('twin_autocalib')
    buf = Buffer()
    TransformListener(buf, node)
    clouds = []
    node.create_subscription(PointCloud2, a.cloud, lambda m: clouds.append(m) if len(clouds) < a.frames else None,
                             qos_profile_sensor_data)
    end = time.monotonic() + 20
    while (len(clouds) < a.frames or time.monotonic() < end - 17) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not clouds:
        sys.exit('no ZED point cloud - are the cameras running?')

    def tf(target, source):
        t = buf.lookup_transform(target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        q, p = t.transform.rotation, t.transform.translation
        m = np.eye(4)
        m[:3, :3] = quat_matrix(q.x, q.y, q.z, q.w)
        m[:3, 3] = (p.x, p.y, p.z)
        return m
    try:
        T_link_cloud = tf(cam['child'], clouds[0].header.frame_id)     # ZED's own static TF
        links = {k: tf('twin/world', 'twin/' + k) for k in twin_links()}
    except Exception as e:      # noqa: BLE001
        sys.exit(f'TF not available ({e}) - is the digital twin running?')
    scene = []
    for m in clouds:
        v = cloud_view(m).ravel()
        p = np.stack([v['x'], v['y'], v['z']], 1).astype(float)
        d = np.linalg.norm(p, axis=1)
        scene.append(p[np.isfinite(d) & (d < a.max_range) & (d > 0.1)])
    node.destroy_node()
    rclpy.try_shutdown()
    return {'scene': voxel(np.concatenate(scene), 0.01), 'T_link_cloud': T_link_cloud, 'links': links,
            'n_clouds': len(clouds)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--max-range', type=float, default=2.2)
    ap.add_argument('--min-inliers', type=int, default=1500)
    ap.add_argument('--method', choices=('plane', 'icp'), default='plane')
    ap.add_argument('--table-z', default='0.0',
                    help='table height in twin/world, metres (default 0 = the Duo mount stands on the table, verified '
                         '2026-11-22: ZED-measured gripper height above the table == twin) or "auto" = search it '
                         '(weak: the score is flat, gave a 12 cm sideways error on real data)')
    ap.add_argument('--min-inliers-plane', type=int, default=300, help='matched robot points needed (plane method)')
    ap.add_argument('--max-rms', type=float, default=0.012)
    ap.add_argument('--start', choices=('config', 'head'), default='head',
                    help='head: search around a ZED on the robot head (default); config: around the current values')
    ap.add_argument('--gripper', action='store_true', help='also fit the gripper coupling height (flange z offset)')
    ap.add_argument('--write', action='store_true', help='write the result to the config (else: report only)')
    ap.add_argument('--save', help='save the captured data (.npz) for offline runs')
    ap.add_argument('--load', help='use saved data instead of the live topics')
    ap.add_argument('--cloud', default='/zed/zed_node/point_cloud/cloud_registered')
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    cam = cfg['cameras']['zed']
    if a.load:
        z = np.load(a.load, allow_pickle=True)
        data = {'scene': z['scene'], 'T_link_cloud': z['T_link_cloud'], 'links': z['links'].item(),
                'n_clouds': int(z['n_clouds'])}
    else:
        data = capture(a)
        if a.save:
            np.savez(a.save, scene=data['scene'], T_link_cloud=data['T_link_cloud'], links=data['links'],
                     n_clouds=data['n_clouds'])
            print(f'saved {a.save}')
    scene, T_lc = data['scene'], data['T_link_cloud']
    grip = cfg.get('gripper', {}).get('xyz', [0, 0, 0.011])[2]
    model = model_points({k: v for k, v in data['links'].items()})
    print(f'{len(scene)} scene points (1 cm voxels from {data["n_clouds"]} clouds), {len(model)} robot model points')
    if a.method == 'plane':
        T, rms, n, info = plane_search(scene, model, T_lc, None if a.table_z == 'auto' else float(a.table_z))
        print(f"table plane: camera {info['height_m']:.3f} m above it, pitched {info['pitch_deg']:.1f} deg, "
              f"table at z = {info['table_z']:.3f} m (scan {info.get('table_z_scan')}), "
              f"{info['table_points']} table / {info['robot_side_points']} points above it")
        a.min_inliers, a.max_rms = min(a.min_inliers, a.min_inliers_plane), max(a.max_rms, 0.0)
    else:
        starts = [mat(cam['xyz'], cam['rpy']) @ T_lc]
        if a.start == 'head':
            starts += head_starts(T_lc)
        T, rms, n = search(scene, model, starts)
    T_new = T @ np.linalg.inv(T_lc)
    xyz, rpy = T_new[:3, 3], matrix_rpy(T_new[:3, :3])
    print(f'fit: {n} robot points matched, RMS {rms * 1000:.1f} mm')
    print(f'old xyz {np.round(cam["xyz"], 3).tolist()} rpy {np.round(cam["rpy"], 3).tolist()}')
    print(f'new xyz {np.round(xyz, 4).tolist()} rpy {np.round(rpy, 4).tolist()}')
    good = n >= a.min_inliers and rms <= a.max_rms
    dz = None
    if good and a.gripper:
        scene_w = scene @ T[:3, :3].T + T[:3, 3]
        dz, nz, table = fit_gripper_offset(scene_w, data['links'])
        base = dict(table)[min(dict(table), key=lambda v: abs(v - 0.0))]
        print('gripper coupling scan (extra dz vs. config, matched points): ' +
              ' '.join(f'{d * 100:.1f}cm:{c}' for d, c in table))
        print(f'best extra dz {dz * 100:.1f} cm ({nz} vs {base} points at 0) -> gripper z = {grip + dz:.3f} m')
        if nz < base * 1.15 or dz == 0.0:
            print('  (no clear improvement - gripper offset unchanged)')
            dz = None
    if not good:
        print('NOT ACCEPTED: ' + ', '.join(x for x, bad in (
            (f'only {n} matched points (< {a.min_inliers})', n < a.min_inliers),
            (f'RMS {rms * 1000:.1f} mm > {a.max_rms * 1000:.0f} mm', rms > a.max_rms)) if bad) +
            '  - start closer with scripts/twin_calibrate.py, then: --start config')
    elif a.write:
        from twin_calibrate import read_block, set_line
        text = open(a.config).read()
        s, e = read_block(text, 'zed')
        blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in xyz) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in rpy) + ']')
        blk = set_line(blk, 'calibrated', 'true')
        blk = set_line(blk, 'parent', 'world')
        text = text[:s] + blk + text[e:]
        if dz is not None:
            gx = cfg['gripper']['xyz']
            text = re.sub(r'^(gripper:\n(?:  #.*\n)*  xyz:\s*)\[[^\]]*\]', lambda m: m.group(1) +
                          f'[{gx[0]}, {gx[1]}, {grip + dz:.4f}]', text, count=1, flags=re.M)
            print('  gripper xyz updated (takes effect when the twin is restarted: URDF)')
        with open(a.config + '.tmp', 'w') as f:
            f.write(text)
        os.replace(a.config + '.tmp', a.config)
        print(f'WRITTEN to {a.config} (camera pose: twin_state reloads it within 1 s)')
    else:
        print('ACCEPTED (report only - run with --write to apply)')
    sys.exit(0 if good else 1)


if __name__ == '__main__':
    main()
