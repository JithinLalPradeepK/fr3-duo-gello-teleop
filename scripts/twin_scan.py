#!/usr/bin/env python3
"""DIGITAL TWIN 3D SCAN: record the wrist D405s during the robots' SCAN (supervisor /demo/scan), fuse a precise coloured
3D model of the surroundings, calibrate the cameras from it. Read-only for the robots (subscribes; never commands).

  python3 scripts/twin_scan.py record     (started by launch/digital_twin.launch.py: waits for "SCAN view" in
                                           /demo/status, records, fuses when the scan completes)
  python3 scripts/twin_scan.py fuse DIR   (offline: fuse / re-fuse a recording)

RECORD   at every pose where both arms stand still (joint speed < 0.01 rad/s for 0.6 s) one frame per D405: aligned depth
         (16UC1, every 2nd pixel), colour, intrinsics, and the joint positions of BOTH arms (/twin/joint_states, by name);
         flange poses come from duo_geometry FK (the same kinematics the twin uses). One ZED cloud at the start.
FUSE     per D405: drop points closer than 0.12 m (own gripper fingers) and farther than 0.9 m; HAND-EYE: the camera pose
         X on the flange is unknown -> coarse search (optical axis directions x roll x mount offsets) scored by how well
         the views agree in POSITION + COLOUR (texture stops flat surfaces from sliding), then Nelder-Mead on the 6
         mount parameters, coarse to fine; robot arms removed from
         every view (duo_geometry capsules + 2 cm, with that view's joint positions); voxel fusion (4 mm, mean colour,
         voxels seen at least twice); ZED extrinsic re-fitted against the fused model (ICP, head-camera search).
         Writes scans/scan_<time>.npz (xyz, rgb) + scans/latest.npz, and - if the fit is good - the D405 / ZED
         extrinsics into config/digital_twin.yaml (calibrated: true).
"""
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry, rpy_matrix, seg_seg_dist  # noqa: E402
from twin_autocalib import head_starts, kabsch, matrix_rpy, search, voxel  # noqa: E402

SCANS = os.path.join(DEMO, 'scans')
CAMS = {'d405_1': 'left', 'd405_2': 'right'}
NEAR, FAR = 0.12, 0.9


# ---------------------------------------------------------------------------------------------- geometry helpers
def rot_to(a, b):
    """Rotation matrix turning unit vector a onto unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1 / (1 + c))


def fibonacci(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


def depth_points(depth_mm, K, stride=1, color=None):
    """depth (h, w) uint16 mm, K (fx, fy, cx, cy) of that image -> (xyz camera optical frame, rgb uint32)."""
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    z = depth_mm[::stride, ::stride].astype(np.float64) * 0.001
    ok = (z > NEAR) & (z < FAR)
    z, u, v = z[ok], u[ok], v[ok]
    xyz = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)
    if color is not None:
        c = color[v, u].astype(np.uint32)
        rgb = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
    else:
        rgb = np.full(len(z), 0xC8C8C8, np.uint32)
    return xyz, rgb


def robot_mask(g, pts_w, ql, qr, pad=0.02):
    """True for points that are NOT inside either arm's capsules (+pad)."""
    keep = np.ones(len(pts_w), bool)
    for arm, q in (('left', ql), ('right', qr)):
        a, b = g.world_capsules(arm, np.asarray(q, float))
        r = np.array([c[3] for c in g.caps])
        for i in range(len(a)):
            d = seg_seg_dist(np.repeat(a[i:i + 1], len(pts_w), 0), np.repeat(b[i:i + 1], len(pts_w), 0), pts_w, pts_w)
            keep &= d > r[i] + pad
    return keep


# ---------------------------------------------------------------------------------------------- hand-eye
def apply(T, p):
    return p @ T[:3, :3].T + T[:3, 3]


LAM = 0.01 / 40          # colour weight: 40 levels of difference count like 1 cm (texture stops planes from sliding)


def features(F, X, P, rgb):
    c = np.stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255], 1).astype(float) * LAM
    return np.hstack([apply(F @ X, P), c])


def he_cost(frames, X, md=0.015, n=2500, seed=0, k=6):
    """Hand-eye cost: mean truncated squared distance (position + colour) of every view's points to the nearest point
    of ANOTHER view (0 = perfect agreement, 1 = no overlap). One KD-tree over all views per call.
    frames: [(F world<-flange, P camera, rgb)]."""
    rng = np.random.default_rng(seed)
    feats, lab = [], []
    for i, (F, P, rgb) in enumerate(frames):
        sel = rng.permutation(len(P))[:n]
        feats.append(features(F, X, P[sel], rgb[sel]))
        lab.append(np.full(len(sel), i))
    A, L = np.concatenate(feats), np.concatenate(lab)
    d, idx = cKDTree(A).query(A, k=k, distance_upper_bound=md)
    idx = np.minimum(idx, len(A) - 1)
    other = (L[idx] != L[:, None]) & np.isfinite(d)
    first = np.where(other.any(1), d[np.arange(len(A)), np.argmax(other, 1)], md)
    return float(np.mean(np.minimum(first, md) ** 2) / md ** 2)


def consistency(frames, X, md=0.01, n=2500):
    """Report: share of each view's points within md (position only) of the other views."""
    rng = np.random.default_rng(3)
    W = [apply(F @ X, P[rng.permutation(len(P))[:n]]) for F, P, _c in frames]
    s = 0.0
    for i in range(len(W)):
        d, _ = cKDTree(np.concatenate([W[j] for j in range(len(W)) if j != i])).query(W[i], distance_upper_bound=md)
        s += np.isfinite(d).mean()
    return s / len(W)


def x_from(v):
    """6-vector (rotation vector, translation) -> 4x4."""
    th = np.linalg.norm(v[:3])
    k = v[:3] / th if th > 1e-12 else np.zeros(3)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    X = np.eye(4)
    X[:3, :3] = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    X[:3, 3] = v[3:]
    return X


def refine_X(frames, X0, stages=((0.03, 800, 0.05, 0.02), (0.015, 2000, 0.02, 0.008), (0.008, 3000, 0.008, 0.003))):
    """Nelder-Mead on the 6 mount parameters, coarse to fine (match radius, points per view, initial simplex)."""
    from scipy.optimize import minimize
    X = X0
    for md, n, srot, strans in stages:
        def f(v):
            return he_cost(frames, X @ x_from(v), md=md, n=n)
        simplex = np.vstack([np.zeros(6)] + [np.eye(6)[k] * (srot if k < 3 else strans) for k in range(6)])
        r = minimize(f, np.zeros(6), method='Nelder-Mead',
                     options={'initial_simplex': simplex, 'xatol': 1e-4, 'fatol': 1e-5, 'maxfev': 400})
        X = X @ x_from(r.x)
    return X


def solve_hand_eye(frames, mount_offsets=((0.0, 0.0, 0.05), (0.06, 0.0, 0.05), (-0.06, 0.0, 0.05),
                                           (0.0, 0.06, 0.05), (0.0, -0.06, 0.05)), dirs=100, rolls=12, keep=3,
                   hops=6, log=print):
    """Coarse search over the optical axis direction (flange frame), roll and mount offset with a fast version of the
    cost, Nelder-Mead from the best `keep`, then `hops` random restarts (3 deg / 2 cm) around the best.
    Returns (X flange<-camera optical, consistency)."""
    cands = []
    for off in mount_offsets:
        for d in fibonacci(dirs):
            R0 = rot_to(np.array([0.0, 0.0, 1.0]), d)
            for k in range(rolls):
                a = 2 * math.pi * k / rolls
                X = np.eye(4)
                X[:3, :3] = R0 @ np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
                X[:3, 3] = off
                cands.append(X)
    scored = sorted((he_cost(frames, X, md=0.03, n=300), i) for i, X in enumerate(cands))
    log(f'  hand-eye coarse: {len(cands)} candidates, best costs {[round(c, 3) for c, _ in scored[:5]]}')
    best = None
    for c0, i in scored[:keep]:
        X = refine_X(frames, cands[i])
        c = he_cost(frames, X, md=0.008, n=3000)
        log(f'    candidate (coarse {c0:.3f}) -> refined cost {c:.4f}')
        if best is None or c < best[1]:
            best = (X, c)
    rng = np.random.default_rng(7)
    for h in range(hops):
        v = np.concatenate([rng.normal(0, math.radians(3), 3), rng.normal(0, 0.02, 3)])
        X = refine_X(frames, best[0] @ x_from(v), stages=((0.015, 2000, 0.02, 0.008), (0.008, 3000, 0.008, 0.003)))
        c = he_cost(frames, X, md=0.008, n=3000)
        if c < best[1] - 1e-4:
            log(f'    restart {h + 1}: cost {best[1]:.4f} -> {c:.4f}')
            best = (X, c)
    return best[0], consistency(frames, best[0])


# ---------------------------------------------------------------------------------------------- fusion
def fuse(rec, cfg, log=print, voxel_size=0.004, min_hits=2):
    """rec: dict from load_recording. Returns dict(xyz, rgb, X{cam}, score{cam}, zed)."""
    g = DuoGeometry()
    out = {'X': {}, 'score': {}}
    world_pts, world_rgb = [], []
    for cam, arm in CAMS.items():
        frames, colored = [], []
        for f in rec['frames']:
            if cam not in f:
                continue
            c = f[cam]
            P, rgb = depth_points(c['depth'], c['K'], 1, c['color'])
            if len(P) < 500:
                continue
            F = g.link_frames(arm, f['q'][arm])[8]
            frames.append((F, P, rgb))
            colored.append((F, P, rgb, f['q']))
        if len(frames) < 4:
            log(f'{cam}: only {len(frames)} usable frames - skipped')
            continue
        log(f'{cam}: {len(frames)} frames, {sum(len(P) for _, P, _c in frames)} points')
        X, score = solve_hand_eye(frames, log=log)
        out['X'][cam], out['score'][cam] = X, score
        log(f'{cam}: hand-eye consistency {score:.3f}  xyz {np.round(X[:3, 3], 4).tolist()} '
            f'rpy {np.round(matrix_rpy(X[:3, :3]), 3).tolist()}')
        for F, P, rgb, q in colored:
            W = apply(F @ X, P)
            keep = robot_mask(g, W, q['left'], q['right'])
            world_pts.append(W[keep])
            world_rgb.append(rgb[keep])
    if not world_pts:
        return None
    P = np.concatenate(world_pts)
    C = np.concatenate(world_rgb)
    k = np.floor(P / voxel_size).astype(np.int64)
    _, inv, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    n = cnt.size
    s = np.zeros((n, 3))
    np.add.at(s, inv, P)
    col = np.zeros((n, 3))
    np.add.at(col, inv, np.stack([(C >> 16) & 255, (C >> 8) & 255, C & 255], 1).astype(float))
    good = cnt >= min_hits
    xyz = (s[good] / cnt[good, None]).astype(np.float32)
    c = (col[good] / cnt[good, None]).astype(np.uint32)
    out['xyz'], out['rgb'] = xyz, (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
    log(f'fused: {len(xyz)} voxels of {voxel_size * 1000:.0f} mm (from {len(P)} points)')
    # ZED: re-fit its pose against the fused model
    if rec.get('zed') is not None and len(xyz) > 5000:
        scene, T_lc = rec['zed']['scene'], rec['zed']['T_link_cloud']
        cam = cfg['cameras']['zed']
        T0 = np.eye(4)
        T0[:3, :3] = rpy_matrix(*cam['rpy'])
        T0[:3, 3] = cam['xyz']
        T, rms, nm = search(scene, voxel(xyz.astype(float), 0.008), [T0 @ T_lc] + head_starts(T_lc))
        Tz = T @ np.linalg.inv(T_lc)
        out['zed'] = {'xyz': Tz[:3, 3], 'rpy': matrix_rpy(Tz[:3, :3]), 'rms': rms, 'n': nm}
        log(f'ZED vs scan: {nm} points matched, RMS {rms * 1000:.1f} mm -> xyz {np.round(Tz[:3, 3], 3).tolist()} '
            f'rpy {np.round(matrix_rpy(Tz[:3, :3]), 3).tolist()}')
    return out


def write_results(out, cfg_path, log=print, min_score=0.5, zed_rms=0.012, zed_n=3000):
    from twin_calibrate import read_block, set_line
    os.makedirs(SCANS, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(SCANS, f'scan_{stamp}.npz')
    np.savez_compressed(path, xyz=out['xyz'], rgb=out['rgb'])
    tmp = os.path.join(SCANS, 'latest.npz.tmp.npz')
    np.savez_compressed(tmp, xyz=out['xyz'], rgb=out['rgb'])
    os.replace(tmp, os.path.join(SCANS, 'latest.npz'))
    log(f'saved {path} and scans/latest.npz')
    text = open(cfg_path).read()
    changed = False
    # GUARD (2026-11-22): mutual consistency alone is degenerate for small scan motions - the real scan gave 0.82
    # consistency with the fused model 7.7 cm above the table. The table (mount base plane, z = 0) must come out at z~0.
    zz = out['xyz'][:, 2]
    hist, edges = np.histogram(zz, bins=np.arange(-0.2, 0.3, 0.005))
    table_z = float(edges[np.argmax(hist)] + 0.0025)
    table_ok = abs(table_z) < 0.01
    log(f'fused model: dominant plane at z = {table_z * 1000:.0f} mm ({"OK" if table_ok else "NOT at the table"})')
    for cam, X in out['X'].items():
        if out['score'][cam] < min_score:
            log(f'{cam}: consistency {out["score"][cam]:.2f} < {min_score} - extrinsics NOT written')
            continue
        if not table_ok:
            log(f'{cam}: table plane at {table_z * 1000:.0f} mm in the fused model - extrinsics NOT written '
                f'(refine with scripts/twin_handeye_ref.py {SCANS}/rec_<time>)')
            continue
        # the config places the RealSense ROOT frame <cam>_link; X is flange <- colour optical frame: the D405 optical
        # frame is (z forward, x right) relative to <cam>_link (x forward, z up) rotated by rpy (-pi/2, 0, -pi/2)
        L_opt = np.eye(4)
        L_opt[:3, :3] = rpy_matrix(-math.pi / 2, 0, -math.pi / 2)
        T = X @ np.linalg.inv(L_opt)
        s, e = read_block(text, cam)
        blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in T[:3, 3]) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in matrix_rpy(T[:3, :3])) + ']')
        blk = set_line(blk, 'calibrated', 'true')
        text = text[:s] + blk + text[e:]
        changed = True
        log(f'{cam}: extrinsics written (calibrated from the scan)')
    z = out.get('zed')
    cz = (yaml.safe_load(text).get('cameras') or {}).get('zed') or {}
    if z and cz.get('calibrated'):                    # GUARD: never replace a verified ZED pose by a scan refit that
        dxyz = float(np.linalg.norm(np.array(z['xyz']) - np.array(cz['xyz'], float)))       # disagrees with it
        drpy = float(np.abs(np.array(z['rpy']) - np.array(cz['rpy'], float)).max())
        if dxyz > 0.03 or drpy > 0.035:
            log(f'ZED: scan fit differs from the calibrated pose by {dxyz * 100:.1f} cm / {np.degrees(drpy):.1f} deg '
                f'- NOT written (calibrated pose kept)')
            z = None
    if z and z['rms'] <= zed_rms and z['n'] >= zed_n:
        s, e = read_block(text, 'zed')
        blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in z['xyz']) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in z['rpy']) + ']')
        blk = set_line(blk, 'calibrated', 'true')
        text = text[:s] + blk + text[e:]
        changed = True
        log('ZED: extrinsics written (fitted to the scan)')
    elif z:
        log(f'ZED: fit not good enough (RMS {z["rms"] * 1000:.1f} mm, {z["n"]} points) - NOT written')
    if changed:
        with open(cfg_path + '.tmp', 'w') as f:
            f.write(text)
        os.replace(cfg_path + '.tmp', cfg_path)
    return path


# ---------------------------------------------------------------------------------------------- recording files
def save_recording(d, rec):
    os.makedirs(d, exist_ok=True)
    meta = []
    for i, f in enumerate(rec['frames']):
        m = {'i': i, 't': f['t'], 'q': {a: list(map(float, f['q'][a])) for a in ('left', 'right')}, 'cams': {}}
        for cam in CAMS:
            if cam in f:
                extra = {'color_full': f[cam]['color_full']} if 'color_full' in f[cam] else {}
                np.savez_compressed(os.path.join(d, f'f{i:03d}_{cam}.npz'), depth=f[cam]['depth'],
                                    color=f[cam]['color'], **extra)
                m['cams'][cam] = list(map(float, f[cam]['K']))
                if 'K_full' in f[cam]:
                    m.setdefault('cams_full', {})[cam] = list(map(float, f[cam]['K_full']))
        meta.append(m)
    json.dump(meta, open(os.path.join(d, 'frames.json'), 'w'), indent=1)
    if rec.get('zed') is not None:
        np.savez_compressed(os.path.join(d, 'zed.npz'), **rec['zed'])
    for i, zp in enumerate(rec.get('zed_photos') or []):
        np.savez_compressed(os.path.join(d, f'zed_photo_{i:02d}.npz'), rgb=zp['rgb'], K=np.array(zp['K']), T=zp['T'],
                            q=json.dumps(zp.get('q') or {}))


def export_views(rec, cfg, out_dir, log=print):
    """Photos of a 3D SCAN for the photoreal twin's multi-view texturing (isaac/texturing.py): every ZED photo (same
    camera, arms elsewhere) and - only if the mounts are CALIBRATED - every wrist D405 photo, with intrinsics, camera
    pose (optical -> world), the arms' joint positions and (D405) the depth image. Writes out_dir/views.json last."""
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):
        os.remove(os.path.join(out_dir, f))
    g = DuoGeometry()
    cams = cfg.get('cameras') or {}
    L_opt = np.eye(4)
    L_opt[:3, :3] = rpy_matrix(-math.pi / 2, 0, -math.pi / 2)
    views = []
    photos = rec.get('zed_photos') or []
    for i, zp in enumerate(photos):
        if not zp.get('q') and len(photos) <= len(rec['frames']):
            zp['q'] = rec['frames'][i]['q']            # photo i is requested at the capture of view i (arms still)
        name = f'zed_{i:02d}.jpg'
        cv2.imwrite(os.path.join(out_dir, name), zp['rgb'][:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
        views.append({'image': name, 'kind': 'zed', 'K': list(map(float, zp['K'])),
                      'wh': [int(zp['rgb'].shape[1]), int(zp['rgb'].shape[0])], 'T': np.asarray(zp['T']).tolist(),
                      'q': zp.get('q') or {}, 'depth': None})
    n_wrist = 0
    for cam, arm in CAMS.items():
        c = cams.get(cam) or {}
        if not c.get('calibrated'):
            continue
        X = np.eye(4)
        X[:3, :3] = rpy_matrix(*[float(v) for v in c['rpy']])
        X[:3, 3] = [float(v) for v in c['xyz']]
        X = X @ L_opt
        for j, f in enumerate(rec['frames']):
            fc = f.get(cam)
            if not fc or 'color_full' not in fc:
                continue
            T = g.link_frames(arm, np.asarray(f['q'][arm], float))[8] @ X
            name = f'{cam}_{j:02d}'
            cv2.imwrite(os.path.join(out_dir, name + '.jpg'), fc['color_full'][:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
            np.save(os.path.join(out_dir, name + '_depth.npy'), (fc['depth'].astype(np.float32) * 0.001).astype(np.float16))
            views.append({'image': name + '.jpg', 'kind': cam, 'K': list(map(float, fc['K_full'])),
                          'wh': [int(fc['color_full'].shape[1]), int(fc['color_full'].shape[0])], 'T': T.tolist(),
                          'q': f['q'], 'depth': name + '_depth.npy',
                          'depth_wh': [int(fc['depth'].shape[1]), int(fc['depth'].shape[0])]})
            n_wrist += 1
    json.dump(views, open(os.path.join(out_dir, 'views.json.tmp'), 'w'))
    os.replace(os.path.join(out_dir, 'views.json.tmp'), os.path.join(out_dir, 'views.json'))
    log(f'texture views exported: {len(views) - n_wrist} head-camera + {n_wrist} wrist photos -> {out_dir}')
    return len(views)


def load_recording(d):
    meta = json.load(open(os.path.join(d, 'frames.json')))
    frames = []
    for m in meta:
        f = {'t': m['t'], 'q': m['q']}
        for cam, K in m['cams'].items():
            z = np.load(os.path.join(d, f'f{m["i"]:03d}_{cam}.npz'))
            f[cam] = {'depth': z['depth'], 'color': z['color'], 'K': K}
            if 'color_full' in z.files and cam in m.get('cams_full', {}):
                f[cam]['color_full'], f[cam]['K_full'] = z['color_full'], m['cams_full'][cam]
        frames.append(f)
    zp = os.path.join(d, 'zed.npz')
    zed = dict(np.load(zp)) if os.path.exists(zp) else None
    photos = []
    for pp in sorted(f for f in os.listdir(d) if f.startswith('zed_photo_') and f.endswith('.npz')):
        z = np.load(os.path.join(d, pp))
        photos.append({'rgb': z['rgb'], 'K': list(z['K']), 'T': z['T'],
                       'q': json.loads(str(z['q'])) if 'q' in z.files else {}})
    return {'frames': frames, 'zed': zed, 'zed_photos': photos}


# ---------------------------------------------------------------------------------------------- ROS recorder
def record(cfg_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from twin_cloud import cloud_view
    from twin_state import quat_matrix

    class Rec(Node):
        def __init__(self):
            super().__init__('twin_scan')
            lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.state, self.msg = '', ''
            self.q, self.hist = {}, []
            self.img = {c: {} for c in CAMS}
            self.rec, self.active, self.last_cap = None, False, None
            self.zed_msg = None
            self.status_pub = self.create_publisher(String, '/twin/scan_status', lat)
            # the ZED map (twin_recon) = verified reference for the wrist-camera mounts; built WHILE the arms are lifted
            self.recon_cmd = self.create_publisher(String, '/twin/recon/cmd', 5)
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self.tfl = TransformListener(self.tf, self)
            self.zinfo, self.photo_sub, self.want_photo = None, None, False
            self.create_subscription(CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info',
                                     lambda m: setattr(self, 'zinfo', m), qos_profile_sensor_data)
            self.create_subscription(String, '/demo/status', self.on_demo, lat)
            self.create_subscription(JointState, '/twin/joint_states', self.on_js, 50)
            for c in CAMS:
                ns = f'/camera/{c}'
                self.create_subscription(CameraInfo, f'{ns}/color/camera_info',
                                         lambda m, c=c: self.img[c].__setitem__('info', m), qos_profile_sensor_data)
                self.create_subscription(Image, f'{ns}/color/image_raw',
                                         lambda m, c=c: self.img[c].__setitem__('color', m), qos_profile_sensor_data)
                self.create_subscription(Image, f'{ns}/aligned_depth_to_color/image_raw',
                                         lambda m, c=c: self.img[c].__setitem__('depth', m), qos_profile_sensor_data)
            self.zed_sub = None
            self.buf = Buffer()
            TransformListener(self.buf, self)
            self.create_timer(0.1, self.tick)
            self.say('idle - waiting for a SCAN (Operator > 3D SCAN)')

        def map_points(self, timeout=30.0):
            """Ask twin_recon to save its map (scans/recon_map.npz), wait for the new file, return its surface points."""
            from twin_handeye_ref import reference_from_map
            cfg = yaml.safe_load(open(cfg_path)) or {}
            path = os.path.join(DEMO, (cfg.get('reconstruction') or {}).get('map_file', 'scans/recon_map.npz'))
            t0 = os.path.getmtime(path) if os.path.exists(path) else 0.0
            self.recon_cmd.publish(String(data=json.dumps({'save': True})))
            end = time.time() + timeout
            while time.time() < end:
                time.sleep(0.5)
                if os.path.exists(path) and os.path.getmtime(path) > t0:
                    time.sleep(1.0)                            # .ply follows; the npz is complete once renamed
                    return reference_from_map(path)
            return np.zeros((0, 3))

        def calibrate_and_integrate(self, out, d, n):
            from twin_handeye_ref import refine_mounts, write_mounts
            log = lambda s: self.get_logger().info(s)        # noqa: E731
            ref = self.map_points()
            if len(ref) < 5000:
                return 'wrist cameras: head-camera map too small for calibration'
            import twin_board
            self.say('wrist cameras: looking for the calibration board ...', frames=n, busy=True)
            res = twin_board.calibrate(self.rec, log=log)
            nw = twin_board.write(cfg_path, res, log=log)
            if not nw:                                         # no board: map-based solve (strict acceptance)
                self.say('calibrating the wrist cameras against the head-camera map (a few minutes) ...', frames=n,
                         busy=True)
                res = refine_mounts(self.rec, yaml.safe_load(open(cfg_path)), ref, log=log, starts_extra=out.get('X'))
                nw = write_mounts(cfg_path, res, log=log)
            for r in res.values():                             # board results have no object metric
                r.setdefault('obj_mm', r.get('spread_mm', 0.0))
            cams = yaml.safe_load(open(cfg_path)).get('cameras') or {}
            if any((cams.get(c) or {}).get('calibrated') for c in CAMS):
                def go():
                    self.recon_cmd.publish(String(data=json.dumps({'integrate': d})))
                    self.once.cancel()
                self.once = self.create_timer(3.0, go)        # after twin_recon reset for the new mounts
            parts = [f"{'L' if c == 'd405_1' else 'R'}-wrist {'calibrated' if r['ok'] else 'not calibrated'} "
                     f"({r['obj_mm']:.0f} mm)" for c, r in res.items()]
            return 'wrist cameras: ' + ', '.join(parts) + (' - fused into the map' if nw or any(
                (cams.get(c) or {}).get('calibrated') for c in CAMS) else ' - head camera only')

        def on_zed_photo(self, m):
            """One ZED photo + its world pose per scan: reference for the ChArUco board (scripts/twin_board.py)."""
            if not self.want_photo or self.zinfo is None:
                return
            try:
                t = self.tf.lookup_transform('twin/world', m.header.frame_id, rclpy.time.Time())
            except Exception:       # noqa: BLE001 - TF not there yet: next image
                return
            import cv2
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            q, p = t.transform.rotation, t.transform.translation
            T = np.eye(4)
            T[:3, :3] = quat_matrix(q.x, q.y, q.z, q.w)
            T[:3, 3] = (p.x, p.y, p.z)
            k = self.zinfo.k
            sx = img.shape[1] / max(self.zinfo.width, 1)
            self.rec.setdefault('zed_photos', []).append({'rgb': img[:, :, ::-1].copy(),
                                                          'K': [k[0] * sx, k[4] * sx, k[2] * sx, k[5] * sx], 'T': T,
                                                          'q': {a: self.q[a][1].tolist() for a in ('left', 'right')}})
            self.want_photo = False
            self.destroy_subscription(self.photo_sub)
            self.photo_sub = None
            self.get_logger().info(f'ZED photo {len(self.rec["zed_photos"])} for the board calibration recorded')

        def say(self, text, **kw):
            self.get_logger().info(text)
            self.status_pub.publish(String(data=json.dumps(dict(kw, text=text, t=time.time()))))

        def on_demo(self, m):
            try:
                s = json.loads(m.data)
            except ValueError:
                return
            self.state, self.msg = s.get('state', ''), s.get('message', '')

        def on_js(self, m):
            if m.header.frame_id == 'twin_placeholder' or not m.name:
                return
            for arm in ('left', 'right'):
                if m.name[0].startswith(f'{arm}_fr3_joint'):
                    p = dict(zip(m.name, m.position))
                    q = [p.get(f'{arm}_fr3_joint{i}') for i in range(1, 8)]
                    if None not in q:
                        self.q[arm] = (time.monotonic(), np.array(q))
                        self.hist.append((time.monotonic(), arm, np.array(q)))
            cut = time.monotonic() - 0.8
            while self.hist and self.hist[0][0] < cut:
                self.hist.pop(0)

        def still(self):
            if len(self.q) < 2:
                return False
            for arm in ('left', 'right'):
                h = [(t, q) for t, a, q in self.hist if a == arm]
                if len(h) < 10 or h[-1][0] - h[0][0] < 0.6:
                    return False
                if np.max(np.abs(h[-1][1] - h[0][1])) > 0.006:
                    return False
            return True

        def tick(self):
            scanning = self.state == 'HOMING' and self.msg.startswith('SCAN view')
            if scanning and not self.active:
                self.active, self.rec, self.last_cap = True, {'frames': [], 'zed': None}, None
                self.zed_sub = self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered',
                                                        lambda m: setattr(self, 'zed_msg', m), qos_profile_sensor_data)
                self.say('recording: the robots look around', frames=0)
            if self.active and self.rec['zed'] is None and self.zed_msg is not None:
                self.grab_zed()
            if self.active and self.want_photo and self.photo_sub is None:
                from sensor_msgs.msg import CompressedImage
                self.photo_sub = self.create_subscription(CompressedImage, '/zed/zed_node/rgb/color/rect/image/compressed',
                                                          self.on_zed_photo, qos_profile_sensor_data)
            if self.active and scanning and self.still():
                pose = np.concatenate([self.q['left'][1], self.q['right'][1]])
                if self.last_cap is None or np.max(np.abs(pose - self.last_cap)) > 0.03:
                    if self.capture():
                        self.last_cap = pose
            if self.active and not scanning and self.state != 'HOMING':
                complete = self.msg.startswith('scan complete')
                if self.still() and complete:          # final view = home: capture it too
                    self.capture()
                self.active = False
                if self.zed_sub is not None:
                    self.destroy_subscription(self.zed_sub)
                    self.zed_sub = None
                n = len(self.rec['frames'])
                d = os.path.join(SCANS, 'rec_' + time.strftime('%Y%m%d_%H%M%S'))
                save_recording(d, self.rec)
                if not complete:
                    return self.say(f'scan aborted ({self.msg}) - {n} views kept in {d}, nothing fused', frames=n)
                self.say(f'fusing {n} views ... (about a minute)', frames=n, busy=True)
                try:
                    out = fuse(self.rec, yaml.safe_load(open(cfg_path)), log=lambda s: self.get_logger().info(s))
                    if out is None:
                        return self.say('fusion failed: not enough depth data', frames=n)
                    write_results(out, cfg_path, log=lambda s: self.get_logger().info(s))
                    wrist = self.calibrate_and_integrate(out, d, n)
                    try:
                        export_views(self.rec, yaml.safe_load(open(cfg_path)), os.path.join(DEMO, 'isaac', 'cache', 'views'),
                                     log=lambda s: self.get_logger().info(s))
                    except Exception as e:      # noqa: BLE001
                        self.get_logger().warn(f'texture views: {e}')
                    self.say(f'3D scan ready: {len(out["xyz"])} points; {wrist}', frames=n, points=int(len(out['xyz'])),
                             score={k: round(v, 3) for k, v in out['score'].items()})
                except Exception as e:      # noqa: BLE001 - never take the twin down
                    self.say(f'fusion error: {e}', frames=n)

        def capture(self):
            f = {'t': time.time(), 'q': {a: self.q[a][1].tolist() for a in ('left', 'right')}}
            for c in CAMS:
                im = self.img[c]
                if not all(k in im for k in ('info', 'color', 'depth')):
                    continue
                d, col, info = im['depth'], im['color'], im['info']
                if d.encoding not in ('16UC1', 'mono16') or col.encoding not in ('rgb8', 'bgr8'):
                    continue
                dep = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, -1)[:, :d.width]
                cc = np.frombuffer(bytes(col.data), np.uint8).reshape(col.height, -1)[:, :col.width * 3]
                cc = cc.reshape(col.height, col.width, 3)
                if col.encoding == 'bgr8':
                    cc = cc[..., ::-1]
                if cc.shape[:2] != dep.shape:
                    continue
                s = 2
                f[c] = {'depth': dep[::s, ::s].copy(), 'color': cc[::s, ::s].copy(),
                        'K': [info.k[0] / s, info.k[4] / s, info.k[2] / s, info.k[5] / s],
                        'color_full': cc.copy(), 'K_full': [info.k[0], info.k[4], info.k[2], info.k[5]]}
            if not any(c in f for c in CAMS):
                return False
            self.rec['frames'].append(f)
            self.want_photo = True               # one ZED photo per view: the board is seen best with the arms lifted
            self.say(f'recording: view {len(self.rec["frames"])} captured', frames=len(self.rec['frames']))
            return True

        def grab_zed(self):
            m = self.zed_msg
            try:
                t = self.buf.lookup_transform('zed_camera_link', m.header.frame_id, rclpy.time.Time())
            except Exception:       # noqa: BLE001
                return
            q, p = t.transform.rotation, t.transform.translation
            T = np.eye(4)
            T[:3, :3] = quat_matrix(q.x, q.y, q.z, q.w)
            T[:3, 3] = (p.x, p.y, p.z)
            v = cloud_view(m).ravel()
            P = np.stack([v['x'], v['y'], v['z']], 1).astype(float)
            dd = np.linalg.norm(P, axis=1)
            self.rec['zed'] = {'scene': voxel(P[np.isfinite(dd) & (dd > 0.1) & (dd < 2.2)], 0.01), 'T_link_cloud': T}

    rclpy.init()
    n = Rec()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=('record', 'fuse'))
    ap.add_argument('dir', nargs='?')
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    ap.add_argument('--no-write', action='store_true')
    a = ap.parse_args(sys.argv[1:] if '--ros-args' not in sys.argv else sys.argv[1:sys.argv.index('--ros-args')])
    if a.mode == 'record':
        return record(a.config)
    rec = load_recording(a.dir)
    out = fuse(rec, yaml.safe_load(open(a.config)))
    if out is None:
        sys.exit('fusion failed')
    if not a.no_write:
        write_results(out, a.config)


if __name__ == '__main__':
    main()
