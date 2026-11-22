#!/usr/bin/env python3
"""Wrist D405 mount (hand-eye) against a VERIFIED reference: the ZED map of the workspace. Offline, read-only.

  python3 scripts/twin_handeye_ref.py scans/rec_<time> [--write]

Why: scripts/twin_scan.py solves the mount from the scan views' mutual consistency only. With small scan motions that
is degenerate: on 2026-11-22 both mounts came out self-consistent (0.82) but the fused model sat 7.7 cm above the table
and ~0.3 m off the ZED map (and its ZED refit then slid onto the wrong model). Here the reference is fixed: the ZED
reconstruction (ZED extrinsic verified by projecting the robot meshes into its image) + the table plane z = 0.

Method (per camera): reference points r (ZED mesh, densified) are moved into each view's FLANGE frame (FK of that
view's joint positions): r' = F_i^-1 r. Then the mount X is ONE rigid transform taking all views' camera points p onto
r' -> trimmed ICP with Kabsch across all views at once (coarse-to-fine match distance). Multi-start (scan result,
nominal guess, wrist-roll variants); score = share of points within 1 cm of the map, minus points below the table.
Accepted only if score is high and both wrists agree (identical hardware) - then --write sets cameras.<d405>.
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
from duo_geometry import DuoGeometry, rpy_matrix  # noqa: E402
from twin_autocalib import kabsch, matrix_rpy  # noqa: E402
from twin_scan import CAMS, depth_points, he_cost, load_recording, x_from  # noqa: E402
from twin_autocalib import table_plane  # noqa: E402
from scipy.optimize import minimize  # noqa: E402


def mat(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


OPT = np.array([[0, 0, 1.0, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])     # camera body -> optical


def reference(mesh_file):
    z = np.load(mesh_file)
    v, f = z['points'].astype(float), z['faces']
    tri = v[f]
    pts = np.concatenate([v, tri.mean(1), (tri[:, 0] + tri[:, 1]) / 2, (tri[:, 1] + tri[:, 2]) / 2,
                          (tri[:, 0] + tri[:, 2]) / 2])
    return pts


def frames_for(rec, cam, arm, g, per_frame=3000, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for f in rec['frames']:
        if cam not in f:
            continue
        p, _ = depth_points(f[cam]['depth'], f[cam]['K'], stride=2)
        if len(p) > per_frame:
            p = p[rng.permutation(len(p))[:per_frame]]
        F = g.link_frames(arm, np.asarray(f['q'][arm], float))[8]
        out.append((F, p))
    return out


def score(frames, X, tree):
    pw = np.concatenate([p @ (F @ X)[:3, :3].T + (F @ X)[:3, 3] for F, p in frames])
    d, _ = tree.query(pw, distance_upper_bound=0.05)
    below = (pw[:, 2] < -0.015).mean()
    return float((d < 0.01).mean() - 2 * below), float(np.median(d[np.isfinite(d)]) if np.isfinite(d).any() else 1), \
        float(below)


def icp_mount(frames, ref, X0, stages=(0.06, 0.03, 0.015, 0.008), iters=12):
    X = X0.copy()
    for md in stages:
        for _ in range(iters):
            src, dst = [], []
            for F, p in frames:
                Fi = np.linalg.inv(F)
                rl = ref @ Fi[:3, :3].T + Fi[:3, 3]            # reference in this view's flange frame
                q = p @ X[:3, :3].T + X[:3, 3]
                d, j = cKDTree(rl).query(q, distance_upper_bound=md)
                ok = np.isfinite(d)
                src.append(p[ok])
                dst.append(rl[j[ok]])
            src, dst = np.concatenate(src), np.concatenate(dst)
            if len(src) < 300:
                break
            Xn = kabsch(src, dst)                          # rigid 4x4 minimising |Xn p - r'|
            if np.abs(Xn - X).max() < 1e-5:
                X = Xn
                break
            X = Xn
    return X


def solve(frames, ref, starts):
    tree = cKDTree(ref)
    res = []
    for name, X0 in starts:
        X = icp_mount(frames, ref, X0)
        s, med, below = score(frames, X, tree)
        res.append((s, name, X, med, below))
    res.sort(key=lambda r: -r[0])
    return res


def plane_frames(rec, cam, arm, g, seed=0):
    """Per view: (F, P, rgb) for the consistency cost and the camera-frame TABLE points (dominant plane, RANSAC)."""
    rng = np.random.default_rng(seed)
    out = []
    for f in rec['frames']:
        if cam not in f:
            continue
        P, rgb = depth_points(f[cam]['depth'], f[cam]['K'], stride=2, color=f[cam]['color'])
        if len(P) < 500:
            continue
        sub = P[rng.permutation(len(P))[:6000]]
        n, d, inl = table_plane(sub, tol=0.006, iters=400)
        tab = sub[inl]
        tab = tab[rng.permutation(len(tab))[:800]]
        F = g.link_frames(arm, np.asarray(f['q'][arm], float))[8]
        out.append((F, P, rgb, tab))
    return out


def v_of(X):
    R = X[:3, :3]
    ang = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if ang < 1e-9:
        rv = np.zeros(3)
    else:
        rv = ang / (2 * np.sin(ang)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return np.concatenate([rv, X[:3, 3]])


def plane_cost(pf, X, md=0.02):
    z = np.concatenate([(tab @ (F @ X)[:3, :3].T + (F @ X)[:3, 3])[:, 2] for F, _P, _c, tab in pf])
    return float(np.mean(np.minimum(np.abs(z), md) ** 2) / md ** 2)


def map_cost(pf, X, tree_obj, md=0.03):
    """Above-table points (tray rims, cube, ball, cups) vs the ZED map's above-table points: pins the in-plane slide
    that the table plane cannot see."""
    pw = np.concatenate([P[::9] @ (F @ X)[:3, :3].T + (F @ X)[:3, 3] for F, P, _c, _t in pf])
    obj = pw[pw[:, 2] > 0.015]
    if len(obj) < 50:
        return 1.0
    d, _ = tree_obj.query(obj, distance_upper_bound=md)
    return float(np.mean(np.minimum(d, md) ** 2) / md ** 2)


def solve_plane(pf, starts, w_cons=1.0, tree_obj=None):
    frames = [(F, P, rgb) for F, P, rgb, _t in pf]
    res = []
    for name, X0 in starts:
        v = minimize(lambda v: plane_cost(pf, x_from(v)), v_of(X0), method='Nelder-Mead',
                     options={'xatol': 1e-5, 'fatol': 1e-6, 'maxiter': 3000}).x
        v = minimize(lambda v: plane_cost(pf, x_from(v)) + w_cons * he_cost(frames, x_from(v), n=1500),
                     v, method='Nelder-Mead', options={'xatol': 1e-5, 'fatol': 1e-6, 'maxiter': 2500}).x
        if tree_obj is not None:                          # last stage: + ZED map objects (in-plane slide)
            v = minimize(lambda v: plane_cost(pf, x_from(v)) + w_cons * he_cost(frames, x_from(v), n=1500)
                         + map_cost(pf, x_from(v), tree_obj), v, method='Nelder-Mead',
                         options={'xatol': 1e-5, 'fatol': 1e-6, 'maxiter': 2500}).x
        X = x_from(v)
        z = np.concatenate([(tab @ (F @ X)[:3, :3].T + (F @ X)[:3, 3])[:, 2] for F, _P, _c, tab in pf])
        pw = np.concatenate([P[::7] @ (F @ X)[:3, :3].T + (F @ X)[:3, 3] for F, P, _c, _t in pf])
        res.append((plane_cost(pf, X) + w_cons * he_cost(frames, X, n=1500), name, X, float(np.median(np.abs(z))),
                    float((pw[:, 2] < -0.015).mean())))
    res.sort(key=lambda r: r[0])
    return res


def object_agreement(pf, X, tree_obj):
    """Above-table wrist points vs the ZED map's above-table points: (median distance of those with a map point within
    5 cm, share within 1 cm)."""
    pw = np.concatenate([P[::5] @ (F @ X)[:3, :3].T + (F @ X)[:3, 3] for F, P, _c, _t in pf])
    obj = pw[pw[:, 2] > 0.015]
    if len(obj) < 100:
        return 1.0, 0.0
    d, _ = tree_obj.query(obj, distance_upper_bound=0.05)
    near = np.isfinite(d)
    return (float(np.median(d[near])) if near.any() else 1.0), float((d < 0.01).mean())


def refine_mounts(rec, cfg, ref, log=print, starts_extra=None, max_plane=0.005, max_obj=0.008, min_obj=0.4,
                  max_cross=0.010, min_cross=0.5):
    """Mounts of both wrist D405s against the verified reference (table z = 0 + ZED map). Returns
    {cam: dict(X=body 4x4, plane_mm, obj_mm, obj_share, ok)} - ok = this camera passed the acceptance checks."""
    g = DuoGeometry()
    tree_obj = cKDTree(ref[ref[:, 2] > 0.015])
    out = {}
    for cam, arm in CAMS.items():
        pf = plane_frames(rec, cam, arm, g)
        if len(pf) < 4:
            log(f'{cam}: only {len(pf)} views - skipped')
            continue
        c = cfg['cameras'][cam]
        starts = [('config', mat(c['xyz'], c['rpy']) @ OPT)]
        if starts_extra and cam in starts_extra:
            starts.append(('scan', starts_extra[cam]))
        res = solve_plane(pf, starts, tree_obj=tree_obj)
        _cost, name, X, plane, _below = res[0]
        obj_med, obj_share = object_agreement(pf, X, tree_obj)
        ok = plane <= max_plane and obj_med <= max_obj and obj_share >= min_obj
        Xb = X @ np.linalg.inv(OPT)
        out[cam] = dict(X=Xb, plane_mm=plane * 1e3, obj_mm=obj_med * 1e3, obj_share=obj_share, ok=ok, start=name)
        log(f'{cam}: plane {plane * 1e3:.1f} mm, objects vs ZED map median {obj_med * 1e3:.1f} mm, '
            f'{obj_share * 100:.0f} % within 1 cm -> {"ACCEPTED" if ok else "not accepted"} '
            f'(xyz {np.round(Xb[:3, 3], 4).tolist()})')
    if len(out) == 2:
        dis = float(np.linalg.norm(out['d405_1']['X'][:3, 3] - out['d405_2']['X'][:3, 3]))
        log(f'the two wrist mounts differ by {dis * 1e3:.1f} mm (identical brackets would give ~0)')
        # ACCEPTANCE 2026-11-22: the ZED map is too coarse for small objects (no clear optimum when shifting the wrist
        # objects onto it), so the independent evidence is: table plane at the MEASURED height (fingertips 27 cm
        # above the table = twin z 0, user ruler measurement) + the two wrists, solved independently, seeing the same
        # objects in the same place.
        med, share = cross_agreement(rec, g, out)
        ok = med <= max_cross and share >= min_cross
        log(f'left vs right wrist on the same objects: median {med * 1e3:.1f} mm, {share * 100:.0f} % within 1 cm '
            f'-> {"ACCEPTED" if ok else "not accepted"}')
        for r in out.values():
            r['ok'] = ok and r['plane_mm'] <= max_plane * 1e3
            r['cross_mm'] = med * 1e3
    return out


def cross_agreement(rec, g, out):
    from twin_recon_core import RobotFilter
    rf = RobotFilter(g)
    W = {}
    for cam, arm in CAMS.items():
        X = out[cam]['X'] @ OPT
        pts = []
        for f in rec['frames']:
            if cam not in f:
                continue
            P, _ = depth_points(f[cam]['depth'], f[cam]['K'], stride=2)
            F = g.link_frames(arm, np.asarray(f['q'][arm], float))[8]
            pw = P @ (F @ X)[:3, :3].T + (F @ X)[:3, 3]
            m = rf.mask(pw, rf.capsules({a: np.asarray(f['q'][a], float) for a in ('left', 'right')}))
            pts.append(pw[~m])
        pw = np.concatenate(pts)
        W[cam] = pw[(pw[:, 2] > 0.015) & (pw[:, 2] < 0.3)][::4]
    if min(len(w) for w in W.values()) < 200:
        return 1.0, 0.0
    d, _ = cKDTree(W['d405_2']).query(W['d405_1'], distance_upper_bound=0.05)
    n = np.isfinite(d)
    return (float(np.median(d[n])) if n.any() else 1.0), (float((d[n] < 0.01).mean()) if n.any() else 0.0)


def write_mounts(cfg_path, out, log=print):
    from twin_calibrate import read_block, set_line
    text = open(cfg_path).read()
    n = 0
    for cam, r in out.items():
        if not r['ok']:
            continue
        Xb = r['X']
        s, e = read_block(text, cam)
        blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in Xb[:3, 3]) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in matrix_rpy(Xb[:3, :3])) + ']')
        blk = set_line(blk, 'calibrated', 'true')
        text = text[:s] + blk + text[e:]
        n += 1
        log(f'{cam}: mount written (calibrated against the ZED map)')
    if n:
        open(cfg_path + '.tmp', 'w').write(text)
        os.replace(cfg_path + '.tmp', cfg_path)
    return n


def reference_from_map(path):
    """Reference points from a saved twin_recon map (scans/recon_map.npz: TSDF) or a mesh npz (points/faces)."""
    z = np.load(path)
    if 'tsdf' in z.files:
        from twin_recon_core import TSDF, surface_nets
        v, f, _c = surface_nets(TSDF.load(path))
        tri = v[f]
        return np.concatenate([v, tri.mean(1)])
    return reference(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('recording')
    ap.add_argument('--mesh', default=os.path.join(DEMO, 'isaac', 'cache', 'mesh_static.npz'),
                    help='ZED reconstruction (vertices/faces, twin/world)')
    ap.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    ap.add_argument('--min-score', type=float, default=0.5)
    ap.add_argument('--max-disagree', type=float, default=0.015, help='m between the two wrists\' mount positions')
    ap.add_argument('--write', action='store_true')
    ap.add_argument('--method', choices=('plane', 'map'), default='plane',
                    help='plane = table plane z=0 + multi-view consistency (default); map = ICP to the ZED map')
    a = ap.parse_args()
    import yaml
    cfg = yaml.safe_load(open(a.config))
    if a.method == 'plane':
        rec = load_recording(a.recording)
        ref = reference_from_map(a.mesh)
        print(f'reference: {len(ref)} points from {a.mesh}; {len(rec["frames"])} views')
        out = refine_mounts(rec, cfg, ref)
        if a.write:
            write_mounts(a.config, out)
        return 0 if out and all(r['ok'] for r in out.values()) else 1
    g = DuoGeometry()
    rec = load_recording(a.recording)
    ref = reference(a.mesh)
    print(f'reference: {len(ref)} points from {a.mesh}; {len(rec["frames"])} views')
    out = {}
    for cam, arm in CAMS.items():
        frames = frames_for(rec, cam, arm, g)
        c = cfg['cameras'][cam]
        starts = [('config/scan', mat(c['xyz'], c['rpy']) @ OPT)]
        for roll in np.radians([0, 90, 180, 270]):           # camera along the flange approach axis, any wrist roll
            starts.append((f'nominal roll {np.degrees(roll):.0f}',
                           mat([0.06, 0.0, 0.05], [0.0, -1.5708, 3.14159]) @ mat([0, 0, 0], [roll, 0, 0]) @ OPT))
        if a.method == 'plane':
            res = solve_plane(plane_frames(rec, cam, arm, g), starts[:2], tree_obj=cKDTree(ref[ref[:, 2] > 0.015]))
            cost, name, X, med, below = res[0]
            s = 1.0 - cost                                   # report as a score (1 = perfect)
            # NOTE: points below z=0 are legitimate here (slotted aluminium-profile table top: slot depth 6-10 cm)
        else:
            res = solve(frames, ref, starts)
            s, name, X, med, below = res[0]
        Xb = X @ np.linalg.inv(OPT)                          # mount of the camera BODY frame (= config convention)
        print(f'{cam} ({arm}): best from "{name}": score {s:.2f}, median {med * 1e3:.1f} mm, below table '
              f'{below * 100:.1f} %, xyz {np.round(Xb[:3, 3], 4).tolist()} rpy {np.round(matrix_rpy(Xb[:3, :3]), 4).tolist()}')
        for r in res[1:]:
            print(f'    other start "{r[1]}": {"cost" if a.method == "plane" else "score"} {r[0]:.3f}')
        out[cam] = (s, Xb)
    (s1, X1), (s2, X2) = out['d405_1'], out['d405_2']
    dis = float(np.linalg.norm(X1[:3, 3] - X2[:3, 3]))
    ok = s1 >= a.min_score and s2 >= a.min_score and dis <= a.max_disagree
    print(f'wrists agree within {dis * 1e3:.1f} mm; {"ACCEPTED" if ok else "NOT accepted"}')
    if ok and a.write:
        from twin_calibrate import read_block, set_line
        text = open(a.config).read()
        for cam, (_, Xb) in out.items():
            s, e = read_block(text, cam)
            blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in Xb[:3, 3]) + ']')
            blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in matrix_rpy(Xb[:3, :3])) + ']')
            blk = set_line(blk, 'calibrated', 'true')
            text = text[:s] + blk + text[e:]
        open(a.config + '.tmp', 'w').write(text)
        os.replace(a.config + '.tmp', a.config)
        print(f'WRITTEN to {a.config}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
