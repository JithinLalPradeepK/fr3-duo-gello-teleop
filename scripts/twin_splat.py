#!/usr/bin/env python3
"""PHOTOREAL surroundings for the twin: 3D Gaussian Splatting from the 3D SCAN's posed photos (no SLAM, no COLMAP).

  python3 scripts/twin_splat.py export scans/rec_<time> [--out splats/<name>]      (system Python + ROS env)
  ~/splat_env/bin/python scripts/twin_splat.py train splats/<name> [--iters 15000]  (GPU, gsplat)

EXPORT  head-camera photos (20 per scan, ZED pose CALIBRATED) + wrist D405 photos (pose = FR3 FK x CALIBRATED mount),
        each with a ROBOT MASK (both arms + grippers rasterised at that photo's joint positions, generous margin: the
        arms move, the splats model only the static scene), and initial coloured points (the TSDF map + ZED cloud).
        Images are prepared in parallel on all CPU cores.
TRAIN   gsplat rasterization + DefaultStrategy densification, loss = 0.8 L1 + 0.2 (1 - SSIM) on unmasked pixels; per-image
        colour correction (affine RGB) so ZED and D405 exposure differences do not end up in the scene; SH degree 3.
        Output: point_cloud.ply in the standard 3DGS layout (Isaac Sim omni.kit.converter.gsplat reads it), cropped to
        the workspace, low-opacity splats pruned.
"""
import argparse
import json
import math
import os
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOUNDS = (np.array([-0.6, -1.2, -0.25]), np.array([2.2, 1.2, 1.2]))     # kept splats (workspace + near background)


# ------------------------------------------------------------------------------------------------ export
_MESH = {}


def robot_silhouette(g, T, K, wh, q_by_arm, gripper_r=0.075):
    """Pixels showing a robot: FR3 link collision meshes posed by FK (close to the real shape, slightly larger) +
    the gripper as a padded capsule; small gap filling only (no smearing onto the scene)."""
    import cv2
    from twin_autocalib import mesh_path, voxel
    from make_duo_geometry import read_stl
    W, H = wh
    pts = []
    for arm, q in q_by_arm.items():
        fr = g.link_frames(arm, np.asarray(q, float))
        for i in range(8):
            key = f'link{i}'
            if key not in _MESH:
                v = read_stl(mesh_path(f'{arm}_fr3_link{i}'))
                tri = v.reshape(-1, 3, 3)
                _MESH[key] = voxel(np.concatenate([v, tri.mean(1)]), 0.004)
            M = fr[i]
            pts.append(_MESH[key] @ M[:3, :3].T + M[:3, 3])
        F = fr[8]                                          # gripper: capsule along the flange z, 0..0.21 m
        ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        ring = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], 1) * gripper_r
        for zz in np.linspace(0.0, 0.21, 40):
            pts.append((ring + [0, 0, zz]) @ F[:3, :3].T + F[:3, 3])
            pts.append((ring * 0.5 + [0, 0, zz]) @ F[:3, :3].T + F[:3, 3])
    P = np.concatenate(pts)
    pc = (P - T[:3, 3]) @ T[:3, :3]
    z = pc[:, 2]
    ok = z > 0.05
    u = (K[0] * pc[ok, 0] / z[ok] + K[2]).astype(int)
    v = (K[1] * pc[ok, 1] / z[ok] + K[3]).astype(int)
    inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    m = np.zeros((H, W), np.uint8)
    m[v[inb], u[inb]] = 1
    k = max(3, W // 160) | 1
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((k * 3, k * 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((k * 2, k * 2), np.uint8))   # drop isolated specks (own upper
    return cv2.dilate(m, np.ones((k, k), np.uint8)) > 0                            # arm behind a wrist camera)


def export(rec_dir, out, zed_scale=0.5, workers=32):
    import cv2
    import yaml
    sys.path.insert(0, os.path.join(DEMO, 'scripts'))
    sys.path.insert(0, os.path.join(DEMO, 'isaac'))
    from duo_geometry import DuoGeometry, rpy_matrix
    from texturing import robot_depth
    from twin_recon_core import TSDF, RobotFilter, surface_nets
    from twin_scan import CAMS, load_recording
    rec = load_recording(rec_dir)
    cfg = yaml.safe_load(open(os.path.join(DEMO, 'config', 'digital_twin.yaml')))
    g = DuoGeometry()
    rf = RobotFilter(g, pad=0.03, gripper_pad=0.09)
    os.makedirs(os.path.join(out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(out, 'masks'), exist_ok=True)
    L_opt = np.eye(4)
    L_opt[:3, :3] = rpy_matrix(-math.pi / 2, 0, -math.pi / 2)
    jobs = []
    for i, zp in enumerate(rec.get('zed_photos') or []):
        q = zp.get('q') or (rec['frames'][i]['q'] if i < len(rec['frames']) else None)
        jobs.append((f'zed_{i:02d}', zp['rgb'], list(zp['K']), np.asarray(zp['T']), q, zed_scale, 'zed'))
    for cam, arm in CAMS.items():
        c = cfg['cameras'][cam]
        if not c.get('calibrated'):
            print(f'{cam}: mount not calibrated - its photos are not used')
            continue
        X = np.eye(4)
        X[:3, :3] = rpy_matrix(*[float(v) for v in c['rpy']])
        X[:3, 3] = [float(v) for v in c['xyz']]
        X = X @ L_opt
        for j, f in enumerate(rec['frames']):
            fc = f.get(cam)
            if fc and 'color_full' in fc:
                T = g.link_frames(arm, np.asarray(f['q'][arm], float))[8] @ X
                jobs.append((f'{cam}_{j:02d}', fc['color_full'], list(fc['K_full']), T, f['q'], 1.0, cam))

    def one(job):
        name, rgb, K, T, q, s, kind = job
        if s != 1.0:
            rgb = cv2.resize(rgb, (int(rgb.shape[1] * s), int(rgb.shape[0] * s)), interpolation=cv2.INTER_AREA)
            K = [K[0] * s, K[1] * s, (K[2] + 0.5) * s - 0.5, (K[3] + 0.5) * s - 0.5]
        h, w = rgb.shape[:2]
        mask = np.full((h, w), 255, np.uint8)
        if q:
            mask[robot_silhouette(g, T, K, (w, h), q)] = 0
        cv2.imwrite(os.path.join(out, 'images', name + '.jpg'), rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 95])
        cv2.imwrite(os.path.join(out, 'masks', name + '.png'), mask)
        return {'name': name, 'kind': kind, 'w': w, 'h': h, 'K': [float(k) for k in K], 'c2w': np.asarray(T).tolist(),
                'masked': float((mask == 0).mean())}
    with ThreadPoolExecutor(workers) as ex:
        cams = list(ex.map(one, jobs))
    json.dump(cams, open(os.path.join(out, 'cameras.json'), 'w'))
    # initial points: the reconstruction map (coloured surface) + the ZED cloud of the scan (background)
    pts, cols = [], []
    mp = os.path.join(DEMO, (cfg.get('reconstruction') or {}).get('map_file', 'scans/recon_map.npz'))
    if os.path.exists(mp):
        v, _f, c = surface_nets(TSDF.load(mp), 3.0, 0, 0)
        pts.append(v)
        cols.append(c.astype(np.float32) / 255)
    if rec.get('zed') is not None and 'xyz' in rec['zed']:
        pts.append(np.asarray(rec['zed']['xyz'], float))
        cols.append(np.full((len(pts[-1]), 3), 0.5, np.float32))
    P = np.concatenate(pts) if pts else np.random.uniform(BOUNDS[0], BOUNDS[1], (50000, 3))
    C = np.concatenate(cols) if cols else np.full((len(P), 3), 0.5, np.float32)
    np.savez(os.path.join(out, 'points.npz'), xyz=P.astype(np.float32), rgb=C.astype(np.float32))
    print(f'exported {len(cams)} images ({sum(c["kind"] == "zed" for c in cams)} head camera, '
          f'{sum(c["kind"] != "zed" for c in cams)} wrist), {len(P)} initial points -> {out}')


# ------------------------------------------------------------------------------------------------ train (GPU)
def ssim(a, b):
    import torch
    import torch.nn.functional as F
    C1, C2 = 0.01 ** 2, 0.03 ** 2
    k = torch.exp(-(torch.arange(11, device=a.device) - 5.0) ** 2 / (2 * 1.5 ** 2))
    k = (k / k.sum())[:, None] @ (k / k.sum())[None, :]
    w = k.expand(3, 1, 11, 11)
    mu_a, mu_b = F.conv2d(a, w, padding=5, groups=3), F.conv2d(b, w, padding=5, groups=3)
    s_a = F.conv2d(a * a, w, padding=5, groups=3) - mu_a ** 2
    s_b = F.conv2d(b * b, w, padding=5, groups=3) - mu_b ** 2
    s_ab = F.conv2d(a * b, w, padding=5, groups=3) - mu_a * mu_b
    return ((2 * mu_a * mu_b + C1) * (2 * s_ab + C2)) / ((mu_a ** 2 + mu_b ** 2 + C1) * (s_a + s_b + C2))


def save_ply(path, means, scales, quats, opac, sh0, shN):
    from plyfile import PlyData, PlyElement
    n = len(means)
    f_rest = shN.transpose(1, 2).reshape(n, -1)                     # 3DGS layout: channel-major
    attrs = ['x', 'y', 'z', 'nx', 'ny', 'nz'] + [f'f_dc_{i}' for i in range(3)] + \
        [f'f_rest_{i}' for i in range(f_rest.shape[1])] + ['opacity'] + [f'scale_{i}' for i in range(3)] + \
        [f'rot_{i}' for i in range(4)]
    data = np.concatenate([means, np.zeros_like(means), sh0.reshape(n, 3), f_rest, opac[:, None], scales, quats], 1)
    el = np.empty(n, dtype=[(a, 'f4') for a in attrs])
    el[:] = list(map(tuple, data.astype(np.float32)))
    PlyData([PlyElement.describe(el, 'vertex')]).write(path)


def train(ds, iters=15000, sh_degree=3, device='cuda'):
    import cv2
    import torch
    from gsplat import rasterization
    from gsplat.strategy import DefaultStrategy
    torch.manual_seed(0)
    cams = json.load(open(os.path.join(ds, 'cameras.json')))
    imgs, masks, K, V = [], [], [], []
    for c in cams:
        im = cv2.imread(os.path.join(ds, 'images', c['name'] + '.jpg'))[:, :, ::-1].astype(np.float32) / 255
        mk = cv2.imread(os.path.join(ds, 'masks', c['name'] + '.png'), 0).astype(np.float32) / 255
        imgs.append(torch.tensor(im, device=device))
        masks.append(torch.tensor(mk, device=device)[..., None])
        K.append(torch.tensor([[c['K'][0], 0, c['K'][2]], [0, c['K'][1], c['K'][3]], [0, 0, 1.0]], device=device))
        V.append(torch.tensor(np.linalg.inv(np.array(c['c2w'])), dtype=torch.float32, device=device))
    pts = np.load(os.path.join(ds, 'points.npz'))
    P = torch.tensor(pts['xyz'], device=device)
    C = torch.tensor(pts['rgb'], device=device).clamp(0.02, 0.98)
    keep = ((P >= torch.tensor(BOUNDS[0], device=device)) & (P <= torch.tensor(BOUNDS[1], device=device))).all(1)
    P, C = P[keep], C[keep]
    if len(P) > 300000:
        sel = torch.randperm(len(P), device=device)[:300000]
        P, C = P[sel], C[sel]
    n = len(P)
    d = torch.cdist(P[torch.randperm(n, device=device)[:4096]], P).topk(4, largest=False).values[:, 1:].mean()
    scale0 = torch.log(torch.full((n, 3), float(d), device=device).clamp(1e-3, 0.02))
    SH_C0 = 0.28209479177387814
    params = torch.nn.ParameterDict({
        'means': torch.nn.Parameter(P),
        'scales': torch.nn.Parameter(scale0),
        'quats': torch.nn.Parameter(torch.nn.functional.normalize(torch.randn(n, 4, device=device), dim=-1)),
        'opacities': torch.nn.Parameter(torch.logit(torch.full((n,), 0.1, device=device))),
        'sh0': torch.nn.Parameter(((C - 0.5) / SH_C0)[:, None, :]),
        'shN': torch.nn.Parameter(torch.zeros(n, (sh_degree + 1) ** 2 - 1, 3, device=device)),
    })
    extent = 1.5
    lrs = {'means': 1.6e-4 * extent, 'scales': 5e-3, 'quats': 1e-3, 'opacities': 5e-2, 'sh0': 2.5e-3, 'shN': 2.5e-3 / 20}
    opts = {k: torch.optim.Adam([params[k]], lr=lr, eps=1e-15) for k, lr in lrs.items()}
    # per-image colour correction (wrist cameras: different exposure / white balance); ZED images stay identity
    corr = torch.nn.Parameter(torch.zeros(len(cams), 12, device=device))
    corr_opt = torch.optim.Adam([corr], lr=1e-3)
    is_zed = torch.tensor([c['kind'] == 'zed' for c in cams], device=device)
    strategy = DefaultStrategy(refine_stop_iter=int(iters * 0.6), refine_every=100, reset_every=3000,
                               grow_grad2d=0.0002, verbose=False)
    strategy.check_sanity(params, opts)
    state = strategy.initialize_state(scene_scale=extent)
    sched = torch.optim.lr_scheduler.ExponentialLR(opts['means'], gamma=0.01 ** (1.0 / iters))
    for it in range(iters):
        i = int(torch.randint(len(cams), (1,)))
        c = cams[i]
        sh = min(it // 1000, sh_degree)
        colors = torch.cat([params['sh0'], params['shN']], 1)
        out, alpha, info = rasterization(params['means'], torch.nn.functional.normalize(params['quats'], dim=-1),
                                         torch.exp(params['scales']), torch.sigmoid(params['opacities']), colors,
                                         V[i][None], K[i][None], c['w'], c['h'], sh_degree=sh, packed=False,
                                         absgrad=False, rasterize_mode='antialiased')
        img = out[0].clamp(0, 1)
        if not bool(is_zed[i]):
            A = torch.eye(3, device=device) + corr[i, :9].view(3, 3)
            img = (img @ A.T + corr[i, 9:]).clamp(0, 1)
        gt, m = imgs[i], masks[i]
        strategy.step_pre_backward(params, opts, state, it, info)
        l1 = (torch.abs(img - gt) * m).sum() / (m.sum() * 3 + 1)
        s = ssim(img.permute(2, 0, 1)[None], gt.permute(2, 0, 1)[None])
        ls = 1 - (s * m.permute(2, 0, 1)[None]).sum() / (m.sum() * 3 + 1)
        loss = 0.8 * l1 + 0.2 * ls + 0.01 * torch.sigmoid(params['opacities']).mean()
        loss.backward()
        for o in opts.values():
            o.step()
            o.zero_grad(set_to_none=True)
        corr_opt.step()
        corr_opt.zero_grad(set_to_none=True)
        sched.step()
        strategy.step_post_backward(params, opts, state, it, info, packed=False)
        if it % 1000 == 0 or it == iters - 1:
            with torch.no_grad():
                psnr = -10 * torch.log10(((img - gt) ** 2 * m).sum() / (m.sum() * 3 + 1) + 1e-10)
            print(f'iter {it:5d}/{iters}  loss {loss.item():.4f}  PSNR {psnr.item():.2f} dB  splats {len(params["means"])}',
                  flush=True)
    with torch.no_grad():
        o = torch.sigmoid(params['opacities'])
        keep = (o > 0.01) & ((params['means'] >= torch.tensor(BOUNDS[0], device=device)) &
                             (params['means'] <= torch.tensor(BOUNDS[1], device=device))).all(1)
        q = torch.nn.functional.normalize(params['quats'][keep], dim=-1)
        path = os.path.join(ds, 'point_cloud.ply')
        save_ply(path, params['means'][keep].cpu().numpy(), params['scales'][keep].cpu().numpy(), q.cpu().numpy(),
                 params['opacities'][keep].cpu().numpy(), params['sh0'][keep].cpu().numpy(),
                 params['shN'][keep].cpu().numpy())
        # quick evaluation renders (head-camera view 0 + a wrist view) for checking
        for j in (0, len(cams) - 1):
            colors = torch.cat([params['sh0'], params['shN']], 1)[keep]
            out, _a, _i = rasterization(params['means'][keep], q, torch.exp(params['scales'][keep]), o[keep], colors,
                                        V[j][None], K[j][None], cams[j]['w'], cams[j]['h'], sh_degree=sh_degree)
            cv2.imwrite(os.path.join(ds, f'render_{cams[j]["name"]}.jpg'),
                        (out[0].clamp(0, 1).cpu().numpy()[:, :, ::-1] * 255).astype(np.uint8))
    print(f'saved {path}: {int(keep.sum())} splats')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('export')
    e.add_argument('recording')
    e.add_argument('--out', default=None)
    t = sub.add_parser('train')
    t.add_argument('dataset')
    t.add_argument('--iters', type=int, default=15000)
    a = ap.parse_args()
    if a.cmd == 'export':
        out = a.out or os.path.join(DEMO, 'splats', os.path.basename(os.path.normpath(a.recording)))
        export(a.recording, out)
    else:
        train(a.dataset, a.iters)


if __name__ == '__main__':
    main()
