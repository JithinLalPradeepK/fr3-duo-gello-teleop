"""Multi-view photo texturing for the photoreal twin (numpy only; used by isaac/twin_isaac.py, tested offline).

Every face of the reconstruction gets the photo that shows it best: among all views (live ZED photo, the ZED photos of
the last 3D SCAN - same camera, arms elsewhere, so areas hidden under the grippers at home are visible -, and the wrist
D405 photos once their mounts are calibrated) choose the one where the face is VISIBLE and seen with the most pixels:
score = (focal / distance) * cos(view angle). Visibility:
  * a depth image of that view (live ZED, D405): the face must be at the measured depth (within tol), else
  * the ROBOTS of that view (arm capsules at its joint positions, splatted into a depth image): a face behind them is
    hidden (the scan's ZED photos: only the arms move, the rest of the scene is static).
Returns per face the chosen view (or -1) and per face-vertex UVs for a faceVarying 'st' primvar.
"""
import numpy as np


def project(T, K, P):
    """World points -> (u, v, z) in a camera with pose T (optical -> world) and K (fx, fy, cx, cy)."""
    pc = (P - T[:3, 3]) @ T[:3, :3]
    z = pc[:, 2]
    zs = np.where(z > 1e-3, z, 1e-3)
    return K[0] * pc[:, 0] / zs + K[2], K[1] * pc[:, 1] / zs + K[3], z


def robot_depth(T, K, wh, caps, step=0.01, dilate=3):
    """Depth image of the robot capsules (nearest surface) for occlusion tests; inf where no robot."""
    A, B, R = caps
    W, H = wh
    dep = np.full((H, W), np.inf, np.float32)
    pts = []
    for a, b, r in zip(A, B, R):
        L = np.linalg.norm(b - a)
        n = int(L / step) + 2
        ax = (b - a) / L if L > 1e-6 else np.array([0.0, 0.0, 1.0])      # sphere-like capsule (p0 == p1)
        u = np.cross(ax, [0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.cross(ax, [1.0, 0, 0])
        u /= np.linalg.norm(u)
        w = np.cross(ax, u)
        ang = np.linspace(0, 2 * np.pi, max(8, int(2 * np.pi * r / step)), endpoint=False)
        ring = r * (np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * w)
        for s in np.linspace(0, 1, n):
            pts.append(a + s * (b - a) + ring)
        for cen, seed in ((a, 0), (b, 1)):                                         # end caps: points ON the sphere
            sph = np.random.default_rng(seed).normal(size=(60, 3))
            pts.append(cen + r * sph / np.linalg.norm(sph, axis=1, keepdims=True))
    if not pts:
        return dep
    P = np.concatenate(pts)
    u, v, z = project(T, K, P)
    ok = (z > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
    ui, vi, z = u[ok].astype(int), v[ok].astype(int), z[ok].astype(np.float32)
    np.minimum.at(dep, (vi, ui), z)
    if dilate:                                  # close the gaps between splatted points: MIN filter of the depth
        import cv2
        d2 = cv2.erode(np.where(np.isfinite(dep), dep, 1e9).astype(np.float32),
                       np.ones((2 * dilate + 1, 2 * dilate + 1), np.uint8))
        dep = np.where(d2 < 1e8, d2, np.inf).astype(np.float32)
    return dep


def choose(verts, faces, views, depth_tol=0.03, min_cos=0.25):
    """views: list of dict(T, K, wh, depth=None | (array, wh_of_depth), robot=None | depth image). Returns
    (best view per face (-1 = none), uv per face-vertex (m, 3, 2) for that view)."""
    nf = len(faces)
    ctr = verts[faces].mean(1)
    nrm = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    best, score = np.full(nf, -1), np.zeros(nf)
    uv = np.zeros((nf, 3, 2), np.float32)
    for i, vw in enumerate(views):
        T, K, (W, H) = vw['T'], vw['K'], vw['wh']
        u, v, z = project(T, K, ctr)
        inside = (z > 0.05) & (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
        ray = ctr - T[:3, 3]
        ray /= np.maximum(np.linalg.norm(ray, axis=1, keepdims=True), 1e-9)
        cos = np.abs((ray * nrm).sum(1))                # faces are double sided: either side may face the camera
        ok = inside & (cos > min_cos)
        ui, vi = np.clip(u.astype(int), 0, W - 1), np.clip(v.astype(int), 0, H - 1)
        if vw.get('depth') is not None:
            d, (dw, dh) = vw['depth']
            du = np.clip((u * dw / W * d.shape[1] / dw).astype(int), 0, d.shape[1] - 1)
            dv = np.clip((v * dh / H * d.shape[0] / dh).astype(int), 0, d.shape[0] - 1)
            dd = d[dv, du]
            ok &= np.isfinite(dd) & (np.abs(dd - z) < depth_tol)
        if vw.get('robot') is not None:
            ok &= ~(vw['robot'][vi, ui] < z - 0.01)       # a robot in front of the face hides it
        sc = np.where(ok, K[0] / np.maximum(z, 1e-3) * cos, 0.0)
        better = sc > score
        if better.any():
            best[better], score[better] = i, sc[better]
            fu, fv, _ = project(T, K, verts[faces[better]].reshape(-1, 3))
            uv[better] = np.stack([fu / W, 1.0 - fv / H], 1).reshape(-1, 3, 2)
    return best, uv
