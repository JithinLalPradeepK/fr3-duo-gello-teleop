#!/usr/bin/env python3
"""Solid, coloured 3D reconstruction for the DIGITAL TWIN - numpy/scipy only (no new dependencies, no GPU, no SLAM).

Used by scripts/twin_recon.py (ROS node) and test/twin_recon_test.py (offline, synthetic). Nothing here talks to a robot.

  TSDF        dense truncated signed distance volume over the workspace (config: reconstruction.bounds / voxel_m).
              Projective integration of depth images with KNOWN camera poses (twin TF + FR3 forward kinematics).
              Weighted running average, weight cap = how fast the static map forgets (moving objects are carved away by
              later free-space observations; something that stays put becomes static after ~max_weight frames).
              Pixels that see a robot are "free space only": they carve the space in front of the robot but never add
              a surface (no robot ghosts in the map).
  surface_nets  smooth triangle mesh of the zero level (one vertex per surface cell = mean of its edge crossings,
              one quad per crossing edge), per-vertex colour, optional Taubin smoothing, small floaters removed.
  live_points  observed points that lie in space the static map KNOWS to be empty = moving objects (hands, cubes).
  RobotFilter  duo_geometry capsules of both arms (+ gripper) at the frame's joint positions.
  marker_bytes  visualization_msgs/Marker with a big points/colors payload serialised from numpy in ~ms (rclpy object
              construction is ~1 s per 100k triangles); publish(bytes) is supported by rclpy Humble.
"""
import struct

import numpy as np

# ------------------------------------------------------------------------------------------------ TSDF volume


class TSDF:
    def __init__(self, lo, hi, voxel=0.01, trunc_vox=4.0, max_weight=40.0):
        self.voxel = float(voxel)
        self.lo = np.asarray(lo, np.float64)
        self.shape = tuple(int(n) for n in np.ceil((np.asarray(hi, float) - self.lo) / self.voxel))
        self.hi = self.lo + np.array(self.shape) * self.voxel
        self.trunc = float(trunc_vox) * self.voxel
        self.max_weight = float(max_weight)
        self.reset()

    def reset(self):
        self.tsdf = np.ones(self.shape, np.float32)
        self.w = np.zeros(self.shape, np.float32)
        self.rgb = np.full(self.shape + (3,), 180, np.uint8)
        self._cache = {}
        self.frames = 0

    @property
    def nbytes(self):
        return self.tsdf.nbytes + self.w.nbytes + self.rgb.nbytes

    # -- integration
    def _frustum_box(self, K, T, wh, near, far):
        fx, fy, cx, cy = K
        w, h = wh
        cs = []
        for z in (near, far):
            for u, v in ((0, 0), (w, 0), (0, h), (w, h)):
                cs.append(((u - cx) * z / fx, (v - cy) * z / fy, z))
        cw = np.array(cs) @ T[:3, :3].T + T[:3, 3]
        a = np.floor((cw.min(0) - self.lo) / self.voxel).astype(int)
        b = np.ceil((cw.max(0) - self.lo) / self.voxel).astype(int) + 1
        a, b = np.clip(a, 0, self.shape), np.clip(b, 0, self.shape)
        return a, b

    def _project(self, K, T, wh, near, far):
        """Voxels of the camera frustum -> (flat voxel index, flat pixel index, voxel depth z). Cached per camera pose
        (a fixed camera - the ZED - costs one gather per frame)."""
        key = (tuple(np.round(T[:3], 4).ravel()), tuple(np.round(K, 3)), wh, near, far)   # 0.1 mm / 0.1 mrad
        hit = self._cache.get(key)
        if hit is not None:
            return hit
        a, b = self._frustum_box(K, T, wh, near, far + self.trunc)
        if np.any(b <= a):
            out = (np.zeros(0, np.int64),) * 2 + (np.zeros(0, np.float32),)
        else:
            gi = [np.arange(a[d], b[d]) for d in range(3)]
            ix, iy, iz = np.meshgrid(*gi, indexing='ij')
            ix, iy, iz = ix.ravel(), iy.ravel(), iz.ravel()
            pw = (np.stack([ix, iy, iz], 1) + 0.5) * self.voxel + self.lo
            Rt = T[:3, :3].T
            pc = (pw - T[:3, 3]) @ Rt.T
            z = pc[:, 2]
            ok = (z > near) & (z < far + self.trunc)
            fx, fy, cx, cy = K
            u = np.floor(fx * pc[:, 0] / np.where(ok, z, 1) + cx + 0.5).astype(np.int64)
            v = np.floor(fy * pc[:, 1] / np.where(ok, z, 1) + cy + 0.5).astype(np.int64)
            ok &= (u >= 0) & (u < wh[0]) & (v >= 0) & (v < wh[1])
            flat = np.ravel_multi_index((ix[ok], iy[ok], iz[ok]), self.shape)
            out = (flat, v[ok] * wh[0] + u[ok], z[ok].astype(np.float32))
        if len(self._cache) >= 6:           # moving wrist cameras: keep only a few poses (oldest out)
            self._cache.pop(next(iter(self._cache)))
        self._cache[key] = out
        return out

    def integrate(self, depth, K, T, rgb=None, free_only=None, near=0.1, far=2.5, weight=1.0, cache=True):
        """depth (h, w) metres (0/nan = no data), K (fx, fy, cx, cy), T 4x4 camera optical frame -> world,
        rgb (h, w, 3) uint8 or None, free_only (h, w) bool = pixels on a robot (carve only, no surface)."""
        h, w = depth.shape
        flat, pix, z = self._project(tuple(K), T, (w, h), near, far)     # cached per pose (fixed / resting camera)
        d = depth.ravel()[pix]
        ok = np.isfinite(d) & (d > near) & (d < far)
        sdf = d - z
        ok &= sdf > -self.trunc
        if free_only is not None:
            ok &= ~(free_only.ravel()[pix] & (sdf < self.trunc))     # robot pixels: only voxels clearly in front
        idx, sdf = flat[ok], sdf[ok]
        obs = np.minimum(sdf / self.trunc, 1.0).astype(np.float32)
        T_, W_ = self.tsdf.reshape(-1), self.w.reshape(-1)
        wo = W_[idx]
        wn = wo + weight
        T_[idx] = (T_[idx] * wo + obs * weight) / wn
        W_[idx] = np.minimum(wn, self.max_weight)
        if rgb is not None:
            near_s = np.abs(obs) < 0.5                                 # colour only from voxels near the surface
            ci, cp = idx[near_s], pix[ok][near_s]
            C_ = self.rgb.reshape(-1, 3)
            a = (weight / np.minimum(wn[near_s], self.max_weight))[:, None]
            C_[ci] = np.clip(C_[ci] * (1 - a) + rgb.reshape(-1, 3)[cp] * a + 0.5, 0, 255).astype(np.uint8)
        self.frames += 1
        return int(len(idx))

    def clear_capsules(self, seg_a, seg_b, radius):
        """Forget (weight 0) every voxel inside the given capsules (the space a robot occupies now)."""
        W_ = self.w.reshape(-1)
        n = 0
        for a, b, r in zip(seg_a, seg_b, radius):
            lo_i = np.clip(np.floor((np.minimum(a, b) - r - self.lo) / self.voxel).astype(int), 0, self.shape)
            hi_i = np.clip(np.ceil((np.maximum(a, b) + r - self.lo) / self.voxel).astype(int) + 1, 0, self.shape)
            if np.any(hi_i <= lo_i):
                continue
            ix, iy, iz = np.meshgrid(*[np.arange(lo_i[d], hi_i[d]) for d in range(3)], indexing='ij')
            p = (np.stack([ix.ravel(), iy.ravel(), iz.ravel()], 1) + 0.5) * self.voxel + self.lo
            inside = point_segment_dist(p, a[None], b[None])[:, 0] < r
            if inside.any():
                W_[np.ravel_multi_index((ix.ravel()[inside], iy.ravel()[inside], iz.ravel()[inside]), self.shape)] = 0
                n += int(inside.sum())
        return n

    def lookup(self, pw):
        """World points -> (inside grid, tsdf, weight) of their voxels."""
        i = np.floor((pw - self.lo) / self.voxel).astype(np.int64)
        ins = np.all((i >= 0) & (i < self.shape), 1)
        t = np.ones(len(pw), np.float32)
        w = np.zeros(len(pw), np.float32)
        if ins.any():
            f = np.ravel_multi_index(i[ins].T, self.shape)
            t[ins], w[ins] = self.tsdf.reshape(-1)[f], self.w.reshape(-1)[f]
        return ins, t, w

    # -- persistence
    def save(self, path, **meta):
        np.savez_compressed(path, tsdf=self.tsdf.astype(np.float16), w=self.w.astype(np.float16), rgb=self.rgb,
                            lo=self.lo, voxel=self.voxel, trunc=self.trunc, max_weight=self.max_weight,
                            frames=self.frames, **meta)

    @classmethod
    def load(cls, path):
        z = np.load(path)
        v = cls.__new__(cls)
        v.voxel, v.lo = float(z['voxel']), np.asarray(z['lo'], float)
        v.tsdf, v.w, v.rgb = z['tsdf'].astype(np.float32), z['w'].astype(np.float32), z['rgb'].copy()
        v.shape = v.tsdf.shape
        v.hi = v.lo + np.array(v.shape) * v.voxel
        v.trunc, v.max_weight, v.frames = float(z['trunc']), float(z['max_weight']), int(z['frames'])
        v._cache = {}
        return v


# ------------------------------------------------------------------------------------------------ meshing

_CORNERS = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]])
_EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if np.abs(_CORNERS[a] - _CORNERS[b]).sum() == 1]


def surface_nets(vol, min_weight=2.0, smooth=2, min_component=60, max_jump=1.2):
    """TSDF zero level -> (vertices (n,3) world, triangles (m,3) int, colours (n,3) uint8). Triangles face free space."""
    T, W = vol.tsdf, vol.w
    known = W >= min_weight
    if not known.any():
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros((0, 3), np.uint8)
    # crop to the observed box (+1)
    nz = np.argwhere(known)
    a, b = np.maximum(nz.min(0) - 1, 0), np.minimum(nz.max(0) + 2, vol.shape)
    sl = tuple(slice(a[d], b[d]) for d in range(3))
    T, W, known, C = T[sl], W[sl], known[sl], vol.rgb[sl]
    pos = T > 0
    sh = np.array(T.shape) - 1
    if np.any(sh < 1):
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros((0, 3), np.uint8)

    def corner(arr, c):
        return arr[c[0]:c[0] + sh[0], c[1]:c[1] + sh[1], c[2]:c[2] + sh[2]]

    allk = np.ones(tuple(sh), bool)
    anyp = np.zeros(tuple(sh), bool)
    anyn = np.zeros(tuple(sh), bool)
    for c in _CORNERS:
        allk &= corner(known, c)
        p = corner(pos, c)
        anyp |= p
        anyn |= ~p
    active = allk & anyp & anyn
    cells = np.argwhere(active)
    if len(cells) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros((0, 3), np.uint8)
    cid = np.full(tuple(sh), -1, np.int64)
    cid[tuple(cells.T)] = np.arange(len(cells))
    # vertex = mean of the valid edge crossings in the cell
    cv = np.stack([T[tuple((cells + c).T)] for c in _CORNERS], 1)          # (n, 8)
    acc = np.zeros((len(cells), 3))
    cnt = np.zeros(len(cells))
    for ea, eb in _EDGES:
        ta, tb = cv[:, ea], cv[:, eb]
        cross = ((ta > 0) != (tb > 0)) & (np.abs(ta - tb) < max_jump)
        t = np.where(cross, ta / np.where(cross, ta - tb, 1), 0)
        acc += cross[:, None] * (_CORNERS[ea] + t[:, None] * (_CORNERS[eb] - _CORNERS[ea]))
        cnt += cross
    good = cnt > 0
    verts = cells + np.where(good[:, None], acc / np.maximum(cnt, 1)[:, None], 0.5)
    verts = (verts + a + 0.5) * vol.voxel + vol.lo
    # colour: corners near the surface, weighted
    cw = np.stack([((1 - np.abs(T[tuple((cells + c).T)])) * W[tuple((cells + c).T)]) for c in _CORNERS], 1)
    cw = np.maximum(cw, 1e-6)
    cc = np.stack([C[tuple((cells + c).T)] for c in _CORNERS], 1).astype(np.float32)   # (n, 8, 3)
    cols = (cc * cw[:, :, None]).sum(1) / cw.sum(1)[:, None]
    # faces: one quad per grid edge with a valid crossing; its 4 neighbour cells
    tris = []
    for ax in range(3):
        o1, o2 = [d for d in range(3) if d != ax]
        lo_ = [1, 1, 1]
        lo_[ax] = 0
        hi_ = list(T.shape)
        hi_[ax] -= 1
        hi_[o1] -= 1
        hi_[o2] -= 1
        e0 = tuple(slice(lo_[d], hi_[d]) for d in range(3))
        step = [0, 0, 0]
        step[ax] = 1
        e1 = tuple(slice(lo_[d] + step[d], hi_[d] + step[d]) for d in range(3))
        ta, tb = T[e0], T[e1]
        cross = ((ta > 0) != (tb > 0)) & known[e0] & known[e1] & (np.abs(ta - tb) < max_jump)
        idx = np.argwhere(cross) + np.array(lo_)
        if len(idx) == 0:
            continue
        flip = (T[tuple(idx.T)] > 0)
        q = []
        for d1, d2 in ((0, 0), (1, 0), (1, 1), (0, 1)):
            c = idx.copy()
            c[:, o1] -= 1 - d1
            c[:, o2] -= 1 - d2
            q.append(cid[tuple(c.T)])
        q = np.stack(q, 1)
        valid = np.all(q >= 0, 1)
        q, flip = q[valid], flip[valid]
        # orientation: with (o1, o2, ax) right-handed the quad (00,10,11,01) faces +ax; face toward positive tsdf
        rh = (o1, o2, ax) in ((0, 1, 2), (1, 2, 0), (2, 0, 1))
        rev = flip if rh else ~flip
        q[rev] = q[rev][:, ::-1]
        tris.append(q[:, [0, 1, 2]])
        tris.append(q[:, [0, 2, 3]])
    if not tris:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros((0, 3), np.uint8)
    tris = np.concatenate(tris)
    verts, tris, cols = _compact(verts, tris, cols)
    if min_component and len(tris):
        verts, tris, cols = drop_small_components(verts, tris, cols, min_component)
    if smooth and len(tris):
        verts = taubin(verts, tris, smooth)
    return verts, tris, np.clip(cols + 0.5, 0, 255).astype(np.uint8)


def _compact(v, t, c):
    used = np.unique(t)
    remap = np.full(len(v), -1, np.int64)
    remap[used] = np.arange(len(used))
    return v[used], remap[t], c[used]


def _adjacency(n, tris):
    from scipy.sparse import coo_matrix
    i = np.concatenate([tris[:, 0], tris[:, 1], tris[:, 2], tris[:, 1], tris[:, 2], tris[:, 0]])
    j = np.concatenate([tris[:, 1], tris[:, 2], tris[:, 0], tris[:, 0], tris[:, 1], tris[:, 2]])
    a = coo_matrix((np.ones(len(i)), (i, j)), shape=(n, n)).tocsr()
    a.data[:] = 1.0
    return a


def drop_small_components(v, t, c, min_tris):
    from scipy.sparse.csgraph import connected_components
    _, lab = connected_components(_adjacency(len(v), t), directed=False)
    size = np.bincount(lab[t[:, 0]], minlength=lab.max() + 1)
    keep = size[lab[t[:, 0]]] >= min_tris
    return _compact(v, t[keep], c)


def taubin(v, t, iters=2, lam=0.5, mu=-0.53):
    """Volume-preserving smoothing (no shrinking): alternating Laplacian steps lam / mu."""
    a = _adjacency(len(v), t)
    deg = np.maximum(np.asarray(a.sum(1)).ravel(), 1)[:, None]
    v = v.copy()
    for _ in range(iters):
        for f in (lam, mu):
            v += f * (a @ v / deg - v)
    return v


def face_normals(v, t):
    n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def vertex_normals(v, t):
    n = np.zeros_like(v)
    fn = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])          # area weighted
    for k in range(3):
        np.add.at(n, t[:, k], fn)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def shade(v, t, c, light=(0.4, 0.3, 1.0), ambient=0.55, diffuse=0.45, sky=0.10):
    """Bake soft lighting into vertex colours (RViz draws TRIANGLE_LIST markers unlit): ambient + Lambert from one
    direction + a little sky light from above. Normals face free space (surface_nets orientation)."""
    if not len(t):
        return c
    n = vertex_normals(v, t)
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    f = ambient + diffuse * np.clip(n @ L, 0, 1) + sky * n[:, 2]
    return np.clip(c.astype(np.float32) * f[:, None], 0, 255).astype(np.uint8)


def write_ply(path, v, t, c):
    """Binary PLY with per-vertex colour (opens in MeshLab / Blender / CloudCompare)."""
    with open(path, 'wb') as f:
        f.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {len(v)}\nproperty float x\nproperty float y\n'
                 f'property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n'
                 f'element face {len(t)}\nproperty list uchar int vertex_indices\nend_header\n').encode())
        vb = np.zeros(len(v), dtype=[('p', '<f4', 3), ('c', 'u1', 3)])
        vb['p'], vb['c'] = v, c
        f.write(vb.tobytes())
        fb = np.zeros(len(t), dtype=[('n', 'u1'), ('i', '<i4', 3)])
        fb['n'], fb['i'] = 3, t
        f.write(fb.tobytes())


# ------------------------------------------------------------------------------------------------ cameras / robots

def backproject(depth, K, T, stride=1):
    """depth (h, w) m -> world points (h*w, 3) (nan where no data)."""
    h, w = depth.shape
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1).reshape(-1, 3)
    return pc @ T[:3, :3].T + T[:3, 3]


def point_segment_dist(p, a, b):
    """p (n,3), segments a,b (m,3) -> distances (n, m)."""
    ab = b - a
    L = np.maximum((ab * ab).sum(1), 1e-12)
    t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(-1) / L, 0, 1)
    c = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(p[:, None, :] - c, axis=-1)


class RobotFilter:
    """Both FR3s + grippers as duo_geometry capsules (the HOME planner's geometry) at given joint positions."""

    def __init__(self, geometry, pad=0.02, gripper_pad=0.07):
        self.g, self.pad = geometry, float(pad)
        self.r = np.array([c[3] for c in geometry.caps])
        # gripper capsule: the real Robotiq + wrist D405 + its cable stick out of the (estimated) gripper capsule;
        # seen 2026-11-22 as dark ghost points 3-6 cm beside the gripper -> extra filter margin there
        self.extra = np.array([max(float(gripper_pad) - self.pad, 0.0) if c[4] == 'gripper' else 0.0
                               for c in geometry.caps])

    def capsules(self, q_by_arm, filter_margin=True):
        """World capsules of both arms; filter_margin=False = the plain geometric radii (rendering / tests)."""
        A, B, R = [], [], []
        for arm, q in q_by_arm.items():
            if q is None:
                continue
            a, b = self.g.world_capsules(arm, np.asarray(q, float))
            A.append(a)
            B.append(b)
            R.append(self.r + self.extra if filter_margin else self.r)   # filter radius for mask / clear_capsules
        if not A:
            return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0)
        return np.concatenate(A), np.concatenate(B), np.concatenate(R)

    def mask(self, pw, caps, pad=None):
        """pw (n,3) world points (nan allowed) -> True where the point is on / inside a robot (capsule + pad)."""
        A, B, R = caps
        pad = self.pad if pad is None else pad
        out = np.zeros(len(pw), bool)
        if not len(A):
            return out
        fin = np.all(np.isfinite(pw), 1)
        lo = np.minimum(A, B).min(0) - R.max() - pad
        hi = np.maximum(A, B).max(0) + R.max() + pad
        idx = np.nonzero(fin & np.all((pw >= lo) & (pw <= hi), 1))[0]
        P = pw[idx]
        hit = np.zeros(len(idx), bool)
        for a, b, r in zip(A, B, R):              # per capsule: box prefilter, then exact distance on the few left
            m = r + pad
            sel = np.nonzero(~hit & np.all((P >= np.minimum(a, b) - m) & (P <= np.maximum(a, b) + m), 1))[0]
            if len(sel):
                hit[sel] = point_segment_dist(P[sel], a[None], b[None])[:, 0] < m
        out[idx] = hit
        return out


def live_points(vol, pw, rgb, min_weight=8.0, free_tsdf=0.3, nb_tsdf=0.0):
    """Observed world points (n,3) that fall into voxels the static map knows to be EMPTY (tsdf > free_tsdf with enough
    weight, and no 3x3x3 neighbour behind a surface: tsdf > nb_tsdf) = moving / new objects. Returns (voxel centres
    (k,3), colours (k,3) uint8), one per occupied voxel.
    Hand-over: an object that stays put is absorbed by the map's running average; the neighbour test fails exactly when
    its voxels cross zero, i.e. when the static mesh shows it - so it is always visible in one of the two layers."""
    fin = np.all(np.isfinite(pw), 1)
    pw, rgb = pw[fin], rgb[fin]
    ins, t, w = vol.lookup(pw)
    dyn = ins & (w >= min_weight) & (t > free_tsdf)
    if dyn.any():                 # whole 3x3x3 neighbourhood must be free too (projective TSDF overshoots at edges)
        i = np.floor((pw[dyn] - vol.lo) / vol.voxel).astype(np.int64)
        off = np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1], indexing='ij')).reshape(3, -1).T
        nb = np.clip(i[:, None, :] + off[None], 0, np.array(vol.shape) - 1)
        f = np.ravel_multi_index(nb.reshape(-1, 3).T, vol.shape)
        tn = vol.tsdf.reshape(-1)[f].reshape(len(i), -1)
        wn = vol.w.reshape(-1)[f].reshape(len(i), -1)
        free_nb = np.where(wn > 0, tn, 1.0).min(1) > nb_tsdf
        sel = np.nonzero(dyn)[0]
        dyn[sel[~free_nb]] = False
    if not dyn.any():
        return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
    i = np.floor((pw[dyn] - vol.lo) / vol.voxel).astype(np.int64)
    f = np.ravel_multi_index(i.T, vol.shape)
    u, first, cnt = np.unique(f, return_index=True, return_counts=True)
    keep = cnt >= 2                                           # a voxel needs 2 hits (speckle filter)
    c = (np.array(np.unravel_index(u[keep], vol.shape)).T + 0.5) * vol.voxel + vol.lo
    return c, rgb[dyn][first[keep]]


# ------------------------------------------------------------------------------------------------ fast Marker bytes

_SENT_P = (1.234567e-7, 2.345678e-7, 3.456789e-7)
_SENT_C = (0.123457, 0.234568, 0.345679, 0.456791)


def marker_bytes(marker, points, colors):
    """CDR bytes of `marker` (visualization_msgs/Marker, points/colors left empty) with points (n,3) float and colors
    (n,4) float 0..1 spliced in from numpy. Publish with publisher.publish(bytes)."""
    from geometry_msgs.msg import Point
    from rclpy.serialization import serialize_message
    from std_msgs.msg import ColorRGBA
    marker.points = [Point(x=_SENT_P[0], y=_SENT_P[1], z=_SENT_P[2])]
    marker.colors = [ColorRGBA(r=_SENT_C[0], g=_SENT_C[1], b=_SENT_C[2], a=_SENT_C[3])]
    s = serialize_message(marker)
    marker.points, marker.colors = [], []
    p0 = s.find(struct.pack('<3d', *_SENT_P))
    c0 = s.find(struct.pack('<4f', *_SENT_C))
    lp = s.rfind(struct.pack('<I', 1), 0, p0)
    if p0 < 0 or c0 < 0 or lp < 0 or p0 - lp - 4 >= 8:
        raise RuntimeError('marker template not understood')
    n = len(points)
    return b''.join([s[:lp], struct.pack('<I', n), s[lp + 4:p0], np.ascontiguousarray(points, '<f8').tobytes(),
                     struct.pack('<I', len(colors)), np.ascontiguousarray(colors, '<f4').tobytes(), s[c0 + 16:]])
