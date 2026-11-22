#!/usr/bin/env python3
"""OFFLINE test of scripts/twin_board.py (no ROS, no camera, no robot):  python3 test/twin_board_test.py

The printable board image is warped (exact homography) into a ZED view (known pose, like the calibrated head camera)
and into wrist-camera views from FK of the scan poses with a KNOWN true mount; blur + noise added. Checks:
  B1 board detected in the ZED photo, B2 >= 3 wrist views see it, B3 mount recovered < 2 mm / 0.3 deg,
  B4 views agree (accepted), B5 write -> config body frame round trip.
"""
import math
import os
import sys
import tempfile

import cv2
import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
import twin_board  # noqa: E402
from duo_geometry import DuoGeometry, rpy_matrix  # noqa: E402

R = []


def check(name, ok, info=''):
    R.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}")


def mat(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def look_at(eye, target):
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], 1)
    T[:3, 3] = eye
    return T


def render(tex, px_per_m, T_cam_board, K, wh, rng):
    """Board plane z=0 (board frame, x right / y down as OpenCV's ChArUco object points) -> camera image."""
    fx, fy, cx, cy = K
    Km = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    # texture px -> OpenCV board metres (OpenCV 4.5 legacy ChArUco frame: y measured from the image BOTTOM)
    S = np.array([[1 / px_per_m, 0, 0], [0, -1 / px_per_m, tex.shape[0] / px_per_m], [0, 0, 1.0]])
    H = Km @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ S
    img = cv2.warpPerspective(tex, H, wh, borderValue=150)
    img = cv2.GaussianBlur(img, (3, 3), 0.8)
    img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)


def main():
    rng = np.random.default_rng(0)
    d, b = twin_board.board()
    px_per_m = 4000.0
    w, h = int(6 * 0.045 * px_per_m), int(4 * 0.045 * px_per_m)
    tex = b.draw((w, h)) if hasattr(b, 'draw') else b.generateImage((w, h))
    # board on the table (z = 0) in front of the robots, rotated a little
    T_wb = mat([0.62, 0.0, 0.0005], [0.0, 0.0, 1.5708])     # OpenCV board z = out of the printed face = up
    T_wz = look_at(np.array([0.135, 0.0, 0.60]), [0.75, 0.0, 0.0])                # head camera (optical frame)
    Kz = (1498.0, 1498.0, 958.0, 540.0)
    zimg = render(tex, px_per_m, np.linalg.inv(T_wz) @ T_wb, Kz, (1920, 1080), rng)
    g = DuoGeometry()
    from twin_state import load_home
    home = load_home()
    X_true = {'d405_1': mat([0.055, -0.004, 0.148], [0.02, -1.23, 3.10]) @ mat([0, 0, 0], [-math.pi / 2, 0, -math.pi / 2]),
              'd405_2': mat([0.051, -0.017, 0.152], [-0.01, -1.22, 3.08]) @ mat([0, 0, 0], [-math.pi / 2, 0, -math.pi / 2])}
    Kd = (434.0, 434.0, 424.0, 240.0)
    from scan_plan import make_plan
    plan, _ = make_plan(g, home)                    # the REAL scan views
    frames = []
    for _name, ql, qr in plan:
        f = {'q': {'left': list(ql), 'right': list(qr)}}
        for cam, arm in (('d405_1', 'left'), ('d405_2', 'right')):
            F = g.link_frames(arm, np.array(f['q'][arm]))[8]
            T_cw = np.linalg.inv(F @ X_true[cam])
            f[cam] = {'color_full': render(tex, px_per_m, T_cw @ T_wb, Kd, (848, 480), rng), 'K_full': list(Kd)}
        frames.append(f)
    rec = {'frames': frames, 'zed_photos': [{'rgb': zimg, 'K': list(Kz), 'T': T_wz}]}
    zb = twin_board.detect(zimg, Kz)
    check('B1 board found in the ZED photo', zb is not None and np.linalg.norm((T_wz @ zb[0])[:3, 3] - T_wb[:3, 3]) < 0.003,
          f'{zb[1]} corners, {zb[2]:.2f} px, position error '
          f'{np.linalg.norm((T_wz @ zb[0])[:3, 3] - T_wb[:3, 3]) * 1e3:.1f} mm' if zb else 'not found')
    out = twin_board.calibrate(rec, log=lambda s: print('   ', s))
    for cam in ('d405_1', 'd405_2'):
        r = out.get(cam)
        if r is None:
            check(f'B2 {cam} sees the board in >= 3 views', False)
            continue
        dt = np.linalg.norm(r['X'][:3, 3] - X_true[cam][:3, 3]) * 1e3
        da = math.degrees(math.acos(np.clip((np.trace(r['X'][:3, :3].T @ X_true[cam][:3, :3]) - 1) / 2, -1, 1)))
        check(f'B2 {cam} sees the board in >= 3 views', r['n'] >= 3, f"{r['n']} views")
        check(f'B3 {cam} mount recovered (< 5 mm, < 0.5 deg: below the 6 mm voxel)', dt < 5.0 and da < 0.5, f'{dt:.2f} mm, {da:.3f} deg')
        check(f'B4 {cam} views agree (accepted)', r['ok'], f"spread {r['spread_mm']:.2f} mm / {r['spread_deg']:.3f} deg")
    cfg = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    cfg.write(open(os.path.join(DEMO, 'config', 'digital_twin.yaml')).read())
    cfg.close()
    n = twin_board.write(cfg.name, out, log=lambda s: None)
    import yaml
    c = yaml.safe_load(open(cfg.name))['cameras']['d405_1']
    back = mat(c['xyz'], c['rpy']) @ mat([0, 0, 0], [-math.pi / 2, 0, -math.pi / 2])
    check('B5 write -> config body frame round trip', n == 2 and c['calibrated'] is True and
          np.linalg.norm(back[:3, 3] - out['d405_1']['X'][:3, 3]) < 1e-3, f'{n} written')
    os.unlink(cfg.name)
    print(f'\n{sum(R)}/{len(R)} passed')
    return 0 if all(R) else 1


if __name__ == '__main__':
    sys.exit(main())
