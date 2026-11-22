#!/usr/bin/env python3
"""ChArUco board for the WRIST D405 mounts (hand-eye), measured against the verified ZED head camera. No ROS needed.

  python3 scripts/twin_board.py make [--out docs/charuco_board_A4.png]   printable board (A4 landscape, 300 dpi)
  python3 scripts/twin_board.py calib scans/rec_<time> [--write]          mounts from a recorded 3D SCAN

Print at 100 % (no "fit to page"), check one square with a ruler (45.0 mm; else pass --square <mm>), lay it FLAT on the
table, centred about 0.6 m in front of the mount (just beyond the hanging grippers, between them), long side across,
then run the 3D SCAN. The head camera photographs it at every scan view (best view used: arms lifted). Board pose in the world = ZED photo (pose verified) + PnP; each wrist view: X = F^-1 T_world_board T_cam_board^-1.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SQUARES = (6, 4)              # squares x, y
SQUARE_M = 0.045
MARKER_M = 0.034
DICT = cv2.aruco.DICT_5X5_100


def board(square=SQUARE_M):
    d = cv2.aruco.getPredefinedDictionary(DICT)
    if hasattr(cv2.aruco, 'CharucoBoard_create'):          # OpenCV 4.5 (system)
        return d, cv2.aruco.CharucoBoard_create(SQUARES[0], SQUARES[1], square, square * MARKER_M / SQUARE_M, d)
    return d, cv2.aruco.CharucoBoard(SQUARES, square, square * MARKER_M / SQUARE_M, d)


def make(out, dpi=300):
    _d, b = board()
    px_per_m = dpi / 0.0254
    w, h = int(SQUARES[0] * SQUARE_M * px_per_m), int(SQUARES[1] * SQUARE_M * px_per_m)
    img = b.draw((w, h)) if hasattr(b, 'draw') else b.generateImage((w, h))
    a4 = np.full((int(0.210 * px_per_m), int(0.297 * px_per_m)), 255, np.uint8)
    y0, x0 = (a4.shape[0] - h) // 2, (a4.shape[1] - w) // 2
    a4[y0:y0 + h, x0:x0 + w] = img
    cv2.putText(a4, f'FR3 Duo twin - ChArUco {SQUARES[0]}x{SQUARES[1]}, square {SQUARE_M * 1000:.1f} mm, '
                f'DICT_5X5_100 - PRINT AT 100 %', (x0, y0 - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    cv2.imwrite(out, a4)
    try:                                                 # PDF with the exact physical size (A4)
        from PIL import Image
        Image.fromarray(a4).save(os.path.splitext(out)[0] + '.pdf', resolution=dpi)
    except Exception as e:      # noqa: BLE001
        print(f'(no PDF: {e})')
    return out


def detect(rgb, K, square=SQUARE_M, min_corners=6):
    """Board pose in the camera OPTICAL frame: (T cam<-board 4x4, corners used, RMS reprojection px) or None."""
    d, b = board(square)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) if rgb.ndim == 3 else rgb
    fx, fy, cx, cy = K
    Km = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    dist = np.zeros(5)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, d)
    if ids is None or len(ids) < 3:
        return None
    n, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, b, cameraMatrix=Km, distCoeffs=dist)
    if cc is None or n < min_corners:
        return None
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(cc, ci, b, Km, dist, None, None)
    if not ok:
        return None
    obj = (b.chessboardCorners if hasattr(b, 'chessboardCorners') else b.getChessboardCorners())[ci.ravel()]
    proj, _ = cv2.projectPoints(obj, rvec, tvec, Km, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - cc.reshape(-1, 2)) ** 2, 1))))
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, int(n), rms


def rot_mean(Rs):
    """Chordal L2 mean of rotation matrices."""
    M = np.sum(Rs, 0)
    u, _s, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return R


def calibrate(rec, square=SQUARE_M, log=print, max_spread_mm=4.0, max_spread_deg=0.5, min_views=3, max_rms=1.5):
    """{cam: dict(X flange<-optical, n, spread_mm, spread_deg, ok)} from a recording with full-res colour + a ZED photo."""
    sys.path.insert(0, os.path.join(DEMO, 'scripts'))
    from duo_geometry import DuoGeometry
    from twin_scan import CAMS
    photos = rec.get('zed_photos') or ([rec['zed_photo']] if rec.get('zed_photo') else [])
    best = None
    for zp in photos:                                    # the photo that sees the board best (arms lifted)
        r = detect(zp['rgb'], zp['K'], square)
        if r is not None and (best is None or r[1] > best[1][1] or (r[1] == best[1][1] and r[2] < best[1][2])):
            best = (zp, r)
    if best is None:
        log(f'board: not found in any of {len(photos)} ZED photos (on the table in front of the robot, centre?)')
        return {}
    zp, zb = best
    T_wb = zp['T'] @ zb[0]
    log(f'board: ZED sees it ({zb[1]} corners, {zb[2]:.2f} px) at world {np.round(T_wb[:3, 3], 3).tolist()}')
    g = DuoGeometry()
    out = {}
    for cam, arm in CAMS.items():
        Xs = []
        for f in rec['frames']:
            c = f.get(cam)
            if not c or 'color_full' not in c:
                continue
            r = detect(c['color_full'], c['K_full'], square)
            if r is None or r[2] > max_rms:
                continue
            F = g.link_frames(arm, np.asarray(f['q'][arm], float))[8]
            Xs.append(np.linalg.inv(F) @ T_wb @ np.linalg.inv(r[0]))
        if len(Xs) < min_views:
            log(f'board: {cam} sees the board in {len(Xs)} views (< {min_views}) - not calibrated by the board')
            continue
        R = rot_mean(np.array([X[:3, :3] for X in Xs]))
        t = np.median(np.array([X[:3, 3] for X in Xs]), 0)
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = R, t
        sp_mm = float(np.median([np.linalg.norm(Xi[:3, 3] - t) for Xi in Xs]) * 1e3)
        sp_deg = float(np.median([math.degrees(math.acos(np.clip((np.trace(R.T @ Xi[:3, :3]) - 1) / 2, -1, 1)))
                                  for Xi in Xs]))
        ok = sp_mm <= max_spread_mm and sp_deg <= max_spread_deg
        out[cam] = dict(X=X, n=len(Xs), spread_mm=sp_mm, spread_deg=sp_deg, ok=ok)
        log(f'board: {cam} {len(Xs)} views, spread {sp_mm:.1f} mm / {sp_deg:.2f} deg -> {"ACCEPTED" if ok else "not accepted"}'
            f' (flange->camera {np.round(t, 4).tolist()})')
    return out


def write(cfg_path, out, log=print):
    """Board results (flange <- OPTICAL) -> config mounts of the camera BODY frame (same convention as twin_scan)."""
    from duo_geometry import rpy_matrix
    from twin_autocalib import matrix_rpy
    from twin_calibrate import read_block, set_line
    L_opt = np.eye(4)
    L_opt[:3, :3] = rpy_matrix(-math.pi / 2, 0, -math.pi / 2)
    text = open(cfg_path).read()
    n = 0
    for cam, r in out.items():
        if not r['ok']:
            continue
        T = r['X'] @ np.linalg.inv(L_opt)
        s, e = read_block(text, cam)
        blk = set_line(text[s:e], 'xyz', '[' + ', '.join(f'{v:.4f}' for v in T[:3, 3]) + ']')
        blk = set_line(blk, 'rpy', '[' + ', '.join(f'{v:.4f}' for v in matrix_rpy(T[:3, :3])) + ']')
        blk = set_line(blk, 'calibrated', 'true')
        text = text[:s] + blk + text[e:]
        n += 1
        log(f'{cam}: mount written (ChArUco board)')
    if n:
        open(cfg_path + '.tmp', 'w').write(text)
        os.replace(cfg_path + '.tmp', cfg_path)
    return n


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('make')
    m.add_argument('--out', default=os.path.join(DEMO, 'docs', 'charuco_board_A4.png'))
    c = sub.add_parser('calib')
    c.add_argument('recording')
    c.add_argument('--square', type=float, default=SQUARE_M * 1000, help='measured square size [mm]')
    c.add_argument('--config', default=os.path.join(DEMO, 'config', 'digital_twin.yaml'))
    c.add_argument('--write', action='store_true')
    a = ap.parse_args()
    if a.cmd == 'make':
        print('board written:', make(a.out))
        return 0
    sys.path.insert(0, os.path.join(DEMO, 'scripts'))
    from twin_scan import load_recording
    out = calibrate(load_recording(a.recording), a.square / 1000)
    if a.write:
        write(a.config, out)
    return 0 if out and all(r['ok'] for r in out.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
