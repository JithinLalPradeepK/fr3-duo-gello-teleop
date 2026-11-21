#!/usr/bin/env python3
"""Unit test of scripts/depth_view.py (depth image -> colourised JPEG). No camera, no robot."""
import os
import sys
import types

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from depth_view import depth_to_jpeg  # noqa: E402

fails = []


def check(what, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {what}' + (f' - {detail}' if detail else ''), flush=True)
    if not ok:
        fails.append(what)


def msg(arr, enc):
    return types.SimpleNamespace(data=arr.tobytes(), encoding=enc, height=arr.shape[0], width=arr.shape[1])


def decode(b):
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)        # BGR


print('== D-1 D405 style: 16UC1 millimetres')
a = np.zeros((480, 848), np.uint16)
a[:, :300] = 100                  # 0.10 m  -> nearest  -> red
a[:, 300:600] = 1000              # 1.00 m  -> farthest -> blue
# columns 600.. stay 0 = no data -> black
b = depth_to_jpeg(msg(a, '16UC1'), 0.10, 1.00, 480)
img = decode(b)
check('JPEG decodes, width 480, aspect kept (848x480 -> 480x272)', img is not None and img.shape[1] == 480 and img.shape[0] == 272,
      str(None if img is None else img.shape))
bgr = lambda x, y: img[int(y * 272 / 480), int(x * 480 / 848)].astype(int)    # noqa: E731
near, far, nodata = bgr(100, 240), bgr(450, 240), bgr(750, 240)
check('near (0.10 m) is RED (R large, B small)', near[2] > 150 and near[0] < 80, str(near))
check('far (1.00 m) is BLUE (B large, R small)', far[0] > 150 and far[2] < 100, str(far))
check('no data (0) is black', nodata.max() < 40, str(nodata))
check('small JPEG (< 60 kB)', len(b) < 60000, f'{len(b) / 1024:.0f} kB')

print('== D-2 ZED style: 32FC1 metres, full HD, with NaN / inf')
z = np.full((1080, 1920), 2.0, np.float32)
z[:, :700] = 0.3
z[:, 700:900] = np.nan
z[:, 900:1000] = np.inf
b = depth_to_jpeg(msg(z, '32FC1'), 0.30, 3.00, 480)
img = decode(b)
check('1920x1080 float -> 480x270 JPEG', img is not None and img.shape[:2] == (270, 480), str(None if img is None else img.shape))
bgr = lambda x, y: img[int(y / 4), int(x / 4)].astype(int)       # noqa: E731
check('0.3 m is red', bgr(100, 500)[2] > 150 and bgr(100, 500)[0] < 80, str(bgr(100, 500)))
check('NaN and inf are black (no crash)', bgr(750, 500).max() < 40 and bgr(950, 500).max() < 40,
      f'{bgr(750, 500)} {bgr(950, 500)}')
check('2.0 m (between) is neither red nor blue-dominated: greenish/yellow', bgr(1500, 500)[1] > 120, str(bgr(1500, 500)))

print('== D-3 robustness')
check('unsupported encoding returns None', depth_to_jpeg(msg(np.zeros((10, 10), np.uint8), 'mono8'), 0.1, 1, 480) is None)
small = depth_to_jpeg(msg(np.full((60, 80), 500, np.uint16), '16UC1'), 0.1, 1.0, 480)
check('an image smaller than the output width is not enlarged', decode(small).shape[1] == 80, str(decode(small).shape))
check('empty / all-invalid frame still produces a JPEG', depth_to_jpeg(msg(np.zeros((480, 848), np.uint16), '16UC1'), 0.1, 1, 480))

print(f'===== {"PASS" if not fails else "FAIL: " + str(fails)}')
sys.exit(1 if fails else 0)
