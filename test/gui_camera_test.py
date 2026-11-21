#!/usr/bin/env python3
"""GUI camera tests (offscreen Qt, ROS domain 199 localhost, no robot, no camera):
  C-1 the three depth topics are configured and subscribed
  C-2 the 180 degree flip: D405 RGB + depth flipped, ZED untouched, --invert changes it
  C-3 the window stays exactly screen-sized (1080 px high) on every page, both roles, and no camera tile leaves it
  C-4 NO HOME PAGE: both displays start on the camera page (main: status closed, second screen: status open on the left),
      the header has only Camera View / Operator Mode, state changes never move the main display to anything else
Run: source env/host_env.sh; python3 test/gui_camera_test.py
"""
import os
import sys
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['ROS_DOMAIN_ID'] = '199'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

import rclpy  # noqa: E402
from PyQt5.QtCore import QBuffer, QIODevice  # noqa: E402
from PyQt5.QtGui import QColor, QFont, QImage, QPainter  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

import demo_gui as G  # noqa: E402
import franka_theme as T  # noqa: E402

fails = []


def check(what, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {what}' + (f' - {detail}' if detail else ''), flush=True)
    if not ok:
        fails.append(what)


rclpy.init()
app = QApplication(sys.argv[:1])
T.load_fonts()
app.setFont(QFont(T.TEXT, 11))
G.BLINK = G.Blinker()


def settle(n=20):
    for _ in range(n):
        app.processEvents()


print('== C-1 depth topics')
want = {'zed': '/zed/zed_node/depth_view/compressed', 'd405_1': '/camera/d405_1/depth_view/compressed',
        'd405_2': '/camera/d405_2/depth_view/compressed'}
have = {key: depth for _t, key, _r, depth in G.CAMERAS}
check('CAMERAS carries a depth topic for the ZED and both D405', have == want, str(have))
real = G.Ros()                                              # not the preview: it subscribes to the topics
subs = {s.topic_name for s in real.node.subscriptions}
check('the GUI subscribes to all three depth topics', set(want.values()) <= subs, str(sorted(subs)))
check('...and to the three colour topics', {r for _t, _k, r, _d in G.CAMERAS} <= subs)
real.shutdown()

print('== C-2 180 degree flip')
img = QImage(640, 360, QImage.Format_RGB32)
img.fill(QColor('white'))
p = QPainter(img)
p.fillRect(0, 0, 80, 80, QColor(255, 0, 0))                    # red marker in the TOP-LEFT corner
p.end()
buf = QBuffer()
buf.open(QIODevice.WriteOnly)
img.save(buf, 'JPG', 95)
jpeg = bytes(buf.data())

feeder = G.PreviewFeeder()
ros = G.Ros(preview=feeder)
hub, model = G.Hub(), G.Model(ros)
hub.pip = True                                                  # decode RGB and DEPTH together
dec = G.Decoder(ros, hub)
got = {}
dec.frame.connect(lambda k, m, o: got.__setitem__((k, m), o))
win = G.MainWindow(model, ros, hub, dec, True, 'main')
win.resize(1920, 1080)
win.show()
settle()


def feed():
    got.clear()
    dec.shown.clear()
    now = time.monotonic()
    with ros.lock:
        for k in ('zed', 'd405_1', 'd405_2'):
            for m in ('rgb', 'depth'):
                ros.frames[(k, m)] = jpeg
                ros.frame_t[(k, m)] = now
    dec.set_visible(win, True)
    dec.tick()
    settle(5)


def corner(pm):
    im = pm.toImage()
    tl, br = QColor(im.pixel(3, 3)), QColor(im.pixel(im.width() - 4, im.height() - 4))
    return tl.red() > 200 and tl.green() < 80, br.red() > 200 and br.green() < 80


print('  flipped by default:', sorted(G.INVERT))
feed()
check('default INVERT is exactly the two D405 wrist cameras', G.INVERT == {'d405_1', 'd405_2'}, str(G.INVERT))
for k in ('d405_1', 'd405_2'):
    for m in ('rgb', 'depth'):
        pm = got.get((k, m))
        c = corner(pm) if hasattr(pm, 'toImage') else None
        check(f'{k} {m}: rotated by 180 (red marker now bottom-right)', c == (False, True), str(c))
for m in ('rgb', 'depth'):
    pm = got.get(('zed', m))
    c = corner(pm) if hasattr(pm, 'toImage') else None
    check(f'zed {m}: NOT rotated (marker stays top-left)', c == (True, False), str(c))
G.INVERT.clear()
G.INVERT.update({'zed'})                                        # what "--invert zed" does
feed()
c1, c2 = corner(got[('zed', 'rgb')]), corner(got[('d405_1', 'rgb')])
check('--invert zed: ZED rotated, D405 not', c1 == (False, True) and c2 == (True, False), f'{c1} {c2}')
G.INVERT.clear()
G.INVERT.update({'d405_1', 'd405_2'})
win.close()

print('== C-3 window stays screen sized')
for role in ('main', 'camera'):
    w = G.MainWindow(model, ros, hub, dec, True, role)
    w.resize(1920, 1080)
    w.show()
    settle()
    feeder.attach(ros)
    feeder.command('bringup')
    t = time.time()
    while time.time() - t < 5:
        settle(5)
        time.sleep(0.05)
    feeder.pedal('start')
    t = time.time()
    while time.time() - t < 4:
        settle(5)
        time.sleep(0.05)
    for name, idx in (('live', w.LIVE), ('operator', w.OPERATOR), ('live', w.LIVE)):
        if role == 'camera' and idx != w.LIVE:
            continue
        w.show_page(idx)
        settle(40)
        bottoms = [t_.mapTo(w, t_.rect().bottomLeft()).y() for t_ in w.findChildren(G.CameraTile) if t_.isVisible()]
        low = max(bottoms) if bottoms else 0
        check(f'{role} / {name} page: window 1920x1080 and tiles inside it', w.width() == 1920 and w.height() == 1080
              and low <= 1080, f'window {w.width()}x{w.height()}, lowest tile bottom {low}')
    w.close()
print('== C-4 no Home page')
check('the Home page is gone (no HomePage class, no HOME index)', not hasattr(G, 'HomePage') and not hasattr(G.MainWindow, 'HOME'))
main_w = G.MainWindow(model, ros, hub, dec, True, 'main')
cam_w = G.MainWindow(model, ros, hub, dec, True, 'camera')
cam_w.main_window = main_w                                      # as main() does: the camera display opens the Operator on the main one
for w in (main_w, cam_w):
    w.resize(1920, 1080)
    w.show()
settle(40)
check('the page stack has exactly two pages: camera view and operator', main_w.stack.count() == 2, str(main_w.stack.count()))
check('header tabs are exactly Camera View / Operator Mode (no Home tab)',
      [b.text() for b in main_w.live.tabs] == ['Camera View', 'Operator Mode'], str([b.text() for b in main_w.live.tabs]))
check('main display starts on the camera page', main_w.stack.currentIndex() == main_w.LIVE)
check('main display: ROBO STATUS closed, cameras fill the width', not main_w.live.status_open and not main_w.live.panel.isVisible(),
      f'open={main_w.live.status_open} visible={main_w.live.panel.isVisible()}')
check('second screen starts on the camera page', cam_w.stack.currentIndex() == cam_w.LIVE)
check('second screen: ROBO STATUS open at once', cam_w.live.status_open and cam_w.live.panel.isVisible()
      and cam_w.live.panel.width() == cam_w.live.SIDE_W, f'open={cam_w.live.status_open} width={cam_w.live.panel.width()}')
tile = cam_w.live.tiles['zed']
left = tile.mapTo(cam_w, tile.rect().topLeft()).x()
check('second screen: the status list is LEFT of the cameras and nothing leaves the window',
      cam_w.live.panel.mapTo(cam_w, cam_w.live.panel.rect().topLeft()).x() < left
      and max(t_.mapTo(cam_w, t_.rect().bottomRight()).y() for t_ in cam_w.findChildren(G.CameraTile) if t_.isVisible()) <= 1080
      and max(t_.mapTo(cam_w, t_.rect().bottomRight()).x() for t_ in cam_w.findChildren(G.CameraTile) if t_.isVisible()) <= 1920,
      f'status x={cam_w.live.panel.mapTo(cam_w, cam_w.live.panel.rect().topLeft()).x()} first tile x={left}')
check('both windows are exactly screen sized', (main_w.width(), main_w.height(), cam_w.width(), cam_w.height()) == (1920, 1080, 1920, 1080),
      f'{main_w.width()}x{main_w.height()} / {cam_w.width()}x{cam_w.height()}')

feeder.command('bringup')
t = time.time()
while time.time() - t < 5:
    settle(5)
    time.sleep(0.05)
check('after BRING UP (IDLE) the main display is still on the camera page', main_w.stack.currentIndex() == main_w.LIVE)
feeder.pedal('start')
t = time.time()
while time.time() - t < 4:
    settle(5)
    time.sleep(0.05)
check('in TELEOP the main display is on the camera page', main_w.stack.currentIndex() == main_w.LIVE and ros is not None
      and model.state in ('TELEOP', 'TELEOP_STARTING'), f'{main_w.stack.currentIndex()} {model.state}')
feeder.pedal('stop')
t = time.time()
while time.time() - t < 4:
    settle(5)
    time.sleep(0.05)
check('after STOP the main display STAYS on the camera page (it used to fall back to Home)',
      main_w.stack.currentIndex() == main_w.LIVE and model.state == 'IDLE', f'{main_w.stack.currentIndex()} {model.state}')
cam_w.open_operator()                                           # OPERATOR tab on the second screen
settle(20)
check('Operator tab on the second screen opens the Operator page on the MAIN display; second screen stays on cameras',
      main_w.stack.currentIndex() == main_w.OPERATOR and cam_w.stack.currentIndex() == cam_w.LIVE)
main_w.operator.back.emit()
settle(20)
check('"Back" on the Operator page returns to the camera page', main_w.stack.currentIndex() == main_w.LIVE)
main_w.open_operator()
settle(20)
feeder.pedal('start')                                           # teleop starts while the Operator page is open
t = time.time()
while time.time() - t < 3:
    settle(5)
    time.sleep(0.05)
check('when teleop starts the main display leaves the Operator page for the cameras', main_w.stack.currentIndex() == main_w.LIVE,
      f'{main_w.stack.currentIndex()} {model.state}')
main_w.close()
cam_w.close()
ros.shutdown()
rclpy.try_shutdown()
print(f'===== {"PASS" if not fails else "FAIL: " + str(fails)}')
sys.exit(1 if fails else 0)
