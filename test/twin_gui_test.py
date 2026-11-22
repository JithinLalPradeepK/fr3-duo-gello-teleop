#!/usr/bin/env python3
"""Digital-twin DISPLAY test on the real X display (no robot, fake data). Run via test/sim_twin_gui_test.sh
(ROS_DOMAIN_ID=78, localhost only; the twin launch with RViz + test/twin_fake_robots.py run alongside).
Opens ONLY the twin window (demo_gui.TwinWindow) full screen on the second monitor (first if there is one) and checks:
  V1 the RViz window gets embedded               V2 only the 3D panel is visible (no RViz menu / tool / status bar)
  V3 the robots are drawn (white pixels)          V4 telemetry: both arms LIVE with rate and latency
  V5 ROBOTS layer off -> robots disappear, on -> back
  V6 RViz killed -> loading animation, respawned RViz is embedded again
Screenshots: <logdir>/twin_*.png
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont, QImage
from PyQt5.QtWidgets import QApplication
from rclpy.signals import SignalHandlerOptions

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
import demo_gui as G  # noqa: E402
import franka_theme as T  # noqa: E402

RESULTS = []
STAGE = (19, 34, 42)


def check(name, ok, detail=''):
    RESULTS.append(bool(ok))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}{"  - " + detail if detail else ""}', flush=True)


def pixel_stats(img, rect):
    """(fraction of stage-coloured pixels, fraction of OBJECT pixels (clearly not stage / grid), fraction of light-grey
    RViz-UI pixels) in rect."""
    stage = bright = ui = n = 0
    x0, y0, w, h = rect
    for y in range(y0, y0 + h, 6):
        for x in range(x0, x0 + w, 6):
            c = img.pixelColor(x, y)
            r, g, b = c.red(), c.green(), c.blue()
            n += 1
            if abs(r - STAGE[0]) < 14 and abs(g - STAGE[1]) < 14 and abs(b - STAGE[2]) < 14:
                stage += 1
            if abs(r - STAGE[0]) + abs(g - STAGE[1]) + abs(b - STAGE[2]) > 150:
                bright += 1
            if 225 <= r <= 245 and 225 <= g <= 245 and 225 <= b <= 245 and max(r, g, b) - min(r, g, b) < 6:
                ui += 1
    return stage / max(n, 1), bright / max(n, 1), ui / max(n, 1)


def rviz_pids():
    out = []
    for p in subprocess.run(['pgrep', '-f', '^/opt/ros/humble/lib/rviz2/rviz2'], capture_output=True, text=True).stdout.split():
        try:
            env = open(f'/proc/{p}/environ', 'rb').read().split(b'\0')
        except OSError:
            continue
        if b'ROS_DOMAIN_ID=78' in env:
            out.append(int(p))
    return out


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '78':
        sys.exit('run via test/sim_twin_gui_test.sh')
    logdir = sys.argv[1]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1])
    T.load_fonts()
    app.setFont(QFont(T.TEXT, 11))
    G.BLINK = G.Blinker()
    ros = G.Ros()
    hub, model = G.Hub(), G.Model(ros)
    link = G.TwinLink(ros)
    win = G.TwinWindow(model, ros, hub, link)
    screens = app.screens()
    G.place(win, screens[1 if len(screens) > 1 else 0], True)
    scr = win.screen()
    steps = []

    def wait(cond, timeout, then):
        t0 = time.monotonic()

        def poll():
            if cond() or time.monotonic() - t0 > timeout:
                then(cond())
            else:
                QTimer.singleShot(200, poll)
        poll()

    def grab(name):
        # grab the twin window itself (a root grab of a full-screen window is black under the compositor)
        img = scr.grabWindow(int(win.winId())).toImage()
        img.save(os.path.join(logdir, f'twin_{name}.png'))
        return img

    def view_rect():
        tl = win.view.mapTo(win, win.view.rect().topLeft())
        return tl.x(), tl.y(), win.view.width(), win.view.height()

    def embedded_for_real():
        v = win.view
        return v.embedded and G.x11_parent(v.wid) == int(v.container.winId())

    def s1(ok):
        ok = ok and embedded_for_real()
        check('V1 RViz window embedded in the twin display (X parent = our container)', ok)
        if not ok:
            return app.quit()
        QTimer.singleShot(3000, s2)

    def s2():
        img = grab('1_embedded')
        x, y, w, h = view_rect()
        stage, bright, _ui = pixel_stats(img, (x, y, w, h))
        top = pixel_stats(img, (x, y, w, 40))
        check('V2 no RViz menu / tool bar: top strip of the view is 3D stage, not grey UI',
              top[2] < 0.05 and top[0] > 0.5, f'stage {top[0]:.2f}, ui {top[2]:.2f}')
        check('V3 3D stage visible with robots (object pixels)', stage > 0.4 and bright > 0.01,
              f'stage {stage:.2f}, objects {bright:.4f}')
        cells = {k: c.val.text() for k, c in win.cells.items()}
        check('V4 telemetry: both arms LIVE with Hz and ms', all(cells[a].startswith('LIVE') and 'Hz' in cells[a] and
                                                                  'ms' in cells[a] for a in ('left', 'right')), str(cells))
        win.layer_btn['robots'].click()
        wait(lambda: not win.layer_btn['robots'].isChecked() and (link_layers().get('robots') is False), 6,
             lambda ok: QTimer.singleShot(1500, lambda: s3(ok, bright)))

    def link_layers():
        return (win.ts or {}).get('layers') or {}

    def s3(ok, bright_before):
        img = grab('2_robots_off')
        _s, bright, _u = pixel_stats(img, view_rect())
        check('V5 ROBOTS layer off -> robots hidden (object pixels drop)', ok and bright < bright_before * 0.6,
              f'objects {bright_before:.4f} -> {bright:.4f}')
        win.layer_btn['robots'].click()
        wait(lambda: link_layers().get('robots') is True, 6, lambda ok2: QTimer.singleShot(1500, lambda: s4(ok2)))

    def s4(ok):
        img = grab('3_robots_on')
        _s, bright, _u = pixel_stats(img, view_rect())
        check('V5 ROBOTS layer on again -> robots back', ok and bright > 0.01, f'objects {bright:.4f}')
        pids = rviz_pids()
        for p in pids:
            os.kill(p, signal.SIGKILL)
        wait(lambda: not win.view.embedded, 5, s5)

    def s5(ok):
        check('V6 RViz killed -> twin display shows the loading animation', ok)
        grab('4_rviz_restarting')
        wait(embedded_for_real, 25, s6)

    def s6(ok):
        check('V6 respawned RViz embedded again', ok)
        QTimer.singleShot(3000, lambda: (grab('5_reembedded'), app.quit()))

    QTimer.singleShot(500, lambda: wait(embedded_for_real, 25, s1))
    app.exec_()
    ros.shutdown()
    rclpy.try_shutdown()
    print(f'\nRESULT: {sum(RESULTS)}/{len(RESULTS)} passed   (screenshots: {logdir})')
    sys.exit(0 if RESULTS and all(RESULTS) else 1)


if __name__ == '__main__':
    main()
