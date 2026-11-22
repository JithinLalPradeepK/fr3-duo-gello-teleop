#!/usr/bin/env python3
"""Test aid: show ONLY the digital-twin window (demo_gui.TwinWindow) on screen N for S seconds (read-only).
  python3 test/twin_window_only.py [--screen 1] [--seconds 30] [--windowed]
"""
import argparse
import os
import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
import demo_gui as G  # noqa: E402
import franka_theme as T  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--screen', type=int, default=1)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--windowed', action='store_true')
    a = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1])
    T.load_fonts()
    app.setFont(QFont(T.TEXT, 11))
    G.BLINK = G.Blinker()
    ros = G.Ros()
    hub, model = G.Hub(), G.Model(ros)
    win = G.TwinWindow(model, ros, hub, G.TwinLink(ros))
    screens = app.screens()
    G.place(win, screens[min(a.screen, len(screens) - 1)], not a.windowed)
    QTimer.singleShot(int(a.seconds * 1000), app.quit)
    app.exec_()
    ros.shutdown()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
