#!/usr/bin/env python3
"""Test aid: save a screenshot of an X11 window (by title) or of a whole screen as PNG (read-only).
  python3 test/grab_window.py OUT.png [--title "FR3 Duo Digital Twin"] [--screen 1]
"""
import argparse
import re
import subprocess
import sys

from PyQt5.QtWidgets import QApplication


def find_window(title):
    r = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True)
    m = re.search(r'^\s*(0x[0-9a-fA-F]+) "' + re.escape(title) + r'\*?":', r.stdout, re.M)
    return int(m.group(1), 16) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--title')
    ap.add_argument('--screen', type=int, default=0)
    a = ap.parse_args()
    app = QApplication(sys.argv)
    screens = app.screens()
    scr = screens[min(a.screen, len(screens) - 1)]
    if a.title:
        wid = find_window(a.title)
        if wid is None:
            sys.exit(f'window "{a.title}" not found')
        pm = scr.grabWindow(wid)
    else:
        g = scr.geometry()
        pm = scr.grabWindow(0, g.x(), g.y(), g.width(), g.height())
    pm.save(a.out)
    print(a.out, pm.width(), 'x', pm.height())


if __name__ == '__main__':
    main()
