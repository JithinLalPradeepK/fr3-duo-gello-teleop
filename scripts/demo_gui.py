#!/usr/bin/env python3
"""Robot demo GUI - a thin client of demo_supervisor and desk_agent (same services as the foot pedals).

  source ~/robot_demo/env/host_env.sh
  python3 scripts/demo_gui.py [--single] [--control-screen 0] [--camera-screen 1]
  python3 scripts/demo_gui.py --preview      design preview with demo data: sends NOTHING
      (own ROS domain 199, localhost only). Preview keys: F5 = pedal START, F6 = STOP, F7 = HOME,
      F9 = demo ERROR, F10 = drop a camera

Redesign (branch features/GUI-development, see docs/GUI_CHANGES.md):
  HOME      white page, robot image + one line, NO buttons. Foot pedals drive everything.
  LIVE      opens by itself when teleop starts: 3 labelled camera views, RGB / DEPTH switch (top right),
            status + errors on the left. Shown identically on BOTH monitors (mirrored).
  OPERATOR  navbar menu: Testing stage, Fix errors, Status & logs, Robot setup, System (all of Jithin's old buttons).
  NAVBAR    logo (black/white with the mode), LEFT / RIGHT arm chips, CAMERAS chip (click = the 3 cameras),
            Operator menu, light / dark switch (light is the default).
The GUI contains no demo logic: every button is a /demo/* service call; demo_supervisor / desk_agent decide.
THE GUI IS NOT A SAFETY DEVICE - the physical E-stop stays in use.
"""
import argparse
import json
import os
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import (QBuffer, QByteArray, QEasingCurve, QIODevice, QObject, QPoint, QPropertyAnimation, QRectF,
                          QSize, Qt, QTimer, pyqtProperty, pyqtSignal)
from PyQt5.QtGui import (QColor, QFont, QImage, QImageReader, QLinearGradient, QPainter, QPainterPath, QPen,
                         QPixmap)
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QFrame, QGridLayout, QHBoxLayout, QLabel, QMenu,
                             QMessageBox, QPlainTextEdit, QPushButton, QShortcut, QSizePolicy, QStackedWidget,
                             QVBoxLayout, QWidget)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import franka_theme as T  # noqa: E402

ARMS = ('left', 'right')
ARM_IP = {'left': '192.168.1.51', 'right': '192.168.1.50'}
# (title on screen, key, RGB topic, DEPTH topic).
# >>> DEPTH: Jithin adds the depth topic here (a sensor_msgs/CompressedImage that QImage can decode, e.g. a
# >>> colourised depth PNG/JPEG). None = the DEPTH view shows "not available yet". Nothing else needs to change.
CAMERAS = [('ZED Mini · Overview', 'zed', '/zed/zed_node/rgb/color/rect/image/compressed', None),
           ('Wrist camera 1 · D405', 'd405_1', '/camera/d405_1/color/image_raw/compressed', None),
           ('Wrist camera 2 · D405', 'd405_2', '/camera/d405_2/color/image_raw/compressed', None)]
CAM_TITLE = {key: title for title, key, _, _ in CAMERAS}
TELEOP_STATES = ('TELEOP_STARTING', 'TELEOP')
HEADLINE = 'Move your hands.\nTwo robots follow.'
SUBLINE = 'FRANKA RESEARCH 3 DUO  ·  GELLO TELEOPERATION'
HOME_LINE = {'STARTING_UP': 'Getting ready ...', 'IDLE': 'Ready.', 'HOMING': 'The robots are returning home ...',
             'STOPPING': 'Stopping ...', 'RECOVERING': 'Recovering ...', 'SHUTTING_DOWN': 'Shutting down ...'}
HINTS = {'STARTING_UP': 'Starting arms, grippers, GELLO and cameras ...',
         'IDLE': 'Press START, then lift both GELLOs out of their holders.',
         'TELEOP_STARTING': 'Hold the GELLOs still - the robots move to the GELLO pose.',
         'TELEOP': 'The robots follow the GELLOs.  STOP ends teleoperation.',
         'STOPPING': 'The robots hold their pose.',
         'HOMING': 'Both arms move to the home pose.',
         'ERROR': 'Open Operator > Fix errors.',
         'RECOVERING': 'Franka error recovery on both arms ...',
         'SHUTTING_DOWN': 'Stopping cameras, GELLO, grippers and arms ...',
         'OFFLINE': 'Set up the robots (Operator > Robot setup), then Operator > System > BRING UP.'}
# Desk toggles: key -> (title, off text, on text, command on, command off, is_on(arm status))
TOGGLES = {
    'brakes': ('BRAKES', 'LOCKED', 'OPEN', 'desk/unlock', 'desk/lock',
               lambda a: {'unlocked': True, 'locked': False}.get(a.get('brakes'))),
    'mode': ('OPERATING MODE', 'PROGRAMMING', 'EXECUTION', 'desk/execution', 'desk/programming',
             lambda a: {'Execution': True, 'Programming': False}.get(a.get('mode'))),
    'fci': ('FCI', 'OFF', 'ON', 'desk/fci_on', 'desk/fci_off', lambda a: a.get('fci')),
}
# technical error words -> plain words
PLAIN = {'cartesian_reflex': 'the robot stopped itself after an unexpected force',
         'self_collision_avoidance_violation': 'the arm came too close to itself',
         'joint_position_limits_violation': 'a joint limit was reached',
         'communication_constraints_violation': 'the robot lost its control connection',
         'cartesian_position_limits_violation': 'a position limit was reached',
         'tau_j_range_violation': 'joint torque too high',
         'REFLEX': 'robot stopped itself (reflex)', 'USER_STOPPED': 'emergency stop pressed'}
SEVERITY = {'error': 0, 'warn': 1, 'info': 2}


def plain(text):
    for k, v in PLAIN.items():
        text = text.replace(k, v)
    return text


# ---------------------------------------------------------------------------------------------- ROS client
class Ros(QObject):
    """rclpy in a background thread; hands data to Qt via signals / latest-frame buffers."""
    status = pyqtSignal(dict)
    desk = pyqtSignal(dict)
    reply = pyqtSignal(str, bool, str)

    def __init__(self, preview=None):
        super().__init__()
        self.preview = preview                      # PreviewFeeder in --preview mode: nothing is ever sent
        self.node = Node('demo_gui_preview' if preview else 'demo_gui')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.frames = {}                            # (camera key, 'rgb'|'depth') -> latest JPEG/PNG bytes
        self.frame_t = {}
        self.lock = threading.Lock()
        if not preview:
            self.node.create_subscription(String, '/demo/status', self.on_status, latched)
            self.node.create_subscription(String, '/demo/desk/status', self.on_desk, latched)
            for _, key, rgb, depth in CAMERAS:
                for mode, topic in (('rgb', rgb), ('depth', depth)):
                    if topic:
                        self.node.create_subscription(CompressedImage, topic,
                                                      lambda m, k=(key, mode): self.on_image(k, m),
                                                      qos_profile_sensor_data)
        self.clients = {}
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_status(self, m):
        try:
            self.status.emit(json.loads(m.data))
        except ValueError:
            pass

    def on_desk(self, m):
        try:
            self.desk.emit(json.loads(m.data))
        except ValueError:
            pass

    def on_image(self, k, m):
        with self.lock:
            self.frames[k] = bytes(m.data)
            self.frame_t[k] = time.monotonic()

    def call(self, name):
        if self.preview:
            self.preview.command(name)
            self.reply.emit(name, True, 'PREVIEW - nothing sent')
            return
        cli = self.clients.get(name) or self.clients.setdefault(
            name, self.node.create_client(Trigger, f'/demo/{name}'))
        if not cli.service_is_ready():
            self.reply.emit(name, False, 'no answer - is the demo running?')
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self.reply.emit(
            name, bool(f.result() and f.result().success),
            f.result().message if f.result() else 'call failed'))

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


# ---------------------------------------------------------------------------------------------- shared state
class Blinker(QObject):
    tick = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.on = True
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._t)
        self.timer.start(550)

    def _t(self):
        self.on = not self.on
        self.tick.emit()


BLINK = None


class Hub(QObject):
    """Settings shared by all windows (both monitors show the same thing)."""
    theme_changed = pyqtSignal(str)
    depth_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.theme, self.depth = 'light', 'rgb'

    def set_theme(self, mode):
        self.theme = T.MODE = mode
        self.theme_changed.emit(mode)

    def set_depth(self, mode):
        self.depth = mode
        self.depth_changed.emit(mode)


class Model(QObject):
    """Latest supervisor / desk status plus everything derived from it (chips, lights, plain-words problems)."""
    updated = pyqtSignal()
    log_line = pyqtSignal(str)

    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.s, self.desk = {}, {}
        self.last_status_t = self.desk_t = 0.0
        self.refused = ('', 0.0)
        self.lines, self.last_logged, self.desk_last = [], None, None
        ros.status.connect(self.on_status)
        ros.desk.connect(self.on_desk)
        ros.reply.connect(self.on_reply)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updated.emit)       # staleness check once a second
        self.timer.start(1000)

    # -- input
    def log(self, text):
        line = f'{time.strftime("%H:%M:%S")}  {text}'
        self.lines.append(line)
        del self.lines[:-500]
        self.log_line.emit(line)

    def on_status(self, s):
        self.s, self.last_status_t = s, time.monotonic()
        key = (s.get('state'), s.get('message'))
        if key != self.last_logged:
            self.last_logged = key
            self.log(f'{key[0]}: {key[1] or ""}')
        self.updated.emit()

    def on_desk(self, d):
        self.desk, self.desk_t = d, time.monotonic()
        if d.get('last') and d['last'] != self.desk_last:
            self.desk_last = d['last']
            self.log(f'Desk: {d["last"]}')
        self.updated.emit()

    def on_reply(self, name, ok, msg):
        self.log(f'  {name}: {"OK" if ok else "REFUSED"} - {msg}')
        if not ok:
            self.refused = (f'{name.upper().split("/")[-1]} refused: {plain(msg)}', time.monotonic())
        self.updated.emit()

    # -- derived
    @property
    def connected(self):
        return time.monotonic() - self.last_status_t <= 3.0

    @property
    def desk_ok(self):
        return bool(self.desk) and time.monotonic() - self.desk_t <= 3.0

    @property
    def state(self):
        return self.s.get('state', 'OFFLINE') if self.connected else 'OFFLINE'

    @property
    def message(self):
        return self.s.get('message', '') if self.connected else 'No connection to the demo'

    def arm(self, name):
        return self.s.get('arms', {}).get(name, {}) if self.connected else {}

    def arm_level(self, name):
        a = self.arm(name)
        ok = (a.get('connected') and a.get('robot_mode') in ('IDLE', 'MOVE') and not a.get('errors'))
        return 'ok' if ok else 'bad'

    def cam_fps(self, key):
        if not self.connected or self.s.get('cameras_launch') != 'running':
            return None
        fps = self.s.get('cameras', {}).get(key)
        return fps if fps and fps >= 1.0 else None

    def cam_level(self, key):
        return 'ok' if self.cam_fps(key) else 'bad'

    def lights(self):
        """System lights for the Status page: name -> (level, detail)."""
        names = ('Desk', 'Robots', 'FCI', 'GELLO', 'Grippers', 'Cameras', 'Pedal')
        out = {n: ('off', 'no data') for n in names}
        if self.desk_ok:
            ready = [self.desk.get(a, {}).get('mode') == 'Execution' and self.desk.get(a, {}).get('brakes') == 'unlocked'
                     and self.desk.get(a, {}).get('fci') for a in ARMS]
            err = any(self.desk.get(a, {}).get('error') for a in ARMS)
            out['Desk'] = ('bad' if err else 'ok' if all(ready) else 'warn', '')
        if not self.connected:
            return out
        s, state = self.s, self.state
        off = state == 'OFFLINE'
        arms = [self.arm(a) for a in ARMS]
        modes = [a.get('robot_mode', '-') for a in arms]
        if any(m in ('REFLEX', 'USER_STOPPED') for m in modes) or any(a.get('errors') for a in arms):
            out['Robots'] = ('bad', ', '.join(modes))
        elif all(m in ('IDLE', 'MOVE') for m in modes):
            out['Robots'] = ('ok', ', '.join(modes))
        else:
            out['Robots'] = ('off' if off else 'warn', ', '.join(modes))
        out['FCI'] = ('ok' if all(a.get('connected') for a in arms) else 'off' if off else 'bad', '')
        gello = [a.get('gello', 'no data') for a in arms]
        out['GELLO'] = ('ok' if all(g in ('in holder', 'out of holder') for g in gello) else 'off' if off else 'bad',
                        ', '.join(gello))
        grip = [a.get('gripper', 'off') for a in arms]
        out['Grippers'] = ('ok' if all(g == 'ready' for g in grip) else 'off' if all(g == 'off' for g in grip)
                           else 'bad', ', '.join(grip))
        lv = [self.cam_level(k) for k in CAM_TITLE]
        out['Cameras'] = ('off' if s.get('cameras_launch', 'off') == 'off' else 'ok' if all(x == 'ok' for x in lv)
                          else 'bad', '')
        out['Pedal'] = ('ok' if s.get('pedal') == 'connected' else 'off', s.get('pedal', ''))
        return out

    def issues(self):
        """Problems in plain words, most severe first: [(level, text)]."""
        out, state = [], self.state
        if not self.connected:
            out.append(('error', 'No connection to the demo system. Start it with the Robot Demo icon.'))
            return out
        if state == 'ERROR':
            out.append(('error', plain(self.message) + '  (Operator > Fix errors)'))
        elif state == 'OFFLINE':
            out.append(('info', 'The demo is offline. Operator > Robot setup, then Operator > System > BRING UP.'))
        if state not in ('OFFLINE', 'ERROR'):
            for a in ARMS:
                d = self.arm(a)
                errs = d.get('errors') or []
                if errs:
                    out.append(('error', f'{a.capitalize()} arm: ' + ', '.join(plain(e) for e in errs)))
                elif d.get('robot_mode') in ('REFLEX', 'USER_STOPPED'):
                    out.append(('error', f'{a.capitalize()} arm: {plain(d["robot_mode"])}'))
                elif not d.get('connected') and state != 'STARTING_UP':
                    out.append(('error', f'{a.capitalize()} arm is not connected'))
                if d.get('gripper') == 'not ready' and state != 'STARTING_UP':
                    out.append(('warn', f'{a.capitalize()} gripper is not ready'))
                if d.get('gello') == 'no data' and state != 'STARTING_UP':
                    out.append(('warn', f'{a.capitalize()} GELLO sends no data'))
            if state != 'STARTING_UP':
                for key, title in CAM_TITLE.items():
                    if not self.cam_fps(key):
                        out.append(('warn', f'{title} is not connected'))
            if self.s.get('pedal') != 'connected':
                out.append(('warn', 'Foot pedal not connected (use Operator > Testing stage)'))
        if self.refused[0] and time.monotonic() - self.refused[1] < 15:
            out.append(('error', self.refused[0]))
        out.sort(key=lambda x: SEVERITY[x[0]])
        return out


# ---------------------------------------------------------------------------------------------- widgets
class Dot(QWidget):
    """Status light. ok = green (blinks), bad = red (blinks), warn = amber, off = grey."""
    COLORS = {'ok': T.SAFE_GREEN, 'bad': T.SAFE_RED, 'warn': T.SAFE_AMBER, 'off': T.OFF_WHITE}

    def __init__(self, level='off', size=12):
        super().__init__()
        self.level = level
        self.setFixedSize(size, size)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        BLINK.tick.connect(self.update)

    def set_level(self, level):
        if level != self.level:
            self.level = level
            self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = QColor(self.COLORS.get(self.level, T.OFF_WHITE))
        if self.level in ('ok', 'bad') and not BLINK.on:
            c.setAlphaF(0.25)
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawEllipse(self.rect().adjusted(1, 1, -1, -1))


class StatusChip(QFrame):
    """Navbar chip: dot + text. Connected = Franka Dark Blue with a blinking green dot; not connected = light
    Franka Grey with a blinking red dot. Clicking shows a small popup with the details."""

    def __init__(self, text):
        super().__init__()
        self.setObjectName('chip')
        self.text, self.level, self.rows = text, 'off', []
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 0, 18, 0)
        lay.setSpacing(10)
        self.dot = Dot()
        self.lab = QLabel(text.upper())
        self.lab.setAttribute(Qt.WA_TransparentForMouseEvents)
        lay.addWidget(self.dot)
        lay.addWidget(self.lab)
        self.setFixedHeight(40)
        self.popup = QFrame(None, Qt.Popup | Qt.FramelessWindowHint)
        self.popup.hideEvent = lambda ev: self.popup_closed()
        self.closed_t = 0.0
        self.setCursor(Qt.PointingHandCursor)
        self.popup_lay = QVBoxLayout(self.popup)
        self.popup_lay.setContentsMargins(16, 12, 20, 12)
        self.popup_lay.setSpacing(8)
        self.restyle()

    def set_state(self, level, rows):
        self.level, self.rows = level, rows
        self.dot.set_level(level)
        self.restyle()
        if self.popup.isVisible():
            self.fill_popup()

    def restyle(self):
        c = T.pal()
        ok = self.level == 'ok'
        bg, fg = (c['chip_ok_bg'], c['chip_ok_fg']) if ok else (c['chip_bad_bg'], c['chip_bad_fg'])
        self.setStyleSheet(f"QFrame#chip {{background:{bg}; border-radius:20px;}}"
                           f"QLabel {{background:transparent; color:{fg}; font-family:'{T.HEAD}'; "
                           "font-weight:600; font-size:12px;}")
        self.popup.setStyleSheet(f"QFrame {{background:{c['popup']}; border-radius:12px;}}"
                                 f"QLabel {{background:transparent; color:{c['popup_fg']}; font-size:13px; "
                                 f"font-family:'{T.TEXT}';}}")

    @staticmethod
    def clear(layout):
        while layout.count():
            item = layout.takeAt(0)
            if item.layout():
                StatusChip.clear(item.layout())
            elif item.widget():
                item.widget().deleteLater()

    def fill_popup(self):
        self.clear(self.popup_lay)
        for level, text in self.rows or [('off', 'no data')]:
            row = QHBoxLayout()
            row.setSpacing(10)
            row.addWidget(Dot(level))
            row.addWidget(QLabel(text))
            row.addStretch(1)
            self.popup_lay.addLayout(row)
        self.popup.adjustSize()

    def mousePressEvent(self, e):
        """Click opens the details popup (a click outside closes it). Hovering does nothing."""
        if e.button() != Qt.LeftButton:
            return
        if time.monotonic() - self.closed_t < 0.25:       # this click just closed the popup: don't reopen it
            return
        self.fill_popup()
        self.popup.move(self.mapToGlobal(QPoint(0, self.height() + 8)))
        self.popup.show()

    def popup_closed(self):
        self.closed_t = time.monotonic()

    def hideEvent(self, e):
        try:
            self.popup.hide()
        except RuntimeError:        # popup already destroyed at shutdown
            pass


class ImageView(QWidget):
    """Draws a pixmap letterboxed (cameras) or a text; used for the robot picture and the camera tiles."""

    def __init__(self, rounded=True, framed=True, icon=False):
        super().__init__()
        self.pm, self.text, self.rounded, self.framed, self.icon = None, '', rounded, framed, icon
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setMinimumSize(120, 90)

    def set_pixmap(self, pm):
        self.pm, self.text = pm, ''
        self.update()

    def set_text(self, text):
        self.pm, self.text = None, text
        self.update()

    def paintEvent(self, _e):
        c = T.pal()
        p = QPainter(self)
        p.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        r = QRectF(self.rect())
        if self.framed:
            path = QPainterPath()
            path.addRoundedRect(r, 14, 14)
            p.fillPath(path, QColor(c['camera_bg']))
            p.setClipPath(path)
        if self.pm and not self.pm.isNull():
            s = self.pm.size().scaled(self.size(), Qt.KeepAspectRatio)
            p.drawPixmap(QRectF((r.width() - s.width()) / 2, (r.height() - s.height()) / 2, s.width(), s.height()),
                         self.pm, QRectF(self.pm.rect()))
        elif self.text:
            f = QFont(T.HEAD, 12)
            f.setWeight(QFont.DemiBold)
            p.setFont(f)
            p.setPen(QColor(c['muted']))
            area = self.rect()
            if self.icon:                       # Franka trademark where the camera picture is missing
                ic = T.icon_pixmap(T.MODE)
                side = max(48, min(160, int(min(self.width(), self.height()) * 0.38)))
                p.setOpacity(0.9)
                p.drawPixmap(QRectF((r.width() - side) / 2, r.height() / 2 - side * 0.75, side, side), ic,
                             QRectF(ic.rect()))
                p.setOpacity(1.0)
                area = self.rect().adjusted(0, int(r.height() / 2 + side * 0.4), 0, 0)
                p.drawText(area, Qt.AlignHCenter | Qt.AlignTop | Qt.TextWordWrap, self.text)
            else:
                p.drawText(area, Qt.AlignCenter | Qt.TextWordWrap, self.text)


def label(text, name='value', wrap=False):
    w = QLabel(text)
    w.setObjectName(name)
    w.setWordWrap(wrap)
    return w


def card(title=None, extra=None):
    f = QFrame()
    f.setObjectName('card')
    v = QVBoxLayout(f)
    v.setContentsMargins(22, 18, 22, 18)
    v.setSpacing(10)
    if title:
        h = QHBoxLayout()
        h.addWidget(label(title, 'cardTitle'))
        h.addStretch(1)
        if extra:
            h.addWidget(label(extra, 'mono'))
        v.addLayout(h)
    return f, v


class Switch(QWidget):
    """Painted toggle switch. value: True (on), False (off), None (unknown or arms differ)."""
    clicked = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setFixedSize(84, 42)
        self.setCursor(Qt.PointingHandCursor)
        self.value, self.pending = None, False
        self._pos = 0.5
        self.anim = QPropertyAnimation(self, b'knob', self)
        self.anim.setDuration(180)
        self.anim.setEasingCurve(QEasingCurve.InOutCubic)

    def get_knob(self):
        return self._pos

    def set_knob(self, p):
        self._pos = p
        self.update()

    knob = pyqtProperty(float, get_knob, set_knob)

    def set_value(self, value, pending):
        if value != self.value:
            self.anim.stop()
            self.anim.setStartValue(self._pos)
            self.anim.setEndValue({True: 1.0, False: 0.0}.get(value, 0.5))
            self.anim.start()
        self.value, self.pending = value, pending
        self.update()

    def mousePressEvent(self, e):
        if self.isEnabled() and e.button() == Qt.LeftButton:
            self.clicked.emit()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if not self.isEnabled():
            p.setOpacity(0.45)
        r = QRectF(3, 3, self.width() - 6, self.height() - 6)
        track = {True: T.MINT, False: T.GREY}.get(self.value, T.OFF_WHITE)
        p.setPen(QPen(QColor(T.SAFE_AMBER), 3) if self.pending else Qt.NoPen)
        p.setBrush(QColor(track))
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2)
        d = r.height() - 8
        x = r.left() + 4 + self._pos * (r.width() - 8 - d)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 40))
        p.drawEllipse(QRectF(x, r.top() + 5, d, d))
        p.setBrush(QColor(T.WHITE))
        p.drawEllipse(QRectF(x, r.top() + 4, d, d))


class Toggle(QWidget):
    """Title + [off text] (switch) [on text]; asks for the opposite of the current value when clicked."""
    requested = pyqtSignal(bool)

    def __init__(self, title, off_text, on_text):
        super().__init__()
        self.setObjectName('plain')
        g = QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(14)
        self.title = label(title, 'caption')
        self.off, self.on = label(off_text, 'plain'), label(on_text, 'plain')
        self.off.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.sw = Switch()
        self.sw.clicked.connect(lambda: self.requested.emit(not self.sw.value if self.sw.value is not None else True))
        g.addWidget(self.title, 0, 0, 1, 3)
        g.addWidget(self.off, 1, 0)
        g.addWidget(self.sw, 1, 1)
        g.addWidget(self.on, 1, 2)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(2, 1)
        self.set_value(None, False)

    def set_value(self, value, pending):
        self.sw.set_value(value, pending)
        c = T.pal()
        for w, active in ((self.off, value is False), (self.on, value is True)):
            w.setStyleSheet(f"background:transparent; font-family:'{T.HEAD}'; font-size:13px; font-weight:600; "
                            f"color:{c['fg'] if active else c['muted']};")

    def setEnabled(self, on):
        super().setEnabled(on)
        self.set_value(self.sw.value, self.sw.pending)


class ConfirmClose:
    """Mixin: when the GUI owns the demo (started by start_demo.sh), closing any window asks first and
    then quits the whole GUI -> start_demo.sh shuts the stack down (gates first, robots hold)."""
    owns_demo = False

    def closeEvent(self, event):
        if not ConfirmClose.owns_demo:
            event.accept()
            return
        answer = QMessageBox.question(
            self, 'Shut down the robot demo?',
            'Closing the GUI shuts down the robot demo:\n'
            '- a running teleop / HOME is stopped (robots hold where they are)\n'
            '- arm stacks, grippers, GELLO and cameras are stopped\n'
            '- Desk control is released (FCI off; brakes and mode stay as they are).\n\nShut down now?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            event.accept()
            QApplication.quit()
        else:
            event.ignore()


# ---------------------------------------------------------------------------------------------- camera decoding
class Decoder(QObject):
    """Decodes the camera streams ONCE (only while a LIVE page is visible) and hands the pictures to every window."""
    frame = pyqtSignal(str, object)         # camera key, QPixmap or a text to show

    def __init__(self, ros, hub):
        super().__init__()
        self.ros, self.hub = ros, hub
        self.visible, self.shown = {}, {}
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        hub.depth_changed.connect(lambda _m: self.shown.clear())

    def set_visible(self, win, on):
        was = any(self.visible.values())
        self.visible[win] = on
        if on and not was:
            self.shown.clear()
        elif on:
            self.shown.clear()                # a second window just opened the page: send the latest picture again

    def tick(self):
        if not any(self.visible.values()):
            return
        mode, now = self.hub.depth, time.monotonic()
        for _title, key, _rgb, depth in CAMERAS:
            if mode == 'depth' and depth is None:
                if self.shown.get(key) != ('na',):
                    self.shown[key] = ('na',)
                    self.frame.emit(key, 'DEPTH VIEW\nNOT AVAILABLE YET')
                continue
            with self.ros.lock:
                data, t = self.ros.frames.get((key, mode)), self.ros.frame_t.get((key, mode), 0.0)
            if data is None or now - t > 1.0:
                if self.shown.get(key) != ('none',):
                    self.shown[key] = ('none',)
                    self.frame.emit(key, 'NO SIGNAL')
                continue
            sh = self.shown.get(key)
            if sh and sh[0] == 'img' and sh[1] is data:
                continue
            buf = QBuffer()
            buf.setData(QByteArray(data))
            buf.open(QIODevice.ReadOnly)
            reader = QImageReader(buf)
            full = reader.size()
            limit = QSize(1280, 720) if key == 'zed' else QSize(960, 540)
            if full.isValid() and (full.width() > limit.width() or full.height() > limit.height()):
                reader.setScaledSize(full.scaled(limit, Qt.KeepAspectRatio))      # cheap libjpeg DCT scaling
            img = reader.read()
            if img.isNull():
                continue
            self.shown[key] = ('img', data)
            self.frame.emit(key, QPixmap.fromImage(img))


# ---------------------------------------------------------------------------------------------- pages
class HomePage(QWidget):
    """Welcome page: picture, one line, no buttons."""

    def __init__(self):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(96, 24, 64, 24)
        lay.setSpacing(40)
        left = QVBoxLayout()
        left.setSpacing(18)
        left.addStretch(1)
        self.head = label(HEADLINE, 'headline')
        self.sub = label(SUBLINE, 'subline')
        self.line = label('', 'subline')
        for w in (self.head, self.sub, self.line):
            left.addWidget(w)
        left.addStretch(1)
        lay.addLayout(left, 5)
        self.img = ImageView(framed=False)
        self.img.setMinimumSize(300, 300)
        self.img.set_pixmap(QPixmap(T.ROBOTS))
        lay.addWidget(self.img, 6)

    def update_state(self, state):
        self.line.setText(HOME_LINE.get(state, ''))


class CameraTile(QWidget):
    def __init__(self, title):
        super().__init__()
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(8)
        h = QHBoxLayout()
        self.title = label(title.upper(), 'caption')
        self.fps = label('', 'mono')
        h.addWidget(self.title)
        h.addStretch(1)
        h.addWidget(self.fps)
        v.addLayout(h)
        self.view = ImageView(icon=True)
        self.view.set_text('NO SIGNAL')
        v.addWidget(self.view, 1)

    def set_frame(self, obj):
        if isinstance(obj, QPixmap):
            self.view.set_pixmap(obj)
        else:
            self.view.set_text(obj)


class LivePage(QWidget):
    """Shown while teleop runs: status on the left, 3 labelled cameras, RGB / DEPTH switch top right."""

    def __init__(self, hub):
        super().__init__()
        self.hub = hub
        lay = QHBoxLayout(self)
        lay.setContentsMargins(48, 20, 48, 32)
        lay.setSpacing(24)
        side, sv = card('STATUS')
        side.setFixedWidth(330)
        self.state = label('', 'pageTitle', wrap=True)
        self.msg = label('', 'value', wrap=True)
        self.hint = label('', 'subline', wrap=True)
        self.issues = label('', 'value', wrap=True)
        self.issues.setTextFormat(Qt.RichText)
        sv.addWidget(self.state)
        sv.addWidget(self.msg)
        sv.addWidget(self.hint)
        sv.addSpacing(8)
        sv.addWidget(label('PROBLEMS', 'caption'))
        sv.addWidget(self.issues)
        sv.addStretch(1)
        lay.addWidget(side)

        right = QVBoxLayout()
        right.setSpacing(12)
        top = QHBoxLayout()
        top.addStretch(1)
        top.addWidget(label('CAMERA VIEW', 'caption'))
        self.btns = {}
        grp = QButtonGroup(self)
        for mode, text in (('rgb', 'RGB'), ('depth', 'DEPTH')):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setMinimumWidth(96)
            grp.addButton(b)
            b.clicked.connect(lambda _c, m=mode: hub.set_depth(m))
            top.addWidget(b)
            self.btns[mode] = b
        self.btns[hub.depth].setChecked(True)
        hub.depth_changed.connect(lambda m: self.btns[m].setChecked(True))
        right.addLayout(top)
        grid = QGridLayout()
        grid.setSpacing(16)
        self.tiles = {}
        cells = {'zed': (0, 0, 2, 1), 'd405_1': (0, 1, 1, 1), 'd405_2': (1, 1, 1, 1)}
        for title, key, _, _ in CAMERAS:
            t = CameraTile(title)
            grid.addWidget(t, *cells[key])
            self.tiles[key] = t
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        right.addLayout(grid, 1)
        lay.addLayout(right, 1)

    def update_model(self, m):
        state = m.state
        self.state.setText(state.replace('_', ' '))
        self.msg.setText(plain(m.message))
        self.hint.setText(HINTS.get(state, ''))
        dots = {'error': T.SAFE_RED, 'warn': T.SAFE_AMBER, 'info': T.GREY}
        iss = m.issues()
        self.issues.setText('<br>'.join(f'<span style="color:{dots[l]}">&#9679;</span>&nbsp; {t}' for l, t in iss)
                            or f'<span style="color:{T.SAFE_GREEN}">&#9679;</span>&nbsp; none')
        for key, tile in self.tiles.items():
            fps = m.cam_fps(key)
            tile.fps.setText(f'{fps:.0f} FPS' if fps else '')


class OperatorPage(QWidget):
    """Everything Jithin's CONTROL window had, grouped into sections (all buttons Franka Grey)."""
    back = pyqtSignal()
    SECTIONS = ['Testing stage', 'Fix errors', 'Status & logs', 'Robot setup', 'System']

    def __init__(self, model, ros):
        super().__init__()
        self.m, self.ros = model, ros
        self.pending = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(48, 24, 48, 32)
        lay.setSpacing(16)
        top = QHBoxLayout()
        top.addWidget(label('OPERATOR', 'pageTitle'))
        top.addStretch(1)
        b = QPushButton('BACK TO DEMO')
        b.clicked.connect(self.back.emit)
        top.addWidget(b)
        lay.addLayout(top)
        body = QHBoxLayout()
        body.setSpacing(24)
        nav = QVBoxLayout()
        nav.setSpacing(8)
        self.stack = QStackedWidget()
        self.nav = []
        grp = QButtonGroup(self)
        builders = [self.build_testing, self.build_fix, self.build_status, self.build_setup, self.build_system]
        for i, (name, build) in enumerate(zip(self.SECTIONS, builders)):
            btn = QPushButton(name.upper().replace('&', '&&'))
            btn.setCheckable(True)
            btn.setMinimumWidth(200)
            btn.clicked.connect(lambda _c, i=i: self.select(i))
            grp.addButton(btn)
            nav.addWidget(btn)
            self.nav.append(btn)
            self.stack.addWidget(build())
        nav.addStretch(1)
        body.addLayout(nav)
        body.addWidget(self.stack, 1)
        lay.addLayout(body, 1)
        self.select(0)
        for line in model.lines:
            self.log.appendPlainText(line)
        model.log_line.connect(self.log.appendPlainText)
        ros.reply.connect(self.on_reply)

    def select(self, i):
        self.nav[i].setChecked(True)
        self.stack.setCurrentIndex(i)

    def page(self):
        w = QWidget()
        w.setObjectName('plain')
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(16)
        return w, v

    def send(self, cmd):
        self.m.log(f'> {cmd.upper()}')
        self.ros.call(cmd)

    def button(self, text, cmd, name=None, big=False):
        b = QPushButton(text)
        if name:
            b.setObjectName(name)
        if big:
            b.setMinimumHeight(72)
            b.setStyleSheet("font-size:20px;")
        b.clicked.connect(lambda: self.send(cmd))
        return b

    # -- sections
    def build_testing(self):
        w, v = self.page()
        c, cv = card('MANUAL CONTROL', 'same commands as the foot pedals: 1 START · 2 STOP · 3 HOME')
        row = QHBoxLayout()
        row.setSpacing(16)
        self.b_start = self.button('START', 'start', big=True)
        self.b_stop = self.button('STOP', 'stop', name='stop', big=True)
        self.b_home = self.button('HOME', 'home', big=True)
        for b in (self.b_start, self.b_stop, self.b_home):
            row.addWidget(b)
        cv.addLayout(row)
        self.hint = label('', 'subline', wrap=True)
        self.reply = label('', 'mono', wrap=True)
        cv.addWidget(self.hint)
        cv.addWidget(self.reply)
        v.addWidget(c)
        c, cv = card('GRIPPERS', 'only while teleop is stopped')
        self.grip = {}
        for arm in ARMS:
            row = QHBoxLayout()
            row.addWidget(label(arm.upper() + ' ARM', 'caption'), 1)
            go, gc = self.button('OPEN GRIPPER', f'gripper/{arm}/open'), self.button('CLOSE GRIPPER', f'gripper/{arm}/close')
            row.addWidget(go)
            row.addWidget(gc)
            cv.addLayout(row)
            self.grip[arm] = (go, gc)
        v.addWidget(c)
        v.addStretch(1)
        return w

    def build_fix(self):
        w, v = self.page()
        c, cv = card('CURRENT PROBLEMS')
        self.fix_issues = label('', 'value', wrap=True)
        self.fix_issues.setTextFormat(Qt.RichText)
        cv.addWidget(self.fix_issues)
        v.addWidget(c)
        c, cv = card('RECOVERY')
        cv.addWidget(label('After a robot reflex or an E-stop: release the E-stop, then SHUT DOWN and BRING UP. '
                           'ERROR RECOVERY only works while the arms are still running.', 'subline', wrap=True))
        row = QHBoxLayout()
        row.setSpacing(12)
        self.b_recover = self.button('ERROR RECOVERY', 'recover')
        self.b_shutdown2 = self.button('SHUT DOWN', 'shutdown')
        self.b_bringup2 = self.button('BRING UP', 'bringup')
        for b in (self.b_recover, self.b_shutdown2, self.b_bringup2):
            row.addWidget(b)
        row.addStretch(1)
        cv.addLayout(row)
        v.addWidget(c)
        v.addStretch(1)
        return w

    def build_status(self):
        w, v = self.page()
        row = QHBoxLayout()
        row.setSpacing(16)
        self.arm_rows = {}
        for arm in ARMS:
            c, cv = card(f'{arm.upper()} ARM', ARM_IP[arm])
            self.arm_rows[arm] = {}
            g = QGridLayout()
            g.setHorizontalSpacing(18)
            for i, name in enumerate(('Robot', 'Errors', 'Teleop', 'GELLO', 'Gripper', 'Desk')):
                g.addWidget(label(name.upper(), 'caption'), i, 0)
                val = label('-', 'value', wrap=True)
                g.addWidget(val, i, 1)
                self.arm_rows[arm][name] = val
            g.setColumnStretch(1, 1)
            cv.addLayout(g)
            row.addWidget(c, 1)
        v.addLayout(row)
        c, cv = card('SYSTEM')
        lights = QHBoxLayout()
        lights.setSpacing(22)
        self.lights = {}
        for name in ('Desk', 'Robots', 'FCI', 'GELLO', 'Grippers', 'Cameras', 'Pedal'):
            item = QHBoxLayout()
            item.setSpacing(8)
            d = Dot()
            item.addWidget(d)
            item.addWidget(label(name.upper(), 'caption'))
            lights.addLayout(item)
            self.lights[name] = d
        lights.addStretch(1)
        cv.addLayout(lights)
        v.addWidget(c)
        c, cv = card('LOG')
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        cv.addWidget(self.log, 1)
        v.addWidget(c, 1)
        return w

    def build_setup(self):
        w, v = self.page()
        c, cv = card('ROBOT SETUP', 'FRANKA DESK · BOTH ARMS')
        self.toggles = {}
        for key, (title, off, on, *_r) in TOGGLES.items():
            t = Toggle(title, off, on)
            t.requested.connect(lambda want, k=key: self.toggle_request(k, want))
            cv.addWidget(t)
            self.toggles[key] = t
        row = QHBoxLayout()
        self.b_prepare = self.button('PREPARE ALL', 'desk/prepare')
        self.b_park = self.button('PARK', 'desk/park')
        row.addWidget(self.b_prepare)
        row.addWidget(self.b_park)
        row.addStretch(1)
        cv.addLayout(row)
        self.desk_note = label('', 'mono', wrap=True)
        cv.addWidget(self.desk_note)
        v.addWidget(c)
        v.addStretch(1)
        return w

    def build_system(self):
        w, v = self.page()
        c, cv = card('DEMO SYSTEM')
        cv.addWidget(label('BRING UP starts arms, grippers, GELLO and cameras. SHUT DOWN stops them again.',
                           'subline', wrap=True))
        row = QHBoxLayout()
        row.setSpacing(12)
        self.b_bringup = self.button('BRING UP', 'bringup')
        self.b_shutdown = self.button('SHUT DOWN', 'shutdown')
        row.addWidget(self.b_bringup)
        row.addWidget(self.b_shutdown)
        row.addStretch(1)
        cv.addLayout(row)
        self.sys_info = label('', 'mono', wrap=True)
        cv.addWidget(self.sys_info)
        v.addWidget(c)
        v.addStretch(1)
        return w

    # -- logic (from Jithin's CONTROL window)
    def toggle_request(self, key, want):
        _t, _off, _on, cmd_on, cmd_off, _ = TOGGLES[key]
        self.pending[key] = (want, time.monotonic() + 15.0)
        self.update_toggles()
        self.send(cmd_on if want else cmd_off)

    def on_reply(self, name, ok, msg):
        self.reply.setText(f'{name}: {"OK" if ok else "REFUSED"} - {msg}')
        if not ok and name.startswith('desk/'):
            self.pending.clear()
            self.update_toggles()

    def update_toggles(self):
        d = self.m.desk if self.m.desk_ok else {}
        for key, t in self.toggles.items():
            vals = [TOGGLES[key][5](d.get(a, {})) for a in ARMS] if d else [None]
            value = vals[0] if all(x == vals[0] for x in vals) else None
            if key in self.pending and self.pending[key][0] == value:
                del self.pending[key]
            t.set_value(value, key in self.pending)

    def refresh(self):
        m, state = self.m, self.m.state
        conn = m.connected
        idle = conn and state == 'IDLE'
        self.b_start.setEnabled(idle)
        self.b_home.setEnabled(idle)
        self.b_stop.setEnabled(True)                              # STOP is never disabled
        for b in (self.b_bringup, self.b_bringup2):
            b.setEnabled(conn and state == 'OFFLINE')
        for b in (self.b_shutdown, self.b_shutdown2):
            b.setEnabled(conn and state in ('IDLE', 'ERROR', 'STARTING_UP'))
        self.b_recover.setEnabled(conn and state in ('ERROR', 'IDLE'))
        grip_ok = conn and state not in ('TELEOP', 'TELEOP_STARTING', 'OFFLINE')
        for pair in self.grip.values():
            for b in pair:
                b.setEnabled(grip_ok)
        desk_free = m.desk_ok and not m.desk.get('busy') and state == 'OFFLINE'
        for wd in (*self.toggles.values(), self.b_prepare, self.b_park):
            if wd.isEnabled() != desk_free:
                wd.setEnabled(desk_free)
        self.desk_note.setText('Available while the demo is shut down.' if state != 'OFFLINE' or not m.desk_ok
                               else 'Desk busy ...' if m.desk.get('busy') else 'Switches act on both arms.')
        if any(time.monotonic() > dl for _, dl in self.pending.values()):
            self.pending = {k: p for k, p in self.pending.items() if time.monotonic() <= p[1]}
        self.update_toggles()
        self.hint.setText(HINTS.get(state, ''))
        dots = {'error': T.SAFE_RED, 'warn': T.SAFE_AMBER, 'info': T.GREY}
        iss = m.issues()
        self.fix_issues.setText('<br>'.join(f'<span style="color:{dots[l]}">&#9679;</span>&nbsp; {t}' for l, t in iss)
                                or f'<span style="color:{T.SAFE_GREEN}">&#9679;</span>&nbsp; no problems')
        for arm in ARMS:
            a, rows = m.arm(arm), self.arm_rows[arm]
            rows['Robot'].setText(a.get('robot_mode', '-'))
            errs = a.get('errors', [])
            rows['Errors'].setText(', '.join(plain(e) for e in errs) if errs else 'none')
            rows['Errors'].setStyleSheet(f"background:transparent; color:{T.SAFE_RED if errs else T.pal()['fg']};")
            rows['Teleop'].setText(a.get('gate', '-'))
            rows['GELLO'].setText(a.get('gello', '-'))
            rows['Gripper'].setText(a.get('gripper', '-'))
            d = m.desk.get(arm, {}) if m.desk_ok else {}
            rows['Desk'].setText(d['error'] if d.get('error') else
                                 f'{d.get("mode", "?")} · brakes {"open" if d.get("brakes") == "unlocked" else d.get("brakes", "?")}'
                                 f' · FCI {"on" if d.get("fci") else "off"}' if d else 'no desk_agent')
        for name, (level, tip) in m.lights().items():
            self.lights[name].set_level(level)
            self.lights[name].setToolTip(tip)
        self.sys_info.setText(f'state {state}   ·   pedal {m.s.get("pedal", "-")}   ·   logs {m.s.get("logdir", "-")}')


# ---------------------------------------------------------------------------------------------- window
class MainWindow(ConfirmClose, QWidget):
    HOME, LIVE, OPERATOR = 0, 1, 2

    def __init__(self, model, ros, hub, decoder, preview=False):
        super().__init__()
        self.m, self.hub, self.decoder = model, hub, decoder
        self.was_teleop = False
        self.setWindowTitle('Robot demo' + (' (PREVIEW)' if preview else ''))
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # navbar
        nav = QFrame()
        nav.setObjectName('navbar')
        nl = QHBoxLayout(nav)
        nl.setContentsMargins(48, 20, 48, 20)         # brand: >= 48 px from the side
        nl.setSpacing(12)
        self.logo = QLabel()
        self.logo.setObjectName('plain')
        nl.addWidget(self.logo, 0, Qt.AlignVCenter)
        nl.addStretch(1)
        self.chip_left, self.chip_right = StatusChip('Left arm'), StatusChip('Right arm')
        self.chip_cams = StatusChip('Cameras')
        for c in (self.chip_left, self.chip_right, self.chip_cams):
            nl.addWidget(c)
        nl.addSpacing(12)
        self.b_operator = QPushButton('OPERATOR  ▾')
        self.b_operator.setObjectName('navbtn')
        self.b_operator.setFixedHeight(40)
        menu = QMenu(self.b_operator)
        for i, name in enumerate(OperatorPage.SECTIONS):
            menu.addAction(name, lambda i=i: self.show_operator(i))
        menu.addSeparator()
        menu.addAction('Back to demo screen', self.go_auto)
        self.b_operator.setMenu(menu)
        self.menu = menu
        self.b_theme = QPushButton()
        self.b_theme.setObjectName('navbtn')
        self.b_theme.setFixedHeight(40)
        self.b_theme.clicked.connect(lambda: hub.set_theme('dark' if hub.theme == 'light' else 'light'))
        nl.addWidget(self.b_operator)
        nl.addWidget(self.b_theme)
        outer.addWidget(nav)

        # problem bar (Home / Operator; the Live page shows problems on its left side)
        self.errbar = QFrame()
        self.errbar.setObjectName('errbar')
        el = QHBoxLayout(self.errbar)
        el.setContentsMargins(48, 10, 48, 10)
        self.err_dot = Dot('bad', 14)
        self.err_text = QLabel()
        self.err_text.setObjectName('plain')
        self.err_text.setWordWrap(True)
        el.addWidget(self.err_dot)
        el.addWidget(self.err_text, 1)
        outer.addWidget(self.errbar)

        # pages
        self.stack = QStackedWidget()
        self.home, self.live = HomePage(), LivePage(hub)
        self.operator = OperatorPage(model, ros)
        self.operator.back.connect(self.go_auto)
        for pg in (self.home, self.live, self.operator):
            self.stack.addWidget(pg)
        outer.addWidget(self.stack, 1)

        decoder.frame.connect(lambda k, o: self.live.tiles[k].set_frame(o))
        model.updated.connect(self.update_all)
        hub.theme_changed.connect(self.apply_theme)
        self.apply_theme(hub.theme)

    # -- navigation
    def show_page(self, idx):
        self.stack.setCurrentIndex(idx)
        self.decoder.set_visible(self, idx == self.LIVE)
        self.update_all()

    def show_operator(self, i):
        self.operator.select(i)
        self.show_page(self.OPERATOR)

    def go_auto(self):
        self.show_page(self.LIVE if self.m.state in TELEOP_STATES else self.HOME)

    # -- look
    def apply_theme(self, mode):
        self.setStyleSheet(T.stylesheet(mode))
        pm, w, h = T.logo_pixmap(mode, 220)
        self.logo.setPixmap(pm.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo.setFixedSize(w, h)
        self.b_theme.setText('☾  DARK' if mode == 'light' else '☀  LIGHT')
        c = T.pal()
        self.errbar.setStyleSheet(f"QFrame#errbar {{background:{c['panel']}; border-bottom:1px solid {c['line']};}}")
        for chip in (self.chip_left, self.chip_right, self.chip_cams):
            chip.restyle()
        self.home.img.update()
        for tile in self.live.tiles.values():
            tile.view.update()
        self.update_all()

    def update_all(self):
        m, state = self.m, self.m.state
        # arm + camera chips
        for chip, arm in ((self.chip_left, 'left'), (self.chip_right, 'right')):
            a = m.arm(arm)
            rows = [(m.arm_level(arm), f'Robot: {a.get("robot_mode", "not connected")}')]
            for e in a.get('errors') or []:
                rows.append(('bad', plain(e)))
            rows.append(('ok' if a.get('gripper') == 'ready' else 'bad', f'Gripper: {a.get("gripper", "-")}'))
            rows.append(('ok' if a.get('gello') in ('in holder', 'out of holder') else 'bad',
                         f'GELLO: {a.get("gello", "-")}'))
            chip.set_state(m.arm_level(arm), rows)
        rows = [(m.cam_level(k), f'{t}   {m.cam_fps(k):.0f} fps' if m.cam_fps(k) else f'{t}   not connected')
                for k, t in CAM_TITLE.items()]
        self.chip_cams.set_state('ok' if all(r[0] == 'ok' for r in rows) else 'bad', rows)
        # automatic page change from the supervisor state (pedals and buttons behave the same)
        teleop = state in TELEOP_STATES
        if teleop != self.was_teleop:
            self.was_teleop = teleop
            if teleop:
                self.show_page(self.LIVE)
                return
            if self.stack.currentIndex() == self.LIVE:
                self.show_page(self.HOME)
                return
        # problem bar
        iss = m.issues()
        idx = self.stack.currentIndex()
        if iss and idx != self.LIVE:
            level = iss[0][0]
            self.err_dot.set_level({'error': 'bad', 'warn': 'warn', 'info': 'off'}[level])
            self.err_text.setText('   ·   '.join(t for _l, t in iss[:2]))
            self.err_text.setStyleSheet(f"background:transparent; color:{T.pal()['fg']}; font-weight:600; font-size:14px;")
            self.errbar.show()
        else:
            self.errbar.hide()
        if idx == self.HOME:
            self.home.update_state(state)
        elif idx == self.LIVE:
            self.live.update_model(m)
        else:
            self.operator.refresh()


# ---------------------------------------------------------------------------------------------- preview
class PreviewFeeder(QObject):
    """--preview: plays the demo states with fake data so the design can be seen. Sends nothing anywhere."""

    def __init__(self):
        super().__init__()
        self.ros = None
        self.state, self.msg = 'OFFLINE', 'not brought up'
        self.arm = {a: {'mode': 'Programming', 'brakes': 'locked', 'fci': False, 'control': 'none', 'error': ''}
                    for a in ARMS}
        self.last = ''
        self.timers = []
        self.drop_cam = False
        self.frame_imgs = {}

    def attach(self, ros):
        self.ros = ros
        t = QTimer(self)
        t.timeout.connect(self.tick)
        t.start(500)
        f = QTimer(self)
        f.timeout.connect(self.frames)
        f.start(33)
        self.timers += [t, f]
        self.tick()

    def later(self, s, fn):
        QTimer.singleShot(int(s * 1000), fn)

    def set(self, state, msg):
        self.state, self.msg = state, msg

    def command(self, name):
        if name == 'start' and self.state == 'IDLE':
            self.set('TELEOP_STARTING', 'left: WAIT_STILL: hold GELLO still; right: WAIT_STILL: hold GELLO still')
            self.later(3, lambda: self.set('TELEOP', 'both arms LIVE'))
        elif name == 'stop' and self.state in ('TELEOP', 'TELEOP_STARTING', 'HOMING'):
            self.set('STOPPING', 'deactivating controllers (robots hold)')
            self.later(1, lambda: self.set('IDLE', 'teleop ended: left: stopped - STOPPED; right: stopped - STOPPED'))
        elif name == 'home' and self.state == 'IDLE':
            self.set('HOMING', 'left: MOVING: ramping to target pose; right: MOVING: ramping to target pose')
            self.later(3, lambda: self.set('IDLE', 'both arms at home'))
        elif name == 'bringup' and self.state == 'OFFLINE':
            self.set('STARTING_UP', 'waiting for: left gripper, right gripper')
            self.later(3, lambda: self.set('IDLE', 'ready'))
        elif name == 'shutdown':
            self.set('SHUTTING_DOWN', 'stopping cameras, GELLO, grippers, arms')
            self.later(2, lambda: self.set('OFFLINE', 'shut down'))
        elif name == 'recover':
            self.later(1, lambda: self.set('IDLE', 'recovered'))
        elif name.startswith('desk/'):
            step = name.split('/')[1]
            change = {'unlock': {'brakes': 'unlocked'}, 'lock': {'brakes': 'locked'},
                      'execution': {'mode': 'Execution'}, 'programming': {'mode': 'Programming'},
                      'fci_on': {'fci': True}, 'fci_off': {'fci': False},
                      'prepare': {'brakes': 'unlocked', 'mode': 'Execution', 'fci': True},
                      'park': {'brakes': 'locked', 'mode': 'Programming', 'fci': False, 'control': 'none'}}[step]

            def apply():
                for a in ARMS:
                    self.arm[a].update(change)
                    if step != 'park':
                        self.arm[a]['control'] = 'demo'
                self.last = f'{time.strftime("%H:%M:%S")} {step.upper()} done'
            self.later(1.2, apply)

    def pedal(self, name):
        """Preview only: pretend a foot pedal was pressed."""
        self.command(name)

    def error(self):
        self.set('ERROR', 'right: motion aborted by robot reflex (cartesian_reflex) - robots stopped by the Franka. '
                          'Check the robots (release E-stop), then SHUTDOWN and BRINGUP')

    def toggle_cam(self):
        self.drop_cam = not self.drop_cam

    def tick(self):
        up = self.state not in ('OFFLINE', 'SHUTTING_DOWN')
        live = self.state in ('TELEOP', 'TELEOP_STARTING', 'HOMING')
        arms = {a: {'robot_mode': ('MOVE' if live else 'IDLE') if up and self.state != 'ERROR' else 'no data',
                    'errors': [], 'connected': up and self.state != 'ERROR',
                    'gate': {'TELEOP': 'LIVE', 'TELEOP_STARTING': 'WAIT_STILL: hold GELLO still',
                             'HOMING': 'MOVING'}.get(self.state, 'not running'),
                    'gello': ('out of holder' if live else 'in holder') if up else 'no data',
                    'gripper': 'ready' if up else 'off'} for a in ARMS}
        cams = {'zed': 29.8, 'd405_1': 30.0, 'd405_2': 29.9} if up else {}
        if self.drop_cam:
            cams.pop('d405_2', None)
        st = {'state': self.state, 'message': self.msg, 'arms': arms, 'pedal': 'connected',
              'cameras_launch': 'running' if up else 'off', 'cameras': cams}
        self.ros.status.emit(st)
        self.ros.desk.emit({**{a: dict(v) for a, v in self.arm.items()}, 'busy': '', 'last': self.last})

    def frames(self):
        if self.state in ('OFFLINE', 'SHUTTING_DOWN'):
            return
        now = time.monotonic()
        with self.ros.lock:
            for title, key, _, _ in CAMERAS:
                if self.drop_cam and key == 'd405_2':
                    continue
                if key not in self.frame_imgs:
                    self.frame_imgs[key] = self.fake_jpeg(title)
                self.ros.frames[(key, 'rgb')] = self.frame_imgs[key]
                self.ros.frame_t[(key, 'rgb')] = now

    @staticmethod
    def fake_jpeg(title):
        img = QImage(960, 540, QImage.Format_RGB32)
        p = QPainter(img)
        g = QLinearGradient(0, 0, 960, 540)
        g.setColorAt(0, QColor(T.GREY))
        g.setColorAt(1, QColor(T.DARK_BLUE))
        p.fillRect(img.rect(), g)
        p.setPen(QColor(T.OFF_WHITE))
        f = QFont(T.HEAD, 30)
        f.setWeight(QFont.DemiBold)
        p.setFont(f)
        p.drawText(img.rect(), Qt.AlignCenter, f'{title.upper()}\nCAMERA PREVIEW')
        p.end()
        buf = QBuffer()
        buf.open(QIODevice.WriteOnly)
        img.save(buf, 'JPG', 85)
        return bytes(buf.data())


def place(widget, screen):
    g = screen.availableGeometry()
    widget.setGeometry(g)
    widget.move(g.topLeft())
    widget.showMaximized()


def main():
    global BLINK
    ap = argparse.ArgumentParser()
    ap.add_argument('--single', action='store_true', help='one window only (default with a single monitor)')
    ap.add_argument('--control-screen', type=int, default=0)
    ap.add_argument('--camera-screen', type=int, default=1)
    ap.add_argument('--owns-demo', action='store_true',
                    help='started by start_demo.sh: closing a window asks, then shuts the demo down')
    ap.add_argument('--preview', action='store_true',
                    help='design preview with demo data; never sends anything (own ROS domain 199)')
    ap.add_argument('--screenshot', metavar='DIR', help=argparse.SUPPRESS)       # offscreen design check
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if a.preview:
        if a.owns_demo:
            sys.exit('--preview cannot own the demo')
        os.environ['ROS_DOMAIN_ID'] = '199'            # isolated from the real demo (domain 0)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv)
    T.load_fonts()
    app.setFont(QFont(T.TEXT, 11))
    BLINK = Blinker()
    ConfirmClose.owns_demo = a.owns_demo
    app.setQuitOnLastWindowClosed(not a.owns_demo)
    feeder = PreviewFeeder() if a.preview else None
    ros = Ros(preview=feeder)
    hub, model = Hub(), Model(ros)
    decoder = Decoder(ros, hub)
    screens = app.screens()
    two = not a.single and len(screens) >= 2
    windows = [MainWindow(model, ros, hub, decoder, a.preview) for _ in range(2 if two else 1)]
    if feeder:
        feeder.attach(ros)
        for w in windows:
            QShortcut(Qt.Key_F5, w, activated=lambda: feeder.pedal('start'))
            QShortcut(Qt.Key_F6, w, activated=lambda: feeder.pedal('stop'))
            QShortcut(Qt.Key_F7, w, activated=lambda: feeder.pedal('home'))
            QShortcut(Qt.Key_F9, w, activated=feeder.error)
            QShortcut(Qt.Key_F10, w, activated=feeder.toggle_cam)
    place(windows[0], screens[min(a.control_screen, len(screens) - 1)])
    if two:
        place(windows[1], screens[min(a.camera_screen, len(screens) - 1)])
    if a.screenshot:
        run_screenshots(app, a.screenshot, windows[0], hub, feeder)
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    tick = QTimer()                      # lets Python handle Ctrl-C while Qt runs
    tick.timeout.connect(lambda: None)
    tick.start(200)
    rc = app.exec_()
    ros.shutdown()
    rclpy.try_shutdown()
    sys.exit(rc)


def run_screenshots(app, outdir, win, hub, feeder):
    """Developer aid: walk through the pages in --preview and save PNGs (works with QT_QPA_PLATFORM=offscreen)."""
    os.makedirs(outdir, exist_ok=True)
    win.resize(1920, 1080)
    steps = []

    def shot(name):
        win.grab().save(os.path.join(outdir, name + '.png'))

    def at(t, fn):
        steps.append(t)
        QTimer.singleShot(int(t * 1000), fn)

    at(1.0, lambda: shot('1_home_offline_light'))
    at(1.2, lambda: feeder.command('bringup'))
    at(5.0, lambda: shot('2_home_ready_light'))
    at(5.2, lambda: hub.set_theme('dark'))
    at(5.8, lambda: shot('3_home_ready_dark'))
    at(6.0, lambda: hub.set_theme('light'))
    at(6.2, lambda: feeder.pedal('start'))
    at(10.0, lambda: shot('4_live_light'))
    at(10.2, lambda: hub.set_depth('depth'))
    at(10.8, lambda: shot('5_live_depth_light'))
    at(11.0, lambda: (hub.set_depth('rgb'), feeder.toggle_cam(), hub.set_theme('dark')))
    at(12.0, lambda: shot('6_live_camdrop_dark'))
    at(12.2, lambda: (feeder.toggle_cam(), feeder.error()))
    at(13.2, lambda: shot('7_live_error_dark'))
    at(13.4, lambda: (hub.set_theme('light'), win.show_operator(0)))
    at(14.4, lambda: shot('8_operator_testing'))
    at(14.6, lambda: win.show_operator(1))
    at(15.2, lambda: shot('9_operator_fix'))
    at(15.4, lambda: win.show_operator(2))
    at(16.0, lambda: shot('10_operator_status'))
    at(16.2, lambda: win.show_operator(3))
    at(16.8, lambda: shot('11_operator_setup'))
    at(17.5, app.quit)


if __name__ == '__main__':
    main()
