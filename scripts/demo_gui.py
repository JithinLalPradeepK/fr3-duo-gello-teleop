#!/usr/bin/env python3
"""Robot demo GUI - a thin client of demo_supervisor and desk_agent (same services as the foot pedals).

  source ~/robot_demo/env/host_env.sh
  python3 scripts/demo_gui.py [--single] [--control-screen 0] [--camera-screen 1]
  python3 scripts/demo_gui.py --preview      design preview with demo data: sends NOTHING
      (own ROS domain 199, localhost only). Preview keys: F5 = pedal START, F6 = STOP, F7 = HOME,
      F9 = demo ERROR, F10 = drop a camera, F11 = depth available / not

TWO DISPLAYS (2026-11-21: the Home page was REMOVED, the cameras are the base page of both displays):
display 1 (--control-screen, default 0) = camera page, ROBO STATUS closed (cameras fill the screen), plus the Operator page;
display 2 (--camera-screen, default 1) = camera page with the ROBO STATUS list OPEN on the left. One monitor / --single:
one window = display 1. The STATUS button opens / closes the list on either display.
Redesign (branch features/GUI-development, see docs/GUI_CHANGES.md):
  LIVE      camera page (the page both displays start on), layout from the design mockup assets/brand/Home@1x(2).png, no
            navbar: header bar (logo, title, CAMERA VIEW / OPERATOR MODE tabs, STATUS / theme / full screen), ROBO STATUS list
            (slides in / out with the STATUS button), 2 x 2 grid: ZED RGB | ZED DEPTH / left wrist | right wrist (wrists carry the
            DEPTH inset).
  OPERATOR  navbar button (no drop-down; also on the camera display, it opens on the main display): ALL sections at once in a grid (Testing stage, Fix errors, Robot setup,
            Status, System, Log) = all of Jithin's old buttons.
  NAVBAR    logo (black/white with the mode), LEFT / RIGHT arm chips, CAMERAS chip (click = the 3 cameras),
            OPERATOR MODE, light / dark switch (light default), FULL SCREEN <-> window (starts full screen). On the
            camera page the chips are replaced by the ROBO STATUS list; the controls sit in the page header.
  NO CAMERA  pulsing Franka trademark in a spinning ring (loading animation) in that camera's tile.
The GUI contains no demo logic: every button is a /demo/* service call; demo_supervisor / desk_agent decide.
THE GUI IS NOT A SAFETY DEVICE - the physical E-stop stays in use.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import (QBuffer, QByteArray, QEasingCurve, QEvent, QIODevice, QObject, QPoint, QPointF, QPropertyAnimation, QRectF,
                          QSize, Qt, QTimer, QVariantAnimation, pyqtProperty, pyqtSignal)
from PyQt5.QtGui import (QColor, QFont, QImage, QImageReader, QLinearGradient, QPainter, QPainterPath, QPen,
                         QPixmap)
from PyQt5.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel,
                             QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QShortcut, QSizePolicy, QStackedWidget,
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
# >>> WRISTS: d405_1 = LEFT wrist, d405_2 = RIGHT wrist is an ASSUMPTION - swap the two titles if it is the other way.
# DEPTH topics (2026-11-21): colourised JPEG (near = red, far = blue) made by scripts/depth_view.py, started by
# launch/cameras.launch.py - one per camera, reading the raw depth only while the GUI is subscribed.
CAMERAS = [('ZED Mini · Overview', 'zed', '/zed/zed_node/rgb/color/rect/image/compressed',
            '/zed/zed_node/depth_view/compressed'),
           ('Left wrist · D405 #1', 'd405_1', '/camera/d405_1/color/image_raw/compressed',
            '/camera/d405_1/depth_view/compressed'),
           ('Right wrist · D405 #2', 'd405_2', '/camera/d405_2/color/image_raw/compressed',
            '/camera/d405_2/depth_view/compressed')]
# Cameras whose picture is shown rotated by 180 degrees (both D405 wrist cameras are mounted upside down). Applies to the
# RGB and the DEPTH picture of that camera. Change with --invert (e.g. --invert d405_1,d405_2,zed or --invert "").
INVERT = {'d405_1', 'd405_2'}
CAM_TITLE = {key: title for title, key, _, _ in CAMERAS}
TELEOP_STATES = ('TELEOP_STARTING', 'TELEOP')
HINTS = {'STARTING_UP': 'Starting arms, grippers, GELLO and cameras ...',
         'IDLE': 'Press START, then lift both GELLOs out of their holders.',
         'TELEOP_STARTING': 'Hold the GELLOs still - the robots move to the GELLO pose.',
         'TELEOP': 'The robots follow the GELLOs.  STOP ends teleoperation.',
         'STOPPING': 'The robots hold their pose.',
         'HOMING': 'Both arms move to the home pose.',
         'ERROR': 'Open Operator > Fix errors.',
         'RECOVERING': 'Error recovery: restarting the arm control or clearing the robot error ...',
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
         'joint_velocity_violation': 'a joint moved too fast for its position (close to its limit)',
         'self-collision avoidance': 'the arm came too close to itself',
         'arm process died': 'the arm control program stopped',
         'REFLEX': 'robot stopped itself (reflex)', 'USER_STOPPED': 'emergency stop pressed'}
NO_SIGNAL = 'WAITING FOR CAMERA ...'
NO_DEPTH = 'WAITING FOR DEPTH ...'
# names on the camera tiles / status list (design spec "Teleop Control Station")
TILE_NAME = {'zed': 'Head Cam – RGB', 'd405_1': 'Hand Cam – Left', 'd405_2': 'Hand Cam – Right'}
INSET_NAME = {'zed': 'Head Depth', 'd405_1': 'Left Depth', 'd405_2': 'Right Depth'}
# status variants (text colour, tint): ok / warn / off - the only three states of the design
VARIANTS = {'light': {'ok': ('#117A3B', '#E4F5EA'), 'warn': ('#A35A00', '#FCEFD9'), 'off': ('#C62828', '#FDE8E8')},
            'dark': {'ok': ('#6FD39A', '#1D3B2C'), 'warn': ('#F2B54A', '#3D3016'), 'off': ('#FF8A8A', '#402222')}}


def variant_colors(v):
    return VARIANTS[T.MODE].get(v, VARIANTS[T.MODE]['off'])
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
    depth_changed = pyqtSignal(str)       # which view is the MAIN picture: 'rgb' | 'depth'
    pip_changed = pyqtSignal(bool)        # small inset with the other view (picture-in-picture)

    def __init__(self):
        super().__init__()
        self.theme, self.depth, self.pip = 'light', 'rgb', True

    def set_pip(self, on):
        self.pip = bool(on)
        self.pip_changed.emit(self.pip)

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
    clicked = pyqtSignal()

    def __init__(self, rounded=True, framed=True, icon=False, clickable=False):
        super().__init__()
        self.pm, self.text, self.rounded, self.framed, self.icon = None, '', rounded, framed, icon
        self.loading, self.phase, self.badge, self.badge_r = False, 0.0, '', ''
        self.badge_variant, self.white_border, self.fill = 'ok', False, False   # fill: crop to fill (insets)
        self.valign = 'center'                   # 'bottom': picture sits on the bottom edge (FR3 Duo in the sidebar)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setMinimumSize(120, 90)
        if clickable:
            self.setCursor(Qt.PointingHandCursor)
        self.clickable = clickable
        self.anim = QTimer(self)                 # loading animation, runs only while waiting for a camera
        self.anim.timeout.connect(self._animate)

    def sizeHint(self):
        return QSize(640, 360)

    def mousePressEvent(self, e):
        if self.clickable and e.button() == Qt.LeftButton:
            self.clicked.emit()

    def _animate(self):
        self.phase = (self.phase + 6.0) % 360.0
        if self.isVisible():
            self.update()

    def set_pixmap(self, pm):
        self.pm, self.text, self.loading = pm, '', False
        self.anim.stop()
        self.update()

    def set_text(self, text, loading=False):
        self.pm, self.text, self.loading = None, text, loading
        if loading and not self.anim.isActive():
            self.anim.start(33)
        elif not loading:
            self.anim.stop()
        self.update()

    def paintEvent(self, _e):
        c = T.pal()
        p = QPainter(self)
        p.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        r = QRectF(self.rect())
        small = self.width() < 330
        radius = 10 if small else 12
        if self.white_border:                    # picture-in-picture inset: 3 px white frame
            outer = QPainterPath()
            outer.addRoundedRect(r, radius, radius)
            p.fillPath(outer, QColor(T.WHITE))
            r = r.adjusted(3, 3, -3, -3)
            radius -= 3
        if self.framed:
            path = QPainterPath()
            path.addRoundedRect(r, radius, radius)
            p.fillPath(path, QColor(c['camera_bg']))
            p.setClipPath(path)
        if self.pm and not self.pm.isNull():
            sz = self.pm.size().scaled(r.size().toSize(), Qt.KeepAspectRatioByExpanding if self.fill else Qt.KeepAspectRatio)
            y = r.bottom() - sz.height() if self.valign == 'bottom' else r.top() + (r.height() - sz.height()) / 2
            p.drawPixmap(QRectF(r.left() + (r.width() - sz.width()) / 2, y, sz.width(), sz.height()), self.pm,
                         QRectF(self.pm.rect()))
        elif self.text:
            f = QFont(T.HEAD, max(7, min(12, int(r.width() / 26))))
            f.setWeight(QFont.DemiBold)
            p.setFont(f)
            p.setPen(QColor(c['muted']))
            if self.icon:                       # Franka trademark where the camera picture is missing
                ic = T.icon_pixmap(T.MODE)
                compact = r.height() < 120      # small inset: icon only, no text
                side = max(30, min(160, int(min(r.width(), r.height()) * (0.46 if compact else 0.36))))
                box = QRectF(r.left() + (r.width() - side) / 2,
                             r.top() + r.height() / 2 - side * (0.5 if compact else 0.75), side, side)
                p.setOpacity(0.9)
                if self.loading:                # pulsing trademark inside a spinning ring
                    p.setOpacity(0.55 + 0.4 * (0.5 + 0.5 * math.sin(math.radians(self.phase * 1.5))))
                p.drawPixmap(box, ic, QRectF(ic.rect()))
                p.setOpacity(1.0)
                if self.loading:
                    ring = box.adjusted(-side * 0.18, -side * 0.18, side * 0.18, side * 0.18)
                    base = QColor(c['muted'])
                    base.setAlphaF(0.25)
                    p.setBrush(Qt.NoBrush)
                    p.setPen(QPen(base, max(2, side / 28)))
                    p.drawEllipse(ring)
                    pen = QPen(QColor(T.GREY), max(2, side / 28))
                    pen.setCapStyle(Qt.RoundCap)
                    p.setPen(pen)
                    p.drawArc(ring, int(-self.phase * 16), 100 * 16)
                if not compact:
                    area = QRectF(r.left(), r.top() + r.height() / 2 + side * 0.5, r.width(), r.height())
                    p.drawText(area, Qt.AlignHCenter | Qt.AlignTop | Qt.TextWordWrap, self.text)
            else:
                p.drawText(r, Qt.AlignCenter | Qt.TextWordWrap, self.text)
        # ---- caption chips (design spec 5.8): name chip top-left, status chip top-right
        if self.badge:
            f = QFont(T.HEAD, 8 if small else 11)
            f.setWeight(QFont.DemiBold)
            p.setFont(f)
            fm = p.fontMetrics()
            w = fm.horizontalAdvance(self.badge) + (16 if small else 28)
            h = fm.height() + (8 if small else 16)
            m = 8 if small else 14
            box = QRectF(r.left() + m, r.top() + m, w, h)
            p.setPen(QPen(QColor(c['line']), 1))
            p.setBrush(QColor(255, 255, 255, 235))
            p.drawRoundedRect(box, 6 if small else 8, 6 if small else 8)
            p.setPen(QColor('#1B2F36'))
            p.drawText(box, Qt.AlignCenter, self.badge)
        if self.badge_r:
            fg, bg = variant_colors(self.badge_variant)
            f = QFont(T.MONO, 8 if small else 9)
            f.setWeight(QFont.Bold)
            p.setFont(f)
            fm = p.fontMetrics()
            w = fm.horizontalAdvance(self.badge_r) + 44
            h = fm.height() + 14
            m = 8 if small else 14
            box = QRectF(r.right() - w - m, r.top() + m, w, h)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(bg))
            p.drawRoundedRect(box, h / 2, h / 2)
            ring = QColor(fg)
            ring.setAlphaF(0.2)
            p.setBrush(ring)
            p.drawEllipse(QPointF(box.left() + 17, box.center().y()), 7, 7)
            p.setBrush(QColor(fg))
            p.drawEllipse(QPointF(box.left() + 17, box.center().y()), 4, 4)
            p.setPen(QColor(fg))
            p.drawText(QRectF(box.left() + 28, box.top(), w - 36, h), Qt.AlignVCenter | Qt.AlignLeft, self.badge_r)


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
    """Decodes the camera streams ONCE (only while a LIVE page is visible) and hands the pictures to every window.
    Both views are decoded while picture-in-picture is on (main view + inset)."""
    frame = pyqtSignal(str, str, object)    # camera key, view ('rgb'|'depth'), QPixmap or a text to show

    def __init__(self, ros, hub):
        super().__init__()
        self.ros, self.hub = ros, hub
        self.visible, self.shown = {}, {}
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        hub.depth_changed.connect(lambda _m: self.shown.clear())
        hub.pip_changed.connect(lambda _b: self.shown.clear())

    def set_visible(self, win, on):
        self.visible[win] = on
        if on:
            self.shown.clear()                # a window just opened the page: send the latest pictures again

    def views(self):
        main = self.hub.depth
        return [main] + ([('rgb' if main == 'depth' else 'depth')] if self.hub.pip else [])

    def tick(self):
        if not any(self.visible.values()):
            return
        now = time.monotonic()
        for _title, key, _rgb, depth in CAMERAS:
            for mode in self.views():
                sk = (key, mode)
                with self.ros.lock:
                    data, t = self.ros.frames.get((key, mode)), self.ros.frame_t.get((key, mode), 0.0)
                if data is None and mode == 'depth' and depth is None:       # no depth topic set yet
                    if self.shown.get(sk) != ('na',):
                        self.shown[sk] = ('na',)
                        self.frame.emit(key, mode, NO_DEPTH)
                    continue
                if data is None or now - t > 1.0:
                    if self.shown.get(sk) != ('none',):
                        self.shown[sk] = ('none',)
                        self.frame.emit(key, mode, NO_SIGNAL)
                    continue
                sh = self.shown.get(sk)
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
                if key in INVERT:
                    img = img.mirrored(True, True)         # horizontal + vertical mirror = rotated by 180 degrees
                self.shown[sk] = ('img', data)
                self.frame.emit(key, mode, QPixmap.fromImage(img))


# ---------------------------------------------------------------------------------------------- pages
class PipView(ImageView):
    """Main camera picture with the depth inset (spec 5.9: 32 % of the tile, 16:9, no frame, 14 px from the
    bottom-right corner)."""

    def __init__(self):
        super().__init__(icon=True)
        self.inset = ImageView(icon=True)
        self.inset.white_border = False          # no white frame around the depth inset
        self.inset.fill = True                   # picture fills the inset to its edge (no hairline)
        self.inset.setParent(self)
        self.inset.setMinimumSize(60, 34)

    def resizeEvent(self, e):
        w = max(90, int(self.width() * 0.32))
        h = int(w * 9 / 16)                      # 16:9 like the camera pictures: no bars around the picture
        m = 14
        self.inset.setGeometry(self.width() - w - m, self.height() - h - m, w, h)
        super().resizeEvent(e)


class CameraTile(QWidget):
    """Feed tile: name chip top-left, status chip top-right, picture (16:9); hand cameras carry the depth inset."""

    def __init__(self, title, key, hub, depth_only=False, pip=True):
        super().__init__()
        self.key, self.hub, self.depth_only, self.pip = key, hub, depth_only, pip     # pip=False: no depth inset
        self.name = 'Head Cam – Depth' if depth_only else TILE_NAME.get(key, title)
        self.objs = {'rgb': NO_SIGNAL, 'depth': NO_SIGNAL}
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        self.view = PipView()
        v.addWidget(self.view, 1)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        hub.depth_changed.connect(lambda _m: self.render())
        hub.pip_changed.connect(lambda _b: self.render())
        self.view.inset.badge = INSET_NAME.get(key, 'Depth')
        self.render()

    def height_for(self, w):
        return int(w * 9 / 16)

    def set_frame(self, mode, obj):
        self.objs[mode] = obj
        self.render()

    def set_status(self, variant, text):
        self.view.badge_variant, self.view.badge_r = variant, text
        self.view.update()

    @staticmethod
    def show(view, obj):
        if isinstance(obj, QPixmap):
            view.set_pixmap(obj)
        else:
            view.set_text(obj, loading=(obj in (NO_SIGNAL, NO_DEPTH)))

    def render(self):
        self.view.badge = self.name
        if self.depth_only:                   # separate DEPTH tile (head camera), no inset
            self.show(self.view, self.objs['depth'])
            self.view.inset.setVisible(False)
            self.view.update()
            return
        main = self.hub.depth
        other = 'rgb' if main == 'depth' else 'depth'
        self.show(self.view, self.objs[main])
        # inset only while there is a picture to show (a missing camera shows just the loading animation)
        show_inset = self.pip and self.hub.pip and (isinstance(self.objs[main], QPixmap) or isinstance(self.objs[other], QPixmap))
        self.view.inset.setVisible(show_inset)
        if show_inset:
            self.show(self.view.inset, self.objs[other])
        self.view.update()

    def refresh(self):
        self.view.update()
        self.view.inset.update()


class StatusPill(QWidget):
    """Status pill (spec 5.6): exactly 128 x 28, tinted, dot with halo + uppercase mono label."""

    def __init__(self):
        super().__init__()
        self.setFixedSize(128, 28)
        self.variant, self.label = 'off', ''

    def set(self, variant, label):
        self.variant, self.label = variant, label
        self.update()

    def paintEvent(self, _e):
        fg, bg = variant_colors(self.variant)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(bg))
        p.drawRoundedRect(QRectF(self.rect()), 14, 14)
        f = QFont(T.MONO, 9)
        f.setWeight(QFont.Bold)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 0.4)
        p.setFont(f)
        tw = p.fontMetrics().horizontalAdvance(self.label)
        x = (self.width() - (8 + 8 + tw)) / 2
        halo = QColor(fg)
        halo.setAlphaF(0.2)
        p.setBrush(halo)
        p.drawEllipse(QPointF(x + 4, 14), 7, 7)
        p.setBrush(QColor(fg))
        p.drawEllipse(QPointF(x + 4, 14), 4, 4)
        p.setPen(QColor(fg))
        p.drawText(QRectF(x + 16, 0, tw + 4, 28), Qt.AlignVCenter | Qt.AlignLeft, self.label)


class ModeSwitch(QWidget):
    """Manual | Execution switch (spec 5.7): 128 wide, exactly one option active. On this page it only SHOWS the
    arm's real Franka Desk mode (Manual = Programming); changing it is done in Operator Mode > Robot setup."""

    def __init__(self):
        super().__init__()
        self.setFixedSize(128, 26)
        self.mode = None
        self.setToolTip('Shows the arm mode. Change it in Operator Mode > Robot setup.')

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.update()

    def paintEvent(self, _e):
        c = T.pal()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(c['line']), 1))
        p.setBrush(QColor(c['track']))
        p.drawRoundedRect(QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5), 13, 13)
        f = QFont(T.HEAD, 7)
        f.setWeight(QFont.DemiBold)
        p.setFont(f)
        ow = (self.width() - 4) / 2
        for i, (key, text) in enumerate((('manual', 'Manual'), ('execution', 'Execution'))):
            box = QRectF(2 + i * ow, 2, ow, 22)
            active = self.mode == key
            if active:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(T.DARK_BLUE if T.MODE == 'light' else T.GREY))
                p.drawRoundedRect(box, 11, 11)
            p.setPen(QColor(T.WHITE if active else c['muted']))
            p.drawText(box, Qt.AlignCenter, text)


class DeviceRow(QFrame):
    """Sidebar row (spec 5.4 / 5.5). Parent: min 48 px, grows with its group. Sub-row: 36 px, indented, accent edge.
    Control on the right: status pill or mode switch (same right edge for all)."""

    def __init__(self, name, sub=False, mode=False):
        super().__init__()
        self.setObjectName('row')
        self.sub = sub
        h = QHBoxLayout(self)
        h.setContentsMargins(12 if sub else 16, 0, 16, 0)
        h.setSpacing(10)
        self.name = QLabel(name)
        self.ctrl = ModeSwitch() if mode else StatusPill()
        h.addWidget(self.name, 1)
        h.addWidget(self.ctrl)
        if sub:
            self.setFixedHeight(36)
        else:
            self.setFixedHeight(44)                  # LivePage.fit_rows() stretches all parent rows equally
        self.restyle()

    def set(self, variant, label):
        self.ctrl.set(variant, label)

    def set_mode(self, mode):
        self.ctrl.set_mode(mode)

    def restyle(self):
        c = T.pal()
        edge = f"border-left:3px solid {T.DARK_BLUE if T.MODE == 'light' else T.OFF_WHITE};" if self.sub else ''
        self.setStyleSheet(f"QFrame#row {{background:{c['row']}; border:1px solid {c['line']}; {edge} "
                           "border-radius:10px;}")
        self.name.setStyleSheet(f"font-size:{14 if self.sub else 16}px; font-weight:500; color:{c['fg']}; "
                                "background:transparent; border:none;")
        self.ctrl.update()


class LivePage(QWidget):
    """Camera page, adapted from the design spec "Teleop Control Station" (Franka brand colours, Montserrat):
    header (logo, title, tabs), ROBO STATUS sidebar (slides out, closed by default) and a 2 x 2 camera grid:
        Head Cam - RGB | Head Cam - Depth
        Hand Cam - Left (+ depth inset) | Hand Cam - Right (+ depth inset)"""
    SIDE_W = 350                          # narrower sidebar -> bigger cameras while the status is open
    GAP = 8                               # small gap between the camera pictures
    MARGIN = 10                           # small side margin
    go_operator = pyqtSignal()

    GROUPS = [('Robots', [('arm_left', 'FR3 Left Arm', 'p'), ('fci_left', 'FCI', 's'), ('mode_left', 'Mode', 'm'),
                          ('arm_right', 'FR3 Right Arm', 'p'), ('fci_right', 'FCI', 's'), ('mode_right', 'Mode', 'm')]),
              ('End Effectors', [('grip', 'Gripper', 'p'), ('grip_left', 'Left Gripper', 's'),
                                 ('grip_right', 'Right Gripper', 's')]),
              ('Cameras', [('cam_zed', 'Head Cam', 'p'), ('cam_d405_1', 'Hand Cam – Left', 'p'),
                           ('cam_d405_2', 'Hand Cam – Right', 'p')]),
              ('Teleop Hardware', [('gello', 'Gello Duo', 'p'), ('gello_left', 'Left Hand', 's'),
                                   ('gello_right', 'Right Hand', 's')])]

    def __init__(self, hub):
        super().__init__()
        self.hub = hub
        root = QVBoxLayout(self)
        root.setContentsMargins(self.MARGIN, 6, self.MARGIN, 6)      # small top / bottom margin (cameras get the height)
        root.setSpacing(6)
        self.side_w, self.status_open = 0, False

        # ---- header bar (76 px): logo + title left, tabs + controls right
        self.header = QFrame()
        self.header.setObjectName('card')
        self.header.setFixedHeight(76)
        hl = QHBoxLayout(self.header)
        hl.setContentsMargins(20, 0, 16, 0)
        hl.setSpacing(16)
        self.logo = QLabel()
        self.logo.setObjectName('plain')
        hl.addWidget(self.logo, 0, Qt.AlignVCenter)
        titles = QVBoxLayout()
        titles.setSpacing(2)
        self.title = QLabel('Teleop Control Station')
        self.title.setObjectName('plain')
        self.subtitle = label('DUAL-ARM  /  GELLO DUO', 'mono')
        titles.addWidget(self.title)
        titles.addWidget(self.subtitle)
        hl.addLayout(titles)
        hl.addStretch(1)
        self.track = QFrame()                              # nav tabs: Camera View | Operator Mode
        self.track.setObjectName('track')
        tl = QHBoxLayout(self.track)
        tl.setContentsMargins(4, 4, 4, 4)
        tl.setSpacing(6)
        self.tabs = []
        for text, slot in (('Camera View', None), ('Operator Mode', self.go_operator.emit)):
            b = QPushButton(text)
            b.setFixedHeight(40)
            b.setProperty('active', slot is None)
            if slot:
                b.clicked.connect(slot)
            else:
                b.setCursor(Qt.ArrowCursor)
            tl.addWidget(b)
            self.tabs.append(b)
        self.track.setFixedHeight(48)
        hl.addWidget(self.track, 0, Qt.AlignVCenter)
        self.strip = QHBoxLayout()                         # STATUS / theme / full screen (moved in by MainWindow)
        self.strip.setSpacing(10)
        hl.addLayout(self.strip)
        root.addWidget(self.header)

        body = QHBoxLayout()
        body.setSpacing(self.GAP)
        # ---- ROBO STATUS sidebar
        self.panel = QFrame()
        self.panel.setObjectName('card')
        self.panel.setFixedWidth(1)
        self.panel.hide()                                  # closed by default (visitors don't see it)
        sv = QVBoxLayout(self.panel)
        sv.setContentsMargins(12, 10, 12, 10)
        sv.setSpacing(8)
        top = QHBoxLayout()
        top.addWidget(label('ROBO STATUS', 'cardTitle'))
        top.addStretch(1)
        top.addWidget(label('11 devices', 'mono'))
        sv.addLayout(top)
        self.state = label('', 'mono')
        self.state.setMaximumHeight(18)
        sv.addWidget(self.state)
        self.rows = {}
        for group, items in self.GROUPS:
            gl = QVBoxLayout()
            gl.setSpacing(6)
            gl.addWidget(label(group.upper(), 'caption'))
            parents = 0
            for key, text, kind in items:
                r = DeviceRow(text, sub=(kind != 'p'), mode=(kind == 'm'))
                self.rows[key] = r
                parents += kind == 'p'
                gl.addWidget(r)
            sv.addLayout(gl)
        body.addWidget(self.panel)

        # ---- camera grid
        self.tiles = {key: CameraTile(title, key, hub, pip=(key != 'zed')) for title, key, _, _ in CAMERAS}
        self.zed_depth = CameraTile('Head Cam – Depth', 'zed', hub, depth_only=True)
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)               # no margins: tile sizes add up exactly (no window growth)
        grid.setSpacing(self.GAP)
        grid.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.tiles['zed'], 0, 0)
        grid.addWidget(self.zed_depth, 0, 1)
        grid.addWidget(self.tiles['d405_1'], 1, 0)
        grid.addWidget(self.tiles['d405_2'], 1, 1)
        gw = QWidget()
        gw.setObjectName('plain')
        gw.setLayout(grid)
        body.addWidget(gw, 1)
        root.addLayout(body, 1)
        self.restyle()

    def minimumSizeHint(self):
        """The tiles are set to fixed sizes by fit_tiles() on every resize, and those fixed sizes used to become this
        page's minimum: the window's stacked layout takes the LARGEST minimum of all pages, so the window grew to
        1173 px on a 1080 px screen (top not covered, bottom of the wrist tiles cut). The page can shrink: it
        re-fits its tiles to whatever size it gets."""
        return QSize(900, 420)

    def set_status_open(self, on, animate=True):
        """Slide the ROBO STATUS sidebar in / out; the cameras grow to fill the freed width.
        animate=False: jump to the final state (used for the camera display, which starts with the status open)."""
        if on == self.status_open:
            return
        self.status_open = on
        if not animate:
            self.side_w = self.SIDE_W if on else 0
            self.panel.setFixedWidth(max(self.side_w, 1))
            self.panel.setVisible(on)
            self.fit_tiles()
            if on:
                QTimer.singleShot(0, self.fit_rows)
            return
        if on:
            self.panel.show()
            QTimer.singleShot(0, self.fit_rows)
        if not hasattr(self, 'anim'):
            self.anim = QVariantAnimation(self)
            self.anim.setDuration(260)
            self.anim.setEasingCurve(QEasingCurve.InOutCubic)
            self.anim.valueChanged.connect(self._set_side)
            self.anim.finished.connect(lambda: (self.panel.setVisible(self.status_open), self.fit_rows()))
        self.anim.stop()
        self.anim.setStartValue(self.side_w)
        self.anim.setEndValue(self.SIDE_W if on else 0)
        self.anim.start()

    def fit_rows(self):
        """Share the free sidebar height equally between the 7 parent rows (spec 5.3); sub-rows stay 36 px."""
        parents = [r for r in self.rows.values() if not r.sub]
        for r in parents:
            r.setFixedHeight(44)
        lay = self.panel.layout()
        lay.activate()
        free = self.panel.height() - lay.minimumSize().height()
        extra = max(0, free) // len(parents)
        for r in parents:
            r.setFixedHeight(44 + extra)

    def _set_side(self, v):
        self.side_w = int(v)
        self.panel.setFixedWidth(max(self.side_w, 1))
        self.fit_tiles()

    def frame(self, key, mode, obj):
        self.tiles[key].set_frame(mode, obj)
        if key == 'zed' and mode == 'depth':
            self.zed_depth.set_frame(mode, obj)

    def set_logo(self, pm, w, h):
        self.logo.setPixmap(pm.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo.setFixedSize(w, h)

    def restyle(self):
        c = T.pal()
        self.title.setStyleSheet(f"font-size:22px; font-weight:700; color:{c['fg']}; background:transparent;")
        self.track.setStyleSheet(f"QFrame#track {{background:{c['bg']}; border:1px solid {c['line']}; "
                                 "border-radius:10px;}")
        for b in self.tabs:       # Franka Grey buttons; the current page (Camera View) in Franka Dark Blue
            active = bool(b.property('active'))
            bg = (T.DARK_BLUE if T.MODE == 'light' else T.BLACK) if active else T.GREY
            hov = bg if active else (T.DARK_BLUE if T.MODE == 'light' else T.GREY_HOVER)
            b.setStyleSheet(f"QPushButton {{background:{bg}; color:{T.WHITE}; border:none; border-radius:8px; "
                            f"padding:0 24px; font-size:15px; font-weight:600;}} QPushButton:hover {{background:{hov};}}")
        for r in self.rows.values():
            r.restyle()
        for t in (*self.tiles.values(), self.zed_depth):
            t.refresh()

    def fit_tiles(self):
        """Four equal 16:9 tiles (2 x 2) that fill everything not used by the header and the sidebar."""
        m = self.layout().contentsMargins()
        side = (self.side_w + self.GAP) if self.side_w > 0 else 0
        avail_w = self.width() - m.left() - m.right() - side
        avail_h = self.height() - m.top() - m.bottom() - self.header.height() - self.layout().spacing()
        if avail_w <= 200 or avail_h <= 200:
            return
        tw = min((avail_w - self.GAP) / 2, (avail_h - self.GAP) / 2 * 16 / 9)
        for t in (*self.tiles.values(), self.zed_depth):
            t.setFixedWidth(int(tw))
            t.setFixedHeight(t.height_for(int(tw)))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.fit_tiles()
        if self.status_open:
            QTimer.singleShot(0, self.fit_rows)

    def showEvent(self, e):
        super().showEvent(e)
        self.fit_tiles()

    def update_model(self, m):
        state = m.state
        self.state.setText(f'{state.replace("_", " ")}  ·  {plain(m.message)}' if m.message else state.replace('_', ' '))
        off = state == 'OFFLINE'
        lv = {}
        for arm in ARMS:
            a = m.arm(arm)
            if off or not a:
                lv['arm_' + arm] = ('off', 'OFFLINE')
            elif a.get('errors') or a.get('robot_mode') in ('REFLEX', 'USER_STOPPED'):
                lv['arm_' + arm] = ('off', 'ERROR')
            elif m.arm_level(arm) == 'ok':
                lv['arm_' + arm] = ('ok', 'ONLINE')
            else:
                lv['arm_' + arm] = ('off', 'OFFLINE')
            d = m.desk.get(arm, {}) if m.desk_ok else {}
            lv['fci_' + arm] = (('ok', 'ENABLED') if d.get('fci') else ('warn', 'DISABLED') if d else ('off', 'NO DATA'))
            self.rows['mode_' + arm].set_mode({'Programming': 'manual', 'Execution': 'execution'}.get(d.get('mode')))
            g = a.get('gripper', 'off')
            lv['grip_' + arm] = (('ok', 'CONNECTED') if g == 'ready' else ('warn', 'NOT READY') if g == 'not ready'
                                 else ('off', 'OFFLINE'))
            h = a.get('gello', 'no data')
            lv['gello_' + arm] = (('ok', 'CONNECTED') if h in ('in holder', 'out of holder')
                                  else ('off', 'OFFLINE') if off else ('off', 'NO DATA'))
        for parent, kids in (('grip', ('grip_left', 'grip_right')), ('gello', ('gello_left', 'gello_right'))):
            levels = [lv[k][0] for k in kids]
            lv[parent] = (('ok', 'CONNECTED') if all(x == 'ok' for x in levels) else
                          ('warn', 'CHECK') if 'warn' in levels or 'ok' in levels else ('off', 'OFFLINE'))
        for key in CAM_TITLE:
            lv['cam_' + key] = ('ok', 'LIVE') if m.cam_fps(key) else ('off', 'NO SIGNAL')
        for key, (variant, text) in lv.items():
            if key in self.rows:
                self.rows[key].set(variant, text)
        for key, tile in self.tiles.items():
            tile.set_status(*lv['cam_' + key])
        self.zed_depth.set_status(*lv['cam_zed'])


class OperatorPage(QWidget):
    """Everything Jithin's CONTROL window had, all sections visible at once in an aligned 3-column grid
    (all buttons Franka Grey)."""
    back = pyqtSignal()

    def __init__(self, model, ros):
        super().__init__()
        self.m, self.ros = model, ros
        self.pending = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(48, 20, 48, 24)
        lay.setSpacing(14)
        top = QHBoxLayout()
        top.addWidget(label('OPERATOR', 'pageTitle'))
        top.addStretch(1)
        b = QPushButton('BACK TO DEMO')
        b.clicked.connect(self.back.emit)
        top.addWidget(b)
        lay.addLayout(top)
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.NoFrame)
        body = QWidget()
        grid = QGridLayout(body)
        grid.setContentsMargins(0, 0, 8, 0)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(16)
        for col in range(3):
            grid.setColumnStretch(col, 1)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        # row 0: TESTING STAGE | FIX ERRORS | ROBOT SETUP      row 1: STATUS (2 columns) | SYSTEM      row 2: LOG
        grid.addWidget(self.build_testing(), 0, 0)
        grid.addWidget(self.build_fix(), 0, 1)
        grid.addWidget(self.build_setup(), 0, 2)
        grid.addWidget(self.build_status(), 1, 0, 1, 2)
        grid.addWidget(self.build_system(), 1, 2)
        grid.addWidget(self.build_log(), 2, 0, 1, 3)
        grid.setRowStretch(0, 0)
        grid.setRowStretch(1, 0)
        grid.setRowStretch(2, 1)
        area.setWidget(body)
        lay.addWidget(area, 1)
        for line in model.lines:
            self.log.appendPlainText(line)
        model.log_line.connect(self.log.appendPlainText)
        ros.reply.connect(self.on_reply)

    def send(self, cmd):
        self.m.log(f'> {cmd.upper()}')
        self.ros.call(cmd)

    def button(self, text, cmd, name=None, big=False):
        b = QPushButton(text)
        if name:
            b.setObjectName(name)
        if big:
            b.setMinimumHeight(60)
            b.setStyleSheet("font-size:18px;")
        b.clicked.connect(lambda: self.send(cmd))
        return b

    # -- cards (grid cells)
    def build_testing(self):
        c, cv = card('TESTING STAGE', 'pedals: 1 START (hold 3 s) · 2 STOP, then HOME · 3 RESET')
        row = QHBoxLayout()
        row.setSpacing(12)
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
        cv.addWidget(label('GRIPPERS  (only while teleop is stopped)', 'caption'))
        self.grip = {}
        for arm in ARMS:
            row = QHBoxLayout()
            row.setSpacing(10)
            row.addWidget(label(arm.upper(), 'caption'), 1)
            go = self.button('OPEN', f'gripper/{arm}/open')
            gc = self.button('CLOSE', f'gripper/{arm}/close')
            row.addWidget(go)
            row.addWidget(gc)
            cv.addLayout(row)
            self.grip[arm] = (go, gc)
        cv.addStretch(1)
        return c

    def build_fix(self):
        c, cv = card('FIX ERRORS')
        cv.addWidget(label('CURRENT PROBLEMS', 'caption'))
        self.fix_issues = label('', 'value', wrap=True)
        self.fix_issues.setTextFormat(Qt.RichText)
        cv.addWidget(self.fix_issues)
        cv.addSpacing(6)
        cv.addWidget(label('After a robot reflex or an E-stop: release the E-stop, then press ERROR RECOVERY. '
                           'If the arm stack stopped, it is restarted (grippers, GELLO and cameras keep running, '
                           'the robots do not move). SHUT DOWN + BRING UP (System) restarts everything.', 'subline',
                           wrap=True))
        self.b_recover = self.button('ERROR RECOVERY', 'recover')
        cv.addWidget(self.b_recover)
        cv.addStretch(1)
        return c

    def build_setup(self):
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
        cv.addLayout(row)
        self.desk_note = label('', 'mono', wrap=True)
        cv.addWidget(self.desk_note)
        cv.addStretch(1)
        return c

    def build_status(self):
        c, cv = card('STATUS')
        row = QHBoxLayout()
        row.setSpacing(24)
        self.arm_rows = {}
        for arm in ARMS:
            g = QGridLayout()
            g.setHorizontalSpacing(14)
            g.addWidget(label(f'{arm.upper()} ARM  ·  {ARM_IP[arm]}', 'caption'), 0, 0, 1, 2)
            self.arm_rows[arm] = {}
            for i, name in enumerate(('Robot', 'Errors', 'Teleop', 'GELLO', 'Gripper', 'Desk')):
                g.addWidget(label(name.upper(), 'caption'), i + 1, 0)
                val = label('-', 'value', wrap=True)
                g.addWidget(val, i + 1, 1)
                self.arm_rows[arm][name] = val
            g.setColumnStretch(1, 1)
            row.addLayout(g, 1)
        cv.addLayout(row)
        cv.addStretch(1)
        return c

    def build_system(self):
        c, cv = card('SYSTEM')
        row = QHBoxLayout()
        row.setSpacing(12)
        self.b_bringup = self.button('BRING UP', 'bringup')
        self.b_shutdown = self.button('SHUT DOWN', 'shutdown')
        row.addWidget(self.b_bringup)
        row.addWidget(self.b_shutdown)
        cv.addLayout(row)
        g = QGridLayout()
        g.setHorizontalSpacing(18)
        g.setVerticalSpacing(6)
        self.lights = {}
        for i, name in enumerate(('Desk', 'Robots', 'FCI', 'GELLO', 'Grippers', 'Cameras', 'Pedal')):
            item = QHBoxLayout()
            item.setSpacing(8)
            d = Dot()
            item.addWidget(d)
            item.addWidget(label(name.upper(), 'caption'))
            item.addStretch(1)
            g.addLayout(item, i // 2, i % 2)
            self.lights[name] = d
        cv.addLayout(g)
        self.sys_info = label('', 'mono', wrap=True)
        cv.addWidget(self.sys_info)
        cv.addStretch(1)
        return c

    def build_log(self):
        c, cv = card('LOG')
        c.setMinimumHeight(170)
        cv.addWidget(self.log, 1)
        return c

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
        self.b_bringup.setEnabled(conn and state == 'OFFLINE')
        self.b_shutdown.setEnabled(conn and state in ('IDLE', 'ERROR', 'STARTING_UP'))
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
    LIVE, OPERATOR = 0, 1                 # the Home page was removed (2026-11-21): cameras are always the base page

    def __init__(self, model, ros, hub, decoder, preview=False, role='main'):
        super().__init__()
        self.m, self.hub, self.decoder = model, hub, decoder
        self.role = role                      # 'main': camera page (status closed) + Operator; 'camera': camera page, status open
        self.was_teleop = False
        self.setWindowTitle('Robot demo' + (' - CAMERAS' if role == 'camera' else '') + (' (PREVIEW)' if preview else ''))
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
        self.controls = QWidget()                    # chips + buttons: in the navbar, or beside the status window
        self.controls.setObjectName('plain')
        cl = QHBoxLayout(self.controls)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(12)
        self.chip_left, self.chip_right = StatusChip('Left arm'), StatusChip('Right arm')
        self.chip_cams = StatusChip('Cameras')

        def navbtn(text='', slot=None):
            b = QPushButton(text)
            b.setObjectName('navbtn')
            b.setFixedHeight(40)
            if slot:
                b.clicked.connect(slot)
            return b
        self.b_status = navbtn(slot=lambda: self.set_status(not self.live.status_open))   # camera page only
        self.b_operator = navbtn('OPERATOR MODE', self.open_operator)   # no drop-down: opens the operator panel
        self.b_theme = navbtn(slot=lambda: hub.set_theme('dark' if hub.theme == 'light' else 'light'))
        self.b_full = navbtn(slot=self.toggle_full)
        self.main_window = self                      # the camera window points at the main window (see main())
        for w in (self.chip_left, self.chip_right, self.chip_cams, self.b_status, self.b_operator, self.b_theme, self.b_full):
            cl.addWidget(w)
        nl.addWidget(self.controls)
        self.nav, self.controls_host = nav, nl
        outer.addWidget(nav)

        # problem bar (Operator page; the camera page shows problems in its status window)
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
        self.live = LivePage(hub)
        self.operator = OperatorPage(model, ros)
        self.operator.back.connect(self.go_auto)
        self.live.go_operator.connect(self.open_operator)
        for pg in (self.live, self.operator):                 # index 0 = LIVE, 1 = OPERATOR
            self.stack.addWidget(pg)
        outer.addWidget(self.stack, 1)

        decoder.frame.connect(self.live.frame)
        model.updated.connect(self.update_all)
        hub.theme_changed.connect(self.apply_theme)
        self.apply_theme(hub.theme)
        self.b_full.setText('FULL SCREEN')
        # both displays start on the camera page: the main one with the status window closed, the camera display (second
        # screen) with the ROBO STATUS list open on the left of the cameras
        self.live.set_status_open(role == 'camera', animate=False)
        self.set_status(role == 'camera', animate=False)
        self.show_page(self.LIVE)

    # -- navigation
    def show_page(self, idx):
        self.stack.setCurrentIndex(idx)
        self.decoder.set_visible(self, idx == self.LIVE)
        on_live = idx == self.LIVE
        # camera page: no navbar - the controls move into the page's header bar (the status list shows the chips' info)
        for chip in (self.chip_left, self.chip_right, self.chip_cams):
            chip.setVisible(not on_live)
        self.b_operator.setVisible(not on_live)       # the camera page has the Operator Mode tab instead
        self.b_status.setVisible(on_live)
        self.controls_host.removeWidget(self.controls)
        self.controls_host = self.live.strip if on_live else self.nav.layout()
        self.controls_host.addWidget(self.controls)
        self.nav.setVisible(not on_live)
        self.live.fit_tiles()
        self.update_all()

    def show_operator(self):
        self.show_page(self.OPERATOR)

    def open_operator(self):
        """OPERATOR button. On the camera display the panel opens on the MAIN display (the camera display stays on
        the cameras)."""
        w = self.main_window
        w.show_operator()
        if w is not self:
            w.raise_()
            w.activateWindow()

    def set_status(self, on, animate=True):
        self.live.set_status_open(on, animate)
        self.b_status.setText('STATUS  ◂' if on else 'STATUS  ▸')

    def toggle_full(self):
        """Full screen <-> normal window (with the title bar's minimise / maximise buttons)."""
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def changeEvent(self, e):
        super().changeEvent(e)
        if e.type() == QEvent.WindowStateChange:
            self.b_full.setText('EXIT FULL SCREEN' if self.isFullScreen() else 'FULL SCREEN')

    def go_auto(self):
        """Back from the Operator page: always the camera page."""
        self.show_page(self.LIVE)

    # -- look
    def apply_theme(self, mode):
        self.setStyleSheet(T.stylesheet(mode))
        pm, w, h = T.logo_pixmap(mode, 220)
        self.logo.setPixmap(pm.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.logo.setFixedSize(w, h)
        self.live.set_logo(pm, w, h)
        self.b_theme.setText('☾  DARK' if mode == 'light' else '☀  LIGHT')
        c = T.pal()
        self.errbar.setStyleSheet(f"QFrame#errbar {{background:{c['panel']}; border-bottom:1px solid {c['line']};}}")
        for chip in (self.chip_left, self.chip_right, self.chip_cams):
            chip.restyle()
        self.live.restyle()
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
        # page change from the supervisor state (pedals and buttons behave the same): the camera display always shows
        # the cameras; when teleop starts the main display leaves the Operator page for the cameras. (No Home page.)
        teleop = state in TELEOP_STATES
        if self.role == 'camera':
            if self.stack.currentIndex() != self.LIVE:
                self.show_page(self.LIVE)
                return
        if self.role == 'main' and teleop != self.was_teleop:
            self.was_teleop = teleop
            if teleop and self.stack.currentIndex() != self.LIVE:
                self.show_page(self.LIVE)
                return
        # problem bar
        iss = m.issues()
        idx = self.stack.currentIndex()
        if idx == self.LIVE:                  # the camera page shows problems in its status window
            iss = []
        if iss:
            level = iss[0][0]
            self.err_dot.set_level({'error': 'bad', 'warn': 'warn', 'info': 'off'}[level])
            self.err_text.setText('   ·   '.join(t for _l, t in iss[:2]))
            self.err_text.setStyleSheet(f"background:transparent; color:{T.pal()['fg']}; font-weight:600; font-size:14px;")
            self.errbar.show()
        else:
            self.errbar.hide()
        if idx == self.LIVE:
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
        self.has_depth = True                 # F11 toggles: depth topic available / not available
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
                          'Check the robots (release E-stop), then press ERROR RECOVERY')

    def toggle_cam(self):
        self.drop_cam = not self.drop_cam

    def toggle_depth(self):
        self.has_depth = not self.has_depth
        if not self.has_depth:
            with self.ros.lock:
                for _t, key, _r, _d in CAMERAS:
                    self.ros.frames.pop((key, 'depth'), None)
                    self.ros.frame_t.pop((key, 'depth'), None)

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
                if self.has_depth:                # fake depth picture, so the picture-in-picture can be seen
                    if (key, 'd') not in self.frame_imgs:
                        self.frame_imgs[(key, 'd')] = self.fake_jpeg(title, depth=True)
                    self.ros.frames[(key, 'depth')] = self.frame_imgs[(key, 'd')]
                    self.ros.frame_t[(key, 'depth')] = now

    @staticmethod
    def fake_jpeg(title, depth=False):
        img = QImage(960, 540, QImage.Format_RGB32)
        p = QPainter(img)
        g = QLinearGradient(0, 0, 960, 540)
        g.setColorAt(0, QColor('#E69500' if depth else T.GREY))
        g.setColorAt(0.5, QColor('#567FC5' if depth else T.GREY))
        g.setColorAt(1, QColor('#1B2F36' if depth else T.DARK_BLUE))
        p.fillRect(img.rect(), g)
        p.setPen(QColor(T.OFF_WHITE))
        f = QFont(T.HEAD, 30)
        f.setWeight(QFont.DemiBold)
        p.setFont(f)
        p.drawText(img.rect(), Qt.AlignCenter, f'{title.upper()}\n{"DEPTH" if depth else "CAMERA"} PREVIEW')
        p.end()
        buf = QBuffer()
        buf.open(QIODevice.WriteOnly)
        img.save(buf, 'JPG', 85)
        return bytes(buf.data())


def place(widget, screen, full=True):
    """Put the window on its screen: full screen by default (the FULL SCREEN button / title bar switch to a window)."""
    g = screen.availableGeometry()
    widget.setGeometry(screen.geometry() if full else g)
    widget.move(screen.geometry().topLeft() if full else g.topLeft())
    widget.showFullScreen() if full else widget.showMaximized()


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
    ap.add_argument('--invert', default=','.join(sorted(INVERT)),
                    help='cameras shown rotated by 180 degrees (RGB and depth), comma separated keys: zed,d405_1,d405_2 '
                         '(default: the two wrist cameras; "" = none)')
    ap.add_argument('--force-two', action='store_true', help=argparse.SUPPRESS)      # test: two windows, one screen
    ap.add_argument('--screenshot', metavar='DIR', help=argparse.SUPPRESS)       # offscreen design check
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    INVERT.clear()
    INVERT.update(k.strip() for k in a.invert.split(',') if k.strip())
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
    two = (not a.single and len(screens) >= 2) or a.force_two
    # display 1 (--control-screen): Home / Operator, switches to the cameras on START; display 2: cameras, always
    windows = [MainWindow(model, ros, hub, decoder, a.preview, 'main')]
    if two:
        windows.append(MainWindow(model, ros, hub, decoder, a.preview, 'camera'))
    if feeder:
        feeder.attach(ros)
        for w in windows:
            QShortcut(Qt.Key_F5, w, activated=lambda: feeder.pedal('start'))
            QShortcut(Qt.Key_F6, w, activated=lambda: feeder.pedal('stop'))
            QShortcut(Qt.Key_F7, w, activated=lambda: feeder.pedal('home'))
            QShortcut(Qt.Key_F9, w, activated=feeder.error)
            QShortcut(Qt.Key_F10, w, activated=feeder.toggle_cam)
            QShortcut(Qt.Key_F11, w, activated=feeder.toggle_depth)
    if two:
        windows[1].main_window = windows[0]       # OPERATOR on the camera display opens on the main display
    full = not a.screenshot
    place(windows[0], screens[min(a.control_screen, len(screens) - 1)], full)
    if two:
        place(windows[1], screens[min(a.camera_screen, len(screens) - 1)], full)
    if a.screenshot:
        run_screenshots(app, a.screenshot, windows, hub, feeder)
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    tick = QTimer()                      # lets Python handle Ctrl-C while Qt runs
    tick.timeout.connect(lambda: None)
    tick.start(200)
    rc = app.exec_()
    ros.shutdown()
    rclpy.try_shutdown()
    sys.exit(rc)


def run_screenshots(app, outdir, windows, hub, feeder):
    """Developer aid: walk through the pages in --preview and save PNGs (works with QT_QPA_PLATFORM=offscreen)."""
    os.makedirs(outdir, exist_ok=True)
    win = windows[0]
    for w in windows:
        w.resize(1920, 1080)
    steps = []

    def shot(name):
        win.grab().save(os.path.join(outdir, name + '.png'))
        if len(windows) > 1:
            windows[1].grab().save(os.path.join(outdir, name + '_cam.png'))

    def at(t, fn):
        steps.append(t)
        QTimer.singleShot(int(t * 1000), fn)

    at(1.0, lambda: shot('1_cameras_offline_light'))
    at(1.2, lambda: feeder.command('bringup'))
    at(5.0, lambda: shot('2_cameras_ready_light'))
    at(5.2, lambda: hub.set_theme('dark'))
    at(5.8, lambda: shot('3_cameras_ready_dark'))
    at(6.0, lambda: hub.set_theme('light'))
    at(6.2, lambda: feeder.pedal('start'))
    at(10.0, lambda: shot('4_live_status_OFF_light'))
    at(10.2, lambda: win.set_status(True))
    at(11.4, lambda: shot('5_live_status_ON_light'))
    at(11.6, lambda: (win.set_status(False), feeder.toggle_cam(), hub.set_theme('dark')))
    at(13.2, lambda: shot('6_live_camdrop_status_OFF_dark'))
    at(13.4, lambda: win.set_status(True))
    at(14.6, lambda: shot('7_live_camdrop_status_ON_dark'))
    at(14.8, lambda: (win.set_status(False), feeder.toggle_cam(), hub.set_theme('light'), feeder.error()))
    at(17.2, lambda: shot('9_cameras_error_light'))
    at(17.4, lambda: win.open_operator())
    at(18.2, lambda: shot('10_operator_light'))
    at(18.4, lambda: hub.set_theme('dark'))
    at(19.0, lambda: shot('11_operator_dark'))
    at(19.8, app.quit)


if __name__ == '__main__':
    main()
