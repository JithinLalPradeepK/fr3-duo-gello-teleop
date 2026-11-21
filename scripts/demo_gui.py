#!/usr/bin/env python3
"""Robot demo GUI - a thin client of demo_supervisor and desk_agent (same services as the foot pedals).

  source ~/robot_demo/env/host_env.sh
  python3 ~/robot_demo/scripts/demo_gui.py [--single] [--control-screen 0] [--camera-screen 1]
  python3 ~/robot_demo/scripts/demo_gui.py --preview      design preview with demo data: sends NOTHING
                                                         (own ROS domain 199, localhost only; F9 = demo ERROR)

Two windows on two monitors: CONTROL (state, START / STOP / HOME, arms, system, robot setup toggles, log) and
CAMERAS (ZED Mini + both D405 color streams). With one monitor (or --single) both are tabs.
Styled after the Franka Robotics brand guidelines (scripts/franka_theme.py).
THE GUI IS NOT A SAFETY DEVICE - the physical E-stop stays in use. It never moves anything itself:
every button is a /demo/* service call, and demo_supervisor / desk_agent decide.
"""
import argparse
import json
import os
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import (QBuffer, QByteArray, QEasingCurve, QIODevice, QObject, QPropertyAnimation, QRectF, Qt,
                          QTimer, pyqtProperty, pyqtSignal)
from PyQt5.QtGui import QColor, QFont, QImage, QImageReader, QLinearGradient, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit,
                             QPushButton, QShortcut, QSizePolicy, QTabWidget, QVBoxLayout, QWidget)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import franka_theme as T  # noqa: E402  (Franka Robotics brand theme; safety colours kept)

ARMS = ('left', 'right')
ARM_IP = {'left': '192.168.1.51', 'right': '192.168.1.50'}
CAMERAS = [('ZED Mini', 'zed', '/zed/zed_node/rgb/color/rect/image/compressed'),
           ('D405 1', 'd405_1', '/camera/d405_1/color/image_raw/compressed'),
           ('D405 2', 'd405_2', '/camera/d405_2/color/image_raw/compressed')]
STATE_COLORS = T.STATE_COLORS
GREEN, AMBER, RED, GREY = T.LIGHT_OK, T.LIGHT_WARN, T.LIGHT_BAD, T.LIGHT_OFF     # status chips (bg, text)
HINTS = {'STARTING_UP': 'Starting arms, grippers, GELLO and cameras ...',
         'IDLE': 'Press START, then lift both GELLOs out of their holders.',
         'TELEOP_STARTING': 'Hold the GELLOs still - the robots move to the GELLO pose.',
         'TELEOP': 'The robots follow the GELLOs.  STOP ends teleoperation.',
         'STOPPING': 'The robots hold their pose.',
         'HOMING': 'Both arms move to the home pose.',
         'ERROR': 'Follow the message above.',
         'RECOVERING': 'Franka error recovery on both arms ...',
         'SHUTTING_DOWN': 'Stopping cameras, GELLO, grippers and arms ...'}
# Desk toggles: key -> (title, off text, on text, command to switch on, command to switch off, is_on(arm status))
TOGGLES = {
    'brakes': ('BRAKES', 'LOCKED', 'OPEN', 'desk/unlock', 'desk/lock',
               lambda a: {'unlocked': True, 'locked': False}.get(a.get('brakes'))),
    'mode': ('OPERATING MODE', 'PROGRAMMING', 'EXECUTION', 'desk/execution', 'desk/programming',
             lambda a: {'Execution': True, 'Programming': False}.get(a.get('mode'))),
    'fci': ('FCI', 'OFF', 'ON', 'desk/fci_on', 'desk/fci_off', lambda a: a.get('fci')),
}


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
        self.frames = {key: None for _, key, _ in CAMERAS}     # latest JPEG bytes
        self.counts = {key: 0 for _, key, _ in CAMERAS}
        self.frame_t = {key: 0.0 for _, key, _ in CAMERAS}
        self.lock = threading.Lock()
        if not preview:
            self.node.create_subscription(String, '/demo/status', self.on_status, latched)
            self.node.create_subscription(String, '/demo/desk/status', self.on_desk, latched)
            for _, key, topic in CAMERAS:
                self.node.create_subscription(CompressedImage, topic, lambda m, k=key: self.on_image(k, m),
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

    def on_image(self, key, m):
        with self.lock:
            self.frames[key] = bytes(m.data)
            self.counts[key] += 1
            self.frame_t[key] = time.monotonic()

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


# ---------------------------------------------------------------------------------------------- widgets
def label(text, name='value', wrap=False):
    w = QLabel(text)
    w.setObjectName(name)
    w.setWordWrap(wrap)
    return w


def card(title=None, extra=None):
    """Rounded panel; returns (frame, inner layout)."""
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


def chip(text):
    w = QLabel(text.upper())
    w.setAlignment(Qt.AlignCenter)
    w.setFixedHeight(34)
    w.setStyleSheet(T.chip_style(GREY))
    return w


def set_light(w, color, tooltip=''):
    w.setStyleSheet(T.chip_style(color))
    w.setToolTip(tooltip)


class BigButton(QPushButton):
    """Large action button: title + small caption (e.g. START / PEDAL 1)."""

    def __init__(self, title, caption, role):
        super().__init__()
        self.setMinimumHeight(118)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.bg, self.fg = T.BUTTON[role]
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 14, 0, 14)
        lay.setSpacing(2)
        self.t = QLabel(title)
        self.c = QLabel(caption)
        for w, size, fam in ((self.t, 34, T.HEAD), (self.c, 12, T.MONO)):
            w.setAlignment(Qt.AlignCenter)
            w.setAttribute(Qt.WA_TransparentForMouseEvents)
            w.setProperty('fs', size)
            w.setProperty('fam', fam)
            lay.addWidget(w)
        self.restyle()

    def restyle(self):
        on = self.isEnabled()
        self.setStyleSheet(f'QPushButton {{background:{self.bg if on else T.PANEL}; border-radius:14px; '
                           f'border:{"none" if on else "1px solid " + T.OFF_WHITE};}}'
                           'QPushButton:pressed {background:#000000;}')
        for w in (self.t, self.c):
            w.setStyleSheet(f"background:transparent; color:{self.fg if on else T.OFF_WHITE}; "
                            f"font-family:'{w.property('fam')}'; font-size:{w.property('fs')}px; "
                            f"font-weight:{600 if w is self.t else 400};")

    def changeEvent(self, e):
        super().changeEvent(e)
        if e.type() == e.EnabledChange:
            self.restyle()


class Switch(QWidget):
    """Painted toggle switch. value: True (right / on), False (left / off), None (unknown or arms differ)."""
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
        p.drawEllipse(QRectF(x, r.top() + 5, d, d))                      # soft shadow
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
        for w, active in ((self.off, value is False), (self.on, value is True)):
            w.setStyleSheet(f"background:transparent; font-family:'{T.HEAD}'; font-size:13px; font-weight:600; "
                            f"color:{T.DARK_BLUE if active else T.OFF_WHITE if not self.isEnabled() else T.GREY};")

    def setEnabled(self, on):
        super().setEnabled(on)
        self.set_value(self.sw.value, self.sw.pending)


# ---------------------------------------------------------------------------------------------- windows
class ControlWindow(ConfirmClose, QWidget):
    def __init__(self, ros, preview=False):
        super().__init__()
        self.ros = ros
        self.state = 'OFFLINE'
        self.last_status_t = 0.0
        self.last_state_logged = None
        self.desk, self.desk_t, self.desk_last = {}, 0.0, None
        self.pending = {}                                   # toggle key -> (wanted value, deadline)
        self.setWindowTitle('Robot demo - CONTROL' + (' (PREVIEW)' if preview else ''))

        # header: logo top-left (brand clear space), title right
        header = QHBoxLayout()
        header.addWidget(T.logo(width=230), 0, Qt.AlignLeft | Qt.AlignVCenter)
        header.addStretch(1)
        titles = QVBoxLayout()
        titles.setSpacing(2)
        t1 = label('TELEOPERATION DEMO', 'title')
        t2 = label('FRANKA RESEARCH 3 DUO  ·  GELLO DUO' + ('  ·  PREVIEW' if preview else ''), 'mono')
        for t in (t1, t2):
            t.setAlignment(Qt.AlignRight)
            titles.addWidget(t)
        header.addLayout(titles)

        # state banner
        self.banner = QFrame()
        bl = QVBoxLayout(self.banner)
        bl.setContentsMargins(32, 18, 32, 18)
        bl.setSpacing(4)
        self.b_state = QLabel('WAITING')
        self.b_msg = QLabel('')
        self.b_hint = QLabel('')
        for w in (self.b_state, self.b_msg, self.b_hint):
            w.setAlignment(Qt.AlignCenter)
            w.setWordWrap(True)
            bl.addWidget(w)
        self.set_banner('OFFLINE', 'waiting for the demo ...', '')

        # main actions
        self.b_start = BigButton('START', 'PEDAL 1', 'start')
        self.b_stop = BigButton('STOP', 'PEDAL 2', 'stop')
        self.b_home = BigButton('HOME', 'PEDAL 3', 'home')
        for b, cmd in ((self.b_start, 'start'), (self.b_stop, 'stop'), (self.b_home, 'home')):
            b.clicked.connect(lambda _, c=cmd: self.send(c))
        actions = QHBoxLayout()
        actions.setSpacing(16)
        for b in (self.b_start, self.b_stop, self.b_home):
            actions.addWidget(b)

        # arms
        self.arm = {}
        arms_row = QHBoxLayout()
        arms_row.setSpacing(16)
        for arm in ARMS:
            f, v = card(f'{arm.upper()} ARM', ARM_IP[arm])
            g = QGridLayout()
            g.setHorizontalSpacing(18)
            g.setVerticalSpacing(8)
            rows = {}
            for i, name in enumerate(('Robot', 'Errors', 'Teleop', 'GELLO', 'Gripper', 'Desk')):
                c = label(name.upper(), 'caption')
                c.setMinimumWidth(70)
                val = label('-', 'value', wrap=True)
                g.addWidget(c, i, 0, Qt.AlignTop)
                g.addWidget(val, i, 1)
                rows[name] = val
            g.setColumnStretch(1, 1)
            v.addLayout(g)
            v.addStretch(1)
            gb = QHBoxLayout()
            go, gc = QPushButton('OPEN GRIPPER'), QPushButton('CLOSE GRIPPER')
            go.clicked.connect(lambda _, a=arm: self.send(f'gripper/{a}/open'))
            gc.clicked.connect(lambda _, a=arm: self.send(f'gripper/{a}/close'))
            gb.addWidget(go)
            gb.addWidget(gc)
            v.addLayout(gb)
            rows['open'], rows['close'] = go, gc
            self.arm[arm] = rows
            arms_row.addWidget(f)

        # log
        log_card, lv = card('LOG')
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        lv.addWidget(self.log)

        # system status chips
        sys_card, sv = card('SYSTEM')
        self.lights = {k: chip(k) for k in ('Desk', 'Robots', 'FCI', 'GELLO', 'Grippers', 'Cameras', 'Pedal')}
        cg = QGridLayout()
        cg.setSpacing(10)
        for i, w in enumerate(self.lights.values()):
            cg.addWidget(w, i // 4, i % 4)
        sv.addLayout(cg)

        # robot setup (Franka Desk) - toggles
        desk_card, dv = card('ROBOT SETUP', 'FRANKA DESK · BOTH ARMS')
        self.toggles = {}
        for key, (title, off, on, cmd_on, cmd_off, _) in TOGGLES.items():
            t = Toggle(title, off, on)
            t.requested.connect(lambda want, k=key: self.toggle_request(k, want))
            self.toggles[key] = t
            dv.addWidget(t)
        pp = QHBoxLayout()
        self.b_prepare = QPushButton('PREPARE ALL')
        self.b_prepare.setStyleSheet(T.button_style('primary'))
        self.b_prepare.setToolTip('Open brakes, Execution mode, FCI on')
        self.b_park = QPushButton('PARK')
        self.b_park.setStyleSheet(T.button_style('secondary'))
        self.b_park.setToolTip('FCI off, Programming mode, lock brakes, release control')
        self.b_prepare.clicked.connect(lambda: self.send('desk/prepare'))
        self.b_park.clicked.connect(lambda: self.send('desk/park'))
        pp.addWidget(self.b_prepare)
        pp.addWidget(self.b_park)
        dv.addLayout(pp)
        self.desk_note = label('Available while the demo is shut down.', 'mono', wrap=True)
        dv.addWidget(self.desk_note)

        # demo stack
        stack_card, kv = card('DEMO')
        self.b_bringup = QPushButton('BRING UP')
        self.b_shutdown = QPushButton('SHUT DOWN')
        self.b_recover = QPushButton('ERROR RECOVERY')
        for b, cmd in ((self.b_bringup, 'bringup'), (self.b_shutdown, 'shutdown'), (self.b_recover, 'recover')):
            b.setMinimumHeight(44)
            b.clicked.connect(lambda _, c=cmd: self.send(c))
        kh = QHBoxLayout()
        for b in (self.b_bringup, self.b_shutdown, self.b_recover):
            kh.addWidget(b)
        kv.addLayout(kh)

        # layout: left = arms + log, right = system / robot setup / demo
        left = QVBoxLayout()
        left.setSpacing(16)
        left.addLayout(arms_row, 3)
        left.addWidget(log_card, 2)
        right = QVBoxLayout()
        right.setSpacing(16)
        right.addWidget(sys_card)
        right.addWidget(desk_card, 1)
        right.addWidget(stack_card)
        body = QHBoxLayout()
        body.setSpacing(16)
        body.addLayout(left, 3)
        body.addLayout(right, 2)

        v = QVBoxLayout(self)
        v.setContentsMargins(48, 40, 48, 32)          # brand: logo >= 48 px from the side, >= 40 px from the top
        v.setSpacing(16)
        v.addLayout(header)
        v.addWidget(self.banner)
        v.addLayout(actions)
        v.addLayout(body, 1)

        ros.status.connect(self.on_status)
        ros.desk.connect(self.on_desk)
        ros.reply.connect(self.on_reply)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_alive)
        self.timer.start(500)
        self.apply_state('OFFLINE', connected=False)

    # -- actions
    def send(self, cmd):
        self.add_log(f'> {cmd.upper()}')
        self.ros.call(cmd)

    def toggle_request(self, key, want):
        title, off, on, cmd_on, cmd_off, _ = TOGGLES[key]
        self.pending[key] = (want, time.monotonic() + 15.0)
        self.update_toggles()
        self.send(cmd_on if want else cmd_off)

    def on_reply(self, name, ok, msg):
        self.add_log(f'  {name}: {"OK" if ok else "REFUSED"} - {msg}')
        if not ok and name.startswith('desk/'):
            self.pending.clear()
            self.update_toggles()

    def add_log(self, text):
        self.log.appendPlainText(f'{time.strftime("%H:%M:%S")}  {text}')

    # -- status
    def set_banner(self, state, message, hint):
        self.banner.setStyleSheet(f'QFrame {{background:{STATE_COLORS.get(state, T.GREY)}; border-radius:16px;}}')
        self.b_state.setText(state.replace('_', ' '))
        self.b_msg.setText(message)
        self.b_hint.setText(hint)
        self.b_hint.setVisible(bool(hint))
        self.b_state.setStyleSheet(f"background:transparent; color:{T.WHITE}; font-family:'{T.HEAD}'; "
                                   'font-size:44px; font-weight:600;')
        self.b_msg.setStyleSheet(f"background:transparent; color:{T.WHITE}; font-family:'{T.TEXT}'; font-size:18px;")
        self.b_hint.setStyleSheet(f"background:transparent; color:{T.OFF_WHITE}; font-family:'{T.HEAD}'; "
                                  'font-size:15px; font-weight:500;')

    def hint(self, state):
        if state == 'OFFLINE':
            ready = self.desk and all(TOGGLES[k][5](self.desk.get(a, {})) for k in TOGGLES for a in ARMS)
            return ('Robots ready - press BRING UP.' if ready else
                    'Set up the robots (ROBOT SETUP), then press BRING UP.')
        return HINTS.get(state, '')

    def check_alive(self):
        now = time.monotonic()
        if now - self.desk_t > 3.0:
            set_light(self.lights['Desk'], GREY, 'no desk_agent')
            for rows in self.arm.values():
                rows['Desk'].setText('no desk_agent')
            self.desk = {}
            self.update_toggles()
        if now - self.last_status_t > 3.0:
            self.set_banner('OFFLINE', 'NO CONNECTION TO THE DEMO', 'Start it with the Robot Demo icon.')
            self.apply_state('OFFLINE', connected=False)
        else:
            self.apply_state(self.state)
        if any(now > dl for _, dl in self.pending.values()):
            self.pending = {k: p for k, p in self.pending.items() if now <= p[1]}
            self.update_toggles()

    def apply_state(self, state, connected=True):
        idle = connected and state == 'IDLE'
        self.b_start.setEnabled(idle)
        self.b_home.setEnabled(idle)
        self.b_stop.setEnabled(True)                      # STOP is never disabled
        self.b_bringup.setEnabled(connected and state == 'OFFLINE')
        self.b_shutdown.setEnabled(connected and state in ('IDLE', 'ERROR', 'STARTING_UP'))
        self.b_recover.setEnabled(connected and state in ('ERROR', 'IDLE'))
        desk_free = bool(self.desk) and not self.desk.get('busy') and state == 'OFFLINE'
        for w in (*self.toggles.values(), self.b_prepare, self.b_park):
            if w.isEnabled() != desk_free:
                w.setEnabled(desk_free)
        self.desk_note.setText('Available while the demo is shut down.' if state != 'OFFLINE' or not self.desk
                               else 'Desk busy ...' if self.desk.get('busy') else 'Switches act on both arms.')
        grip_ok = connected and state not in ('TELEOP', 'TELEOP_STARTING', 'OFFLINE')
        for rows in self.arm.values():
            rows['open'].setEnabled(grip_ok)
            rows['close'].setEnabled(grip_ok)

    def update_toggles(self):
        for key, t in self.toggles.items():
            vals = [TOGGLES[key][5](self.desk.get(a, {})) for a in ARMS] if self.desk else [None]
            value = vals[0] if all(x == vals[0] for x in vals) else None
            if key in self.pending and self.pending[key][0] == value:
                del self.pending[key]                       # Desk confirmed
            t.set_value(value, key in self.pending)

    def on_desk(self, d):
        self.desk, self.desk_t = d, time.monotonic()
        if d.get('last') and d['last'] != self.desk_last:
            self.desk_last = d['last']
            self.add_log(f'Desk: {d["last"]}')
            if 'FAILED' in d['last']:
                self.pending.clear()
        ready, words = [], []
        for arm in ARMS:
            a = d.get(arm, {})
            if a.get('error'):
                txt, ok = a['error'], False
            else:
                txt = (f'{a.get("mode", "?")} · brakes {"open" if a.get("brakes") == "unlocked" else a.get("brakes", "?")}'
                       f' · FCI {"on" if a.get("fci") else "off"} · control {a.get("control", "?")}')
                ok = a.get('mode') == 'Execution' and a.get('brakes') == 'unlocked' and a.get('fci')
            self.arm[arm]['Desk'].setText(txt)
            ready.append(ok)
            words.append(f'{arm}: {txt}')
        err = any(d.get(arm, {}).get('error') for arm in ARMS)
        set_light(self.lights['Desk'], RED if err else GREEN if all(ready) else AMBER, '\n'.join(words))
        self.update_toggles()
        self.apply_state(self.state, connected=time.monotonic() - self.last_status_t <= 3.0)
        if self.state == 'OFFLINE' and time.monotonic() - self.last_status_t <= 3.0:
            self.b_hint.setText(self.hint('OFFLINE'))

    def on_status(self, s):
        self.last_status_t = time.monotonic()
        state = s.get('state', '?')
        self.state = state
        self.set_banner(state, s.get('message', ''), self.hint(state))
        if (state, s.get('message')) != self.last_state_logged:
            self.last_state_logged = (state, s.get('message'))
            self.add_log(f'{state}: {s.get("message", "")}')
        self.apply_state(state)

        arms = s.get('arms', {})
        modes, conn, gello, grip = [], [], [], []
        for arm in ARMS:
            a = arms.get(arm, {})
            rows = self.arm[arm]
            mode = a.get('robot_mode', '-')
            rows['Robot'].setText(mode)
            errs = a.get('errors', [])
            rows['Errors'].setText(', '.join(errs) if errs else 'none')
            rows['Errors'].setStyleSheet(f"background:transparent; font-size:16px; "
                                         f"color:{T.SAFE_RED if errs else T.BLACK};")
            rows['Teleop'].setText(a.get('gate', '-'))
            rows['GELLO'].setText(a.get('gello', '-'))
            rows['Gripper'].setText(a.get('gripper', '-'))
            modes.append(mode)
            conn.append(a.get('connected', False))
            gello.append(a.get('gello', 'no data'))
            grip.append(a.get('gripper', 'off'))

        if any(m in ('REFLEX', 'USER_STOPPED') for m in modes) or any(arms.get(a, {}).get('errors') for a in ARMS):
            set_light(self.lights['Robots'], RED, ', '.join(modes))
        elif all(m in ('IDLE', 'MOVE') for m in modes):
            set_light(self.lights['Robots'], GREEN, ', '.join(modes))
        else:
            set_light(self.lights['Robots'], GREY if state == 'OFFLINE' else AMBER, ', '.join(modes))
        set_light(self.lights['FCI'], GREEN if all(conn) else (GREY if state == 'OFFLINE' else RED))
        set_light(self.lights['GELLO'], GREEN if all(g in ('in holder', 'out of holder') for g in gello)
                  else (GREY if state == 'OFFLINE' else RED), ', '.join(gello))
        set_light(self.lights['Grippers'], GREEN if all(g == 'ready' for g in grip)
                  else (GREY if all(g == 'off' for g in grip) else RED), ', '.join(grip))
        fps = s.get('cameras', {})
        cam_launch = s.get('cameras_launch', 'off')
        good = [v for v in fps.values() if v >= 20]
        set_light(self.lights['Cameras'], GREY if cam_launch == 'off' else
                  RED if cam_launch == 'stopped' or not good else GREEN if len(good) == len(fps) else AMBER,
                  ', '.join(f'{k} {v:.0f} fps' for k, v in fps.items()))
        set_light(self.lights['Pedal'], GREEN if s.get('pedal') == 'connected' else GREY, s.get('pedal', ''))


class CameraWindow(ConfirmClose, QWidget):
    def __init__(self, ros, preview=False):
        super().__init__()
        self.ros = ros
        self.setWindowTitle('Robot demo - CAMERAS' + (' (PREVIEW)' if preview else ''))
        self.setStyleSheet(f'background:{T.DARK_BLUE}; color:{T.OFF_WHITE};')
        self.views, self.labels, self.shown = {}, {}, {}
        outer = QVBoxLayout(self)
        outer.setContentsMargins(48, 40, 48, 32)
        top = QHBoxLayout()
        top.addWidget(T.logo(negative=True, width=230), 0, Qt.AlignLeft | Qt.AlignVCenter)
        top.addStretch(1)
        ct = QLabel('LIVE CAMERAS' + ('  ·  PREVIEW' if preview else ''))
        ct.setStyleSheet(f"font-family:'{T.HEAD}'; font-size:28px; font-weight:600; color:{T.WHITE};")
        top.addWidget(ct)
        outer.addLayout(top)
        outer.addSpacing(20)
        grid = QGridLayout()
        grid.setSpacing(16)
        outer.addLayout(grid, 1)
        # ZED Mini large on the left, the two wrist D405s stacked on the right
        cells = {'zed': (0, 0, 2, 1), 'd405_1': (0, 1, 1, 1), 'd405_2': (1, 1, 1, 1)}
        for title, key, _ in CAMERAS:
            t = QLabel(title.upper())
            t.setStyleSheet(f"font-family:'{T.MONO}'; font-size:15px; color:{T.OFF_WHITE};")
            view = QLabel('NO SIGNAL')
            view.setAlignment(Qt.AlignCenter)
            view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            view.setMinimumSize(160, 120)
            view.setStyleSheet(f"background:#0E1A1E; color:{T.GREY}; border-radius:14px; "
                               f"font-family:'{T.HEAD}'; font-weight:600; font-size:14px;")
            tile = QWidget()
            tl = QVBoxLayout(tile)
            tl.setContentsMargins(0, 0, 0, 0)
            tl.setSpacing(8)
            tl.addWidget(t)
            tl.addWidget(view, 1)
            grid.addWidget(tile, *cells[key])
            self.views[key], self.labels[key], self.shown[key] = view, t, None
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        self.t_fps = time.monotonic()
        self.last_counts = dict(ros.counts)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)                                 # ~30 Hz, latest frame only

    def refresh(self):
        now = time.monotonic()
        with self.ros.lock:
            frames = dict(self.ros.frames)
            ages = {k: now - t for k, t in self.ros.frame_t.items()}
            counts = dict(self.ros.counts)
        for title, key, _ in CAMERAS:
            view = self.views[key]
            if ages[key] > 1.0:
                if self.shown[key] is not None:
                    view.setText('NO SIGNAL')
                    self.shown[key] = None
                continue
            data = frames[key]
            if data is None or data is self.shown[key]:
                continue
            # decode straight to the display size (libjpeg DCT scaling) - much cheaper than full 1080p
            buf = QBuffer()
            buf.setData(QByteArray(data))
            buf.open(QIODevice.ReadOnly)
            reader = QImageReader(buf, b'jpeg')
            full = reader.size()
            if full.isValid():
                reader.setScaledSize(full.scaled(view.size(), Qt.KeepAspectRatio))
            img = reader.read()
            if img.isNull():
                continue
            view.setPixmap(QPixmap.fromImage(img))
            self.shown[key] = data
        if now - self.t_fps >= 1.0:
            for title, key, _ in CAMERAS:
                fps = (counts[key] - self.last_counts[key]) / (now - self.t_fps)
                self.labels[key].setText(f'{title.upper()}   ·   {fps:4.1f} FPS')
            self.last_counts, self.t_fps = counts, now


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
        self.t0 = time.monotonic()
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

    def error(self):
        self.set('ERROR', 'right: motion aborted by robot reflex (cartesian_reflex) - robots stopped by the Franka. '
                          'Check the robots (release E-stop), then SHUTDOWN and BRINGUP')

    def tick(self):
        up = self.state not in ('OFFLINE', 'SHUTTING_DOWN')
        live = self.state in ('TELEOP', 'TELEOP_STARTING', 'HOMING')
        arms = {a: {'robot_mode': ('MOVE' if live else 'IDLE') if up and self.state != 'ERROR' else 'no data',
                    'errors': [], 'connected': up and self.state != 'ERROR',
                    'gate': {'TELEOP': 'LIVE', 'TELEOP_STARTING': 'WAIT_STILL: hold GELLO still',
                             'HOMING': 'MOVING'}.get(self.state, 'not running'),
                    'gello': ('out of holder' if live else 'in holder') if up else 'no data',
                    'gripper': 'ready' if up else 'off'} for a in ARMS}
        st = {'state': self.state, 'message': self.msg, 'arms': arms, 'pedal': 'connected',
              'cameras_launch': 'running' if up else 'off',
              'cameras': {'zed': 29.8, 'd405_1': 30.0, 'd405_2': 29.9} if up else {}}
        self.ros.status.emit(st)
        self.ros.desk.emit({**{a: dict(v) for a, v in self.arm.items()}, 'busy': '', 'last': self.last})

    def frames(self):
        if self.state in ('OFFLINE', 'SHUTTING_DOWN'):
            return
        now = time.monotonic()
        with self.ros.lock:
            for title, key, _ in CAMERAS:
                if key not in self.frame_imgs:
                    self.frame_imgs[key] = self.fake_jpeg(title)
                self.ros.frames[key] = self.frame_imgs[key]
                self.ros.counts[key] += 1
                self.ros.frame_t[key] = now

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
    ap = argparse.ArgumentParser()
    ap.add_argument('--single', action='store_true', help='both views as tabs in one window')
    ap.add_argument('--control-screen', type=int, default=0)
    ap.add_argument('--camera-screen', type=int, default=1)
    ap.add_argument('--owns-demo', action='store_true',
                    help='started by start_demo.sh: closing a window asks, then shuts the demo down')
    ap.add_argument('--preview', action='store_true',
                    help='design preview with demo data; never sends anything (own ROS domain 199)')
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
    app.setStyleSheet(T.stylesheet())
    ConfirmClose.owns_demo = a.owns_demo
    app.setQuitOnLastWindowClosed(not a.owns_demo)
    feeder = PreviewFeeder() if a.preview else None
    ros = Ros(preview=feeder)
    control, cameras = ControlWindow(ros, a.preview), CameraWindow(ros, a.preview)
    if feeder:
        feeder.attach(ros)
        QShortcut(Qt.Key_F9, control, activated=feeder.error)
    screens = app.screens()
    if a.single or len(screens) < 2:
        tabs = type('Tabs', (ConfirmClose, QTabWidget), {})()
        tabs.setWindowTitle('Robot demo' + (' (PREVIEW)' if a.preview else ''))
        tabs.addTab(control, 'CONTROL')
        tabs.addTab(cameras, 'CAMERAS')
        place(tabs, screens[min(a.control_screen, len(screens) - 1)])
        keep = [tabs]
    else:
        place(control, screens[min(a.control_screen, len(screens) - 1)])
        place(cameras, screens[min(a.camera_screen, len(screens) - 1)])
        keep = [control, cameras]
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    tick = QTimer()                      # lets Python handle Ctrl-C while Qt runs
    tick.timeout.connect(lambda: None)
    tick.start(200)
    rc = app.exec_()
    del keep
    ros.shutdown()
    rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
