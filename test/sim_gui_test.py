#!/usr/bin/env python3
"""End-to-end SIM test of the GUI (scripts/demo_gui.py) against the real demo_supervisor on fake hardware.
Run via test/sim_gui_test.sh (ROS_DOMAIN_ID=77, localhost only, offscreen Qt). No robot connection.

The GUI is built in-process with both windows (control + camera display). The test clicks the GUI's own
buttons (Operator menu), so the whole chain button -> service -> supervisor -> gates -> status JSON -> GUI is
exercised. Checks page switching, enable rules, problem texts; saves a screenshot of every step.
"""
import os
import signal
import subprocess
import sys
import threading
import time

import yaml

DEMO = os.path.expanduser('~/robot_demo')
sys.path.insert(0, os.path.join(DEMO, 'scripts'))
sys.path.insert(0, os.path.join(DEMO, 'test'))

import rclpy  # noqa: E402
from PyQt5.QtCore import QObject, pyqtSignal  # noqa: E402
from PyQt5.QtGui import QFont  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402

import demo_gui as G  # noqa: E402
import franka_theme as T  # noqa: E402
from sim_gate_driver import MOCK_BASE, SIM_HOLDER, Arm, Results, safety_check  # noqa: E402

SUP = os.path.join(DEMO, 'scripts', 'demo_supervisor.py')
CM_PATTERN = r'^/opt/ros/humble/lib/controller_manager/ros2_control_node'


class Gui(QObject):
    """Run callables in the Qt thread from the test thread and wait for the result."""
    run = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.run.connect(lambda job: job())

    def call(self, fn, timeout=5.0):
        done, box = threading.Event(), {}

        def job():
            try:
                box['v'] = fn()
            except Exception as e:  # noqa: BLE001
                box['e'] = e
            done.set()
        self.run.emit(job)
        if not done.wait(timeout):
            raise TimeoutError('GUI thread did not answer')
        if 'e' in box:
            raise box['e']
        return box.get('v')


def sim_pids(pattern):
    out = []
    for pid in subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split():
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if b'ROS_DOMAIN_ID=77\0' in f.read():
                    out.append(int(pid))
        except OSError:
            pass
    return out


def scenario(gui, win, cam, model, arms, logdir, sim_cfg, R, app):
    shots = os.path.join(logdir, 'shots')
    os.makedirs(shots, exist_ok=True)
    holder = os.path.join(logdir, 'sim_holder_poses.yaml')
    with open(holder, 'w') as f:
        yaml.safe_dump({'left': SIM_HOLDER, 'right': SIM_HOLDER}, f)
    home = os.path.join(logdir, 'sim_home_poses.txt')
    with open(home, 'w') as f:
        f.write('0.0.0.0 ' + ' '.join(f'{q:.6f}' for q in MOCK_BASE) + '\n')
    log = open(os.path.join(logdir, 'supervisor.log'), 'a')
    sup = subprocess.Popen([sys.executable, SUP, '--sim', '--robot-config', sim_cfg, '--no-grippers',
                            '--no-gello', '--no-cameras', '--holder-file', holder, '--home-file', home,
                            '--lock-file', os.path.join(logdir, 'home.lock'),
                            '--gate-args', f'--holder-file {holder} --wait-timeout 10'],
                           stdout=log, stderr=subprocess.STDOUT)
    op = win.operator

    def gello(q):
        for a in arms.values():
            a.set_gello(lambda t, q=q: q)

    def state():
        return gui.call(lambda: model.state)

    def wait_state(want, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if state() == want:
                return True
            time.sleep(0.1)
        return False

    def page(w):
        return gui.call(lambda: {w.HOME: 'HOME', w.LIVE: 'LIVE', w.OPERATOR: 'OPERATOR'}.get(
            w.stack.currentIndex(), '?'))

    def shot(name):
        time.sleep(0.6)                                   # let the 1 s staleness timer / blink settle
        gui.call(lambda: win.grab().save(os.path.join(shots, name + '.png')))
        gui.call(lambda: cam.grab().save(os.path.join(shots, name + '_cam.png')))

    def click(button_attr, section=0):
        def f():
            win.show_operator()
            b = getattr(op, button_attr)
            if not b.isEnabled():
                print(f'    (click {button_attr}: disabled; state {model.state}, connected {model.connected}, '
                      f'visible {b.isVisible()}, page {win.stack.currentIndex()}/{op.stack.currentIndex()})',
                      flush=True)
                return False
            b.click()
            return True
        return gui.call(f)

    def enabled(*attrs):
        def f():
            here = win.stack.currentIndex()
            win.show_operator()                          # buttons are refreshed while the Operator page shows
            out = {a: getattr(op, a).isEnabled() for a in attrs}
            if here != win.OPERATOR:
                win.go_auto()
            return out
        return gui.call(f)

    def issues():
        return gui.call(lambda: ' | '.join(t for _l, t in model.issues()))

    gello(SIM_HOLDER)
    sc = 'G1 supervisor up, OFFLINE'
    print(f'\n== {sc}', flush=True)
    end = time.monotonic() + 30
    while time.monotonic() < end and not gui.call(lambda: model.connected):
        time.sleep(0.1)
    R.check(sc, 'GUI connected to the supervisor, sees OFFLINE',
            gui.call(lambda: model.connected) and wait_state('OFFLINE', 5), str(state()))
    R.check(sc, 'control display HOME, camera display LIVE', page(win) == 'HOME' and page(cam) == 'LIVE',
            f'{page(win)} / {page(cam)}')
    R.check(sc, 'START/HOME disabled, STOP enabled, BRING UP enabled',
            enabled('b_start', 'b_home', 'b_stop', 'b_bringup') ==
            {'b_start': False, 'b_home': False, 'b_stop': True, 'b_bringup': True},
            str(enabled('b_start', 'b_home', 'b_stop', 'b_bringup')))
    shot('G1_offline')

    sc = 'G2 BRING UP button -> IDLE'
    print(f'\n== {sc}', flush=True)
    R.check(sc, 'BRING UP clicked', click('b_bringup', 4))
    R.check(sc, 'IDLE', wait_state('IDLE', 90), str(state()))
    gui.call(win.go_auto)
    R.check(sc, 'START/HOME/STOP enabled, BRING UP disabled',
            enabled('b_start', 'b_home', 'b_stop', 'b_bringup') ==
            {'b_start': True, 'b_home': True, 'b_stop': True, 'b_bringup': False},
            str(enabled('b_start', 'b_home', 'b_stop', 'b_bringup')))
    R.check(sc, 'control display back on HOME', page(win) == 'HOME', page(win))
    shot('G2_idle')

    sc = 'G3 START button -> live page -> TELEOP'
    print(f'\n== {sc}', flush=True)
    R.check(sc, 'START clicked', click('b_start', 0))
    R.check(sc, 'TELEOP_STARTING', wait_state('TELEOP_STARTING', 10), str(state()))
    time.sleep(1.5)
    R.check(sc, 'control display switched to LIVE by the state', page(win) == 'LIVE', page(win))
    shot('G3_teleop_starting')
    gello(MOCK_BASE)                                  # take both GELLOs out
    R.check(sc, 'TELEOP', wait_state('TELEOP', 25), str(state()))
    R.check(sc, 'START/HOME disabled, STOP enabled in TELEOP',
            enabled('b_start', 'b_home', 'b_stop') == {'b_start': False, 'b_home': False, 'b_stop': True})
    shot('G3_teleop')

    sc = 'G4 STOP button -> IDLE -> HOME page'
    print(f'\n== {sc}', flush=True)
    R.check(sc, 'STOP clicked (Operator > Testing stage)', click('b_stop', 0))
    R.check(sc, 'IDLE', wait_state('IDLE', 20), str(state()))
    gui.call(win.go_auto)
    R.check(sc, 'control display HOME after STOP', page(win) == 'HOME', page(win))
    shot('G4_stopped')

    sc = 'G5 HOME button -> HOMING -> IDLE'
    print(f'\n== {sc}', flush=True)
    gello(SIM_HOLDER)
    R.check(sc, 'HOME clicked', click('b_home', 0))
    R.check(sc, 'HOMING', wait_state('HOMING', 10), str(state()))
    gui.call(win.go_auto)
    R.check(sc, 'control display not on LIVE while HOMING', page(win) != 'LIVE', page(win))
    shot('G5_homing')
    R.check(sc, 'IDLE again', wait_state('IDLE', 40), str(state()))
    R.check(sc, 'message both arms at home', 'both arms at home' in gui.call(lambda: model.message))

    sc = 'G6 arm crash during TELEOP -> ERROR text -> ERROR RECOVERY button -> IDLE'
    print(f'\n== {sc}', flush=True)
    click('b_start', 0)
    gello(MOCK_BASE)
    R.check(sc, 'TELEOP', wait_state('TELEOP', 25), str(state()))
    pids = sim_pids(CM_PATTERN + r' --ros-args -r __ns:=/right ')
    R.check(sc, 'one sim right arm CM found', len(pids) == 1, str(pids))
    for pid in pids:
        os.kill(pid, signal.SIGABRT)
    R.check(sc, 'ERROR', wait_state('ERROR', 30), str(state()))
    time.sleep(1.5)
    R.check(sc, 'control display left LIVE on ERROR', page(win) != 'LIVE', page(win))
    txt = issues()
    R.check(sc, 'problem text names the right arm and ERROR RECOVERY', 'right' in txt and 'ERROR RECOVERY' in txt,
            txt)
    R.check(sc, 'ERROR RECOVERY + STOP enabled, START disabled',
            enabled('b_recover', 'b_stop', 'b_start') == {'b_recover': True, 'b_stop': True, 'b_start': False})
    shot('G6_error')
    gello(SIM_HOLDER)
    R.check(sc, 'ERROR RECOVERY clicked', click('b_recover', 1))
    R.check(sc, 'IDLE after recovery', wait_state('IDLE', 90), str(state()))
    gui.call(win.go_auto)
    shot('G6_recovered')

    sc = 'G7 SHUT DOWN button -> OFFLINE; supervisor gone -> "no connection"'
    print(f'\n== {sc}', flush=True)
    R.check(sc, 'SHUT DOWN clicked', click('b_shutdown', 4))
    R.check(sc, 'OFFLINE', wait_state('OFFLINE', 60), str(state()))
    sup.send_signal(signal.SIGINT)
    try:
        sup.wait(timeout=30)
    except subprocess.TimeoutExpired:
        sup.kill()
    time.sleep(4.5)
    txt = issues()
    R.check(sc, 'GUI says no connection', 'No connection' in txt, txt)
    R.check(sc, 'STOP still enabled without connection', enabled('b_stop') == {'b_stop': True})
    shot('G7_no_connection')
    R.check(sc, 'no sim ros2_control_node left', not sim_pids(CM_PATTERN))
    gui.call(app.quit)


def main():
    logdir, sim_cfg = sys.argv[1], sys.argv[2]
    safety_check(sim_cfg)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1])
    T.load_fonts()
    app.setFont(QFont(T.TEXT, 11))
    G.BLINK = G.Blinker()
    ros = G.Ros()
    hub, model = G.Hub(), G.Model(ros)
    decoder = G.Decoder(ros, hub)
    win = G.MainWindow(model, ros, hub, decoder, False, 'main')
    cam = G.MainWindow(model, ros, hub, decoder, False, 'camera')
    for w in (win, cam):
        w.resize(1920, 1080)
        w.show()
    node = Node('sim_gui_test')
    arms = {a: Arm(node, a) for a in ('left', 'right')}
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    gui = Gui()
    R = Results()
    t = threading.Thread(target=scenario, args=(gui, win, cam, model, arms, logdir, sim_cfg, R, app), daemon=True)
    t.start()
    app.exec_()
    t.join(timeout=5)
    for a in arms.values():
        a.set_gello(None)
    code = R.summary()
    ex.shutdown()
    ros.shutdown()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
