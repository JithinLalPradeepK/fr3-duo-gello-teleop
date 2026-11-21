#!/usr/bin/env python3
"""Minimal Franka Desk client for FR3 system 5.x (endpoints taken from the robot's own Desk web app, 5.10.0).

Credentials come from the GNOME keyring (service robot_demo_franka_desk, user franka) - never from code/files.
  python3 franka_desk.py status [--arm left]          read-only: operating mode, brakes, FCI, control token
  python3 franka_desk.py run --steps fci_on,... [--hold s] [--release]   (CHANGES the robot - ask first)

Desk API used:
  POST /admin/api/login                       body {login, password}; password = base64("b0,b1,..") of
                                              sha256("<password>#<user>@franka") -> token (cookie "authorization")
  POST /admin/api/control-token/request       body {requestedBy}  -> {id, token}   (?force needs a button press)
  DELETE /admin/api/control-token             body {token}
  POST /desk/api/joints/unlock | /joints/lock                       (header X-Control-Token)
  POST /desk/api/operating-mode/execution | /programming            (header X-Control-Token)
  POST /desk/api/system/fci                   form token=base64(token)
  DELETE /admin/api/control-token/fci         body {token}
  WS   /desk/api/system/status, /admin/api/control-token, /desk/api/robot/status   (status streams)
The robots use a self-signed certificate on the isolated robot network -> TLS verification is off.
"""
import argparse
import base64
import hashlib
import json
import ssl
import sys
import time

import keyring
import requests
import urllib3
import websocket

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

KEYRING_SERVICE = 'robot_demo_franka_desk'
DEFAULT_USER = 'franka'
ROBOTS = {'left': '192.168.1.51', 'right': '192.168.1.50'}


def encode_password(user, password):
    digest = hashlib.sha256(f'{password}#{user}@franka'.encode()).digest()
    return base64.b64encode(','.join(str(b) for b in digest).encode()).decode()


class Desk:
    def __init__(self, ip, user=DEFAULT_USER, timeout=5.0):
        self.ip, self.user, self.timeout = ip, user, timeout
        self.s = requests.Session()
        self.s.verify = False
        self.auth = None
        self.control_token = None
        self.control_token_id = None

    def url(self, path):
        return f'https://{self.ip}{path}'

    def login(self):
        password = keyring.get_password(KEYRING_SERVICE, self.user)
        if not password:
            raise RuntimeError(f'no password in keyring for {KEYRING_SERVICE}/{self.user}')
        r = self.s.post(self.url('/admin/api/login'), timeout=self.timeout,
                        json={'login': self.user, 'password': encode_password(self.user, password)})
        if r.status_code != 200:
            raise RuntimeError(f'{self.ip}: login failed ({r.status_code} {r.text[:200]})')
        self.auth = r.text.strip().strip('"')
        self.s.cookies.set('authorization', self.auth)

    def take_control(self, requested_by='robot_demo', force=False):
        """Request the control token. Returns the HTTP response (token stored on success)."""
        r = self.s.post(self.url('/admin/api/control-token/request' + ('?force' if force else '')),
                        json={'requestedBy': requested_by}, timeout=self.timeout)
        if r.status_code == 200:
            j = r.json()
            self.control_token, self.control_token_id = j['token'], j['id']
        return r

    def release_control(self):
        if not self.control_token:
            return None
        r = self.s.delete(self.url('/admin/api/control-token'), json={'token': self.control_token},
                          timeout=self.timeout)
        if r.status_code == 200:
            self.control_token = self.control_token_id = None
        return r

    def _ctl(self, method, path, **kw):
        if not self.control_token:
            raise RuntimeError(f'{self.ip}: no control token')
        headers = kw.pop('headers', {})
        headers['X-Control-Token'] = self.control_token
        r = self.s.request(method, self.url(path), headers=headers, timeout=kw.pop('timeout', 15.0), **kw)
        if r.status_code != 200:
            raise RuntimeError(f'{self.ip}: {method} {path} -> {r.status_code} {r.text[:200]}')
        return r

    def unlock_brakes(self):
        return self._ctl('POST', '/desk/api/joints/unlock', timeout=30.0)

    def lock_brakes(self):
        return self._ctl('POST', '/desk/api/joints/lock', timeout=30.0)

    def set_mode(self, mode):                     # 'execution' | 'programming'
        assert mode in ('execution', 'programming')
        return self._ctl('POST', f'/desk/api/operating-mode/{mode}')

    def fci_on(self):
        return self._ctl('POST', '/desk/api/system/fci',
                         data={'token': base64.b64encode(self.control_token.encode()).decode()})

    def fci_off(self):
        return self._ctl('DELETE', '/admin/api/control-token/fci', json={'token': self.control_token})

    def status(self):
        """Compact status from /admin/api/system-status."""
        m = self.ws_first('/admin/api/system-status')
        saf, tok = m.get('safety') or {}, m.get('controlToken') or {}
        return {'mode': (m.get('derived') or {}).get('operatingMode'),
                'brakes': saf.get('brakeState'),
                'fci': tok.get('fciActive'),
                'token': tok.get('activeToken'),
                'token_request': tok.get('tokenRequest'),
                'estop_x31': (saf.get('safeInputState') or {}).get('x31'),
                'safety': saf.get('safetyControllerStatus')}

    def ws_first(self, path):
        """First message of a Desk status websocket (JSON)."""
        ws = websocket.create_connection(f'wss://{self.ip}{path}', timeout=self.timeout,
                                         sslopt={'cert_reqs': ssl.CERT_NONE},
                                         cookie=f'authorization={self.auth}')
        try:
            return json.loads(ws.recv())
        finally:
            ws.close()


STEPS = {'fci_on': Desk.fci_on, 'fci_off': Desk.fci_off, 'unlock': Desk.unlock_brakes, 'lock': Desk.lock_brakes,
         'execution': lambda d: d.set_mode('execution'), 'programming': lambda d: d.set_mode('programming')}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('cmd', choices=['status', 'run'])
    ap.add_argument('--arm', choices=list(ROBOTS), action='append')
    ap.add_argument('--steps', default='', help=f'run: comma list of {sorted(STEPS)}')
    ap.add_argument('--hold', type=float, default=0.0, help='run: keep the control token this long (s) afterwards')
    ap.add_argument('--release', action='store_true', help='run: release the control token at the end')
    a = ap.parse_args()
    arms = a.arm or list(ROBOTS)
    desks = {}
    for arm in arms:
        desks[arm] = Desk(ROBOTS[arm])
        desks[arm].login()
    if a.cmd == 'status':
        for arm, d in desks.items():
            print(f'{arm:5s} {json.dumps(d.status())}')
        return 0
    steps = [x for x in a.steps.split(',') if x]
    bad = [x for x in steps if x not in STEPS]
    if bad:
        sys.exit(f'unknown steps {bad}')
    try:
        for arm, d in desks.items():
            r = d.take_control()
            print(f'{arm}: take control -> HTTP {r.status_code} {r.text[:120] if r.status_code != 200 else ""}')
            if r.status_code != 200:
                return 1
        for step in steps:
            for arm, d in desks.items():
                t = time.monotonic()
                STEPS[step](d)
                print(f'{arm}: {step} OK ({time.monotonic() - t:.1f} s)')
            time.sleep(1.0)
            for arm, d in desks.items():
                print(f'{arm}:   {json.dumps(d.status())}')
        end = time.monotonic() + a.hold
        while time.monotonic() < end:
            time.sleep(min(5.0, max(0.0, end - time.monotonic())))
            for arm, d in desks.items():
                print(f'{arm}: holding  {json.dumps(d.status())}')
    finally:
        if a.release:
            for arm, d in desks.items():
                r = d.release_control()
                print(f'{arm}: release -> {r.status_code if r is not None else "no token"}')
                time.sleep(0.5)
                print(f'{arm}:   {json.dumps(d.status())}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
