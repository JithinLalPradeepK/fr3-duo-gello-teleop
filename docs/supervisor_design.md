# demo_supervisor — design proposal

Status: DECIDED 2026-11-20 — A persistent bringup: yes; HOME: B1 (gate target mode); status: JSON;
pedals 1 START / 2 STOP / 3 HOME; START always both arms. Implementation: scripts/demo_supervisor.py. Builds only on what is verified on hardware:
gated teleop (left, right, duo, duo + grippers), holder rule, Ctrl-C → deactivate → robots hold.

## 1. Goal

One ROS 2 node, `demo_supervisor`, owns all demo logic. The GUI and the foot-pedal node are thin
clients: they call the same services and read the same status, so a button and a pedal can never
behave differently.

| Command | Meaning | Allowed from |
|---|---|---|
| START  | gated GELLO teleop on both arms (holder rule, ramp, LIVE) | IDLE |
| STOP   | deactivate arm controllers → robots hold where they are | always (never refused) |
| HOME   | both FR3s to the saved home pose | IDLE only (never during teleop) |
| RECOVER| Franka automatic error recovery after reflex / E-stop release | ERROR, IDLE |

The GUI is not a safety device; the physical E-stop stays in use.

## 2. Key decision A — persistent bringup (recommended)

Today `teleop_duo.sh` starts *everything* per run (arms ~3–15 s, grippers incl. finger activation
cycle, GELLO, gates) and tears it all down on Ctrl-C.

**Proposal:** the supervisor keeps a persistent *bringup* running while the demo is on:

- `arms_gated.launch.py` (both arms, `joint_impedance_controller` loaded **inactive**)
- both Robotiq grippers (activation cycle once, at bringup only)
- `gello_check.launch.py` (GELLO on `/gello_check_*` only)
- `cameras.launch.py`

START / STOP then only start / stop the two **gate processes**:

- START → robot moving in ~2 s instead of ~20 s; no finger cycle on every START.
- Robot mode, errors and joint states are visible all the time (GUI status lights work while idle).
- STOP = SIGINT to the gates = the path verified on hardware (deactivate → hold).

Bringup itself is a supervisor command (BRINGUP / SHUTDOWN, GUI only, not on a pedal).
`teleop_duo.sh` stays as the manual fallback and test tool.

## 3. Key decision B — how HOME moves the robots

`fr3_duo_home` needs exclusive FCI; while the bringup holds FCI it cannot connect.

| Option | How | Pro | Con |
|---|---|---|---|
| **B1 (recommended)** | gate "target mode": `gello_gate.py --arm X --target-file home` ramps the impedance controller to the saved home pose (same fresh-instance + snapshot-relay path as START), waits until converged, deactivates, exits | reuses the hardware-tested ramp path; no FCI hand-over; HOME takes only the move time | new gate mode needs sim + hardware test; speed fixed by the controller (factor 0.2) |
| B2 | supervisor stops the arm launch, runs `fr3_duo_home --yes`, restarts the arm launch | uses the existing, verified homing tool | ~20–30 s extra per HOME; FCI hand-over each time; status gap |

Home poses stay in `~/fr3_tools/fr3_duo_home/home_poses.txt` (per robot IP), read by both options,
so `--save` keeps working. B2 remains available as fallback when the bringup is down.

## 4. States

```
OFFLINE ──BRINGUP──▶ STARTING_UP ──all ready──▶ IDLE
IDLE ──START──▶ TELEOP_STARTING (gates: WAIT_STILL/MOVING/HOLD) ──both LIVE──▶ TELEOP
TELEOP / TELEOP_STARTING ──STOP or gate ended──▶ STOPPING ──gates exited──▶ IDLE
IDLE ──HOME──▶ HOMING ──both converged & deactivated──▶ IDLE   (STOP during HOMING → hold, IDLE)
any ──robot REFLEX / USER_STOPPED / component died──▶ ERROR ──RECOVER (+ cause cleared)──▶ IDLE
IDLE / ERROR ──SHUTDOWN──▶ OFFLINE
```

Rules:
- One command at a time; **STOP preempts everything** and is idempotent.
- If one arm's gate refuses / faults, the other is stopped (as `teleop_duo.sh` does today).
- Teleop with one arm only: optional `arms` parameter (default both).
- While in IDLE the gate is not running → nothing publishes `/<arm>/gello/joint_states`
  (the gate's exclusive-publisher guard stays the last line of defence).

## 5. ROS interface

All under `/demo/`. Commands are `std_srvs/srv/Trigger` and return immediately with
`success` = accepted / refused and a plain-words `message`; progress is reported in status.

| Service | Notes |
|---|---|
| `/demo/start`, `/demo/stop`, `/demo/home`, `/demo/recover` | pedal + GUI |
| `/demo/bringup`, `/demo/shutdown` | GUI only |
| `/demo/gripper/{left,right}/{open,close}` | GUI; refused during TELEOP (GELLO owns the gripper) |

Status: `/demo/status` — `std_msgs/String` containing JSON, 2 Hz + on change, transient-local.
(A custom message package is cleaner but needs a new colcon package; JSON is enough for GUI/pedal
and can be swapped later.) Content:

```json
{"state": "TELEOP", "message": "both arms LIVE", "since": 1795157592.3,
 "arms": {"left": {"connected": true, "robot_mode": "MOVE", "errors": [],
                    "gate": "LIVE", "gello": "out of holder", "gripper": "ready"}, "right": {...}},
 "cameras": {"zed": 30.1, "d405_1": 30.0, "d405_2": 30.0}, "pedal": "connected",
 "last_event": "START by pedal"}
```

Robot errors come from `/<arm>/franka_robot_state_broadcaster/robot_state` (`robot_mode`,
`current_errors`, `last_motion_errors`, 36 named flags) and are translated to plain words
(e.g. `joint_position_limits_violation` → "joint limit reached").
RECOVER calls the `franka_msgs/action/ErrorRecovery` action of franka_hardware's `action_server`
node (exact name `/<arm>/action_server/error_recovery` to be confirmed on the running stack).

## 6. Safety design

- **Never auto-move.** No automatic restart of anything that can move (gates, HOME). Bringup
  restarts only on explicit command.
- **E-stop / reflex:** `robot_mode` REFLEX or USER_STOPPED → STOP gates, state ERROR; START refused
  until RECOVER succeeded and mode is IDLE.
- **Homing lock:** the supervisor holds `~/fr3_tools/.home.lock` while the bringup runs, so the
  GNOME pedal shortcut / manual `fr3_duo_home` cannot run in parallel (`home_fr3s.sh` also refuses
  because `ros2_control_node` runs).
- **Supervisor dies:** everything is started from the supervisor's main thread with `PR_SET_PDEATHSIG`:
  gates get SIGTERM → deactivate their controllers (verified path, robots hold); launches get SIGINT →
  clean shutdown (changed from "bringup keeps running": an orphaned stack would hold FCI after the
  homing lock is released).
- **Process hygiene:** every launch via `scripts/sigint_default`; anchored pgrep; logs per session
  under `logs/demo_<time>/`.
- **Pedal:** START/HOME on key-down with ~300 ms debounce; STOP is never debounced or ignored.
  Proposed mapping: pedal 1 START, pedal 2 STOP, pedal 3 HOME (to confirm).

## 7. Files (all in ~/robot_demo, nothing in gello_software)

- `scripts/demo_supervisor.py` — rclpy node, process manager + state machine.
- `scripts/gello_gate.py` — add target mode for HOME (option B1), shown as a diff before change.
- `launch/demo_bringup.launch.py` — optional single entry point (supervisor + cameras).
- `test/sim_supervisor_test.py` — sim scenarios on `ROS_DOMAIN_ID=77`, fake GELLO, `--no-grippers`.

## 8. Test plan

1. **Sim** (domain 77, fake hardware): bringup → IDLE; START → both LIVE; STOP from every state;
   HOME refused during TELEOP; HOME in sim (B1 mock cannot move → expect move-timeout path);
   one gate refuses → other stopped; kill -9 the supervisor during TELEOP → gates deactivate;
   status JSON content; service refusals with reasons.
2. **Hardware, no motion:** bringup, status, robot mode/errors shown, IDLE.
3. **Hardware, motion (each with your OK):** START/STOP via service call; HOME (B1);
   E-stop during TELEOP → ERROR → release → RECOVER → START again.
4. Then the GUI and pedal node on top.

## 9. Questions (answered 2026-11-20, see Status)

1. Persistent bringup (decision A) — OK?
2. HOME via gate target mode (B1) or via `fr3_duo_home` with FCI hand-over (B2)?
3. Status as JSON string for now — OK?
4. Pedal mapping 1 START / 2 STOP / 3 HOME — correct?
5. START always both arms, or should the GUI allow single-arm teleop?
