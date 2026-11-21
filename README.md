# FR3 Duo GELLO teleoperation demo

Exhibition demo: two Franka Research 3 arms with Robotiq 2F grippers, teleoperated with a
**GELLO Duo** leader device, plus a ZED Mini and two RealSense D405 cameras, a 3-pedal foot switch
and a PyQt5 GUI on two monitors. It runs as one ROS 2 Humble system on one Ubuntu 22.04 workstation
(no Docker).

> **Safety.** This software moves real robots. The GUI and the foot pedal are **not** safety devices.
> Always keep a physical E-stop in hand when robots are powered. Never run the arm controllers
> together with the GELLO publisher without the start gate (see [Start gate](#start-gate)).

## Architecture

```
 foot pedal ──► demo_pedal.py ─┐                         ┌─► arms_gated.launch.py (ros2_control, FR3 x2)
                               ├─► demo_supervisor.py ───┼─► gello_gate.py x2 (start gate per arm)
 GUI ─────────► demo_gui.py ───┘   (all demo logic,      ├─► gello_check.launch.py (GELLO publishers)
                    ▲               services + JSON      ├─► Robotiq gripper drivers x2
                    │               status)              ├─► cameras.launch.py (ZED Mini, 2x D405)
                    └── /demo/status, camera topics      └─► fr3 home (gate target mode)
 desk_agent.py: holds the Franka Desk control token; brakes / mode / FCI services
```

- **`demo_supervisor`** owns all logic. GUI and pedal are thin clients: a button and a pedal call
  the same ROS service, so they always behave the same.
- Commands: **START** (gated teleop, both arms), **STOP** (deactivate controllers, robots hold
  pose; never refused), **HOME** (both arms to the saved home pose; only when stopped),
  **BRINGUP / SHUTDOWN**, **RECOVER**.
- Pedals: 1 = START, 2 = STOP, 3 = HOME.

Details: [docs/supervisor_design.md](docs/supervisor_design.md).

## Repository layout

| Path | Contents |
|---|---|
| `scripts/start_demo.sh` | Starts the whole demo: desk_agent, supervisor, pedal, GUI. Closing the GUI shuts everything down. |
| `scripts/demo_supervisor.py` | Supervisor node (state machine, services, `/demo/status` JSON). |
| `scripts/demo_gui.py`, `scripts/franka_theme.py` | PyQt5 GUI (CONTROL + CAMERAS windows) and styling. |
| `scripts/demo_pedal.py` | Foot pedal node (evdev). |
| `scripts/desk_agent.py`, `scripts/franka_desk.py` | Franka Desk API client (brakes, Programming/Execution mode, FCI). |
| `scripts/gello_gate.py` | Start gate: safe hand-over from GELLO to the robot (one per arm). |
| `scripts/teleop_duo.sh` | Standalone gated teleop without supervisor/GUI (for testing). |
| `scripts/demo_cmd.sh` | CLI client: `demo_cmd.sh start / stop / home / status ...` |
| `scripts/gello_align.py`, `scripts/record_gello_holder.py` | Calibration helpers (GELLO vs. robot diff, holder pose recording). |
| `scripts/test_recorder.py` | Read-only event recorder for test sessions. |
| `launch/` | `arms_gated`, `gello_check`, `cameras` launch files. |
| `config/` | GELLO publisher config (isolated namespaces), holder poses, fake-hardware robot config. |
| `env/host_env.sh` | Sources all workspaces in the right order. |
| `test/` | Simulation tests (fake hardware, separate ROS domain). |
| `external/` | Our changes to third-party code (patches) and the standalone homing tool. See [external/README.md](external/README.md). |
| `system/` | udev rule (foot pedal), realtime limits, desktop entry. |
| `docs/` | Design notes, test reports, GUI developer guide. |
| `CLAUDE.md` | Detailed running log of the setup, decisions and lessons learned (most complete reference). |

## Hardware

- 2x Franka Research 3 (system 5.10.0, libfranka 0.18.2), FCI on a dedicated NIC (192.168.1.0/24).
  Left = 192.168.1.51, Right = 192.168.1.50 (left/right as seen from the front).
- 2x Robotiq 2F gripper over USB-RS485 (FTDI); GELLO Duo for Franka (2x OpenRB-150).
- ZED Mini (USB 3 required), 2x Intel RealSense D405.
- PCsensor foot switch (USB 3553:b001), programmed with ElfKey: pedal 1 = `A`, 2 = `B`, 3 = `F12`.

## Software setup

Tested on Ubuntu 22.04.5, kernel 6.8.1-realtime, ROS 2 Humble, NVIDIA driver 580 / CUDA 12.9.

1. **ROS 2 Humble** desktop, `ros-humble-ros2-control`, `ros-humble-rmw-cyclonedds-cpp`,
   `ros-humble-realsense2-camera`, `ros-humble-topic-tools`.
2. **`~/gello_deps_ws`** (build with `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`):

   | Package | Source | Commit |
   |---|---|---|
   | libfranka | github.com/frankarobotics/libfranka | `b9091305` (0.18.2) |
   | franka_ros2 | github.com/frankarobotics/franka_ros2 | `76a3418` (v2.1.0) |
   | franka_description | github.com/frankarobotics/franka_description | `1ccde30` (1.3.0) |
   | ros2_robotiq_gripper | github.com/PickNikRobotics/ros2_robotiq_gripper | `2ff8545` + `external/ros2_robotiq_gripper_2ff8545.patch` |
   | serial | github.com/tylerjw/serial | `d8d1606` |
3. **`~/gello_software`**: github.com/wuphilipp/gello_software at `204f53a`, apply
   `external/gello_software_204f53a.patch` (robot IPs, USB serials of our devices), build `ros2/`.
4. **`~/zed_ws`**: ZED SDK 5.4.1 + github.com/stereolabs/zed-ros2-wrapper (`1aa4249`).
5. **Python** (user pip): `dynamixel_sdk tyro pyserial evdev==1.6.1 keyring`; PyQt5 from apt.
6. **Homing tool**: build `external/fr3_duo_home` against libfranka 0.18.2 into
   `~/fr3_tools/fr3_duo_home/build`; copy `home_poses.txt` and `home_fr3s.sh` to `~/fr3_tools/`.
7. **System files** (`system/`): udev rule to `/etc/udev/rules.d/` (and a `pcsensorhid` group),
   realtime limits to `/etc/security/limits.d/`, desktop entry to `~/.local/share/applications/`.
8. Store the Franka Desk password in the GNOME keyring (never in code):
   `python3 -c "import keyring; keyring.set_password('robot_demo_franka_desk', 'franka', input('pw: '))"`

The repo is expected at `~/robot_demo`.

> **Your own hardware:** device serials, robot IPs, holder poses (`config/gello_holder_poses.yaml`) and
> home poses are specific to our setup. Change them before you use the code anywhere else.

## Running

```bash
~/robot_demo/scripts/start_demo.sh       # or the "Robot Demo" desktop icon
```

The demo starts **OFFLINE**. In the GUI: **PREPARE ALL** (unlock brakes, Execution mode, FCI on) →
**BRING UP** → **START** / **STOP** / **HOME**. To finish: **PARK**, then close the GUI.

After a robot reflex or an E-stop the arm stack is gone: release the E-stop, then **SHUTDOWN** and
**BRING UP**.

## Start gate

The stock FR3 joint impedance controller fixes its trajectory start when it is activated. If GELLO
data arrives later, the robot jumps to the GELLO pose at full stiffness. `gello_gate.py` prevents this:

1. The controller is loaded **inactive**. GELLO publishes on `/gello_check_<arm>/...`, so the
   controller can't hear it.
2. GELLO in its holder → the robot never moves. GELLO taken out and held still for 1 s → the gate
   takes a snapshot of the pose.
3. The gate relays the frozen snapshot, then activates a **fresh** controller instance, so the
   robot ramps smoothly to the snapshot.
4. HOLD until the live GELLO matches the snapshot (< 0.08 rad) → LIVE teleop.
5. On STOP, a failure or a signal, the gate deactivates the controller and the robot holds its pose.

## Tests (simulation, no robot needed)

```bash
test/sim_gate_test.sh all          # start gate against fake hardware
test/sim_supervisor_test.sh        # supervisor state machine
```

## Working on the GUI without robots

See [docs/GUI_DEVELOPMENT.md](docs/GUI_DEVELOPMENT.md). Short version:
`python3 scripts/demo_gui.py --preview --single` runs the GUI with demo data and sends nothing.

## Brand assets

The GUI uses the Franka Robotics logo. The logo is a trademark and is **not** in this repository.
Put the PNGs listed in [docs/HANDOFF_GUI.md](docs/HANDOFF_GUI.md) (`logo_black.png`, `logo_white.png`, `fr3_duo.png`, the two
`FrankaRobotics_Trademark_*` icons) into `assets/brand/`. Without them the logo and picture areas stay empty. The Montserrat fonts are included
under the SIL Open Font License (`assets/brand/fonts/OFL.txt`).

## Status

The hardware test passed for gated duo teleop with grippers, HOME, E-stop handling, the pedal and the
GUI. Open issues and next steps: [docs/test_notes_2026-11-20.md](docs/test_notes_2026-11-20.md)
(Cartesian reflexes during teleop, supervisor fixes S1–S11).
