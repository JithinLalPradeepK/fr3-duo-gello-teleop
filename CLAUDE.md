# Robot demo workstation: FR3 Duo + GELLO Duo + Robotiq + cameras + foot pedal

Context for Claude Code on WorkStation-1. Keep it up to date when the setup changes.

## Goal
One ROS 2 Humble system on this workstation (no Docker) that runs the whole demo:
GELLO Duo teleop of two FR3s with Robotiq 2F grippers, ZED Mini + 2x D405 cameras,
a 3-pedal foot switch, and a PyQt GUI on two monitors.
- GUI buttons and foot pedals behave identically:
  START (safe start of GELLO teleop via the start gate), STOP (stop teleop, robots hold where they are),
  HOME (both FR3s to the saved home pose, only when teleop is stopped).
- Architecture: a `demo_supervisor` ROS 2 node owns all logic and exposes services/status;
  the GUI and a pedal node are thin clients of it.

## SAFETY RULES (always follow)
- Never run anything that moves a robot or gripper (fr3_duo_home without --show, arm controllers,
  gripper launch, GELLO + controllers) without asking me first in that moment. I keep the E-stop in hand.
- Never start the FR3 arm controllers together with the GELLO publisher WITHOUT the start gate
  (see below): the stock controller jumps to the GELLO pose at full stiffness.
- Homing (FCI) and teleop (FCI) must never run at the same time.
- Never run `apt upgrade`/`autoremove`, touch the NVIDIA driver, realtime kernel, /etc/dkms,
  or ~/.bashrc without asking. Keep the stock 6.8.0-40-generic kernel (fallback).
- Never `sudo colcon build`. Don't change working code in place without showing me the diff.

## Machine
- AMD Threadripper 9970X (32c), 32 GB, RTX 5070 Ti 16 GB, Ubuntu 22.04.5, user `workstation-1`.
- Kernel 6.8.1-1064-realtime (Ubuntu Pro, linux-realtime-hwe-22.04). Livepatch disabled.
- NVIDIA 580.178.04 open driver (DKMS with IGNORE_PREEMPT_RT_PRESENCE=1 in /etc/dkms/framework.conf).
  CUDA 12.9 toolkit (NVIDIA apt repo added by the ZED installer). Single GPU drives the display.
- Groups: realtime (rtprio 99, memlock unlimited via /etc/security/limits.d/99-realtime.conf),
  dialout, video, plugdev.
- ROS 2 Humble desktop + ros2_control 2.54.x, cyclonedds, realsense2_camera 4.59.1 (apt).
- Python: system numpy 1.21 (apt). pip --user: dynamixel_sdk, tyro, pyserial, evdev 1.6.1.
  pyzed NOT installed on purpose (avoids numpy 2 conflict seen on the laptop).

## Robots and network
- Robot NIC enp97s0 = 192.168.1.10/24. Left FR3 = 192.168.1.51, Right FR3 = 192.168.1.50
  (left/right as seen from the front). Ping ~0.3 ms.
- libfranka must be 0.18.2 (robots' version). load_gripper "false" (Robotiq mounted).

## Workspaces (sourcing order)
1. /opt/ros/humble
2. ~/gello_deps_ws  — libfranka 0.18.2, franka_ros2 v2.1.0, franka_description 1.3.0,
   PickNik ros2_robotiq_gripper @2ff85455 (kGripperMaxSpeed patched 0.150 -> 1.0), serial. 19 pkgs OK.
   Built with -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF.
3. ~/gello_software/ros2 (commit 204f53a) — normal build/ install/ (no dev-container here).
4. ~/zed_ws — zed-ros2-wrapper (master), ZED SDK 5.4.1.
- ~/robot_demo/env/host_env.sh should source exactly these + RMW_IMPLEMENTATION=rmw_cyclonedds_cpp.

## GELLO config edits (in ~/gello_software/ros2/src, uncommitted — keep them)
- franka_fr3_arm_controllers/config/example_fr3_duo_config.yaml: LEFT .51, RIGHT .50, load_gripper "false".
- franka_gripper_manager/config/example_fr3_duo_config_robotiq.yaml:
  LEFT FTDI DABI6GMM, RIGHT FTDI DABI69PP.
- franka_gello_state_publisher/config/franka_gello_duo.yaml:
  LEFT OpenRB-150 55DFEA165157375037202020FF122B11, RIGHT 051C587A5157375037202020FF120E11.
  Default assembly_offsets are correct (pre-assembled Franka GELLO Duo; verified, no calibration needed).
  Note: our "left" uses the README's "right" sign pattern (j1 > 0) — naming is from the front; it matches the robots.

## Verified working
- fr3_duo_home --show on both robots (no motion yet on this machine).
- GELLO publisher: /left/gello/joint_states and /right/gello/joint_states at 25 Hz.
- Cameras at 30 Hz with real JPEG data:
  /zed/zed_node/rgb/color/rect/image/compressed (~0.5 MB/frame),
  /camera/d405_1/color/image_raw/compressed, /camera/d405_2/color/image_raw/compressed (~0.15 MB/frame).
  D405 serials: d405_1 = 262622274776, d405_2 = 262422272263 (launch serial_no with leading "_").
  ZED Mini must enumerate as 2b03:f682 "ZED-M camera" at 5000M (lsusb -t). If only 2b03:f681 (HID) shows on a
  480M bus: wrong USB-C orientation at the camera or USB2 port/hub -> "CAMERA STREAM FAILED TO START" (2026-11-20).
  Supervisor treats a camera failure as status only ("cameras_launch": stopped), never as ERROR.
- Both Robotiq grippers (2026-11-19): robotiq.launch.py namespace:=<arm>/gripper com_port:=<FTDI by-id>,
  activation cycle OK, target_gripper_width_percent 0.5 -> knuckle 0.499 rad, 1.0 -> 0.0 (open).
  Mapping: knuckle target = 1 - value (rad), closed = 0.79 -> 0.5 is ~63 % closed, 1.0 = fully open.
- Start gate SIM test (2026-11-19): test/sim_gate_test.sh -> 57/57 single + 15/15 duo checks passed.
- Not yet tested here: arm controllers on real robots, teleop, grippers inside teleop.

## Homing tool (standalone, no ROS)
- ~/fr3_tools/fr3_duo_home/build/fr3_duo_home (libfranka 0.18.2 in ~/fr3_tools/libfranka-install, RPATH).
  Home poses per robot IP in ~/fr3_tools/fr3_duo_home/home_poses.txt. Flags: --save, --show, --yes, --vel.
- ~/fr3_tools/home_fr3s.sh: flock (one homing at a time) + refuses if ros2_control_node/teleop runs.

## START GATE (rewritten 2026-11-19 in ~/robot_demo, sim-tested; design notes below)
Problem: franka_fr3_arm_controllers' joint impedance controller fixes its motion-generator start time at
activation; if GELLO data arrives later, the robot steps to the GELLO pose at full stiffness.
Design that worked:
- Launch arm controllers with joint_impedance_controller loaded INACTIVE (spawner --inactive);
  joint_state_broadcaster active so robot pose is readable.
- GELLO publisher publishes on /gello_check_<arm>/... (a copy of franka_gello_duo.yaml with different
  namespaces) so the controllers don't hear it directly.
- gello_gate node per arm: WAIT_STILL (GELLO OUT OF ITS HOLDER and still < 0.03 rad over 1.0 s) ->
  [RULE since 2026-11-20, user decision: GELLO in its holder -> robot never moves; GELLO out of the holder ->
  robot moves to the GELLO pose WHATEVER the distance, then teleop. The old 1.0 rad distance limit is off
  by default (--max-start-diff). "In holder" = every joint within 0.25 rad of config/gello_holder_poses.yaml
  (record with scripts/record_gello_holder.py --arm <arm>, GELLO resting in holder; gate refuses if missing).
  Snapshot within 0.1 rad of an FR3 joint limit -> REFUSED. Old gate: scripts/.backup/] -> snapshot GELLO pose, relay the frozen snapshot (re-stamped, 25 Hz) to
  /<arm>/gello/joint_states, then ACTIVATE the controller (smooth move, speed factor 0.2) -> when robot within
  converge tol of snapshot: HOLD -> when live GELLO within 0.08 rad of snapshot: LIVE (relay live GELLO + gripper
  target /<arm>/gripper/gripper_client/target_gripper_width_percent).
- On any failure, Ctrl-C or SIGTERM: deactivate the controllers it activated (robots hold pose).
  Handle SIGINT/SIGTERM yourself (rclpy's default handler shut the context too early on the laptop).
- Test against a simulated robot on a separate ROS_DOMAIN_ID first, then one arm, then duo.
Implementation facts learned on this machine (do not regress):
- Stock controller only resets its ramp flags in the constructor -> re-activating the same instance skips the
  ramp (jump). gello_gate unloads/loads/configures a FRESH instance before every activation.
- New instance subscribes in on_configure; the gate waits until DDS matched it (publisher subscription count)
  before activating, relays the snapshot at 100 Hz and stays in MOVING >= 0.5 s. Without this the controller
  missed the snapshot and initialised its ramp on live data (seen in sim).
- Background jobs of non-interactive bash start with SIGINT ignored; ros2 launch then ignores Ctrl-C.
  Always start launches via scripts/sigint_default. ROS setup.bash is not `set -u` safe (use set +u around it).
- Sim test with ROS_LOCALHOST_ONLY=1 needs test/cyclonedds_sim.xml (more participant indices on lo).
- Use anchored pgrep patterns (pgrep -f matches shell command text otherwise).

## ~/robot_demo layout
- GitHub (PUBLIC, 2026-11-21): github.com/JithinLalPradeepK/fr3-duo-gello-teleop, branch main. Colleagues work on the
  GUI (docs/GUI_DEVELOPMENT.md); user does demo fixes. Not in git: logs/, scripts/.backup/, docs/brand/, Franka logo
  files (trademark). external/ + system/ = copies of out-of-repo files - refresh them when those change.
- env/host_env.sh; launch/cameras.launch.py, arms_gated.launch.py (arms:=, robot_config:=),
  gello_check.launch.py; config/franka_gello_duo_check.yaml (keep in sync with franka_gello_duo.yaml),
  config/fr3_duo_sim.yaml (fake hardware); scripts/gello_gate.py (--arm, exit 0 stop/2 refused/3 fault/
  4 deactivation failed), scripts/teleop_duo.sh (--arms, --robot-config, --no-gello, --gate-args; holds
  ~/fr3_tools/.home.lock), scripts/sigint_default; test/sim_gate_test.sh [single|duo|all]; logs/.
- scripts/start_demo.sh (+ desktop entry), scripts/demo_gui.py, scripts/demo_pedal.py,
  scripts/demo_supervisor.py, scripts/demo_cmd.sh, docs/supervisor_design.md, test/sim_supervisor_test.sh;
  scripts/gello_align.py, scripts/record_gello_holder.py, config/gello_holder_poses.yaml; scripts/.backup/.

## STATUS / NEXT STEP (update as you go)
Done 2026-11-19: step 1 (env + cameras), step 2 (gate + sim test), gripper test both arms.
Done 2026-11-20: Stage A left (FCI OK, robot_mode idle, controller inactive, GELLO isolated on /gello_check_left,
  joint directions OK during alignment). Holder rule added to gate (see START GATE); left holder pose recorded.
  Tools: scripts/gello_align.py (live diff readout), scripts/record_gello_holder.py. Both holder poses recorded.
  Gate also refuses/faults if any other publisher is on /<arm>/gello/joint_states (sim: 91/91 + 15/15).
  Stage B LEFT PASSED (teleop_duo.sh --arms left): ramp goal == snapshot (10 ms after activation), HOLD->LIVE,
  ~19 s teleop, Ctrl-C -> robot held pose. Note: if GELLO is already out of holder at START, robot moves ~2 s
  after start (by design). teleop_duo.sh now uses tee -i (Ctrl-C had cut the gate log before STOPPED).
Stage A + B RIGHT PASSED (2026-11-20): holder detected (robot did not move), taken out -> ramp goal == snapshot,
  LIVE ~13 s, Ctrl-C -> STOPPING/deactivated/STOPPED logged, robot held pose.
DUO TEST PASSED (2026-11-20, teleop_duo.sh): both GELLOs held in holder (no motion), taken out -> both ramps
  goal == snapshot, both LIVE ~25 s, Ctrl-C -> both deactivated/STOPPED at the same instant, robots held.
DUO + GRIPPERS PASSED (2026-11-20): teleop_duo.sh now starts both Robotiq grippers (ports from
  franka_gripper_manager example_fr3_duo_config_robotiq.yaml; activation cycle at startup; --no-grippers for sim).
  ~51 s LIVE, ~60 gripper commands per arm from GELLO triggers, Ctrl-C -> both held. Cosmetic: stock
  robotiq_gripper_client dies with exit -2 on Ctrl-C (no SIGINT handling, in gello_software, left untouched).
  Observed: spawners sometimes ~13 s after FCI connect in duo (startup only).
SUPERVISOR (2026-11-20): design docs/supervisor_design.md (decided: persistent bringup, HOME via gate target mode
  --target-joints, JSON status, pedals 1 START/2 STOP/3 HOME, always both arms). scripts/demo_supervisor.py +
  scripts/demo_cmd.sh (CLI client). SIM: test/sim_supervisor_test.sh 37/37 (incl. SIGKILL of supervisor during
  TELEOP -> gates deactivate within 3 ms, stack shuts down); gate suite still 91/91 + 15/15 after target mode.
  Gotcha: never name a Node method `handle` (hides rclpy Node.handle).
H1 PASSED (2026-11-20): supervisor bringup -> IDLE in ~4 s, robots IDLE no errors, GELLOs in holder, grippers
  ready, cameras ~30 fps.
H2 PASSED: demo_cmd start -> TELEOP (both LIVE, robot mode MOVE), stop -> both deactivated in 10 ms, IDLE.
H3 PASSED: first real HOME via gate target mode: ramps 0.7-1.0 s, DONE, final error <= 0.032 rad (j7, soft k=20).
  Bug found+fixed: home pose starting with '-' was parsed by argparse as an option -> always pass
  --target-joints=<values> (sim test now uses a home pose starting with -0.000000).
GUI (2026-11-20): scripts/demo_gui.py - CONTROL on DP-0, CAMERAS on DP-2 (3 streams ~30 fps), --single for tabs.
  Verified on screen. GUI ~47% CPU (Qt drawing), supervisor ~15% after the fix below.
  ros-humble-topic-tools installed (2 pkgs, 0 upgrades). Supervisor starts a C++ topic_tools throttle per arm:
  /<arm>/franka_robot_state_broadcaster/robot_state (1 kHz) -> /<arm>/robot_state_10hz. Python on 2x1 kHz cost ~70%.
DEFERRED by user: H4 (E-stop during TELEOP -> ERROR -> RECOVER). Do it before relying on RECOVER in the GUI.
PEDAL (2026-11-20): /etc/udev/rules.d/99-footswitch-event.rules (event* -> group pcsensorhid, 0660).
  ElfKey mapping: pedal 1 = KEY_A, 2 = KEY_B, 3 = KEY_F12 (with auto-repeat). scripts/demo_pedal.py grabs the
  pedal exclusively, key-down only, STOP never debounced, START/HOME 300 ms debounce, 1 Hz heartbeat
  /demo/pedal/status. Hardware test passed: START (in holder: no motion), HOME refused during start-up, STOP,
  HOME. Pedal 2 once sent 2 key-downs 80 ms apart (bounce?) - harmless for STOP. No GNOME pedal shortcut set.
  Gotcha: Node attribute names `handle` and `clients` are taken by rclpy (AST check done on all nodes).
WRAP-UP (2026-11-20): scripts/start_demo.sh starts supervisor (--bringup) + pedal + GUI (--owns-demo); refuses if
  already running; closing the GUI (confirm dialog) or Ctrl-C/SIGTERM -> pedal, then supervisor (gates first,
  robots hold, then stack). Logs logs/start_demo_<ts>/. Desktop entry ~/.local/share/applications/robot-demo.desktop
  ("Robot Demo", verified with gtk-launch: IDLE, pedal connected, both windows, 3 cameras ~30 fps).
Window-close shutdown + icon restart verified by user.
REFLEX INCIDENT (2026-11-20 14:56, logs/demo_20261120_145348): right FR3 "cartesian_reflex" during LIVE teleop.
  franka_hardware readOnceActiveControl does not catch franka::ControlException -> right ros2_control_node
  aborted (-6) -> launch shut down both arm stacks (left deactivated cleanly, robots stopped by Franka).
  => any reflex / E-stop during control kills the arm stack: RECOVER (action server) cannot work;
  recovery = SHUTDOWN + BRINGUP (initializeTorqueInterface does automaticErrorRecovery on activation; not yet
  verified on hardware). franka_hardware sets NO collision thresholds (robot's own config); Robotiq mass is
  not in the load model (load_gripper false) - candidate cause; SetLoad / thresholds = user decision, not done.
  Fix (sim T6b, suite now 48/48): supervisor reads the cause from arms.log ("right: motion aborted by robot
  reflex (cartesian_reflex) - ... SHUTDOWN and BRINGUP"), no false "use E-stop" alarm from gate exit 4 when the
  stack is gone, RECOVER refused after a crash. Pedal node: own SIGINT/SIGTERM handlers (no traceback).
  Backups scripts/.backup/*.20261120_crashmsg.
H4 PASSED (2026-11-20 16:41, logs/demo_20261120_163454): E-stop during TELEOP -> left ros2_control_node aborted
  ("User Stop pressed!"), launch stopped right cleanly, supervisor ERROR with the clear message (no false alarm).
  E-stops released -> GUI Shut down -> Bring up -> IDLE in 3 s, both robots IDLE, NO Error recovery needed
  (franka_hardware recovers on connect). START -> LIVE in 5 s, STOP -> both deactivated in 6 ms, robots held.
  => after a reflex / E-stop: release, SHUTDOWN, BRINGUP. RECOVER only for errors while the stack is alive.
Small items DONE (2026-11-20 17:20): (a) supervisor sends each signal to a child once (pedal STOP bounce no longer
  kills a gate mid-exit; backup .20261120_h4; sim 48/48); (b) d405_1 ~10 fps = USB 2.1 link (user's USB
  extension adapter) - accepted by user; (d) pedal.log clean at GUI close (no traceback).
(c) MEASURED 2026-11-20 17:35 (arm stack only, controllers inactive, robots at rest, 3 s avg):
  Desk end-effector mass is ALREADY 1.2 kg on both FR3s (load 0) -> Robotiq weight is compensated.
  Rest |F_ext| left 2.1 N, right 4.5 N (right torque ~2.6 Nm, cable pull?). => Do NOT call SetLoad
  (would double-count). The 14:56 right reflex was not a gripper-mass offset; most likely fast teleop motion
  or contact. Collision thresholds untouched (user decision if reflexes recur).
DESK API (2026-11-20, system 5.10.0 on both FR3s): scripts/franka_desk.py (status | run --steps ... --hold --release).
  Password in GNOME keyring: service robot_demo_franka_desk, user franka (never in code). Safety-operator login NOT stored.
  Endpoints read from the robot's own Desk JS bundle (see docstring). Status: WS /admin/api/system-status
  (derived.operatingMode, safety.brakeState, controlToken.fciActive/activeToken, safeInputState.x31).
  Tested on both robots: control token (no force) granted when Desk released it; FCI on OK; RELEASING THE TOKEN
  TURNS FCI OFF -> a process must hold the token while the demo runs; Programming<->Execution OK (safety Idle/Work);
  brakes lock 2.3 s / unlock ~6 s. If another session holds the token, taking it needs ?force + button on robot.
DESK IN THE DEMO (2026-11-20): scripts/desk_agent.py (started by start_demo.sh; holds the token; services
  /demo/desk/{prepare,park,unlock,lock,execution,programming,fci_on,fci_off}, refused while an arm stack runs;
  on exit releases control only -> FCI off, brakes/mode unchanged). Demo now starts OFFLINE (user decision:
  manual PREPARE / single buttons, then BRING UP). Supervisor refuses BRINGUP while desk_agent reports FCI off.
  GUI: purple "Robots via Franka Desk" box with 8 buttons + Desk light + per-arm Desk line. User: keep the
  "FCI" light name and the Bring up colour. Tested: Prepare, Park, Unlock, Execution, FCI on -> Bring up IDLE.
  Report for the user: docs/report_2026-11-20.md.
EXHIBITION GUI (2026-11-20, for DIDAC 2026): scripts/demo_gui.py redesigned + scripts/franka_theme.py (Franka
  brand guide docs/brand/: logo assets/brand/*.svg, Montserrat in assets/brand/fonts (OFL), Lato, DejaVu Sans Mono
  for RM Mono). Toggle switches for Brakes / Mode / FCI (+ PREPARE ALL, PARK). No "not a safety device" note (user).
  Safety colours kept (START green, STOP/ERROR red, motion amber). `demo_gui.py --preview` = design preview with
  demo data, ROS domain 199, sends nothing (F9 = demo ERROR). Backup .backup/demo_gui.py.20261120_brand.
  Supervisor logs "demo_supervisor running" at start -> start_demo.sh opens GUI at once.
GUI REDESIGN (2026-11-21, branch features/GUI-development, Akshay): demo_gui.py + franka_theme.py rewritten (white
  home without buttons, navbar chips, Live camera page mirrored on 2 monitors, Operator menu, light/dark, grey buttons,
  Franka trademark in empty camera tiles). Preview-tested only. See docs/GUI_CHANGES.md and docs/HANDOFF_GUI.md.
  Later 2026-11-21: 2 displays (1 = Home/Operator -> cameras on START, 2 = cameras always), camera page layout (ZED centre,
  wrists bottom corners, depth as picture-in-picture, slide status panel), Operator = one grid page, loading animation.
  Depth view: add the topic to CAMERAS in demo_gui.py. PNG brand assets stay out of git (copy into assets/brand/).
TEST SESSION 1 (2026-11-20 20:33-21:26): scripts/test_recorder.py (read-only) -> logs/test_20261120_203258/.
  Findings + suggestions S1-S11: docs/test_notes_2026-11-20.md. Key: 5 cartesian reflexes in ~5 min LIVE
  (hypothesis: 25 Hz GELLO steps relayed raw, K=240 -> torque steps), 1 self-collision reflex during ramp;
  supervisor bugs O8 (STOP in ERROR overwrites message), O9 (SHUTDOWN while gates exiting), O10 (HOME: DONE arm
  stops the other), O4 (consequence error listed as cause).
  Robots left: Programming mode, FCI off, no control holder.
S2-S6 DONE in sim (2026-11-21, branch fix/s2-s6-recorder-supervisor, sim 68/68 incl. new T6c, T8; diff shown):
  User: 20:43 double reflex = Rubik's cube held in one hand, other hand twisting a stuck layer (arms pushed against
  each other via the cube); one single reflex = gripper touched the table. Other 2 single reflexes unexplained.
  S2: test_recorder.py writes motion_<arm>.csv (|F_ext|, tau_ext, dq, command error/step, contact flags, 10 Hz) +
  PEAK / "LAST 3 s" lines. S3 STOP in ERROR keeps the message. S4 gates killed once arm launch + all
  ros2_control_nodes are gone; SHUTDOWN waits for gates. S5 HOME: arm that arrives (exit 0, unsignalled) doesn't
  stop the other. S6 crash_causes(): first failure = cause, later "also ... X s later" / "stopped as a consequence".
  ERROR RECOVERY after an arm crash now RESTARTS the arm stack only (grippers/GELLO/cameras keep running,
  controllers inactive; sim 2 s); no new GUI button. START timeout 60 s (--start-timeout, gate --wait-timeout).
  Gate (sim 109/109 + 15/15): S1 LIVE smoothing (LiveSmoother: segment to each GELLO sample over the measured
  interval, 200 Hz, cap 2.0 rad/s; --live-hz 0 = old raw relay), S8 near joint limit -> keep waiting with hint,
  S7 RESYNC in HOLD (GELLO still 2 s off snapshot -> deactivate, fresh instance, ramp to new pose; --resync-time 0
  = off). Supervisor sim 69/69. Backups of pre-change files: git (commit 4ea0637).
  HARDWARE TESTS OPEN (ask first): S1 smoothing feel + reflex rate with test_recorder running, RESYNC, ERROR
  RECOVERY restart after a real reflex / E-stop. DECIDED 2026-11-21: keep the robots' own collision limits (no SetForceTorqueCollisionBehavior);
  revisit only with S2 force data. S11 DECIDED 2026-11-21: keep as is (no max start distance, ramp
  speed factor 0.2 unchanged). Push to GitHub only
  at the very end (user).
OLD NEXT (ask user first, diff + sim test before hardware): user to answer what happened at 20:43:10, 21:02:06,
  21:03:43; then S1 (gate LIVE smoothing 200 Hz + speed cap) + S2 (recorder logs external force), then S3-S6 bug
  fixes, then S7-S9 UX; S11 is a user decision (max start distance / slower ramp). Run test_recorder during tests.
OLD PLAN, supervisor on hardware:
  H3 first real HOME (show home pose vs current first); H4 E-stop in TELEOP -> ERROR -> RECOVER -> START.
  Then GUI (control + cameras), then foot pedal node + udev rule.
DONE EARLIER - single-arm test on LEFT FR3 (.51), asking before each stage:
  Stage A (no motion): arms_gated.launch.py arms:=left (real config) + gello_check.launch.py arms:=left;
    check FCI connects, impedance controller inactive, broadcaster active, /left/franka/joint_states and
    /gello_check_left/gello/joint_states (25 Hz); print per-joint GELLO-robot difference. No gate.
  Stage B (motion, separate OK, user runs it in own terminal):
    ~/robot_demo/scripts/teleop_duo.sh --arms left
    -> take GELLO out of holder, hold still 1 s -> robot ramps to GELLO pose -> HOLD -> LIVE;
       Ctrl-C -> check robot HOLDS pose on deactivation (not sim-testable).
Then: right arm, duo test, add grippers to teleop_duo.sh, then supervisor/GUI/pedal (not started yet).
Open: home_fr3s.sh uses unanchored pgrep (only false refusals, safe) - offer diff to anchor it.

## GUI requirements
Two windows on two monitors (PyQt5 + rclpy; fall back to tabs on one screen):
1. CONTROL: START / STOP / HOME (same supervisor services as the pedals); per arm: robot mode, errors in plain
   words, error recovery, gripper open/close; status lights (robots, FCI, GELLO, grippers, cameras, pedal);
   log panel. Later: unlock/lock brakes and FCI on/off via the Franka Desk API (check client vs robot system version;
   credentials in keyring, never in code).
2. CAMERAS: ZED Mini + both D405 color streams side by side from the /compressed topics above, decoded with
   QImage.fromData (no cv_bridge needed); show fps per stream.
The GUI is not a safety device; the physical E-stop stays in use.

## Foot pedal
POWBORO FS2020 (PCsensor, USB 3553:b001), programmed with ElfKey, appears as a keyboard. Plan: pedal node reads
/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd with evdev (needs a udev rule for event* access) and calls the
same supervisor services. Until then pedal 3 can be a GNOME shortcut to ~/fr3_tools/home_fr3s.sh.

## NEW GUI MERGED (2026-11-21, from origin/features/GUI-development by Akshay; see docs/HANDOFF_GUI.md)
- Home (white, no buttons, pedals drive) / Live camera page (auto on TELEOP_STARTING/TELEOP) / Operator menu
  (Testing stage, Fix errors, Status & logs, Robot setup, System). Display 1 (DP-0) home->live on START,
  display 2 (DP-2) always cameras. USER DECISIONS 2026-11-21: NO extra STOP button on home/live (pedal 2 +
  Operator menu + E-stop); buttons stay Franka Grey (safety colours dropped on purpose).
- Assets (not in git): logo_black/white.png rendered from our SVGs, Trademark_negative copied from docs/brand.
  fr3_duo.png (FR3 Duo picture for home) MISSING - ask Akshay (HumanoidFR3 assets/logos).
- Fixed in GUI: Fix-errors text (ERROR RECOVERY restarts arm stack), PLAIN words (joint_velocity_violation...).
- test/sim_gui_test.sh: GUI in-process (offscreen, both windows) + real supervisor on fake hardware, clicks the
  GUI's own buttons: 33/33 (screenshots logs/simgui_*/shots). Operator buttons refresh only while the Operator
  page is visible (harness opens it first). In sim arm chips are red (no robot_mode) - expected.
- Sim status after merge + F1-F3: gate 117/117 + 15/15, supervisor 69/69, GUI 33/33. HARDWARE TEST NEXT.

