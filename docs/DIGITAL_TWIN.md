# Digital twin (display 2)

A real-time, interactive 3D twin of both FR3s on the second monitor. **Read-only**: it never commands a robot, gripper or
controller and does not touch the supervisor, the start gate, the pedals or the safety logic.

## What you see
- **Robots**: Franka's FR3 Duo mount + cover, two FR3 arms (franka_description meshes) with Robotiq 2F-85 grippers
  (robotiq_description). Arm base poses = `config/duo_geometry.yaml` + the namespace -> slot map of `config/duo_guard.yaml`
  (same geometry as the HOME collision planner; flange TF == `duo_geometry` FK, checked in the sim test).
- **Joint data**: the MEASURED positions on `/<arm>/franka/joint_states` (1 kHz, not GELLO commands), C++ throttle to
  100 Hz -> `/twin/joint_states` -> twin `robot_state_publisher` (frame prefix `twin/`). Gripper knuckles from
  `/<arm>/gripper/joint_states`. An arm without data is drawn at its saved HOME pose and labelled OFFLINE; data that stops
  -> STALE (last pose kept).
- **Overlays** (from `/<arm>/robot_state_10hz`, the supervisor's throttle): joint load spheres (|tau_ext|, mint -> amber ->
  red), external force arrow at the end effector (red while the robot reports contact), inter-arm clearance line + cm
  (`duo_geometry` capsules; only while both arms are live and closer than 10 cm).
- **3D scene**: ZED Mini live point cloud (cropped to 2.5 m), optional D405 wrist clouds (aligned depth ->
  coloured points), optional 3D map (ZED SDK spatial mapping via the wrapper's `enable_mapping` service). Clouds are LAZY:
  a camera is only read while its layer is on. RTAB-Map is not installed and not needed for a fixed camera.
- **Telemetry bar**: per-arm LIVE/STALE/NO DATA + rate + latency (joint state stamp -> twin), clearance, contact force,
  scene state; header shows the demo state and errors (warnings stay on display 1).
- **Interaction**: orbit = left drag, pan = middle drag (or shift + left), zoom = wheel. LAYERS buttons: Robots, Joint load,
  Contact force, Clearance, Head cloud, Wrist clouds, 3D map.

## How it runs
`scripts/start_demo.sh` starts `launch/digital_twin.launch.py` before the GUI; the GUI's display-2 window (`TwinWindow`
in `scripts/demo_gui.py`) embeds the RViz window (X11) and shows only its 3D panel. `DEMO_TWIN=0 start_demo.sh` = old
behaviour (camera page on display 2). Standalone: `ros2 launch launch/digital_twin.launch.py [rviz:=false] [clouds:=false]`.

| File | Role |
|---|---|
| `description/fr3_duo_twin.urdf.xacro` | twin URDF (no ros2_control) |
| `launch/digital_twin.launch.py` | rsp, throttles, twin_state, twin_cloud x4, rviz2 (respawn) |
| `scripts/twin_state.py` | gripper joints, overlays, layers, camera TFs, `/twin/status` |
| `scripts/twin_cloud.py` | lazy clouds: zed / d405_1 / d405_2 / map |
| `scripts/twin_calibrate.py` | keyboard nudge of camera extrinsics (edits `config/digital_twin.yaml`, live reload) |
| `config/digital_twin.yaml` | rates, thresholds, gripper mount, camera extrinsics, default layers |
| `config/digital_twin.rviz` | RViz look (Franka dark-blue stage, no panels) |
| `config/zed_twin_override.yaml` | ZED point cloud resolution (COMPACT) |
| `launch/cameras.launch.py` | CHANGED: ZED `publish_tf:=false` (fixed camera; the twin places it), cloud override |

## Camera calibration (needed before the clouds line up)
Extrinsics in `config/digital_twin.yaml` are GUESSES (`calibrated: false` -> "(not calibrated)" in the bar). With the demo
running and the Head cloud layer on: `python3 scripts/twin_calibrate.py --camera zed`, nudge until the cloud of the mount /
arms matches the model, press `c`. Same for `d405_1` / `d405_2` (relative to the flange; d405_1 = left wrist is an
assumption).

## Verified vs. hardware-dependent
Verified in simulation (fake data, ROS domain 78, no robot): `test/sim_twin_test.sh` 27/27 (rates ~89 Hz, latency 0.3 ms
median, TF/FK match 0.00 mm, overlays, layers, STALE/NO DATA, D405 + ZED cloud conversion, config hot reload),
`test/sim_twin_gui_test.sh` 8/8 on the real screen x3 (embedding, no RViz chrome, layer switch, RViz respawn re-embed).
Regression: `test/sim_gui_test.sh` 35/35, `test/gui_camera_test.py`, geometry / pedal / depth tests pass.
Twin CPU ~40 % of one core (twin_state 12 %, ZED cloud 12 %, RViz 11 %), GPU ~30 %.

NOT yet verified on hardware: real joint-state topic rates/latency, Robotiq knuckle mapping direction, gripper mount
offset/rotation, `o_t_ee` / force arrow frame, real ZED cloud cost and look, ZED with `publish_tf:=false`, D405 cloud
alignment, ZED spatial mapping, all camera extrinsics.

## Hardware results (2026-11-21)
Twin on the real FR3 Duo: 95 Hz per arm, 0.6 ms median latency. During 184 s of GELLO teleop (`test/twin_track_check.py`):
shoulder/elbow joints follow the GELLO with scale 0.96-1.00 and 110-180 ms lag; wrist joints 0.91-0.96 and up to 0.5 s
lag on fast twists. This is the robot's own following error (start gate speed limit), which the twin shows faithfully.
The ZED is mounted on the robot head; its extrinsics still need calibrating (auto-calibration under development).

## Solid 3D reconstruction (2026-11-22)
Replaces the raw point clouds as the default scene: a **solid, coloured, lit triangle mesh** of the workspace plus
**live objects** (things moving in space the map knows is empty). Read-only, asynchronous, own process.

**Why this approach.** ZED spatial mapping needs the SDK's positional tracking (SLAM), and the ROS wrapper only publishes
a fused *cloud*, not a mesh. Isaac ROS nvblox needs the Isaac apt/Docker stack and a matching CUDA build for the RTX 5070 Ti
(a large dependency change). Open3D is not installed: a pip install of ~400 MB, and its wheel's CUDA does not cover
this GPU, so it would run on the CPU anyway. Chosen: own numpy/scipy TSDF (`scripts/twin_recon_core.py`) with **known camera poses** (twin TF:
fixed ZED extrinsic, D405 = FR3 forward kinematics @ calibrated mount, looked up at each depth stamp) + surface-nets meshing.
No new dependencies, no GPU, no SLAM. Open3D/nvblox remain an upgrade path if ever approved.

| Piece | What it does |
|---|---|
| TSDF | 1 cm voxels over `reconstruction.bounds_*` (60 MB). Weighted running average capped at `max_weight` frames, so a moved object is carved away and a placed one appears after ~10 s. ZED projection cached (fixed camera); D405 frames used only when calibrated and the arm is slower than 0.4 rad/s |
| Robot filter | depth pixels within `robot_pad_m` of the duo_geometry capsules (both arms + grippers, joint positions at the frame stamp) only carve free space, never add surface; voxels inside the arms are cleared before each mesh. The live RobotModel stays the only robot |
| Mesh | surface nets (smooth, one vertex per cell), Taubin smoothing, specks removed, per-vertex colour with baked soft lighting (`reconstruction.shading`), triangles face free space. Published as `TRIANGLE_LIST` chunks serialised straight from numpy (100k triangles in ~10 ms) on `/twin/mesh/static` (latched) |
| Live objects | observed points in known-empty space -> voxel cubes in their real colour on `/twin/mesh/live`. They hand over seamlessly to the static mesh if they stay put |

**Launch.** Started automatically with the twin (`start_demo.sh` -> `launch/digital_twin.launch.py`, `recon:=false` to
disable). It needs the cameras (`launch/cameras.launch.py`, part of the demo bring-up). Standalone:
`ros2 launch launch/digital_twin.launch.py` (+ cameras running). Layer buttons: **3D mesh**, **Live objects**; the raw
**Head cloud / Wrist clouds / Scan points** remain as optional overlays (off by default).

**Map commands** (also usable from `ros2 service call`):
```
ros2 service call /twin/recon/save    std_srvs/srv/Trigger      # scans/recon_map.npz + scans/recon_map.ply (MeshLab/Blender)
ros2 service call /twin/recon/load    std_srvs/srv/Trigger      # (also auto-loaded at start: reconstruction.autoload)
ros2 service call /twin/recon/reset   std_srvs/srv/Trigger      # empty map; re-reads voxel_m / bounds from the config
ros2 service call /twin/recon/mapping std_srvs/srv/SetBool "{data: false}"   # freeze the map, live objects keep updating
ros2 topic echo /twin/recon/status                              # per camera state, triangles, frame / mesh time
```
Resolution: `reconstruction.voxel_m` (1 cm default; 6 mm = 280 MB and ~5x slower meshing), then `reset`.

**Tests.** `python3 test/twin_recon_test.py` (offline, synthetic, 13/13: surfaces median 0.3 mm at 1 cm voxels, no robot
ghosts with a swinging arm, live object, hand-over, carve-out, wrist camera via FK, save/load, Marker bytes ==
rclpy). `test/sim_recon_test.sh` (ROS domain 79, fake robots + fake ZED/D405 through the real launch, throttles and TF:
9/9; worker 12 ms/frame, mesh 80-90 ms/s). `test/twin_fake_depth.py --replay snap.npz` replays a captured real ZED
snapshot. Real ZED data (one read-only snapshot, 2026-11-22) meshes cleanly; placement awaits the ZED extrinsic
calibration (still a guess). **Hardware open:** ZED extrinsic, D405 mounts (`calibrated: true` via 3D SCAN), the look
with real arms masked by real joint states.

## Photoreal twin - NVIDIA Isaac Sim 6.2 (2026-11-22)
RTX real-time path tracing of the live twin, full screen on display 2 (it covers the RViz twin window). READ-ONLY, no
physics, no ROS inside Isaac. Installed in an isolated venv: `~/isaacsim_env` (Python 3.12 via `uv`, ~23 GB, user
level, no sudo; ROS / CUDA / NVIDIA driver untouched). The NVIDIA Omniverse EULA is accepted via OMNI_KIT_ACCEPT_EULA=YES
in the launcher.

    scripts/start_isaac_twin.sh            # demo / twin must be running; Ctrl-C (or SIGTERM) stops it
    scripts/start_isaac_twin.sh --windowed --x 0     # windowed on display 1

| Piece | Role |
|---|---|
| `scripts/twin_isaac_bridge.py` (system ROS) | twin URDF with absolute paths, all 26 joints (arms, knuckles, mimic) over UDP 127.0.0.1:47811 at 60 Hz, reconstruction mesh / live objects / ZED photo + pose + depth into `isaac/cache/` |
| `isaac/convert_twin.py` | URDF -> USD once (launcher re-runs it when the URDF changes) |
| `isaac/twin_isaac.py` | kinematic posing (link local transform = joint origin x axis rotation; flanges == ROS TF within 1 mm), studio lighting, ACES, table, PROJECTIVE PHOTO TEXTURE: faces the ZED sees now show its 1080p photo, the rest the fused vertex colours |

Measured: 56-60 Hz joint updates, 6-8 ms bridge -> render, GPU ~60-99 %, 3.5 GB VRAM. RT latency of a SCHED_FIFO 1 kHz
probe with Isaac rendering: max 62 us (it does NOT disturb the realtime loop).

## IMPORTANT: ROS 2 traffic on loopback (FCI safety, 2026-11-22)
Without a Cyclone config, DDS bound to enp97s0 = the ROBOT network and multicast camera data onto it: 110 MB/s outgoing
on the FCI link (407 GB since boot). Result: left FR3 `communication_constraints_violation` reflex 0.4 s into a HOME.
`env/host_env.sh` now sets ROS_LOCALHOST_ONLY=1 + `config/cyclonedds_host.xml` (participant indices for unicast on lo).
After the fix: robot link 9 kB/s with the full demo, 0 "Failed to lock the realtime publisher" errors (were 10s-1000s per
session), HOME + 9-view SCAN without a fault. Terminals that do not source env/host_env.sh no longer see the demo's
topics (restart the ros2 daemon after changing the env: `ros2 daemon stop`).

## Calibration state (2026-11-22)
- ZED (head camera): CALIBRATED, verified twice: robot meshes projected into the live image land on the real grippers,
  table plane height == ZED-measured gripper height. `scripts/twin_autocalib.py` (default `--method plane`: table plane
  gives roll / pitch / height, then x / y / yaw against the encoder-posed robot meshes; the old free ICP gave false fits).
- D405 wrists: NOT calibrated. The real 3D SCAN (9 views) was degenerate for the consistency-only solve (fused table at
  +8.8 cm; it also dragged the ZED 25 cm off - both now refused by guards in twin_scan.write_results).
  `scripts/twin_handeye_ref.py scans/rec_<time>` (table plane z=0 + consistency + ZED map) gets height / tilt right
  (2.7 mm plane fit) but ~cm in-plane uncertainty remains -> stored as ESTIMATE, `calibrated: false` (not fused).
  Precise next step: a ChArUco board on the table (mm-level hand-eye) or scan views with larger wrist rotations.
  Note: the table top is slotted aluminium profile - wrist depth sees 6-10 cm into the slots (not an error).
- The RealSense driver publishes NO link -> optical TF: twin_state now publishes `<cam>_link -> <cam>_color_optical_frame`
  (cameras.<cam>.optical_frame).
- twin_recon resets its map automatically when a fused camera's extrinsics change in the config; a (re)started node
  clears its marker namespaces first.

## Photoreal twin in the BROWSER + "Scan surroundings" (2026-11-22, hardware-verified)
    scripts/start_isaac_twin.sh [--open]        # demo running; UI http://127.0.0.1:8211/ (Firefox), stream :8212
GUI: Operator > Testing stage > "PHOTOREAL TWIN" opens the page. `--window` = old desktop window instead.
- Isaac renders HEADLESS into a render product (RTX real-time path tracing) -> MJPEG (~20 fps at 1600x900). Note:
  plain app.update() never fills a replicator annotator; the loop uses rep.orchestrator.step(wait_for_render=False).
- Page: live view (drag = orbit, right/shift-drag = pan, wheel = zoom; presets incl. "Head camera" = the CALIBRATED ZED
  pose + its 65 deg FOV, so render and real photo are directly comparable), robot / scan / 3D scene status, buttons
  SCAN SURROUNDINGS, HOME, grippers, reset map, STOP. Commands use the supervisor services (same gating as GUI/pedals).
  Both servers bind to 127.0.0.1 (they can move the robots); --bind only if you mean it.
- ACCURACY (measured during a real scan, flange speeds up to 17 cm/s): rendered vs measured flange position median
  0.2 mm, p95 0.8 mm, max 2.1 mm; exact at rest. The renderer extrapolates each joint by its measured velocity x data
  age (<= 80 ms) to the render instant; without it: median 1.4 mm, p95 4.9 mm (40 ms data age). Display (JPEG +
  browser) adds a few tens of ms on top. Gripper direction verified (close/open on the real left gripper).
- Realism: photo-textured faces are EMISSIVE (the ZED photo already contains the real light and shadows); dark room;
  reconstruction now 6 mm. What still limits realism from viewpoints the head camera does not see: the wrist cameras.
- SCAN flow (twin_scan): every view also records a ZED photo + full-res wrist colour. After the scan: (1) ChArUco
  board calibration of the wrist mounts if the board is visible (scripts/twin_board.py; synthetic 8/8, 2-4 mm),
  else (2) the map-based solve with strict acceptance; accepted mounts -> config, twin_recon resets and integrates the
  recording (/twin/recon/cmd {"integrate": dir}; D405 frames weigh 3x). Real scans 2026-11-22: no board -> map-based
  solve refused (objects 23 mm off the ZED map) -> wrists still uncalibrated.
- TO GET SHARP OBJECTS: print docs/charuco_board_A4.pdf at 100 % (check: square = 45 mm), lay it flat ~0.6 m in front
  of the mount centred between the grippers, press SCAN SURROUNDINGS, remove the board, scan again.
