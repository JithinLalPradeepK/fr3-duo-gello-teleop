# External code and local changes

Copies of things that live outside this repo on the demo machine, so the setup can be rebuilt.

| File | Applies to | What it changes |
|---|---|---|
| `gello_software_204f53a.patch` | `~/gello_software` @ `204f53a` (`git apply`) | Robot IPs (.51 left / .50 right), `load_gripper: "false"` (Robotiq mounted), USB serials of our GELLO OpenRB-150 boards and Robotiq FTDI adapters |
| `ros2_robotiq_gripper_2ff8545.patch` | `~/gello_deps_ws/src/ros2_robotiq_gripper` @ `2ff8545` | `kGripperMaxSpeed` 0.150 → 1.0 |
| `fr3_duo_home/` | builds to `~/fr3_tools/fr3_duo_home/build/fr3_duo_home` | Standalone libfranka homing tool (`--show`, `--save`, `--yes`, `--vel`); `home_poses.txt` = home pose per robot IP |
| `home_fr3s.sh` | `~/fr3_tools/home_fr3s.sh` | Wrapper: one homing at a time (flock), refuses while teleop / ros2_control runs |

`config/franka_gello_duo_check.yaml` must stay in sync with the patched `franka_gello_duo.yaml`
(only the namespaces differ).

`fr3_duo_home/home_poses.txt` was made MIRROR-SYMMETRIC on 2026-11-21 with `scripts/make_mirrored_home.py` (average of the
saved poses; the Duo's arms are mirror images, q_left = (-q1, q2, -q3, q4, -q5, q6, -q7) * q_right). The previous values
are kept next to the working copy as `home_poses.txt.<date>.bak` on the demo machine.
