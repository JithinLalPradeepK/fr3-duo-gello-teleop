# GUI development guide

The GUI (`scripts/demo_gui.py` + `scripts/franka_theme.py`) is a **thin client**. It contains no demo
logic: it only calls supervisor services and shows the JSON status. Keep it that way. Any behaviour
change (what START does, when HOME is allowed, ...) belongs in `demo_supervisor.py`, so the foot pedal
and the GUI keep behaving the same.

## Run it without robots

Needs ROS 2 Humble (`rclpy`, `std_srvs`, `sensor_msgs`) and PyQt5 (`sudo apt install python3-pyqt5
python3-pyqt5.qtsvg`).

```bash
source /opt/ros/humble/setup.bash
python3 scripts/demo_gui.py --preview            # two windows (CONTROL + CAMERAS)
python3 scripts/demo_gui.py --preview --single   # one window with tabs (laptop)
```

`--preview` uses ROS domain 199, fills the GUI with demo data and **sends nothing**. Press **F9** to
show a demo ERROR state (F5/F6/F7 = pedal START/STOP/HOME, F10 = drop a camera, F11 = depth available / not). See [GUI_CHANGES.md](GUI_CHANGES.md).

## Interface to the supervisor (do not change without agreeing with the supervisor side)

| Kind | Name | Type | Meaning |
|---|---|---|---|
| service | `/demo/start`, `/demo/stop`, `/demo/home`, `/demo/recover`, `/demo/bringup`, `/demo/shutdown` | `std_srvs/Trigger` | Demo commands; `message` = reason if refused |
| service | `/demo/stop_home` (pedal 2), `/demo/reset` (pedal 3) | `std_srvs/Trigger` | `stop_home`: STOP at once, a stopped teleop is followed by HOME after 1 s. `reset`: stop, clear errors (Desk prepare / error recovery / arm restart / bring up), HOME, ready again; refused while an emergency stop is pressed; STOP aborts it |
| service | `/demo/gripper/<left\|right>/<open\|close>` | `std_srvs/Trigger` | Gripper buttons |
| service | `/demo/desk/<prepare\|park\|unlock\|lock\|execution\|programming\|fci_on\|fci_off>` | `std_srvs/Trigger` | Franka Desk (desk_agent.py) |
| topic | `/demo/status` | `std_msgs/String` (JSON, latched) | Full demo state (see `status_dict()` in `demo_supervisor.py`) |
| topic | `/demo/desk/status` | `std_msgs/String` (JSON, latched) | Desk state per arm |
| topic | `/demo/pedal/status` | `std_msgs/String` (JSON, 1 Hz) | Pedal heartbeat |
| topic | `/demo/pedal/hold` | `std_msgs/String` (JSON, ~10 Hz while a pedal is held) | `{"pedal": "start"\|"reset"\|null, "progress": 0..1, "hold_s": 3.0, "fired": bool}` - START (pedal 1) and RESET (pedal 3) fire after a 3 s hold; **GUI TODO: show a progress ring/bar while `pedal` is not null** (null = released or cancelled). pedal 2 (stop_home) is immediate and never appears here |
| status field | `/demo/status` -> `reset` | step name (`stop`, `desk`, `bringup`, `recover`, `home`) / `null` | the RESET macro is running (show "resetting ...") |
| status field | `/demo/status` -> `arming` | `"start"` / `null` | the gates are pre-warmed because the pedal is being held (state is still IDLE) |
| status text | gate states in `/demo/status` -> `arms.<arm>.gate` | string | new states: `ARMED` (standby), `BARRIER` (waiting for the other arm / collision check), `LIVE: limited by the collision guard (...)`, `RESYNC` |
| topic | `/zed/zed_node/rgb/color/rect/image/compressed`, `/camera/d405_1/color/image_raw/compressed`, `/camera/d405_2/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera streams (JPEG, decode with `QImage.fromData`) |
| topic | `/zed/zed_node/depth_view/compressed`, `/camera/d405_1/depth_view/compressed`, `/camera/d405_2/depth_view/compressed` | `sensor_msgs/CompressedImage` | **Depth feeds (2026-11-21)**: colourised JPEG, near = red, far = blue, no data = black, ~10 fps, 480 px wide. Made by `scripts/depth_view.py` (started by `launch/cameras.launch.py`); it only reads the raw depth while the GUI is subscribed. Already wired into `CAMERAS` (4th field) |

## Rules

- Buttons are Franka Grey (brand); STOP keeps a red outline and must always be enabled. Status dots: green ok, red problem.
- The GUI must never send motion commands by itself (no timers or auto-retries of START or HOME).
- Don't name Node attributes `handle` or `clients` (these names are taken by rclpy).
- Watch the CPU: the GUI already uses ~47 % of a core for drawing 3 streams at 30 fps.
- Test on the real demo machine before the demo (two monitors: CONTROL on DP-0, CAMERAS on DP-2).

## Workflow

Work on a branch (e.g. `gui/<topic>`) and open a pull request into `main`. Demo/supervisor fixes go
on separate branches. Changes to the files above should get a review from the supervisor side.

## Camera feed orientation and window size (2026-11-21)
- **Both D405 wrist cameras are mounted upside down**: their RGB *and* depth pictures are shown rotated by 180 degrees
  (`INVERT` in `demo_gui.py`, `--invert d405_1,d405_2,zed` / `--invert ""` on the command line). The flip is done once in the
  `Decoder`, so every window shows the same orientation.
- **The window must stay screen sized.** `LivePage.minimumSizeHint()` returns a small size on purpose: `fit_tiles()` sets
  FIXED tile sizes, which used to become the page's minimum, and the window's stacked layout takes the largest minimum of
  all pages - the control window grew to 1173 px on a 1080 px screen (top not covered, bottom of the wrist tiles cut).
  `python3 test/gui_camera_test.py` checks the window size on every page for both roles.

## Update 2026-11-21: the Home page was removed
Both displays now start on the **camera page**: the main display (display 1) with the ROBO STATUS list closed (cameras fill
the screen), the second screen (display 2) with the ROBO STATUS list **open on the left** of the cameras. The header tabs are
`Camera View | Operator Mode` (no Home tab); the Operator page's "Back" button returns to the camera page; when teleop starts
while the Operator page is open, the main display goes back to the cameras. Nothing else changed: the Operator page, theme and
full-screen buttons, depth views and the pedals work as before. Note: problems are now only shown in the ROBO STATUS list (open on
the second screen; on the main display open it with STATUS) - the old problem bar existed on the Home page only.
