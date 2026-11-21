# Handoff: GUI redesign (branch `features/GUI-development`)

For Jithin. Author of the redesign: Akshay (with Claude Code). Date: 2026-11-21.
Status: **tested only in `--preview` (fake data, offscreen screenshots). NOT yet tested with the real supervisor,
the two real monitors or the pedals.** Do that on the workstation with the E-stop in hand before the demo.

## What changed (vs. your `demo_gui.py`)
Only `scripts/demo_gui.py` and `scripts/franka_theme.py` were rewritten. Supervisor, pedal node, desk agent,
`start_demo.sh` (`--owns-demo`), and all service / topic names are untouched. The GUI is still a thin client.
Full before/after table: [GUI_CHANGES.md](GUI_CHANGES.md).

- **Home**: white page, FR3 Duo picture, one line, no buttons. Pedals drive everything.
- **Navbar**: logo (black on light, white on dark), LEFT ARM / RIGHT ARM / CAMERAS chips (click = details popup),
  Operator menu, light / dark switch (light default).
  Chip connected = Franka Dark Blue + blinking green dot; not connected = light Franka Grey + blinking red dot.
- **Live (camera) page**: opens on `TELEOP_STARTING` / `TELEOP`, back to Home on STOP / error.
  Layout: ZED Mini overview centre, **left wrist bottom-left, right wrist bottom-right** (ASSUMPTION: `d405_1` = left, `d405_2` = right;
  swap the titles in `CAMERAS` if wrong). Tiles are 16:9 and sized to fill the page (`LivePage.fit_tiles`: ZED row + wrist row fill the height under the toolbar,
  wrists = 62 % of the ZED width via `WRIST_RATIO`; at 1080p: ZED ~935 px, wrists ~580 px wide).
  **Depth = picture-in-picture** on every camera: main view RGB + DEPTH inset (or reverse via MAIN VIEW switch top right);
  click the inset to swap; PICTURE-IN-PICTURE button switches it off. Both views are decoded once (Decoder) and only while visible.
  **Status panel** = slide panel (STATUS button top left), sits in the free space beside the ZED overview (top-left).
  Problems in plain words in the panel. Missing picture = loading animation (see below).
- **Two monitors**: display 1 (`--control-screen`, default 0) shows Home / Operator and switches to the Live page on START (back to Home on STOP);
  display 2 (`--camera-screen`, default 1) shows the Live page ALWAYS (no Operator menu there). Cameras are decoded once and shared.
  One monitor or `--single` = one window (Home, then Live on START). Theme switch applies to both.
- **No camera**: the tile shows the Franka trademark pulsing inside a spinning ring + "WAITING FOR CAMERA ..." (loading animation, runs only while waiting).
- **Operator panel** (navbar Operator > Operator panel): ONE page with all sections visible at once, 3-column grid:
  Testing stage (START / STOP / HOME, grippers) | Fix errors (problems, ERROR RECOVERY) | Robot setup (Desk switches, PREPARE ALL, PARK);
  Status (both arms) | System (BRING UP / SHUT DOWN, lights); Log. Same service calls and enable rules as your CONTROL window.
- **Problem bar** (Home / Operator) and the Live side panel list issues in plain words (`PLAIN` dict, `Model.issues()`).
- **Buttons**: all Franka Grey (brand). STOP keeps a thin red outline and is never disabled.
  (Your old rule "START green / STOP red" was dropped on request - revert in `franka_theme.stylesheet()` if wanted.)

## How to run
```bash
source /opt/ros/humble/setup.bash
python3 scripts/demo_gui.py --preview --single      # fake data, sends nothing (ROS domain 199)
~/robot_demo/scripts/start_demo.sh                  # real demo (moves robots - E-stop in hand)
```
Preview keys: F5 / F6 / F7 = pedal START / STOP / HOME, F9 = demo ERROR, F10 = drop wrist camera 2, F11 = depth available / not.
Start in the preview with Operator > System > BRING UP, then F5.
Preview with two windows on one screen: add `--force-two`. Screenshots of every page (needs no display): `QT_QPA_PLATFORM=offscreen python3 scripts/demo_gui.py --preview --single --screenshot DIR`.

## Assets (NOT in git - `.gitignore` excludes `assets/brand/*.png`, trademarks)
Copy into `assets/brand/` locally:
`logo_black.png`, `logo_white.png` (company logo, transparent), `fr3_duo.png` (FR3 Duo, transparent, trimmed),
`FrankaRobotics_Trademark_Digital_RGB.png`, `FrankaRobotics_Trademark_negative_Digital_RGB.png` (icon for empty camera tiles).
Source files: HumanoidFR3 `assets/logos/`. Montserrat fonts are in the repo. Missing PNGs = empty logo / picture areas, no crash.
Paths are now relative to the repo (no more hard-coded `~/robot_demo`).

## TODO / hooks for you
1. **Depth view**: add the depth topic as the 4th field of the `CAMERAS` list at the top of `demo_gui.py`
   (a `sensor_msgs/CompressedImage` that `QImage` can decode, e.g. colourised depth). `None` = "not available yet". Nothing else to change.
2. Confirm which D405 is the left / right wrist camera (assumed d405_1 = left, d405_2 = right).
   Picture-in-picture is verified only with FAKE depth frames in `--preview` (F11 toggles depth available / not);
   with the real depth topic still to be checked (decode cost of 3 more streams).
3. Hardware test: two monitors (`--control-screen` / `--camera-screen`), real status JSON, pedals -> page switching,
   OFFLINE look (arm + camera chips are red-blinking until BRING UP - intended, tell us if too noisy), CPU (2 windows).
4. Decide the home headline (`HEADLINE` in `demo_gui.py`) and whether Home should show more state text (`HOME_LINE`).
5. Per-arm "connected" uses the status JSON `arms.<arm>.connected` + `robot_mode in (IDLE, MOVE)`;
   a camera counts as connected at >= 1 fps while `cameras_launch == running`.

## Safety notes
No GUI code sends motion by itself; page switching only reads state. The GUI is not a safety device; keep the E-stop.
