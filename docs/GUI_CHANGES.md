# GUI redesign (branch `features/GUI-development`)

Files changed: `scripts/demo_gui.py` (rewritten), `scripts/franka_theme.py` (rewritten), `assets/brand/` (new images).
Not touched: supervisor, pedal node, desk agent, start_demo.sh (`demo_gui.py --owns-demo` still works), service/topic names.

| Area | Jithin's version | This branch |
|---|---|---|
| Home | CONTROL window: banner, START/STOP/HOME, cards, log | White page, FR3 Duo picture, one line, **no buttons** |
| Pedals | Same services as buttons | Same; the GUI follows the supervisor state (TELEOP -> Live page, back to Home on STOP) |
| Navbar | none | Logo (black/white by mode), LEFT/RIGHT arm chips, CAMERAS chip (click = details of the 3 cameras), OPERATOR MODE button (no drop-down), light/dark, FULL SCREEN <-> window |
| Chips | coloured status chips in the System card | Connected: Franka Dark Blue + blinking green dot. Not connected: light Franka Grey + blinking red dot |
| Errors | state banner + per-arm "Errors" row | Problem bar under the navbar (Home/Operator) and left panel on the Live page, in plain words |
| Cameras | separate dark CAMERAS window | Camera page, layout of the design mockup `assets/brand/Home@1x(2).png`: header bar, ROBO STATUS list left, 2 x 2 grid right: ZED RGB | ZED DEPTH, left wrist | right wrist (four equal 16:9 tiles, ~700 px wide at 1080p; the ZED RGB tile has no inset because the depth tile sits right next to it; the two wrist cameras keep a depth picture-in-picture inset). **Display 1 = Home (-> cameras on START), display 2 = cameras always.** One monitor: one window |
| No camera | `NO SIGNAL` text | Until a stream connects (camera, depth tile or inset): pulsing Franka trademark in a spinning ring (loading animation) with "waiting for camera / depth" |
| Depth | - | **Picture-in-picture**: every camera shows RGB as the main picture with the DEPTH view as a small inset, always on (no RGB/DEPTH or picture-in-picture switches). Needs the depth topic in `CAMERAS` (4th field) in `demo_gui.py`; until then the inset shows the loading icon (preview feeds fake depth to show it) |
| Status | fixed card on the Live page | **ROBO STATUS** device list (11 devices: arms, grippers, cameras, GELLO hands) with ONLINE / CONNECTED / LIVE / NO SIGNAL per device. **Slides out when the STATUS button (header) is clicked; closed by default so visitors don't see it.** Closed: the four camera pictures grow to fill the screen (8 px gap, 12 px margins; camera name + fps are badges on the pictures, no title rows) |
| Camera page design | - | Adapted from the colleague's spec `Teleop Control Station – UI Design Specification` (Downloads): 76 px header (logo, title, tabs Home / Camera View / Operator Mode), 350 px ROBO STATUS sidebar (spec: 400) with parent + sub rows (FCI, Mode per arm), exactly 128 px status pills (ok / warn / off), Manual / Execution switch, camera tiles with name chip + status chip, depth inset 32 % / 16:10 / white frame. **Brand adaptations:** Franka dark blue instead of #0b6cff, Franka grey buttons, Montserrat + DejaVu Mono instead of Inter + JetBrains Mono, trademark loading animation instead of the camera-icon placeholder, light AND dark mode, `--ok` already the darker #117a3b. **Differences kept from our requirements:** sidebar slides out and is closed by default, 8 px gaps (spec: 16), 10 px side margins and only 6 px top / bottom margin, narrower 350 px sidebar with smaller padding (cameras are bigger while the status is open), no depth inset on the head RGB tile (own depth tile), responsive instead of a scaled 1920 x 1080 stage. The Mode switch only shows the real Franka Desk mode (Manual = Programming); changing it stays in Operator Mode > Robot setup because the Desk services switch both arms at once |
| Camera page controls | - | No navbar on the camera page: header bar with logo + title, HOME, OPERATOR MODE, light/dark, FULL SCREEN (the chips' info is in ROBO STATUS). Brand colours instead of the mockup's blue |
| Window | maximised | Starts **full screen**; FULL SCREEN button toggles to a normal window with the title bar's minimise / maximise buttons |
| Theme | light control, dark cameras | White default, dark option; logo swaps |
| Buttons | green START, red STOP, blue HOME ... | All Franka Grey (STOP keeps a thin red outline) |
| Operator | everything on the CONTROL window | One page, **all sections visible at once** in a 3-column grid: Testing stage (START/STOP/HOME, grippers) · Fix errors · Robot setup (Desk switches) / Status (both arms) · System (bring up/shut down, lights) / Log |
| Logo / path | SVG, repo at `~/robot_demo` | PNG (transparent), paths relative to the repo |

Preview keys (`--preview`, sends nothing): F5 pedal START, F6 STOP, F7 HOME, F9 error, F10 drop a camera, F11 depth available / not.
Open questions: which D405 is left/right wrist (ASSUMED d405_1 = left, d405_2 = right, see `CAMERAS`), final headline text, depth topic.
