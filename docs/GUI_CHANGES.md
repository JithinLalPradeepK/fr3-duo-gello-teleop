# GUI redesign (branch `features/GUI-development`)

Files changed: `scripts/demo_gui.py` (rewritten), `scripts/franka_theme.py` (rewritten), `assets/brand/` (new images).
Not touched: supervisor, pedal node, desk agent, start_demo.sh (`demo_gui.py --owns-demo` still works), service/topic names.

| Area | Jithin's version | This branch |
|---|---|---|
| Home | CONTROL window: banner, START/STOP/HOME, cards, log | White page, FR3 Duo picture, one line, **no buttons** |
| Pedals | Same services as buttons | Same; the GUI follows the supervisor state (TELEOP -> Live page, back to Home on STOP) |
| Navbar | none | Logo (black/white by mode), LEFT/RIGHT arm chips, CAMERAS chip (click = details of the 3 cameras), Operator menu, light/dark |
| Chips | coloured status chips in the System card | Connected: Franka Dark Blue + blinking green dot. Not connected: light Franka Grey + blinking red dot |
| Errors | state banner + per-arm "Errors" row | Problem bar under the navbar (Home/Operator) and left panel on the Live page, in plain words |
| Cameras | separate dark CAMERAS window | Camera page: ZED Mini overview centre, left wrist bottom-left, right wrist bottom-right (16:9 tiles sized to fill the page, wrists enlarged: ~580 px wide at 1080p). Main view RGB / DEPTH switch top right. **Display 1 = Home (-> cameras on START), display 2 = cameras always.** One monitor: one window |
| No camera | `NO SIGNAL` text | Pulsing Franka trademark in a spinning ring (loading animation) in that camera's tile |
| Depth | - | **Picture-in-picture**: every camera shows the other view as a small inset (RGB main + DEPTH inset, or the reverse); click the inset to swap, PICTURE-IN-PICTURE button turns it off. Needs the depth topic in `CAMERAS` (4th field) in `demo_gui.py`; until then the inset shows the loading icon (preview feeds fake depth to show it) |
| Status panel | fixed card on the Live page | Slide panel (STATUS button, animated), uses only the free space beside the ZED overview (top-left) |
| Theme | light control, dark cameras | White default, dark option; logo swaps |
| Buttons | green START, red STOP, blue HOME ... | All Franka Grey (STOP keeps a thin red outline) |
| Operator | everything on the CONTROL window | One page, **all sections visible at once** in a 3-column grid: Testing stage (START/STOP/HOME, grippers) · Fix errors · Robot setup (Desk switches) / Status (both arms) · System (bring up/shut down, lights) / Log |
| Logo / path | SVG, repo at `~/robot_demo` | PNG (transparent), paths relative to the repo |

Preview keys (`--preview`, sends nothing): F5 pedal START, F6 STOP, F7 HOME, F9 error, F10 drop a camera, F11 depth available / not.
Open questions: which D405 is left/right wrist (ASSUMED d405_1 = left, d405_2 = right, see `CAMERAS`), final headline text, depth topic.
