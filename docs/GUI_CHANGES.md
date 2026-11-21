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
| Cameras | separate dark CAMERAS window | Camera page (labelled ZED Mini / Wrist camera 1 / 2, RGB-DEPTH switch top right). **Display 1 = Home (-> cameras on START), display 2 = cameras always.** One monitor: one window |
| No camera | `NO SIGNAL` text | Pulsing Franka trademark in a spinning ring (loading animation) in that camera's tile |
| Depth | - | Switch ready; set the topic in `CAMERAS` (4th field) in `demo_gui.py` |
| Theme | light control, dark cameras | White default, dark option; logo swaps |
| Buttons | green START, red STOP, blue HOME ... | All Franka Grey (STOP keeps a thin red outline) |
| Operator | everything on the CONTROL window | Menu: Testing stage (START/STOP/HOME, grippers), Fix errors, Status & logs, Robot setup (Desk switches), System (bring up/shut down) |
| Logo / path | SVG, repo at `~/robot_demo` | PNG (transparent), paths relative to the repo |

Preview keys (`--preview`, sends nothing): F5 pedal START, F6 STOP, F7 HOME, F9 error, F10 drop a camera.
Open questions: which D405 is left/right wrist (labels say 1 and 2), final headline text.
