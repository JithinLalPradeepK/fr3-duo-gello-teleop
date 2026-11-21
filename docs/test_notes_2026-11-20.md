# Test session notes - 2026-11-20 (new exhibition GUI)

Recorder: `logs/test_20261120_203258/events.log` (scripts/test_recorder.py, read-only, 1383 events).
Demo logs: `logs/start_demo_20261120_203258/` (supervisor, desk_agent, pedal, gui) and `logs/demo_20261120_*` (stack).
Session 20:32:58 - 21:25:55: 7 bring-ups, ~12 teleop starts, 5 HOME, 6 robot reflexes, Desk switches used.

## Setup at start
- New GUI installed (backup scripts/.backup/demo_gui.py.20261120_brand), franka_theme.py, Montserrat fonts.
- Supervisor startup log line added -> start_demo.sh opens the GUI immediately (was ~15 s). Verified.

## Operator requests
| # | Time | Request | Status |
|---|---|---|---|
| - | - | (none yet) | |

## Observations
| # | Time | What happened | Assessment |
|---|---|---|---|
| O1 | 20:40:35 | right `cartesian_reflex` 21 s into TELEOP | see O7 |
| O2 | 20:43:10 | left AND right `cartesian_reflex` at the same moment, 34 s into TELEOP | see O7 |
| O3 | 20:53:30 | left `self_collision_avoidance_violation` while RAMPING to the GELLO pose (max diff 0.79 rad); after bringup left stayed in REFLEX (self-collision) until the operator switched Desk mode Programming -> Execution | ramp to an arbitrary GELLO pose can hit self-collision limits; Franka does NOT check left-right arm collisions |
| O4 | 21:02:06 | left `cartesian_reflex`; right `communication_constraints_violation` 1.26 s later | right was a CONSEQUENCE (launch shut down its controller while active); the GUI lists both as equal causes |
| O5 | 21:03:43 | right `cartesian_reflex` 26 s into TELEOP | see O7 |
| O6 | 21:14 | HOME ok | |
| O7 | all | 5 Cartesian reflexes in ~5 min of LIVE teleop | likely cause: GELLO publishes at 25 Hz and the gate relays raw samples -> controller target jumps every 40 ms, K = 240 Nm/rad on j1-j4 -> torque steps (e.g. 0.04 rad step = ~10 Nm) -> estimated external force spikes -> Cartesian reflex |
| O8 | 20:43:14, 21:03:54 | STOP pressed while in ERROR -> state flipped to STOPPING, then the clear reflex message was replaced by "component stopped: arms" | BUG |
| O9 | 21:02:09-21:02:42 | SHUTDOWN accepted while the gates were still exiting (they retry deactivation ~15 s with no controller manager) -> OFFLINE, BRING UP refused "gate already running", then ERROR re-appeared; operator pressed STOP 13x without feedback | BUG |
| O10 | 20:45:42, 20:46:03 | HOME: one arm DONE first -> supervisor stopped the other arm's gate during its final 1 s HOLD ("left: STOPPED") | BUG: if the other arm were still moving it would stop short of home |
| O11 | 20:41:49-20:42:28 | left in HOLD for 40 s: robot at snapshot, but GELLO moved during the ramp (j1 off 0.3-0.5 rad); operator could not find the 0.08 rad match | UX |
| O12 | 20:59:39 | START refused for both: left GELLO j4 -2.985 outside FR3 limit -2.977 (left holder pose j4 = -2.987, GELLO almost in holder but not detected as in holder) | UX: abort instead of waiting |
| O13 | 20:52:12 | START refused: "GELLO not still within 30 s" (both) | UX |
| O14 | 20:39-21:03 | Desk PREPARE / single switches worked every time; FCI-on before bringup OK | OK |
| O15 | all | ZED 22-29 fps after start, D405 #2 30 fps, D405 #1 ~10 fps (USB 2 adapter, accepted) | OK |
| O16 | all | recorder: camera "LOW" lines while cameras are off | noise (own tool) |

## My suggestions (priority order)
| # | Suggestion | Fixes | Effort |
|---|---|---|---|
| S1 | Gate LIVE: smooth the GELLO stream - relay at 200 Hz, interpolate between GELLO samples (~40 ms delay) and cap joint speed; sim test, then hardware | O7 (reflexes) | medium |
| S2 | Recorder logs per-arm external force (robot_state_10hz O_F_ext) and joint speeds during TELEOP, to verify S1 | O7 | small |
| S3 | STOP while in ERROR: no state change, reply "robots already stopped by the Franka" | O8, O9 | small |
| S4 | After an arm crash kill the gates at once (no controller manager left to ask); SHUTDOWN waits until gates are gone | O9 | small |
| S5 | HOME: a gate that ends DONE must not stop the other arm | O10 | small |
| S6 | Error message names the FIRST failure as the cause, later ones "(stopped as a consequence)" | O4 | small |
| S7 | HOLD re-sync: if the GELLO is still again but > 0.08 rad from the robot for 2 s, ramp again to the new GELLO pose | O11 | medium |
| S8 | Snapshot near a joint limit: keep waiting with "move GELLO <arm> j4 away from its limit" instead of aborting START | O12, O13 | small |
| S9 | GUI: after bringup with a robot in REFLEX, banner hint "press ERROR RECOVERY" | O3 | small |
| S10 | Recorder: no camera LOW while cameras are off | O16 | small |
| S11 | Consider a max start distance again (or a slower ramp for big moves) to reduce ramp self-collisions; Franka does not check arm-arm collisions | O3 | decision |
