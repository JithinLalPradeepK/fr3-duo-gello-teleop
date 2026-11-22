#!/usr/bin/env bash
# Start the whole robot demo: desk_agent + demo_supervisor + foot pedal node + GUI.
#   ~/robot_demo/scripts/start_demo.sh          (also what the "Robot Demo" desktop icon runs)
# The demo starts OFFLINE (user decision 2026-11-20): press PREPARE ROBOTS (Desk: control token, brakes open,
# Execution, FCI on), then BRING UP in the GUI. Bringup makes both Robotiq grippers open/close once.
# The arms only move on START / HOME.
# Closing the GUI (or Ctrl-C / SIGTERM here) shuts everything down: pedal, then the supervisor (gates first,
# robots hold, then arm stacks, grippers, GELLO, cameras), then desk_agent, which releases the Desk control
# token (-> FCI off). Brakes and operating mode are left as they are.
# Display 2 shows the 3D DIGITAL TWIN (launch/digital_twin.launch.py, read-only: robot model from the measured joint
# states, overlays, point clouds). DEMO_TWIN=0 start_demo.sh = old behaviour (camera page on display 2, no twin).
# Default view on display 2 = PHOTOREAL (Isaac Sim, scripts/start_isaac_twin.sh); DEMO_TWIN_VIEW=rviz = the RViz twin.
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
export DISPLAY="${DISPLAY:-:1}"
LOGDIR="$DEMO/logs/start_demo_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGDIR"
log() { echo "[start_demo $(date +%T)] $*" | tee -a "$LOGDIR/start_demo.log"; }
notify() { command -v notify-send >/dev/null && notify-send -t 6000 "Robot Demo" "$1"; log "$1"; }

if pgrep -f '^(/usr/bin/)?python3 \S*demo_supervisor\.py' >/dev/null; then
  notify "Robot Demo is already running."; exit 1
fi
if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node|^\S*fr3_duo_home( |$)' >/dev/null; then
  notify "Refused: a robot stack or homing is already running (teleop_duo.sh / fr3_duo_home?)."; exit 1
fi

SUP_PID=""; PEDAL_PID=""; GUI_PID=""; DESK_PID=""; TWIN_PID=""; PHOTO_PID=""
TWIN="${DEMO_TWIN:-1}"
if [[ $TWIN == 1 ]] && pgrep -f '^\S*python3 \S*ros2 launch \S*digital_twin\.launch\.py' >/dev/null; then
  log "a digital twin is already running - using it (not starting a second RViz)"; TWIN=reuse
fi
stop_pid() {  # pid timeout_s
  local pid=$1 t=$2
  [[ -z "$pid" ]] && return 0
  kill -INT "$pid" 2>/dev/null
  for ((i = 0; i < t * 10; i++)); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
  log "pid $pid still alive after ${t}s -> SIGTERM"; kill -TERM "$pid" 2>/dev/null; return 1
}
CLEANED=0
cleanup() {
  [[ $CLEANED == 1 ]] && return; CLEANED=1
  trap '' INT TERM
  log "shutting down"
  [[ -n "$GUI_PID" ]] && stop_pid "$GUI_PID" 5
  [[ -n "$PEDAL_PID" ]] && stop_pid "$PEDAL_PID" 5
  [[ -n "${PHOTO_PID:-}" ]] && { log "stopping photoreal renderer"; kill -TERM "$PHOTO_PID" 2>/dev/null; stop_pid "$PHOTO_PID" 20; }
  [[ -n "$TWIN_PID" ]] && { log "stopping digital twin (read-only)"; stop_pid "$TWIN_PID" 10; }
  [[ -n "$SUP_PID" ]] && { log "stopping supervisor (gates, then stack)"; stop_pid "$SUP_PID" 90; }
  [[ -n "$DESK_PID" ]] && { log "stopping desk_agent (releases Desk control -> FCI off)"; stop_pid "$DESK_PID" 70; }
  log "done. logs: $LOGDIR"
}
trap 'cleanup; exit 130' INT TERM

log "starting desk_agent"
setsid python3 "$DEMO/scripts/desk_agent.py" > "$LOGDIR/desk_agent.log" 2>&1 &
DESK_PID=$!

log "starting demo_supervisor (OFFLINE - press PREPARE ROBOTS, then BRING UP)"
setsid python3 "$DEMO/scripts/demo_supervisor.py" > "$LOGDIR/supervisor.log" 2>&1 &
SUP_PID=$!
for ((i = 0; i < 30; i++)); do
  kill -0 "$SUP_PID" 2>/dev/null || { notify "Supervisor failed to start - see $LOGDIR/supervisor.log"; cleanup; exit 1; }
  grep -q 'STARTING_UP\|IDLE\|OFFLINE' "$LOGDIR/supervisor.log" && break
  sleep 0.5
done

log "starting pedal node"
setsid python3 "$DEMO/scripts/demo_pedal.py" > "$LOGDIR/pedal.log" 2>&1 &
PEDAL_PID=$!

export DEMO_TWIN_VIEW="${DEMO_TWIN_VIEW:-photoreal}"     # display 2: photoreal (Isaac Sim) | rviz
if [[ $TWIN == 1 ]]; then
  if [[ $DEMO_TWIN_VIEW == photoreal && -x "$HOME/isaacsim_env/bin/python" ]]; then
    log "starting digital twin (twin nodes without RViz + PHOTOREAL renderer for display 2, read-only)"
    setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/digital_twin.launch.py" rviz:=false \
      > "$LOGDIR/twin.log" 2>&1 &
    TWIN_PID=$!
    setsid "$DEMO/scripts/start_isaac_twin.sh" > "$LOGDIR/photoreal.log" 2>&1 &
    PHOTO_PID=$!
  else
    export DEMO_TWIN_VIEW=rviz
    log "starting digital twin (RViz on display 2, read-only)"
    setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/digital_twin.launch.py" > "$LOGDIR/twin.log" 2>&1 &
    TWIN_PID=$!
  fi
fi
SECOND=twin; [[ $TWIN == 0 ]] && SECOND=cameras

log "starting GUI"
python3 "$DEMO/scripts/demo_gui.py" --owns-demo --second-display "$SECOND" > "$LOGDIR/gui.log" 2>&1 &
GUI_PID=$!
wait "$GUI_PID"
log "GUI closed"
GUI_PID=""
cleanup
