#!/usr/bin/env bash
# Start the whole robot demo: desk_agent + demo_supervisor + foot pedal node + GUI.
#   ~/robot_demo/scripts/start_demo.sh          (also what the "Robot Demo" desktop icon runs)
# The demo starts OFFLINE (user decision 2026-11-20): press PREPARE ROBOTS (Desk: control token, brakes open,
# Execution, FCI on), then BRING UP in the GUI. Bringup makes both Robotiq grippers open/close once.
# The arms only move on START / HOME.
# Closing the GUI (or Ctrl-C / SIGTERM here) shuts everything down: pedal, then the supervisor (gates first,
# robots hold, then arm stacks, grippers, GELLO, cameras), then desk_agent, which releases the Desk control
# token (-> FCI off). Brakes and operating mode are left as they are.
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

SUP_PID=""; PEDAL_PID=""; GUI_PID=""; DESK_PID=""
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

log "starting GUI"
python3 "$DEMO/scripts/demo_gui.py" --owns-demo > "$LOGDIR/gui.log" 2>&1 &
GUI_PID=$!
wait "$GUI_PID"
log "GUI closed"
GUI_PID=""
cleanup
