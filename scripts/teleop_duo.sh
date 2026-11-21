#!/usr/bin/env bash
# GATED GELLO teleop: arms (impedance controller inactive) -> Robotiq grippers -> GELLO on /gello_check_*
# -> one gello_gate per arm.
#
#   ~/robot_demo/scripts/teleop_duo.sh [--arms left,right] [--robot-config /abs/cfg.yaml] [--no-gello]
#                                      [--no-grippers] [--gate-args "--converge-tol 0.05 ..."]
#
# THIS MOVES THE ROBOTS (with the real robot config). Keep the E-stop in hand.
# - Holds ~/fr3_tools/.home.lock for the whole session: homing cannot run at the same time.
# - If one gate stops (refused / fault / signal), the other gates are stopped too.
# - Ctrl-C / SIGTERM: gates first (controllers deactivated, robots hold), then GELLO, grippers, arms.
# - Grippers: robotiq.launch.py per arm (ports from franka_gripper_manager example_fr3_duo_config_robotiq.yaml).
#   THE FINGERS OPEN/CLOSE ONCE AT STARTUP (Robotiq activation). The gate sends GELLO gripper targets
#   only in LIVE; otherwise the gripper keeps its last position.
# - --no-gello: don't start the GELLO publishers (sim tests provide fake GELLO data).
# - --no-grippers: don't start the Robotiq grippers (sim tests).
set -u
ARMS="left,right"; ROBOT_CFG=""; START_GELLO=1; START_GRIPPERS=1; GATE_ARGS=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --arms) ARMS="$2"; shift 2;;
    --robot-config) ROBOT_CFG="$2"; shift 2;;
    --no-gello) START_GELLO=0; shift;;
    --no-grippers) START_GRIPPERS=0; shift;;
    --gate-args) GATE_ARGS="$2"; shift 2;;
    -h|--help) sed -n 2,17p "$0"; exit 0;;
    *) echo "unknown argument: $1" >&2; exit 64;;
  esac
done
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO="$(dirname "$HERE")"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u   # ROS setup files are not set -u safe
LOGDIR="$DEMO/logs/teleop_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGDIR"
log() { echo "[teleop $(date +%T)] $*" | tee -a "$LOGDIR/teleop.log"; }
IFS=',' read -r -a ARM_LIST <<< "$ARMS"

exec 9>"$HOME/fr3_tools/.home.lock"
flock -n 9 || { log "REFUSED: homing is running (lock held)"; exit 1; }
if pgrep -f '^\S*fr3_duo_home( |$)' >/dev/null; then log "REFUSED: fr3_duo_home is running"; exit 1; fi
if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py' >/dev/null; then
  log "REFUSED: ros2_control_node / gello_gate already running (another session?)"; exit 1
fi

ARMS_PID=""; GELLO_PID=""; declare -A GATE_PID=(); declare -A GRIP_PID=()
stop_pid() {  # pid signal timeout_s
  local pid=$1 sig=$2 t=$3
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill "-$sig" "$pid" 2>/dev/null
  for ((i = 0; i < t * 10; i++)); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
  log "pid $pid still alive after SIG$sig ${t}s -> SIGTERM/SIGKILL"
  kill -TERM "$pid" 2>/dev/null; sleep 3; kill -KILL "$pid" 2>/dev/null; return 1
}
CLEANED=0
cleanup() {
  [[ $CLEANED == 1 ]] && return; CLEANED=1
  trap '' INT TERM
  log "shutting down: gates first"
  for a in "${!GATE_PID[@]}"; do kill -TERM "${GATE_PID[$a]}" 2>/dev/null; done
  for a in "${!GATE_PID[@]}"; do stop_pid "${GATE_PID[$a]}" TERM 15; done
  [[ -n "$GELLO_PID" ]] && { log "stopping GELLO"; stop_pid "$GELLO_PID" INT 10; }
  for a in "${!GRIP_PID[@]}"; do log "stopping gripper $a"; stop_pid "${GRIP_PID[$a]}" INT 10; done
  [[ -n "$ARMS_PID" ]] && { log "stopping arms"; stop_pid "$ARMS_PID" INT 20; }
  log "done. logs: $LOGDIR"
}
trap 'cleanup; exit 130' INT TERM

ARMS_ARGS=("arms:=$ARMS"); [[ -n "$ROBOT_CFG" ]] && ARMS_ARGS+=("robot_config:=$ROBOT_CFG")
log "starting arms (${ARMS_ARGS[*]})"
setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/arms_gated.launch.py" "${ARMS_ARGS[@]}" > "$LOGDIR/arms.log" 2>&1 &
ARMS_PID=$!

for a in "${ARM_LIST[@]}"; do
  log "waiting for /$a controllers (impedance inactive, broadcaster active)"
  ok=0
  for ((i = 0; i < 60; i++)); do
    kill -0 "$ARMS_PID" 2>/dev/null || { log "arm launch died - see $LOGDIR/arms.log"; cleanup; exit 1; }
    out=$(timeout 5 ros2 control list_controllers -c "/$a/controller_manager" 2>/dev/null)
    if grep -Eq 'joint_impedance_controller.*inactive' <<< "$out" &&
       grep -Eq 'joint_state_broadcaster.*active' <<< "$out" &&
       ! grep -Eq 'joint_state_broadcaster.*inactive' <<< "$out"; then ok=1; break; fi
    sleep 1
  done
  [[ $ok == 1 ]] || { log "/$a controllers not ready after 60 s"; cleanup; exit 1; }
done

if [[ $START_GRIPPERS == 1 ]]; then
  GRIP_CFG="$(ros2 pkg prefix franka_gripper_manager)/share/franka_gripper_manager/config/example_fr3_duo_config_robotiq.yaml"
  for a in "${ARM_LIST[@]}"; do
    port=$(python3 -c "import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
print(next(v['com_port'] for v in c.values() if v['namespace'] == sys.argv[2] + '/gripper'))" "$GRIP_CFG" "$a") ||
      { log "no gripper entry for $a in $GRIP_CFG"; cleanup; exit 1; }
    [[ -e "$port" ]] || { log "gripper $a port $port not present"; cleanup; exit 1; }
    log "starting gripper $a ($port) - fingers open/close once (activation)"
    setsid "$DEMO/scripts/sigint_default" ros2 launch franka_gripper_manager robotiq.launch.py \
      "namespace:=$a/gripper" "com_port:=$port" > "$LOGDIR/gripper_$a.log" 2>&1 &
    GRIP_PID[$a]=$!
  done
  for a in "${ARM_LIST[@]}"; do
    ok=0
    for ((i = 0; i < 30; i++)); do
      kill -0 "${GRIP_PID[$a]}" 2>/dev/null || { log "gripper $a launch died - see $LOGDIR/gripper_$a.log"; cleanup; exit 1; }
      grep -q 'Gripper action server is up' "$LOGDIR/gripper_$a.log" && { ok=1; break; }
      sleep 1
    done
    [[ $ok == 1 ]] || { log "gripper $a not ready after 30 s"; cleanup; exit 1; }
    log "gripper $a ready"
  done
fi

if [[ $START_GELLO == 1 ]]; then
  log "starting GELLO publishers on /gello_check_*"
  setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/gello_check.launch.py" "arms:=$ARMS" > "$LOGDIR/gello.log" 2>&1 &
  GELLO_PID=$!
fi

for a in "${ARM_LIST[@]}"; do
  log "starting gate $a"
  # shellcheck disable=SC2086
  python3 "$DEMO/scripts/gello_gate.py" --arm "$a" $GATE_ARGS > >(tee -i -a "$LOGDIR/gate_$a.log") 2>&1 &   # -i: Ctrl-C must not kill the log
  GATE_PID[$a]=$!
done

# First gate to end (any reason) stops the others.
wait -n "${GATE_PID[@]}"; rc=$?
for a in "${!GATE_PID[@]}"; do
  kill -0 "${GATE_PID[$a]}" 2>/dev/null || log "gate $a ended"
done
log "a gate ended with code $rc (0 stopped, 2 refused, 3 fault, 4 DEACTIVATION FAILED)"
cleanup
exit "$rc"
