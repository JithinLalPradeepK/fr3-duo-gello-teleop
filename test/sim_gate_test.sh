#!/usr/bin/env bash
# SIM test of the start gate. Fake hardware only, ROS_DOMAIN_ID=77, localhost only. No robot connection.
#   ~/robot_demo/test/sim_gate_test.sh [single|duo|all]   (default all)
set -u
MODE="${1:-all}"
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_CFG="$DEMO/config/fr3_duo_sim.yaml"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u   # ROS setup files are not set -u safe
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$DEMO/test/cyclonedds_sim.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
LOGDIR="$DEMO/logs/simtest_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGDIR"
echo "logs: $LOGDIR"

python3 - "$SIM_CFG" <<'EOF' || exit 1
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
bad = [k for k, c in cfg.items() if str(c.get('use_fake_hardware')).lower() != 'true']
if bad:
    sys.exit(f'REFUSING: {sys.argv[1]} arms {bad} not use_fake_hardware "true"')
print('sim config OK: all arms fake hardware')
EOF
if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py|^\S*fr3_duo_home( |$)' >/dev/null; then
  echo "REFUSING: ros2_control_node / gello_gate / fr3_duo_home already running"; exit 1
fi

rc_single=0; rc_duo=0
if [[ $MODE == single || $MODE == all ]]; then
  echo "== single-arm scenarios (sim left)"
  setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/arms_gated.launch.py" arms:=left robot_config:="$SIM_CFG" \
    > "$LOGDIR/arms_single.log" 2>&1 &
  ARMS_PID=$!
  trap 'kill -INT $ARMS_PID 2>/dev/null; exit 130' INT TERM
  ready=0
  for i in $(seq 1 60); do
    if timeout 5 ros2 control list_controllers -c /left/controller_manager 2>/dev/null |
       grep -q 'joint_impedance_controller.*inactive'; then ready=1; break; fi
    kill -0 $ARMS_PID 2>/dev/null || break
    sleep 1
  done
  if [[ $ready == 1 ]]; then
    python3 "$DEMO/test/sim_gate_driver.py" single --arm left --cm-log "$LOGDIR/arms_single.log" \
      --logdir "$LOGDIR" 2>&1 | tee "$LOGDIR/driver_single.log"
    rc_single=${PIPESTATUS[0]}
  else
    echo "sim arm launch not ready - see $LOGDIR/arms_single.log"; rc_single=1
  fi
  kill -INT $ARMS_PID 2>/dev/null
  for i in $(seq 1 100); do kill -0 $ARMS_PID 2>/dev/null || break; sleep 0.2; done
  kill -0 $ARMS_PID 2>/dev/null && kill -TERM $ARMS_PID
  trap - INT TERM
  for i in $(seq 1 30); do pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node' >/dev/null || break; sleep 1; done
  if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node' >/dev/null; then
    echo "sim ros2_control_node did not exit - aborting before duo"; exit 1
  fi
fi
if [[ $MODE == duo || $MODE == all ]]; then
  echo "== duo scenarios via teleop_duo.sh (sim config)"
  python3 "$DEMO/test/sim_gate_driver.py" duo --sim-config "$SIM_CFG" --logdir "$LOGDIR" 2>&1 |
    tee "$LOGDIR/driver_duo.log"
  rc_duo=${PIPESTATUS[0]}
fi
echo "== RESULT single=$rc_single duo=$rc_duo  (logs: $LOGDIR)"
exit $(( rc_single || rc_duo ))
