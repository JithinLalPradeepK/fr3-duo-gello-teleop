#!/usr/bin/env bash
# SIM test of demo_supervisor. Fake hardware only, ROS_DOMAIN_ID=77, localhost only. No robot connection.
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_CFG="$DEMO/config/fr3_duo_sim.yaml"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$DEMO/test/cyclonedds_sim.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
LOGDIR="$DEMO/logs/simsup_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGDIR"
echo "logs: $LOGDIR"
if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py|^(/usr/bin/)?python3 \S*demo_supervisor\.py' >/dev/null; then
  echo "REFUSING: ros2_control_node / gate / supervisor already running"; exit 1
fi
python3 "$DEMO/test/sim_supervisor_test.py" "$LOGDIR" "$SIM_CFG" 2>&1 | tee "$LOGDIR/test.log"
rc=${PIPESTATUS[0]}
# never leave anything behind
sim_only_kill() {  # SIGINT only processes of THIS sim (ROS_DOMAIN_ID=77), never a real stack
  for p in $(pgrep -f "$1"); do
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=77$' && kill -INT "$p"
  done
}
sim_only_kill '^(/usr/bin/)?python3 \S*demo_supervisor\.py'
sleep 2
sim_only_kill '^/opt/ros/humble/lib/controller_manager/ros2_control_node'
echo "== RESULT $rc (logs: $LOGDIR)"
exit "$rc"
