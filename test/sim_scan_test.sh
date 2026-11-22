#!/usr/bin/env bash
# SIM test of the 3D SCAN in demo_supervisor. Fake hardware only, ROS_DOMAIN_ID=77, localhost only. No robot connection.
# (Unlike sim_supervisor_test.sh this may run while a real demo is OPEN but OFFLINE: everything here is domain 77,
#  own lock / home / holder files. It refuses if a real ARM STACK runs.)
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$DEMO/test/cyclonedds_sim.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
LOGDIR="$DEMO/logs/simscan_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGDIR"
if pgrep -f '^/opt/ros/humble/lib/controller_manager/ros2_control_node|^(/usr/bin/)?python3 \S*gello_gate\.py' >/dev/null; then
  echo "REFUSING: an arm stack / gate is running"; exit 1
fi
python3 "$DEMO/test/sim_scan_test.py" "$LOGDIR" 2>&1 | tee "$LOGDIR/test.log"
rc=${PIPESTATUS[0]}
for p in $(pgrep -f 'demo_supervisor\.py|ros2_control_node|gello_gate\.py|ros2 launch'); do
  tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=77$' && kill -INT "$p" 2>/dev/null
done
echo "== RESULT $rc (logs: $LOGDIR)"
exit "$rc"
