#!/usr/bin/env bash
# Digital-twin DISPLAY test on the real screen: twin launch WITH RViz + fake robot data, ROS_DOMAIN_ID=78, localhost
# only. No robot, camera or supervisor. Opens the twin window full screen on the second monitor for ~40 s.
#   test/sim_twin_gui_test.sh
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 DISPLAY="${DISPLAY:-:1}"
export CYCLONEDDS_URI="file://$DEMO/test/cyclonedds_sim.xml"
OUT="$DEMO/logs/simtwingui_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
sim_only_kill() {
  for p in $(pgrep -f 'twin_|topic_tools/throttle|robot_state_publisher|digital_twin.launch|rviz2'); do
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=78$' && kill -INT "$p" 2>/dev/null
  done
  sleep 3
  for p in $(pgrep -f 'twin_|topic_tools/throttle|robot_state_publisher|digital_twin.launch|rviz2'); do   # respawned RViz
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=78$' && kill -TERM "$p" 2>/dev/null
  done
}
trap sim_only_kill EXIT
setsid "$DEMO/scripts/sigint_default" ros2 launch "$DEMO/launch/digital_twin.launch.py" > "$OUT/launch.log" 2>&1 &
setsid python3 "$DEMO/test/twin_fake_robots.py" > "$OUT/fake.log" 2>&1 &
sleep 4
python3 "$DEMO/test/twin_gui_test.py" "$OUT" 2>&1 | grep -v multicast | tee "$OUT/result.txt"
exit "${PIPESTATUS[0]}"
