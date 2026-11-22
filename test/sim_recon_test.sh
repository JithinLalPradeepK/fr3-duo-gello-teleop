#!/usr/bin/env bash
# SIM test of the twin reconstruction (scripts/twin_recon.py). Fake robots + fake ZED/D405 depth only, ROS_DOMAIN_ID=79,
# localhost only. No robot, camera or GUI.   test/sim_recon_test.sh
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$DEMO/test/cyclonedds_sim.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
OUT="$DEMO/logs/simrecon_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
sim_only_kill() {  # SIGINT only processes of THIS sim (ROS_DOMAIN_ID=79)
  for p in $(pgrep -f 'twin_|topic_tools/throttle|robot_state_publisher|digital_twin.launch'); do
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=79$' && kill -INT "$p" 2>/dev/null
  done
}
trap sim_only_kill EXIT
python3 "$DEMO/test/sim_recon_test.py" "$OUT" 2>&1 | tee "$OUT/result.txt"
exit "${PIPESTATUS[0]}"
