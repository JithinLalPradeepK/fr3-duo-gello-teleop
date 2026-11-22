#!/usr/bin/env bash
# Command-line client for demo_supervisor (same services the GUI and pedals use).
#   demo_cmd.sh start|stop|home|recover|bringup|shutdown|scan
#   demo_cmd.sh gripper left|right open|close
#   demo_cmd.sh status            # one status snapshot, pretty-printed
#   demo_cmd.sh watch             # status every second (Ctrl-C to quit)
set +u; source "$HOME/robot_demo/env/host_env.sh"; set -u
call() { timeout 10 ros2 service call "/demo/$1" std_srvs/srv/Trigger 2>/dev/null | grep -oE "success=[A-Za-z]+, message=.*\)" || echo "no answer within 10 s - is demo_supervisor running?"; }
status() {
  timeout 5 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
    --field data /demo/status std_msgs/msg/String 2>/dev/null | head -1 |
    python3 -c "
import json, sys
raw = sys.stdin.read().strip()
if not raw:
    sys.exit('no status - is demo_supervisor running?')
s = json.loads(raw.strip(\"'\").encode().decode('unicode_escape') if raw.startswith(\"'\") else raw)
print(f\"STATE {s['state']}: {s['message']}\")
for arm, a in s['arms'].items():
    errs = ', '.join(a['errors']) or 'none'
    print(f\"  {arm:5}  robot {a['robot_mode']:12} errors: {errs:20} gate: {a['gate']:30} gello: {a['gello']:14} gripper: {a['gripper']}\")
print('  cameras', s['cameras'], s.get('cameras_launch', ''), ' pedal:', s['pedal'], ' last:', s['last_event'])
"
}
case "${1:-}" in
  start|stop|home|recover|bringup|shutdown|scan) call "$1";;
  gripper) call "gripper/${2:?left|right}/${3:?open|close}";;
  status) status;;
  watch) while true; do clear; date +%T; status; sleep 1; done;;
  *) sed -n 2,7p "$0"; exit 64;;
esac
