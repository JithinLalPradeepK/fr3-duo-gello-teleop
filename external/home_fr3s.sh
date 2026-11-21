#!/usr/bin/env bash
# Move both FR3s to the saved home pose (no prompt). Standalone: libfranka only.
# Bound to foot pedal 3 via a GNOME custom shortcut.
#  - only one homing at a time (a second press is ignored)
#  - refuses while GELLO teleop / any ros2_control or franka controller is running
LOG="$HOME/fr3_tools/home_fr3s.log"
BIN="$HOME/fr3_tools/fr3_duo_home/build/fr3_duo_home"
note() { echo "$(date +%T) $1" >> "$LOG"; command -v notify-send >/dev/null && notify-send -t "${2:-3000}" "FR3" "$1"; }

exec 9>"$HOME/fr3_tools/.home.lock"
flock -n 9 || { note "Homing already running - ignored"; exit 0; }

if pgrep -f 'ros2_control_node|franka_fr3_arm_controllers|gello_gate' >/dev/null; then
  note "Homing refused: teleop / robot controllers are running. Stop them first." 6000
  exit 1
fi

echo "===== $(date)" >> "$LOG"
note "Homing both robots..."
if "$BIN" --yes --vel 0.3 >> "$LOG" 2>&1; then
  note "Both robots at home"
else
  note "Homing FAILED - see ~/fr3_tools/home_fr3s.log" 6000
  exit 1
fi
