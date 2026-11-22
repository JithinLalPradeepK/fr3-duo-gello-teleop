#!/usr/bin/env bash
# PHOTOREAL digital twin (NVIDIA Isaac Sim 6.2, RTX real-time path tracing) in the BROWSER (default) or a window.
#   scripts/start_isaac_twin.sh [--open] [twin_isaac.py args]   -> browser UI http://127.0.0.1:8211/ (stream :8212)
#   scripts/start_isaac_twin.sh --window [--windowed --x 0]     -> old full-screen desktop window on display 2
# The browser UI can move the robots (SCAN / HOME / grippers via the supervisor): it binds to this computer only.
# Needs: the demo / digital twin running (twin_state, twin_recon publish what is drawn), Isaac Sim in ~/isaacsim_env
# (isolated Python 3.12 venv; ROS / CUDA / driver untouched). Ctrl-C stops both processes.
# 1. scripts/twin_isaac_bridge.py (system ROS): URDF, joint states over UDP, mesh / live objects / ZED photo files
# 2. isaac/convert_twin.py once (URDF -> USD, re-run when the twin URDF changes), 3. isaac/twin_isaac.py (renderer)
set -u
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_PY="${ISAAC_PY:-$HOME/isaacsim_env/bin/python}"
MODE=(--stream --headless); OPEN=0; ARGS=()
for x in "$@"; do
  case "$x" in --window) MODE=();; --open) OPEN=1;; *) ARGS+=("$x");; esac
done
[[ -x "$ISAAC_PY" ]] || { echo "Isaac Sim venv not found ($ISAAC_PY)"; exit 1; }
if pgrep -f '^\S*python3? \S*twin_isaac\.py' >/dev/null; then echo "photoreal twin already running"; exit 1; fi
LOG="$DEMO/logs/isaac_twin_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG"
export OMNI_KIT_ACCEPT_EULA=YES
# shellcheck disable=SC1091
set +u; source "$DEMO/env/host_env.sh"; set -u
python3 "$DEMO/scripts/twin_isaac_bridge.py" > "$LOG/bridge.log" 2>&1 &
BRIDGE=$!
cleanup() {   # TERM: children of a background script ignore SIGINT
  kill -TERM "$BRIDGE" 2>/dev/null; [[ -n "${APP:-}" ]] && kill -TERM "$APP" 2>/dev/null; wait 2>/dev/null; }
trap 'cleanup; exit 130' INT TERM
for _ in $(seq 1 50); do [[ -s "$DEMO/isaac/cache/fr3_duo_twin.urdf" ]] && break; sleep 0.2; done
URDF="$DEMO/isaac/cache/fr3_duo_twin.urdf"; USD="$DEMO/isaac/cache/usd/fr3_duo_twin/fr3_duo_twin.usda"
if [[ ! -f "$USD" || "$URDF" -nt "$USD" ]] && ! cmp -s "$URDF" "$DEMO/isaac/cache/converted.urdf"; then
  echo "converting the twin URDF to USD (once, ~1 min)"
  # env -u: the Isaac venv must not see the system ROS Python paths
  env -u PYTHONPATH -u AMENT_PREFIX_PATH "$ISAAC_PY" "$DEMO/isaac/convert_twin.py" > "$LOG/convert.log" 2>&1 \
    && cp "$URDF" "$DEMO/isaac/cache/converted.urdf" || { echo "USD conversion failed ($LOG/convert.log)"; cleanup; exit 1; }
fi
echo "photoreal twin starting (first start compiles shaders: ~1-2 min). Logs: $LOG"
env -u PYTHONPATH -u AMENT_PREFIX_PATH "$ISAAC_PY" "$DEMO/isaac/twin_isaac.py" "${MODE[@]}" "${ARGS[@]}" > "$LOG/isaac.log" 2>&1 &
APP=$!
if [[ ${#MODE[@]} -gt 0 ]]; then
  echo "browser UI: http://127.0.0.1:8211/  (live view appears when the renderer is up)"
  [[ $OPEN == 1 ]] && (firefox --new-window http://127.0.0.1:8211/ >/dev/null 2>&1 &)
fi
wait "$APP"
cleanup
