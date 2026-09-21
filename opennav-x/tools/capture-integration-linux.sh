#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
if [[ -x .local/sysroot/usr/bin/cmake ]]; then source tools/local-env.sh; fi
mode="${1:-xnav}"
case "$mode" in xnav|legacy|safe-mode) ;; *) exit 2 ;; esac
mkdir -p evidence/local build/profiles
profile="$root/build/profiles/$mode-$(date +%s)-$$"
python tools/prepare-test-profile.py --build build/xnav-linux --profile "$profile"
display_number=94
while [[ -e "/tmp/.X${display_number}-lock" ]]; do ((display_number+=1)); done
export DISPLAY=":$display_number"
Xvfb "$DISPLAY" -screen 0 1280x800x24 -nolisten tcp > evidence/local/xnav-xvfb.log 2>&1 &
xvfb_pid=$!
app_pid=''
cleanup() {
  if [[ -n "$app_pid" ]]; then kill "$app_pid" 2>/dev/null || true; fi
  kill "$xvfb_pid" 2>/dev/null || true
  cp -r "$profile" "evidence/local/$mode-linux-profile" || true
}
trap cleanup EXIT
sleep 1
build/xnav-install/bin/opencpn --configdir "$profile" --no_opengl "--$mode" \
  > "evidence/local/$mode-linux-launch.log" 2>&1 &
app_pid=$!
window=''
for ((attempt=0; attempt<60; attempt++)); do
  kill -0 "$app_pid"
  window=$(xdotool search --onlyvisible --pid "$app_pid" --name '^(OpenNav X / OpenCPN|OpenCPN / Legacy|OpenNav Safe Mode / OpenCPN)$' | head -1 || true)
  if [[ -n "$window" ]] && rg -q 'OnInitTimer.*Finalize Canvases' "$profile/opencpn.log"; then break; fi
  sleep 1
done
[[ -n "$window" ]] || { printf 'OpenNav window not found\n' >&2; exit 1; }
rg -q 'OnInitTimer.*Finalize Canvases' "$profile/opencpn.log"
xdotool windowsize "$window" 1280 800
xdotool windowmove "$window" 0 0
sleep 1
import -window root "evidence/local/$mode-linux.png"
# The integrated timer guard is explicitly exercised by IPC quit here.
build/xnav-install/bin/opencpn --configdir "$profile" --remote --quit
for ((attempt=0; attempt<30; attempt++)); do
  if ! kill -0 "$app_pid" 2>/dev/null; then break; fi
  sleep 1
done
if kill -0 "$app_pid" 2>/dev/null; then
  printf 'OpenNav did not close gracefully\n' >&2
  exit 1
fi
wait "$app_pid"
app_pid=''
printf 'Linux development evidence only; software rendering; no live chart or vessel input.\n' \
  > "evidence/local/$mode-linux.txt"
