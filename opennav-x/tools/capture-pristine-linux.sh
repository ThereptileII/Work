#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
if [[ -x .local/sysroot/usr/bin/cmake ]]; then source tools/local-env.sh; fi
mkdir -p evidence/local build/profiles
profile="$root/build/profiles/baseline-$(date +%s)-$$"
python tools/prepare-test-profile.py --build build/pristine-linux --profile "$profile"
# Private X server avoids interacting with the user's desktop/session.
display_number=93
while [[ -e "/tmp/.X${display_number}-lock" ]]; do ((display_number+=1)); done
export DISPLAY=":$display_number"
Xvfb "$DISPLAY" -screen 0 1280x800x24 -nolisten tcp > evidence/local/xvfb.log 2>&1 &
xvfb_pid=$!
app_pid=''
cleanup() {
  if [[ -n "$app_pid" ]]; then kill "$app_pid" 2>/dev/null || true; fi
  kill "$xvfb_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 1
build/pristine-install/bin/opencpn --configdir "$profile" --no_opengl \
  > evidence/local/linux-launch.log 2>&1 &
app_pid=$!
window=''
for ((attempt=0; attempt<60; attempt++)); do
  kill -0 "$app_pid"
  window=$(xdotool search --onlyvisible --pid "$app_pid" --name '^OpenCPN' | head -1 || true)
  if [[ -n "$window" ]]; then break; fi
  sleep 1
done
[[ -n "$window" ]] || { printf 'OpenCPN window not found\n' >&2; exit 1; }
xdotool windowsize "$window" 1280 800
xdotool windowmove "$window" 0 0
sleep 5
import -window root evidence/local/11-legacy-mode-linux.png
# Exercise the normal user exit path through the chart canvas.
xdotool windowfocus "$window"
xdotool key --clearmodifiers ctrl+q
for ((attempt=0; attempt<30; attempt++)); do
  if ! kill -0 "$app_pid" 2>/dev/null; then break; fi
  sleep 1
done
if kill -0 "$app_pid" 2>/dev/null; then
  printf 'OpenCPN did not close gracefully\n' >&2
  exit 1
fi
wait "$app_pid"
app_pid=''
cp -r "$profile" evidence/local/linux-baseline-profile
printf 'Linux development screenshot only; software rendering; no charts or vessel inputs loaded.\n' \
  > evidence/local/11-legacy-mode-linux.txt
