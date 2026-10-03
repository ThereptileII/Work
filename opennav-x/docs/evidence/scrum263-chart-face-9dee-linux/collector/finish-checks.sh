#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
root=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
[[ $(git -C "$root" rev-parse HEAD) == 9dee9b148f4d6ebdd20bb4c49229fe19340df209 ]]
[[ $(cat "$prep/output/build.exit") == 2 && ! -e "$prep/output/checks.exit" ]]
grep -q 'Unable to initialize GTK+' "$prep/output/font-probe-initial.log"
trap 'result=$?; echo "$result" > "$prep/output/checks.exit"' EXIT
bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp \
 --bind "$root" "$root" --bind "$prep" "$prep" --setenv GIT_OPTIONAL_LOCKS 0 \
 --setenv HOME "$root/.local/combined-test-home" --setenv XDG_RUNTIME_DIR "$root/.local/combined-test-runtime" \
 --setenv XDG_CONFIG_HOME "$root/.local/combined-test-home/.config" --setenv XDG_DATA_HOME "$root/.local/combined-test-home/.local/share" \
 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY --chdir "$root" bash -s -- "$prep" <<'INNER'
set -euo pipefail
prep=$1
source tools/local-env.sh
xvfb-run -n 249 -e "$prep/output/font-probe-xvfb.log" build/xnav-linux/ui_font_resolution_test 2>&1 | tee "$prep/output/ui-font-resolution.log"
dbus-run-session -- ctest --test-dir build/xnav-linux/test --output-on-failure --no-tests=error -E '^tests$' --timeout 90 --output-junit "$prep/output/linux-tests.xml" 2>&1 | tee "$prep/output/ctest.log"
INNER
