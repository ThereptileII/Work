#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
prep="$physical/evidence/local/ca-light-e6ef384"
[[ ! -e "$prep/output/lifetime-object.log" && ! -e "$prep/output/lifetime-object.exit" ]]
trap 'result=$?; echo "$result" > "$prep/output/lifetime-object.exit"' EXIT
exec >"$prep/output/lifetime-object.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --bind "$physical" "$virtual" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv HOME "$virtual/.local/ca-e6-home" \
 --setenv XDG_RUNTIME_DIR "$virtual/.local/ca-e6-runtime" \
 --setenv XDG_CONFIG_HOME "$virtual/.local/ca-e6-home/.config" \
 --setenv XDG_DATA_HOME "$virtual/.local/ca-e6-home/.local/share" \
 --chdir "$virtual" bash -s <<'INNER'
set -euo pipefail
[[ $(git rev-parse HEAD) == c24c15482293ad2e2f3ce4e1c8af2ca5669381de ]]
[[ -z $(git status --porcelain --untracked-files=no) ]]
[[ -d .git && ! -L .git ]]
mkdir -p .local/ca-e6-home/.opencpn .local/ca-e6-runtime
chmod 700 .local/ca-e6-runtime
source tools/local-env.sh
cmake --build build/xnav-linux --target CMakeFiles/opencpn.dir/gui/src/s57chart.cpp.o --parallel 1
INNER
