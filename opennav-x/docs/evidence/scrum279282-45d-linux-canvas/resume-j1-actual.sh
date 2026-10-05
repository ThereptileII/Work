#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
prep="$physical/evidence/local/combined-symbols-45d73e8"
[[ ! -e "$prep/output/resume-j1-actual.log" && ! -e "$prep/output/resume-j1-actual.exit" ]]
trap 'result=$?; echo "$result" > "$prep/output/resume-j1-actual.exit"' EXIT
exec >"$prep/output/resume-j1-actual.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --bind "$physical" "$virtual" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv HOME "$virtual/.local/ca-e6-home" \
 --setenv XDG_RUNTIME_DIR "$virtual/.local/ca-e6-runtime" \
 --setenv XDG_CONFIG_HOME "$virtual/.local/ca-e6-home/.config" \
 --setenv XDG_DATA_HOME "$virtual/.local/ca-e6-home/.local/share" \
 --chdir "$virtual" bash -s <<'INNER'
set -euo pipefail
[[ $(git rev-parse HEAD) == 45d73e831409d602e8205f32fcba08947ea370bc ]]
[[ -z $(git status --porcelain --untracked-files=no) ]]
[[ -d .git && ! -L .git ]]
mkdir -p .local/ca-e6-home/.opencpn .local/ca-e6-runtime
chmod 700 .local/ca-e6-runtime
source tools/local-env.sh
time cmake --build build/xnav-linux --target opencpn --parallel 1
cmake --install build/xnav-linux
INNER
