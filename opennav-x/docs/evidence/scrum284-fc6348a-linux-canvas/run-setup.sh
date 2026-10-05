#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/circle-fc6348a/iho
prep="$physical/$relative"
[[ $(cat "$physical/evidence/local/circle-fc6348a/output/build.exit") == 0 ]]
[[ ! -e "$prep/output/setup.exit" && ! -e "$prep/output/setup.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/setup.exit"' EXIT
exec >"$prep/output/setup.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --ro-bind "$physical" "$virtual" \
 --bind "$prep" "$virtual/$relative" --ro-bind "$prep/fixture" "$virtual/$relative/fixture" \
 --ro-bind "$prep/seattle-fixture" "$virtual/$relative/seattle-fixture" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv SKAGER_PRIVATE_CACHE "$virtual/$relative/inputs" \
 --setenv SKAGER_CAPTURE_SCRATCH "$virtual/$relative/output" \
 --chdir "$virtual/$relative" bash -s <<'INNER'
set -euo pipefail
python3 seal-inputs.py --expected-commit fc6348a6df8374878802b3e051a3670d83ed9a56
python3 compile-layout.py
INNER
