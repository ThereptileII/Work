#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/scrum265-27e93e4
prep="$physical/$relative"
[[ ! -e "$prep/output/loader-run.log" && ! -e "$prep/output/loader-run.exit" ]]
trap 'result=$?; echo "$result" > "$prep/output/loader-run.exit"' EXIT
exec >"$prep/output/loader-run.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --ro-bind "$physical" "$virtual" \
 --bind "$prep" "$virtual/$relative" --chdir "$virtual" bash -s <<'INNER'
set -euo pipefail
source tools/local-env.sh
xvfb-run -a -s '-screen 0 1280x800x24 -nolisten tcp' python3 evidence/local/scrum265-27e93e4/verify-loader-once.py \
 --source build/integration-source \
 --generated /home/standard/Projects/X-nav-worktrees/scrum265-building-point/.local/building-point/generated \
 --output evidence/local/scrum265-27e93e4/loader \
 --wx-config /home/standard/Projects/X-nav/.local/sysroot/usr/bin/wx-config \
 --wx-prefix /home/standard/Projects/X-nav/.local/sysroot/usr \
 --seamarks --private-source /home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source
INNER
