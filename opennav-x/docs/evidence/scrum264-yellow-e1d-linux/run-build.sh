#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum264-combined-linux-33d90c3
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
prep="$physical/evidence/local/combined-e1d0136"
[[ ! -e "$prep/output/build.log" && ! -e "$prep/output/build.exit" ]]
trap 'result=$?; echo "$result" > "$prep/output/build.exit"' EXIT
exec >"$prep/output/build.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --bind "$physical" "$virtual" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv HOME "$virtual/.local/combined33-home" \
 --setenv XDG_RUNTIME_DIR "$virtual/.local/combined33-runtime" \
 --setenv XDG_CONFIG_HOME "$virtual/.local/combined33-home/.config" \
 --setenv XDG_DATA_HOME "$virtual/.local/combined33-home/.local/share" \
 --chdir "$virtual" bash -s <<'INNER'
set -euo pipefail
[[ $(git rev-parse HEAD) == e1d01367692f7b8c866be369417e68c836c5d563 ]]
[[ -z $(git status --porcelain --untracked-files=no) ]]
[[ -d .git && ! -L .git ]]
mkdir -p .local/combined33-home/.opencpn .local/combined33-runtime
chmod 700 .local/combined33-runtime
source tools/local-env.sh
python3 - <<'PYTHON' > evidence/local/combined-e1d0136/build-prefix.sh
from pathlib import Path
s=Path('tools/build-integration-linux.sh').read_text()
prefix=s[:s.index('xvfb-run -a build/xnav-linux/chart_name_text_test')]
old='root="$(cd "$(dirname "$0")/.." && pwd)"'
assert prefix.count(old)==1
prefix=prefix.replace('cmake --build build/xnav-linux --parallel', 'cmake --build build/xnav-linux --target opencpn --parallel')
print(prefix.replace(old,'root="$PWD"'))
PYTHON
source evidence/local/combined-e1d0136/build-prefix.sh
INNER
