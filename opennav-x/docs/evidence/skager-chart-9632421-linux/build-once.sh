#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
root=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
expected=${1:?Pass root-authorized final source SHA}
[[ $expected =~ ^[0-9a-f]{40}$ ]]
[[ $(git -C "$root" rev-parse HEAD) == "$expected" ]]
[[ -z $(git -C "$root" status --porcelain --untracked-files=no) ]]
[[ ! -e "$prep/output/build.exit" && ! -e "$prep/output/build.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/build.exit"' EXIT
exec 1>"$prep/output/build.log" 2>&1
exec_env=(bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp --bind "$root" "$root" --bind "$prep" "$prep" --setenv GIT_OPTIONAL_LOCKS 0 --setenv HOME "$root/.local/combined-test-home" --setenv XDG_RUNTIME_DIR "$root/.local/combined-test-runtime" --setenv XDG_CONFIG_HOME "$root/.local/combined-test-home/.config" --setenv XDG_DATA_HOME "$root/.local/combined-test-home/.local/share" --chdir "$root")
"${exec_env[@]}" bash -s <<'INNER'
set -euo pipefail
mkdir -p .local/combined-test-home/.opencpn .local/combined-test-runtime
chmod 700 .local/combined-test-runtime
source tools/local-env.sh
# Use the actual current build/install recipe once. The five unchanged geometry
# fixtures are deliberately not rerun; the existing 147-case suite runs once.
python3 - <<'PY' > .local/combined-build-prefix.sh
from pathlib import Path
s=Path('tools/build-integration-linux.sh').read_text()
a=s.index('xvfb-run -a build/xnav-linux/chart_name_text_test')
prefix=s[:a]
old='root="$(cd "$(dirname "$0")/.." && pwd)"'
assert prefix.count(old)==1
print(prefix.replace(old,'root="$PWD"'))
PY
# Preserve the actual recipe with only its root resolved to the current worktree.
source .local/combined-build-prefix.sh
xvfb-run -a build/xnav-linux/skager_wordmark_test evidence/local/combined-wordmark.png 2>&1 | tee evidence/local/combined-wordmark.log
xvfb-run -a build/xnav-linux/ui_font_resolution_test 2>&1 | tee evidence/local/combined-font-resolution.log
dbus-run-session -- ctest --test-dir build/xnav-linux/test --output-on-failure --no-tests=error -E '^tests$' --timeout 90 --output-junit "$PWD/evidence/local/combined-linux-tests.xml" 2>&1 | tee evidence/local/combined-linux-tests.log
INNER
