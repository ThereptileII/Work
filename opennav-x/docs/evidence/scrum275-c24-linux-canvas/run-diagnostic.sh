#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/ca-light-e6ef384/diagnostic
prep="$physical/$relative"
[[ $(cat "$physical/evidence/local/ca-light-e6ef384/output/lifetime-build.exit") == 0 ]]
[[ ! -e "$prep/output/diagnostic.exit" && ! -e "$prep/output/diagnostic.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/diagnostic.exit"' EXIT
exec >"$prep/output/diagnostic.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --ro-bind "$physical" "$virtual" \
 --bind "$prep" "$virtual/$relative" --ro-bind "$prep/fixture" "$virtual/$relative/fixture" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv SKAGER_PRIVATE_CACHE "$virtual/$relative/inputs" \
 --setenv SKAGER_CAPTURE_SCRATCH "$virtual/$relative/output" \
 --chdir "$virtual/$relative" bash -s <<'INNER'
set -euo pipefail
exe=$(sha256sum inputs/install/bin/opencpn | cut -d' ' -f1)
manifest=$(sha256sum inputs/install/share/opencpn/opennav/chart-style/v1/manifest.json | cut -d' ' -f1)
[[ "$manifest" == ffa75c9d097a143e58aa13f9300b04bebac0836ab849bd4284401df1866f4b2a ]]
python3 compile-layout.py
for renderer in opengl; do
 display=:275
 [[ $renderer == opengl ]] && display=:276
 /home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 diagnostic.py "ca-light-c24c154-diagnostic" --display "$display" --renderer "$renderer" \
 --expected-commit c24c15482293ad2e2f3ce4e1c8af2ca5669381de --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest"
done
INNER
