#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum264-combined-linux-33d90c3
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/combined-e1d0136/iho
prep="$physical/$relative"
[[ $(cat "$physical/evidence/local/combined-e1d0136/output/build.exit") == 0 ]]
[[ ! -e "$prep/output/opengl.exit" && ! -e "$prep/output/opengl.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/opengl.exit"' EXIT
exec >"$prep/output/opengl.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / \
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
for renderer in opengl; do
 display=:259
 [[ $renderer == opengl ]] && display=:260
 /home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 capture-s64.py "yellow-e1d0136-$renderer" --display "$display" --renderer "$renderer" \
 --expected-commit e1d01367692f7b8c866be369417e68c836c5d563 --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest"
done
INNER
