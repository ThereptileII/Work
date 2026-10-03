#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/scrum268-5bb/yellow
prep="$physical/$relative"
renderer=$1
scene=$2
[[ "$renderer" == software || "$renderer" == opengl ]]
[[ "$scene" == s64-yellow ]]
[[ $(cat "$physical/evidence/local/scrum268-5bb/output/build.exit") == 0 ]]
[[ ! -e "$prep/output/$renderer-$scene.exit" && ! -e "$prep/output/$renderer-$scene.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/$renderer-$scene.exit"' EXIT
exec >"$prep/output/$renderer-$scene.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --ro-bind "$physical" "$virtual" \
 --bind "$prep" "$virtual/$relative" --ro-bind "$prep/fixture" "$virtual/$relative/fixture" \
 --setenv CAPTURE_RENDERER "$renderer" --setenv CAPTURE_SCENE "$scene" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv SKAGER_PRIVATE_CACHE "$virtual/$relative/inputs" \
 --setenv SKAGER_CAPTURE_SCRATCH "$virtual/$relative/output" \
 --chdir "$virtual/$relative" bash -s <<'INNER'
set -euo pipefail
exe=$(sha256sum inputs/install/bin/opencpn | cut -d' ' -f1)
manifest=$(sha256sum inputs/install/share/opencpn/opennav/chart-style/v1/manifest.json | cut -d' ' -f1)
[[ "$manifest" == ffa75c9d097a143e58aa13f9300b04bebac0836ab849bd4284401df1866f4b2a ]]
for renderer in "$CAPTURE_RENDERER"; do
 display=:275
 [[ $renderer == opengl ]] && display=:279
 /home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 capture-s64.py "5bb7e05-$CAPTURE_SCENE-$renderer" --scene "$CAPTURE_SCENE" --display "$display" --renderer "$renderer" \
 --expected-commit 5bb7e0584029c72ce21f9230e246dad5771dc321 --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest"
done
INNER
