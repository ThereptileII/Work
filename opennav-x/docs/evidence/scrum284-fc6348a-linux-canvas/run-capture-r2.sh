#!/usr/bin/env bash
set -euo pipefail
physical=/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384
virtual=/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29
relative=evidence/local/circle-fc6348a/iho
prep="$physical/$relative"
renderer=$1
scene=$2
[[ "$renderer" == software || "$renderer" == opengl ]]
[[ "$scene" == s64-light-fog || "$scene" == s64-rocks || "$scene" == s64-wreck || "$scene" == seattle ]]
[[ $(cat "$physical/evidence/local/circle-fc6348a/output/build.exit") == 0 ]]
[[ ! -e "$prep/output/$renderer-$scene-r2.exit" && ! -e "$prep/output/$renderer-$scene-r2.log" ]]
trap 'result=$?; echo "$result" > "$prep/output/$renderer-$scene-r2.exit"' EXIT
exec >"$prep/output/$renderer-$scene-r2.log" 2>&1
bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / \
 --proc /proc --dev /dev --tmpfs /tmp --ro-bind "$physical" "$virtual" \
 --bind "$prep" "$virtual/$relative" --ro-bind "$prep/fixture" "$virtual/$relative/fixture" \
 --ro-bind "$prep/seattle-fixture" "$virtual/$relative/seattle-fixture" \
 --setenv CAPTURE_RENDERER "$renderer" --setenv CAPTURE_SCENE "$scene" \
 --setenv GIT_OPTIONAL_LOCKS 0 --setenv GDK_BACKEND x11 --unsetenv WAYLAND_DISPLAY \
 --setenv SKAGER_PRIVATE_CACHE "$virtual/$relative/inputs" \
 --setenv SKAGER_CAPTURE_SCRATCH "$virtual/$relative/output" \
 --chdir "$virtual/$relative" bash -s <<'INNER'
set -euo pipefail
exe=$(sha256sum inputs/install/bin/opencpn | cut -d' ' -f1)
manifest=$(sha256sum inputs/install/share/opencpn/opennav/chart-style/v1/manifest.json | cut -d' ' -f1)
[[ "$manifest" == cecfd92eff1b2c9a9c2aa2e64e9a16968a84bdcdee14b77eba337a9277c07185 ]]
for renderer in "$CAPTURE_RENDERER"; do
 display=:275
 [[ $renderer == opengl ]] && display=:276
 /home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 capture-s64.py "fc6348a-$CAPTURE_SCENE-$renderer-r2" --scene "$CAPTURE_SCENE" --display "$display" --renderer "$renderer" \
 --expected-commit fc6348a6df8374878802b3e051a3670d83ed9a56 --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest"
done
INNER
