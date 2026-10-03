#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
expected=${1:?exact commit}; exe=${2:?exact ELF SHA256}; manifest=${3:?exact manifest SHA256}
[[ $expected =~ ^[0-9a-f]{40}$ && $exe =~ ^[0-9a-f]{64}$ && $manifest =~ ^[0-9a-f]{64}$ ]]
[[ $(cat "$prep/output/build.exit") == 0 ]]
export SKAGER_CAPTURE_PYTHON=/home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3
root=/home/standard/Projects/X-nav-worktrees/skager-product-fidelity
for scene in coast pier57; do
  for renderer in software opengl; do
    display=:237
    [[ $renderer == opengl ]] && display=:238
    if [[ $scene == pier57 ]]; then display=:239; [[ $renderer == opengl ]] && display=:240; fi
    phase="${scene}-${expected:0:7}-${renderer}"
    "$prep/$scene/run-capture-isolated.sh" "$phase" --display "$display" --renderer "$renderer" \
      --expected-commit "$expected" --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest" \
      --baseline-report "$root/docs/evidence/skager-chart-1356fd1-linux/$renderer/report.json" \
      > "$prep/$scene/output/$phase.log" 2>&1
    baseline="$root/docs/evidence/skager-product-fidelity-78eccb8-linux/$renderer"
    if [[ $scene == pier57 ]]; then baseline=/home/standard/Projects/X-nav-worktrees/scrum264-real-enc-scenes/docs/evidence/scrum264-public-enc-scenes/$renderer; fi
    "$SKAGER_CAPTURE_PYTHON" "$prep/$scene/compare-captures.py" "$prep/$scene/output/capture-$phase" --baseline "$baseline" \
      > "$prep/$scene/output/$phase-comparison.log" 2>&1
  done
done
