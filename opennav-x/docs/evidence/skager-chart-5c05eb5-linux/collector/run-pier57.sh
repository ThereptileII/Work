#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
expected=${1:?exact commit}; exe=${2:?exact ELF SHA256}; manifest=${3:?exact manifest SHA256}
[[ $expected == 5c05eb55c15b67d4014df55896d95452e9cc9d64 && $exe =~ ^[0-9a-f]{64}$ && $manifest =~ ^[0-9a-f]{64}$ ]]
[[ $(cat "$prep/output/build.exit") == 0 ]]
export SKAGER_CAPTURE_PYTHON=/home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3
root=/home/standard/Projects/X-nav-worktrees/skager-product-fidelity
for renderer in software opengl; do
  display=:239
  [[ $renderer == opengl ]] && display=:240
  phase="pier57-${expected:0:7}-${renderer}"
  baseline="$root/docs/evidence/skager-chart-9632421-linux/pier57/$renderer"
  "$prep/pier57/run-capture-isolated.sh" "$phase" --display "$display" --renderer "$renderer" \
    --expected-commit "$expected" --expected-exe-sha256 "$exe" --expected-manifest-sha256 "$manifest" \
    --baseline-report "$baseline/report.json" > "$prep/pier57/output/$phase.log" 2>&1
  "$SKAGER_CAPTURE_PYTHON" "$prep/pier57/compare-captures.py" "$prep/pier57/output/capture-$phase" --baseline "$baseline" \
    > "$prep/pier57/output/$phase-comparison.log" 2>&1
done
