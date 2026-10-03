#!/usr/bin/env bash
set -euo pipefail
prep=$(cd -- "$(dirname -- "$0")" && pwd)
cache=/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity
: "${SKAGER_CAPTURE_PYTHON:?Set the verified Pillow-capable Python path before authorized capture}"
# Only this preparation/output folder and private /tmp are writable. All cached
# source, object files, staged install, source metadata and manifests are read-only.
exec bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp \
 --bind "$prep" "$prep" \
 --ro-bind "$cache/app" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression \
 --ro-bind "$cache/build" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux \
 --ro-bind "$cache/upstream" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source \
 --ro-bind "$cache/install" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/build/xnav-install \
 --setenv SKAGER_PRIVATE_CACHE "$cache" --setenv GIT_OPTIONAL_LOCKS 0 \
 --setenv SKAGER_CAPTURE_SCRATCH "$prep/output" \
 --chdir "$prep/output" "$SKAGER_CAPTURE_PYTHON" "$prep/capture-final.py" "$@"
