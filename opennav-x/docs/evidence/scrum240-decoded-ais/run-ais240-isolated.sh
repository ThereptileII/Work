#!/usr/bin/env bash
set -euo pipefail
cache=/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity
exec bwrap --die-with-parent --unshare-user --unshare-pid --unshare-net --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp \
 --bind "$cache" "$cache" \
 --bind "$cache/app" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression \
 --bind "$cache/build" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux \
 --bind "$cache/upstream" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source \
 --bind "$cache/install" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/build/xnav-install \
 --setenv SKAGER_PRIVATE_CACHE "$cache" --setenv GIT_OPTIONAL_LOCKS 0 \
 --chdir /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux "$@"
