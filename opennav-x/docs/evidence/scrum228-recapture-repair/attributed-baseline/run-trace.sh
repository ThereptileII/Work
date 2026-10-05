#!/bin/bash
set -eu
cache=/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity
private=/home/standard/Projects/X-nav-worktrees/scrum15-gl-framebuffer-sync/.local/stale-overlay-probe
exec bwrap --die-with-parent --unshare-user --unshare-pid --ro-bind / / --proc /proc --dev /dev --tmpfs /tmp \
 --bind "$private" "$private" \
 --ro-bind "$cache/app" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression \
 --ro-bind "$cache/build" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux \
 --ro-bind "$cache/upstream" /home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source \
 --ro-bind "$cache/install" /home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/build/xnav-install \
 --setenv PATH /home/standard/Projects/X-nav/.local/sysroot/usr/bin:/usr/bin:/bin \
 --setenv LD_LIBRARY_PATH /home/standard/Projects/X-nav/.local/sysroot/usr/lib \
 --chdir "$private" python3 "$private/smoke-navigation-stack-trace.py" --route-fixture --renderer opengl
