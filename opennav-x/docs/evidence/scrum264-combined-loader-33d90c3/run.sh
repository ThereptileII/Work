#!/bin/bash
set -eu
export PATH=/home/standard/Projects/X-nav/.local/sysroot/usr/bin:/usr/local/bin:/usr/bin:/bin
export LD_LIBRARY_PATH=/home/standard/Projects/X-nav/.local/sysroot/usr/lib
export GDK_BACKEND=x11
unset WAYLAND_DISPLAY
xvfb-run -a python3 .local/combined-loader/positive-only.py \
 --source /home/standard/Projects/X-nav-worktrees/scrum264-yellow-special/.local/yellow/patched-core \
 --private-source /home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source \
 --private-render-source /home/standard/Projects/X-nav-worktrees/scrum264-yellow-special/.local/yellow/patched-private \
 --generated /home/standard/Projects/X-nav-worktrees/audit-combined-symbol-loader/.local/combined-loader/resources \
 --output "$PWD/.local/combined-loader/result" \
 --wx-config /home/standard/Projects/X-nav/.local/sysroot/usr/bin/wx-config \
 --wx-prefix /home/standard/Projects/X-nav/.local/sysroot/usr --seamarks
