# AIS drawer scrolling regression (SCRUM-302)

This small, offline native component executable links the production AIS drawer,
scroll controls and model helpers. It never starts OpenCPN, connects to a service,
reads saved credentials or operates equipment. It creates no screenshot evidence.

The test covers child-window wheel routing, fractional wheel input, both bounds,
native pan events, pointer-compatible touch dragging, canceling pressed buttons
and list rows, correct row identity after nested-list scroll, position retention
on live refresh and theme rebuild, top reset on page changes/reopen, resize,
separate modal input and untouched outside-owner events. One OS pointer injection
smoke verifies native input delivery as well as the dispatched regressions.

Run on Linux in an isolated X11 desktop (the explicit backend prevents an
inherited Wayland setting from bypassing Xvfb):

```sh
source /home/standard/Projects/X-nav/tools/local-env.sh
cmake -S tests/ais_drawer_scroll -B build/scrum302-ais-scroll -G Ninja \
  -DwxWidgets_CONFIG_EXECUTABLE=/home/standard/Projects/X-nav/tools/wx-config-local \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/scrum302-ais-scroll -j2
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 xvfb-run -a \
  -s '-screen 0 1280x800x24' \
  ctest --test-dir build/scrum302-ais-scroll --output-on-failure
```

On the interactive native Windows test desktop, configure the same directory
with the qualified Win32 wxWidgets toolchain, build `ais_drawer_scroll_test`,
and run CTest with `-C Release --output-on-failure`. Do not run its OS input
injection concurrently with another UI harness. Real chart wheel/drag isolation,
physical touch behavior and release acceptance remain integration/native gates;
an event counter on the fixture owner is not evidence of real chart behavior.

Position policy: navigating to a page or reopening the panel starts at the top.
Live data refresh and rebuilding the same body for a theme or settings action
preserve its scroll offset (clamped to the current content). The vessel list
retains its own bounded viewport and identity-based click behavior.
