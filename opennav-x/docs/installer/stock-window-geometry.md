# Fixed stock window recovery

The actual boat stock window exposed an offscreen-restore defect: its maximized
frame fit the monitor, but `SW_RESTORE` exposed a partially offscreen saved normal
rectangle. The original helper required complete monitor containment at that
intermediate point and refused before its fixed resize. A later ordinary capture
also correctly refused while a Windows notification covered the chart.

`review-stock.ps1 -Action Resize1280x800` now handles this one recovery case. The
same exact stock launch, executable, PID/start time, profile and commissioning
proof remains mandatory. The operation validates the visible, enabled top-level
OpenCPN frame, bounded dimensions/DPI and selected monitor. It uses ordinary
foreground activation, restores a maximized frame, rechecks identity and stable
geometry, then performs one fixed placement at the original monitor work-area
origin. Only a bounded invisible DWM border is added to the requested outer size.
The result must be a nonmaximized, fully contained **1280×800 physical-pixel**
visible frame. It never changes resolution, scaling, taskbar or system policy.

Complete containment is relaxed only inside that fixed resize operation.
Capture, normal foreground and close retain their existing strict frame checks.
Screenshot publication still refuses any overlapping visible window. A changed
identity, disabled/minimized frame, insufficient or changed work area, unexpected
border or incorrect final geometry refuses without a resize retry.

Success returns before/restored/after bounds, DPI/maximized state and the pinned
monitor/work area. A private `resize.json` preserves that numeric evidence before
the separate screenshot, so a later notification can refuse capture without
losing the completed geometry observation. `captureVerified` is explicitly false
in this intermediate record. Refusal messages retain bounded numeric geometry
and the failing stage; no foreign window titles or navigation data are added.

The disposable native `test-stock-resize-windows.ps1` uses actual owned marker
windows to cover partially offscreen and maximized-to-offscreen restoration,
successful strict capture afterward, changed title during restoration, wrong
PID/title and disabled-window refusal. It also calls the same pre-mutation work
area guard with insufficient dimensions without altering the desktop. These
fixtures do not launch OpenCPN or establish boat/chart acceptance. Native
qualification and a fresh actual boat resize/capture remain required.
