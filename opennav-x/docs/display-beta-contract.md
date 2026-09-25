# Beta display and touch contract

Primary XNav pages, sheets, data rail and alerts use the shared theme tokens.
Sheets reserve the global status/alert band and navigation row; Cancel stays
visible to return to alarm/manual controls.
The same `XNavScroll` component supports mouse wheel, keyboard, vertical native
pan gestures and persistent 48-DIP-or-larger Up/Down controls. Native scrollbars
are hidden; the alternatives are present whenever content overflows. A pan on a
button cancels its pending click. No gesture generates pilot course commands.
The layout override retains the full virtual height: pinned wxWidgets otherwise
treats hidden bars as disabled scrolling and compresses child controls. Actual
card-height assertions catch clipped status labels.
[wxWidgets 3.2.8 ScrollLayout](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/generic/scrlwing.cpp).

Pilot opens directly from the navigation action bar. STBY remains at a fixed
position across pages and becomes actionable only when the current manual
controller is enabled and advertises Standby. Its request uses the same adapter
and feedback/timeout lifecycle as the expanded panel. It is not an autonomous
response to a fault and does not replace physical STANDBY access.

The software shapefile background in pinned OpenCPN otherwise retains day land
color. XNav copies the exact `GSHHSChart::SetColorScheme` land palette into
`ShapeBaseChartSet` before the existing frame color-scheme call, and after
normal deferred initialization. Day retains the original colors; dusk/night
use upstream's 0.5/0.25 multipliers. ENC/raster palettes, geometry, chart selection
and Legacy behavior remain upstream-owned. No additional source hook is added.

The Windows frame uses documented DWM per-window caption color attributes when
supported. These are documented for Windows 11 build 22000 onward; failure leaves
the OS caption intact and diagnostics report that limitation. No global theme,
registry setting, undocumented theme ordinal or wxWidgets ABI upgrade is used.
[Microsoft DWM attributes](https://learn.microsoft.com/en-us/windows/win32/api/dwmapi/ne-dwmapi-dwmwindowattribute).
Full-screen remains a deliberate Display setting. Native file pickers and
Advanced/Legacy/plugin dialogs retain their OS/upstream appearance and can be
bright: prepare imports/exports/settings before night operation.

Runtime diagnostics report palette, caption support, page scroll position and
scroll availability. Native DPI tests require actual GetDpiForWindow values at
100/125/150%, measure touch targets, interact only after scrolling controls into
view, test touch taps/pan, and capture primary workflows in night mode. Coastline
checks require both upstream land and water colors; dark primary surfaces are
checked separately. Injected touch is software evidence, not acceptance of a
physical marine touchscreen, wet/gloved use or the target GPU.

The Linux palette harness separates single clicks beyond GTK's double-click
interval and verifies each palette transition. Rapid input may be coalesced as
a double-click; tests must not assume it represents two independent activations.
This does not relax pilot command rate limits or create repeat actions.

## Scroll endpoint regression

Native qualification at 150% exposed wx panel focus delegation: disabling a
focused Down button at the endpoint and calling ordinary panel `SetFocus()`
could focus a child and scroll back to it. The replacement uses
`SetFocusIgnoringChildren()` for page and rail viewports. This preserves keyboard
focus without changing the visible position. Native 100/125/150% tests require
the last menu action fully visible after settling at the bottom, in addition to
touch pan, Up/Down, complete night workflows and mode/chart checks. Failed DPI
runs now retain the visible failure capture and control geometry.

Native `92a695f` confirms the repaired 150% Menu endpoint and all primary
night-page/touch-scroll checks. The subsequent page assertion rejected a valid
492-pixel viewport because its old fixed minimum was 500 pixels. The visible
56-DIP alarm consumes 84 pixels at 150%. The replacement asserts exact native
center boundaries below status/alerts and above navigation, near-full width,
at least half-window usable height, and the existing sibling-occlusion check.
No interaction/capture is removed. Replacement acceptance remains required.
