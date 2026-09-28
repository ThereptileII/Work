# Native prototype shell migration — not accepted

Reference intent: exact 68px header, 80px left navigation, 186px persistent
rail, 132px horizon and 34px footer. Existing OpenCPN canvas stays in its own
parent/AUI pane. Navigation controls float on the chart; prototype SVG icon
paths replace the old independent icon drawings.

First implementation: centralized palette and installed-font stack; primary
button fill/hover/disabled states; left navigation; bounded native chart tools;
advisory horizon from existing SmartNav events only. Pages retain the rail and
left navigation. No simulated source was added to the installed application.

Remaining differences to resolve through capture/review: header composition,
rail typography/detail hierarchy and configured values, selected navigation
state, floating-card corner/shadow rendering, compact route/AIS sheets, energy/
instrument page composition, full prototype input/return flows, chart resources
and overlays. The new horizon does not invent missing destination/CPA events.
Measured depth retains its actual datum; prototype "below surface" wording
cannot replace a below-transducer measurement without a valid offset.

The old geometry test requires ≥60% chart area and all permanent controls below/
outside the chart. The HTML explicitly defines a 1014×566 canvas (56.05% of the
1280×800 view) and bounded floating map controls. Replace that obsolete layout
expectation with exact prototype geometry and explicit overlay bounds; preserve
coastline-content, clipping, input, mode and persistence tests. Do not loosen
pixel tolerances or accept a blank chart to accommodate the new design.

Linux component compilation and prototype token checks pass. Integrated runtime
captures, native MSVC/regressions, comparison/diff and boat refinement are still
pending. This record is a work log, not a visual PASS.

Second Linux review at local `545feca`: corrected GTK character em size and
Arial/fontconfig fallback; header brand, selected sidebar item, rail heading,
energy two-card/compact-value composition. Exact native screenshots and unmasked
reference/current/diff sets are in `evidence/local/prototype/comparison-pass2`.
The first second-pass capture was contaminated by concurrent upstream REST
unit tests; it is retained as a tooling failure, not design evidence. The full
110-test gate and eight captures were repeated serially and passed execution.

Remaining visible defects include stock chart colors, chart-frame 1px border,
header controls/spacing, absent settings drawers, rail label/value/source geometry,
floating surface corners/shadows, old page interactions, and extra page-scroll
controls. Empty sensor readings are deliberate unavailable states. The passage
energy graph uses only model-supported endpoint values with an explicit steady-
consumption label; the illustrative curved forecast is not synthesized.
The next correction removes grey child-control backgrounds exposed by capture.
All visual acceptance fields remain Pending.
