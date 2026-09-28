# Native prototype shell migration — not accepted

Replacement review, remote `e51a7df` / local `37b36dc`: downloaded native
navigation Day and XNav/Standard public ENC Night images show the floating
controls painted above the chart. Exact geometry and actual pointer zoom gates
pass. Day/Dusk/Night/Day snapshots agree with the requested state and all main
processes exit normally. The earlier absent-control and capture-timing defects
are closed. Font/rail hierarchy, primary sheets, stock label density and weak
Night hazard contrast remain visible differences; no screen gains acceptance.
See [verified Windows evidence](../../evidence/prototype-native-e51a7df.json).

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

The old geometry test required ≥60% chart area and all permanent controls below/
outside the chart. The HTML explicitly defines a 1014×566 canvas (56.05% of the
1280×800 view) and bounded floating map controls. The replacement asserts the
exact 68/80/186/132/34 composition and each of six bounded overlay controls;
it rejects 18 small, unsettled, clipped, overlapping or obsolete arrangements.
Only the explicit six controls may cover the chart. This preserves
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

### Native Windows pass at 8357239 / local a6a9259

The fixture-free MSVC build and all 102 integrated Windows unit regressions
passed. Eight actual 1280x800 client captures were downloaded and artifact-hash
verified. They are not accepted: floating chart actions were occluded after
upstream raised its canvas; the Up/Down page controls differ from the reference;
stock coastline colors and older page components remain visibly different.
The capture job failed during secondary `--remote --quit` process execution
with access violation. No clean-exit or visual PASS is assigned to that run.

Follow-up keeps overlay controls alive through shutdown's ShowNavigation call
(MSW child destruction is immediate) and repairs z-order only when the actual
canvas is above them. Windows capture uses the existing normal WM_CLOSE path,
asserts clean application exit and records the executable's own build commit.
This does not claim the secondary command-line quit failure is fixed. Another
native run is required. The first Linux captures had build-commit metadata from
the checkout rather than the executable; binary hashes and diagnostic build
commits remain available and the collector now records both explicitly.

### Owned floating surfaces — corrective Linux passes

Child controls remain vulnerable to upstream software/GL canvas raising. They
now live in three owned borderless native tool surfaces, outside the persisted
AUI perspective. They are not global always-on-top windows. GTK restacks each
directly above its owner without activation; this also supports the isolated
Xvfb display with no window manager. Windows uses native owner ordering.

The first capture exposed an unrelated selector bug: xdotool's default OR
combination selected the first floating frame by PID instead of the named main
frame. Searches with both PID and name now explicitly require both. The next
capture checks actual surface and antialiased glyph pixels; `visible=true`
alone cannot pass. Real zoom clicks must change OpenCPN's viewport scale.

The native/GTK light-mode collectors now wait for observed mode and subsequent
application ticks. Upstream's theme switch resets the AUI border metric, so XNav
restores its transient zero-border presentation after every switch and restores
the original metric when the shell is destroyed. Corrective software and Mesa
llvmpipe captures retain the controls through the theme cycle. Windows, physical
GPU and boat replacement evidence remain mandatory; corners/shadows and the
larger screen set are still not visually accepted.


### Rail audit against the final HTML cascade

At 1280×800 the four prototype metrics occupy x=1113, width=149 and
135.75px each, starting at y=110. Labels start 10px into each row; values
start 31px in, use 48px regular tabular text with −3px tracking and 50.88px
line height. The pilot summary is x=1113, y=666, 149×87, radius10, padding
11×12, with a 13px preceding gap and 13px bottom inset.

The current native capture still has 142px rows, vertically centered content,
no source/age baseline and a plain 64px pilot button. These are open geometric
and information-hierarchy defects. Correcting them must preserve the user's
four selected quantities and actual measurement datum. A depth sample cannot
be relabeled "below surface" without an established offset. A pilot summary
must show unavailable/stale when feedback is absent; opening it must never
send a command. Any sparkline must use retained real observations, not the
prototype's illustrative curve. This audit is not a visual PASS.


Rail corrective implementation: `XNavDataRail` distributes native rows with
cumulative rounding instead of truncating every row. The first capture exposed
that cumulative displacement; the second verifies all four actual content
rectangles and the 149×87 pilot card against canonical HTML (at most one native
pixel rounding). Labels use the measured top inset, numbers use −3px tracking,
and age/source state occupy the lower baseline. The pilot card uses fresh
confirmed adapter state; missing/stale feedback cannot become Standby. It only
opens the existing guarded panel. Depth explicitly remains transducer-relative.

`evidence/local/prototype/rail-pass2` retains twelve fixture-free captures and
settings flows. Day (first pass) and Night (corrective pass) were inspected;
all PNG hashes verify. All 120 integrated tests passed before the rounding-only
correction. Populated-data/interaction replacement, native Windows typography,
physical boat, sparkline/relational detail, header and remaining sheets stay
open. This is a geometry improvement, not full navigation conformance.
