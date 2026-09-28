# Instruments — prototype migration in progress

The immutable HTML `instrumentsView()` is the authority. Its Windows full view
is (80,68,1014,566), with the horizon still visible. The content inset is 32px;
the dashboard starts at client y146. Columns are 1.1:1 with an 18px gap. The
wind card is 488×540 at (112,214); the eight main tiles are two columns of
approximately 216×126 with 12px gaps. The original SVG uses a 280×280 viewBox
at 270×270, circles r120/r94, 36 ticks and its original arrow/hull paths.
The renderer now measures the individual instrument selectors as well.

The previous Beta page hid the horizon and used wide family cards. The new
native component uses the prototype composition, shared tokens/buttons and
assessed, copied readings. No chart, route or marine-source ownership changes.
Inspection covered `N2kInstruments` PGN130306 reference handling, selected
navigation input and `VesselState` units/freshness. No new decoder or upstream
hook is necessary. Other configured instruments remain below the main eight;
the existing selection editor remains accessible. No settings are migrated or
silently discarded.

## Required navigation-meaning differences

The mock SVG fixes north and its hull at the top while displaying heading041°
and rotates a relative wind angle against that fixed north. That combination
cannot be used as a real compass. The native north-up rose rotates the hull by
valid **true heading**, and rotates the wind arrow by heading + signed relative
true-wind angle. The graphic requires both observations to be usable and within
two seconds of each other. Without heading it shows no hull/arrow orientation;
the separately valid relative-angle number may remain. COG never replaces
heading. No derived direction is published back into Vessel Data or SmartNav.

True wind retains its estimated quality. Missing, uncertain, stale or future
values are withheld; age is recomputed without renewing observations. Depth is
labelled **Depth / transducer** because no sounder installation offset exists
in this contract. Source/age lines retain quality where the mock's fixed live
values have none. These are safety/data requirements, not visual discretion.

## Gates

Implementation and validation are in progress. The first compile identified an
incorrect generic-DC graphics-context overload; the call is corrected to the
wxWidgets generic-DC factory. No screen, Windows typography or boat rendering
is accepted by this record. Captures, corrective comparison, full native tests
and boat gates must follow.

The existing grouped-region regression assumed every instrument was a 168px
family row and that the first row was fully visible. That assumption conflicts
with the supplied 540px wind card (which scrolls beneath the horizon) and 126px
tiles. The regression retains its original branch for other product pages and
adds Instruments checks for actual tile visibility, 126px tile height and
reachable readings after scrolling at increased DPI. The dedicated prototype
capture additionally checks exact wind/card alignment and all three themes.
No test or test case is removed.

First Linux visual review caught a graphics-context transform leaking into the
DC text layer: compass labels were displaced outside the rose even though the
geometry assertions passed. The shared DC state is now restored before labels
are drawn; the capture checks actual heading ink inside its expected region.
The wind-card border and the prototype's -1.2px heading tracking were corrected
in the same review. Negative images are retained. The offline test's initial
frame also needed an explicit host panel to avoid wxFrame expanding its only
child. These are evidence-driven corrections, not visual acceptance.

After the final corrections, the integrated Linux suite passes 122/122 in
19.27 seconds (26 new provenance assertions). The non-installed native widget
passes 36 checks and five actual-screen captures. Two corrective product runs
retain 16 images each, including three Instruments themes, Close back to the
1014×566 chart/horizon and the existing AIS settings Back/Close flow. All 42
recorded images verify. The unavailable-direction annotation is now a single
bounded line, clear of the wind stats. See [local evidence](../../evidence/prototype-instruments-local.json).

Two earlier Linux product runs missed a pointer transition (AIS Back, Passage
Close). The harness now leaves the pointer on its target until native dispatch
settles, then moves it aside for the screenshot. No retry click or command
substitute is sent. Later runs validate the actual resulting page and geometry;
the earlier negatives remain recorded and their cause is not claimed proven.

Remaining: native Windows font/geometry comparison, physical boat capture,
DPI/installed lifecycle gates, and removal of older top-bar Up/Down chrome in
favor of the prototype's scrolling flow. No screen-level PASS is recorded.
