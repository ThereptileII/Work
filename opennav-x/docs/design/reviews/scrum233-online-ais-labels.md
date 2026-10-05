# SCRUM-233: available online AIS name labels

Source base `cf07197`; SCRUM-15 child selected in Jira comment 10545. This is
supplemental online chart presentation only. Onboard AIS, ENC symbols, routes,
ownship and navigation state are unchanged.

The final immutable HTML `.ais-ship text` uses 9px text, `#835d70`, with its
alphabetic baseline at `(14,-4)` relative to the target. No later theme selector
changes that fill. Night's ancestor `.chart-canvas` has `brightness(.78)`, giving
effective label ink `#664957`; Day and Dusk remain `#835d70`. The new centralized
`OnlineChartPalette.label` applies this only to labels, not to real chart pixels.
Production `UiFont` supplies the existing face selection and native-DPI scaling.

`ChartTarget` now owns the available static name and includes it in equality,
so a name-only update repaints without renewing the position timestamp. Empty,
whitespace-only, oversized or ASCII-control-containing names are omitted without
discarding the target. The painter also omits invalid UTF-8. Metadata is copied;
no provider pointers, synthesized names or independent navigation data are kept.
Labels accompany only live/aging marks. Stale/lost transitions clear the label
at both snapshot and paint-time aging boundaries; expiry removes the whole mark
as before. Existing onboard suppression, doubtful/duplicate refusal, provenance,
age marks, orientation and target hit testing are unchanged.

The shared painter measures the real font and converts the SVG baseline to
native text coordinates. A complete label must fit the viewport. Other target
symbols and previously accepted labels block collisions; selected identities
are considered first, then MMSI provides stable ordering. At most 128 names draw
from the already bounded 2000-target snapshot. Labels are omitted rather than
shifted or shortened. All target symbols and their existing selection rings
paint afterward, above their own labels; this keeps selection/provenance marks
visible where the prototype's fixed anchor intersects the added selection ring.
Name text does not enlarge the existing hit area. ENC text decluttering is not
available through this overlay boundary, so dense real charts still require
native review.

Pinned `gui/src/ocpndc.cpp:1834–1870` clamps measured text width and height to
500 pixels, while `DrawText` paints the complete string. The follow-up painter
therefore omits either extent at or above 500 before bounds/collision checks.
This deliberately also omits a genuinely exact-500-pixel name: the renderer
does not expose enough information to distinguish it from a longer name. No
text truncation, name limit, symbol or hit-test semantics change.

Only verified active XNav chart presentation enables the label pass, using the
existing `ChartBackground` availability gate. Standard and missing/changed
presentation-resource fallback retain their existing supplemental AIS symbols
without the new labels. The existing outer XNav guard keeps Legacy/Safe behavior.
No illustrative course vector is added.

## Focused evidence

- 58 existing/extended chart-target, owned-name lifetime, rename, age, omission,
  label bounds/collision and effective-theme checks passed in the initial
  increment; their inputs are unchanged and they were not rerun for the clamp fix.
- 26 real wxMemoryDC shared-painter checks pass at 96 DPI: selected priority,
  live/aging labels, missing/stale/invalid/clipped/colliding omission, exact
  baseline/font/ink and drawing-state restoration. One inspected 960x320 PNG
  shows Day/Dusk/Night. The circles are fixture anchors, not an AIS glyph test.
  Three added checks use an adapter reproducing the pinned 500-pixel metric
  clamp while forwarding complete text to wxMemoryDC. A valid 128-byte name
  measures 1024 pixels, reports 500, would falsely fit at x=460 in a 960-pixel
  viewport, and now produces zero draw calls. The canonical PNG hash is unchanged.
- The complete final `OnlineAisOverlay.cpp` compiles against actual prepared
  pinned OpenCPN headers with the retained production macros and include paths.
  No macro suppression, source shim or full application rebuild was used.

The small retained [review record](../../evidence/scrum233-ais-labels/review.json)
binds source, executable and capture hashes/bytes, counts, command and runtime.
The [production object record](../../evidence/scrum233-ais-labels/production-object.json)
contains the exact compile command, 41 non-system dependency hashes and object
identity. Development binaries remain in ignored local evidence. The pure
checks use the existing `online_ais_chart_tests` target. The standalone painter
fixture is manually compiled in this increment; it does not require OpenCPN.

The fixture uses Linux font fallback and wxMemoryDC only. It does not qualify
GL text, Windows fonts/DPI, real ENC collision density or physical boat use.
Those remain native/boat gates, including a visible named/unnamed/aged target
comparison and verified Standard/resource-fallback behavior. No full suite,
Windows CI, publication, credentials, network or boat action was performed.
