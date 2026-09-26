# Beta 2 selected-object cards — development review

Reference intent: preserve the dominant chart, show only useful information
for the selected vessel/mark, use quiet dark surfaces and readable values, and
open full Details only when requested.

The prior selection hooks immediately opened large product pages. The new
shared modeless card keeps the chart and four primary rail values visible.
Waypoints show name, position, direct range/bearing, Go To, Details, Edit and
Remove. AIS shows name, status, SOG/COG, upstream CPA/TCPA, range/bearing, Show
on chart and Details. The chart-position variant shares the same dismissal
and action behavior. None retains native object pointers or captures the mouse.

Linux 1280×800 captures reviewed:

- `objects-waypoint-card.png`: coastline dominates; card fits within the chart;
  true bearing and direct NM range are separate from remaining route distance.
- `objects-waypoint-stale-position.png`: missing current GPS suppresses both
  calculated values and disables Go To. The critical GPS alert and its action
  remain visible; none of the four rail values moves out of view.
- `objects-ais-card.png`: target and upstream encounter values fit in the compact
  card. An initial numeric status caption was replaced with bounded native
  status text, including native beacon-class semantics.
- `objects-waypoint-details.png` and `objects-ais-details.png`: Details paths
  preserve the larger existing views. Their chart-return/selection actions work.

The isolated object interaction gate passes 17 grouped native checks and eleven
captures, including late-added real loopback GPS/AIS, deleted/ambiguous waypoint
identity, retained copies, native antimeridian geometry, stale GPS action gating
and both compact/full AIS selection. Settings rebuilds are checked separately
from a hidden product page and while Navigation is already visible. These are local development results from
the working tree, not exact-commit Windows or boat-display acceptance. Native
Windows card/DPI interaction is added to the next candidate gates.

Bare-Xvfb owned-window stacking is recorded explicitly by the object harness.
The ordinary chart-position user-flow gate additionally passes physical pointer
input without a manual raise after avoiding redundant Navigation layout.

The Linux `ee720380` CI object gate stopped at compact waypoint Details. Its
stale-position capture shows the correctly placed card and visible Details
button; the final diagnostics show Navigation and no card controls. This is
consistent with an outside-click dismissal, but the exact event sequence was
not recorded and the unmodified test passed on a local repeat. It is not
evidence that the Details callback itself failed.

The Linux object harness now settles pointer motion before the already
documented bare-Xvfb ownership adjustment, asks XQueryPointer to verify the
actual context window under the pointer, and sends one separated press/release.
Two consecutive native hit-target observations are required before pressing.
It records these observations and still requires the full Details page and
chart return; there is no callback injection or action retry. Windows input is
unchanged. The revised local gate passes all 17 object groups and eleven
captures, including both waypoint and AIS Details. Exact-commit CI confirmation
remains required; no production UI change was inferred from this one failure.

Remaining review: Windows typography, modeless outside/escape behavior and
100/125/150% layout, then supported-binary real boat-display captures. Do not
close these gates using Linux screenshots.
