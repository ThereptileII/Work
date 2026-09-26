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

Remaining review: Windows typography, modeless outside/escape behavior and
100/125/150% layout, then supported-binary real boat-display captures. Do not
close these gates using Linux screenshots.
