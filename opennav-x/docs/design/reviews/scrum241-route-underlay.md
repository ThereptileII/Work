# SCRUM-241 — bounded active-route understroke

Base: `1a02ae0`. Pinned OpenCPN:
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

The immutable `.chart-route-under` specifies `--floating`, width 6 and opacity
.6, with SVG's default miter joins, miter limit 4 and butt caps. The new owned
`ChartRouteUnderlay` collects existing projected screen legs, unions their
stroke contours with the already-linked pinned `ocpn::tess2`, and fills once.
It uses the existing Day/Dusk/Night floating palette; the whole-chart Night
brightness treatment is not newly applied to chart data by this increment.
There is no 32px route corridor, boat-to-route projection, invented navigation
state, or change to stored route geometry/progress/configuration.

## Eligibility, collection and paint order

The existing `DefaultChartRouteStyle` and verified `ChartActiveRouteInk` guards
are reused: only factory-equivalent visible active-route presentation in the
verified SKAGER chart style. Custom route/global width, explicit route style or
color, selection, editing, highlight, MOB and Standard/Legacy/Safe retain their
existing behavior. Equal-to-default intent cannot be inferred from a user's
stored values; this remains the selected presentation policy from SCRUM-237.

The software loop has a projection-only collection pass through the same
visibility and world-wrap decisions. Collection replaces waypoint drawing with
its existing first-step projection. `RenderSegment` collects only after its
existing rectangle rejection and integer Cohen-Sutherland clip decision; clipped
endpoints do not become miter vertices. Then the union is drawn once, followed
by the unchanged normal waypoint/foreground/arrow pass. Collection neither draws
waypoints nor changes their state.

GL collection uses the existing `DrawGLLines` draw boundary after its projection,
latitude/longitude rejection and rotated world-wrap decisions. Its normal
`m_pos_on_screen` updates occur only in the original normal pass. The underlay
is drawn before foreground/arrows and the existing later waypoint pass. Leg
indices and exact endpoint equality prevent joins across rejected legs or
between separated wrap copies. Explicit zero-length intermediate legs can be
skipped to join the same real vertex. Dirty bounds include the possible 12px
miter extent plus antialiasing margin.

Software uses one compound `wxGraphicsContext` fill; GL submits one triangle
batch through the existing color shader, restoring program and blend state.
No full-canvas image allocation/upload, generic renderer patch or route-state
processing is added. The separate foreground and point tasks keep their hooks.

## Atomic fallback and retained failure

The pinned float tessellator has a reproduced coincident-edge defect. Two
0.01px perpendicular legs plus their join produce area **13.6049** instead of
**9.1199** and include a point outside the intended stroke. A 3px cap/join case
also dropped valid paint during development. Boundary extraction followed by
triangulation and constrained-Delaunay options did not resolve this class of
failure. The raw 0.01px reproduction remains in the test source and
[`pinned-tess2-defect.txt`](../../evidence/scrum241-route-underlay/pinned-tess2-defect.txt).
It is deliberately an expected-failing diagnostic, not a passing assertion of
correct tessellation.

The production guard omits the **entire decorative understroke** if either
incident visible leg at a join is at most one full stroke width (6 logical px,
scaled for display density, with a 1e-6 device-pixel comparison tolerance).
This conservatively keeps nearby cap and join features apart; it is not a
claim that all possible tessellator inputs are mathematically certified.
The existing foreground and waypoints still draw. This can also omit the layer
near a viewport edge where clipping leaves a short incident leg. Geometry is
never simplified, lengthened or partly painted to conceal the limitation.
Supported 7/10/12px joins are checked against an independent finite-stroke
point oracle; tiny input, all tested densities and both painter paths verify
an empty/failed mesh produces no draw and no partial earlier-leg paint.

Other whole-layer fallbacks: more than 1,024 collected legs (including wrap
copies/zero markers), invalid/overlarge numeric input, tessellation failure,
8MiB tessellator allocation budget or the output-triangle cap. Owned vectors
are bounded by input/output caps in addition to the tessellator budget. The
existing foreground/waypoint pass proceeds in every case. The guard is local
presentation policy and never alters stored routes.

## Focused evidence and reproduction

Run with a display and a patched pinned OpenCPN source tree:

```sh
python tools/test-route-underlay.py --wx-config /path/to/wx-config \
  --opencpn-source /path/to/patched-OpenCPN --output /tmp/route-underlay-test
```

The runner builds the pinned tess2 C sources, actual owned geometry and extracted
production software/GL painter. A second executable executes the production GL
collector and software clip prefix with the actual pinned integer clip function;
its deterministic projection adapter is explicitly not a projection-engine test.
No installed charts, navigation profile, hardware or network feed is needed.
Add `--raw-tess2-defect` to reproduce the retained expected failure (exit 1).

- 308,138 geometry/painter/workload checks; 1,426 production collection checks.
- Width/alpha read independently from immutable HTML; butt endpoints, miter
  limit, repeated/missing/clipped/wrapped vertices, double legs/self-crossings,
  union area and triangle nonoverlap, short-leg fallback, budget rejection.
- Every +/-1–178 degree long turn at three rotations checked against an
  independent area oracle; finite short joins checked against a point oracle.
- Real wx raster alpha is identical at doubled/crossing and single-leg interiors
  for all three themes. Recording GL verifies one union batch and state restore;
  it does not qualify actual driver output.
- Existing default/custom/MOB foreground guard fixture: 78 checks passed.
- Actual Linux `ChartRouteUnderlay.cpp`, `ChartRouteUnderlayGeometry.cpp` and
  patched `route_gui.cpp` objects compiled using the existing main-target
  GL/GLSL flags and `-Werror`. All nine patches apply to pinned source in an
  isolated index; prototype originals unchanged.

The retained [900×570 diagnostic fixture](../../evidence/scrum241-route-underlay/route-underlay-turns-crossings.png)
shows sharp turns, duplicated/crossing legs and foreground painted above the
understroke in three theme rows. Its last-painted circle is an explicit point
stand-in to show order, not evidence for production waypoint fidelity.

The [focused log](../../evidence/scrum241-route-underlay/focused-test.txt) records
geometry-only timings on this Linux development host: about 0.015ms for four
wrapped legs, 0.3ms for a 128-leg serpentine and 4.8ms for 1,024 legs. A development
probe at 4,096 legs took about 38ms, motivating the smaller production cap.
These exclude projection, collection, painter/driver and frame overhead and do
not establish acceptable boat pan performance or a worst-case time bound.

Actual integrated chart screenshots, real GL, native Windows/MSVC/DPI, complete
1280×800 and 1920×1080 scenes, route lifecycle and boat pan/visibility acceptance
remain pending under SCRUM-241/SCRUM-15. This is bounded implementation evidence,
not unrestricted route conformance or release acceptance. No full CI, frozen
running source/cache, release candidate or boat was used or changed.

## Exact local compilation metadata

The original `build/xnav-linux/compile_commands.json` contains only IXWebSocket
entries. Read the actual main-target commands without modifying its cache:

```sh
.local/sysroot/usr/bin/ninja -C build/xnav-linux -t commands \
  CMakeFiles/opencpn.dir/home/standard/Projects/X-nav/src/integration/ChartPresentation.cpp.o
.local/sysroot/usr/bin/ninja -C build/xnav-linux -t commands \
  CMakeFiles/opencpn.dir/gui/src/route_gui.cpp.o
```

Use the final compiler line; substitute integration sources with this worktree
and `build/integration-source` with `/tmp/scrum241-full-source`, a separate pinned
checkout with this worktree's nine patches. For the two new helpers use the
ChartPresentation command, replacing only its final source filename. Redirect
`-o`, `-MF`, `-MT` to `/tmp/scrum241-underlay-test/{underlay,geometry,route}`.
The compiler scripts/expanded commands remain locally as
`/tmp/scrum241-compile.py` and `/tmp/scrum241-underlay-test/*.command`.
`LD_LIBRARY_PATH=/home/standard/Projects/X-nav/.local/sysroot/usr/lib` and the
wx wrapper select that same sysroot's wxWidgets 3.2.
