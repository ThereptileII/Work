# SCRUM-237 — bounded default active-route foreground

This increment implements the immutable prototype's final 2.6 CSS-pixel
foreground and round interior joins. It does **not** complete route conformance
or SCRUM-15. The 6px / 0.6-opacity understroke and 32px / 0.045-opacity illustrative
context remain open. No corridor, maneuver, route-progress value or safety
meaning is invented.

## Source and scope

Base: `e28459b126135a2456ef6a234c65ebe18bfd2f85`. Inspected pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`: `RouteGui::Draw`, `RenderSegment`,
`DrawSegment`, `DrawGLRouteLines`, `DrawGLLines`, `RoutePointGui::Draw/DrawGL`,
`ocpnDC::StrokeLine/DrawLine/DrawPolygon/DrawBitmap`, shader helpers and manual/
AIS MOB construction. The immutable HTML's first `.chart-route` gives round
joins; its later rule overrides 2.5 to 2.6. SVG end caps default to butt.

`DefaultChartRouteStyle` permits only visible, active, unselected, unhighlighted,
unedited routes with no explicit route color/width/style and the upstream global
line-width default of 2. Every route containing the upstream `mob` point icon
falls back. The earlier ink hooks now share this eligibility gate. Custom and
special states keep upstream presentation behavior; neither route properties,
manager pens nor global preferences are written. Standard, Legacy, Safe and
unverified presentation resources retain stock paint.

Upstream still chooses projection, longitude wrapping, visibility, arrow
geometry, active-point arrow suppression and waypoint order. A small shared
screen-space triangle mesh supplies only the foreground. The software and GL
paths consume the same mesh; incremental segments get the same width with butt
ends because that API supplies no waypoint-join identity. Whole-route passes
identify interior joins without joining across separate wrapped segments.
The transient ownship-to-active leg keeps upstream paint.

The mesh has a 2.6px nominal cross-section, scales through the canvas DIP factor,
and approximates each round join by a 32-sided inscribed disk (radial error below
0.0063 logical pixels). It clips to a stroke-inflated viewport before GPU
submission and rejects nonfinite, empty or unusable geometry. The maximum normal
segment mesh is 66 triangles / 396 floats. Software fills one native graphics
path; GL submits one triangle batch to the existing solid-color chart shader.
There is no bitmap, texture upload, persistent cache, global line-width change,
or generic renderer modification. The shader retains upstream RGB/256 ink
normalization. Dirty bounds include stroke extent, and existing pen/brush state
is preserved.

## Focused evidence

`tools/test-route-foreground.py` extracts the actual production eligibility and
painter, and independently reads the final width/joins from the unchanged HTML.
On Linux the real wx software painter plus a recording GL interface passed 78
checks: explicit/global custom preferences, inactive/hidden/selected/edited/
highlighted/MOB exclusion; widths at 100/125/150/200%; diagonal and clipped
geometry; butt ends and rounded joins; zero/nonfinite inputs; theme ink; GL
submission and blend/program restoration; software pen/brush preservation.
The retained PNG shows Day/Dusk/Night rows at 100/125/150% on a neutral test
background. It is a painter fixture, not real chart or navigation evidence.

One local optimized geometry-only run generated 10,000 fully joined meshes in
18.4691ms. This is CPU geometry cost only; it does not qualify renderer frame
rate, chart panning, driver behavior or boat responsiveness.

All nine production patches applied cleanly to a fresh disposable pinned
OpenCPN checkout. The changed `ChartPresentation.cpp` and patched
`gui/src/route_gui.cpp` objects compiled with the existing Linux GL/GLSL build
flags and `-Werror`, using private source and output paths. Initial compilation
caught private `m_bVisible` access and a missing configuration declaration;
these were corrected to `IsVisible()` and the explicit configuration header.
No active candidate source/cache, release run or CI was modified or triggered.

## Remaining acceptance

- 6px translucent understroke and 32px illustrative context; avoid repeated
  per-segment alpha darkening at joins, and do not imply a safety corridor.
- Exact integrated software and actual native GL visual output, including GL
  edge antialiasing, Night ancestor brightness behavior, wrap/clip/pan, join
  redraw, dirty-region refresh, custom/special-state regression and progress.
- Full waypoint, inactive/selected route, track and emergency visual hierarchy.
- Fresh exact-revision native Windows 1280×800 and 1920×1080, DPI, boat display
  and real boat responsiveness. Recording GL calls are not driver execution.

SCRUM-237 remains an implementation increment with outstanding acceptance,
not a claim that the full default route matches the prototype.
