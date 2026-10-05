# SCRUM-242: proven default route waypoint markers

This is a bounded correction from `1a02ae083501eb05feeef7879e6ee450c076be69`,
using pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
It does not qualify the complete chart presentation.

The immutable HTML `.map-waypoint` uses a radius of 10 CSS pixels, a centered
2-pixel route-colored outline, floating-surface fill and an 8-pixel, weight-650
ordinal at vertical offset +0.5. The production helper draws this artwork at
the existing projected waypoint position, preserving the existing mark scale.
The software path uses a fractional graphics pen; GL uses two opaque triangle
fans with radii 11 and 9, followed by the native OpenCPN text path. No GL line
width limit, icon texture replacement, or persisted font change is involved.
Night's existing chart brightness effect is applied to the new fill only; route
ink already carries the theme's effective chart color.

## Provenance and state boundary

Pinned OpenCPN deliberately loads UserIcons after stock icons, replacing an
existing `MarkIcon` in `ProcessIcon`. Plugins use the same entry point. An icon
name whitelist would erase those overrides. A new internal `MarkIcon` ownership
bit defaults false and is revoked on every `ProcessIcon` replacement. Only the
stock legacy loader may grant it, requiring the `diamond` key and the exact
493-byte `data/svg/markicons/1st-Diamond.svg` input with SHA-256
`ebb4c7e751b9d21db60d1bbf1041eb5434616b6b90570cd7ca478480263e8a9d`.

The upstream SVG disk cache uses path/size, not content, in its key. Therefore
ownership additionally requires the decoded image to equal an uncached render
of that pinned source (alpha and all visible RGB pixels). Modified or stale
cached artwork remains upstream-rendered. The paint query requires the exact
currently owned bitmap pointer; missing-key fallback cannot qualify. No profile,
icon image, user file, cache file, or registry identity is rewritten.

Only verified SKAGER chart style and an untouched, visible active route qualify.
The point must be an ordinary, non-layer diamond occurring exactly once across
all actual routes, including hidden routes. The ordinal is counted in the
active route list; no name parsing or illustrative number is used. Shared and
repeated points, ordinal >99, and libraries exceeding the 4096-entry paint scan
budget remain stock. Standalone marks, MOB and other icon roles, active
destination, selection, blinking, editing, drag handles, anchor-watch points,
and enabled range rings also remain stock. Existing names and their visibility
remain unchanged. The route policy independently excludes custom route paint,
selection, editing, highlight and MOB routes.

The hooks preserve upstream projection, visibility/scamin, stored point/route
values, hit testing, labels, range rings, and bitmap fallback. Dirty rectangles
include the new radius. Eligible GL points rebuild their geographic dirty bounds
before cached visibility rejection, including after selection or scale changes.
Standard, Legacy and Safe never enable the painter.

## Focused evidence

- 62 checks execute the production ordinal, painter, font and replacement-query
  bodies. They include exact source/changed source, actual SVG-to-PNG cache
  round-trip, changed cached pixel/alpha, same-name custom/plugin replacements,
  foreign bitmap identity, shared/repeated routes, state exclusions, >99 ordinal,
  bounded route-library traversal, software pixels and GL submission/state.
- Real pinned `route_point_gui.cpp`, `waypointman_gui.cpp`, and
  `ChartRouteWaypoint.cpp` each compile with both software and GL/GLSL flags.
  These are six actual production object builds with `-Werror`, using the frozen
  developer build configuration read-only. The final GL dirty-bounds hook also
  received fresh software and GL object builds.
- All nine reviewed patches apply to the pinned temporary index and exactly
  match the private upstream tree; patch whitespace checks pass.
- [Actual software fixture](../../evidence/scrum-242-route-waypoints/markers.png):
  rows Day/Dusk/Night, columns existing mark scale 100/125/150/200%. Inspected:
  ordinal 02 remains centered, circle/outline scale together, floating fill and
  route ink track the theme. These columns are mark scaling, **not native DPI
  acceptance**. The separate recorded GL check uses fractional 125% geometry.

The first compile exposed an incorrect wxGraphicsContext pen overload and was
corrected. Fixture setup also exposed missing PNG handler initialization and an
expired temporary display; these were harness failures, corrected before the
62-check result. No failure assertion was removed.

The GL fixture records production submissions; it is not an actual driver
capture. Windows MSVC/DPI/font/driver rendering, integrated real-route ENC
captures, mode returns and boat display acceptance remain required. Linux uses
its installed font fallback. Active-destination and special waypoint artwork
are intentionally not claimed to match the prototype by this increment.
