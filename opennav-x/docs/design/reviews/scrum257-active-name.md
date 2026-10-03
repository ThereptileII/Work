# SCRUM-257 — readable default active-waypoint name

The integrated Night capture left the actual active waypoint name stock black.
The label previously inherited marker exclusions even though upstream names
remain visible while the active icon blinks. The change gives only the actual
current active point the existing prototype card, preserving its distinct
upstream active icon and blink behavior. Base: root1443445.

The application-thread label eligibility helper allows active/blink flags only
when the point is marked active and its pointer equals OpenCPN's active point.
It shares the existing bounded route-identity traversal and every other guard:
verified default diamond and route style, no custom/shared/repeated/layer,
selected/edit/drag, MOB/anchor/range rings, and existing maximum99 ordinal/4096
visited items. Non-active blinking points and contradictory active pointers
remain stock. Standard, Legacy and Safe still fail the presentation gate.

Both upstream painters calculate the label ordinal once, then force marker
ordinal0 for any active point. The activepoint bitmap/texture substitution,
icon drawing and blink branches are unchanged. Labels already draw outside the
blink branch. GL includes label eligibility in bounds invalidation before
culling, including a newly eligible active point without a previous cache.

The existing real-name/font/color/default-offset guards, native text shaping,
25px/radius5 card and exact Day/Dusk/Night colors are reused. Night includes the
prototype chart ancestor brightness(.78): fill#101a20/text#8e9888. No new strings,
font state, navigation state, geometry, persistence or resource changes. The
raster, tight bounds, texture ownership/upload/failure cleanup remain unchanged.

Focused verification:

- 76 actual production-body marker/eligibility checks pass, including actual
 active identity, other-pointer rejection, inherited exclusions, shared route,
 custom icon/route, non-active blink and Standard/Legacy/Safe.
- 93 existing native label/font/raster/cache/fallback checks pass.
- All nine pinned patches apply; the resulting route_point_gui file matches the
 one compiled. ChartRouteWaypoint.cpp and route_point_gui.cpp compile in both
 software and GL configurations (four successful changed production objects).

The private collector is updated only for this reviewed behavior: both visible
real names now require exact card fill, while the actual active point must keep
its stock icon rather than a numbered circle. An additional bounded Day capture
requires visible and hidden phases of the red upstream active icon with the
name card present throughout. Existing hot themes, inactive numbered marker,
understroke, stale controls, clean shutdown and every original111 Python
assertion/all26 route contract checks are retained. Complete real names and
numerals still receive visual review; pixel probes do not claim OCR.

The prepared collector is versioned with focused receipts under
[docs/evidence/scrum257-active-name](../../evidence/scrum257-active-name/).
No integrated app launch, CI or boat action was performed. Actual combined
software/GL captures await the next exact staged executable. Native MSVC and
boat visual acceptance remain open; SCRUM-257 is Testing, not Done.
