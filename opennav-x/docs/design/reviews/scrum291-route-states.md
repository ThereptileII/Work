# SCRUM-291 — route presentation states

The former presentation gate accepted only active, unselected default routes.
Ordinary saved routes therefore reverted to legacy paint before activation.
The same bounded native foreground/underlay now covers visible default inactive,
active and selected routes. Active uses prototype route ink, inactive uses muted
chart ink and selected uses the theme's selection accent. Each software segment
resolves its route ink after waypoint painting; OpenGL uses the same palette.

Unique, ordinary route waypoints retain their real numbered ordinal even when
no route is active. Shared/repeated, edited, emergency, custom icons and protected
waypoint states keep their established exceptions. Explicit user route colors,
line styles and widths remain respected. Geometry, clipping, wrap, direction,
storage and navigation processing are unchanged.

Focused foreground tests pass 80 checks and waypoint tests 77 checks. Changed
ChartPresentation, ChartRouteWaypoint and patched RouteGui compile against the
integrated pinned Linux headers. Existing assertions excluding inactive/selected
default routes were updated specifically for this approved scope expansion;
custom/emergency rejection assertions remain. Native/boat rendering is pending.
