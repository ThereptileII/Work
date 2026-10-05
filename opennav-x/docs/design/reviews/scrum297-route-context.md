# SCRUM-297: route context

OpenCPN retains route-segment hit testing and rollover lifetime. XNav receives
only the selected route identity and shows a compact native card with current
route name/state, endpoints and point count. View on chart and Details resolve
the identity again before invoking existing actions. Full route editing remains
in its existing modern detail workflow. Deleted/unavailable routes disable actions.

Hover does not replace an open task or steal focus. Escape, outside press and
touch Close dismiss; the same route under an unchanged pointer cannot reopen
immediately. Explicit selection bypasses this hover guard. Legacy retains its
rollover; track/AIS rollover behavior is unchanged.

Focused presentation/hover-guard and native card tests pass for the three themes,
invalid/deleted route, exact identity dispatch and dismissal. Shell, integration
bridge and patched software/GL canvas compile. Native Windows and boat
interaction acceptance remain pending.
