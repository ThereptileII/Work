# Owned anchor presentation

OpenCPN remains the watch/alarm owner. The existing `AfterAnchorWatch` hook runs
after normal `MyFrame::ProcessAnchorWatch`; no UI read invokes that processing.
`ObserveAnchor` validates waypoint registration on the application thread and
copies identity, coordinate, signed radius, upstream alarm and selected GPS
observation. Consumers hold no route/waypoint pointers.

Pinned source inspected: `gui/src/ocpn_frame.cpp::ProcessAnchorWatch` and
`AnchorDistFix`, and `model/src/georef.cpp::DistanceBearingMercator` at
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. Positive watch radius alarms outside;
negative radius alarms inside. The existing alarm is copied, never recalculated
by the presentation layer.

`ProjectAnchorPosition` calls the same pinned distance/bearing routine with
vessel as destination and anchor as origin. It copies metres east/north for the
north-up plot together with coordinate, GPS source and observation time. Pole,
invalid, non-finite and overflow results stay unavailable. The distance displayed
to the user remains the normal upstream watch distance, not the plot's norm.

`PresentAnchor` only exposes current distance/position when both selected GPS
components are measured and current, coherent in source/time, and match the
copied position, source and distance timestamp. UI observation cannot refresh
their ages. Changed, missing or stale GPS withholds distance/current position.
The upstream alarm remains visible. A genuine observed zero distance is valid;
an absent watch is not a zero-distance watch.

History is bounded to 300 observed positions, strictly increasing in time. A
changed/deleted/moved anchor clears its previous trail. Retained snapshots remain
safe after later edits or deletion. History duration is the elapsed time between
actual first/last recorded points, not a sample count or assumed rate. Historic
positions are not labelled as current navigation input.

Five integrated automated regressions cover current/no watch, stale/future GPS,
source/coordinate/time changes, invalid/negative/infinite data, signed upstream
alarm semantics, bounded history, repeated and out-of-order reads, anchor move,
deletion and owned lifetime, plus projection compared directly with the pinned
OpenCPN routine for cardinal positions and an antimeridian crossing. The separate
widget executable cannot access charts, profiles or equipment and is never
installed. It checks presentation, range input and confirmation cancellation.
