# Next slice: read-only remaining route distance

Source inspection only; no route-distance bridge is implemented or accepted yet.
References below apply to pinned OpenCPN 5.12.4 (`37fd0cd`).

- `gui/src/concanv.cpp`, route-total branch: the existing navigation console
  starts with current range to the active waypoint, finds that waypoint in the
  active route, then adds each subsequent point's `m_seg_len`.
- `model/src/route.cpp::UpdateSegmentDistance` uses
  `DistanceBearingMercator` and stores the leg distance on its **destination**
  point. `UpdateSegmentDistances` also recomputes `m_route_length`, which is
  the whole planned route, not the current remaining distance.
- `model/include/model/routeman.h` exposes active route/point and current range
  getters, plus data validity and route-update notifications. These are
  integration-layer concerns; the energy module must not own OpenCPN objects.
- `gui/src/routeman_gui.cpp::UpdateProgress` calculates current waypoint range
  with `DistGreatCircle` (unlike the later planned legs), advances route progress
  and sends configured autopilot output. `gui/src/ocpn_frame.cpp` calls it during normal
  navigation processing. A bridge must read coherent progress after this work,
  not invoke it as a convenient getter.
- `model/src/routeman.cpp::UpdateAutopilot` publishes `json_leg_info`, but also
  invokes output drivers. **Never call this method to obtain route data.**
  Subscribing to an existing notification does not imply permission to trigger
  the producer or to freshen a retained range without a new position sample.

Before implementation, settle the update ordering and provenance contract.
Copy route identifiers and finite distances into a read-only snapshot on the
application thread; do not retain raw route/waypoint pointers in Vessel Data or
SmartNav. Retain the position observation time and route identity/revision.
Treat absent/deactivated routes, missing or ambiguous active points, invalid
legs, stale position, deleted/reversed routes and out-of-order observations
explicitly. Missing route distance must never become an arrival at zero miles.

Required checks include first/middle/last active waypoint, skip/arrival,
deactivation/deletion, reverse and edit, repeated waypoint identity, invalid or
overflowing leg lengths, missing/stale position and an antimeridian fixture
compared against upstream geometry. Any design which recomputes distance must
preserve upstream navigation semantics and units rather than silently selecting
a different geodesic model. Then run both platform gates and review native
Windows interaction before accepting the slice.
