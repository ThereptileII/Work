# Owned anchor presentation

OpenCPN remains the watch/alarm owner. The existing `AfterAnchorWatch` hook runs
after normal `MyFrame::ProcessAnchorWatch`; no UI read invokes that processing.
`ObserveAnchor` also refreshes the owned presentation snapshot on read, using
the same tick time as the consumer. This avoids rejecting a current selected
fix merely because it arrived after the last normal watch hook. It validates
waypoint registration on the application thread and copies identity, coordinate,
signed radius and the last upstream alarm. The selected GPS must be measured,
current and coherent, with `bGPSValid` and coordinates equal to OpenCPN's
accepted position. Consumers hold no route/waypoint pointers. Observation does
not change the selected fix's timestamp or process the alarm.

Pinned source inspected: `gui/src/ocpn_frame.cpp::ProcessAnchorWatch` and
`AnchorDistFix`, and `model/src/georef.cpp::DistanceBearingMercator` at
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. Positive watch radius alarms outside;
negative radius alarms inside. The existing alarm is copied, never recalculated
by the presentation layer.

`ProjectAnchorPosition` calls the same pinned distance/bearing routine with
vessel as destination and anchor as origin. It copies metres east/north for the
north-up plot together with coordinate, GPS source and observation time. Pole,
invalid, non-finite and overflow results stay unavailable. The distance displayed
to the user uses the same pinned routine and argument order as the normal
upstream watch, not the plot's norm. Integration copies OpenCPN's distance-unit
label and conversion factor; small distances retain three decimals in nautical
miles, statute miles or kilometres. Geometry and radius configuration stay in
metres, with the radius explicitly labelled `m`.

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

Eight integrated automated regressions cover current/no watch, stale/future GPS,
source/coordinate/time changes, invalid/negative/infinite data, signed upstream
alarm semantics, bounded history, repeated and out-of-order reads, anchor move,
deletion and owned lifetime, distance-unit conversion, a deterministic sequence
of GPS updates between normal watch hooks, plus projection compared directly
with the pinned OpenCPN routine for cardinal positions and an antimeridian
crossing. The separate
widget executable cannot access charts, profiles or equipment and is never
installed. It checks presentation, range input and confirmation cancellation.

Setting an anchor in XNav now stops active OpenCPN route navigation only after
the new anchor mark is saved. It reuses pinned `Routeman::DeactivateRoute`, which
clears active progress without deleting the route or its waypoints. Failed
anchor persistence leaves navigation unchanged. A defensive failed-deactivation
path rolls back the saved mark; if that database rollback also fails, the mark
remains visible/selectable and its identity is reported, without arming a watch.

Activating a saved route while anchored requires the XNav stop-watch
confirmation. Cancel dispatches no mutation. The callback revalidates the route
revision and the complete owned selection of both native watch slots, including
mark revisions, after the modal interaction. Changed, added, missing or
unavailable watches require a fresh decision. Route persistence is checked
before clearing watch state. The transition preserves all anchor marks,
including temporary SKAGER marks, and then uses native route activation.
Explicit manual watch clearing retains its separately confirmed temporary-mark
removal behavior. Go To is refused while a watch is active and directs the user
to stop it first; it does not silently bypass the confirmation. Replay blocks
the transition callback, including replay started while confirmation is open.

The focused transition executable covers cancellation, both watches, stale
selection, persistence ordering/failure and replay using the production helpers.
The native navigation-object scenario additionally checks actual route/watch
transitions and registered/persisted mark preservation. These assertions require
the native application gate; portable tests do not qualify Windows or boat use.
