# Read-only remaining active-route distance

Alpha adds owned remaining waypoint/leg steps to the same coherent publication
for [advisory timeline consumers](smartnav-alpha-contract.md). This does not
replace the accepted distance calculation or loosen its validity rules.

Status: accepted development slice at `954b4505e18e9128dc02e75cf05d0c02bdbad188`.
Both platform gates and native 1280×800 review passed in
[run 35648822128](https://github.com/ThereptileII/Work/actions/runs/35648822128).
The Developer Preview now consumes this contract for advisory presentation.
Live battery acquisition and calibrated configuration remain unavailable; see
[the preview contract](developer-preview-contract.md).

## Ownership and boundary

OpenCPN owns routes, waypoints, geometry, progress, arrival/skip behavior and
configured navigation output. OpenNav adds two guarded calls around the existing
`RoutemanGui::UpdateProgress` call in `MyFrame::OnFrameTimer1`. It never calls
progress processing or `UpdateAutopilot` to obtain data. Legacy/Safe do not create
the route observer. The unchanged normal OpenCPN processing remains authoritative.

The integration reader copies route GUID, ordered waypoint GUIDs/coordinates,
stored incoming leg lengths, active point membership, selected position metadata
and upstream current range on the application thread. It checks route registry
membership before dereferencing an active route. All OpenCPN pointers remain
inside that synchronous read. A writer in `integration/RouteProgressInput`
validates these copies and publishes `shared_ptr<const RouteProgressSnapshot>`.
Vessel Data and SmartNav contain only owned values, never upstream object handles.

A snapshot is historical evidence as of its observation. Retaining it after
route deletion is memory-safe; retention does not make it the current route.
Consumers acquire the current publication through `CurrentRouteProgress()` on
the application thread, then use `AssessRoute(snapshot, now)` or
`RouteDistanceSample(snapshot, now)` at consumption time. Acquisition checks
current route identity/geometry/active point and can invalidate a previous
publication. It cannot calculate distance or renew its observation time.

For example, a future consumer acquires and assesses the current publication on
the application thread before scheduling any work with its owned values:

```cpp
const auto snapshot = opennav::CurrentRouteProgress();
if (snapshot) {
  const auto assessment = opennav::vessel::AssessRoute(
      *snapshot, opennav::vessel::Clock::now());
  // Use assessment.remaining_distance_nm only when it has a value.
  // Preserve snapshot identity, revision and timestamps with derived results.
}
```

An asynchronous result must be discarded if its originating route revision or
active waypoint no longer matches the current publication. Reassessing only
the age of an old retained snapshot cannot detect a later route edit.

## Snapshot and units

- Route GUID and revision scope/revision number.
- Active waypoint GUID and **zero-based** index; total waypoint count.
- Optional remaining distance in **nautical miles**. Missing is never zero.
- Completed-progress observation time and separate position observation time,
  both in the process's monotonic `vessel::Clock` domain, not UTC.
- Explicit validity/rejection state; progress and position provenance strings.

OpenCPN 5.12.4 exposes no route revision counter. The bridge assigns a monotonic
process-local revision when observed route identity, ordered point identities,
coordinates or stored leg lengths change, including activation/deactivation.
The scope identifies that bridge session. This is not a persistent database
revision or a history of every edit. Waypoint advance changes active identity
and index without pretending that the route geometry itself changed.

## Upstream distance semantics

The reducer starts with `Routeman::GetCurrentRngToActivePoint()` and adds
`m_seg_len` for points strictly **after** the active point, matching the route-total
branch of `gui/src/concanv.cpp`. It excludes the active point's incoming leg and
never substitutes `m_route_length`. There is no independent geodesic calculation
in the bridge or Vessel Data. OpenCPN's current range uses `DistGreatCircle`;
its stored later legs use `DistanceBearingMercator`. The contract retains double
precision; the legacy console rounds through float and display formatting.

The first point's incoming length is ignored because that point has no incoming
route leg; reversal/shared-point history can leave an irrelevant value there.
All actual stored route legs must be finite and nonnegative. Ordered waypoint
GUIDs must be nonempty and unique, and the route/manager active point must agree
and appear exactly once in the route. Invalid coordinates are rejected too.

## Coherence and freshness

Copies bracket **normal** upstream progress. A route geometry, active point or
position change inside that pass rejects the result. This matters because
upstream can advance to another waypoint while retaining the just-computed
range to the previous one. An observed edit/reversal or active-point change
between passes also gets an unavailable transition. An open route/waypoint edit
or route creation is withheld even if its copied values temporarily stop changing.
A subsequent stable normal
pass can produce distance for the new state.

A temporary event filter detects nested event dispatch during the pass. It
never modifies/consumes events. Such a pass is rejected as `InterruptedPass`,
including edit-then-restore sequences which compare equal afterwards. The
filter is detached before any OpenNav diagnostic/test consumer runs.

Position must be measured, sourced, finite, within coordinate bounds, have
matching latitude/longitude timestamps, and match upstream `gLat/gLon` while
upstream considers GPS valid. Missing/uncertain/future position suppresses
range. The established 2-second aging / 5-second stale rules apply. A retained
snapshot can also become stale if normal progress stops even while position
continues. Consumer assessment checks both ages and withholds stale distance.
Old observation/position timestamps cannot overwrite a newer distance as valid.

No route, removal, deletion, ambiguous active point, invalid leg/range, overflow,
changed/interrupted processing and stale/missing position all yield an absent
distance with an explicit reason. A zero distance is possible only from an
explicit coherent finite upstream zero and zero subsequent legs. It is not an
OpenNav arrival event or a control instruction.

## Tests and isolation

`route_progress_contract` exercises first/middle/final points, arrival/advance,
skip, deactivation/deletion, reverse/edit, repeated identity, invalid active
point, negative/nonfinite/overflow distances, missing/stale/future position,
out-of-order observations, revision changes, no route, interrupted processing,
consumer-read aging and retained-copy lifetime. A test-only energy consumer
uses the existing `EnergyInputs` contract; production energy/UI wiring is absent.

`OpenNavRouteGeometry.*` is attached to the real pinned OpenCPN model test target.
It constructs actual route/waypoint objects and compares first/middle/final
antimeridian distances against pinned geometry/stored legs and legacy traversal.
It also reverses/edits/deletes objects, tests duplicate GUID/object membership,
checks worker-thread rejection and simulates a nested edit-then-restore event.

`smoke-navigation.py --route-fixture` feeds synthetic NMEA into a private
loopback-only profile. An opt-in test-build driver creates disposable routes and
observes normal application timer passes through activation, arrival, skip,
reverse/edit, duplicates, input loss/recovery, deactivation and deletion. It
compares snapshots against actual upstream progress, records provenance/times,
and captures the native window. It never manually runs route processing.
The driver requires `OPENNAV_ENABLE_ROUTE_SCENARIO=ON`, upstream tests enabled,
an explicit CLI flag and profile marker; it rejects output-capable connections.
Release/default builds do not include the driver. Existing mode, navigation,
energy and restart tests remain required independently.

No final route UI, route editing workflow, battery adapter, production arrival-SOC
connection or steering capability is added by this slice.

## Required scenario coverage

| Requirement | Automated evidence |
| --- | --- |
| First / middle / final active waypoint | Portable cases; real upstream antimeridian test; normal-timer scenario |
| Arrival / advance | Portable old-range rejection and recovery; upstream actually advances during application scenario |
| Skipped waypoint | Portable transition; actual `ActivateNextPoint(..., true)` followed by normal timer |
| Deactivation | Portable absence; immediate getter invalidation and following application pass |
| Deletion | Real model removal and retained lifetime; deletion while active in application scenario |
| Reversal | Actual pinned `Route::Reverse`, revision rejection and stable recovery |
| Active edit | Geometry/leg change; open route or waypoint edit flags; application edit and recovery |
| Repeated identity | Duplicate GUIDs and the same object appearing twice in a real route |
| Invalid leg distance | Negative and nonfinite incoming legs, including already-travelled legs |
| NaN / infinite / overflow distance | Explicit portable rejection with absent result |
| Missing position | Portable absent sample; no default coordinate/distance |
| Stale position | Exact 5-second boundary, retained-snapshot assessment, loopback stop/resume |
| Out-of-order observation | Observation and position watermarks independently tested |
| Route identity / revision change | Stable revision, changed GUID, edit/reverse revisions and transition suppression |
| Antimeridian geometry | Actual pinned `DistGreatCircle` and stored Mercator legs, compared with console traversal |
| No active route | Portable absence and normal application pass |
| Route changed during read | Before/after rejection plus nested event edit-then-restore guard |
| Lifetime / thread boundary | Immutable retained values survive model deletion; worker-thread read rejected |

The model fixture supplies a minimal GUI-friend shim to initialize empty icon
collections normally created by the real GUI. The model test executable does
not link the GUI implementation. This keeps normal model destruction valid
without changing upstream access, ownership or navigation calculations.

## Accepted evidence

All seven portable contracts pass on Linux and native Win32 MSVC, including 28
route scenarios and the existing energy tests. Each platform also passes ten
additional restart lifecycle repetitions. Clean integrated builds pass 67
compiled Linux tests and 57 Windows tests, including seven route/model tests.
Each application passes all 26 normal-timer route observations, the synthetic
navigation-input sequence and the shared-profile XNav / Legacy / Safe cycle.

The [native review](evidence/windows-954b450-review.json) records 13 reviewed
1280×800/96-DPI captures, executable and artifact hashes. The
[Linux record](evidence/linux-954b450-review.json) records the clean hosted gate.
[Original native observations](evidence/route-954b450-observations.json) retain
validity, provenance and times; the matching Linux run agrees on every state,
route revision, active identity/index and distance. No invalid observation
contains a distance value. These are synthetic-data development gates; hardware,
installer, real-chart and broader DPI release acceptance remain open.
