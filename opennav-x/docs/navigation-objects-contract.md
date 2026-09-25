# Alpha navigation objects and context cards

Status: Linux and native Windows qualification passed at `7bc36e4`; final
packaged release acceptance is recorded in [status](status.md). OpenCPN remains
the sole owner of navigation objects and their storage.

## Boundary

`integration/NavigationObjects` copies routes, route points and AIS targets on
the application thread. `application/NavigationObjects` defines owned values
and explicit human-action callbacks. `ui/ProductPanel` receives no upstream
pointer. A retained route, mark, target or anchor observation is independent of
subsequent model changes.

The catalog carries GUID, name, description, coordinates, protection state and
an exact change identity serialized from the copied object. Route identities
include ordered waypoint revisions and stored leg distance/course. Commands
resolve a unique GUID and recompare the current revision before mutation. A
stale selection requires refreshing the catalog. Duplicate GUIDs, active edits,
layers, active-route points and anchor marks are protected. A route member may
not be deleted as an isolated mark. Large catalogs are capped and direct users
to the existing manager.

## Reused OpenCPN operations

- Route activation uses `Routeman::FindBestActivatePoint` and `ActivateRoute`
  with the selected navigation position; XNav refuses missing/stale position.
  The confirmation identifies that normal configured OpenCPN navigation outputs
  retain their existing behavior. XNav adds no autopilot output path.
- Stop navigation calls the normal `DeactivateRoute`.
- Reverse calls `Route::Reverse(false)` and updates the existing selectables
  and `NavObj_dB`. Names are retained. Active/protected routes cannot reverse.
- Basic route/waypoint properties are written through `NavObj_dB`; failures
  restore prior in-memory text. Reversal write failure restores prior order.
- Creating a route uses the existing chart interaction (`StartRoute` and
  `FinishRoute`). Creating a mark uses `RoutePoint`, `InsertRoutePoint` and the
  existing selectables. Coordinates are explicit chart-center coordinates.
- Isolated waypoint deletion follows the stock canvas `Undo_DeleteWaypoint`
  action. OpenCPN owns the orphaned point through its undo history; XNav does
  not delete a pointer which the upstream undo stack may still need. Database
  failure cancels the undo candidate without freeing the registered point.
  Hidden legacy mark properties are cleared; open property edits are protected.
- Catalog viewing centers the chart on the selected point/first route point.
  Advanced route geometry operations, GPX and legacy dialogs remain accessible.
- Measure, orientation, AIS visibility, object query, zoom/follow and fullscreen
  call the existing canvas/frame commands.

Demo telemetry does not create an alternate route database. The catalog is
explicitly labelled as real OpenCPN objects even in Demo mode; starting real
navigation and anchor watch from synthetic position are disabled.

## AIS

AIS cards/list read `AisDecoder::GetTargetList` and copy existing target state,
CPA, TCPA, range, bearing, alarm and motion results. There is no second collision
or COLREG calculation. No `UpdateCPA` or autopilot method is called as a getter.
Position-report timestamps establish target observation age; relative quantities
also require fresh selected own-ship position. Lost/doubtful/unreported targets
and AIS unavailable sentinel values have no valid numeric value. Repeated UI
reads do not make old target reports fresh. The Demo source produces explicitly
labelled, owned encounter fixtures and never inserts them into the live AIS model.

Narrow chart-selection hooks defer route, mark and AIS context cards until the
upstream mouse/menu stack has unwound. Only the GUID or MMSI is captured by the
deferred call. Legacy continues down the original dialog path.

## Anchor watch

A narrow hook observes after normal `MyFrame::ProcessAnchorWatch`. It copies the
registered watch mark, upstream radius convention and alarm state. The displayed
distance uses the pinned OpenCPN `DistanceBearingMercator` path; no independent
drag detector is introduced. The snapshot retains source, observation time and
selected-position time. Movement history is bounded and only advances on new
position observations. Clearing retains the mark, emits the normal plugin
notification and invalidates the displayed watch; setting an existing watch is
never implicit. Demo cannot set or clear a real anchor watch.

## Tests and acceptance

`NavigationObjectScenario` is enabled only by a dedicated command-line flag and
marker in a disposable test profile. It refuses output-capable connections.
`smoke-navigation.py --objects` drives the real input bus, actual database and
GUI hooks, checks the three selection cards, exits cleanly, then queries the
persisted database. Existing route-distance, selected-navigation, instruments,
mode/restart and portable package gates remain in place. Native Windows remains
required for acceptance; this document alone does not assert a passed gate.

Local pre-commit validation passes 81 integrated tests, 22 portable contracts,
mode lifecycle/persistence, selected navigation, route progress, live instruments,
and the ten-check navigation-object scenario. The preview gate opens eight Alpha
pages and retains seven deterministic coastline checks. A Linux Ctrl+Shift+U
collision with GTK Unicode input was caught by the interaction test; the pilot
shortcut is now Ctrl+Shift+Y and the test requires an explicit diagnostic page
identity before capture. Native property-sheet and pilot command interactions
now pass in the native Windows gate. The full qualification retains 90 Linux /
80 native Windows integrated cases and 30 portable suites on each platform,
including persisted object edits, actual native sheets, AIS cards, anchor state
and feedback-confirmed simulator controls.
[Exact qualification evidence](evidence/alpha-installer-7bc36e4-qualification.json).

The first route point has no incoming planned leg. Its OpenCPN route-properties
course is not initialized by the route constructor, so the bridge never reads
that field for the first point, including when constructing a revision identity.
Subsequent leg courses are the stored values set by normal upstream route logic.
