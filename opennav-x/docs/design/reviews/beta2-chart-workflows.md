# Beta 2 chart interaction review

The reference intent is a dominant chart with brief, legible object cards and
focused editing sheets. OpenCPN remains the only route/waypoint database.

The chart-position card now shares the AIS/waypoint component, with four clear
actions and explicit outside/Escape dismissal. Its coordinates are copied from
the actual chart gesture. Go To requires current measured position; historical
or simulated state cannot create a route or waypoint through this card.

The first mouse-input test exposed an interaction problem hidden by the
earlier model tests: re-entering an already-visible Navigation view needlessly
laid out/resized the canvas. OpenCPN's normal delayed frame recapture could then
raise the main frame above an owned context card in bare Xvfb. Navigation now
avoids that unnecessary layout. The passing chart workflow does not need a
synthetic button event or a manual native-window raise to dispatch the action.

Linux development evidence is `evidence/local/user-flows-results.json` and the
eight `flow-*-linux.png` screenshots. The input-only test created a waypoint at
the selected chart location, started/stopped an upstream Go To route, entered
three route points, undid one, cancelled naming without losing the draft, and
saved the named two-point route. A separate discarded draft left that route and
the original waypoint intact. The final SQLite audit is read-only.

The first review found truncated Done/Undo/Cancel labels on the 48-DIP tool
rail, and a route-detail screen dominated by six equally prominent actions.
The second pass moves draft actions into the existing bottom row, replacing
the three navigation-page actions only while creating a route. Cancel/Undo/Done
are 88×48 DIP; Pilot/STBY/System remain available. The normal chart rail and the
chart viewport do not change size. The repeated flow now asserts that geometry.

Route detail now leads with destination/departure or active remaining-distance
information, two primary actions, and readable planned-leg rows. Less frequent
editing/reversal actions are quieter and follow the leg list; long routes use
the existing touch scrolling. Remaining distance uses the assessed route
snapshot, and estimated travel time uses matching SmartNav advice. Planned
distances/courses are the copied upstream leg values, not a new calculation.

The repeated Linux test passed seven workflow groups and captured eight screens.
Reviewed `flow-05-route-name-linux.png` now shows all three draft labels in full;
`flow-06-saved-route-linux.png` shows the passage and planned legs ahead of the
editing options. The dark naming sheets and chart card remain unclipped at
1280×800. Native Windows, non-default DPI and boat-display review must still
qualify these replacement screens; this Linux record does not claim those gates.

## Route detail lifecycle correction

The source audit found that an already-open detail retained its selected route
copy after normal arrival/deactivation, and that an old Stop confirmation could
stop whichever route was active when confirmed. Beta 2 now reconciles only the
selected owned route once per second while visible. The bridge rechecks the
rendered identity/revision and requires that exact native route to be active
before Stop. Missing or ambiguous identities show unavailable and remove route
actions. Commands capture the rendered selection before entering a modal sheet.

Timer-driven rebuilds are deferred while a modal dialog exists, preserving
pending changes until it closes. The actual GTK test showed that the parent
frame can remain `IsEnabled()` during a native modal grab; the guard therefore
also checks the real `wxDialog::IsModal()` lifecycle. No route-progress or
hardware-output method is used as a getter, and no upstream hook changed.

The expanded input-only object fixture observes external rename and activation,
normal upstream waypoint advance and completion, and deletion without reopening
the detail. A real activation sheet stays open through an external route edit
and two refresh intervals; confirming its original selection is rejected.
Native Windows qualification for these changes remains required.

Linux development revalidation passed 21 grouped object checks with 18 captures
(`objects-input-results.json`) and the existing seven chart-workflow groups with
eight captures (`user-flows-results.json`). Pixel review of the final active and
advanced detail captures shows 14.5 NM / next point 3 changing to 8.7 NM / next
point 2 after normal upstream arrival. Completion restores Activate, deletion
shows Route unavailable with no mutation actions, and the unchanged confirmation
sheet remains intact while its original selection becomes stale. These are
isolated fixture results, not live navigation or native Windows acceptance.
