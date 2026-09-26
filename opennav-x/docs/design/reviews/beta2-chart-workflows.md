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
