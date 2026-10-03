# SCRUM-252 software route capture: missing fixture repaint notification

The apparent software painter failure in the f0976cc capture was caused by
missing invalidation after **direct test-fixture mutations**. Normal route
commands already request repaint. This change only repairs the opt-in
RouteProgressScenario; no production painter, eligibility, geometry, navigation
processing, appearance or display default changes.

The bounded diagnostic kept the original zero-fill visual assertion and frozen
executable unchanged. On failure it clicked the existing Day → Dusk → Night →
Day control and retained each image, then still failed the original assertion.
All three refreshed software captures display the actual SIM 3 floating name
and 01 circle. The exact card-fill probe has431 matching pixels in Dusk, Night
and returned Day. The previously stale leg now extends to the bottom of the
chart, matching the real restored geometry. The active SIM 2 name remains stock.
The software understroke is also present after repaint: at y260, Day pixels
x633 and638 are(243,244,234), surrounding the teal core at635/636(38,124,118),
against land(238,238,226). Dusk/Night show the corresponding darker surface
understroke. It was not necessary to change the graphics context or guards.

Pinned upstream `chcanv.cpp:11995,12162–12172` renders overlays but finally
blits only the invalidated `rgn_blit`. The fixture already refreshed creation,
but its subsequent active-point changes, skip, arrival setup, reversal, geometry
edit, deactivate/reactivate and deletion bypassed normal UI invalidation.

Normal notifications are confirmed in pinned `routemanagerdialog.cpp`:
`OnRteReverseClick` calls `RefreshAllCanvas` at1566 and `OnRteActivateClick` at1742.
SKAGER normal activate/deactivate/reverse/edit actions all pass through the result
wrapper in `src/integration/NavigationActions.cpp:22–25`, which invalidates GL
and refreshes every canvas. There is no demonstrated normal product painter
failure from this fixture capture.

`tests/RouteProgressScenario.cpp` now schedules `RefreshAllCanvas` after only
the direct visible mutation steps2,4,6,8,10,17,18,20. This is an asynchronous
paint notification, not a navigation update, and it does not rewrite any model
value. Removing only the added notification statements/comments yields the
previous scenario byte-for-byte: every original assertion and operation,
including all26 contract checks, remains unchanged. Identity-only corruption/
restoration steps and observation/wait steps receive no added redraw.

The actual changed scenario object compiles successfully against the final
family's pinned patched upstream headers and production flags. A new integrated
executable is required to verify the repaired scenario through all26 checks and
unchanged hot-theme visual assertions. Only `RouteProgressScenario.cpp.o`,
linking and normal candidate metadata/staging need rebuilding; no upstream or
product helper objects changed. Current diagnostic screenshots demonstrate the
cause and already-working software paint after a normal repaint, not a passing
run of the new source. Native Windows/boat/release gates remain open.

[Focused receipts](../../evidence/scrum252-fixture-repaint/) include the original
failure, controlled repaint images, source/executable/resource identities,
actual compilation command/output and byte-preservation proof result.
