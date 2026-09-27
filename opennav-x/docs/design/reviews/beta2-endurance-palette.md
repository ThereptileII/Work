# Native endurance palette control

Source review found that the native endurance harness still requested the old
`Light` caption, although the actual status-bar control now reads `Day`, `Dusk`
or `Night`. The exact-caption helper has no alias, so this would fail on the
first Windows endurance iteration. This finding preceded that step in candidate
`8e780edc34f68abd693a5d5f6aecdb3ba05a75c4`; its active run was not changed.

The harness now uses the existing `cycle_light` helper, which resolves the
unique palette control beside the status-bar brand. Each action also waits for
the expected next palette in the actual diagnostic state and records it.
No duration, source-loss, route-progress, resource-growth or clean-exit check
was removed.

The other native controls were checked against the current Shell, ProductPanel
and deterministic scenario source: Navigation, Route, Energy, Menu, Vessel
instruments, AIS targets, SmartNav advisories, Demo, Cruising, Sensors stale,
Sensors unavailable, and the two zoom symbols still match their actual paths.
These are isolated fixture controls, never installed-product Demo controls.

Python compilation passes. Actual native execution and the required three-hour
run remain gates for the next exact product candidate; source inspection does
not establish endurance acceptance.
