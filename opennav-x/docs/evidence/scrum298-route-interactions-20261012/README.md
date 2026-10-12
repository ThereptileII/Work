# Route completion and selection — focused source evidence

Source: `36bcd5a7`, integrating route completion `e5ac8b37` and route DPI
`3a11990e`. Exact file hashes and the pinned upstream revision are recorded in
`source-receipt.json`. These are source/component results, not installed Windows
or boat acceptance.

## Findings and changes

Native mouse/touch reuse and great-circle dialogs, keyboard context menus and
right-click menus overwrote `m_FinishRouteOnKillFocus` with `true`. An XNav route
draft could therefore be finished by focus loss before the explicit naming
action. The new narrow upstream patch uses a scoped restoration of the caller's
policy across all eight modal scopes. Existing Legacy policy is preserved,
including exceptions and early exits. Chart-name lookup and stored navigation
semantics are unchanged.

Upstream initialization replaces the selection pixel radius with fixed 12/50
pixel values. The existing XNav route multiplier consequently produced a fixed
18/75 pixel target even at higher DPI. The second patch supplies the actual
querying canvas's physical pixels per DIP at all 15 native contexts. Only
enlarged route segments use this scale. Legacy, default/three-argument callers,
waypoints, tracks, AIS, tides and currents retain their prior behavior. Invalid
DPI scales fall back to the original radius. Selection geometry remains pinned
OpenCPN Mercator/vector code.

Both patches are registered in actual preparation, corresponding source and
Windows dependency input inventories. Linux CI now invokes both focused runners
after fetching the pin. The selection runner refuses omitted/duplicate actual
patch registration.

## Verification

- `focus.log`: 250 assertions per desktop and macOS timer configuration pass,
  compiling extracted actual focus, popup and mouse/touch modal handlers with
  inert collaborators. These are 500 assertions, not 500 independent test cases.
- `hit.log`: 295 assertions pass across actual selection methods and pinned
  geometry, including 100/125/150% and physical-pixel canvas conversion.
- `source.log`: one directly affected archive/hash test passes; both route patch
  hashes are present in the exact source inventory.
- `reuse-corrected.log`: one directly affected dependency-input tamper test
  passes, including the new patch input.
- Agent controls reproduce the original focus overwrite and 125% selection
  failure without their respective fixes. Both failing controls are retained
  here as `original-focus-control.log` and `original-hit-control.log`; the latter
  was also reproduced against the root combined patch inventory.

Failures are preserved: `focus-extraction-failure.log` is an initial fixture
extraction error, corrected before the expanded checks; `reuse.log` is the root
runner's incorrect unittest class name, corrected without product changes in
`reuse-corrected.log`. Neither is a product or acceptance pass.

## Remaining gates

Native wx focus/DPI/event delivery, actual named route persistence/cancellation,
combined Windows build and boat interaction remain pending. Extracted handlers
and inert collaborators do not qualify them. No full application build, CI
dispatch, source push, installed app/profile change or hardware command occurred.
