# Beta 2 waypoint detail and route draft state

This is a local Linux workflow review, not native Windows or boat-PC acceptance.

## Reference intent

Visible actions must describe what can actually be done. An open detail must
follow the shared OpenCPN object; source loss must not leave GO TO apparently
usable. Draft controls must retain OpenCPN's existing undo semantics.

## Findings and changes

Waypoint detail previously retained its first catalog copy. It now reads the
unique copied waypoint context at most once per second while visible, updates
names and protection, and removes actions when the identity is deleted or
ambiguous. Invalid coordinates display **Position unavailable**. GO TO requires
current selected position and a usable waypoint; view/edit actions remain
independent of GPS where appropriate. Modal intent remains an owned revision
which integration rechecks after confirmation.

Draft Undo was always enabled and discarded failure results. Its enabled state
now inspects the same native draft/undo stack predicate as execution. It is
disabled with zero or one route point, and unexpected command failure opens a
visible message. Neither check invokes route progress or hardware output.

## Local evidence

The integrated fixture build passed. The expanded real-loopback object scenario
passed **24 groups / 25 captures**, including a detail kept open through rename,
protection, invalid geometry, duplicate identity and native deletion. Stopped GPS
disables GO TO on both the compact card and detail; resumed GPS restores it.
Missing, invalid and duplicate chart-centering requests leave the viewport
unchanged. The affected replay control-isolation contract passed.

The actual pointer workflow passed **8 groups / 9 captures**. It verifies Undo
disabled at zero/one point, enabled at two, disabled after undoing back to one,
and enabled again after another chart point. The final read-only database audit
still proves the named two-point route, preserved original destination, and no
cancelled draft. Existing Go To, stop, orientation and modal-cancellation checks
remain in the same run.

Reviewed 1280×800 images in ignored local evidence:

- `objects-waypoint-detail-stale-position.png`: critical position-loss notice,
  explicit GO TO explanation, disabled GO TO, and usable independent actions.
- `objects-waypoint-detail-invalid.png`: no NaN display; all geometry-dependent
  actions disabled and Back available.
- `objects-waypoint-detail-deleted.png`: unavailable title with no stale object
  actions; Back remains visible.

These screenshots use the dedicated fixture executable, whose test-only Demo
shortcut is expected. The installed product still excludes test fixtures.
Exact-commit native Windows and physical-display review remain required.
