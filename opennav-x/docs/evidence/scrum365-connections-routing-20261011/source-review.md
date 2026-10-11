# SCRUM-365: existing Connections routing recovered

Reviewed coordinator `c63dee40445f4b084468e3076c7d33a90eea118a` on
2026-10-11 following selection comment 11319. The fix is already present;
this increment records the source evidence and changes no routing code.
Jira's earlier comment 11242 also records this fix in Staging
`bf68b355cd319a0ef837f4119b41d4824f95022c`. Local source provenance is
integration commit `b3cef57cc52091cbba4e2036423abf1b440bc238`.

The primary path is `SettingsDrawer.cpp:379` (Sensors / Add a sensor) to
`Shell.cpp:1626-1627`, which calls `ShowNavigation()` before the separate
`connections_settings` callback. `ShowNavigation()` dismisses the settings
drawer and restores the navigation panes (`Shell.cpp:1309` onward). The
callback at `NavigationActions.cpp:354-367` saves the previous page and
subpage, sets page 2 and subpage -1, calls the existing `frame.DoSettings()`,
then restores both saved values. It does not create a connection or claim
that a sensor has been added.

The exact locked OpenCPN source is
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` (Release 5.12.4). In that source:

- `gui/src/options.cpp:6299-6332` creates Display, Charts, then Connections,
  making Connections index 2. Its narrow-screen caption can be Connect.
- `gui/src/options.cpp:6379-6400` implements `SetInitialPage`; subpage -1
  leaves notebook subpage selection alone.
- `gui/src/ocpn_frame.cpp:3119` delegates `DoSettings()` to
  `DoOptionsDialog()`. The latter sets the normal working/configuration
  pointers and reads the page globals into `SetInitialPage` at line 4182,
  before modeless `Show()` at line 4231. The temporary override is therefore
  consumed before the callback restores it.
- `gui/src/options.cpp:8729-8736` records actual page changes in the dialog;
  `gui/src/ocpn_frame.cpp:4127-4137` retains upstream's normal close-time
  page/subpage preference. Immediate restoration prevents the forced open
  preference from persisting in the globals; it does not disable upstream's
  normal memory of a page the user leaves selected on close.

The unrelated Advanced entry follows `Shell.cpp:1624-1625` to
`NavigationActions.cpp:353`, whose callback still calls normal
`frame.DoSettings()` without a forced page. These are distinct production
callbacks. The existing drawer fixture delegates Add a sensor at
`tests/settings_drawer_test.cpp:349-350`, but aliases its callback to the
Advanced counter. That fixture is not evidence of the actual OpenCPN page,
keyboard/touch activation, window return, or persistence.

Source inspection confirms the existing implementation; no product fix,
new test, full build, native run, dispatch, push, visual review, or physical
operation was performed in this increment. The receipt records exact source
identities. JSON parsing and diff whitespace checks validate this evidence
increment only.

SCRUM-365 remains open for coordinator-owned native Windows and boat-PC
verification on the exact built candidate: open a different Options page,
close it, activate Settings / Sensors / Add a sensor by keyboard and touch,
observe the actual Connections page, exercise the normal connection editor
without an invented success state, close/cancel back to XNav, and verify
unrelated Advanced/Legacy behavior and normal page/subpage preferences.
Existing source alone does not accept those runtime gates. A rebuild is not
required by this evidence-only increment; a tested executable must already
contain the recovered production fix.
