# SCRUM-216 Display preferences — local increment, 2026-10-01

The immutable HTML Display section supplies two draft selects: Interface scale
100/125/150% and Chart layout Balanced/Chart focus/Instrument focus. Apply
commits both. Theme remains immediate; rebuilding the section discards
unapplied select choices. This increment replaces the inactive native Display
copy with owner-drawn select fields and a deliberate Apply action. The existing
theme, instrument-personalisation and fullscreen actions remain available;
chart-presentation controls remain under Navigation settings.

The choices use a separate, strictly decoded `/OpenNav/DisplayPreferencesV1`
entry in the shared OpenCPN profile. The older `/OpenNav/AlphaSettings` record
is byte-preserved for rollback compatibility. A failed profile flush restores
the previous display entry and leaves the in-memory applied preference intact.
The XNav shell applies the committed desktop layout to its data rail and
horizon panes and adjusts its rail metric font. It does not change OpenCPN
chart geometry, navigation settings, OS DPI, sensor data or Legacy Mode.

The original `79179ad` increment applied interface scale to the Settings
drawer. `943c8f4` extends it selectively to the owned ProductPanel and drawer
controls, and to modal sheets. Scale changes are event-driven when Display
preferences are applied; the sensor `Tick()` path does not propagate scale.
Shell-owned sheet calls pass the applied scale explicitly. For other calls,
`EditSheet` and `ConfirmSheet` infer scale only when a containing owner is an
XNav ProductPanel or drawer; standalone sheets retain their 100% default.
The AISStream credential prompt remains a separate masked field with its
512-character limit and protected-storage path unchanged.

This is selective propagation, not a claim that every prototype global
selector or surface is complete. The subsequent large-desktop geometry
increment maps the prototype's >=1500px shell sizes into the owned top/nav,
rail, horizon and drawer, including the 1500x900 metric-size breakpoint.
The <=760px Display variant still requires checking.
SCRUM-216 remains In Progress. The 100% Linux capture still wraps the Display
tab to a second row and differs from the immutable browser reference. Native
Windows typography/DPI, 1280x800 and 1920x1080 layout, mobile/responsive
acceptance, and boat-screen acceptance remain open. This local component
evidence does not qualify a product release.

Focused local checks in the isolated `scrum216-display-preferences` worktree:

- `opennav_ui` built with the project-local wxWidgets toolchain, jobs=2.
- The changed `OpenCPNIntegration.cpp` passed syntax-only compilation with
  the actual pinned integrated Linux build flags and upstream headers; this
  does not establish a full linked OpenCPN build.
- SettingsStore tests pass 3/3 new cases: all nine scale/layout combinations
  reload from disk, invalid records are preserved and rejected, and failed
  flush restores the old applied value.
- ChoiceField's isolated X11/Xvfb lifecycle test passes, including keyboard
  commit, Escape/reopen, disabled and owner-destruction behavior.
- The earlier Settings drawer's 1280x800 X11/Xvfb driver passed 154 checks,
  including real pointer popup activation, draft-only choices, failed
  Apply/retry, immediate 432-to-460px width change and existing Vessel/Sensors
  paths. Its ignored local captures are under
  `build/settings-display-capture/`; this is historical evidence from before
  the selective propagation increment.
- On the updated worktree, the Settings drawer X11/Xvfb driver passes 163
  checks and records 14 captures. The machine-readable result is at
  `build/settings-display-final-capture/result.json` in the
  `scrum216-display-preferences` worktree. It includes 125% Chart Focus and
  an injected 150% Night state to inspect 56px fields/actions and scrolling.
  The 150% state was set by the offline component driver, not a product
  profile write. These Linux captures document local behavior only, not
  Windows visual acceptance.
- The large-desktop geometry test passes its browser-derived 1280x800,
  1280x600, 1500x800, 1500x900 and 1920x1080 cases, plus Chart Focus and
  viewport-driven metric label checks. The X11/Xvfb Horizon component passes
  257 checks with a real 1920x1080 capture under
  `build/horizon-wide-capture/`.
- A derived offline renderer exercised 12 combinations of the unchanged HTML
  at desktop and responsive viewport sizes under
  `build/prototype-display-reference/`. Linux browser captures are reference
  behavior evidence, not Windows acceptance.

Native Windows and boat acceptance are still open. The frozen `e9737d6` native
candidate, its CI, and the boat PC were not changed by this increment.
