# SCRUM-229: Preferences touch-scroll target at 125% native DPI

The retained FFE native run completed 100% DPI, then failed at 125% / 120 DPI
with `Lower Preferences action cannot be reached by touch`. This review and
harness correction use source base `138ebb5`; no product control behavior is
changed. Native positive replay remains pending.

## Retained failure and source boundary

Reviewed `prototype-post88/evidence/local/ffe-native-dpi/files/dpi-results.json`
and the actual 1280x800 `dpi-125-failure-visible.png`. Their SHA256 identities:

- JSON: `224a76f2ca32e421e6450dafc41e1ed6f2dd9a974724cd7666a19a31d68f05f5`
- PNG: `04ad300a2799579207447f852b0e4ddfefab36a98ed54c17b8a0938bd4533fe8`

The foreground is `OpenNav preferences`. The drawer is x545,y128,w513,h605.
The old endpoint loop starts its pan at drawer center / bottom minus 50:
**(801,683)**, ending at **(801,483)**. The visible native field
`Usable battery capacity · kWh` occupies **x590,y683,w422,h28**. Thus the
nominal scroll gesture starts exactly on that editable field, not on the
scroll body. The screenshot still shows the top Vessel fields after the
40-attempt loop. The lower action remains at y1067 and Save at y869.

`SettingsDrawer::VesselForm` creates the field with `wxTextCtrl`. It does not
install `EnableScrollGesture` on the input or its enclosing painted panels.
`XNavScroll` does install that handler on the body; its `Pan` changes the
actual view start. wxWidgets 3.2.8 configures gestures per HWND and dispatches
`WM_GESTURE` to the receiving window:
[official MSW implementation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/window.cpp#L852).
The retained old loop did not record `WindowFromPoint`, gesture delivery or
per-attempt displacement. Consequently, these records establish a faulty
unverified start point, not a proven defect in native text-edit gesture
handling. The correction does not change editable-field semantics.

## Bounded correction

Both Preferences scroll consumers now use one reach-only helper. It obtains
the actual action HWND and its actual drawer-body parent, pairs the action's
native bounds with fresh diagnostics, and chooses a vertical path in the
body's existing side padding. **Both path endpoints must hit the body HWND
itself**; an arbitrary descendant such as an Edit is not sufficient. The
foreground and hits are checked again immediately before real injected touch.
No native click message, accelerator, wheel or direct scroll call substitutes
for the gesture.

Each attempt retains paired target bounds/tick and the injected path, body
bounds, native HWND/class and exact body-hit result (including rejected candidates). A gesture that does not
move a still-clipped action fails on the next paired observation instead of
repeating an unverified gesture 40 times. Missing/covered body padding fails;
there is no fallback to a field or another surface.

The first consumer only reaches the endpoint. Its existing 1.2-second
stability check, full visibility, all five field identities, enabled Save,
and fully visible Save remain required; it does not activate Save or the
lower action. The later action consumer additionally verifies the actual hit
and injects a real tap on the reached action.

## Validation and remaining native gate

Nine focused Python checks pass. The negative regression uses the exact
retained drawer/field rectangles and proves the old point lands on the field.
Positive selector checks use explicitly synthetic body/hit fixtures; they
cover field avoidance, rejected child/covered endpoints, alternate gutter,
reverse direction, short-body refusal and 100/125/150% distances. They do not
claim Windows gesture delivery or product touch acceptance. Python syntax
and whitespace checks pass; no application build or CI was run.

The completed FFE artifact does not retain `opencpn.exe`, so it cannot support
a direct full-app replay. The opt-in `[preferences-touch]` push marker on
`skager-windows-changed-units`, or manual input `preferences_touch: true`, now
selects `test-windows-changed-units.py --settings-touch-only`. The existing
15-minute job compiles the production UI library, its existing SettingsStore
compile guard, a separate non-installed Settings touch host, and the existing
`tests/WindowsDpi.cpp` helper. It does not build OpenCPN or dependency producers,
or run the normal Settings/Search/Chart/Energy component sequences.

The touch host establishes PerMonitorV2 before wxEntry, then the runner requires
both GetDpiForWindow and wx DPI 120. Host frame style matches pinned
`ocpn_app.cpp:1747` (`wxDEFAULT_FRAME_STYLE | wxWANTS_CHARS`); its workspace uses
the same production DisplayDesktop calculation as Shell::DrawerWorkspace.
The runner sets the same 1280x800 outer frame and strictly requires the retained
1262x753 client, drawer (545,128,513,605), and battery field (590,683,422,28) before
the negative gesture. A mismatch fails; no adjusted negative-control point or
tolerance is allowed. The host uses actual SettingsDrawer widgets and empty
synthetic vessel fields, not a mirrored control implementation.

The native sequence records a single old field-targeted pan, requiring actual
Edit hit, native child input and zero body displacement. A read-only native
subclass observes WM_GESTURE and WM_LBUTTONDOWN and always delegates to the real
widget. The corrected sequence uses the exact shared `preferences-touch.py`
reach helper also called by `smoke-dpi-windows.py`; both endpoint hits must be
the real body. It requires actual body pan delivery and displacement, stable
fully visible lower action after 1.2 seconds, all five fields, fully visible
enabled Save, unchanged field values and zero saves/actions while reaching.
One final injected touch must invoke the actual battery callback exactly once.
Three screenshots and before/after native observations are retained. No widget
scroll calls, accelerator or HWND click messages substitute for touch.

Work is bounded by a 65-second observer/injection budget and 90-second outer
process timeout; the host uses a single-shot timer and 85-second deadline.
Cleanup and original-DPI restoration are attempted independently; either failure
fails the proof. Source, executable, DPI helper and locked wx runtime hashes
are retained/rechecked. The normal 96-DPI Settings driver is unchanged.

Local validation: nine focused selector tests, four existing runtime-staging
tests, three native/mode refusal checks (no output directory created), Python
syntax, workflow YAML parsing and whitespace checks pass. Native compilation,
negative/positive touch behavior and screenshots remain pending; no workflow
was dispatched here. The full combined candidate still requires the complete
100/125/150% native DPI run. Physical boat touch acceptance remains separate.

## First native component result and observation correction

[Run 37055218734](https://github.com/ThereptileII/Work/actions/runs/37055218734),
job `110998128701`, source `e0e0e9ef71714932ab69d533d1f86cbcd48e33b3`
(local equivalent `cf07197`), compiled and launched the native component.
Retained artifact `11248396913` is 12,075,096 bytes, ZIP SHA256
`90c99917abe86a615cc946696bcbb251cc8dc355474403a258a371ef954878c2`.
Its `windows-changed-units/settings-touch/result.json` records failure at
`Old field pan unexpectedly reached body`, before the positive body-gutter
sequence. The exact frame/client/drawer/field assertions passed at native and
wx DPI 120. The old point (801,683) hit the expected battery `Edit` HWND 66124.
After real touch, scroll stayed zero and all recorded control geometry and
field values remained unchanged. Counters increased from zero to 12 child and
4 body pan messages. The inspected screenshot still shows the top Vessel
fields. Host cleanup completed with zero actions and saves; original 100% DPI
was restored. No full-application or boat result is implied.

The zero-body-message assertion was an unsupported hypothesis about delivery.
`tests/settings_touch_test.cpp` counts native message arrival before forwarding
to `DefSubclassProc`; it does not measure successful scroll handling.
[wxWidgets 3.2.8 WM_GESTURE dispatch](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/window.cpp#L3167)
converts arriving messages and closes their handles only when processed;
[HandlePanGesture](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/window.cpp#L5731)
separately computes a delta and dispatches the wx event. Thus body message
arrival cannot establish useful scroll displacement or exclusive field
consumption. The evidence does not identify the complete per-message route.

The follow-up removes only that assertion. All counters remain retained, as
do the exact negative Edit hit, zero displacement, native child input,
unchanged fields and every positive body-hit, displacement, stable endpoint,
visible enabled Save and single-action assertion. Product behavior is unchanged.
Only Python syntax and whitespace/diff checks were run for this correction;
the positive native sequence remains pending a separate reviewed replay.
