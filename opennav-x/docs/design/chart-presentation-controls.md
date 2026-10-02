# Chart presentation controls — SCRUM-15 integration contract

This bounded increment implements the copied state and commands for the original
HTML `layers` panel. The selected Jira issue is SCRUM-15 (10475). The separate
[native drawer review](reviews/scrum-14-chart-presentation-local.md) records UI
implementation and its limited local evidence. The source baseline is OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`; the implementation starts from
OpenNav `24d0a5a148398e6adc943ee87fdfc03174b06e2d`.

## Original design and semantic decision

The immutable `docs/design/prototype/index.html:326` defines the chart-format
segment, eight layer rows and North/Course/Head-up segment. `renderPanel` at 321
first asks `renderSuitePanel` at 496, whose switch has no layers override. The
appended symbol handlers at 1184–1185 apply to the original layers panel.

The **Symbol labels** row controls OpenCPN's **ENC text master switch**. Its
supporting copy must make that scope explicit. It preserves the independent
buoy-label and light-description preferences. This is a documented semantic
adaptation of the illustrative prototype, whose labels also include hazards,
landmarks and harbour features. It is not a claim that all symbol labels are
visible when the master preference is enabled.

The Vector/Raster segment describes the current chart family, or quilt reference
family. It is observational only. It never changes charts or selects the
separate XNav/Standard palette preference. Unsupported/unknown families remain
unavailable. A quilt reference does not describe every overlay or member.

## Boundary and ownership

`application/NavigationObjects.h` defines `ChartPresentationState`,
`ChartLayerState`, `ChartOrientation`, `ChartFormat` and
`ChartPresentationResult`. All members are owned copies. Layer `visible` is the
observed upstream presentation preference: missing is unobserved/unsupported,
false is hidden. `editable` is separate from that preference. The UI must keep
unavailable reasons visible and must not turn a missing value into an off state.

`NavigationActions` provides `chart_presentation`, `set_chart_ais`,
`set_chart_enc_text`, `set_chart_soundings` and `set_chart_orientation`.
The integration binds those callbacks to the existing frame; each invocation
checks the application thread before accessing the frame, obtains its current
primary canvas and handles absence without dereferencing it. Desired-state
commands compare actual canvas state before invoking an upstream toggle. Every
result returns a fresh copied observation, including failures. No canvas/chart
pointer escapes, and no parallel preference cache is created.

The ENC controls are conservatively editable only with an observed vector chart,
an initialized S-52 library and a display category other than Base. A raster
chart's text and soundings are image content, unaffected by these controls.
AIS visibility remains independent of reception, lists and alarm calculation.
Orientation is the actual selected upstream mode, not evidence of valid heading
or course data. Existing source-health presentation remains necessary.

## Verified upstream behavior

All paths here are relative to the pinned OpenCPN root.

| Control | Read / existing command | Evidence |
|---|---|---|
| AIS vessels | `GetShowAIS` / `MyFrame::ToggleAISDisplay` | `gui/src/ocpn_frame.cpp:3545`; menu update and refresh |
| Symbol labels (ENC text) | `GetShowENCText` / `MyFrame::ToggleENCText` | `gui/src/ocpn_frame.cpp:3434`; menu update and viewport reload |
| Depth soundings | `GetShowENCDepth` / `MyFrame::ToggleSoundings` | `gui/src/ocpn_frame.cpp:3460`; menu update and viewport reload |
| Orientation | `GetUpMode` / `MyFrame::SetUpMode` | `gui/src/ocpn_frame.cpp:3421`; `gui/src/chcanv.cpp:3426–3465` |
| Chart family | Existing single chart or bounded current quilt-reference entry | `gui/src/chcanv.cpp:14212–14222`; no loading/stack rebuild |

`gui/src/chcanv.cpp:14423–14443` reapplies canvas-owned S-52 state. The new
boundary does not write shared PLIB fields. `libs/s52plib/src/s52plib.cpp:2324`
gates all text first, then the independent navaid/light text preferences.
`gui/src/CanvasOptions.cpp:426–455` establishes ENC/Base-category restrictions.
`gui/src/navutil.cpp:2025–2049` persists the existing canvas preferences through
the normal OpenCPN configuration lifecycle; command success does not promise an
immediate disk flush. Online AIS already checks `GetShowAIS` for both painting
and hit testing in OpenNav `src/integration/OnlineAisOverlay.cpp:54,99`.

## Unavailable controls preserved

- Chart symbols: no equivalent all-symbols switch; do not replace it with
  Base/Standard/All display category or hide navigation objects.
- Depth contours: retain upstream safety-contour presentation.
  `libs/s52plib/src/s52cnsy.cpp:709–721,838–844` requires/promotes the safety
  contour to DISPLAYBASE. `AddObjNoshow("DEPCNT")` would run before category
  protection (`s52plib.cpp:9654–9668`) and is deliberately not used.
- Route corridor: the existing provider reports unavailable. The canvas
  `ToggleLookahead` offsets ownship; it is not corridor coverage or rendering.
- Wind vectors: prototype line 369 draws fixed decorative arrows. Vessel wind
  observations do not establish a spatial vector field.
- Radar overlay: the current `UnavailableRadar` has no receive/overlay/control
  capability. No synthetic echoes, scanner commands or plugin inference exist
  in this boundary.

These fields expose no mutation callback. No chart-database provider, navigation
calculation, chart/style switching, synthetic data or new settings store is added.

## Focused local evidence and limits

Both changed integration translation units compiled with the existing local
`build/xnav-linux/build.ninja` flags, dependencies and generated headers, using
this isolated worktree's application/integration headers and sources. Output
objects were written outside the shared build. The first compile identified the
upstream chart-family getter's `int` return; the corrected objects both passed.
No full OpenCPN build or executable relink was performed.

A temporary C++ harness compiled the exact new implementation block with host
doubles and the actual application contract header: **100 assertions passed**.
Checks covered off-thread rejection before frame access, missing canvas,
owned-copy independence, idempotent desired-state writes, refused mutation
readback, every orientation plus invalid input, Base/raster/no-PLIB gating,
bounded and busy/invalid quilt-reference observation, unsupported layers and
canvas loss during a command. These are boundary checks, not native rendering
or real-host behavior evidence. `git diff --check` passed.

Native Windows integration, real ENC/raster presentation, shared Legacy
preferences, supported renderer behavior and boat-display acceptance remain
required gates. No acceptance of those gates is implied by the local checks.
