# Fixed display review actions

These optional boat-development actions extend `review-window.ps1`. They use the
existing exact installed generation, fixture-free build, audited launch,
process/creation/session and private screenshot checks. They neither launch an
application nor change Windows display settings, connections, navigation objects
or equipment controls. Complete native tool qualification before boat use.

Perform one action, inspect its actual before/after images, then choose the next.
The normal path to Display is `Menu` → `Settings` → `Display`. A clipped control
must first be brought into view through the separately requested `PageDown`
action; the tool does not scroll or click speculatively.

| Action | Required visible source control | Effect |
| --- | --- | --- |
| `Display` | Unique `DISPLAY`, a direct child of the exact Settings page | Opens Display settings |
| `ToggleFullscreen` | Unique `Fullscreen / window`, a direct child of the exact Display page | Uses the application's normal fullscreen/window action once |
| `ToggleOrientation` | `North` or `Course` on the chart, sharing the frame's tools pane with `+`, `−`, and `Center`; no product page visible | Uses the normal North/Course chart-orientation action once |
| `CyclePalette` | Day/Dusk/Night control in the top bar beside `Menu` | Cycles the palette once; matching Display-page palette controls are excluded |

Source boundaries are `ProductPanel.cpp` Settings, `ProductSettings.cpp`
Display, and `Shell.cpp` chart/status controls, with the current orientation
caption supplied by `OpenCPNIntegration.cpp`. No caller supplies a caption,
coordinate, key sequence or native command ID. An unsupported orientation,
missing control, duplicate control, unexpected page or modal causes refusal.

Every reviewed button press now rechecks its same handle, process, parent,
caption, class, enabled/visible state and geometry after the synchronous press
callback and before release. Fixed actions also repeat source-scope resolution;
the button must still be unobscured. Replacement, relabeling, movement, ambiguity
or timeout refuses release without retry. An uncertain press requires inspection.
Only actual post-action captures establish the resulting view.

Fullscreen changes only the application window. `Resize1280x800` remains a
separate action and still refuses if that physical rectangle cannot fit the
current monitor work area. These tools do not hide the taskbar, change DPI or
screen resolution, or force another application's focus. Application display
choices can be saved normally in the existing profile on clean exit; later
launch/restart must still satisfy its independent profile audit.

## Qualification

- 159 portable policy/native compilation checks pass, including current source
  captions and fixed page scopes. Existing completed-restart-child review tests
  also pass (132 checks) with the extended display-only action list.
- `tools/test-display-window-native.ps1` defines 15 disposable WinForms cases:
  Settings→Display, fullscreen and return, both North/Course states, top palette
  selection with duplicate Display-page labels, wrong-page refusals, ambiguous
  controls, replacement/relabeling/movement/duplication during press, and a modal.
  Successful cases require exactly one callback; refusal cases require none.
  No OpenCPN, profile, plugin or marine process is loaded. The marker has a fixed
  lifetime and exits normally; the runner never force-terminates it.
- Native Windows execution passed all 15 cases on tooling commit
  `0c751177`, [run 36287451428](https://github.com/ThereptileII/Work/actions/runs/36287451428):
  six successful paths, nine refusals, and no cleanup errors. The existing nine
  mode-window cases also passed. Artifact `10921210420`, SHA-256
  `a52e81489c8d64d0a735678b2f9901b5ff5aeab89c50f623ab0bdb6dceaaf68b`,
  matches the API digest, upload log and downloaded bytes. The entire tooling
  run failed in a separate temporary broker-fixture module import; this result
  qualifies only the completed native window cases.
- Actual installed wx behavior and boat-display acceptance remain separate
  pending gates. Marker-window results do not establish physical fullscreen,
  chart-orientation or touch acceptance.

## Bounded chart pan (replacement tooling qualification pending)

`PanRight` addresses the remaining actual-boat pan review gap. It consumes a
fresh (at most five seconds old), exact-build **INSTALLED PRODUCT** Navigation
diagnostic record with normal OpenCPN data mode and no route creation. Copied
chart geometry must identify one enabled direct-child canvas of the exact
foreground frame; its center must hit that canvas or its OpenGL child. A product
page, modal, other pointer capture/menu, held mouse button/modifier/arrow or
ambiguous native geometry refuses input. It never clicks a chart object.

The sole action sends one target-local Right-arrow down/up to that verified
canvas. At pinned OpenCPN `37fd0cdd`, `ChartCanvas::OnKeyDown` either pans the
viewport or starts normal smooth movement; `OnKeyUp` stops that movement. This
is **screen-right**, not an independently calculated geographic bearing. The
upstream callback first offers key events to subscribed plugins; the retained
bundled Dashboard/GRIB/chart downloader/WMM and reviewed o-charts source do not
subscribe to keyboard events. The existing complete plugin/read-only launch
audit remains mandatory. No route operation, arbitrary key, global input or
equipment command is exposed. Window and process identity are checked again
before releasing the same target. An uncertain result is not retried.

Before/after native image review must establish actual chart movement and
continued content; a successful helper return alone is not pan acceptance. Use
the separately reviewed Center action afterwards only with valid ownship data.
This keyboard-based review does not qualify physical touch dragging.

Local checks: **178** window-review policy/compilation checks, **133** completed
restart-window policy checks and all **17** copied broker dependency checks pass.
The native disposable suite now defines **20** cases: the previous 15 plus
pan key-pair receipt on a canvas and a canvas with a child surface, wrong-page,
wrong-geometry and modal refusals. Successful pan cases require exactly one
down/up pair; refusal cases require no event. Native execution and boat use are
still pending; no marker result substitutes for actual OpenCPN chart behavior.
