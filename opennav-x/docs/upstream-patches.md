# Direct OpenCPN Upstream Modifications

## Baseline state — 2026-09-21

No direct upstream modifications. OpenCPN is pinned at
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` (`Release_5.12.4`).
`tools/verify-upstream.py` rejects the wrong commit or modified tracked files
before a pristine build. All policy code and build/validation scripts live
outside the submodule. Build-tree generated files are not source patches.

This file must list every modification made directly to upstream OpenCPN source.

For each change record:

- OpenNav commit
- upstream file(s)
- reason
- smallest possible description of the hook/change
- whether an upstream/public API alternative was investigated
- regression tests covering the change
- merge/rebase risk

Do not leave undocumented direct OpenCPN modifications.

## First dual-mode integration (accepted development slice)

`patches/opencpn-5.12.4-xnav.patch` is applied only to the disposable
`build/integration-source` worktree. The pinned submodule remains pristine.
`tools/prepare-integration.py` refuses a different revision or unexpected edits.

| File | Hook and purpose | Regression / merge risk |
| --- | --- | --- |
| CMakeLists.txt | Optional OPENNAV_ROOT adds separately maintained modules | Default pristine build; low risk |
| gui/src/ocpn_app.cpp | CLI, selection after config load, attach shell, restart after cleanup | Mode precedence and shared profile; medium lifecycle risk |
| gui/src/ocpn_frame.cpp | Hide native chrome only in XNav, Legacy switch menu, detach panes before close | Close veto, restart, AUI persistence; medium risk |
| gui/src/toolbar.cpp | Suppress only main stock toolbar rendering and mouse handling in XNav | Legacy toolbar and plugin tools; medium risk |
| gui/src/chcanv.cpp | Skip main MUI chrome in XNav; chart logic unchanged | Chart interaction and Legacy controls; low risk |
| gui/src/routeman_gui.cpp | Alpha: suppress native active-leg console show only in XNav | Real active-route widget assertion, original Legacy/Safe callback retained; low presentation risk |
| gui/src/canvasMenu.cpp | Context-menu fallback for mode switch | Menu access with hidden menu bar; low risk |
| model/src/plugin_loader.cpp | Keep plugins inactive in Safe Mode without persisting disabled preferences | Enabled Dashboard fixture through Safe and normal restart; low risk |
| gui/src/pluginmanager.cpp | Avoid saving temporary Safe Mode plugin states as normal preferences | Same shared-profile fixture; low risk |
| model/include/model/comm_drv_n2k_net.h | Beta: read-only detected format, monotonic connection generation/time | No output/discovery getters; low API risk |
| model/src/comm_drv_n2k_net.cpp | Beta: advance connection provenance at socket replacement/connect/loss/close boundaries | Same-driver reconnect loopback; medium event-order risk |

Public plugin API 1.20 does not provide ownership of application startup,
main-frame chrome or shutdown. Narrow core hooks are necessary; zoom, follow,
theme, chart, route and configuration behavior reuse the existing implementation.
All GUI hooks are guarded by OPENNAV_X. No device command logic is added.
The expanded native Windows mode-cycle and visual review passed at `c5a0fd0`
(see baseline.md), including the shutdown and Safe Mode preference fixes below.
Selected-navigation integration passed at `bc0af30`. These are development-slice
gates, not production release acceptance.

### Shutdown timer guard

The frame timer hook also returns after ProcessQuitFlag closes the frame. A
pristine Linux core showed APConsole::IsShown reached later in the same timer
callback after cleanup. This one guard affects no normal navigation work and
runs only in the integrated build. Regression: normal close, mode restart and
IPC quit must all exit without a crash. Pristine source retains the original
behavior for comparison. Individual integrated Linux IPC-close checks pass in
all three modes; the shared-profile cycle also checks IPC close after restart.

OpenNav mode requests now queue the close with `CallAfter`. A Linux core showed
that immediate close from a canvas popup deleted the canvas before
`InvokeCanvasMenu` finished unbinding its handlers. This fix lives in the
OpenNav bridge; it adds no further upstream edits. The Linux cycle uses this
context-menu path, while Windows exercises the Legacy menu-bar path.

### Safe Mode plugin preferences

Upstream's loader both disables a bundled plugin in memory and writes `bEnabled`
false; the GUI plugin manager can also save that temporary state. The integration
guards those two writes in Safe Mode. Loading and initialization remain disabled
as upstream requires, and normal-mode changes still save normally. The model
definition is scoped to `plugin_loader.cpp` from OpenNav's CMake integration.
This prevents even an intermediate flush or abnormal Safe Mode exit from
permanently disabling the user's bundled plugins. A bundled Dashboard plugin
enabled in the fixture must remain enabled after every mode transition.

## Upstream regression-test repairs (separate patch)

`opencpn-5.12.4-regression-tests.patch` changes tests only:

- `test/ipc-srv-tests.cpp`: own the callback instead of capturing a constructor
  parameter by reference; use atomic result flags; allow up to 10 seconds for
  process startup instead of 100 ms; request event-loop exit on the main thread.
  A separately compiled diagnostic version with capture/deadline fixes completed
  all four IPC commands in 456 ms where pristine failed or hung.
- `test/n2k_tests.cpp`: use registry Deactivate for the registry-removal assertion.
  A driver's Close closes transport but does not relinquish registry ownership.
  No production driver behavior is changed.
- `test/CMakeLists.txt`: discover compiled gtest cases at test time instead of
  registering #ifdef-disabled tests by scanning source. This avoids false passes
  for test names with no compiled matching case.

These are independently reviewable from the GUI hooks. Pristine tests retain
original behavior and logs. Integrated Linux regressions must pass the repaired
suite; new or unrelated failures are not covered by a baseline exception.

## Remaining-route observer (accepted development slice)

Accepted code/test commit: `954b4505e18e9128dc02e75cf05d0c02bdbad188`
(local `a7f2b33`), with both platform gates and native review recorded in
[status.md](status.md).

The frame timer has two additional `OPENNAV_X`-guarded calls immediately before
and after its existing `RoutemanGui::UpdateProgress()` call. They copy and validate
state; they do not trigger, reorder or replace navigation processing. No model
route/autopilot implementation is patched. Merge risk is the progress-call
location/order and upstream route lifetime semantics. Coverage: portable route
contracts, real upstream model/antimeridian tests, and the opt-in normal-timer
route scenario on Linux and native Windows. See [route contract](route-progress-contract.md).

Inspection found no public plugin snapshot that provides coherent active range,
ordered stored legs and selected-position provenance after a normal progress
pass. Existing model getters supply the values; the narrow timer boundary is
needed to reject waypoint advance and reentrant edits without triggering progress
or autopilot output. The hook adds no navigation command or independent geometry.

The integration CMake hook attaches model-bound tests after the upstream test
target is declared, using CMake's deferred call facility. Test sources remain
outside upstream. The optional scenario driver is compiled only when explicitly
enabled with upstream testing; production/default builds omit it.

## Portable Developer Preview isolation

Three further `OPENNAV_X`-guarded hooks in `gui/src/ocpn_app.cpp` apply only when
`IsPortablePreview()` has validated the package marker and local profile:

- Preserve forced portable mode when upstream parses `-p` after the early hook.
- Treat preview launch as explicit startup, avoiding forwarding to an existing
  normal OpenCPN instance.
- Skip LAN REST server/mDNS setup for the isolated preview.

Inspection used `MyApp::OnCmdLineParsed`, the subsequent startup/instance path,
`OCPNPlatform::GetPrivateDataDir`, and the LAN service setup in `OnInit`.
Profile override is copied before initialization; no normal installation is
patched. Outside the package the existing behavior remains unchanged. Merge
risk is startup ordering. Portable contract tests, direct-EXE and launcher
smoke tests with an external-profile canary cover the boundary on native Windows.
No route-progress/autopilot implementation changes were needed for the preview.

The preview integration also sets the working directory to the private profile
before initialization. Inspection of `AbstractPlatform::NormalizePath` and the
startup tide-data defaults showed that portable relative paths use that base.
An extracted native candidate exposed missing tide files when started from the
package root; no navigation or resource-loading implementation was changed.

Preview content pages use the existing AUI manager's center-pane support.
Inspection of `MyFrame::CreateCanvasLayout` identifies `ChartCanvas` and
`ChartCanvas2`; `MyFrame::ODoSetSize` updates that manager on resize. Integration
passes only those pane names into the shell. The shell restores the original
visibility before the existing `PrepareClose` hook lets upstream persist its
perspective. No new upstream hook or canvas reparenting is needed. This fixes
native Windows covering unmanaged overlay pages during resize.

Packaging also follows `PluginPaths::InitWindowsPaths` and
`AbstractPlatform::GetPluginDataPath`: portable plugin binaries and resources
must be available in `PrivateDataDir/plugins`. The installed `app/plugins`
directory alone does not make them discoverable in portable mode. Supplying
the bundled copies in `profile/plugins` needs no upstream loading change.
Native package tests verify initialization/unloading and Safe suppression.

## Portable chart restoration after mode restart

The user's 2026-09-24 Windows test exposed a lost background coastline after
Legacy → XNav. Inspection traced it to `MyConfig::UpdateSettings` in `navutil.cpp`
calling `AbstractPlatform::NormalizePath` for an empty `gWorldShapefileLocation`.
In portable mode this serializes as `./` (Windows `.\`). On the next launch,
`ShapeBaseChartSet::Reset` treats that as an explicit profile-directory location
instead of selecting the bundled `basemap_shp` default.

The repair uses the existing `SelectMode` hook immediately after `LoadMyConfig`
and before canvas creation. For a validated portable preview only, it resolves
an empty default to existing bundled shapefiles. It repairs the old dot-directory
value only when the profile contains no shapefile basemap. Custom paths, including
missing custom paths, remain unchanged. OpenCPN's normal save logic then stores
the nonempty path relative to the portable profile. No additional upstream patch,
renderer, chart-database, chart-directory or navigation change is needed.

Regression coverage adds portable resource policy cases, real coastline pixel
checks through the controlled XNav/Legacy/XNav/Safe cycle, and native direct
startup with the old broken setting. Linux preview smoke now uses portable
resource layout as Windows does; the existing non-portable mode/input regressions
remain separate. The old executable fails the new rendering check on both the
Legacy and returned-XNav captures. Native acceptance is recorded in status.md.

## Alpha navigation context and anchor observation

Four narrow additions stay inside the existing reviewed GUI patch paths:

- Immediately after normal `MyFrame::ProcessAnchorWatch`, copy the anchor result
  into OpenNav values. Reads never run anchor-watch processing.
- `ChartCanvas::ShowMarkPropertiesDialog` and `ShowRoutePropertiesDialog` offer
  an XNav context-card dispatch before opening the normal legacy dialog.
- `ShowAISTargetQueryDialog` similarly offers an XNav target card.

Each dispatch copies GUID/MMSI and defers UI work until the upstream event stack
unwinds. If XNav is not active, the original path remains unchanged. No model,
storage, AIS calculation, autopilot output or plugin ABI method changes. See
[navigation object contract](navigation-objects-contract.md). Normal frame/canvas
commands are used through the integration action service rather than new hooks
for every toolbar control.

## Alpha startup recovery

The existing `ocpn_app.cpp` patch adds one guarded call to
`CheckStartupRecovery()` after the normal single-instance check and before
`safe_mode::check_last_start()`. A blocked XNav startup calls upstream
`safe_mode::set_mode(true)` before plugin/GL setup. Normal OpenCPN Safe Restart
and `startcheck.dat` behavior remain intact. The existing SelectMode/Attach,
normal frame-processing and close boundaries account for startup health and
clean exit; no new navigation processing is triggered. See
[startup recovery contract](startup-recovery.md).

Alpha chart/plugin diagnostics add no direct upstream patch. GUI-thread copies
read the pinned viewport, quilt index vector, chart table and plugin loader
records; they do not open/recompose charts or invoke plugin methods. The XNav
plugin entry uses upstream's built-in initial-page mechanism (Plugins index 5
in the inspected pinned `options::CreateControls`). Chart route editing reuses
OpenCPN's normal point dragging. See [chart/plugin/performance gate](chart-plugin-performance-validation.md).

## Alpha installer loader check

The existing `ocpn_app.cpp` patch adds a guarded explicit self-test exit path.
Command parsing recognizes `--opennav-self-test` before portable/profile logic;
`OnCmdLineParsed` returns before upstream argument side effects. Immediately
after `wxApp::OnInit`, `OnInit` runs the loader/resource report and uses the
existing `m_exitcode` / `OnRun` mechanism. `OnExit` bypasses normal teardown only
for this mode because platform/profile services were never initialized. Normal
starts follow the existing code. This avoids using a full OpenCPN startup as an
installer probe that could modify the shared profile. See
[transaction contract](installer-transaction-contract.md).

## Recovery notice after deferred startup

A guarded call at the end of `MyFrame::OnInitTimer`, only when
`g_bDeferredInitDone` is true, schedules `AfterDeferredInitialization()`.
The informational recovery notice is queued once after upstream focus, frame
raise, chart finalization and canvas refresh work finishes. Initial Safe
selection remains before plugin/GL setup. This replaces the earlier Attach-time
modal, which overlapped deferred startup and failed native dismissal. No
navigation processing is triggered and ordinary Legacy startup is unaffected.
The real-process gate asserts notice ordering, actual dismissal, enabled parent,
retained chart/data and three separate native recovery cycles.
[Failure and replacement](evidence/windows-recovery-bc892a7-startup-order.json).

## Stable installed resource defaults

A guarded `InitializeResourceDefaults()` call in `MyApp::OnInit`, after locale
initialization and immediately before the pinned GSHHS/tide/AIS-sound default
block, fills only unset selections from an installer-owned stock locator.
These defaults refer to the untouched original installation, not a removable
Alpha generation. `navutil.cpp` retains normal serialization and
`TCMgr::LoadDataSources` retains normal harmonic decoding and warnings.
Existing user paths are never replaced or supplemented. Installed XNav/Legacy/
Safe share this boundary; unmarked and portable starts remain unchanged.

The original post-config boundary is too early for Unicode tide paths:
`wxString::ToStdString` there runs before `ChangeLocale` and can yield empty
strings. The dedicated hook preserves upstream's locale/conversion ordering.
Because the pinned config loader converts saved tide-source names before locale
setup too, installed modes reread only that list after locale initialization,
using the same entry order and duplicate removal. This repairs conversion, not
the selected data: missing/custom paths remain selected and normal warnings
remain enabled. The config group scope is restored without writing it.
The actual Linux application regression loads Unicode harmonic paths across
XNav, Legacy and Safe starts while removing the prior executable generations.
[Native lifetime failure](evidence/installer-resources-2803773-failure.json).

The installed-resource hooks pass the full Linux/native Windows qualification at
`7bc36e426a55926045ea1aece0ebe96ef9417863`, including actual installed mode
returns, resource lifetime, recovery, objects/AIS/anchor and restored stock.
[Qualification evidence](evidence/alpha-installer-7bc36e4-qualification.json).
The final packaged revision and delivery acceptance are recorded in [status](status.md).

## XNav active-leg console suppression

Visual review of `32a6564` found the native "This Leg" console covering the XNav
rail during actual route activation. `RoutemanGui::GetDlgCtx` now returns early
from only its `show_with_fresh_fonts` callback when `opennav::IsXNav()` is true.
The inspected `ConsoleCanvasWin/Frame::ShowWithFreshFonts` path handles only
widget hiding, font/layout, positioning and showing; navigation processing and
outputs are separate and unchanged. Legacy and Safe execute the original path.
There is no public plugin API to replace this main-frame chrome policy.

This adds the ninth production patch file; the pinned checkout remains pristine.
The real GUI `RouteProgressScenario` asserts that the existing `APConsole` is
hidden on each valid first/middle/final, advanced, reversed and reactivated route
publication. It fails on the prior executable behavior and passes only when the
rail remains unobscured. Existing mode, chart and installer tests remain required.
[Finding and red-test evidence](evidence/alpha-console-32a6564-review.json).

The console hook and all preceding hooks also pass the final packaged Alpha
revision `08bc92f`: [same-commit acceptance](evidence/alpha1-08bc92f-accepted.json).

## Beta commissioning and recording integration

The live-input/recording increments add no direct upstream patch files. N2K
acquisition still observes `NavMsgBus`; the portable diagnostics service owns
copied Vessel Data and route snapshots only. The integration supplies an output
preflight using the pinned application's main-thread `CommDriverRegistry` and
read-only `GetAttributes().ioDirection`; output-capable or unknown-direction
marine connections refuse replay. OpenNav pilot enable/command callbacks and
navigation/settings mutation callbacks are independently guarded during replay.
Replay never invokes route processing, autopilot output or a marine send method.
No upstream route, waypoint, driver or chart pointer escapes into the recorder.
Arbitrary third-party plugin transports are not intercepted; offline recordings
should be reviewed in the isolated portable profile.

## Beta manual pilot transport provenance

The pilot increment adds two production patch files (eleven total), the N2K
network header and implementation listed above. Registry notifications alone
cannot identify reconnects inside a retained driver object. Public plugin API
1.20 exposes neither detected wire format nor connection generation/time;
`WriteCommDriverN2K` also discards send success. The bridge therefore performs
short-lived application-thread registry lookup and explicit driver sends only
for human requests, guarded by identity/permission/session/replay boundaries.

The added getters only observe format and monotonic transport provenance.
Connection event ordering is the merge risk; inspect all close/reconnect paths
when rebasing. No navigation or vendor command encoding is added upstream.
`smoke-pilot.py` exercises actual TCP bytes, feedback, timeout and same-object
reconnect in an isolated loopback profile on Linux/native Windows. Existing N2K
identity/loss, recording/replay, mode and route regressions remain mandatory.
Portable tests cover the adapter and settings independently, including command
interpretation by the hash-pinned actual boat firmware parser. None of these
desktop checks claim physical SeaTalk/N2K delivery.
