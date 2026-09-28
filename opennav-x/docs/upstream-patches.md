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

### Online AIS settings and native drawer boundary

The prototype target drawer now reports its actual list/target/settings view
and returns to the chart only after successful identity/freshness validation
and the upstream chart-position action. The existing owned AIS copy preserves
the upstream report observation time in its summary as well as each field;
UI reads cannot refresh report age. Neither change adds an upstream hook.
The existing settings-reconfiguration hook now restores OpenNavHorizon along
with the two side panes; it had been omitted when the prototype introduced the
timeline. The object regression checks exact navigation composition after both
hidden-page and already-navigation reconfiguration, in addition to coastline.

The application bridge reads the already-computed `ViewPort::GetBBox()` and
`IsValid()` on its normal application-thread tick. Pinned `SetBoxes()` publishes
ordered, possibly unwrapped longitudes; tested `AisViewport` normalization
copies those values without refreshing OpenCPN navigation. Selecting an online
position revalidates the owned aggregate before the existing
`MyFrame::JumpToPosition` chart action. No target is inserted into the upstream
AIS decoder, route model or CPA calculation. SmartNav/alarms retain onboard-only
input. This settings/drawer increment adds no direct upstream patch; the
separate online overlay remains pending.

### Beta 2 chart presentation boundary

The integration bridge reads `ChartCanvas::GetUpMode()` for the XNav orientation
label and reuses its existing `MyFrame::SetUpMode` human action. It hides the
native compass/GPS widget with per-canvas `SetShowGPSCompassWindow(false)` only
while XNav owns the frame, also after deferred initialization or settings
reconfiguration. It never changes the persisted global `g_bShowCompassWin`.
These are calls to existing pinned APIs, with no new upstream patch. Legacy and
Safe continue through normal startup. Mode-cycle/chart-content checks and native
DPI interaction review cover the presentation change; boat review is pending.

### Beta 2 saved plugin workspace restoration

The XNav shell is attached before OpenCPN's deferred plugin initialization.
The pinned `MyFrame::OnInitTimer` validates saved AUI perspectives by requiring
every current pane name to be present. XNav deliberately removes its temporary
panes before saving; their presence during startup therefore made the upstream
validation skip the entire saved plugin layout. The corresponding locale
reload contains the same validation/load sequence.

Both boundaries now exclude only actual Shell-owned window pointers from that
completeness check. All chart and plugin panes retain the upstream check. When
the same manager loads the saved perspective, Shell copies its own pane infos,
calls the existing wxAUI loader unchanged, and restores only those temporary
infos with `SafeSet`. This matters because wxAUI first hides/docks every current
pane. Active product-page chart visibility is also retained until that page
closes. A different manager and all Legacy/Safe calls use normal wxAUI loading.
No product code parses or merges perspective strings, reparents canvases,
changes plugin configuration, or loads extra plugins.

Tests seed an actual Dashboard window with nondefault floating position, size
and dock proportion and check its runtime and saved state through
XNav → Legacy → XNav, with the XNav rail visible and temporary pane names absent
from the saved workspace. The isolated object regression verifies that a
foreign manager with a pane named `OpenNavTop` receives ordinary behavior.
Native Windows qualification remains required. Merge risk is medium: recheck
the two upstream completeness/load locations and wxAUI semantics when rebasing.

### Beta 2 software Course-up repaint loop

The replacement for native candidate `12100a74` changes only the software
basemap block in `ChartCanvas::OnPaint`. The pinned implementation temporarily
called `SetVPRotation(skew)` and then restored the live canvas rotation. Both
calls enter `SetViewPoint`; `Quilt::IsQuiltDelta` treats the rotation difference
as a change and the setter recomposes and calls `Refresh(false)`. A rotated
paint therefore schedules another paint. Windows processes paint messages
before timer messages, matching the retained native trace: the orientation
callback and its complete Shell update return, then no timer updates arrive.

The block now copies the viewport, sets its background rotation to the same
skew, calls the pinned `ViewPort::SetBoxes`, and passes that copy to the existing
basemap renderer. Physical pixel dimensions, center, scale, projection and
Mercator override remain those of the original viewport. It deliberately does
not substitute the expanded `svp` bitmap dimensions. The live canvas, quilt,
cursor/plugin notifications and navigation state are untouched during this
background drawing operation. The pristine pinned submodule is unchanged;
the reviewed integration patch provides this correction in all integrated
interface modes, including Legacy.

The actual pointer workflow retains its North→Course→North interaction and
requires at least four further Shell updates plus a newer LIVE GPS observation
while Course-up remains selected. A Course-up screenshot is now retained as
well as the restored North-up view. The Linux integrated build and all seven
workflow groups pass locally. This regression had failed twice on native
Windows before the fix; replacement native execution and rotated coastline
review remain mandatory. See
[the native trace review](design/reviews/beta2-native-12100a74-course-up.md).

## First dual-mode integration (accepted development slice)

`patches/opencpn-5.12.4-xnav.patch` is applied only to the disposable
`build/integration-source` worktree. The pinned submodule remains pristine.
`tools/prepare-integration.py` refuses a different revision or unexpected edits.

| File | Hook and purpose | Regression / merge risk |
| --- | --- | --- |
| CMakeLists.txt | Optional OPENNAV_ROOT adds separately maintained modules | Default pristine build; low risk |
| gui/src/ocpn_app.cpp | CLI, selection after config load, attach shell, restart after cleanup | Mode precedence and shared profile; medium lifecycle risk |
| gui/src/ocpn_frame.cpp | Hide native chrome only in XNav, Legacy switch menu, detach panes before close; exclude owned temporary panes from persistent plugin-workspace validation and preserve them around loading | Close veto, restart, actual Dashboard workspace, foreign manager isolation; medium risk |
| gui/src/toolbar.cpp | Suppress only main stock toolbar rendering and mouse handling in XNav | Legacy toolbar and plugin tools; medium risk |
| gui/src/chcanv.cpp | Skip main MUI chrome in XNav; software basemap painting uses a copied viewport instead of changing the canvas rotation | Chart interaction, rotated coastline rendering and Legacy controls; medium rendering risk |
| gui/src/routeman_gui.cpp | Alpha: suppress native active-leg console show only in XNav | Real active-route widget assertion, original Legacy/Safe callback retained; low presentation risk |
| gui/src/canvasMenu.cpp | Context-menu fallback for mode switch | Menu access with hidden menu bar; low risk |
| model/src/plugin_loader.cpp | Keep plugins inactive in Safe Mode without persisting disabled preferences | Enabled Dashboard fixture through Safe and normal restart; low risk |
| gui/src/pluginmanager.cpp | Avoid saving temporary Safe Mode plugin states as normal preferences | Same shared-profile fixture; low risk |
| model/include/model/comm_drv_n2k_net.h | Beta: read-only detected format, monotonic connection generation/time | No output/discovery getters; low API risk |
| model/src/comm_drv_n2k_net.cpp | Beta: advance connection provenance at socket replacement/connect/loss/close boundaries | Same-driver reconnect loopback; medium event-order risk |
| model/src/comm_drv_signalk_net.cpp | Beta: bounded UTF-8/JSON preflight before recursive parser; type/length/control validation before handshake GetString access | Actual malformed/valid WebSocket input, all modes retain valid-message path; low decoder-entry merge risk |
| model/src/ser_ports.cpp | Beta discovery lifetime repair: release udev references, Windows SetupAPI lists and query registry keys; guard unavailable discovery | Actual API ownership/failure tests, allocation profile and elapsed endurance; low catalog-lifetime risk |
| model/src/garmin_protocol_mgr.cpp | Beta: release the SetupAPI list in read-only IsGarminPlugged on every path; failed discovery returns false and failed detail allocation is guarded | Repeated actual native queries; no USB start/command path changed |

Public plugin API 1.20 does not provide ownership of application startup,
main-frame chrome or shutdown. Narrow core hooks are necessary; zoom, follow,
theme, chart, route and configuration behavior reuse the existing implementation.
All GUI hooks are guarded by OPENNAV_X. No device command logic is added.
The Signal K guard is also OPENNAV_X scoped, with the source-only model definition
and include path in `OpenCPN.cmake`. It protects all integrated modes, leaves the
pristine build untouched and does not replace upstream parsing or publication.
See [Beta robustness](beta-robustness.md).
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

The subsequent boat-propulsion adapter adds no OpenCPN patch. It observes vendor
61184 only behind an explicit interface/NAME binding and reuses standard marine
decoders for all standard fields. Producer expiry changes are isolated in
`hardware/leaf-bridge/`, applied to the separately hash-pinned boat firmware by
`prepare-boat-firmware.py`; they never patch installed PC software or flash a board.

## Beta AIS selection frame

The next Beta change adds `gui/src/ais.cpp` as the twelfth production patch file.
It guards one call to the existing `TargetFrame` rendering function with
`OPENNAV_X` and a read-only `opennav::IsAisSelected(MMSI)` predicate. The existing
Legacy query/alert highlighting remains unchanged. Selection retains one owned
AIS snapshot and expires on target loss, deletion, ambiguity, age, out-of-order
replacement or Demo/replay. No decoder state, CPA/TCPA, target geometry or
COLREG interpretation changes. The plugin API offers overlays, not this existing
AIS symbol-selection predicate; reusing upstream framing avoids a second symbol
renderer. Merge risk is low and localized to the existing query-highlight block.

Validation: `ais_selection_lifetime`, existing integrated AIS contracts and the
actual AIS card → chart workflow with native/Linux captures. Exact replacement
acceptance passed at `a3e6e08`; see [Beta 1 acceptance](evidence/beta1-a3e6e08-accepted.json).

Beta night rendering uses the existing deferred-initialization hook and public
`ShapeBaseChartSet::SetBasemapLandColor` with the pinned `GSHHSChart` palette.
No new patched source is needed. The software land color now follows the same
upstream dusk/night multiplier as water in XNav; Legacy and ENC rules are intact.

Native run `36114659033` rejected patch parsing at a blank AIS context line
converted to CRLF by checkout. Patch files now have LF attributes and proper
unified-diff context prefixes. Preparation feeds the identical LF-normalized
stream to check/apply/temporary-index verification on every platform. Exact
pinned revision and reviewed-worktree comparison remain mandatory; no ignored
hunks or weakened source checks are introduced.

## Beta serial-discovery lifetime repair

A four-minute allocation profile of the real integrated Linux process found
14.31 MB retained in libudev scan allocations, with stacks through
`EnumerateSerialPorts` / `LoadSerialPorts`. Inspection of the pinned
`model/src/ser_ports.cpp` confirmed no unref for `udev_new()` or
`udev_enumerate_new()`; repeated background connection discovery retained them.
The reviewed patch uses local unique ownership for context, enumeration and
device references, handles failed creation and disappearing device nodes, and
keeps the existing catalog/filter/link semantics. The initial Linux repair changes
no boat transport or public API; the Windows discovery follow-up is below. A plugin/public getter cannot fix this lifetime
inside upstream discovery. The pristine source stays unchanged.

Three Linux/libudev-only integration tests link wrappers around the real library
entry points, require each owned reference released across eight real discovery
calls, and inject failed context/enumeration creation. They do not replace the
catalog with invented devices. Allocation-profile comparison and the full
three-hour application gate remain necessary to qualify the observed growth.
The patch is independent of XNav presentation and also protects integrated
Legacy/Safe. The Linux merge boundary is the two small discovery functions.

Windows source inspection then found unclosed `SetupDiGetClassDevs` lists and
`SetupDiOpenDevRegKey` handles in the same discovery function, plus an unclosed
list in its read-only `GarminProtocolHandler::IsGarminPlugged` call. Scoped
ownership now releases these, including failed/absent-device paths. Invalid
Garmin enumeration returns false (the upstream INVALID_HANDLE_VALUE converted
to true), and detail-size/allocation failure cannot dereference a null buffer.
Garmin USB startup/output is unchanged. The source finding is not represented
as a measured three-hour result.

Five native integration tests compile the exact reviewed serial-discovery source
with test-only Win32 API spies: real device-list lifetime, invalid-list failure,
real temporary HKCU query handles with present/missing values, and repeated
actual Garmin presence queries with bounded process handles. The application
has no spies or test hooks. Temporary registry data is isolated and cleaned.
The same-commit Windows build and real-process endurance remain mandatory.
[SetupAPI ownership](https://learn.microsoft.com/en-us/windows/win32/api/setupapi/nf-setupapi-setupdicreatedeviceinfolist),
[registry-key lifetime](https://learn.microsoft.com/en-us/windows-hardware/drivers/install/accessing-custom-device-properties).

## Final Beta 1 qualification

All documented hooks at `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a` passed
106 Linux and 98 native Windows integrated cases, transport/UI/chart/mode gates
and actual three-hour endurance on each platform. [Beta 1 acceptance](evidence/beta1-a3e6e08-accepted.json) identifies the
exact artifacts and reviewed executable. Five Windows actual-API discovery
ownership cases and three Linux udev reference cases are included in those
counts. No further upstream code change is introduced by the acceptance-only
documentation follow-up. Physical hardware and Windows GPU validation remain
separate; the pristine pinned upstream checkout is unchanged.

## Beta 2 settings and chart context hooks

Additional XNav-only hooks are under development. At the end of normal
`MyFrame::ScheduleReconfigAndSettingsReload`, `AfterSettingsReconfigured`
reconciles shell visibility after Options detaches/re-registers native canvas
panes. This avoids competing center panes and hidden charts without restarting
or replacing OpenCPN's chart/configuration processing. The object scenario
requires a visible usable chart after that exact reconfiguration path.

At the beginning of `ChartCanvas::InvokeCanvasMenu`, ordinary XNav position,
waypoint, route and AIS contexts reuse upstream hit-testing/conversion and
defer copied identities/coordinates to owned XNav cards. No raw pointer crosses
the integration boundary. Measurement, route creation and unhandled advanced
contexts retain native behavior; Legacy/Safe retain original menus.

XNav's `ChartCanvas::OnKillFocus` leaves route completion to its explicit
Finish/Cancel controls. Pinned mouse handling can reset the existing
`m_FinishRouteOnKillFocus` flag after waypoint dialogs, so setting that flag
once at Start Route was insufficient. The guard prevents a touch on Undo or
Finish from committing a draft before the action callback; Legacy focus-loss
behavior is unchanged.

The patched command-line help/startup-option list now omits Demo for production
builds, matching the compile-time fixture separation. No runtime profile or
environment setting can enable missing generators. The reviewed worktree check
passes; exact-release native/boat validation is still required. See
[Beta 2 integration inspection](beta2-integration-feedback.md).


## Beta 2 bundled Dashboard presentation

A private additive bridge in `DashboardPresentationApi.h` is enabled only for the
source-pinned bundled Dashboard by `OPENNAV_DASHBOARD_PLUGIN`. The plugin registers
actual pane windows after AddPane and unregisters at destruction. RAII scopes
bracket preferences, toolbar visibility, UpdateAuiStatus, ApplyConfig, SaveConfig,
ShowDashboard and orientation changes. Existing plugin API/vtables and unknown
plugins are unchanged; a pristine upstream build does not enable these calls.

Host-side application-thread weak references preserve complete original pane
state while hiding these panes in XNav. Deferred initialization enables suppression
after normal perspective loading; later owned-manager perspective loads use the
same layout scope. Close restores state before the shell and upstream persistence.
No sensor, route or hardware behavior changes. Legacy follows normal Dashboard
behavior. [Rationale, test changes and pending Windows/boat gate](design/reviews/beta2-plugin-workspace.md).

### Supplemental internet AIS: opt-in IXWebSocket receive limits

`opencpn-5.12.4-ais-transport.patch` extends the pinned bundled IXWebSocket
client with `setUntrustedClientLimits`. Existing callers retain zero/default
limits and their existing behavior. The AIS internet client will opt into a
64KiB wire/decompressed-message limit, 256-fragment bound, bounded receive
buffer, 4KiB HTTP status line, 16KiB aggregate headers and redirect refusal.
The inflater checks its output budget before appending. Large advertised frame
sizes are rejected before allocation, including on the supported Win32 ABI.

Files changed are exclusively in `libs/IXWebSocket/ixwebsocket`: WebSocket and
Transport configuration, Handshake/HTTP headers/Socket line-reading limits,
and PerMessageDeflate/Codec output bounds. No plugin API, OpenCPN marine-data
processing, chart state or navigation behavior changes. The public pinned
client had no receive-size API, bounded inflater or redirect policy; a JSON
parser size check alone would run after these allocations. Reusing its existing
TLS and compression implementation avoids a second network stack. This is a
moderate library merge risk; preserve defaults and re-run the local raw-server
suite after any upstream update.

`tests/ais_transport/network_tests.py` uses only a disposable loopback TLS server
and generated test certificate. Eighteen scenarios cover valid binary,
compression, exact boundary, oversized messages, inflation bomb, fragmented
reports, empty-fragment flood, 32/64-bit advertised overflow, continuous traffic,
HTTP bounds, redirects, untrusted certificate and hostname mismatch. Linux
passes all 18 against the patched pinned library; native Windows qualification
is pending. Native `d92e187` caught a fragmented receive stall: stopping at the
bounded dispatch buffer can leave plaintext inside OpenSSL after the OS socket
empties. A bounded client now resumes nonblocking receive after dispatch before
polling the socket again. Valid back-to-back reports across that boundary are
also tested. The unbounded default still follows the original poll path. The
dedicated client executable is never installed.

### XNav S-52 and basemap presentation selection

`opencpn-5.12.4-chart-presentation.patch` adds an optional
`allowCwdOverride=true` argument through `s52plib` construction/load and
`ChartSymbols::LoadConfigFile`. The unchanged default preserves existing
callers. XNav opts out to prevent a working-directory XML from shadowing its
verified resources or explicit Standard fallback. `LoadS57` delegates only its
initial construction to the integration boundary. CSV registrars, safety-depth
selection, lookups, conditional symbols and chart ownership remain upstream.

The same constructor has `useS52DefaultTextColor=false`, enabled only after
XNav's resource verification succeeds. In `RenderText`'s cached GL glyph branch,
it applies the existing software branch's default-black-to-S52-LUP-color rule.
Explicit user chart-text colors remain honored. Standard, fallback, Legacy and
Safe retain the false default; the special-character/rotated texture path is
unchanged. Actual software/GL Night captures exposed this pinned branch
difference. Pixel review and native/boat GL gates remain necessary.

Two small GUI hooks let `GSHHSChart::SetColorScheme` and the `LANDBACK`/`BLUEBACK`
background colors use XNav's verified palette. They return normal upstream
behavior in Legacy/Safe/Standard. No chart objects or sensor state are changed.
The existing palette setters were inspected: GSHHS otherwise hard-codes a
separate olive/blue palette while shapefile/GL paths use global background
colors. The constructor hook is necessary because replacing the library after
charts retain lookup pointers would be unsafe. Merge risk is localized to these
initialization/color boundaries and the loader's new defaulted argument.

Tests: deterministic resource generation and protected source hashes; integrated
build/tests; real ENC XNav/Standard Day/Dusk/Night and mode-cycle content capture;
OpenGL/software and native Windows/boat validation. Resource checks and the
Linux integrated build/110 tests pass. Initial Linux software ENC captures
retain actual upstream quilt identity and detail through all three palettes,
but text/land/overlay mismatches remain. Rendering acceptance is pending.

### Supplemental online AIS paint and selection

The existing `xnav.patch` adds one guarded call at `gui/src/ais.cpp::AISDraw`
before local decoder enumeration. Only XNav's owned supplemental overlay draws;
Legacy/Safe and all original local AIS symbols/calculations retain their path.
Both software `ChartCanvas::DrawOverlayObjects` / `UpdateAIS` and
`glChartCanvas::DrawFloatingOverlayObjects` already use this upstream boundary.
The overlay honors each canvas's existing AIS visibility and projection. It
does not insert targets, calculate collisions, refresh observations or invoke
OpenCPN navigation/output methods from paint.

The existing XNav `ChartCanvas::InvokeCanvasMenu` hook adds an online hit test
only within its unknown-object branch. Upstream AIS, route/waypoint selection,
route editing and measurement precedence are preserved. OpenNav receives copied
MMSI only after the event stack unwinds. The integration keeps no canvas pointer
in the retained chart marks. New numerical/identity/aging/precedence tests are
portable; native/software/GL/populated-target and boat capture gates remain open.
