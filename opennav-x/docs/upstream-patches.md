# Direct OpenCPN Upstream Modifications

## Actual serial pilot boundary — SCRUM-313

The tenth reviewed patch file, `patches/opencpn-5.12.4-pilot-serial.patch`,
changes only `model/src/comm_drv_n2k_serial.cpp` at the pinned 5.12.4 revision.
It removes an outgoing eight-byte payload read which overran a three-byte
ISO address-claim request. Arbitrary transmitted data is no longer interpreted
as a source NAME. Existing transmit notifications retain type `0x94`, an unknown
NAME and null source address; they cannot constitute received physical feedback.
Message/address type, payload length, priority and PGN are checked before use.
The serializer's index/storage now handles the full legal 223-byte payload,
including escaped bytes. No worker means failure; a successful return means
queue acceptance only.

The isolated `tests/pilot_serial_bounds` target extracts the actual patched
writer and serializer and links the pinned `N2kMsg.cpp`. Its queue/listener are
fakes: no port is opened. The original exact source reproduces the short-request
overread under ASan; the patched source passes ASan/UBSan locally. Native results
are recorded separately. Source archives and Windows source-reconstruction
inventories include the new patch; no existing history/evidence is rewritten.

This patch does **not** qualify serial hardware control. Queued messages across
reconnects, physical write failures, receive timestamps, connection generations
and explicit per-session permission remain separate SCRUM-313 requirements.
No plugin output permission, connection direction or automatic pilot control is
changed. See [the boat integration inspection](pilot-boat-integration.md).

## Boat feedback integration — SCRUM-291 / 294–300 / 303–309

The existing reviewed hooks remain pinned to OpenCPN 5.12.4. This batch adds:

* Completed normal route-progress capture of `GetCurrentXTEToActivePoint` and
  `GetXTEDir`; no OpenNav-triggered route update or autopilot getter call.
* Native chart-position/object query presentation using the complete upstream
  object-query output. Legacy/Safe keep their original dialog path.
* Existing route hit-testing/rollover copied into an owned route context card;
  track and AIS rollovers retain their original semantics.
* Prototype ownship paint for verified fixed/scaled bitmap cases and a read-only
  ownship-state getter. Custom/scaled-vector exceptions preserve upstream.
* Theme ink at the actual compiled S-57 light-hover painter; unchanged sector
  geometry and Standard/Legacy fallback.
* Narrow classified light-support tower aliases in core and private RenderSY,
  preserving ordinary/conspicuous shapes and all original lookup resources.
* Anchor-watch paint hooks using the existing selected watch identities/radii.
  Mark provenance verifies the pinned anchor SVG hash/pixels; user/plugin
  replacement revokes provenance. Both render paths preserve custom marks.

Waypoint/route operations continue through the existing OpenCPN storage and
change-notification boundary. Chart-derived suggested names use only already
loaded native ENC objects with bounded queries, never an independent database.
Read-only pilot discovery uses existing OpenCPN connections and fresh identity/
feedback. It does not enable physical command output.

The [batch evidence](evidence/2026-10-05-boat-feedback/README.md) links individual
increments and focused checks. All patches reproduce exactly against the pin.
Integrated native Windows and boat acceptance remain distinct pending gates.

## Ordinary chart typeface (SCRUM-263)

The core and private libraries gain an optional presentation-owned text face,
set only during verified SKAGER construction before any label cache exists.
At ordinary `RenderT_All` font establishment, a successful Segoe UI/Arial font
may replace only the face; existing family, style, S-52 size and weight remain.
The original cached font remains the fallback. Exact geographic and generated
LIGHTS roles are excluded even when their specialized resolver declines.
No painter, label string, placement, chart preference or Standard/Legacy path
is rewritten. Enumeration is cached per module, outside painting. See
[scope and focused evidence](evidence/scrum263-ordinary-chart-face/README.md).

The geographic resolver separately follows the prototype's explicit Segoe UI
family for Land labels; Water keeps the inherited main stack. This changes only
the two core/private face-choice expressions. Sizes, weights, tracking, opacity,
LIGHTS behavior and resources are unchanged. [Exact resolver evidence](evidence/scrum263-geographic-face/README.md).

## Classified prototype light and special-buoy aliases (SCRUM-264)

The core and private `RenderSY` hooks select only stable library-owned alias
Rules after verified SKAGER resource loading. Original LIGHTS11/12/13 vectors
remain stock; compact light aliases require absence of an ORIENT attribute.
Encoded directions and all upstream conditional/angle handling are retained.
The separate special-buoy hook requires Simplified lookup and the exact inspected
white/orange horizontal-band pillar attributes. It changes no lookup or object
metadata. Day/Night may use XNSPPW01; Dusk and unknown schemes retain the original
Rule. Missing or invalid aliases, Standard, disabled integration and unclassified
objects retain stock. Separate TOPMAR and LIGHTS composition remains unchanged.
See the [light boundary](evidence/scrum264-oriented-light-aliases/README.md) and
[buoy boundary](evidence/scrum264-white-orange-pillar/README.md) for exact checks,
rejected color trial and remaining actual native/boat gates.

## Effective point-symbol presentation (SCRUM-267) — in progress

The supplied modern art maps to Simplified lookup records, but upstream defaults
to Paper. The verified SKAGER S-52 instance therefore enables a nonpersistent
effective Simplified policy, separate from `m_nSymbolStyle`. Rendering, object
queries, caches, mariner parameters and plugin presentation messages use the
effective getter; configuration and advanced preference readers/writers retain
the original field. Standard/Legacy/fallback instances remain ordinary. Core
diagnostics record both values. The private o-charts port must agree before
qualification. [Focused source/method evidence](evidence/scrum267-effective-symbol-style/README.md)
records unchanged complete persistence/conditional sources, 31 method checks and
seven actual production object compilations. Actual profile/mode cycles and
native/boat visual validation remain open.

## Exact-plugin presentation selection (SCRUM-259) — in progress

The chart-presentation patch adds one optional hook at the shared
`model/src/plugin_loader.cpp` load boundary, covering initial load and reload.
The original `m_plugin_file` remains unchanged. The registered application hook
may load one hash-qualified SKAGER-owned o-charts adapter; false retains the
original module, and failure to unload a rejected module stops that load.
Model-only tools have no registration and preserve the stock path. Adapter
selection is unavailable in Standard/Legacy/Safe and when the qualified package
or resources are absent. No global shared-data API, CWD, original plugin, helper
or chart file is rewritten. [Ownership and acceptance boundary](architecture/ocharts-presentation-adapter.md).

The current default build carries no qualified adapter. Linux source checks
alone do not accept this Windows/module-lifetime boundary.

## Actual active waypoint name presentation (SCRUM-257) — 2026-10-03

The chart-presentation patch now obtains a separate label ordinal in the pinned
`gui/src/route_point_gui.cpp` software and GL paths. Only the actual active point
from `Routeman::GetpActivePoint()` can retain the prototype name card while its
upstream active icon blinks. The numbered-marker ordinal remains zero for that
point; stock icon substitution and blink branches are unchanged. All existing
custom-icon, shared/layer, selection/edit/drag, anchor/MOB and bounded-route
eligibility checks remain. GL bounds are invalidated before culling when label
eligibility changes. Owned label cache lifetime and Standard fallback remain
unchanged. This is presentation only, with no navigation processing or output.

The [focused review](design/reviews/scrum257-active-name.md) records 76 eligibility
checks, 93 cache/raster checks, nine-patch application and four actual-source
object compilations. Integrated software/GL, native Windows and boat rendering
remain separate gates. The test fixture repaint correction changes only
`tests/RouteProgressScenario.cpp`, not an upstream navigation hook.

## Public SKAGER menu and launch help (SCRUM-235) — 2026-10-02

The existing `opencpn-5.12.4-xnav.patch` now names the integrated Legacy mode
menu SKAGER and uses SKAGER in CLI interface/recovery help. Its menu lookup uses
the same public caption. Command switches, navigation behavior, upstream
OpenCPN attribution and internal integration symbols are unchanged. See the
[bounded native branding inventory](architecture/skager-native-branding.md).
Native Windows and boat acceptance remain open for the integrated revision.

## Local peer sharing unavailable in public-beta candidates (SCRUM-212) — 2026-10-01

The ninth reviewed patch, `patches/opencpn-5.12.4-peer-unavailable.patch`,
contains local peer transfer until authenticated peer identity and credential
handling are qualified. The integrated application does not generate a peer
certificate, start its REST listener or advertise its peer service. Outgoing
discovery and transfer UI are unavailable, and model entrypoints reject forced
calls before credential access or navigation-object serialization. The same
policy applies to XNav, Legacy and Safe modes in the integrated product.
The shipped CLI also refuses peer-key generation/storage before credential
access, and the model key-check symbol rejects direct callers.

Stock/pristine OpenCPN is unchanged. The server model remains available to
upstream isolated tests; the audited application startup call site is the
inbound containment boundary. This preserves the existing upstream REST tests
without adding a runtime setting or insecure pairing fallback. Normal local
route/track/waypoint storage, file export and Send-to-GPS remain intact.

The source archive inventories all nine patches. Integrated model rejection
tests and mode-cycle process-owned listener/credential-preservation checks
are required; native Windows acceptance is still pending. See
[peer unavailable boundary](architecture/peer-unavailable-boundary.md).

## Maintained Windows curl integration (SCRUM-209) — 2026-09-30

`patches/opencpn-5.12.4-maintained-curl.patch` applies only to the disposable
integration source. It keeps the pinned `WIN32_LIBCURL` and `WIN32_ZLIB1`
imported targets and their cache paths unchanged. After the source builders
verify curl's complete OpenSSL 3.5.9 and zlib 1.3.2 import closure, the patch
removes the legacy CA bundle, `libeay32.dll` and `ssleay32.dll` from the
integrated CMake install list and installs the verified maintained
`libcurl.dll` there instead.

The pristine source and build retain their stock dependency behavior. No user
installation is mutated or cleaned by this patch. Merge risk is low and
localized to `model/cmake/Curl.cmake`; native Windows application link/import,
TLS, installer upgrade/rollback and runtime acceptance remain required. See
`docs/architecture/windows-native-dependency-integration.md`.
## Windows numeric-limit compilation boundary (SCRUM-224) — 2026-10-02

Native candidate `eb86e4799c292a19a218626272d5a5bfa25aad5a` reached
application compilation after the maintained dependency suites, then failed
because Windows' function-like `max` macro expanded seven added
`std::numeric_limits<T>::max()` calls in Downloader and the peer response buffer.
The repair parenthesizes those function names as
`(std::numeric_limits<T>::max)()`. Every overflow/size bound and failure behavior
is preserved. It does not change global `NOMINMAX` or upstream Windows headers.
A focused native actual-source compile must pass before the next integrated
candidate. A previous isolated plugin test's `NOMINMAX` definition masked this
environment mismatch; it must not be used to qualify production compilation.

## Public Downloader TLS boundary (SCRUM-211) — 2026-09-30

The reviewed integration patch
`patches/opencpn-5.12.4-download-trust.patch` changes only the pinned
Downloader GET/HEAD path and the network `PluginHandler::InstallPlugin`
overload. It requires verified chain and hostname, permits only HTTPS initial
and redirect protocols, checks curl setup failures and stages file downloads
until the complete transfer succeeds. Windows selects curl's native CA support;
Linux retains libcurl's configured system trust. Manual local plugin archive
installation is unchanged.

The actual patched Downloader passes the isolated loopback cases recorded in
`docs/architecture/download-trust-inspection.md`. Native Windows trust, the
maintained curl integration, full PluginHandler extraction, WXCURL and local
peer identity remain separate gates. Merge risk is localized to the pinned
Downloader and PluginHandler functions; the patch list applies it only to the
disposable integration worktree.

## WXCURL and bounded peer responses (SCRUM-211/212) — 2026-09-30

`patches/opencpn-5.12.4-wxcurl-trust.patch` changes only the pinned
`libs/wxcurl/include/wx/curl/base.h` and `libs/wxcurl/src/base.cpp`.
It verifies chain/hostname, selects Windows native CA trust independently of
initial protocol, prevents HTTPS redirects from downgrading, and refuses to
perform a partially configured curl handle. Existing HTTP/FTP/Telnet entry
points remain. The real Linux harness passes 13 transport/configuration cases;
the native Windows harness is a required, still-pending gate. See
[wxcurl trust](architecture/wxcurl-download-trust.md).

`patches/opencpn-5.12.4-peer-response-buffer.patch` changes only
`model/src/peer_client.cpp`: a NUL-initialized 64 KiB response buffer, checked
allocation/overflow and request initialization. The exact patched code has
portable failure-injection coverage. This does **not** resolve peer identity,
pairing, credential logging or malformed JSON semantics. Those security gates
remain open under SCRUM-212; see
[peer inspection](architecture/local-peer-trust-inspection.md).

Both patches are applied only to the disposable pinned integration tree and
are included in the corresponding-source inventory. The reviewed patch
result is verified before building; native plugin, installer and boat acceptance
remain separate from these local checks.

## Windows OpenSSL build boundary (SCRUM-208) — 2026-09-30

`3cbd7e5` adds an integration-only dependency override, without editing the
pinned OpenCPN source or its pristine dependency reference. After upstream
`buildwin/win_deps.bat`, `tools/build-openssl-windows.ps1` compiles the verified
OpenSSL 3.5.9 archive with MSVC `VC-WIN32 shared`. It runs upstream `nmake test`
before installation and copies headers, import libraries and major-version-3
DLLs into the disposable integration tree's `cache/buildwin`, which the pinned
Windows CMake files already consume. The app/plugin ABI remains x86/Win32.

The source lock records SHA-256, archive size and the separately verified
OpenSSL signing fingerprint. NASM 3.02 is a build-host tool; its isolated
fallback comes from an official HTTPS ZIP with a pinned hash. NASM's vendor
publishes no independent checksum/signature for that ZIP, so it is not described
as signature-verified. Neither tool changes an installed OpenCPN or boat profile.

The generated `openssl-build.json` binds source, build/test completion,
toolchain log and produced hashes. The integration installer compares the
installed DLLs with that manifest. `bb294f3` retains its referenced compiler
and upstream-test output in `evidence/local/windows-openssl-native-output.log`.
PowerShell parsing and the isolated NASM archive-entry checks pass locally;
native MSVC, upstream OpenSSL tests, TLS, plugin imports and installer/recovery
qualification are still pending. A successful source verification is not a
successful native build.

This boundary covers `libssl-3.dll` / `libcrypto-3.dll`. Retained stock
`libcurl.dll` imports a separate `ssleay32.dll` / `libeay32.dll` pair identifying
OpenSSL 1.0.2n. Their current integrated-package provenance and replacement are
under separate investigation in SCRUM-208; they have not been deleted or
silently relabelled. Full dependency security and license acceptance remain open.

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

### Opt-in Linux fixture startup observation (SCRUM-98)

The retained `3fb6635` preview failure observes a replacement process exiting
255 before its first normal OpenCPN log record. A separate syscall-instrumented
run passes; that run does not establish a repair. The new hook is diagnostic,
not a change to recovery policy or an accepted fix.

In `gui/include/gui/ocpn_app.h` and `gui/src/ocpn_app.cpp`, only a Linux/GTK
fixture build adds stage observation around the original application entry,
`wxApp::Initialize`, `wxApp::OnInit`, command-line parsing, config-directory
validation and `OCPNPlatform::InitializeLogFile`. Each original operation runs
exactly once, with its original arguments, result and exception behavior.
The entry uses the same wx application initializer and single `wxEntry` call;
it introduces no GTK initialization probe, delay, retry or alternate parser.
The OpenNav parser records fixed rejection categories in the integration layer.

Observation additionally requires explicit environment opt-in and a marked
disposable file sink. Only PID, fixed stage and signed numeric result are
written, with a per-process limit. The hook never records arguments, environment
values, configuration contents or network input. Normal installed/product and
native Windows builds retain the original entry and delegates. See the
[trace contract](startup-observation.md) for the exact sink policy.

The public/plugin APIs start too late to distinguish these pre-log failures.
Merge risk is medium: re-inspect the wx entry macro and the pinned startup/parser
order when updating upstream or wxWidgets. Tests cover exact-once delegation,
signed/boolean return values, exceptions, errno, the fixed record format, record
bound, opt-in/path refusals, and product/Windows compile exclusion. The latter
is a Linux compile-policy test, not native Windows qualification. Integrated
execution and replacement platform evidence remain separate gates.

### Prototype navigation horizon (SCRUM-100)

This correction adds no direct upstream patch. The native horizon formats
copied Vessel Data and the existing ordered SmartNav advice. Full passage opens
the existing drawer; NOW uses the existing `MyFrame::TogglebFollow` callback;
route and AIS context actions use the existing owned navigation interfaces.
Click-time checks re-read owned snapshots on the application thread and reject
stale or changed identities. No route processing, sensor refresh, autopilot
output or alternate navigation calculation is used as a getter. Geometry is
copied into existing diagnostics. The integration test attachment adds the
owned Horizon model suite. Legacy, Safe and OpenCPN storage are unchanged.

### Prototype owned surfaces and deferred frame raise

The pinned `MyFrame::ProcessCanvasResize()` schedules `OnRecaptureTimer()` one
second later. That callback previously raised the main frame unconditionally.
After leaving a full-page view for a Passage or Alerts sheet, Linux pointer
evidence showed the underlying timeline receiving the click during this raise,
even though the sheet returned above it on the following UI tick. Per-tick
restacking and a longer mouse press did not close that input interval.

The callback now asks `HasXNavTransientSurface()` before raising. Only the
active XNav shell's visible drawer, context card, or frame-owned modal dialog
suppresses that delayed raise. No surface is made globally topmost and no
foreign window is activated. With no such surface, and in Legacy/Safe/pristine
builds, the original `Raise()` remains. There is no chart reparenting, timer
rescheduling, navigation update, sensor mutation, or hardware command.

Source boundary: `gui/src/ocpn_frame.cpp`, `OnRecaptureTimer`, in the existing
version-checked XNav patch. A public after-raise callback was not present in the
pinned implementation; asynchronous restoration leaves the observed input gap.
Merge risk is low: re-inspect the recapture/resize sequence on every upstream
upgrade. Gates: actual route gestures after deactivation, readable alert
acknowledgement, XNav/Legacy/Safe lifecycle, primary captures, and native Windows
qualification. Replacement results are tracked in status; this is not a release
acceptance claim.

### Prototype Instruments presentation

The Instruments increment adds no OpenCPN source hook. Its native page copies
the existing VesselState through `application::PresentInstruments`, retains
the XNav horizon and delegates configuration to existing Settings actions.
The north-up wind drawing requires true heading plus a coherent signed
relative angle; it does not publish a new vessel quantity or alter upstream
navigation. The integration CMake attachment adds the independent provenance
test and non-installed widget capture executable. Legacy/Safe and chart
parentage/storage are unchanged.

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

## Authenticated update startup receipt

The secure updater reuses the existing command-line and main-thread health
integration hooks; it adds no new direct OpenCPN patch. `ParseCommandLine`
captures and clears the bounded update challenge before plugin startup.
The existing XNav shell readiness observation and successful durable
`RecoveryStore` checkpoint feed an owned one-shot receipt. A separate worker
writes to the supervisor's local pipe without retaining application pointers.
Legacy/Safe cannot acknowledge XNav health. See
[the secure updater contract](installer/secure-updater.md). Native fixture
evidence and actual installed-application qualification remain distinct.

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

The chart depth-unit presentation now has two narrow paint hooks at the existing
software and GL emboss sites. `ChartCanvas::GetChartDepthUnit` extracts the
original quilt/single-chart resolution body unchanged; `EmbossDepthScale` calls
the same read-only method. XNav's verified style can draw the actual unit using
the prototype metadata font/ink. Unknown/mixed units are not guessed and the
upstream visibility preference remains effective. Standard/Legacy/Safe use the
stock emboss path. Overzoom indication is unchanged. A const text-color getter
on `ocpnDC` lets the integration restore drawing state after either renderer.
This getter adds no data member, plugin ABI or input/output behavior. The
diagnostic snapshot reports the actual upstream enum and visibility preference;
the public ENC capture can explicitly test Feet, Meters and Fathoms. Native and
boat replacement rendering gates are still required.

`ChartCanvas::ScaleBarDraw` keeps its existing geographic conversion, user unit
selection, nice-distance rounding and projected length. For verified XNav only,
a hook moves its origin beside Follow Boat and provides a smaller reference
span before that same computation. A second hook paints the resulting upstream
label/length and updates the existing scale bounds. Standard/Legacy/Safe return
to the original span and paint. There is no independent scale calculation.
Diagnostics copy the existing `GetScaleBarRect` result; capture tests ensure
the legend is inside the chart and cannot overlap Follow Boat.

Two small GUI hooks let `GSHHSChart::SetColorScheme` and the `LANDBACK`/`BLUEBACK`
background colors use XNav's verified palette. They return normal upstream
behavior in Legacy/Safe/Standard. No chart objects or sensor state are changed.
The existing palette setters were inspected: GSHHS otherwise hard-codes a
separate olive/blue palette while shapefile/GL paths use global background
colors. The constructor hook is necessary because replacing the library after
charts retain lookup pointers would be unsafe. Merge risk is localized to these
initialization/color boundaries and the loader's new defaulted argument.

The active-route ink increment adds three paint-only calls in
`gui/src/route_gui.cpp`: `RouteGui::Draw`, incremental `DrawSegment`, and
`DrawGLRouteLines`. `ChartActiveRouteInk` returns a copied prototype color only
on the application thread, in XNav, after presentation resources verify. The
upstream pen/brush is copied locally; no global route pen, saved property or
route/waypoint state is changed. Selected routes remain in the existing
selection path. Inactive/custom routes, width/style, arrows, clipping,
antimeridian geometry and route progress remain upstream. Standard, failed
resource verification, Legacy and Safe do not apply this override.

Rendering qualification uses the existing isolated `RouteProgressScenario`:
it copies pixel positions from `ChartCanvas::GetCanvasPointPix` and the real
upstream active pen into the test report. The external capture checks actual
stroke pixels against the independent HTML token, or the copied stock pen for
Standard. Theme and actual renderer must match the requested fixture. This
does not compute navigation geometry independently or expose pointers outside
the integration test. Exact native/boat qualification remains pending.

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

### Native Passage drawer

The prototype Passage migration adds no direct OpenCPN hook. It consumes the
accepted route, advisory and energy publications and uses existing copied
`NavigationActions` for explicit human actions. `OpenCPN.cmake` attaches a new
provenance regression and a non-installed offline widget executable; neither
introduces synthetic data into the fixture-free product.

### Native Anchor drawer

No additional direct upstream hook. The existing post-`ProcessAnchorWatch`
observer copies bounded movement history using `integration/AnchorGeometry` and
the pinned `DistanceBearingMercator`. Anchor move/deletion clears incompatible
history. New `AnchorView` requires the current selected GPS source/time/coordinate
to match before exposing retained distance. Neither the native range control nor
paint invokes progress, alarm processing or navigation commands. Existing signed
radius/alarm semantics and confirmed start/clear commands remain unchanged.
See [owned anchor contract](anchor-presentation-contract.md).

### Native manual pilot drawer

No direct upstream change. `XNavPilotDrawer` replaces the legacy XNav full-page
pilot presentation using the existing owned `PilotView`, configured permissions
and guarded manual callbacks. `PilotPresentation` withholds stale/invalid
magnetic headings without deriving a replacement. Driver observers, identity
binding, transport, command acknowledgement and rate limits are unchanged.
See [presentation contract](pilot-presentation-contract.md).

Prototype notification-centre migration changes no upstream hook. It consumes
existing copied AlertCenter episodes; pinned OpenCPN AIS/anchor alarm semantics
and pilot adapters are untouched. Existing interaction diagnostics also copy
a readable control name beside its caption; no new upstream hook or input
injection boundary is added. See `alert-presentation-contract.md`.

Prototype Radar Focus inspects the pinned `ocpn_plugin.h` generic CPU/GL overlay
callbacks, which provide no standardized owned radar-image/control contract.
No direct hook is added. `XNavRadarPanel` consumes copied unavailable/status
data and cannot call scanner commands. Shared confirmation-sheet sizing now
follows the native ownership chain to the application frame; OpenCPN core,
chart/model processing and hardware transports are unchanged.

### SCRUM-232 — default healthy ownship artwork

The existing chart-presentation patch adds a guarded paint hook at the final
fixed-bitmap sites in software `ChartCanvas::ShipDraw` and GL `ShipDraw`.
Only verified XNav style, `SHIP_NORMAL`, default fixed icon and no user image
receive the shared prototype chevron. Standard/fallback/Legacy/Safe, custom,
invalid/low-accuracy, small-scale and true-scale/scaled symbols remain upstream.
Both hooks retain projection, heading/rotation, stock predictor `img_height`,
antenna offsets, existing bounding boxes and cleanup. The new painter adds its
outline bounds and restores drawing state; it does not affect hardware output.

The four-point GL polygon is cyclically ordered so the renderer's existing
triangle strip preserves the concave stern notch. Both callers pass the same
user factor and the painter applies one logical-pixel conversion; GL bitmap
texture tint, extra size factor and content-scale are not used for this glyph.
Palette is read per paint, with Night's prototype brightness applied only to the
new artwork. Merge risk is limited to these pinned bitmap call sites and their
retained cleanup/predictor context. See
`docs/design/reviews/scrum232-ownship-chevron.md` for focused proof and explicit
Windows/GL/DPI/boat gates.


### SCRUM-238: selected-style geographic-name fonts

The chart-presentation patch adds an optional font resolver at the existing
`RenderT_All` font-cache construction boundary. Default is null. Verified SKAGER
integration applies prototype name fonts only to the four geographic feature
classes and leading OBJNAM TX; cached-font ownership stays with FontMgr. Other
text/render/visibility logic and Standard/Legacy behavior are unchanged. A
separate XNGEO resource role changes only 18 pinned geographic-name ink tokens;
full-tree integrity remains enforced. See
[scope and pending native evidence](design/reviews/scrum238-geographic-names.md).

### SCRUM-237 — default active-route foreground (partial)

The chart-presentation patch threads a bounded foreground-paint option through
`RouteGui::RenderSegment` and `DrawGLLines`, with defaults preserving existing
callers. Their existing projected/wrapped segment endpoints feed the shared
`ChartRouteSegmentMesh` / `DrawChartRouteSegment` helper. Upstream arrows,
waypoints, selection/highlight, editing, custom properties, MOB and navigation
processing stay upstream. The previous route-ink hooks now use the same strict
default-style eligibility gate. No `ocpnDC` implementation or generic GL renderer
is patched. Two changed Linux production objects compile; the production-painter
fixture passes 78 checks. The 6px underlay, 32px illustrative context, integrated
native GL/Windows/boat and full hierarchy remain open. See
`docs/design/reviews/scrum237-route-foreground.md` for exact scope and evidence.

### SCRUM-239 — healthy factory-equivalent COG line paint

One `ChartCanvas::ShipIndicatorsDraw` hook covers software and GL. Appearance
ownership is captured from configuration before density mutation; the per-frame
check tracks upstream's expected width increase and revokes ownership on custom
runtime changes. Only the healthy default-icon COG line/black inner stroke is
replaced. Existing projection, prediction time, COG/SOG and visibility guards,
HDT, COG endpoint markers, custom settings and range rings remain upstream.
No configuration write or generic renderer change is introduced. The new
fractional dashed mesh/painter passes 72 focused checks; the previous route
fixture passes 78. Both changed Linux production objects compile with `-Werror`.
See `docs/design/reviews/scrum239-cog-predictor.md` for policy ambiguities,
density-persistence limitation and outstanding native/boat visual gates.

### SCRUM-240 — healthy onboard AIS base body

`AISDrawTarget` copies appearance inputs into a paint-only helper at the
ordinary ship-body branch. Verified SKAGER presentation replaces only eligible
healthy A/B body geometry/ink. Class A retains a triangular stern; Class B uses
the prototype notch. All warning/navigation/special/Inland/realtime-prediction
states fall back, while projection, user scaling, attenuation, selection and
subsequent overlays remain upstream. The shared explicit triangle mesh avoids
the pinned GL strip's concavity problem. No AIS semantic colors or global
metrics change. See `docs/design/reviews/scrum240-onboard-ais-body.md` for exact
eligibility, focused evidence and outstanding native/boat qualification.

### SCRUM-243 — geographic name spacing and alpha

The optional S-52 text-font resolver also supplies tracking and opacity for the
bounded geographic names selected in SCRUM-238. Default fields are zero/opaque.
The existing software and cached whole-string GL text paths consume a bounded
integration-only helper; collision/justification widths include tracking.
Geographic GL textures delete/recreate on their own scale/content-scale/ink
change; explicit text-color preferences remain intact. S52PLIB opts into this
helper only through the OpenNav integration CMake hook. Stock presentation,
all navigation labels, chart strings and visibility remain unchanged.
See `docs/design/reviews/scrum243-chart-name-spacing.md` for the native-shaping
fallback, corrected painter evidence and outstanding platform gates.

### SCRUM-242: proven default route waypoint artwork

The chart-presentation patch adds an internal `MarkIcon` provenance bit, granted
only by the pinned legacy diamond loader after source hash and uncached pixel
verification, and revoked by user/plugin `ProcessIcon` replacements. A narrow
`WayPointmanGui` query checks the exact owned bitmap instance. Software and GL
`route_point_gui.cpp` hooks call the separate `ChartRouteWaypoint` helper for
unique ordinary points in a default active route; all special/custom states and
Standard/Legacy/Safe fall through. Dirty bounds include the new artwork; GL
revalidates eligible point bounds before cached culling. No route/point data or
navigation semantics are changed. See
[the boundary and evidence](design/reviews/scrum242-route-waypoint-markers.md).

## SCRUM-244: derived ACHARE51 anchorage artwork (no upstream patch)

The resource generator relocates only the effective final `ACHARE51` RCID1105
bitmap into proven unused transparent atlas space at `(20,1160,20,20)`, pivot
`(10,10)`, preserving the geographic hotspot and exact prototype anchor scale.
Only its six numeric bitmap fields and 126 formerly transparent pixels per
palette change. The stock tile, other atlas pixels/alpha, atlas dimensions and
all S-52 lookup/boundary/restriction/depth/hazard rules remain unchanged.
See `docs/design/reviews/scrum244-anchorage-art.md` and
`docs/evidence/scrum244-anchorage/review.json` for the authorized fit adjustment,
real pinned-loader fixture and open full-chart/Windows/boat gates.

### SCRUM-246 — local vector chart-selector palette

SCRUM-266 additionally records the actual CHBLK color used when
`Piano::BuildGLTexture` finishes constructing its atlas. `DrawGLSL` invalidates
the atlas before the existing height/rebuild check if that resolved color has
changed. Lazy S-52 creation can change the resolved outline after initial style
activation without changing either tracked vector brush. This fixes the retained
1,270-pixel GL Day-return mismatch without changing selector geometry, chart
selection, hit regions, persistence, software drawing or any palette value.
Deferred atlas builds do not stamp new ink. The 21-case actual method/bitmap
fixture reproduces the original failure and passes the correction; 159 existing
brush checks and the real production GL compilation also pass. Integrated and
native/boat gates remain open. [Evidence](evidence/scrum266-selector-cache/README.md).

`Piano::SetColorScheme` invokes one verified-SKAGER palette resolver after stock
brush construction and before existing GL-atlas invalidation. Only selected and
unselected vector-key fills use prototype route/floating-muted ink. No global
colors, other chart families/states, geometry, chart data, selection or input
handlers change. See [the scope and focused evidence](design/reviews/scrum246-chart-selector.md).

### SCRUM-241 — bounded active-route understroke

`RouteGui` now collects its already projected/rejected/clipped/wrapped legs before
its existing paint pass. The software collector substitutes projection for point
drawing; GL collection suppresses normal point-state writes until the unchanged
normal pass. One owned `ChartRouteUnderlay` unions the 6px miter/butt stroke with
existing `ocpn::tess2` and paints once at .6 alpha, before foreground/waypoints.
No generic GL, `ocpnDC`, route model, configuration or navigation processing is
changed. The same verified factory-equivalent/custom/MOB guards apply.

The whole decorative layer is omitted for joined visible legs <=6 logical px,
more than 1,024 collected legs, invalid geometry or tessellation/resource failure.
This preserves all foreground/waypoint behavior. A demonstrated pinned-tess2
coincident-edge defect and its intentionally failing raw reproduction are retained;
no route simplification or partial alpha painting hides it. See
[SCRUM-241 review](design/reviews/scrum241-route-underlay.md) for exact collection
boundaries, memory/workload limits, fixture and reproduction. Focused geometry,
real wx alpha and recorded GL submission tests pass, the actual changed Linux
objects compile, and all nine patches apply to pinned upstream. Integrated real
GL/native Windows/DPI/boat and full route conformance remain open.


### SCRUM-239 follow-up — enabled COG endpoint fill

One additional `ShipIndicatorsDraw` brush substitution uses prototype route ink
only when the existing `xnav_cog_painted` result is true and chart ink is verified.
The VECGND02-derived quad, projected prediction position, GPS offset, scale,
stock black border, separate endmarker preference and HDT remain unchanged.
No new geometry, preference mutation or renderer is introduced. The actual
endpoint/guard/software polygon bodies pass 5,477 checks against the unmodified
pinned marker; existing predictor checks pass 72. The changed production canvas
object compiles and all nine patches apply. See
[endpoint review](design/reviews/scrum239-cog-endpoint.md) for the explicit
semantic extension, fixture and outstanding native/GL/boat gates.
### SCRUM-243 reviewed text-boundary corrections

The styled geographic whole-label GL path now preserves overlap rejection and
uses the standard screen-space rotation before collision checks. Its cached
metrics match the unscaled raster quad; styled software/GL offsets use measured
native glyph units. Nonstyled paths retain their existing results and metrics.
See `docs/design/reviews/scrum243-text-boundaries.md` for actual-method fixture,
negative controls, production objects and remaining native gates. Styled GL
font requests and tracking also follow software at content scales above one.

### SCRUM-246 — deferred presentation fallback

`Piano::SyncChartPresentation` runs before software painting and the GL atlas
validity check. It restores the two stock vector brushes if deferred library
creation revokes verified SKAGER presentation, and invalidates the existing
atlas only when those colors change. Key geometry and interaction are untouched.
The actual-method fixture covers late failure and unchanged-frame caching;
integrated/native rendering qualification remains required.
### SCRUM-249: verified sounding digit font

The chart-presentation patch adds an optional owner-held sounding font policy in
`s52plib::RenderSoundingSymbol`, installed only on the successfully verified
SKAGER library. It maps the final prototype 10px normal font stack while keeping
the sounding-size preference and semantic colors/qualifiers. `DepthFont::Build`
accepts an optional exact-font flag to preserve the same fractional font as the
software painter; existing callers retain their default path. Cache invalidation
covers preference, content scale and DIP factor. All rule/pivot/rotation/color
code after font selection and the complete multipoint semantic painter remain
pinned. See `docs/design/reviews/scrum249-sounding-typography.md` for focused
raster/object evidence and the outstanding Windows/actual-GL/boat gates.
### SCRUM-248 LIGHTS descriptions

The chart-presentation patch extends the optional text resolver with a bounded
normal generated LIGHTS role. Existing RenderT_All/RenderText retain actual
strings, visibility, placement and overlap logic; a shared cached label raster
supplies prototype font/tracking/ink/water halo only for factory-equivalent
ChartTexts appearance. Runtime custom appearance and unsupported raster bounds
restore stock presentation. Other labels, global CHBLK, symbols and Standard
remain unchanged. See [scope and evidence](design/reviews/scrum248-light-description-typography.md).

### SCRUM-250: final chart GL framebuffer size

The chart-presentation patch adds a SKAGER-shell-only check before the existing
FBO cache selection. It reconciles the child with its final parent client size,
rebuilds an undersized existing cache and falls back to direct chart rendering
if it still cannot cover the viewport. This corrects the measured1012×558 cache
sampled across1014×566 after theme layout, without clamping or cropping chart
content. Legacy/Safe, software, normal cache reuse and chart semantics remain
unchanged. See `docs/design/reviews/scrum250-chart-framebuffer.md` for actual
Mesa baseline traces, extracted-method checks, production GL object and open
combined/native/boat acceptance gates.

### SCRUM-251 submarine cable paint

No C++ patch is added. The verified resource generator changes only CBLSUB06
line-style RCID2012 color-ref ACHMGD to AXNCBL, adding three dedicated prototype
area-hue colors (including Night brightness). A reverse-equality validator
protects HPGL, all symbol geometry, global CHMGD and navigation lookups. The
helper is included in CMake regeneration dependencies and Windows preflight
input identity. See [real ENC evidence](design/reviews/scrum251-submarine-cable-paint.md).

### SCRUM-228: delayed GTK owner recapture and chart-control stacking

The xnav patch adds a GTK-only hook immediately after the pinned
`MyFrame::OnRecaptureTimer` owner Raise. It restores only already visible,
mapped SKAGER-owned chart controls above that owner using the existing native
restack primitive. The recorded stale-time raising caller is this exact timer;
the repair does not add polling, focus activation, global topmost state or a
generic event filter. The existing transient-surface guard and Legacy/Safe
behavior remain unchanged. See [the focused repair review](design/reviews/scrum228-recapture-repair.md)
for native inversion/repair pixels, negative control and production objects.

### SCRUM-254: isolated pilot boarding and radar beacon artwork

The verified SKAGER resource generator also derives only the effective
`PILBOP02` RCID 1 and `RTPBCN02` RCID 2259 bitmap metadata and dedicated atlas
pixels from the immutable prototype. Existing S-52 rules and renderer remain
unchanged. Source locks, collision/transparent-moat refusal, full rule-tree
reverse equality and native loader/crop evidence are documented in
`docs/design/reviews/scrum254-service-glyphs.md`. Standard/Legacy/Safe retain stock
resources; full chart, native Windows and boat visual acceptance remain open.

### SCRUM-256: explicitly classified Simplified cardinal artwork

Only BOYCAR01–04 effective raster metadata and four isolated atlas tiles change
in verified SKAGER resources. CATCAM1–4, all S-52 selectors, Paper bodies/topmarks,
labels and upstream scale/placement remain unchanged. The source-locked exact
prototype paths, category-color limits and focused native-loader evidence are in
`docs/design/reviews/scrum256-cardinal-glyphs.md`. Standard/Legacy remains stock;
Windows, real-ENC recognition and boat acceptance are not claimed.

### SCRUM-264: supplied generic beacon resource mapping

The resource generator derives `XNBCNG01` from the immutable prototype and
redirects only the two pinned Simplified generic lookup tokens (1696/31748 and
1708/31760). It does not edit renderer code, original BCNGEN01 definitions,
classified/Paper consumers, topmarks or Standard resources. The source locks,
whole-resource inverse tests and exact prototype crop comparison are recorded
in [the focused evidence](evidence/scrum264-generic-beacon/README.md).

### SCRUM-264: explicit yellow special-mark composition

The core/private chart presentation patches share `ChartYellowBuoySymbol.h`.
`RenderSY` can select a verified yellow body only after the original lookup chose
BOYSPP11 and typed attributes prove the supported class. A separate fitted-X
call follows `ObjectRenderCheckRules` and DC setup in `DoRenderObject`; it accepts
only the original empty Simplified TOPMAR rule and an explicit TOPSHP7/COLOUR6
object with one matching eligible floating platform. It draws an existing
library-owned raster Rule and leaves upstream lookups, caches, priorities,
projection and Standard behavior intact. See the
[focused source and mutation proof](evidence/scrum264-yellow-special/README.md).

### SCRUM-267: private point-style observation

Four activity calls in the private plugin's Init/DeInit/destructor govern the
new copied-data observation export. No existing binding/status ABI layout or
chart-selection behavior changes. Host diagnostics distinguish core from private
effective-table observations; unavailable private data is never inferred from
requested style. The strict native/package export inventory includes the new
export. Exact lifecycle and unchanged-helper source hashes are recorded in
[the observation evidence](evidence/scrum267-private-diagnostics/README.md).

### SCRUM-271 — upstream notification affordance presentation

`opencpn-5.12.4-chart-presentation.patch` now adds a presentation-only entry hook
in `NotificationButton::CreateBmp`, an explicit successful-SKAGER-bitmap flag,
and DPI-size cache invalidation. `ui/NotificationButtonBitmap.h` reuses the
prototype bell path, 44 DIP icon target, 9 px radius and theme tokens. The
informational/warning/critical icon names resolved by upstream select cyan,
amber and red respectively, on the prototype alert surface. Unknown artwork,
bitmap failure, Legacy and Safe Mode use the unchanged stock drawing body.
The existing rectangle receives the bitmap's physical size; upstream placement,
logical hit conversion, click routing and notification-list acknowledgement
remain authoritative. No illustrative count is added.

For the successful SKAGER bitmap only, the existing texture upload copies its
alpha and `NotificationButtonBlend` temporarily enables normal alpha blending,
then restores the incoming blend enable, RGB/alpha factors and equations. Stock
textures retain their original opaque upload and no blend-state intervention.
No upstream rendering method is duplicated or replaced. NotificationManager,
ChartCanvas visibility/count/maximum-severity logic, NotificationsList and its
GUID acknowledgement code are unchanged.

Focused compilation, paint/cache/source boundaries and unqualified native/GL
and boat gates are recorded in
[the SCRUM-271 review](design/reviews/scrum271-notification-style.md).

### SCRUM-274: owned AIS connection observation

The AIS transport patch adds an owned numeric connection value to IX's existing
client init result and Open event. After successful TLS/HTTP upgrade, while the
transport owns its socket mutex, the socket copies both OS-reported endpoints
under its base socket mutex. Values include address family, network-order
address bytes, host-order ports, IPv6 scope IDs and a steady-clock capture time.
Failure remains unavailable and never changes connection success. No descriptor,
socket pointer, hostname, credential or peer text crosses this boundary. The
new value header is included in IX's existing header/install list.

The provider adds its lifetime-scoped generation and passes the value through an
accepted session Open transition only. The session clears it whenever state
leaves Subscribing/Connected. Repeated enable calls and viewport subscription
changes preserve the original capture. Actual enable transitions and credential
changes invalidate in-flight callbacks and credential reads; a read completing
after such a change is discarded before connection. Snapshots retain owned
historical values and Read never refreshes their timestamps. Observation is not
a current-liveness or process-identity claim.

Dedicated loopback tests alone may pause a genuine IX Open before the provider
mutex/generation check via OPENNAV_AIS_TEST_TRANSPORT. This bounded gate is absent
from production compilation and has a finite test deadline. It exercises delayed
Open versus disable/re-enable and credential replacement without a socket getter
or a production fault-injection API. Evidence and remaining native/WFP/boat gates:
[SCRUM-274 checks](evidence/scrum274-connection-observation/README.md).

## SCRUM-275: scoped ordinary CA light point (pending canvas qualification)

The chart-presentation core patch and private o-charts patch add one bounded
presentation inventory around existing point passes (core GL/DC, private GL's
two rectangles/DC). Only verified Simplified ordinary homogeneous LIGHTS CA
records can add an existing library-owned prototype point after the unchanged
arc painter. Co-located structures/topmarks, mixed/uncertain/directional cases
and the legacy no-arc GL path retain original painting. Scope RAII borrows no
object beyond that synchronous pass. No conditional calculation, resource,
lookup, arc, visibility or navigation behavior is replaced. Exact boundaries,
focused method/changed-object proof, source counts and remaining canvas/native/
boat gates are in `docs/evidence/scrum275-ca-light-point/README.md`.

### SCRUM-275 follow-on: mixed ordinary sectors and exact offset fog

The subsequent shared inventory helper allows different individually eligible
ordinary single-color CA records to share one generic `XNLIT013` location point.
Each original CA instruction still paints independently. Only the full verified
Simplified FOGSIG/31164 → `SY(FOGSIG01)` signature is exempted from co-located
point refusal; a loaded rule must also retain its pinned raster identity and
geometry. A null lazy rule is not processed as a getter. Independent tower,
pile, topmark and all other point classes still refuse the added location point.
No upstream patch body, resource or conditional rule changes in this increment.
The source-bound bitmap-selection proof, focused checks, changed-object builds
and unqualified canvas/native/boat limits are in
`docs/evidence/scrum275-ca-light-fog-mixed/README.md`.

### SCRUM-275 lifetime correction after normal Release compilation

The unchanged normal GCC16 Release `-O3 -Werror` build rejected storing the
stack inventory address in the library. The shared helper now owns a stable
inventory allocation for the synchronous pass and restores the prior library
pointer before freeing it. Disabled/Paper and allocation-failed scopes shadow
an outer inventory with null; they never consume that outer pass's points.
Disabled/Paper passes allocate nothing. Existing bounded map-allocation failure
and original stock rendering are preserved. No compiler diagnostic is suppressed,
no upstream painter or conditional changes, and no pointer lifetime escapes the
integration pass. [Focused checks and limits](evidence/scrum275-scope-lifetime/README.md)
retain the original failure separately. Native and actual canvas acceptance
remain independent gates.

SCRUM-276 adds guarded ordinary compact sector fan paint at the existing core
and private `RenderCARC_GLSL` / `RenderCARC_VBO` boundaries. It uses exact prototype
wash/line inks, weights and alpha with each renderer's unchanged CA geometry.
The shared header owns only bounded transient tiles; it neither changes shared
Rule caches nor persists GL state. Disabled, nonordinary, unsupported viewport
and excessive-work cases retain the original painter. The private preparation
closure includes the header. Focused actual-method software/Mesa, source patch,
O3 object and explicit near-cap performance limitations are recorded in
`docs/evidence/scrum276-compact-ca-fan/README.md`.

### SCRUM-275 actual GL multipoint-sounding correction

Pinned `SetMultipointGeometry` stores SOUNDG parents as GEO_POINT with arrays,
without scalar coordinates. Both shared inventory walks now exclude only the
validated non-clone multipoint container before scalar access. Independent
malformed points retain refusal; no sounding data or painter changes. The
[negative reproduction and 147/147/145 focused checks](evidence/scrum275-multipoint-sounding/README.md)
precede actual combined canvas/native verification.

### SCRUM-268: ordinary GL text metric consistency

Both core and private `s52plib::RenderText` measured atlas `M` width only when
creating a glyph atlas. A new text object reusing it kept the different upstream
`X` width, so identical chart labels shifted after palette changes. The two
bounded hunks move the existing measurement/assignment immediately outside that
cache-miss branch. Font, formatted content, anchor, offsets, scale, rotation,
decluttering and paint calls are unchanged. This intentionally fixes ordinary
Standard/Legacy text too, retaining its original first-draw placement. It adds
one cached character-extent query, no new atlas/font/allocation. Software and
specialized whole-label paths are unchanged.

[Actual-method negative controls and core/private checks](evidence/scrum268-cache-metric-consistency/README.md)
and [the corrected Linux application/canvas proof](evidence/scrum268-5bb-linux-canvas/README.md)
retain original failures and exact source/binary identities. Native Windows,
private DLL runtime and boat acceptance remain separate; frozen63c1029 excludes
this later correction.

### SCRUM-265: default building point resource alias

`XNBLDG01`/RCID60016 redirects only pinned Simplified BUISGL lookup 1091/31143. Its
9×9 atlas tile at (788,1160) preserves pivot (4,4) and all 81 alpha values. The original
symbol and Paper/specialized/CONVIS1 rules remain untouched. Two isolated inks
use prototype service/neutral roles; a source-locked signed per-pixel transfer
retains the original two-pen geometry and baked edge filtering. Its vector
geometry is byte-identical, changing only the two color bindings. No painter,
conditional calculation, global CHBRN/LANDF or private C++ hook changes.
Existing manifest/header copying carries the new resource into the private
adapter package. [Source, inverse proof and explicit night limitations](evidence/scrum265-building-point-alias/README.md)
precede actual canvas/native/boat review; this is not claimed as supplied custom
building artwork.

### SCRUM-279/280: supplied marina and hazard raster artwork

The owned resource generator changes the effective bitmap metadata of SMCFAC02,
UWTROC03, UWTROC04 and WRECKS05 to four isolated 24-pixel tiles. Original lookup
selection, conditional procedures, HPGL and stock resources remain unchanged.
These shared names also affect their existing Paper/area consumers; this is not
a Simplified-only claim. No new renderer hook is introduced. The source-locked
SVGs and alpha masks preserve their geographic anchors. Focused resource and
actual-loader evidence is under `scrum279-marina` and `scrum280-hazard-glyphs`;
hazard recognition and actual chart/native/boat acceptance remain open.

### SCRUM-281: exact supplied cable waveform

Only owned CBLSUB06/RCID2012 receives new physical motif metadata and a bounded
HPGL fallback. The supplied 24-pixel motif is 635 HPGL units at nominal 96 DPI;
the prior 2293-unit width was rejected as visibly oversized. Two existing
`draw_lc_poly` HPGL calls per renderer gain a verified-owned-rule branch using
the exact quadratic stroke. Original vertices, tangents, masking, traversal,
clipping and straight remainders remain unchanged. CATCBL6 and other symbols
remain stock. The existing transient RGBA uploader accepts optional original
line-shader matrices; its CA-light defaults remain unchanged. Core/private
owner macros are distinct, with explicit inactive-guard negative coverage.

The helper retains no chart pointers or GL objects after the synchronous call,
caps tile work and restores touched GL state. Refusal uses the existing HPGL
renderer with the owned fallback resource. Dense cable upload cost, actual ENC
rendering and native/boat acceptance remain open. See
`docs/evidence/scrum281-cable-waveform/INCREMENT.md`.

### SCRUM-282: supplied fishing-area motif with native repeat spacing

Only Plain FSHFAC lookup65/RCID32101/CATFIF1 redirects AP(FSHFAC03) to owned
XNFISH03/RCID60017. The original same-name point symbol and stock area pattern
remain unchanged. `CreatePatternBufferSpec` gains one core/private owned-alias
branch which composes the supplied diamond/cross in the original vector-derived
cell. Spacing, stagger, polygon anchor and outer SW/GL renderers are unchanged.
The minimum glyph inset is bounded and measured across the three target DPIs;
it does not recenter or enlarge the repeat lattice.

Invalid identity, unsupported scale, clipping or failed RGB/alpha allocation
declines to the retained HPGL pattern. The private hook uses its actual
`SKAGER_OCHARTS_ADAPTER` guard, corrected after root review caught an initially
inactive host-only guard. Actual private object compilation and distinct-guard
fixtures are recorded under `scrum282-fishing-pattern/private-guard`; full
polygon, native DLL and boat acceptance are still separate gates.

### SCRUM-283: private associated-area isolated-danger query

The third ordered private patch appends one adapter-only callback to the private
chart context, binds it at the existing active initializer and initializes the
borrowed object context to null before attachment. A file-local RAII bridge calls
the unchanged private `GetAssociatedObjects` query and returns an owned list of
borrowed chart objects to the original synchronous `_UDWHAZ03` consumer. No host
API17 structure/export or original query/selection geometry changes. The original
stock-private path remains under the disabled adapter macro.

The two retained failing point cases now select ISODGR51/DisplayBase in focused
actual-source fixtures. Null/refusal, exceptions, conversion allocation failure
and lifetime boundaries have focused coverage. Unavailable evaluation is not a
safe-area assertion. The original reference-point/first-matching-area limitation
for line/area features remains documented. Evidence and original failure are in
`docs/evidence/scrum283-private-hazard-association/`; native private-DLL and boat
qualification must pass before the safety issue is closed.

### SCRUM-281 supplied submarine-cable waveform

The core/private chart-presentation patches add only two guarded motif paint
calls inside existing `draw_lc_poly`. Exact owned CBLSUB06 metadata/HPGL and the
verified-presentation flag select a24CSS-unit quadratic motif and1.3 round
stroke. Existing traversal, masks, tangents, clipping and straight remainders
remain upstream. Shared RGBA upload accepts optional actual line-shader matrices;
existing CA defaults are unchanged. The owned definition explicitly changes
repeat density to635HPGL units; Standard/Legacy resources and CATCBL6 stay exact.
See [scope, failed experiments and focused proof](evidence/scrum281-cable-waveform/INCREMENT.md).

### SCRUM-284: ordinary all-round light outline

The existing core/private CARC methods gain an outline-only branch for the exact
ordinary Simplified LIGHTS lookup, known positive range and white/red/green
full-circle signature. Both object attributes and actual conditional CA output
must match. Original classification, per-renderer center/radius, bounds and
visibility remain unchanged. Special, uncertain, directional and unsupported
objects retain stock portrayal. The prototype's finite-sector arc paint supplies
width 1.2 and opacity .8; it supplies no physical tower or exact full-circle asset.
The shared bounded tile/uploader retains no chart objects and falls back on
allocation/identity refusal. Standard and Legacy remain stock.

[Focused core/private proof](evidence/scrum284-all-round-light/README.md) and
[actual Linux chart comparison](evidence/scrum284-fc6348a-linux-canvas/README.md)
record unchanged Standard pixels and SKAGER differences confined to the old ring.
Native private-DLL and boat recognition remain separate gates.

### SCRUM-286: existing overzoom warning presentation

The software and GL overlay callers still obtain the original
`EmbossOverzoomIndicator` exactly once. For a non-null result, a verified active
SKAGER presentation may draw its translated warning using existing prototype
warning-callout roles at the upstream coordinates. Every refusal passes the same
map to original `DrawEmboss`. The indicator, stock font/map creation, 3.9 threshold,
quilt/MBTiles behavior and toolbar offset are byte-identical. Standard/Legacy/Safe
and resource failure retain the stock path. No navigation or warning trigger is
changed, and the warning cannot be dismissed by this helper.

The complete translated label must fit, without elision, before painting.
Font/pen/brush/text state is restored; no new texture cache is added. This is an
explicit required-warning extension because the supplied prototype has no literal
overzoom asset. [Focused source/painter proof](evidence/scrum286-overzoom-warning/README.md)
passes. The [actual core application comparison](evidence/scrum286-98d2c45-linux-canvas/README.md)
also passes, with changes confined to the old warning region and identical
Standard chart controls. Native Windows and boat readability remain separate.

## SCRUM-312: supervised navigation-warning phases

The core XNav patch adds two notifications around the pinned desktop
`MyApp::OnInit` call to `ShowNavWarning`, gated by selected XNav mode. It preserves
the original warning, condition, return and config semantics. The integration
reports waiting/actual choice to the updater; it cannot choose for the user.
See [startup protocol](installer/startup-human-wait.md). The modal precedes
`Attach` and deferred initialization, so shell-only reporting is insufficient.
