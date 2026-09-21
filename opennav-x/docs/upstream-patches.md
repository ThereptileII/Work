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
| gui/src/canvasMenu.cpp | Context-menu fallback for mode switch | Menu access with hidden menu bar; low risk |
| model/src/plugin_loader.cpp | Keep plugins inactive in Safe Mode without persisting disabled preferences | Enabled Dashboard fixture through Safe and normal restart; low risk |
| gui/src/pluginmanager.cpp | Avoid saving temporary Safe Mode plugin states as normal preferences | Same shared-profile fixture; low risk |

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
