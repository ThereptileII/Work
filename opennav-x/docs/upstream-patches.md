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

## First dual-mode integration (in progress)

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

Public plugin API 1.20 does not provide ownership of application startup,
main-frame chrome or shutdown. Narrow core hooks are necessary; zoom, follow,
theme, chart, route and configuration behavior reuse the existing implementation.
All GUI hooks are guarded by OPENNAV_X. No device command logic is added.
Windows and Linux interaction acceptance remains pending; compilation alone is
not acceptance. See baseline.md for the independent pristine-build results.

### Shutdown timer guard

The frame timer hook also returns after ProcessQuitFlag closes the frame. A
pristine Linux core showed APConsole::IsShown reached later in the same timer
callback after cleanup. This one guard affects no normal navigation work and
runs only in the integrated build. Regression: normal close, mode restart and
IPC quit must all exit without a crash. Pristine source retains the original
behavior for comparison. This change is pending runtime validation.

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
