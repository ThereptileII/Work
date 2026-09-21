# Pristine OpenCPN baseline

Status: initial dual-mode development slice has passed both platform gates at
`c5a0fd0`; this is not production-release acceptance.

## Provenance

- Upstream: https://github.com/OpenCPN/OpenCPN
- Tag: `Release_5.12.4`
- Commit: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
- Plugin API: **1.20**, verified in `include/ocpn_plugin.h:68-69`.
- Source: pinned submodule `upstream/OpenCPN`, no tracked modifications.
- Approved Windows target: x86 application/plugin ABI on native Windows x64.
  See ADR-001 for the user's explicit override of the literal x64 app requirement.

## Linux development environment

Omarchy/Arch x86_64, Linux 7.2.5-3-omarchy, GCC 16.2.1, CMake 4.4.3,
Ninja 1.13.2, wxGTK 3.2.11, GTK 3.24.52, GLEW 2.3.1. Missing development
packages are extracted into project-local `.local/sysroot`, not system paths.
Current rootless dependency package hashes are in `evidence/local`.

Commands from the project root:

```bash
source tools/local-env.sh  # only for the optional project-local Arch packages
bash tools/build-pristine-linux.sh
cmake -S . -B build/contracts -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/contracts
ctest --test-dir build/contracts --output-on-failure
```

The script records exact configuration/build/test output. It uses Release,
`OCPN_BUILD_TEST=ON`, a local install prefix, and CMake's compatibility floor
3.5 for older bundled dependencies. It bundles genuine upstream coastline and tide resources; docs are not bundled. Local wxGTK webview is disabled because that optional dependency is
not installed. These are baseline limitations, not approved release omissions.
Upstream's GTK version probe omits `wxWidgets_CONFIG_OPTIONS`, so
`tools/wx-config-local` supplies the relocated prefix to every probe.

Paths:

| Item | Development location |
|---|---|
| Binary | `build/pristine-linux/opencpn` |
| Installed binary | `build/pristine-install/bin/opencpn` |
| Installed shared data | `build/pristine-install/share/opencpn` |
| Normal Linux profile | OpenCPN's wxStandardPaths, normally `~/.opencpn` |
| Visual test profile | new directory under `build/profiles`, passed via `--configdir` |
| Test configuration | `<test-profile>/opencpn.conf` |
| User charts | configured by OpenCPN, no charts added or moved |

Never launch a development build against a real navigation profile by default.

## Windows validation

Workflow: `.github/workflows/opennav-baseline.yml` on the `opennav-x` branch
of https://github.com/ThereptileII/Work. The workflow lives at repository root;
project files live under `opennav-x/`. Both platform jobs use the same commit.

Native runner: `windows-2022`, Visual Studio 17 2022, `-A Win32`, Release,
wxWidgets 3.2.8 vc14x DLLs and OpenCPN core build-support v0.5, matching upstream.

```powershell
./tools/build-pristine-windows.ps1 -Architecture Win32
```

The script configures, builds, installs in a disposable prefix, runs CTest,
records executable SHA-256 and captures a native 1280x800 window. Full logs and
screenshots are uploaded as commit-labelled CI artifacts. A captured image
still requires visual review. DLL/plugin behavior, DPI and actual mode switching
remain separate required checks.

First Windows attempt discovered Poedit now installs to `Program Files`, while
upstream assumes `Program Files (x86)`. Our wrapper discovers either actual
location without modifying OpenCPN. Native output is archived explicitly.

Windows default config is `opencpn.ini` resolved by OpenCPN's platform code.
Explicit `--configdir` uses `opencpn.conf` even on Windows. No OpenNav profile
copy or alternate route database will be introduced.

## Source inventory and integration constraints

| Concern | Inspected source / reuse point |
|---|---|
| CLI startup | `gui/src/ocpn_app.cpp`, `MyApp::OnInitCmdLine`, `OnCmdLineParsed` |
| Existing recovery | `gui/src/safe_mode_gui.cpp`, `model/src/safe_mode.cpp`; upstream flag is `--safe_mode` |
| Profile authority | `model/src/base_platform.cpp`, `GetPrivateDataDir`, `GetConfigFileName` |
| Configuration | `gui/src/navutil.cpp`, `MyConfig::LoadMyConfigRaw`, `UpdateSettings` |
| Frame lifetime | `gui/src/ocpn_frame.cpp`, `MyFrame`, `OnCloseWindow` |
| Canvas hosting | `MyFrame::CreateCanvasLayout`, wxAUI centre pane; preserve primary ChartCanvas and multi-canvas support |
| Chart rendering | `gui/src/chcanv.cpp`; leave chart rendering/symbology intact |
| Legacy actions | `gui/src/toolbar.cpp`, `canvasMenu.cpp`, `options.cpp`, `connections_dlg.cpp` |
| Route/AIS UI | `gui/src/route_*`, `routemanagerdialog.cpp`, `ais_*` |
| Domain/navigation objects | `model/src`, including `navobj_db`; use existing storage/model |
| Plugin ABI/manager | `include/ocpn_plugin.h`, `gui/src/pluginmanager.cpp`; preserve loading and lifecycle |
| Shutdown persistence | `OnCloseWindow` saves config, navigation and AUI state; it can refuse close during initialization/chart processing |
| Healthy startup | upstream `startcheck.dat` recovery mechanism; do not confuse startup success with graceful shutdown |

Mode plumbing must account for upstream `--safe_mode` as well as OpenNav's
`--safe-mode`, reject ambiguous normal flags, preserve command-line profile
arguments, and wait for successful graceful shutdown before relaunch. Module
blocking must happen before any new hardware/control objects can be created.
The independent startup-policy tests do not yet establish runtime behavior.

## Evidence status — 2026-09-21

- [Run 35618922901](https://github.com/ThereptileII/Work/actions/runs/35618922901),
  commit `f81d54402edeace7406dcb100bf4291f7d9a4535`: all seven CI jobs passed,
  including native MSVC Win32 pristine and integrated builds on Windows x64,
  portable contracts on both platforms and strict integrated Linux regressions.
- Reviewed native Windows 1280×800 screenshots: XNav unavailable, day/dusk/night,
  explicit simulation, paused/stale data, Legacy, Safe Mode and return to XNav.
  No clipping or overlap at the tested 100% DPI setting. This is first-slice
  visual evidence, not acceptance of the complete visual design or release UI.
  Local artifact copy: `evidence/local/windows-f81d544/`; CI artifacts retain
  the full commit in their names. No real nautical charts or sensors loaded.
- Windows mode-cycle fixtures retain waypoint, route and track GUIDs, names,
  ordered coordinates/timestamps in the actual SQLite navigation database,
  disabled input-connection configuration and AIS CPA-warning configuration.
  Database integrity and foreign-key checks pass. This does not establish AIS
  target behavior, live connection behavior or third-party plugin compatibility.
- All four portable contracts pass on Linux and native Windows, including
  parent-exit ordering and restart argument preservation. Integrated Linux
  discovers and passes 60 compiled gtest cases. Pristine source-scanned CTest
  entry totals may include names without a compiled matching case.
- Pristine Linux's known registry/IPC test defects are reported by the baseline
  classifier, not counted as passing tests. The integrated test-only patch fixes
  callback ownership, event-loop exit, timeout and registry ownership assertions.
- A pristine Linux IPC quit crashed in APConsole::IsShown after the timer's
  ProcessQuitFlag closed the frame. The guarded integration returns after close;
  individual XNav, Legacy and Safe IPC-close checks pass locally. Backtrace:
  `evidence/local/pristine-shutdown-backtrace.log`.
- A subsequent Linux canvas-menu mode switch exposed synchronous canvas deletion
  during menu-handler unwinding. OpenNav now queues the close. Local shared-profile
  cycling reaches XNav → Legacy → XNav → Safe → persisted XNav with clean exits.
  Its initial IPC test also exposed a test-profile socket pathname over Linux's
  107-byte limit; the fixture now uses a short private temporary path with spaces.
- The integrated Safe Mode hooks also preserve bundled plugin enabled preferences;
  the expanded Linux cycle passes with Dashboard enabled throughout.
- Run GUI interaction tests after CTest, not concurrently: the application and
  upstream REST tests use the same port 8443. A parallel local attempt reached
  the GUI server and failed authentication; its log is retained separately.
- These subsequent fixes and the expanded same-profile Safe Mode/native process
  exit assertions require a new same-commit Windows gate before slice acceptance.
- Hosted Linux's Azure kernel has no vcan module; real CAN tests remain unavailable.
- Real-chart regression, representative plugins, non-default Windows DPI,
  hardware interaction and installer lifecycle remain release gates. SmartNav,
  hardware adapters and installer integration have not started.

## First dual-mode slice acceptance

[Run 35637082041](https://github.com/ThereptileII/Work/actions/runs/35637082041),
commit `c5a0fd0b34126da0c93bdaaf78970e8eb03c3c7e`, passed the Linux integrated
regression/mode-cycle gate and native MSVC integration gate. Native Windows
confirms clean exits, same-profile Safe override, saved XNav preference after
Safe, and the enabled Dashboard preference preserved through every transition.
Reviewed 1280×800 captures report 96 DPI. Review and image hashes are recorded in
`docs/evidence/windows-c5a0fd0-review.json`. This closes the first shell/mode
slice only. The release limitations above remain open.

The read-only selected-navigation slice also passed both gates at `bc0af30`;
see `docs/navigation-data-bridge.md` for native screenshot review and limitations.
The first advisory energy calculation component is documented in
`docs/energy-model.md`; no energy values are yet displayed in the shell.
