# Pristine OpenCPN baseline

Status: baseline validation in progress; not a completed product milestone.

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

- Pristine Linux Release build and local install succeed. Normal Ctrl+Q exit
  succeeds; the genuine coastline baseline is `evidence/local/11-legacy-mode-linux.png`.
- Native Windows MSVC Win32 build and install succeed. Run 35613737769,
  commit a2c9d6630bbddd314036a2796726a48863e9e114: **60/60 CTest entries pass**.
  Its screenshot exposed a missing tide-data startup modal and is rejected.
  Resource bundling and deferred-startup checks are being revalidated.
- Portable startup, vessel-quality and Windows-argument contracts pass on both
  platforms. The added process-lifecycle test currently passes locally; native
  Windows execution of this new test is pending.
- Local Swedish-locale test passes after generating the locale. Linux upstream
  DriverRegistry.RegisterDriver fails even with vcan0: its test calls Close()
  then expects registry removal, but removal belongs to Deactivate().
- Upstream IpcServer.Commands fails/aborts on Linux. Its test has a callback
  captured by reference beyond parameter lifetime; investigation remains open.
  These failures are retained as failures, not silently marked passed.
- Upstream CMake scans test source and can register tests disabled by #ifdef;
  CTest entry totals are not a claim that every entry executed a gtest case.
- A pristine IPC quit produced SIGSEGV in APConsole::IsShown from
  MyFrame::OnFrameTimer1. ProcessQuitFlag can close the frame, after which the
  timer handler continues. Normal GUI close works. Backtrace retained in
  `evidence/local/pristine-shutdown-backtrace.log`.
- Run 35616403676, commit 324a72b67fb6de7399ccc244d8d83383805f0bb7, introduces
  the first guarded integration build. Windows results pending. Linux CI
  dependency setup found the hosted Azure kernel has no vcan module; this is
  an environment limitation, not a reason to claim CAN tests passed.
- Real-chart regression, route/waypoint/track/AIS/connection persistence,
  plugin behavior, non-default DPI and mode-switch interaction remain pending.
- The dual-mode slice is **in progress**. SmartNav, hardware adapters and
  installer integration are not started. No production-release acceptance.
