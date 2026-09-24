# OpenNav X — Codex Project Specification

**Revision 3 — Dual Interface + Visual Design + Linux/Windows Development Workflow**

> Modern Marine Navigation Interface for OpenCPN  
> Target: Windows PC with existing OpenCPN installation  
> Development: Codex primarily on Linux; native Windows x64 is the authoritative release-validation platform  
> Reference baseline: OpenCPN 5.12.x; pin exact supported builds

## Executive summary
OpenNav X is a productization layer for OpenCPN. Keep OpenCPN's mature chart engine, routes, tracks, waypoints, AIS, connections and plugin ecosystem. Replace the desktop-style UX with a modern touch-first marine interface. Put new propulsion, prediction and navigation intelligence into separate modules, not into the UI.

The final product must install after OpenCPN using `OpenNavX-Setup.exe`, detect the exact OpenCPN version, validate compatibility, back up affected files, install the integration and companion modules, run self-tests, and provide update, repair, rollback, diagnostics and clean uninstall. The same installation must offer **XNav**, **Legacy OpenCPN**, and **Safe Mode** over the same OpenCPN navigation data. Codex may run primarily on Linux, but every UI/integration milestone must pass a native Windows x64 build and Windows visual/platform validation before it is considered complete.

## Architecture
```text
OpenNav UI
  |
  +-- Vessel Data API <--- NMEA2000 / NMEA0183 / Signal K / plugin messages
  +-- SmartNav Engine <--- routes / charts / AIS / vessel state / energy model
  +-- Control Adapters
  |     +-- Autopilot
  |     +-- Radar
  +-- OpenCPN Core
        +-- Chart engine
        +-- ENC/raster charts
        +-- Routes/tracks/waypoints
        +-- AIS
        +-- connections
        +-- plugin manager
```

## Engineering principle
UI does not contain navigation algorithms. SmartNav does not render widgets. Hardware adapters do not know layout. Prefer new files and narrow hooks over large invasive upstream rewrites.

## UI-only / mostly UI
- Top/status bars, touch controls, menus, cards and panels
- Modern AIS target card using existing AIS state
- Route/waypoint sidebars and context sheets
- Wind/depth/SOG/COG/rudder instruments
- Day/dusk/night theme
- Modern settings facade over existing configuration
- Autopilot/radar control presentation (actual commands belong to adapters)

## New modules
### Vessel Data
Canonical timestamped state with source, validity and age:
```cpp
struct VesselState {
  Navigation navigation;
  Environment environment;
  Wind wind;
  Propulsion propulsion;
  Battery battery;
  Rudder rudder;
  Tanks tanks;
  Connectivity connectivity;
};
```
Never display stale data as current. Missing values are unavailable, not zero. Prefer the existing ESP32 Leaf-CAN → NMEA2000 bridge and consume normalized marine data on the PC.

### SmartNav
Advisory only in the first release:
- next-turn prediction
- route-event timeline
- shallow-water look-ahead
- AIS encounter context
- energy/range/arrival-SOC prediction
- future sailing laylines/VMG
- future anchor intelligence

### Hardware adapters
Stable interfaces for autopilot, radar and future switching. UI sends high-level actions; adapters speak target protocols. Require acknowledgement/state feedback and prevent command storms.

## OpenCPN source starting points
Inspect the exact source checkout before editing:
- `gui/src/ocpn_frame.cpp` — top-level frame/lifecycle
- `gui/src/chcanv.cpp` — chart canvas
- `gui/src/toolbar.cpp` — legacy toolbar
- `gui/src/canvas_menu.cpp` — canvas context actions
- `gui/src/options.cpp` — legacy Options UI
- `gui/src/connections_dlg.cpp` — connection management
- `gui/src/ais_*` — AIS presentation
- `gui/src/route_*`, `routemanagerdialog.cpp` — route UI
- `gui/src/pluginmanager.cpp` — preserve unless absolutely necessary
- `model/src/*` — reuse domain model; extend only behind tests

## Installer flow
1. Detect OpenCPN install.
2. Read exact version/architecture/hashes.
3. Match compatibility manifest.
4. Refuse unknown builds.
5. Back up every modified/replaced file.
6. Install OpenNav-owned binaries/resources.
7. Apply version-specific integration package.
8. Install dependencies if needed.
9. Migrate config.
10. Run post-install self-test.
11. Launch and provide diagnostics on failure.

Required modes: Install, Update, Repair, Rollback, Diagnostics, Uninstall.

## Compatibility manifest
```json
{
  "openNavVersion": "0.1.0",
  "supportedOpenCpn": [
    {
      "version": "5.12.4",
      "arch": "x64",
      "executableSha256": "...",
      "integrationPackage": "win-x64-ocpn-5.12.4",
      "pluginApiMin": "1.20"
    }
  ]
}
```

## Codex workflow
1. Pin the exact upstream OpenCPN tag/commit.
2. Build pristine OpenCPN in the Linux Codex environment and record the Linux toolchain/commands.
3. Establish native Windows x64 validation immediately: Windows CI and/or a disposable Windows VM/test machine using the supported OpenCPN Windows/MSVC toolchain.
4. Build the same pristine pinned OpenCPN revision on Windows before accepting invasive XNav core-integration work.
5. Capture baseline screenshots; Windows 1280×800 is the visual authority.
6. Add OpenNav skeleton/build flags and platform abstractions without changing behavior.
7. Add XNav/Legacy/Safe Mode plumbing.
8. Build the shell around the unchanged chart canvas and add design tokens/custom controls.
9. Replace workflows incrementally.
10. Add Vessel Data + simulator.
11. Add SmartNav one feature at a time behind cross-platform tests.
12. Add hardware adapters after simulation works.
13. Require Linux and Windows gates for every meaningful milestone.
14. Build the installer only after a reproducible native Windows build exists.
15. Test install/update/repair/rollback/uninstall on a disposable Windows environment.
16. Validate the release candidate on the representative Windows navigation PC at 1280×800.

## Continuous verification
For every milestone retain:
- clean build log
- unit/integration test results
- successful launch log
- 1280×800 screenshots
- visual regression diff summary
- interaction smoke-test checklist
- simulator expected-vs-actual data
- stale-sensor test
- installer/repair/rollback logs and hashes
- Linux development build/test log
- native Windows x64 build/test log for the same commit
- Windows 1280×800 screenshots for visual changes
- Windows DPI/font/plugin/DLL/mode-switch smoke-test results
- target-system release-candidate checklist when applicable

## MVP
Include:
- OpenNav shell and styling
- unchanged chart engine underneath
- AIS target card
- route/waypoint context UI
- Vessel Data
- wind/depth/SOG/COG/rudder instruments
- electric propulsion/battery page
- basic energy/range/arrival SOC
- autopilot status + safe manual commands
- installer/update/repair/rollback/diagnostics

Defer:
- autonomous steering
- radar/AIS fusion
- complex weather routing
- ML navigation
- Lidar docking intelligence

## Repository layout
```text
opennav-x/
  upstream/OpenCPN/
  src/ui/
  src/vessel/
  src/smartnav/
  src/adapters/
  src/integration/
  src/platform/
    linux/
    windows/
  plugins/
  installer/windows/
  tests/
  tools/
  docs/
```

## Definition of done
A non-developer can install supported OpenCPN then OpenNav X with one setup program. The main workflow looks like a finished modern navigation product, OpenCPN navigation capabilities remain functional, vessel data is unified and inspectable, propulsion/energy features work, basic SmartNav features are tested, autopilot integration is safely isolated, and install/update/repair/rollback/uninstall all pass on clean Windows. The release commit must pass both the Linux development gate and the native Windows x64 build/test gate. Primary UI screens must pass Windows 1280×800 visual review, including Windows-specific DPI/font/runtime behavior.

## Agent goal statement
Build OpenNav X: a polished, touch-first Windows marine navigation product that installs on top of a supported existing OpenCPN installation and transforms its user experience while preserving OpenCPN's proven chart, route, waypoint, AIS, connection and plugin capabilities. The product must support three interface/recovery modes over the same OpenCPN navigation state: XNav, the modern default interface; Legacy, the original OpenCPN interface retained as a fully usable fallback and diagnostic reference; and Safe Mode, which starts the Legacy interface with OpenNav SmartNav and hardware-control modules disabled. Initial switching may use a controlled restart and must be available from both interfaces plus command-line overrides --xnav, --legacy and --safe-mode. Implement OpenNav as a maintainable OpenCPN GUI integration plus clearly separated Vessel Data, SmartNav and hardware-adapter modules. The first release must provide a modern chart-first interface matching the OpenNav visual design system, contextual AIS and route controls, day/dusk/night themes, unified vessel instrumentation, electric propulsion and battery presentation, basic energy and arrival prediction, and safe manual autopilot integration. Deliver a professional Windows installer/updater that detects and verifies the OpenCPN version, backs up modified files, installs the correct integration package, creates XNav/Legacy/Safe Mode launch shortcuts, runs self-tests, and supports repair, rollback, diagnostics and clean uninstall. Use the Linux Codex environment as the primary development workspace, but treat native Windows x64 as the authoritative release platform: establish Windows CI/VM validation at the start of integration work, require the same commit to build and test on both platforms, and require Windows screenshots/platform tests for UI completion. Work incrementally from a pinned upstream OpenCPN build; keep XNav and Legacy runnable after every milestone; verify changes with builds, automated tests, simulator-driven data, 1280×800 screenshots, interaction smoke tests, mode-switch tests and installer tests. Never patch unknown OpenCPN versions, never fabricate unavailable sensor/navigation data, and do not put autonomous steering or safety-critical decisions into the initial release. The finished result must look and behave like a coherent commercial marine navigation product, not a collection of plugins or a developer skin.

## First instructions
1. Read this specification completely, including the Dual Interface, Visual Style and Cross-Platform Development sections.
2. Clone/pin the exact OpenCPN upstream version selected for support.
3. Build unmodified OpenCPN successfully in the Linux Codex environment and document the exact Linux toolchain/commands.
4. Establish native Windows x64 validation immediately (Windows CI and/or a disposable Windows VM/test machine) using the supported OpenCPN Windows/MSVC toolchain.
5. Build the same unmodified pinned OpenCPN commit successfully on Windows before invasive XNav core integration.
6. Create `docs/baseline.md` with versions, Linux and Windows toolchains, exact build commands, plugin API, paths and pristine Windows 1280×800 screenshots.
7. Inventory relevant GUI/model/plugin files and platform-specific paths.
8. Add mode plumbing first: `--legacy`, `--xnav`, `--safe-mode`.
9. Create OpenNav module skeleton/build flags and `src/platform/linux` + `src/platform/windows` abstraction points.
10. Implement first vertical slice: XNav shell + unchanged chart canvas + top bar + simulated right data rail + day/night toggle + `Open Legacy OpenCPN`.
11. Add `Switch to XNav` in Legacy using a controlled restart.
12. Run Linux build/tests, then build the same commit on Windows and capture the required Windows screenshots.
13. Verify routes, waypoints, AIS and connection configuration persist across XNav → Legacy → XNav on Windows.
14. Do not mark the slice complete until both platform gates pass.
15. Do not begin SmartNav or invasive installer patching until the first dual-mode slice is stable on Windows.

## Primary upstream references
- https://github.com/OpenCPN/OpenCPN
- https://github.com/OpenCPN/OpenCPN/releases
- https://opencpn-manuals.github.io/main/
- https://opencpn-manuals.github.io/main/ocpn-dev-manual/0.1/pm-plugin-api-overview.html
- https://opencpn-manuals.github.io/main/opencpn-dev/plugin-api.html
- https://opencpn-manuals.github.io/main/ocpn-dev-manual/0.1/pm-plugin-api-versions.html
- https://opencpn-manuals.github.io/main/plugin-installer/Tarballs.html
- https://opencpn-manuals.github.io/main/plugin-installer/Installation-paths.html


---

# Mandatory Product Additions — Dual Interface and Visual Design

## Dual interface and recovery architecture

**Product name:** OpenNav X  
**Modern interface mode:** XNav  
**Fallback/reference interface:** Legacy OpenCPN  
**Recovery launch:** Safe Mode

All three modes operate on the same OpenCPN installation/profile and the same navigation state.

```text
OpenCPN core / user data
        |
        +----------------------+----------------------+
        |                      |                      |
        v                      v                      v
     XNav UI              Legacy UI               Safe Mode
 modern OpenNav shell     stock OpenCPN UI         Legacy UI
 OpenNav modules ON       reference/fallback       SmartNav OFF
                                                  controls OFF
```

### Startup precedence
1. `--safe-mode`
2. explicit `--xnav` or `--legacy`
3. persisted `InterfaceMode`
4. first-run default = XNav

### Mode rules
- Persist `InterfaceMode=xnav|legacy` in an OpenNav-owned config namespace.
- Safe Mode is an override and is not persisted by default.
- Create shortcuts: **OpenNav X**, **OpenCPN Legacy**, **OpenNav Safe Mode**.
- XNav must expose **System → Open Legacy OpenCPN**.
- Legacy must expose **Switch to XNav**.
- v1 may switch through a graceful restart. Prefer this over live destruction/re-parenting of the entire wxWidgets hierarchy.
- Never duplicate charts, routes, tracks, waypoints, AIS state or connection databases to implement modes.
- XNav-specific presentation preferences may be separate; navigation data is shared.
- Safe Mode must not emit autopilot/radar/control commands.
- If XNav repeatedly fails before a healthy-start marker, offer Legacy/Safe recovery instead of a crash loop.

### Required mode-switch test
1. Start XNav.
2. Create/edit a route and observe AIS/sensor state.
3. Switch to Legacy.
4. Verify the same route/profile/connections are present.
5. Modify the route in Legacy.
6. Switch back to XNav.
7. Verify the Legacy change appears in XNav with no import/export.

## Visual style guideline

The concept board generated for the project is the approved aesthetic direction. The implementation must use design tokens and reusable components rather than screen-specific ad hoc styling.

### Design principles
- **Chart first:** the chart is the visual field; UI frames it.
- **Quiet by default:** minimal permanent chrome; contextual detail.
- **Touch first:** large hit areas; mouse/keyboard still supported.
- **Dark marine instrumentation aesthetic:** integrated panels, not Windows desktop dialogs.
- **Semantic color:** color indicates meaning/state, not decoration.
- **Progressive disclosure:** simple navigation first, technical data on demand.
- **One product language:** navigation, radar, autopilot, propulsion, anchor and settings use the same geometry, typography and state model.

### Core color tokens
| Token | Hex | Use |
|---|---|---|
| Background / Void | `#07141C` | Application surround and modal scrim base |
| Surface 1 | `#0B1922` | Cards, rails, top/bottom bars |
| Surface 2 | `#10232E` | Nested/selected surfaces |
| Surface Elevated | `#132B37` | Popovers and context cards |
| Border | `#284653` | 1 px structural borders/dividers |
| Text Primary | `#F2F6F8` | Important labels/values |
| Text Secondary | `#A9BAC3` | Units/helper labels |
| Text Muted | `#708791` | Inactive metadata/stale context |
| Navigation Cyan | `#00B8E6` | Selection/focus/navigation actions |
| Active Green | `#00E08A` | Connected/healthy/active states |
| Attention Amber | `#F5B942` | Advisories and confirmation-required state |
| Alarm Red | `#FF4D5A` | Critical/danger/destructive state |
| AIS/Selection Magenta | `#F04F9B` | Selected targets where separation from route cyan is useful |

### Day / dusk / night
- **Day:** OpenCPN day chart palette + dark OpenNav chrome. Do not turn the application into a white desktop theme.
- **Dusk:** reduce luminance/saturation, warm secondary text, use active colors sparingly.
- **Night:** near-black surfaces, deep red interaction accents, highly restrained white. No surprise bright white primary dialogs.

### Typography
Windows v1 recommendation: **Segoe UI Variable / Segoe UI**. Do not require an extra font installation.

| Role | Target size | Weight |
|---|---:|---:|
| App/page title | 22–28 px | 700 |
| Panel heading | 16–18 px | 600–700 |
| Primary numeric value | 28–40 px | 650–750 |
| Standard UI | 14–16 px | 400–600 |
| Card label | 11–12 px | 600 |
| Metadata | 11–12 px | 400 |

Use tabular numerals when practical. Units are subordinate to the main value.

### 1280×800 layout reference
```text
+--------------------------------------------------------------+
| Top status bar: 40-44 px                                     |
+----+---------------------------------------------------+-----+
|    |                                                   |     |
| L  |                  CHART CANVAS                     | R   |
| 56 |                                                   |100- |
| px |                                                   |140px|
|    |                                                   |     |
+----+---------------------------------------------------+-----+
| Bottom action bar: 52-60 px; context expansion up to ~132 px |
+--------------------------------------------------------------+
Outer margin: 8 px   Standard gap: 8 px   Compact gap: 4 px
```

Do not hard-code the whole UI to 1280×800. Test Windows 100%, 125% and 150% scaling.

### Geometry / touch tokens
- Base spacing: **8 px**
- Compact spacing: **4 px**
- Panel radius: **8 px**
- Small control radius: **6 px**
- Border: **1 px**
- Minimum touch hit target: **48×48 px**
- Primary action height: **52–56 px**
- Icon size: **20–24 px**

### Panels / cards
- Surface + 1 px border; avoid heavy shadows.
- Floating context cards may have a subtle elevation shadow.
- One dominant value/decision per card.
- Avoid dashboard tile soup.
- Context cards should remain near the selected chart object without obscuring it when practical.

### Buttons / state
- Default: dark fill + Border outline.
- Focus: visible Navigation Cyan ring.
- Selected navigation: Cyan.
- Active/healthy: restrained Green.
- Attention/pending: Amber.
- Critical/destructive: Red and deliberate confirmation.
- Disabled: 35–45% emphasis but still readable.

### Icons
- One coherent SVG/vector outline family.
- ~2 px nominal stroke at 24 px.
- Rounded caps/corners.
- No emoji as production icons.
- Every icon-only button has a tooltip/accessibility name and ≥48 px hit box.

### Data quality states
| State | Treatment |
|---|---|
| Live | Normal primary value; optional green source dot |
| Aging | Secondary value / show age if material |
| Stale | Muted + explicit stale indicator; SmartNav stops relying on it |
| Unavailable | `—` / `--`, never fake zero |
| Estimated | Mark `EST` / prediction symbol |
| Uncertain | Show qualifier/confidence or suppress advice |

### Motion
- Normal transitions: **120–180 ms ease-out**.
- Use slide/fade only when spatially useful.
- No decorative chart animation.
- No pulsing except genuine alarm/time-sensitive attention.
- Hardware-control UI reflects acknowledged state, not optimistic animation alone.

### Screen patterns
- **Main navigation:** chart dominant, narrow left tools, right telemetry, bottom action/AP strip.
- **AIS:** elevated target card; identity + CPA/TCPA first.
- **Route:** active route clear; destination DTW/ETA/XTE/BTW + next maneuver.
- **Radar overlay:** chart remains legible; compact controls.
- **Autopilot:** large target heading; `-10`, `-1`, `+1`, `+10`; clear STANDBY/TRACK/WIND.
- **Sailing:** wind/TWA/VMG dominant; avoid unnecessary fake analog gauges.
- **Propulsion:** SOC/power dominant; range and arrival SOC visibly marked as predictions.
- **Anchor:** boat-centered swing display + radius/depth/wind/battery.
- **Settings:** left navigation rail + right content; raw data is advanced, not default.

### Alert hierarchy
- Informational: neutral/cyan, non-blocking.
- Advisory: amber, inspectable/dismissible where appropriate.
- Warning: persistent amber/red depending severity; relevant chart object highlighted.
- Critical: red, highest z-order, concise action language, never hidden by another XNav panel.

### Implementation rules
- Centralize tokens in one theme/design-token module.
- Build reusable primitives before screens: `XNavButton`, `XNavCard`, `XNavToggle`, `XNavDataValue`, `XNavStatusDot`, `XNavPanel`, `XNavContextCard`.
- Use vector assets and DPI scaling.
- Do not modify official OpenCPN chart symbology merely to match the chrome.
- Keep Legacy styling stock/near-stock for upstream maintainability.

## Dual-mode verification matrix
| Scenario | XNav | Legacy | Safe Mode |
|---|---|---|---|
| Cold start | Modern shell + same profile | Stock shell + same profile | Legacy shell; OpenNav SmartNav/control OFF |
| Charts | Same installed charts | Same installed charts | Same installed charts |
| Routes | Shared OpenCPN data | Same data | Read/edit through Legacy |
| AIS | XNav card | Stock AIS UI | Stock AIS UI |
| Sensor loss | Explicit stale/unavailable | Upstream behavior | No OpenNav advisory/control output |
| Mode switch | Set mode + restart | Set mode + restart | Override not persisted by default |
| Autopilot commands | Adapter + acknowledgement required | No new XNav UI required | Disabled |
| XNav failure | Offer Legacy/Safe | Remains usable | Recovery path |

## Visual QA screenshot set
Codex should produce deterministic simulator/demo screenshots:
- `01-main-navigation-day.png`
- `02-ais-target-card.png`
- `03-route-navigation.png`
- `04-radar-overlay.png`
- `05-autopilot-control.png`
- `06-sailing-instruments.png`
- `07-propulsion-energy.png`
- `08-anchor-mode.png`
- `09-settings-connections.png`
- `10-main-navigation-night.png`
- `11-legacy-mode.png`
- `12-safe-mode-diagnostics.png`

A visual milestone is complete only when there is no overlap/clipping, the design tokens are followed, the chart remains readable, stale/unavailable data is correct, and no accidental native Windows controls appear in the primary XNav workflow.


---

# Revision 3 Mandatory Addition — Linux Development / Windows Release Validation

## Principle
The expected working arrangement is **Codex on Linux, product on Windows**. This is supported and should be the default workflow. Linux is the fast development environment. Native Windows x64 is the authoritative integration, visual, packaging and release environment.

```text
                         Git repository
                              |
             +----------------+----------------+
             |                                 |
             v                                 v
      Linux Codex workspace               Windows validation
      ---------------------               ------------------
      source editing                       native MSVC build
      Linux OpenCPN build                  Windows wxWidgets
      SmartNav/unit tests                  DLL/plugin loading
      Vessel Data simulator                1280x800 screenshots
      static analysis                      DPI/font/touch checks
             |                                 |
             +----------------+----------------+
                              |
                              v
                       Windows release
                   installer + target-PC QA
```

## Platform responsibility matrix

| Area | Linux Codex | Windows authority |
|---|---|---|
| SmartNav algorithms | Primary development/testing | Confirm same tests pass |
| Vessel Data/simulator | Primary development/testing | Runtime confirmation |
| Shared C++ / OpenCPN integration | Compile continuously | Native MSVC build required |
| wxWidgets layout | Useful feedback | Final visual authority |
| Fonts / DPI scaling | Approximate | Required verification |
| Plugins / ABI / DLL loading | Source/logic work | Native runtime test required |
| XNav / Legacy / Safe Mode | Shared logic | Restart/shortcut/runtime test |
| Registry / Program Files / UAC | Interface/abstraction only | Windows implementation/execution |
| Installer/update/repair/rollback | Manifest/logic design | Build and execution required |
| Touch / navigation-PC behavior | Simulation | Release-candidate target test |
| N2K/radar/autopilot hardware | Fixtures/simulation | Final Windows/boat integration |

## Two-gate milestone rule

```text
GATE A - LINUX DEVELOPMENT
  configure/build succeeds
  unit/integration tests pass
  simulator fixtures pass
  shared code remains portable

GATE B - WINDOWS RELEASE VALIDATION
  same commit builds natively on Windows x64
  relevant tests pass
  plugin/DLL/runtime integration succeeds
  XNav and Legacy both launch
  Windows 1280x800 screenshot reviewed when UI changed
  affected Windows-specific behavior is tested

A feature is not DONE until every applicable gate passes.
```

## Windows validation environment
- Maintain native Windows x64 CI and preferably a disposable Windows VM or physical test system.
- Use the supported OpenCPN Windows/MSVC toolchain and the same pinned upstream revision as Linux.
- Wine is not release validation.
- Linux-to-Windows cross-compilation is not the canonical release build unless separately proven compatible with the native OpenCPN/plugin ABI.
- Keep Windows build/configuration scripted so Codex can fix failures from CI logs while remaining on Linux.

## Windows visual rules
- Canonical visual reference: Windows, 1280×800.
- Capture standard XNav screen set in day/night modes after major visual changes.
- Test 100% scaling and at least one non-100% Windows DPI/scaling setting before release.
- Verify text wrapping, font fallback, glyphs, custom-control geometry, mouse/touch states, system-dialog behavior and window chrome.
- Linux screenshots are development evidence only, never final visual approval.

## Platform abstraction
Windows-specific behavior must stay isolated from SmartNav, Vessel Data and normal UI logic.

```text
src/platform/
  PlatformIntegration.h
  linux/LinuxPlatformIntegration.cpp
  windows/WindowsPlatformIntegration.cpp
```

Typical platform services include restart-by-mode, locating OpenCPN, resolving config/log paths, opening diagnostics, and Windows-only installer/elevation/shortcut integration.

## Windows CI minimum gate
1. Checkout the exact same commit tested on Linux.
2. Configure/build OpenCPN/OpenNav x64 using the native supported toolchain.
3. Build companion plugins/modules.
4. Run unit/integration tests.
5. Archive compiler/test logs and version provenance.
6. For packaging branches, build the installer and retain its hash.
7. Fail the milestone when Windows fails even if Linux passes.

## Target Windows release-candidate test

```text
Install supported stock OpenCPN
        -> Install OpenNav X
        -> Launch XNav
        -> Verify 1280x800 UI / DPI / fonts
        -> Switch XNav -> Legacy -> XNav
        -> Launch Safe Mode
        -> Verify plugins / simulator / device data
        -> Repair
        -> Update
        -> Rollback
        -> Uninstall
        -> Confirm stock OpenCPN remains usable and user navigation data is preserved
```

## Completion rules for Codex
- Never report a UI/integration feature finished from Linux-only evidence.
- If Linux passes and Windows fails, the feature remains in progress; the Windows log becomes the next debugging input.
- Pure platform-independent SmartNav/Vessel Data logic may be logically complete after Linux tests, but it cannot enter a release candidate until the Windows pipeline confirms the same commit.
- Installer, shortcuts, Registry/UAC, Windows paths, plugin DLL loading, DPI and final visual behavior require Windows execution evidence.
- Keep fixes portable and shared whenever possible; use conditional compilation only at explicit platform boundaries.
