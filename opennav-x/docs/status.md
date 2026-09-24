# Development status — 2026-09-25

**Development build; production Definition of Done remains open.** The selected
baseline is OpenCPN 5.12.4 at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The approved Windows target preserves its Win32 application/plugin ABI on a
native Windows x64 host. The tracked upstream submodule remains pristine;
reviewed patches apply to a disposable integration worktree.

**Current native product/DPI candidate:** `a749e8d2a38bec811b0b011919bf4ea383388343`
passes 90 Linux / 80 Windows integrated tests and 27 portable contracts per
platform. Native package interactions, manual DEMO pilot feedback, settings and
actual 100/125/150% DPI/touch assertions pass. Eight replacement native images
were reviewed; UTF-8 labels and scaled coastline returns are correct. The full
candidate fails the new ENC fixture: its unescaped Windows configuration path
leaves the upstream chart database empty. The fixture now writes forward-slash
paths, consistent with pinned wxFileConfig escaping; native rerun is required.
No empty chart is accepted. [Evidence](evidence/windows-alpha-a749e8d-review.json).
Historical pending records below remain for traceability; this is not Alpha release acceptance.

**Marine input gate accepted:** `785aa451293a8e137c130ef63829f3e5997bbabd`
passes all eight jobs in [run 36057131335](https://github.com/ThereptileII/Work/actions/runs/36057131335).
Linux passes 81 integrated cases and Windows 71; both pass 22 portable contracts.
The new loopback instrument scenario verifies heading, STW, wind, depth, rudder
and temperature, including stopped and invalid sensor reports. Native live/stale/
unavailable screens and two return-to-XNav coastline captures were reviewed.
The failed predecessor and explicit Windows clock-precision correction remain
recorded in [gate evidence](evidence/alpha-marine-785aa45-gates.json).

**Alpha product workflows in progress:** copied route/waypoint catalogs, guarded
human actions, deferred chart context cards, AIS, instruments, SmartNav timeline,
manual autopilot simulation and anchor-watch observation are implemented. The
`e4821d3` candidate passes Linux and native MSVC compilation/71 tests, but its
new Windows property-sheet automation misread a cross-process edit buffer.
[Failure record](evidence/alpha-products-e4821d3-candidate.json) is retained;
`bdb58f9` corrects the edit helper; its property-sheet assertions pass, but
Windows then exposed an unclosed SQLite verifier connection during cleanup.
[Second record](evidence/alpha-products-bdb58f9-candidate.json). `5c8428e` closes
that connection explicitly. Its native 76 integrated cases and object/settings
regressions pass; package interaction then exposed a selector matching the
AUTO confirmation heading instead of its identically labelled button. The
selector now excludes non-actionable static text; native rerun is pending. No assertion
was removed. This UI milestone is not yet accepted on Windows.

**Live settings increment in progress:** explicit battery/current/curve
configuration, per-quantity source selection and freshness, shared-profile
persistence, live energy consumers, vessel safety assumptions and radar status.
[Contract](alpha-settings-contract.md). Linux currently passes 25 portable and
86 integrated tests; expanded Linux preview and instrument loopback pass.
Native acceptance is pending. The installer,
broader chart/plugin/DPI validation and final Alpha packaging remain ahead.
This is not an Alpha release candidate.

**Native scaling validation in progress:** the disposable Windows gate now
requests actual monitor 100/125/150% scaling and verifies both Win32 and wx
DPI. It exercises native touch injection, sheets, day/night and Legacy/Safe
round trips with coastline assertions. This is pending native results; no
rescaled screenshot is treated as DPI acceptance.

**Startup recovery increment in progress:** the XNav startup journal selects
upstream Safe Mode after two unfinished starts and preserves human retry
records. Linux passes 26 portable and 89 integrated tests plus the actual
process-termination/recovery/persistence scenario, including two coastline
checks. Native 79 integrated cases and the actual recovery scenario now pass at
`0a1ed8d`; reviewed all three native recovery captures. Full candidate acceptance
is still blocked by the separate confirmation-selector package test.
[Native evidence](evidence/windows-alpha-recovery-0a1ed8d-review.json).
[Contract](startup-recovery.md).

**Chart/plugin/performance increment in progress:** Linux passes 89 integrated
and 26 portable cases plus the expanded preview and NOAA ENC software/OpenGL
scenario. Zoom, pan, selected-position follow, overlays and Legacy returns retain
real ENC quilt content. Dashboard/WMM/GRIB loader records pass. Linux OpenGL uses
llvmpipe, not a physical GPU. Native manager/route-gesture/chart gates are pending.
[Local evidence](evidence/alpha-chart-local-validation.json),
[contract](chart-plugin-performance-validation.md). The side-by-side
[installer design](installer-alpha-design.md) is recorded; implementation and
native lifecycle acceptance remain open.

**Explicit propulsion mapping increment in progress:** Data Sources imports
bounded, documented Signal K motor-temperature/electrical/shaft-power mappings,
stores them in the shared profile and clears observations after interpretation
changes. No mapping or Leaf CAN assumption is enabled by default. Linux passes
27 portable and 90 integrated cases, instrument loopback and the preview smoke.
Source-policy validation now uses the reducer's limits before saving.
[Contract](propulsion-source-mapping.md), [local evidence](evidence/alpha-mapping-local-validation.json).
Native acceptance remains pending.

**Alpha 1 foundation gate closed:** `16dbaf72d7923742a5acaadf0fae42ee490e7cf6`
passes all eight jobs in [run 36047288188](https://github.com/ThereptileII/Work/actions/runs/36047288188).
Linux passes 67 integrated tests, Windows 57; each passes ten portable contracts
and ten extra restart repeats. Existing selected-navigation, route, Demo,
lifecycle and persistence gates pass. Seven Linux and nine native Windows
chart-content checks cover direct XNav/Legacy/Safe startup, XNav → Legacy → XNav,
Safe → XNav and the old portable basemap setting. Reviewed all nine relevant
native 1280×800/96-DPI chart captures; none is blank/all-water. The chart-restart
regression is closed for this foundation. Higher DPI, OpenGL and nautical-chart
coverage are separate Alpha gates, not implied by this result.

[Windows review](evidence/windows-foundation-16dbaf7-review.json),
[Linux review](evidence/linux-foundation-16dbaf7-review.json),
[package audit](evidence/foundation-16dbaf7-package.json) and
[exact CI jobs/artifacts](evidence/foundation-16dbaf7-gates.json) preserve the
replacement evidence. [Foundation preview download](https://github.com/ThereptileII/Work/actions/runs/36047288188/artifacts/10830135079)
contains the verified application ZIP; all 997 file hashes pass and its
executable matches the native tested binary. It is still Developer Preview 0.1,
not the requested Alpha 1 deliverable. Alpha implementation now proceeds under
the [stage plan](alpha1-plan.md) and [inspected boundaries](alpha1-source-inspection.md).

**Alpha core gates passed:** `0e65cd5fade0cc8dd493ca734c61502325e912aa`
adds instrument source policies, coherent battery-current normalization and
calibrated propulsion curves. Its eight CI jobs pass with 15 portable suites
per platform, Linux 67 / Windows 57 integrated tests and native review of four
representative captures. [Gate](evidence/alpha-core-0e65cd5-gates.json),
[review](evidence/windows-alpha-core-0e65cd5-review.json).
`82efd086b5eaa752c0cbd7c3b90718e9bbd4c426` adds owned route steps,
advisory timeline/turn/energy/AIS events, the unavailable live chart-corridor
boundary, manual autopilot simulator/feedback and radar status interfaces.
Its eight jobs pass with 22 portable suites per platform and the retained
67 Linux / 57 Windows integrated regressions.
[Gate and test incident note](evidence/alpha-advisory-82efd08-gates.json).
These are internal core milestones, not an Alpha product release. See
[sources](vessel-source-contract.md), [energy](energy-model.md),
[SmartNav](smartnav-alpha-contract.md) and [adapters](hardware-adapter-contract.md).
Live command output remains disabled.

The marine bridge reuses OpenCPN's NMEA 2000, NMEA 0183 and Signal K input
bus. See the [marine input contract](marine-input-contract.md), including the
pinned battery voltage limit and compiled-out coolant PGN. Codec/loopback
acceptance does not imply a passed physical boat gate.

**Manual-preview feedback:** the user completed the Windows test and reported
one issue: the chart background disappears after Legacy → XNav. This is reproduced
with the accepted executable in portable mode. The previous mode gate verified
process/data persistence but missed coastline rendering after restart; its
all-water Legacy/Safe captures should not have been accepted as normal.
See [reproduction evidence](evidence/chart-restart-b21bd05-reproduction.json).
**The repair has passed Linux and native Windows gates, package tests and native
screenshot review at `bcc3fa2bd3ec01b3217a530c3079ae82e0683d0c`.** The replacement
download below supersedes the affected `b21bd05` preview.

The read-only remaining active-route distance slice has passed both platform
gates and native screenshot review; see [its contract](route-progress-contract.md).
**Developer Preview 0.1 has passed both platform gates, extracted-package tests
and native screenshot review.** It adds explicit deterministic Demo data,
advisory energy presentation and an isolated portable Windows package; see
[its contract](developer-preview-contract.md). It is ready for manual Windows
testing, not approved for navigation. Live battery acquisition and calibration
remain unavailable.

[Download the corrected Windows artifact](https://github.com/ThereptileII/Work/actions/runs/36044190692/artifacts/10829050279).
It contains `OpenNavX-DeveloperPreview-win64.zip` and its SHA-256 sidecar. Extract
the application ZIP into a short writable path and run `Run-XNav-Demo.cmd`.
Follow [TEST_ME_FIRST](preview/TEST_ME_FIRST.md). The preview uses its own profile
and bundled runtime; it does not patch an installed OpenCPN.

## Accepted development increments

| Increment | Evidence and practical limit |
| --- | --- |
| Initial XNav shell | Existing OpenCPN chart canvas, custom touch controls, day/dusk/night tokens, explicit simulator and unavailable/stale states. Native 1280×800 review at 96 DPI; full product screens and chart-content review remain open. |
| XNav / Legacy / Safe | Controlled restart, shared configuration/navigation database, Safe precedence, saved normal-mode preference, clean exits and bundled Dashboard enabled preference preserved. Synthetic waypoint/route/track/connection/AIS-setting fixtures survive the cycle. |
| Selected navigation | Read-only position/SOG/COG from OpenCPN's selected navigation bus. Real decoder input through an isolated synthetic NMEA loopback stream; position-only updates do not freshen old speed/course. Heading is withheld where provenance is insufficient. |
| Advisory energy core | Tested reserve-aware range and arrival SOC, explicit model assumptions, no prediction from invalid/stale/missing inputs, energy-shortfall reporting. Preview consumers now map valid owned snapshots into this model; live batteries and calibrated capacity/reserve remain unavailable. |
| Remaining active-route distance | Immutable owned snapshot copied around normal OpenCPN progress: current active-point range plus subsequent stored legs, NM, route/revision/active-point identity and separate observation/position times. Edits, skips, reversal, deletion, ambiguity and stale/invalid inputs withhold distance. Preview consumers retain this contract; Demo has explicitly synthetic provenance. |
| Developer Preview 0.1 | Navigation, Route, Energy and System views; expanded owned Vessel Data; eight deterministic Demo scenarios; advisory estimates; isolated portable Windows ZIP. Native page visibility, resize, plugin lifecycle and package launch checks pass. No navigation-use or production-installer approval. |

The shell/mode slice first passed both platforms at `c5a0fd0`; selected
navigation passed at `bc0af30`. Their native review records are in
[`docs/evidence`](evidence/). See [baseline](baseline.md),
[navigation bridge](navigation-data-bridge.md) and [energy model](energy-model.md)
for the inspected source boundaries and exact limitations.

## Accepted chart-restoration repair

Packaged revision: `bcc3fa2bd3ec01b3217a530c3079ae82e0683d0c`;
local equivalent: `9ede87671687c653c531e0307e237155536d6906`.
[Exact successful CI run](https://github.com/ThereptileII/Work/actions/runs/36044190692).
All eight jobs passed. OpenCPN remains pinned at 5.12.4 with its supported Win32
application/plugin ABI on Windows x64.

The existing post-config-load integration hook now resolves an empty portable
basemap default to existing bundled shapefiles before canvas creation. It repairs
the old preview's generated `./` setting when that directory contains no basemap
shapefiles. Custom locations, chart directories/databases and rendering remain
owned by OpenCPN. No additional upstream patch was needed.

- Linux: 67 integrated tests, ten portable contracts, ten extra restart repeats,
  mode/persistence, selected NMEA, 26 route observations and all eight Demo
  scenarios pass. Portable preview smoke now also exercises the Windows-style
  resource lifecycle; Legacy/returned-XNav/Safe coastline checks pass.
- Native Windows: 57 integrated tests, ten portable contracts, ten extra restart
  repeats and existing mode/navigation/route/Demo regressions pass. Extracted ZIP
  tests pass four coastline checks, including migration of the old setting on
  direct startup. Launcher/isolation, DLL and plugin lifecycle checks pass.
- Reviewed all 12 native preview captures at 1280×800/96 DPI. Coastline remains
  visible in Legacy, returned XNav, Safe and the migrated profile; other preview
  screens retain their layout and validity states. Higher DPI remains untested.
- The downloaded application ZIP is 48,389,585 bytes, SHA-256
  `2923f22853aa77c367f374246b7a0926d9ac91a534c512435adf91b8b90bf90a`.
  All 997 file hashes pass; its executable matches the native tested binary.
  All 112 compared local/remote source/build/test/patch inputs match.

[Windows review](evidence/windows-bcc3fa2-review.json),
[Linux review](evidence/linux-bcc3fa2-review.json),
[contracts](evidence/chart-fix-bcc3fa2-contracts.json),
[package audit](evidence/chart-fix-bcc3fa2-package.json),
[CI jobs/artifacts](evidence/chart-fix-bcc3fa2-gates.json) and
[source equivalence](evidence/source-mirror-bcc3fa2.json) preserve the evidence.
Documentation-only commits after this revision do not change the packaged binary.
The user has now accepted the preview direction and authorized the
[Alpha 1 stage](alpha1-plan.md). The expanded foundation gate is closed above.
Alpha functionality and the Alpha installer remain in progress; no Alpha
package has been accepted yet.

## Initial Developer Preview gate (superseded for the chart-restart bug)

Packaged code/test revision: `b21bd05ce75f22c91ac12927207d264b8b3efde4` on the
remote `opennav-x` branch; local equivalent:
`ff02ebd71d1fb67bedfb898489d2d8b5f76691df`.
[Exact CI run](https://github.com/ThereptileII/Work/actions/runs/35663416666).
All eight jobs passed. Documentation/evidence commits after this revision do
not change the tested executable or downloadable ZIP.

- Linux: 67 integrated regressions, nine portable contract suites and ten extra
  restart lifecycle repeats passed.
- Native Windows: MSVC 19.44.35228.0, supported Win32 ABI on Windows x64;
  57 integrated regressions, nine portable contract suites and ten extra restart
  lifecycle repeats passed.
- Both platforms: existing mode/persistence and synthetic NMEA checks,
  26 normal-timer route observations, eight Demo scenarios and UI interactions
  passed. Integrated JUnit reports have zero failures, errors or skips.
- Windows package: four launchers, direct executable startup, refused external
  profile, normal-profile canary, restricted runtime PATH and shared persistence
  passed. Dashboard initialized/unloaded in six normal launches and remained
  inactive in two Safe launches. All 87 installed PE files have resolved imports;
  four private-profile plugin copies match the installed binaries.
- Reviewed 15 native 1280×800/96-DPI images: ten preview captures covering Day,
  Night, Route, Energy, Diagnostics, stale, unavailable, shortfall, Legacy and Safe;
  plus five selected-navigation/route regression captures. No unintended clipping
  or overlap found. 125/150% DPI and touch remain untested.

The published application ZIP is 48,386,718 bytes, SHA-256
`cfa9e9e462aea63da6df5c8cce324d4fd8769e2799b0976d02931f03be0c8e58`.
The downloaded archive's 997 file hashes pass, and its executable matches the
native tested binary. Corresponding source is available in the same run as
`OpenNavX-DeveloperPreview-corresponding-source`. All 108 local source/build/test/
patch inputs compared with the remote revision match.

Evidence: [Windows review](evidence/windows-b21bd05-review.json),
[Linux review](evidence/linux-b21bd05-review.json),
[contracts](evidence/preview-b21bd05-contracts.json),
[package audit](evidence/preview-b21bd05-package.json),
[CI jobs/artifacts](evidence/preview-b21bd05-gates.json) and
[validation procedure](preview-validation.md).

Earlier candidates exposed a missing test marker, portable resource working
directory, fixture SQLite cleanup, invisible Windows content pages, a route-summary
resize overlap and the portable plugin directory. These were corrected and
retested before acceptance; candidate evidence remains under `docs/evidence`.
Long temporary paths still produce nonfatal upstream SVG cache warnings; the
shipped guide recommends a short writable extraction path. Hosted isolation
checks include a normal-profile canary but no preinstalled production OpenCPN;
real installed-application coexistence remains part of the manual PC review.

## Preceding accepted route-distance gate

Code/test revision: `954b4505e18e9128dc02e75cf05d0c02bdbad188` on the remote
`opennav-x` branch; local equivalent: `a7f2b33`.
[Native and Linux CI run](https://github.com/ThereptileII/Work/actions/runs/35648822128).

All seven CI jobs passed. Win32 and Linux portable lanes pass all seven contracts,
plus ten consecutive restart lifecycle checks each. Integrated application
regressions pass 57 compiled tests on Windows and 67 on Linux, including seven
tests against real upstream route objects and antimeridian geometry. The portable
route contract covers 28 scenarios; the normal-timer application fixture passes
26 observations on each platform, including deletion while active. Corresponding
states, identities, revisions and distances agree between Linux and Windows.
Shared-profile mode cycles and synthetic NMEA input checks also pass. The pristine
Linux baseline classifies its documented upstream test defects; those defects
are not counted as passing tests.

Reviewed native 1280×800/96-DPI captures and executable/artifact hashes are in
[the final Windows review](evidence/windows-954b450-review.json), with
[Linux evidence](evidence/linux-954b450-review.json) and the
[recorded route observations](evidence/route-954b450-observations.json). All 86 local
source/build/test/patch files match the tested remote revision. Documentation
commits after this code revision do not change the tested executable.

The first route candidate, `c923b87`, failed both integrated builds because its
model fixture accessed private icon collections. The fixture now initializes
those collections through the existing GUI friend boundary. A preceding local
teardown crash was traced with GDB to the missing GUI initialization. This repair
changes test setup, not upstream model access or navigation behavior. The corrected
implementation passed at `8062fb2`; the accepted revision above additionally
tests active deletion and waits for the stale UI refresh before capture.
The earlier restart-test publication repair and its regression history remain
recorded in [the prior evidence](evidence/restart-handoff-c2535f5.json).

## Remaining product and release gates

- Extend the initial Navigation/Route/Energy/System views with full route and
  waypoint editing, AIS, instruments and settings workflows. Preserve existing
  functionality through Legacy throughout that work.
- Acquire wind, depth, rudder, propulsion, battery, tank and connectivity data
  with source/age metadata. Their owned state contracts now exist; integrate
  actual hardware acquisition and calibrate live energy inputs against recorded
  boat data. Demo estimates are explicitly advisory fixture results.
- Add advisory SmartNav route events, turn/timeline prediction, hazard look-ahead
  and AIS context behind tested source contracts. Later sailing/anchor work
  remains separate. No autonomous steering is implemented.
- Implement isolated hardware adapters and feedback-confirmed manual control
  paths; validate with simulator and actual devices. No hardware commands are
  currently exposed by OpenNav.
- Implement Windows detection, verified compatibility manifest, install/update,
  repair, diagnostics, rollback and uninstall. The supported-installation
  allowlist is deliberately empty; current scripts build isolated source trees
  and do not patch installed OpenCPN.
- Exercise real charts and AIS targets, representative third-party plugins,
  hardware, OpenGL, touch/full-screen operation, crash recovery and non-default
  Windows DPI. Current hosted UI evidence is 96 DPI/software rendering with
  synthetic data and no nautical chart collection.
- Complete the native installer lifecycle and final 1280×800 checks on a
  representative Windows navigation PC. Hosted MSVC evidence is necessary but
  does not establish target-hardware or at-sea acceptance.

The isolated Windows Developer Preview milestone is delivered. **Stop for the
user's manual Windows test and feedback; do not start the production installer.**
[Test guide](preview/TEST_ME_FIRST.md) and
[known limitations](preview/KNOWN_LIMITATIONS.md) ship inside the package.

## Installer qualification increment (not release acceptance)

The side-by-side Alpha setup and transaction engine now exist as a candidate.
The public compatibility allowlist remains empty. Native NSIS/PowerShell 5.1
qualification will exercise the official pinned stock binary, shared profile,
mode/chart behavior, repair/update/rollback/uninstall and interrupted commits.
Linux passes 90 integrated tests and the actual loader/resource self-test,
including no profile mutation and report-overwrite refusal. Native acceptance,
UI review, final Alpha distribution names and the manual-test guide remain open.
See [installer transaction contract](installer-transaction-contract.md).
Independent native package, installer, DPI and chart gates now each run after a
successful integrated build even when another independent gate fails. A failure
still fails the job and prevents publishing; this preserves diagnostics without
weakening any acceptance assertion.

**Configurable display increment:** ordered data-rail/instrument selections and
navigation/sailing/energy rail presets use existing Vessel Data without changing
sample age. Display exposes Day/Dusk/Night and fullscreen through existing
OpenCPN actions. Linux passes 28 portable contracts, 90 integrated tests and the
expanded preview smoke. Native selector/palette review is pending.
[Contract](display-layout-contract.md), [local evidence](evidence/alpha-layout-local-validation.json).
