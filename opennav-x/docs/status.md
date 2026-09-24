# Development status — 2026-09-24

**Development build; production Definition of Done remains open.** The selected
baseline is OpenCPN 5.12.4 at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The approved Windows target preserves its Win32 application/plugin ABI on a
native Windows x64 host. The tracked upstream submodule remains pristine;
reviewed patches apply to a disposable integration worktree.

**Manual-preview feedback:** the user completed the Windows test and reported
one issue: the chart background disappears after Legacy → XNav. This is reproduced
with the accepted executable in portable mode. The previous mode gate verified
process/data persistence but missed coastline rendering after restart; its
all-water Legacy/Safe captures should not have been accepted as normal.
See [reproduction evidence](evidence/chart-restart-b21bd05-reproduction.json).
The repair is implemented; its replacement ZIP awaits both platform gates and
native screenshot review. Existing accepted artifact links below identify the
affected build until replacement evidence is published.

The read-only remaining active-route distance slice has passed both platform
gates and native screenshot review; see [its contract](route-progress-contract.md).
**Developer Preview 0.1 has passed both platform gates, extracted-package tests
and native screenshot review.** It adds explicit deterministic Demo data,
advisory energy presentation and an isolated portable Windows package; see
[its contract](developer-preview-contract.md). It is ready for manual Windows
testing, not approved for navigation. Live battery acquisition and calibration
remain unavailable.

[Download the Windows artifact](https://github.com/ThereptileII/Work/actions/runs/35663416666/artifacts/10668776591).
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

## Accepted Developer Preview gate

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
