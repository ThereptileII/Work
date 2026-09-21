# Development status — 2026-09-21

**Development build; production Definition of Done remains open.** The selected
baseline is OpenCPN 5.12.4 at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The approved Windows target preserves its Win32 application/plugin ABI on a
native Windows x64 host. The tracked upstream submodule remains pristine;
reviewed patches apply to a disposable integration worktree.

The read-only remaining active-route distance slice has passed both platform
gates and native screenshot review; see [its contract](route-progress-contract.md).
Developer Preview 0.1 is implemented and undergoing package/platform acceptance.
It adds explicit deterministic Demo data, advisory energy presentation and an
isolated portable Windows package; see [its contract](developer-preview-contract.md).
The latest accepted gate below remains the route slice until preview evidence
is recorded. Live battery acquisition and calibration remain unavailable.

## Accepted development increments

| Increment | Evidence and practical limit |
| --- | --- |
| Initial XNav shell | Existing OpenCPN chart canvas, custom touch controls, day/dusk/night tokens, explicit simulator and unavailable/stale states. Native 1280×800 review at 96 DPI; full product screens and chart-content review remain open. |
| XNav / Legacy / Safe | Controlled restart, shared configuration/navigation database, Safe precedence, saved normal-mode preference, clean exits and bundled Dashboard enabled preference preserved. Synthetic waypoint/route/track/connection/AIS-setting fixtures survive the cycle. |
| Selected navigation | Read-only position/SOG/COG from OpenCPN's selected navigation bus. Real decoder input through an isolated synthetic NMEA loopback stream; position-only updates do not freshen old speed/course. Heading is withheld where provenance is insufficient. |
| Advisory energy core | Tested reserve-aware range and arrival SOC, explicit model assumptions, no prediction from invalid/stale/missing inputs, energy-shortfall reporting. This calculation module is not connected to live batteries, route distance or a product screen. |
| Remaining active-route distance | Immutable owned snapshot copied around normal OpenCPN progress: current active-point range plus subsequent stored legs, NM, route/revision/active-point identity and separate observation/position times. Edits, skips, reversal, deletion, ambiguity and stale/invalid inputs withhold distance. No production energy/UI connection. |

The shell/mode slice first passed both platforms at `c5a0fd0`; selected
navigation passed at `bc0af30`. Their native review records are in
[`docs/evidence`](evidence/). See [baseline](baseline.md),
[navigation bridge](navigation-data-bridge.md) and [energy model](energy-model.md)
for the inspected source boundaries and exact limitations.

## Latest regression gate

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

- Replace route/waypoint/AIS, instruments, propulsion, settings and other primary
  workflows incrementally with the approved XNav presentation. Preserve existing
  functionality through Legacy throughout that work.
- Acquire wind, depth, rudder, propulsion, battery, tank and connectivity data
  with source/age metadata. Integrate actual battery inputs and the accepted
  route-distance contract before exposing energy estimates; calibrate against
  recorded boat data.
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

The current milestone is the isolated Windows Developer Preview. Stop after
publishing and validating its ZIP for manual testing; do not start the production
installer. [Test guide](preview/TEST_ME_FIRST.md) and
[known limitations](preview/KNOWN_LIMITATIONS.md) ship inside the package.
