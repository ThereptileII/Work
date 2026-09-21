# Development status — 2026-09-21

**Development build; production Definition of Done remains open.** The selected
baseline is OpenCPN 5.12.4 at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The approved Windows target preserves its Win32 application/plugin ABI on a
native Windows x64 host. The tracked upstream submodule remains pristine;
reviewed patches apply to a disposable integration worktree.

## Accepted development increments

| Increment | Evidence and practical limit |
| --- | --- |
| Initial XNav shell | Existing OpenCPN chart canvas, custom touch controls, day/dusk/night tokens, explicit simulator and unavailable/stale states. Native 1280×800 review at 96 DPI; full product screens and chart-content review remain open. |
| XNav / Legacy / Safe | Controlled restart, shared configuration/navigation database, Safe precedence, saved normal-mode preference, clean exits and bundled Dashboard enabled preference preserved. Synthetic waypoint/route/track/connection/AIS-setting fixtures survive the cycle. |
| Selected navigation | Read-only position/SOG/COG from OpenCPN's selected navigation bus. Real decoder input through an isolated synthetic NMEA loopback stream; position-only updates do not freshen old speed/course. Heading is withheld where provenance is insufficient. |
| Advisory energy core | Tested reserve-aware range and arrival SOC, explicit model assumptions, no prediction from invalid/stale/missing inputs, energy-shortfall reporting. This calculation module is not connected to live batteries, route distance or a product screen. |

The shell/mode slice first passed both platforms at `c5a0fd0`; selected
navigation passed at `bc0af30`. Their native review records are in
[`docs/evidence`](evidence/). See [baseline](baseline.md),
[navigation bridge](navigation-data-bridge.md) and [energy model](energy-model.md)
for the inspected source boundaries and exact limitations.

## Latest regression gate

Code/test revision: `c2535f5aa94e9693f999f65952dda144a3fe6c24` on the remote
`opennav-x` branch; local equivalent: `42f8410`.
[Native and Linux CI run](https://github.com/ThereptileII/Work/actions/runs/35641825356).

All seven CI jobs passed. Win32 and Linux portable lanes pass all six contracts,
plus ten consecutive restart lifecycle checks each. Integrated application
regressions pass 50 compiled tests on Windows and 60 on Linux. Shared-profile
mode cycles and synthetic NMEA input checks pass on both platforms. The pristine
Linux baseline classifies its documented upstream test defects; those defects
are not counted as passing tests.

Reviewed native 1280×800/96-DPI captures and executable/artifact hashes are in
[the final Windows review](evidence/windows-c2535f5-review.json). All 75 local
source/build/test/patch files match the tested remote revision. Documentation
commits after this code revision do not change the tested executable.

A previous Win32 contract run at `0cc8330` failed while reading restart-test
arguments. The old probe exposed its result filename before writing finished.
Adding a delay to that write reproduced the assertion locally without changing
arguments. The test now records process startup separately and atomically
publishes the completed result; a deliberate write delay and ten repeated
checks exercise this timing window. Production restart code did not change.
This failure is retained in the history rather than hidden by a successful rerun.

## Remaining product and release gates

- Replace route/waypoint/AIS, instruments, propulsion, settings and other primary
  workflows incrementally with the approved XNav presentation. Preserve existing
  functionality through Legacy throughout that work.
- Acquire wind, depth, rudder, propulsion, battery, tank and connectivity data
  with source/age metadata. Integrate remaining route geometry and actual battery
  inputs before exposing energy estimates; calibrate against recorded boat data.
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

Next vertical slice: use the [active-route source inspection](route-distance-inspection.md) to expose a
read-only remaining-distance contract, with route-transition/invalid-data tests
and both platform gates, before connecting arrival-energy predictions to UI.
