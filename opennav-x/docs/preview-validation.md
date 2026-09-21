# Developer Preview 0.1 validation record

Accepted application revision: `b21bd05ce75f22c91ac12927207d264b8b3efde4`.
[Native/Linux CI run](https://github.com/ThereptileII/Work/actions/runs/35663416666).
Review date: 2026-09-22 (the CI run occurred on 2026-09-21 UTC).

The preceding route-distance slice was accepted separately at `954b4505` before
preview energy consumers were implemented. Its source ownership, pinned geometry,
normal-progress observation and validity rules remain unchanged.

## Automated gates

The workflow builds the pinned OpenCPN 5.12.4 commit
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` in disposable source trees and verifies
the approved Win32 ABI on an x64 Windows host. Linux and native MSVC integrated
builds pass 67 and 57 tests respectively. Nine portable contract suites per
platform cover existing mode, restart, navigation, route and energy behavior plus
the new deterministic Demo/energy consumer and portable-profile boundary. Each
lane also repeats the restart handoff ten times. These are suite/test counts,
not a claim that each internal assertion is a separate CTest test.

Existing application smoke scripts exercise shared navigation/config persistence,
selected synthetic NMEA and all 26 normal-timer route observations. The seven
integrated route cases retain actual pinned OpenCPN objects and geometry; the
portable route contract covers 28 cases. `tools/smoke-preview.py` drives all eight
Demo scenarios, checks moving values, withheld/shortfall predictions and mode
transitions. Demo does not substitute for the real OpenCPN route tests.

`tools/package-preview-windows.ps1` stages the native install, app-local MSVC
runtime, private portable resources/plugins, clean profile, launchers, docs and
source archive. `tools/verify-preview-pe.py` verifies the architecture and import
closure of 87 installed EXE/DLL files. The extracted ZIP is then launched with
only Windows system directories on PATH. Tests cover all four launchers, direct
EXE startup, external-profile refusal, unchanged normal-profile canary, no new
external OpenCPN files, persisted route/waypoint/track/connection/AIS/plugin data,
and clean mode restarts. The hosted runner has no production OpenCPN installation;
manual installed-application coexistence is not claimed by that test.

Dashboard must actually initialize and unload cleanly in six normal launches,
and must stay inactive in two Safe launches. The package supplies portable plugin
DLLs/data under `profile/plugins`, as required by the pinned upstream loader.
The test fixture marker and temporary navigation data exist only in the test
extraction and are absent from the published ZIP.

## Native interaction and visual review

The authoritative desktop is Windows Server 2022 x64, MSVC 19.44.35228.0,
wxWidgets 3.2.8, 1280×800 at 96 DPI, software rendering. Fifteen captures were
reviewed: all ten preview images (Day, Night, Route, Energy, Diagnostics, stale,
unavailable, shortfall, Legacy and Safe) and five existing navigation/route
regression images. See the [Windows review](evidence/windows-b21bd05-review.json)
for their hashes and observations.

The test additionally verifies that each content page has visible, uncovered
native bounds, that chart navigation works after returning from it, and that the
bottom route summary remains beside its buttons after narrow-to-wide resizing.
No unintended clipping or overlap was found at the accepted resolution. 125%
and 150% Windows scaling, touch, OpenGL and target-PC behavior remain open manual
checks. Nonfatal upstream SVG-cache warnings occur on deeply nested paths; the
test guide recommends a short writable extraction path.

## Download audit and evidence

Official artifact: [OpenNavX-DeveloperPreview-win64](https://github.com/ThereptileII/Work/actions/runs/35663416666/artifacts/10668776591).
It contains `OpenNavX-DeveloperPreview-win64.zip` (48,386,718 bytes) and its
SHA-256 sidecar. Application ZIP SHA-256:
`cfa9e9e462aea63da6df5c8cce324d4fd8769e2799b0976d02931f03be0c8e58`.
GitHub's outer artifact wrapper has a separate hash recorded in the evidence.

The downloaded package passes all 997 manifest hashes. Its executable SHA-256
matches the native tested executable and its manifest requests `asInvoker`.
Four private-profile plugin DLLs match their installed counterparts. The separate
corresponding-source archive is published in the same run; 103 implementation
files and the integrated `ocpn_app.cpp` were compared with the local source.
The remote/local mirror audit separately compares 108 source/build/test/patch
inputs, with no mismatches.

- [All eight CI gates and artifact metadata](evidence/preview-b21bd05-gates.json)
- [Portable contracts and repeated restarts](evidence/preview-b21bd05-contracts.json)
- [Linux review](evidence/linux-b21bd05-review.json)
- [Native Windows review](evidence/windows-b21bd05-review.json)
- [Downloaded package/source audit](evidence/preview-b21bd05-package.json)
- [Local/remote source equivalence](evidence/source-mirror-b21bd05.json)

All original test logs, JSON observations and screenshots remain in the same
run's platform evidence artifacts. Documentation-only acceptance commits after
the packaged revision do not alter its binary. Earlier candidate failures remain
recorded; they are not substituted for final passing evidence.

The next gate is the user's manual run of [TEST_ME_FIRST](preview/TEST_ME_FIRST.md).
This preview is not approved for navigation. No production installer work begins
automatically.
