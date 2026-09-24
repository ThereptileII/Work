# Architecture Decision Log

## ADR-001 — Preserve OpenCPN's supported Windows ABI

**Status:** Accepted by user, 2026-09-21.

OpenCPN Release_5.12.4 at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
builds `-A Win32` in `ci/appveyor.bat` and `.github/workflows/windows.yaml`.
Its wxWidgets 3.2.8 and core-support dependency bundle are Win32. Existing
plugin DLLs must match the process architecture.

The user explicitly chose: **Preserve supported OpenCPN ABI on 64-bit Windows**.
This overrides the specification's literal x64 application-build requirement.
Native Windows x64 remains mandatory; OpenCPN/OpenNav application and plugins
use the supported x86 ABI. No cross-compilation or Wine substitution.

Verify the executable's PE machine field, native MSVC builds, runtime DLL and
plugin loading. A future x64 application port is separate work and must not
silently drop plugin compatibility.

## ADR-002 — Baseline before runtime integration

**Status:** Accepted, 2026-09-21.

Keep the pinned OpenCPN submodule unmodified during pristine validation.
Portable startup-policy contracts can be tested independently; they are not
presented as implemented OpenCPN mode switching. No SmartNav, hardware control
or installer patching until the Windows dual-mode slice is stable.

Use narrow, documented source patches applied to a separate integration
worktree later, preserving a directly comparable pristine source checkout.

## ADR-003 — Remote validation location

**Status:** Selected under user's delegated repository/runner choice, 2026-09-21.

Use `ThereptileII/Work`, isolated `opennav-x` branch, project under `opennav-x/`
and workflow `.github/workflows/opennav-baseline.yml`. Retain all pre-existing
repository files. Local project root is `Projects/X-nav`. Both CI platforms
check out the same remote commit. Windows runner: `windows-2022`, VS 2022.
Linux CI runner: `ubuntu-24.04`. No default-branch changes or releases.

## ADR-004 — Isolated visual-test profiles

**Status:** Accepted for development evidence, 2026-09-21.

Baseline capture uses a newly created `--configdir` profile with no connections,
charts or recorded vessel state. Test configuration bypasses the first-run
wizard/warning only in this disposable profile; production behavior is unchanged.
The upstream source uses `opencpn.conf` with `--configdir` on both platforms,
despite the usual Windows default being `opencpn.ini`.

Software-rendered baseline screenshots (`--no_opengl`) must be identified as
such. They do not prove OpenGL, real-chart readability or target-PC acceptance.

## ADR-005 — Per-user side-by-side Alpha installation

**Status:** Implemented candidate; native lifecycle qualification pending.

Install complete integration generations under LocalAppData and keep the exact
hash-verified stock OpenCPN executable unchanged. All installed modes use the
normal shared profile located by OpenCPN. The portable distribution retains its
separate profile. This refines the specification's replace/back-up flow: no
Program Files replacement or elevation is necessary for OpenNav-owned files.
Atomic generation selection, retained prior files and verified ownership support
repair/update/rollback/uninstall without restoring old navigation data. The
public compatibility allowlist remains empty until disposable native lifecycle
and UI gates pass. See [transaction contract](installer-transaction-contract.md).

## ADR-006 — Explicit Alpha hardware and chart-query boundaries

**Status:** Implemented and contract-tested; physical gates remain open.

Autopilot commands pass only through the manual adapter interface with fresh
feedback, pending-command identity and timeout handling. Alpha supplies a
simulator; live output is disabled. SmartNav has no dependency that can send a
command. Radar reports unavailable without an accepted source. The hazard
corridor has a tested provider contract but no live ENC coverage provider:
viewport object queries alone cannot establish complete future-path coverage or
safe clearance. This uses the user's permission to complete abstractions and
continue while retaining explicit physical/upstream integration gates.

## ADR-007 — Reuse OpenCPN transport and selected navigation

**Status:** Marine input gate accepted; calibration/physical validation open.

OpenNav subscribes to existing OpenCPN marine-message services and its selected
navigation contract; it does not open a second NMEA/Signal K transport. Owned
Vessel Data separates normalized quantities, source precedence, provenance and
freshness from UI and SmartNav. Standard marine meanings are retained. Optional
vendor/boat Signal K mappings are explicitly configured and never depend on
Leaf EV-CAN identifiers. Capacity, reserve and pack-current sign have no invented
live defaults. A mapping interpretation change invalidates retained observations.
See [marine bridge](navigation-data-bridge.md) and
[propulsion mapping](propulsion-source-mapping.md).

Record significant decisions here using this format.

## ADR-000 — Template

**Status:** Proposed / Accepted / Superseded  
**Date:** YYYY-MM-DD  
**Decision:** Short title

### Context
What problem or constraint prompted the decision?

### Decision
What was chosen?

### Alternatives considered
What other options were considered?

### Consequences
What becomes easier/harder?

### Verification
How will this decision be tested?
