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
