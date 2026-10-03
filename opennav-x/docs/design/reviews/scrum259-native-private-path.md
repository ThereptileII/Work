# SCRUM-259: native private-adapter path correction

The failed full run [37106815245](https://github.com/ThereptileII/Work/actions/runs/37106815245/job/111157801437)
reached private CMake configuration after the maintained producers passed. Its
untyped `SKAGER_PREPARED` cache value contained native backslashes. Expansion of
the wxCurl source list then failed with `Invalid character escape '\a'` at
`add_library`, before any private adapter object or DLL was built. The retained
failure receipt is `docs/evidence/scrum259-private-configure-d2787`.

Normalize that one root before verification and use an explicit CMake PATH type
at the PowerShell call. Every target, source, include, definition and link
argument remains unchanged. `Targets.cmake` holds the byte-identical former
wx/target block; production still unconditionally verifies preparation before
including it. Both new CMake includes are in `prepare.INPUTS`, which also drives
package verification and corresponding-source archive membership.

The dedicated `skager-ocharts-compile` branch/workflow initially runs only the
actual path/glob/add-library fragment. On Windows it must reproduce the original
escape failure with a native path containing spaces, then configure successfully
with normalization (both untyped and explicit PATH arguments). It records exact
commands, inputs and full logs. Tiny source content isolates configuration only;
this regression does not claim private source compilation or a usable DLL.

Local checks: 16 preparation contracts, PowerShell parse/order/reuse plus 16
rejection cases, exact target-block equality against 5c05eb5, and actual CMake
4.4.3 POSIX configuration with spaces passed. Native negative/positive cases
remain pending. The failed artifact lacks prepared SDK/source and maintained
link libraries, so it cannot qualify a dependency-reuse or DLL-link attempt.

A separate compile-only entry will use the shared targets with independently
locked actual headers. It must not create a production preparation receipt or
relax the production validator. Final DLL linking, package checks, actual host
module loading, rendering and boat acceptance remain separate gates.

## Actual production-object follow-up

`tools/test-ocharts-compile-windows.py` uses the same `Targets.cmake` through a
separate, explicitly unqualified entry. Its SDK contains real locked wx 3.2.8,
GLEW, curl 8.22.0 public headers and zlib 1.3.2 public headers; it neither builds
maintained producers nor manufactures their manifests or missing libraries.
Pinned plugin/API-17 source and current owned overlays/patches are prepared by
the production source functions. The real chart generator consumes the exact
five pinned OpenCPN inputs. Production preparation/package validators are
unchanged and are not bypassed in any production invocation.

The native gate configures that actual target graph, cross-checks the CMake
codemodel against all ten generated projects and an independently counted
70-translation-unit inventory, then invokes only `ClCompile` with project
reference builds disabled. Every expected object must exist and be x86 COFF;
a linked DLL or library in the probe build fails the gate. Full commands,
compiler projects, actual source copies, objects and source/header/resource
hashes are retained, including on failure. Both local standalone and published
`Work/opennav-x` workflow locations are supported. No altered header, forced
include or NOMINMAX override is introduced.

Local staging-only validation passed: 219 patched source files, 15 owned files,
1,629 SDK files, five original resources and seven generated outputs. Generated
manifest SHA-256 equals the sealed 5c05 build:
`beff73cae53211c4ef0b0c3fa2ed83e5290680f19f48781646e1f1d0d4f88e53`.
The object guard accepted an actual retained native x86 object and rejected an
architecture mutation and missing-object control. Python syntax checks passed.
These checks do not qualify native configuration/compilation: the dedicated
Windows job remains required. Even a successful 70-object gate does not prove
DLL link/export resolution, dependency-producer validity, package identity,
actual-host loading or private renderer acceptance.
