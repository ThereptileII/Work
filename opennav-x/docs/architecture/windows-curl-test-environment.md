# Windows curl source-test environment

SCRUM-224 qualifies the maintained curl dependency before the SKAGER application.
The supported application ABI remains Win32/x86 on a 64-bit Windows host.

## Observed failure

At candidate `45b8a9d`, curl compiled but its upstream source checks 1119 and
1167 failed. The native configure-only comparison at `ac067b3` reproduced two
independent problems: the parent did not expose MSVC/SDK headers, and MSYS
converted `/DWIN32` and `/D_WINDOWS` options into nonexistent source paths.
Initializing MSVC alone repaired the header failure. Initializing MSVC and
setting `MSYS2_ARG_CONV_EXCL=/D` passed both unchanged scripts: exact `OK` for
1119 and 1,422 analyzed symbols for 1167.

[Native comparison](https://github.com/ThereptileII/Work/actions/runs/36968122673)
uses the locked curl archive and actual CMake-generated `configurehelp.pm`.
It configures Schannel without external dependencies; it does not qualify the
production OpenSSL/zlib build or the application.

## Scoped integration

The curl producer initializes the selected Visual Studio x86 environment,
imports only relevant build variables, selects the explicit MSYS Perl, and
preserves compiler `/D` options. All modified variables are restored on success,
failure, and the tool-only reuse return. The full environment is never logged.
The normal build and reuse verification enter the same scope; native tool
facts retain the argument-conversion setting and initializer identity, and the
reuse receipt binds helper bytes.

A configure-only early preflight detects these failures before expensive
OpenSSL/application builds. The producer also runs the unchanged checks against
its actual production-generated helper before curl compilation. The complete
upstream curl test target and nonzero all-tests-passed assertion remain required.
No curl navigation or runtime code is changed by this repair.

## Qualification

Focused native tests must exercise the shared helper and restoration, including
failure, with PowerShell 7 and Windows PowerShell 5.1. The early production-only
path must pass on the same revision before launching one full integrated Windows
candidate. Production compilation, full upstream tests, dependency reuse,
application tests, installer/recovery, visuals and endurance remain required.
The boat PC stays unchanged until a verified functional review package exists.
