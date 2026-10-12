# SCRUM-209: native byte-policy test blocked

On 2026-10-12, native MSVC discovery over the `boat` SSH alias failed before source transfer or compilation. The intended test was only `opennav-x/tests/plugin_tls_byte_policy_test.cpp` at source commit `eede2fe90c1ccc31123bd7fdcf5c728763c3cc6f`, targeting real MSVC x86.

`vswhere` exists and reports a complete Visual Studio Build Tools 2019 installation (`16.11.36128.20`), but its query requiring `Microsoft.VisualStudio.Component.VC.Tools.x86.x64` returned `[]`. A follow-up read-only probe searched both standard Visual Studio installation roots recursively for `vcvarsall.bat` and `cl.exe`; neither produced a match. Registered Build Tools alone does not establish an installed C++ compiler.

The follow-up probe is retained in `probe-installed-msvc.ps1`. It was sent as a UTF-16LE PowerShell encoded command with `-NoLogo -NoProfile -NonInteractive`. `result.json` retains the sanitized selected fields from its stdout and the exact component-query arguments/result. The raw component-query stdout is retained separately. SSH returned exit code 0 for both probes. The first stdout line confirms the Windows OS is 64-bit, not the missing compiler architecture.

The existing application PID 1852 was still present after discovery. No UI commands, application restart/termination, profile/configuration/hardware/remote-access changes, plugin loading, source transfer, private Windows workspace creation, compiler installation, cross-compilation, full build, or test execution occurred. Native compilation and execution remain unperformed; compiler version, architecture, executable hash and test stdout are therefore null. No alternative result is substituted for this gate.

The local source manifest in `result.json` hashes the test and its three repository-owned headers and verifies every byte against `git show` of the source commit. These bytes were not transferred because toolchain discovery blocked the task. The test additionally requires the pinned external `picosha2.h`; no dependency transfer or compilation was reached.

The existing Linux result does not close this native gate. Resume only on a native host with an already available MSVC x86 toolchain, using the exact pinned source and required headers; preserve this blocked evidence.

## Actual portable CMake/CTest wiring

The exact source revision also passed an isolated Linux CMake configuration with UI OFF, runtime fixtures OFF, pilot loopback OFF and tests ON. Only target `plugin_tls_byte_policy_test` was built. Only registered CTest case `plugin_tls_known_obsolete_bytes` ran: **1/1 passed, 241 checks**. The existing project-local CMake/CTest 4.4.3 and Ninja 1.13.2 were reused with scoped `LD_LIBRARY_PATH`; GNU C++ 16.2.1 compiled the target. No download or installation occurred. Explicit cached RapidJSON and hash/size-verified pinned picosha2 paths plus `FETCHCONTENT_FULLY_DISCONNECTED=ON` kept configuration offline.

`linux-result.json` contains exact source-input hashes, tool/executable identities and scope. `linux-cmake-commands.json`, configure/build/CTest logs, the selected generated compile command and generated registration capture the actual target and CTest wiring. No direct standalone compiler rerun, full application build or broad suite occurred. The initial missing-PATH launcher, missing-library launcher and `/tmp` quota failures are retained; scoped existing library search paths and a private build `TMPDIR` resolved them before successful configuration. These Linux results do not qualify native MSVC x86 or close SCRUM-209.

Saved Linux log text normalizes trailing whitespace and surplus final blank lines only; all diagnostic and result text is retained.
