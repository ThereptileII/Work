# Early Windows Gettext prerequisite (SCRUM-255)

Candidate 9fcd3db's native full build failed before application compilation:
Chocolatey's Poedit 3.9.1 feed request returned HTTP504 at 00:12:49, upstream
`win_deps.bat` continued, and the late Gettext check rejected missing `msgfmt`
at 01:09:09 after the costly maintained dependency work. The earlier native
17-unit chart preflight passed independently; neither result is product runtime
acceptance. Root retains the failed full artifact and its exact identity.

`build-pristine-windows.ps1` now invokes `windows_gettext.py ensure` before the
curl source preflight, changed-unit build, wx acquisition, `win_deps.bat` and
maintained dependency builds. Both `msgfmt.exe` and `msgmerge.exe` must exist
and answer `--version` successfully from the same known
`ProgramFiles/Poedit/Gettexttools/bin` or `ProgramFiles(x86)` equivalent.
PATH lookalikes, partial pairs, redirected paths, nonzero exits and unrelated
version banners are rejected. The receipt records exact paths, SHA-256, byte
length and GNU Gettext version, and tools may not change during probing.

Only explicit `ensure --allow-install` authorizes installation. The normal
build is already an explicit dependency-acquisition operation and passes that
flag; `verify` and default `ensure` never install. A usable preinstalled pair
requires no package operation. Its origin is described honestly as a known
preinstalled Poedit path, not a claim that a fresh package was authenticated.

For missing/unusable tools, acquisition uses the same provider observed in the
failure: `https://community.chocolatey.org/api/v2/`, package `poedit`, version
**3.9.1**. The known Chocolatey installation path is selected, not PATH. Up to
three attempts use a 120-second package execution limit, 180-second process
limit and 5/10-second retry delays. Nonzero package exits are never accepted,
even if they leave files behind. A zero exit must also pass both real tool
probes. An installer or version-probe timeout aborts without another acquisition;
only its owned process tree is targeted for cleanup, and cleanup is recorded.
Exact separate stdout/stderr and per-attempt status remain in the evidence.
The process runtime is bounded; Python buffers its output and rejects more than
1 MiB after capture. That output check is not a streaming memory bound.

This retains the existing Chocolatey community-feed/installer trust boundary.
Pinning a provider and package version does not create a reviewed installer
SHA-256 or prove publisher provenance; observed executable identities are bound
for the current build. No alternate mirror, arbitrary PATH executable, blanket
retry or upstream test bypass was introduced.

The verified bin is prepended for stock `win_deps.bat`'s `msgmerge` probe.
Before application CMake, `verify` reprobes and rehashes the exact receipt tools;
CMake receives explicit `GETTEXT_MSGFMT_EXECUTABLE` and
`GETTEXT_MSGMERGE_EXECUTABLE` paths. Same-job dependency receipt inputs include
the new helper. All existing dependency, application and qualification assertions
remain; the old late existence-only test is replaced by stronger pair/identity
checks, not suppressed.

## Focused verification

Local evidence: `docs/evidence/scrum255-gettext-local/`. Seventeen Python tests
cover known x64/x86 directories, no-install behavior, PATH lookalikes, partial
pairs, bad versions/nonzero exits, failed-package files, bounded recovery,
exhaustion, timeout/no-retry, retained streams, tool drift, receipt retargeting,
and actual orchestration order. Ten existing same-job dependency receipt tests
pass; the changed PowerShell orchestration parses using PowerShell 7 on Linux.
No Windows success is inferred from these checks.

The opt-in `skager-gettext-prerequisite` workflow runs the same contracts on a
fresh Windows 2022 runner, then acquires/reprobes actual tools, merges a UTF-8
catalog, compiles it and independently reads the resulting translation. It
records exact source/receipt/catalog identities and has no app, upstream or
maintained dependency build. Root must publish and verify this native proof
before another full candidate. Native proof, full candidate, runtime/installer
and boat acceptance remain open at this implementation commit.
