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

### Native path-spelling correction

The first short native proof failed inside the offline contracts before package
acquisition: a lexical path versus `resolve()` comparison rejected regular
fixture paths after canonical spelling changed. The correction checks actual
`lstat` symlink/reparse attributes on the file and all ancestors instead. Known
roots, regular-file checks and exact identities remain required. Windows 8.3
aliases must pass a real same-file proof; junction ancestry must be refused.
See `docs/evidence/scrum255-gettext-path-failure/` for the unchanged failed
artifact, precise evidence limits and the pending native alias/junction proof.

The next native proof passed all 23 contracts and confirmed actual 8.3 alias
acceptance and junction refusal. Poedit then installed successfully, but its
valid `msgfmt.exe (GNU gettext-tools) 0.26` banner exposed an overly strict
stem-only parser. The parser now allows only an optional literal `.exe` after
the exact tool stem, with every other validation unchanged. Failed evidence and
the captured-banner regression are retained in
`docs/evidence/scrum255-gettext-banner-failure/`; functional native catalog proof
still awaits the corrected short workflow.

### Direct vendor CI preparation (2026-10-19, SCRUM-16)

Candidate `11348664` stopped before native AIS/application compilation after all
three pinned Chocolatey metadata requests returned HTTP503. A separate
`prepare-windows-poedit.py` step now prepares the same Poedit 3.9.1 prerequisite
before AIS, and in the focused native Gettext workflow. The original helper,
build script, SDK producer inputs and native identity checks remain unchanged.
An already usable known-path pair needs no installation. Otherwise the helper
downloads only `https://download.poedit.com/Poedit-3.9.1-setup.exe`, requires
20,038,048 bytes and SHA-256
`a6cb88246834e6e54224b923a839c9566580cb5d4be95d4eba1d18bd34e8ecc9`,
then uses `/verysilent /norestart`, accepting only exit zero. The checksum and
arguments come from the package maintainer's immutable
[3.9.1 recipe](https://github.com/nykula/chocolatey-packages/blob/17164f32f21671a6d9c17bebc183eb1c15913859/automatic/poedit/tools/chocolateyInstall.ps1);
the public vendor download was independently hashed to that value.

The download has a 120-second transfer limit and 150-second owned-process bound;
installation has a 180-second bound. Existing native process cleanup/logging is
reused. A failed/timeout operation cannot accept leftover files. Fresh native
tool probes with installation disabled must pass before the new receipt passes.
Separate preparation evidence preserves the original native proof directory.
This supplies a reviewed alternative acquisition path for disposable CI; it
does not replace the native UTF-8 catalog proof, SDK checks or product gates.
