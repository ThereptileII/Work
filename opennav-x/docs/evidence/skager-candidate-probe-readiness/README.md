# SKAGER candidate security-probe readiness

This is test-tool compatibility work for SCRUM-211/212. It neither changes nor
qualifies frozen application `17ab044a1e5222dc71791ac8118454219efe8734`
(local `0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4`). No candidate artifact
request is present and no native probe or boat operation was executed.

The original prepared tools expected old development package names and an old
window caption. They now require `SKAGER-Beta2-Setup.exe`,
`SKAGER-Beta2-Portable-Recovery.zip` and its exact SKAGER directory prefix. The
peer harness uses the current `SKAGER / OpenCPN` caption with the existing exact
PID check; its pinned SHA-256 was updated with that single title change. A
fixture using the old recovery prefix is refused.

The PluginHandler native probe's Python caller now uses typed forward-slash
CMake paths. `tests/plugin_download_guard/InputPaths.cmake` normalizes the
boundary before path expansion and refuses missing or wrong-kind required inputs.
The helper is included in the explicit probe-input hash inventory. This applies
the already-established Windows path convention; it is not a diagnosis of the
currently running native installed-product stage.

Root inspected the latest remote tooling branch `20cff4d` and independently
reconstructed all 1,451 mapped entries against local `ca0c025`. The compatibility
change was rebased onto that actual baseline, preserving its subsequent exact
Setup refusal and recoverable-profile tests. No existing prerequisite, TLS case,
runtime/SDK identity check, trust cleanup, timeout owner or profile-restoration
assertion was removed. Internal ownership/configuration/self-test identifiers
remain unchanged.

Focused checks:

- Candidate adapter: 8 tests passed after integration, including mandatory CLI
  prerequisite refusal and old recovery identity rejection.
- Request parser: 2 tests passed.
- CMake input boundary: 5 cases passed using `cmake -P` only: spaces/backslashes
  with drive/UNC hints, missing input, invalid directory, missing import and
  directory-as-import.
- Existing installer profile-preservation contracts: 11 tests passed after
  integration, including cleanup failures and uncertainty handling.
- Independent source review found no further SKAGER mismatch in the later
  installer probe; relevant trust/peer patches and dependency locks match the
  frozen candidate.

These are host-side preparation checks, not Windows TLS/plugin/peer acceptance.
Native execution still requires an independently audited original eligible
candidate artifact and exact same-run restart receipt. The four request fields
(run, commit, artifact and digest) must be bound to that evidence before dispatch.
An early pending-endurance artifact supports development review only. Product,
private-chart, full CI, real boat and public-release gates remain open.
