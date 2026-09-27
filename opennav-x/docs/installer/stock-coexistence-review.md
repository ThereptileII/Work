# Actual stock OpenCPN review after uninstall

`run-stock.ps1` and `review-stock.ps1` provide a separate read-only desktop
coexistence check after OpenNav uninstall. They do not invent an installed XNav
generation, use a portable profile, or pass `--legacy` to upstream OpenCPN.
The official executable receives **empty arguments** and its ordinary shared
profile. Stock Safe Mode is not exercised by these tools.

Only the already validated OpenCPN 5.12.4 Win32 executable is accepted:
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`, pinned source
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. No parameter can extend that allowlist.
The exact PE/version check is repeated by commissioning context discovery.
Any OpenNav `state.json`, unfinished `transaction.json`, or uninstall
registration in either current-user registry view blocks this stock path.
Retained ownership/recovery logs are not treated as an active installation.

## Required sequence

1. Close XNav and its plugin helpers normally; restore any active commissioning
   transaction before installer maintenance. Perform the existing verified
   uninstall and inspect its results. Preserve the user's navigation/profile data.
2. Prepare **a new stock-only commissioning transaction** using
   `commission-read-only.ps1`. Its context must have `installation: null` and
   exactly the actual stock and managed plugin roots. An earlier installed
   transaction is refused, even if its executable originated from the same
   OpenCPN version. The existing fixed recovery baseline still applies; these
   tools do not broaden recovery or erase intervening user changes.
3. Review the full inventory/source plan and apply its input-only/quarantine
   preparation. Independently add `stockReadOnlyAudit` to the private
   `boat-target.json`, retaining the normal `stockExecutable` and
   `profileDirectory`. The audit contains:
   - `launchKind: "StockLegacy"`, the exact `executableSha256` above, and the
     pinned `upstreamCommit` above;
   - actual reviewed `profileIniSha256` and `reviewedUtc` (maximum age 24 hours);
   - explicit reviewed booleans `connectionsOutputDisabled`,
     `pluginOutputsReviewed`, `noActiveRouteOutput`;
   - `commissioning.record`, `recordSha256`, and `appliedSha256` from this new
     transaction. No tool manufactures these approvals.
4. Run `run-stock.ps1 -Workspace <workspace>`. Both dispatch and interactive
   execution verify the active prepared/applied journal, source evidence,
   original/input profile byte proof, complete plugin/helper trees, absent
   quarantined DLLs and their preserved recovery copies, source review age, and
   current input-only shared profile. The child receives the reviewed explicit
   application/System32/Windows PATH and working directory; no global environment
   changes occur and no OpenNav restart broker is armed.
5. Preserve the returned private `runs/*-launchstock-*/result.json` and its hash.
   Use `review-stock.ps1 -Workspace <workspace> -LaunchResult <result.json>
   -ExpectedLaunchSha256 <hash> -Action Capture` to inspect the real chart window.
   `-Action Resize1280x800` permits only that fixed physical window size and
   captures its result. If the monitor work area is smaller, it refuses; it does
   not hide the taskbar or change display resolution/scaling. Capture at the
   existing size remains available.
6. After human chart/profile review, use the same launch binding with
   `-Action Close`. It requests normal application shutdown and verifies exit
   code zero. It does not kill a process. Inspect normal profile writes and the
   existing commissioning restore report separately before restoring the
   original output/control configuration. Do not automatically launch afterward.

Review permits only Capture, Resize1280x800 and Close. No keys, menu navigation,
route activation, plugin controls, radar controls or autopilot commands exist in
this path. Each operation binds the launch request/result hashes, exact PID and
creation tick, executable, current user/session, unchanged target record and
active stock transaction. It rechecks complete plugin trees, source evidence,
quarantine/recovery bytes and input-only navigation configuration. A restored or
changed transaction refuses review. Images go only to a new private directory
for the current SID, administrators and SYSTEM; obscured, partial or changed
window captures are refused. Native pixels require human inspection—successful
capture is not a chart-content or navigation-acceptance assertion.

## Verification

`test-stock-review.ps1` exercises the fixed stock identity, empty-argument and
command refusals, expired/changed evidence, exact process creation identity,
native-helper compilation, and a complete disposable stock-only commissioning
fixture. That fixture mutates helpers, retained plugins, quarantine/recovery
bytes, source evidence, applied journals, profile and loader-root context and
requires refusal. Existing installed-verifier tests run unchanged in their
mandatory installed parameter set. The native maintenance jobs include both
suites. Linux portable checks pass: 80 stock groups (68 policy/native-compilation and
12 full-transaction groups), the existing 11 installed-launch groups, and five
portable maintenance groups. These execute no real application. Native Windows
and actual post-uninstall stock launch/capture/normal-close remain separate,
pending gates.
