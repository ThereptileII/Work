# Original short maintenance gate — 2026-10-14

Source: `4fe70d8ae80c5f71c0d127bd1ab6c6eb0d0d54b2`.
[Run 38580830570](https://github.com/ThereptileII/Work/actions/runs/38580830570),
attempt 1, completed successfully. Existing `skager-startup-restoration.yml`,
no new job/input, no application build, no installer or boat execution.

Original authenticated artifact metadata, downloaded ZIP bytes and all CRCs
match. The downloaded `tests.json` and `maintenance-contracts.json` agree on
source/run/attempt. No test or package failure occurred.

| Platform | Original job | Artifact | ZIP bytes | SHA-256 | Top-level passes |
| --- | --- | --- | ---: | --- | ---: |
| Windows 2022 / Go 386 | 115909967326 | 11903201519 | 54077 | `6a1f098ef85130c4e039a0965a0634546426c0efbad7e39b7f837cd768c57cbf` | 283 |
| Linux Ubuntu 24.04 / amd64 | 115909967677 | 11901479955 | 50448 | `e0ec4096460db0bb4ca0f1982d8d2da70e89930b52adeb403dd5fb0542ef4d2d` | 188 |

Test stream hashes: Windows
`c10d04e24e1368f68ba10d2ad0496e38649333680607d60482b33cfe1c60a519`;
Linux `1d858d50ca0b2bba6b84e14f6e4ee8584613c43c76156f7e47c822156b2f8f18`.
All required 30 Windows / 15 Linux maintenance cases ran and passed exactly once,
without skipped/failed subcases. These include original process/token custody,
selected-history paging, previous qualified selection and customer rollback
contracts. Native Windows job: 14:28:34–14:30:28 UTC.

Windows skips (not acceptance evidence):

- `TestNativeMaintenanceBarrierSubstRemapCannotReplaceOriginalDirectory`:
  explicitly requires disposable Windows runner opt-in.
- `TestDecommissionExplicitAPIRequiresWindows`,
  `TestMaintenanceExplicitAPIRequiresWindows`,
  `TestStartupRestorationPublicNativeBoundary`,
  `TestStartupExplicitAPIRefusesNonWindows`: non-Windows boundary tests.

Linux has no skips. Counts exclude subcases to avoid double counting.

This qualifies the selected helper/native ownership controls only. Actual
customer UAC/dialog execution, broker conversation, previous installed-version
journey, stock reopen, complete Repair and boat acceptance remain separate.
The in-progress Repair branch is not part of this source revision. No Jira
acceptance or release qualification follows merely from these passing helpers.
