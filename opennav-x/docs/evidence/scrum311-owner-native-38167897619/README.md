# SCRUM-311 native owner audit — partial progress, overall FAILED

[Run 38167897619](https://github.com/ThereptileII/Work/actions/runs/38167897619),
attempt 1, used exact commit `0e808824d9ff2d6a4890e608d72805df16174cd8`.
Terminal API run/job/artifact identities and the native receipts agree.
Go Win32 passed. Both PowerShell owner harnesses completed; the x64 transaction
step passed, but the x86 transaction step failed during fixture cleanup.
**The overall run and PowerShell job failed. No complete x86 transaction or
product qualification is claimed.**

| Job / fixture step | API interval UTC | Interval | Result |
| --- | --- | ---: | --- |
| [Go job 114554181173](https://github.com/ThereptileII/Work/actions/runs/38167897619/job/114554181173) | 19:24:43–19:25:38 | 55 s | Passed |
| Go native fixture step | 19:25:26–19:25:34 | 8 s | Passed |
| [PowerShell job 114554181288](https://github.com/ThereptileII/Work/actions/runs/38167897619/job/114554181288) | 19:24:43–19:26:44 | 121 s | Failed |
| x64 owner + transaction step | 19:24:54–19:25:55 | 61 s | Passed |
| x86 owner + transaction step | 19:25:55–19:26:40 | 45 s | Failed |

These durations are terminal API `completed_at - started_at` intervals on
2026-10-11. Job intervals include setup/teardown; they are not individual test
durations. The suspended-only matrix step and four other jobs were skipped.

## Executed results

**Go `386`:** all nine required fixtures passed with no failed or skipped events.
The retained test JSON has 52 events, ten top-level passes (nine required plus
the child entry helper), two additional subtest passes, and one package pass.
The receipt records `arch=386` and host toolchain `go version go1.27.1
windows/amd64`; the job also records `GOARCH=386`. Actual creator, physical
custody, SUBST remapping, corruption/wrong-image refusals, wrong hash/hardlink,
suspended-refusal cleanup and bounds are covered by these selected fixtures.
All **20 Go input hashes** match exact-revision Git bytes after explicit
LF-to-CRLF conversion.

**PowerShell owner harness, both hosts:** both retained owner logs reach the
final bounded-fixture PASS and contain 79 `PASS:` lines each. Actual assertions
include correct image after SUBST remap, normal close of that original child,
released image/directory pins, PID/challenge/frame health authentication,
supervisor normal cleanup, refusal without fallback, resume uncertainty, and
fresh interrupted cleanup restricted to the matching recorded child. The logs
are byte-identical; the job log separately records execution under the default
host and then the explicit SysWOW64 x86 PowerShell driver with a `.ps1` file.
PASS-line counts describe retained log entries, not a count of all assertions.

**PowerShell x64 transaction:** 51 `PASS:` lines include the native health and
phase cases, suspended-loader good/wrong-hash cases, known-good bootstrap,
one-shot candidate health, guarded rollback, pre-create refusal, integrity
failure, failed graceful-close recovery, lost managed return after resume, and
identity-persistence refusal. The step succeeds and emits its final receipt:
PowerShell **5.1.20348.5622**, **64-bit**, OS **Microsoft Windows NT
10.0.20348.0**, exact commit/run/attempt, `passed=true`. All **five PowerShell
input hashes** match the exact Git revision after LF-to-CRLF conversion.

**PowerShell x86 transaction, incomplete:** the owner harness completed, then
the transaction log records 43 `PASS:` lines through both suspended-loader
startup cases, ending with `wrongHash=True`. It then records the original error:

```text
Remove-Item : Cannot remove item ...\generations\bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\app\opencpn.exe:
Access to the path 'opencpn.exe' is denied.
At ...\tools\test-update-transaction.ps1:565 char:42
```

The raw log preserves the full path, error category and
`UnauthorizedAccessException` bytes. No later transaction supervisor-completion
messages or x86 overall receipt exist in the artifact. The cleanup error may
obscure an earlier failure; this audit does not identify a holding handle/process
or infer a cause. An owner-harness PASS cannot substitute for the missing x86
transaction completion. Only the x64 receipt reports measured version/bits and
native source hashes; no independent x86 overall metadata receipt is available.

## Integrity and retained records

Both archives were already downloaded once by the coordinator. Their API
ownership/size/digest matches the local bytes, and all **seven members pass CRC**.

| Artifact | ZIP bytes | API/local SHA-256 | Members |
| --- | ---: | --- | ---: |
| Go **11712059273** | 2,488 | `08f8594b41450abafc3b5d02555d94432dcbd04f99ddbd12136f7a474a35f990` | 2 |
| PowerShell **11712803435** | 7,029 | `d98f551a726ef48190de1933715b346d56190250e28d9abe5f55ba1ec955a8a8` | 5 |

`raw/` retains **11 byte-exact records, 247,744 bytes**: terminal API run/jobs/
artifact JSON; combined job log; Go test JSON/receipt; x64 owner/transaction
logs/receipt; and x86 owner/partial transaction logs. `audit.json` records their
hashes/lengths, all 25 verified source identities, actual assertions and timing.
`.gitattributes` prevents conversion of raw CRLF, UTF-16 logs and whitespace.
No ZIP, executable, source snapshot or native fixture binary is committed.

[The earlier first-close and x86-driver failures](../scrum311-owner-native-38167390164/README.md)
remain historical failures at their own revision. This result demonstrates
progress in the owner fixtures, without rewriting those outcomes or proving
their root causes. Later transaction fixture changes require separate
exact-revision native evidence.

Only this dedicated evidence directory changes. No new download, rerun, native
execution, build, CI dispatch, source/workflow/product/status/contract edit,
boat operation or Claude review was performed. These bounded native passes do
not qualify a complete application/package/update lifecycle or close SCRUM-311;
application-build, product and boat acceptance remain false in both receipts.
