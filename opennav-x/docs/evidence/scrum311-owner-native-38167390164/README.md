# SCRUM-311 native owner audit — mixed result, overall FAILED

[Run 38167390164](https://github.com/ThereptileII/Work/actions/runs/38167390164),
attempt 1, used exact commit `6445d67e27c6b6f4254849556efea28248204711`.
The terminal API run and all six job identities agree with that candidate.
The Go native fixture job passed; the PowerShell owner job failed. Four other
jobs were skipped. **The overall run failed; this is not PowerShell or product
qualification.**

| Job | Result | Bounded observation |
| --- | --- | --- |
| [114552679528 — Native Win32 ordinary image ownership](https://github.com/ThereptileII/Work/actions/runs/38167390164/job/114552679528) | Passed | Go `386`, all nine required native fixtures passed, no skips |
| [114552679184 — Windows PowerShell 5.1 supervised image ownership](https://github.com/ThereptileII/Work/actions/runs/38167390164/job/114552679184) | Failed | Default PowerShell step created the correct held-image child after SUBST remap; first owner close failed. x86 driver failed before running the harness. |

## Go result

The retained receipt records `arch=386`, toolchain `go version go1.27.1
windows/amd64`, and the exact run/attempt/commit. The job log records
`GOARCH=386` and `go test -count=1 -json ./launcher -run '^TestNativeOrdinary'`.
The toolchain host string does not change the selected Win32 test architecture.

`tests.json` contains 52 JSON events: 12 test runs, 12 test passes, one package
pass, and no failed or skipped events. There are ten top-level passes: all nine
required fixtures plus the child entry helper; the wrong-hash/hardlink fixture
also has two passing subtests. The nine required names are retained in the raw
receipt and derived audit, with exactly one run/pass pair each.

This executes the actual ordinary creator, physical custody before resume,
owned SUBST remapping, selected-application corruption refusal, wrong suspended
image refusal, wrong hash/hardlink refusal, suspended-refusal cleanup, and
name/environment/application bounds. It is a selected native fixture pass,
not a full launcher/application/update-lifecycle suite or PowerShell acceptance.
The receipt explicitly sets application-build, product and boat acceptance false.

All **20 recorded Go input hashes** match the exact Git revision after explicit
LF-to-CRLF conversion. `audit.json` records both Git and native lengths/hashes.

## PowerShell failures and limits

The default Windows PowerShell 5.1 step records 57 `PASS:` lines, including
selected application refusal, retained-image/ancestor locks, no replacement
execution after alias remapping, and the created child's held physical-image
match. Its next operation is the **first** `$owner.Close($child,10000)` at
`test-native-update-image-owner.ps1:146`. The original log then records:

```text
Exception calling "Close" with "2" argument(s):
"Owned child remains live; no forced termination or rollback."
```

The later actual-supervisor health assertion at source line 161 was not reached.
No successful normal close, health, transaction or complete-owner receipt is
present. The failure's cause is unproven here; the coordinator's inert-window
visibility investigation is separate and does not rewrite this result.

The x86 step invoked
`C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe ... -File {0}`.
PowerShell rejected the runner's extensionless temporary file because it lacked
a `.ps1` extension, before the harness ran. This is a driver invocation failure,
not evidence of x86 owner behavior. No x86 owner log or success receipt exists
in the artifact. The suspended-only creation matrix step was skipped in this run.

The API/job log establishes the exact checkout revision. Because PowerShell
failed before its final receipt, **no observed PowerShell source-hash record**
was emitted. The audit separately labels production/harness/workflow Git and
expected Windows CRLF hashes; these are not presented as native receipt hashes.

## Retained evidence integrity

Both archives were already downloaded once by the coordinator. This audit used
those local bytes only; no further download or execution occurred.

| Artifact | ZIP bytes | API/local SHA-256 | Members |
| --- | ---: | --- | ---: |
| Go **11712403646** | 2,494 | `2d93189e17a32385b6a314234103aa46b5ae1bffb3d37581e30d4c82a47ad00e` | 2 |
| PowerShell **11712727661** | 1,463 | `95b796a8bc582f31cba18633b4953e68b61f309c4b0599d503a1c0faff3b735e` | 1 |

Artifact run/candidate ownership, API size/digest and all **three member CRCs**
match. `raw/` retains seven byte-exact records, **179,635 bytes** total: terminal
run/jobs/artifact API JSON, combined job log, Go test JSON and receipt, and the
original UTF-16 PowerShell owner log. `audit.json` lists their lengths/hashes and
derived checks. `.gitattributes` disables text conversion for raw records;
original CRLF, encoding and log whitespace are intentionally preserved.
No ZIP, executable, source snapshot or fixture binary is committed.

Only this dedicated evidence directory changes. No native rerun, CI dispatch,
source build, product/status/contract edit, boat action or Claude review was
performed. Later fixture/driver fixes and real PowerShell owner qualification
require their own exact-revision native evidence; the Go pass does not cover them.
