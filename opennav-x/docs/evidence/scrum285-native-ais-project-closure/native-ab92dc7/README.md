# SCRUM-285 native parser proof: independently audited pass

[Run 37162505696](https://github.com/ThereptileII/Work/actions/runs/37162505696),
attempt 1, job `111318683379`, passed on 2026-10-03 at 23:40:54 UTC.
The downloaded original artifact is retained unchanged as `original-artifact.zip`:
ID `11288127267`, 27,576 bytes, SHA256
`4af23ff5417a38c1a9a916af4145e68e15673f5df1b28faaeeb36cc93f5943f0`.
Its API digest matches; all 11 entry CRCs pass.

The actual Windows log and result contain **8 passed tests, 0 skips**. They prove
the complete eight-project closure from the original native MSBuild files,
metadata exclusion, reproduction of the original defect, malformed/missing edge
refusal, unchanged downstream source guard, and the native-job identity policy.
The original failure job `111300070821` was completed/failed, with step 17 success
and step 18 failure, while its parent run still had unrelated work in progress.
The corrected proof authenticated that exact boundary successfully.

The parent verified publication mapping is local
`c552f6158fda4ccdb995f05d876a6bf6107e326d` → remote
`ab92dc78cd99e945a08b7a00d2b8166a5841f954`, tree
`89a3b50b8ef965089648aa03faadbb922b1e1c2f`: 5,678 mapped entries, only 17 changed
files over frozen local `48c2f8b8d5d4103bcbaacc45a18c4e6a238d4c52` / remote
`4ddf1f383e495150551946dd36103cd77cea85eb`.
This artifact audit independently matches all ten reported execution-input hashes
to that exact local source (Windows CRLF for text; unchanged ZIP bytes), and all
eight uploaded project files to their original artifact member hashes. The
current root AIS wrapper also remains byte-identical to the qualified helper.
`audit.json` distinguishes these byte checks from the parent's whole-tree mapping.

The runner reports Windows Server 2022 and Python 3.12.10. Python executable bytes
are not in the compact artifact: its hash is a CI-recorded receipt, not a newly
rehash-checked executable. Original ZIP contents include the exact report, test
log/result and eight projects; no unrelated GitHub account metadata is retained.

This qualifies the parser repair only. It performed no native binary compilation,
AIS TLS/lifecycle execution, maintained-dependency reuse or application/package
qualification. The full `4dd` candidate remains failed and ineligible. Actual AIS
runtime acceptance still requires the later same-job native gate. No tests were
rerun during this audit; no CI dispatch or boat action occurred.
