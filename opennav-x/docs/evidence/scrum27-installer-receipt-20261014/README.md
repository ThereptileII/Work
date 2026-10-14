# SCRUM-27 native installer receipt preflight — 2026-10-14

The focused native Windows check passed at exact source
`1107c04f55a5a51660e424493bf083fc83106443`:
[run 38548714640, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38548714640),
[job 115801121992](https://github.com/ThereptileII/Work/actions/runs/38548714640/job/115801121992).
All **44 receipt tests passed** in 9.903 seconds. The separate stat probe also
completed successfully on `windows-2022`, Python **3.12.10, 64 bit (AMD64)**.

The probe reads the actual Python executable and inert temporary EXE/TXT bytes
through the repaired receipt reader. It reported:

| Source | Path/descriptor fields that differed | Common identity / independent complete baselines |
| --- | --- | --- |
| Actual Python executable | `st_mode`, `st_ctime_ns` | Matched / stable |
| Inert EXE | `st_mode` | Matched / stable |
| Inert TXT | None | Matched / stable |

The reader retains device/inode, file type, size, modification time, link count
and reparse identity across open, then checks each source's full metadata
baseline independently. Tests retain mutation, replacement/native replacement
exclusion, link/reparse, exact source namespace, checksum, hash, archive and
publication rollback refusal checks. This follow-up also uses real Windows case
renames, exact CRLF fixture bytes, and physical short/long-path identity checks.

`native-receipt.json` records GitHub API run/job/artifact corroboration, source
Git-blob hashes, the original member hashes, the test summary and probe result.
Original artifact **11885931109** is **2,589 bytes**, matching GitHub's digest
`e19e6579d3d9ad69a26008ec12cb505563d83d89ab2c1b56fae781a2da51f553`.
All three original ZIP member CRCs passed. The original archive remains retained
outside this documentation; the receipt is a derived verification record.

Failed [run 38547844263](https://github.com/ThereptileII/Work/actions/runs/38547844263)
at `1cb779eea610d88f0ddbf9a88e85d5c67ba931be` remains failed: 43 tests,
5 failures and 4 errors; its separate probe was skipped. Its unchanged artifact
**11885681081** is **2,865 bytes**, SHA-256
`58081498ea2b114d8c11b87788cdf7d6350606f0b847e05aaa528d3986e0aecf`,
with both original member CRCs verified. No failed evidence was rewritten.

These observed differences do **not** prove which field caused the original
Setup read refusal in run `38529654044`; that run logged no differing field and
retained no original Setup bytes. This preflight builds no installer and grants
no installed-product, GUI/UAC, boat or release acceptance. Authenticated
packaging continuation and the remaining acceptance gates stay separate.
