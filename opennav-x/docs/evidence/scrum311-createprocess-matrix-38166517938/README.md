# SCRUM-311 crossed argv[0] and held-GUID matrix: overall run failed

[Run 38166517938](https://github.com/ThereptileII/Work/actions/runs/38166517938),
[job 114550099612](https://github.com/ThereptileII/Work/actions/runs/38166517938/job/114550099612),
exact candidate `3d13f1e93f40099df20cc75edb8d77755b9a270a`, attempt 1,
**FAILED**. The diagnostic step succeeded and retained all seventeen named
observations. The subsequent unchanged production-owner test failed at its
first `Start` with `Suspended image creation refused (Win32 87)`, before resume.
Diagnostic observations do not make that production path pass.

This extends the [previous ten-row observation](../scrum311-createprocess-matrix-38165504516/README.md)
with separate application/argv[0] controls and four held-file-derived volume-GUID
application cases. Only newly compiled inert x86 marker images were used on the
native Windows-2022 disposable runner, under Windows PowerShell 5.1 x64. The
exact embedded production owner supplied the reflected CreateProcessW signature.
The image and physical ancestors remained held after the private input DOS alias
was remapped to different inert marker bytes. All rows used CREATE_SUSPENDED;
the diagnostic helper contains no resume invocation and rejects a marker file
that would indicate execution.

| Application | argv[0] | CWD | Environment | Flags | Created | Win32 error |
|---|---|---|---|---|---|---|
| Physical DOS | Physical DOS | NULL/inherited | Owner Unicode | `0x404` | Yes | 0 |
| Physical DOS | Physical DOS | Physical DOS | Owner Unicode | `0x404` | Yes | 0 |
| Physical DOS | Physical DOS | Held GLOBALROOT | Owner Unicode | `0x404` | Yes | 0 |
| Held GLOBALROOT | Held GLOBALROOT | NULL/inherited | Owner Unicode | `0x404` | No | 87 |
| Held GLOBALROOT | Held GLOBALROOT | Physical DOS | Owner Unicode | `0x404` | No | 87 |
| Held GLOBALROOT | Held GLOBALROOT | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |
| Physical DOS | Physical DOS | Physical DOS | NULL/inherited | `0x404` | Yes | 0 |
| Physical DOS | Physical DOS | Physical DOS | NULL/inherited | `0x4` | Yes | 0 |
| Physical DOS | Physical DOS | Physical DOS | Minimal Unicode | `0x404` | Yes | 0 |
| Physical DOS | Physical DOS | Physical DOS | Owner Unicode | `0x08000404` | Yes | 0 |
| Held GLOBALROOT | Basename | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |
| Held GLOBALROOT | Physical DOS | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |
| Physical DOS | Held GLOBALROOT | Held GLOBALROOT | Owner Unicode | `0x404` | Yes | 0 |
| Held volume GUID | Held volume GUID | NULL/inherited | Owner Unicode | `0x404` | No | 87 |
| Held volume GUID | Held volume GUID | Physical DOS | Owner Unicode | `0x404` | No | 87 |
| Held volume GUID | Held volume GUID | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |
| Held volume GUID | Held GLOBALROOT | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |

All **eight DOS application rows created suspended children**. All **five
GLOBALROOT application rows** and **four GUID application rows** refused with
Win32 87. With the same GLOBALROOT CWD, owner Unicode environment and flags,
GLOBALROOT application still failed with basename or DOS argv[0], while DOS
application succeeded with GLOBALROOT argv[0]. These crossed controls isolate
the application spelling in this fixture's observed GLOBALROOT refusal; they
do not establish an exact OS-internal cause or general namespace guarantees.
They do not qualify the later application-spelling correction, immutable naming,
loader/resource/environment custody, retry or production fallback.

The GUID name came from the retained file handle with normalized-name/GUID
flags 1, without a DOS fallback. Its exact suffix matched the held native image.
Reopened GUID root/file native names and volume/file identities matched the held
pins; single-link, size, type and no-reparse checks passed. Nine physical
ancestors remained held. The GUID namespace link itself was not pinned. All
four application cases still failed, including GLOBALROOT argv[0]/CWD; a
successfully reopened GUID path is not successful process creation.

Recorded application UTF-16 lengths are DOS 119, GLOBALROOT 154 and GUID 165;
argv[0] additionally uses basename length 10. Quoted commands have the exact
argv[0] length plus two. Explicit CWD lengths are DOS 108 and GLOBALROOT 143;
inherited CWD values remain genuine JSON null. Every retained string length and
row parameter was independently checked. The owner environment contained 9,210
characters; environment values were not logged.

Every successful row reports the exact held native image, a positive returned
PID and confirmed cleanup. Exact source inspection confirms cleanup uses only
that successful suspended call's returned raw process handle for termination,
a bounded exit wait, and closure of both raw handles. Failed calls report PID
zero and no image. Their `cleanupConfirmed=true` means no returned child state
required cleanup; it does not mean a process was terminated. Every row's failure
field is null, `resumeAttempted=false`, and private fixture cleanup is confirmed.
No diagnostic child was resumed. A failed native call authorizes no fallback.

The five alias audit states match the owned stack exactly: add A gives `[A]`,
add B gives `[B,A]`, exact removal of B gives `[A]`, exact removal of A restores
absence/error 2, and the independent final query confirms absence/error 2 again.
The operator SID is not LocalSystem. These checks concern only the disposable
private alias, not a preexisting or global mapping.

The unchanged production `owner.log` has sixteen PASS lines before error 87.
It establishes no successful marker execution, resume, health, normal/interrupted
cleanup or uncertain-resume result. The overall failed run and job are preserved.

[audit.json](audit.json) retains the independent provenance, source and row
checks. Both native PowerShell source hashes match exact Git candidate `3d13f1e9`
after LF-to-CRLF conversion, and the embedded C# here-string body independently
matches its recorded hash. No source was compiled or executed during this audit.

Original artifact **11712027529**: **3,860 bytes**, API/job-log/local SHA-256
`fc1044be9e58f9e77f1f7fef9cf7e199b04fcf2b7140e53dee89a250e56ac0ed`.
All **three ZIP members** passed CRC, unique case-folded path and plain-record
checks. The already-downloaded archive was used without another download.
`raw/` retains the three API responses, job log and all three ZIP members:
**seven byte-exact records**, **103,856 bytes** total, with original names,
lengths and hashes in the audit. UTF-16 log bytes and JSON BOM/CRLF bytes remain
unchanged. `owner-decoded.log` and `matrix-decoded.log` are derived UTF-8 reading
copies with CRLF normalized to LF; the owner copy also trims trailing whitespace.
No ZIP, executable or other native binary is committed.

This is an evidence-only increment. Production, product and boat acceptance
remain false. It changes no product, workflow, contract, status or Jira state,
and performs no rerun, native call, build, dispatch, boat action, network access
or independent external review.
