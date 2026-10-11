# SCRUM-311 suspended creation matrix: observations, overall run failed

[Run 38165504516](https://github.com/ThereptileII/Work/actions/runs/38165504516),
[job 114547116715](https://github.com/ThereptileII/Work/actions/runs/38165504516/job/114547116715),
exact candidate `afa5fbcf33cbf93f0f6c3f74fd72acf0b22086bf`, attempt 1,
**FAILED**. The diagnostic matrix step succeeded and retained all ten
observations. The following unchanged production-owner test failed at its first
`Start` with `Suspended image creation refused (Win32 87)`, before resume.
The diagnostic result does not make that production path pass.

The matrix used only newly compiled inert x86 markers on the disposable native
Windows runner. It compiled the exact embedded production P/Invoke owner and
called its private CreateProcessW signature through reflection. Every row used
CREATE_SUSPENDED; there is no diagnostic resume invocation. The acquired marker
image and its physical ancestors remained held after the private DOS alias was
remapped to a different inert marker.

Application and command-line argv[0] spellings always changed together:

| Application and argv[0] | CWD | Environment | Flags | Created | Win32 error |
|---|---|---|---|---|---|
| Original physical DOS | NULL/inherited | Owner Unicode | `0x404` | Yes | 0 |
| Original physical DOS | Original physical DOS | Owner Unicode | `0x404` | Yes | 0 |
| Original physical DOS | Held GLOBALROOT | Owner Unicode | `0x404` | Yes | 0 |
| Held GLOBALROOT | NULL/inherited | Owner Unicode | `0x404` | No | 87 |
| Held GLOBALROOT | Original physical DOS | Owner Unicode | `0x404` | No | 87 |
| Held GLOBALROOT | Held GLOBALROOT | Owner Unicode | `0x404` | No | 87 |
| Original physical DOS | Original physical DOS | NULL/inherited | `0x404` | Yes | 0 |
| Original physical DOS | Original physical DOS | NULL/inherited | `0x4` | Yes | 0 |
| Original physical DOS | Original physical DOS | Minimal Unicode | `0x404` | Yes | 0 |
| Original physical DOS | Original physical DOS | Owner Unicode | `0x08000404` | Yes | 0 |

All **seven DOS application/argv[0] rows created suspended children**, including
the GLOBALROOT-CWD row. All **three GLOBALROOT application/argv[0] rows failed
with error 87**, across NULL, DOS and GLOBALROOT CWD. Application UTF-16 lengths
were 119 for DOS and 154 for GLOBALROOT; quoted command lengths were 121 and
156. Explicit CWD lengths were 108 and 143. All recorded lengths were checked
against their retained strings; genuine NULL CWDs remain JSON null. The owner
Unicode environment contained 9,210 characters; its values were not logged.

These observations narrow this fixture's failing parameter combination to the
GLOBALROOT application/argv[0] pair, rather than demonstrating that GLOBALROOT
CWD alone causes refusal. They **do not distinguish lpApplicationName from
command-line argv[0]**, or their interaction. They do not prove general namespace
support, qualify a replacement path/alias strategy, or authorize weaker custody,
retry or fallback. Flags/environment controls used the DOS application pair;
they do not exhaust interactions with the GLOBALROOT pair. No production fix is
selected by this evidence.

Each successful child reported the exact held native image, a positive returned
PID and confirmed cleanup. The exact diagnostic source requires termination
through only that successful suspended call's returned raw process handle,
a bounded exit wait, and closure of both raw handles. Failed calls reported PID
zero and no image; their `cleanupConfirmed=true` means no returned child state
required cleanup, not that an existing process was terminated. All ten rows have
no failure field, the receipt reports `resumeAttempted=false`, and private fixture
cleanup is confirmed. A failed call does not authorize production fallback.

All five recorded local alias states match the exact owned stack: add A gives
`[A]`; add B gives `[B,A]`; exact removal of B gives `[A]`; exact removal of A
gives absence/error 2; the final independent query again gives absence/error 2.
The operator SID is not LocalSystem. No preexisting or global definition was
selected. These checks concern the disposable diagnostic alias only.

The later production test's raw `owner.log` has 16 preceding PASS lines, then
the unchanged error-87 creation refusal. It establishes no successful marker
execution, resume, health, normal/interrupted cleanup or uncertain-resume result.
Its failure and the overall run/job failure are preserved without reclassification.

`audit.json` verifies the terminal API candidate/run/job/attempt and artifact
ownership; all ten names, flags, string lengths, outcomes and cleanup fields;
all alias-stack states; and the production failure. Both native PowerShell source
hashes match this exact Git commit after LF-to-CRLF conversion. The embedded
C# here-string body independently matches the retained native embedded-source
hash. The receipt records Git/native hashes and the transformation explicitly.
No source was compiled or executed during this audit.

Original artifact **11712011315**: **3,289 bytes**, API and local SHA-256
`d0e1146430356760dfa20df40f5478eddfd77928311645a19c9c839226e3b177`.
All **three ZIP members** passed CRC, unique case-fold path and plain-record
checks. The coordinator's already-downloaded artifact was used; no second
download occurred. `raw/` retains three API responses, the job log and all three
artifact members: **seven byte-exact records**, **88,502 bytes** total, each
listed with its original name, length and hash. Original UTF-16 log bytes and
JSON BOM/CRLF bytes are preserved. `owner-decoded.log` and `matrix-decoded.log`
are derived UTF-8 copies with CRLF changed to LF for reading; the owner copy
also trims trailing whitespace. No ZIP, executable
or other native binary is committed.

This is an evidence-only increment. Production, product and boat acceptance
remain false. It changes no source, workflow, status or Jira state and performs
no test, build, dispatch, application execution, boat action or external transfer.
