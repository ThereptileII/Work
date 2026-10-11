# SCRUM-311 suspended process creation: failure and diagnostic matrix

[Run 38163791374](https://github.com/ThereptileII/Work/actions/runs/38163791374),
[job 114542010715](https://github.com/ThereptileII/Work/actions/runs/38163791374/job/114542010715),
at exact candidate `01ac92a64dc90d2f0521bd50d6f1088da4a6f04b` **failed**.
Its artifact `11711267075` is 942 ZIP bytes. API digest and independently computed
SHA-256 match
`5a7dfb93328928a8ac996528a78d468b3f608412285e4c423d847a418550aef3`;
the sole `owner.log` member passes CRC validation. `raw/` preserves byte-exact
UTF-16 native output and API/job receipts; `owner-decoded.log` is a UTF-8 decoding
with normalized line endings/trailing whitespace for reading. The ZIP, compiled
images and private fixture are not committed.

Sixteen PASS lines precede failure. They cover production parsing/embedded-owner
compilation, durable armed/historical/malformed native-identity refusals,
harmless hardlink creation and acquired-image refusal, distinct A/B marker bytes,
denied concurrent image writes, and the pinned physical ancestor above the SUBST
root refusing rename. The first actual owner `Start` then reports
`Suspended image creation refused (Win32 87)`.
It fails at `UpdateTransaction.ps1:242`, before the primary-thread resume at
line 251. No successful marker execution, alias-safe process creation, health,
normal/interrupted cleanup or resume-uncertainty result is established by this
run. Production still conservatively arms uncertain creation and denies fallback.

The failing call simultaneously supplies `\\?\GLOBALROOT` application and CWD
paths, a sorted UTF-16 environment block and flags `0x404` (suspended plus Unicode
environment). The inspected P/Invoke signature uses mutable `StringBuilder`
command storage and `SetLastError=true`. The startup structure matches the
documented field order; portable reflection records 104/24-byte startup/child
structures on this 64-bit host. These checks narrow the investigation but do
**not** identify which argument caused error 87. A GLOBALROOT CWD compatibility
problem is a hypothesis, as are path length or a different runtime parameter
constraint. Microsoft documents API-specific namespace support; successful file
opens do not prove process-creation/CWD compatibility.

The new `tools/diagnose-native-update-createprocess.ps1` compiles the exact
embedded owner and invokes its private native signature through reflection.
It changes no production source. It holds an image acquired through a temporary
local DOS alias, remaps that owned alias to a different inert image, then obtains
ten bounded observations:

| Rows | Application | CWD | Environment | Flags |
|---|---|---|---|---|
| 6 path combinations | original physical DOS / held GLOBALROOT | actual NULL / original physical DOS / held GLOBALROOT | owner-style Unicode | `0x404` |
| 1 control | original physical DOS | original physical DOS | inherited NULL | `0x404` |
| 1 control | original physical DOS | original physical DOS | inherited NULL | `0x4` |
| 1 control | original physical DOS | original physical DOS | minimal Unicode | `0x404` |
| 1 control | original physical DOS | original physical DOS | owner-style Unicode | `0x08000404` |

Every successful call must return its own suspended child, whose native image
matches the held marker. No resume call exists in the diagnostic. Only the raw
handle returned by that successful suspended call permits inert-child termination;
the wait is bounded to 10 seconds and both returned handles are closed. Failed
creation with unexpected returned child state, image mismatch or uncertain
cleanup records failure and stops further cases. A refused call with zero
returned handles requires no owned-handle cleanup; this is not proof authorizing
production fallback. Individual CreateProcess calls are synchronous; the native
CI job provides the outer deadline, not a precise real-time cleanup guarantee.

The receipt records the immediate Win32 error, startup/child sizes and offsets,
P/Invoke metadata, exact source hashes, native image, and UTF-16 application,
CWD and command lengths. Lengths exclude terminating NUL. Environment values
are never logged. An object parameter preserves a genuine NULL CWD: a local
binder check showed PowerShell converts `$null` to an empty string for a .NET
string parameter, while an object parameter retains NULL.

Native execution requires the exact repository `ThereptileII/Work`,
GitHub-hosted Windows CI, Windows PowerShell 5.1 and a non-LocalSystem identity.
Only an initially absent local alias is used. Full QueryDosDevice MULTI_SZ
results must match the owned definition stack after each addition/removal;
removal uses the exact-owned-target flag, and final original absence is audited.
No global definition, preexisting alias, real app, installer or operator profile
is selected. Cleanup uncertainty retains the disposable fixture and fails.

Coordinator invocation, in a fresh Windows PowerShell process:

```powershell
& ./opennav-x/tools/diagnose-native-update-createprocess.ps1 `
  -EvidenceDirectory ./opennav-x/evidence/local/native-update-createprocess
```

The new source and exact embedded owner compile on local PowerShell 7.6.6 with
`-CompileOnly`, with zero native rows (`portable-compile.json`). The native path
also refuses this non-Windows host before fixture creation and retains a failed
zero-row receipt (`portable-refusal.json`/`.log`). The focused helper probe
refuses absent `CREATE_SUSPENDED` before owner reflection/native access and
confirms the object/string NULL binder distinction (`portable-suspended-guard.log`).
The first private probe selected an extra AST string literal and refused helper
ambiguity; selecting the unique diagnostic class produced the retained passing
result. A fresh evidence directory is required. **The ten native
observations have not run.** An `observed` diagnostic receipt will only mean all
rows and cleanup were collected; it will not turn the unchanged owner test or
production acceptance green. No production fallback/change, application build,
dispatch, push, external code transfer, boat or hardware action is included.

Primary references checked for this diagnosis:

- [CreateProcessW parameters, mutable command line, environment and CWD](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-createprocessw)
- [STARTUPINFOW field order and reserved fields](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/ns-processthreadsapi-startupinfow)
- [Suspended, Unicode environment and no-window creation flags](https://learn.microsoft.com/en-us/windows/win32/procthread/process-creation-flags)
- [File namespaces and API-specific support](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-file)
- [QueryDosDevice current and prior mapping strings](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-querydosdevicew)
- [DefineDosDevice exact removal and local versus global namespaces](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-definedosdevicew)
