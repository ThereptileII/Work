# Guarded in-app restart during boat commissioning

The standalone native transport gate has passed; complete broker/product and
physical acceptance remain separate gates.
This opt-in guard does not change ordinary XNav/Legacy/Safe restart behavior.
It applies only to an explicitly prepared read-only commissioning session.
No guard invokes a shell command, modifies the boat profile, changes connections,
or grants permission to operate equipment.

## Boundary

The existing `PrepareClose` hook runs before the pinned OpenCPN shutdown saves
all configuration and unloads plugins. `CompleteRestart` runs from `OnExit`;
the companion helper waits for the old process to exit before creating the new
one. A pre-click INI hash cannot authorize the bytes saved during shutdown.

The optional gate sits in that companion helper after successful parent exit
and before its sole replacement `CreateProcessW`. A separately started,
same-user/session PowerShell verifier reuses the complete commissioning audit
against the final saved INI and current plugin trees. A strict reviewed delta
policy must account for normal persistence; it must not blindly renew a profile
hash or allow an arbitrary `Settings/*` change. The independent cold-launch
audit remains unchanged.

Retained plugin shutdown callbacks must also be reviewed. This gate cannot
prevent or undo a command issued by the old process during shutdown. Known
gateway initialization/query behavior remains a separate hardware boundary.

## Arming and transport

The audited cold launcher supplies both environment variables:

* `OPENNAV_COMMISSIONING_RESTART_SESSION`: random 256-bit lowercase hex session.
* `OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256`: exact immutable session-record hash.

Native static initialization captures both before plugins load. Presence of
either variable, including an empty or malformed value, arms refusal unless both
are valid. The replacement receives the same captured binding even if the
running application later changes its environment. A missing listener during a
later switch never permits the ordinary unguarded path.

The same initialization captures the complete audited environment and working
directory before OpenCPN adds plugin search directories. The helper and child
receive that original block with only the verified PATH and immutable binding
substituted. The request must match the cold-session record. Runtime additions,
deletions or changes to PATH, APPDATA or LOCALAPPDATA cannot redirect the child's
DLL or plugin search. Transport deadlines are checked before starting each I/O;
already-expired calls do not read or write bytes.

The fixed local pipe is `\\.\pipe\OpenNavX-CommissioningRestart-<session>`.
The verifier must create its restricted pipe deliberately before one reviewed
mode-switch action. It must reject remote clients and compare the OS-reported
client PID with the exact installed helper, user, session and creation time.
The helper checks server user/session and the Windows System32 native PowerShell
image. No executable path or command to run is accepted from the application.

In the armed path only, the parent duplicates a handle to itself with
`SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION`. An explicit inherited-handle
list passes only that genuine handle to the companion. The helper checks PID,
image, creation time and exit code 0; it cannot substitute a reused PID.
The ordinary restart branch retains its existing process-wait behavior.

The image proof compares `QueryFullProcessImageNameW(parent,
PROCESS_NAME_NATIVE)` with `GetFinalPathNameByHandleW(executable,
FILE_NAME_NORMALIZED | VOLUME_NAME_NT)` on the exact opened executable. PID,
creation and successful exit remain independent mandatory checks. A failed
query or different file path refuses the transition. No delay or fallback skips
image proof when the parent exits before its companion initializes. The native
runner retains a genuinely exited marker-process handle and records both
Win32/native query success, error codes and the opened file's NT path, including
a negative comparison with the different helper executable.

## Actual product capability

The Windows application loader self-test calls the linked guard's
`ProtocolCapability()` without initializing a profile or plugins. The staged
helper separately answers `--commissioning-protocol-self-test`. Packaging
executes both actual binaries with the same app-local/OS-only environment and
requires matching integer protocol 1, successful reports and no declared side
effects. Old, missing, Boolean/string/float, mismatched or malformed reports
refuse packaging. Linux explicitly reports protocol 0.

Only those executed reports authorize `PRODUCT_BUILD.json` to contain
`commissioning_restart_protocol: 1`, alongside the exact application and helper
SHA-256 values. No version string or unexecuted metadata grants this capability.
Portable capability tests cover 30 acceptance/refusal cases in four groups;
the actual integrated Windows loader, package and installed behavior still need
their full native gates after merging this implementation.

## Wire format, version 1

Every message has a four-byte little-endian unsigned payload byte count, in
the range 1–65,536, followed by exactly that many bytes. No BOM, padding or
trailing fields are accepted. Request and receipt are UTF-8 JSON. Protocol is
the integer `1`; process IDs, FILETIME timestamps and error values are canonical
decimal strings to avoid JSON numeric precision loss.

Request fields, in the native encoder's order:

```
protocol, kind="request", session, recordSha256, nonce,
parentPid, parentCreatedFiletime, parentExitCode="0",
helperPid, helperCreatedFiletime, windowsSessionId,
executable, executableSha256, helper, helperSha256,
workingDirectory, path, arguments
```

`nonce` is a new random 256-bit lowercase hex value. `arguments` contains exactly
one of `--xnav`, `--legacy`, or `--safe-mode`; profile overrides, synthetic modes
and additional switches are refused. The request digest is SHA-256 of the exact
JSON payload bytes, excluding its four-byte length prefix.

Reply payload is a string vector: unsigned little-endian uint32 field count,
then a uint32 UTF-8 byte count and exact UTF-8 bytes for each field. Fields are
nonempty, no larger than 32,768 bytes and contain no control characters. An
allow reply has exactly these 17 fields:

```
OpenNavX.CommissioningRestart.1
ALLOW
session
recordSha256
nonce
requestSha256
issuedFiletime
expiresFiletime
executable
executableSha256
helper
helperSha256
profile
profileSha256
workingDirectory
path
permitId
```

A denial consists only of `[OpenNavX.CommissioningRestart.1, DENY]`; any malformed
or unexpected response also denies. Each permit has a new 256-bit hex ID,
matches the request binding and expires no more than ten seconds after issue.
The verifier journals it as consumed **before** sending ALLOW. Issue time is
assigned after expensive audits; the helper checks it again after final hashing.

The verifier validates complete installation, stock/profile context, quarantine,
dependencies, active commissioning records, requested mode and reviewed profile
deltas. Only then may it provide the exact resulting INI hash and clean child
working directory/PATH from the existing commissioning environment policy.
The helper independently rehashes executable, helper and INI with deny-write/
delete handles held through `CreateProcessW`. It preserves the armed binding in
an explicit Unicode child environment and permits no second launch attempt.

Receipt fields are:

```
protocol, kind="receipt", session, recordSha256, nonce, requestSha256,
permitId, status="started"|"failed", childPid, childCreatedFiletime, win32Error
```

Missing receipt after ALLOW is an uncertain outcome requiring process inspection,
not permission to retry. Parent exit wait is limited to 30 seconds, connection to
five seconds, verifier I/O to an absolute 120 seconds and receipt to five seconds.
No timeout kills or relaunches an application.

## Required qualification

Portable tests execute the actual framing/selection policy and reject truncation,
extra fields, malformed UTF-8, invalid IDs/hashes, unexpected arguments and
expired/future/overlong permits. These do not exercise Windows process creation.

Native Windows tests must additionally verify inherited-handle isolation,
clean-exit proof, wrong peer/user/session refusal, no listener, changed files,
profile/plugin/connection mutation, expired/replayed permit, exactly one child,
receipt uncertainty, unchanged ordinary restart and guard retention in the child.
Only disposable profiles and marker-only test executables may be used initially.

The standalone native gate builds the actual helper/platform implementation
against a marker-only process, without wxWidgets or OpenCPN marine code:

```powershell
cmake -S tests/commissioning-restart -B build/restart-native -A Win32
cmake --build build/restart-native --config Release --parallel 3
ctest --test-dir build/restart-native -C Release --output-on-failure
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test-commissioning-restart-windows.ps1 -Binaries build/restart-native/Release -Evidence evidence/restart-native
powershell -NoProfile -ExecutionPolicy Bypass -File tools/boat/test-restart-commissioning.ps1
```

### Fixture readiness publication (SCRUM-98)

Candidate `701149fea50bdfd0ce5d58d9dd1fb377ab4575d0`, native run
[36603349213](https://github.com/ThereptileII/Work/actions/runs/36603349213),
failed at the first `ReadAllLines(child--xnav.txt)` after file existence was
observed. Windows reported a sharing `IOException`. The exact process holding
the file was not identified. Source inspection did establish a fixture race:
the final marker name existed before the fixture closed its output stream.

The marker-only executable now uses `tests/commissioning-restart/AtomicMarker.h`
for parent, child and chained readiness. It reserves a unique sibling staging
directory, checks write/flush/close, then atomically renames the closed file to
the final name. Failure does not publish readiness and removes staging. Marker
paths have one producer; Windows rename refuses a pre-existing final marker.
The reader's original bounded existence wait and single read remain unchanged;
no retry hides sharing failures. The harness also requires complete two-line
parent and ten-line child records before interpreting them.

`marker_publication_tests` is test-only and runs in both the portable suite and
the standalone native gate. It proves final-name absence while the stream is
open, first-read completeness, preservation of existing destinations, cleanup
after preparation failures, and 16 concurrent publication/read cases. Native
Windows additionally opens the closed staged file with exclusive sharing to
prove the writer handle is released before publication. Production restart
guard, helper, protocol, deadlines and process-identity checks are unchanged.

Local development passed **70 portable publication checks**, the corresponding
CTest entry and PowerShell syntax parsing. Evidence is under
`evidence/local/scrum98-marker/`. These do not qualify Win32 sharing or the full
24-case process matrix. A fresh exact-commit native run is required; the failed
candidate evidence remains relevant until replacement qualification passes.

### Previously accepted protocol/process evidence

Current local qualification: 886 portable codec/policy checks pass on Linux.
Native run [36282089707](https://github.com/ThereptileII/Work/actions/runs/36282089707)
at `fe397d85727dac015cea11615ae8b66f5caf788a` passed MSVC compilation,
886 codec checks and 359 process/I/O assertions across all 24 marker cases.
The downloaded artifact hash is recorded in
[the sanitized evidence record](../evidence/commissioning-restart-fe397.json).
The genuinely exited-process probe confirmed Win32 image-query failure
`ERROR_GEN_FAILURE` (31), successful native image query and exact equality with
the held executable file's NT path. The immediate-parent-exit case now starts
one bound child and returns helper exit 0. Earlier .NET Framework tests found
premature
`AsyncWaitHandle` disposal; the verifier now lets `Task.Factory.FromAsync` own
the matching `End*` call and checks bounded cancellation of pending connect/read.
The 24-case matrix covers all three
allowed modes, unchanged unarmed behavior, malformed or empty arming, missing
listeners, failed parent exit, tampered/expired permits, changed files, retained
startup binding/environment and refusal of a subsequent switch without a broker.
These results do not qualify the complete PowerShell cold-audit integration or
actual installed OpenCPN restart, which require their separate native gates.

After those gates pass, real boat acceptance uses one deliberate transition at a
time, records parent/helper/child identities, observes the native window and
chart content, and rechecks the commissioning state. Cold close plus separately
audited launch remains a different test and must not be called in-app restart.

Native API references: [restricted inherited handles](https://learn.microsoft.com/en-us/windows/win32/procthread/inheritance),
[handle-list attribute](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-updateprocthreadattribute),
and [named-pipe server identity](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-getnamedpipeserverprocessid).
Native image-path flags: [QueryFullProcessImageNameW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-queryfullprocessimagenamew)
and [GetFinalPathNameByHandleW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfinalpathnamebyhandlew).
