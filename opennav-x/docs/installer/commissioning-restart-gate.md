# Guarded in-app restart during boat commissioning

Implementation in progress; no native or physical acceptance is implied.
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

Current local qualification: 886 portable codec/policy checks pass on Linux;
the PowerShell parser/policy suite passes 209 checks. Native compilation and the
24 marker-process scenarios have not yet run. Those scenarios cover all three
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
