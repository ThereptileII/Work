# Focused Windows startup observation and disconnected profile

Helper revision `1aa6275486ff6f981d3d46562c85705a3622b169`,
[run 38643305138 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38643305138),
job `116123937795`. Portable checks pass 60 probe and 17 profile-adapter cases.
This is a diagnostic helper execution, not another application build.

The create-only profile step **passes** on native Windows. The actual ProgramData
known-folder lookup, retained canonical directory checks, relative `NtCreateFile`
creation, file-handle conversion and maintained `.ini`/navigation observer all
execute. The controlled route/waypoint fixture and disconnected settings hashes
are retained in the [receipt](receipt.json). No existing profile was adopted or
changed. This does not prove application profile loading/migration or an installed
customer journey; no application was installed or launched by this step.

The original LIMITED-primary account probe still reaches its 60-second timeout,
so the overall job remains **failed**. Original process/thread identity verifies
before resume and at timeout; the single ResumeThread call returns 1. The bounded
observation reports:

- Before resume: module enumeration is unavailable with Windows error 299.
- At timeout: enumeration succeeds; only `ntdll.dll` is observed among the seven
  explicitly retained module names. Kernel32, KernelBase, Python312, UCRTBase,
  User32 and `_ctypes` are not observed by this query.
- Original primary-thread accounted kernel/user CPU is 0/0 in both samples;
  pending I/O is false and the bounded thread-window list is empty.
- No child progress bytes, entry marker, final report or exit are observed.
  Caller desktop and UAC policy remain unchanged (1/0/1).

These observations distinguish an early startup boundary but do not prove zero
instructions, inactivity of all threads, a complete mapped-image inventory,
permanent suspension, an exact loader wait, console cause or SKAGER crash.
No launch-policy/privilege change or equivalent retry is selected from these
observations. Cleanup remains correctly deferred to disposable job retirement.

Both original archives were authenticated against original run/job/upload
metadata, size, provider SHA-256, sole result member and CRC. Root independently
rechecked archive hashes, lengths, CRCs and member equality. The receipt excludes
account SID and process identifiers. Its account helper hash identifies the
unchanged `installed_customer_launch.py` child under Windows CRLF checkout,
not the new parent observer. Private original evidence stays outside Git.

Application candidate `89854110b41286cab0c6bf9748f37882a595a366` remains unchanged.
Native installed/UAC, joined update/recovery and boat acceptance remain pending.

## Original primary-thread state follow-up

Helper `f35b52768d3d7596b538ad5379d40890215caf57`,
[run 38645101455 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38645101455),
job `116129891765`, adds the bounded read-only Windows process/thread snapshot.
The 68 focused probe tests pass and independent source review found no material
findings. The native job still fails on the original child timeout; disconnected
profile preparation passes. Root independently verifies both original archives'
size, provider digest, CRC and sole member against the retained originals.
[Sanitized follow-up receipt](thread-state-receipt.json).

Both native queries succeed with status 0 and report exactly one thread in the
original process. Before the sole resume, primary state is 5 and wait reason 5.
At timeout, primary state is 5 and wait reason 17. Accounted user CPU increases
from 0 to 156,250 units of 100 ns (15.625 ms); kernel CPU remains 0. Original
identity checks pass, `ResumeThread` returns 1 once, and no file marker or exit
is observed. Module/I/O/window observations remain as above.

The changed wait category and nonzero accounted CPU rule out describing this
execution as never resumed or never scheduled. They do not identify the blocking
server, exact function, missing permission, console cause or a SKAGER defect.
No extra resume, privileged debugger, policy change or application rebuild is
authorized by this diagnostic. Cleanup remains deferred to disposable retirement.

## Explicit detached-console request refused

Helper `e97356e164e8ea2497dd9bc7d24fc29d10e02e77`,
[run 38645846804 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38645846804),
job `116132371417`, changes only this file-only child's requested creation flags
to `CREATE_SUSPENDED | DETACHED_PROCESS` (12). The existing 68 focused cases pass.
The [Microsoft flag contract](https://learn.microsoft.com/en-us/windows/win32/procthread/process-creation-flags)
lists the flag, but this actual `CreateProcessWithTokenW` invocation refuses with
Windows error 87 (`ERROR_INVALID_PARAMETER`) before creating a child. There is
no resume, worker marker, exit report or startup snapshot in this run. No fallback
was attempted; only the original owned disposable account/profile was removed.
Caller desktop and UAC checks remain unchanged. The separate profile adapter passes.

[Original authenticated receipt](detached-receipt.json). Root independently
rechecked both archives' provider hashes, lengths, CRCs and sole members.
The result does not prove the original timeout's console cause, a usable
non-console setup or any installed/customer acceptance. The application candidate
remains unchanged. Do not repeat this rejected flag combination.

## Fixed GUI-entrypoint result and host boundary

Helper `7d0e051abe4709e505f0c30323633bbfff797d29`,
[run 38647259035 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38647259035),
job `116137096731`, uses only the actual interpreter's fixed same-directory
`pythonw.exe` sibling and `CREATE_SUSPENDED` (4). The original executable is
retained read-only with deny-write/delete sharing, hashed, and checked for
canonical path and stable file identity across launch and original-child exit.
All 73 focused probe checks pass; independent source/test review found no
material findings. The installed customer launcher and application are unchanged.

This native execution creates the matching LIMITED-primary child, resumes it
once (return 1), and observes original exit code 1. It no longer times out or
fails the process-creation call, but produces zero file-marker/report bytes.
The exact exception/startup cause is unproved; do not call this worker success.
Owned-account/profile cleanup and separate disconnected profile preparation pass.
Caller desktop and UAC policy stay unchanged. Root independently verifies both
original archives' lengths, provider hashes, CRCs and sole result members.
[Sanitized original receipt](pythonw-receipt.json).

Independently of this file-worker result, the recorded hosted environment cannot
qualify the actual customer/UAC gate. Its policy is `1/0/1`; Microsoft defines
[`ConsentPromptBehaviorAdmin=0`](https://learn.microsoft.com/en-us/windows/security/application-security/application-control/user-account-control/settings-and-configuration)
as elevation without prompting. The required genuine signed-in split-token
customer session is not established: the fresh account's FULL partner is an
identification-level impersonation token, not the original FULL primary context
required by `installed_customer_launch.py::_normal`. File-worker success would
not resolve either boundary. No further launch variants or diagnostic layers
are selected to qualify customer UAC on this unchanged hosted configuration.

The remaining interactive acceptance needs a disposable non-boat Windows VM/PC
with a real signed-in non-RID500 administrator, its original FULL/LIMITED primary
token pair, normal secure-desktop consent and human access to approve it. Retain
the same account/installation across qualified B0, B1, rollback and uninstall.
The maintained journey currently admits only explicitly opted-in GitHub-hosted
jobs; another host needs an explicit reviewed admission path, not environment
spoofing or weaker token/approval/package checks. Native component/build results
remain useful and separate from this outstanding installed-customer acceptance.
