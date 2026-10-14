# Fresh-account Windows-managed desktop provisioning result

Exact helper `81cd082186d423c537717aa3877677bba357cd1d`,
[run 38639838756 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38639838756),
job `116112246866`. The probe alone uses the documented inherited-desktop behavior
of [CreateProcessWithTokenW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createprocesswithtokenw).
Windows is asked to add the fresh account's desktop access; no unchanged-ACL
claim is made. No manual ACL, policy, privilege, product or boat changes occur.
Focused checks pass 52 probe and 43 existing customer-helper cases.

The one native run still times out without an observed original-child exit.
ResumeThread returns 1; no progress marker or final document arrives. Caller
WinSta0/Default observations and actual UAC policy (1/0/1) remain unchanged and
pass before account creation, before launch, before resume and after observation.
The child's actual desktop/policy cannot be established without its report.
This provisioning correction therefore did not resolve startup and does not
establish an exact stall cause, SKAGER failure, usable FULL-token owner or human
UAC/customer acceptance. No repeated equivalent probe is selected.

Original artifact length/digest, exact API source/run/attempt/job, upload log,
sole result member/CRC and reported helper hash against the Windows CRLF checkout
verify. The root independently rechecked archive digest, length, CRC and exact
member bytes. Original private evidence is retained outside Git; the
[sanitized receipt](receipt.json) excludes account SID and process identifiers.
Cleanup correctly remains deferred to disposal of the uncertain runner state.
The separate full Staging candidate stays frozen at `89854110`; this helper
neither rebuilds nor relabels it. Actual installed qualification needs a usable
disposable interactive Windows context and its separately authorized inputs.

The prepared follow-up queries original retained handles before resume and on
timeout: [thread CPU time](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-getthreadtimes),
[pending I/O](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-getthreadiopendingflag),
a bounded [module allowlist](https://learn.microsoft.com/en-us/windows/win32/api/psapi/nf-psapi-enumprocessmodulesex),
and bounded [original-thread window classes](https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-enumthreadwindows).
Only numeric results, fixed module booleans and dialog/console/other classes are
retained. Identity is checked around each sample; diagnostics cannot replace the
first failure or grant acceptance. CPU deltas indicate activity, pending I/O is
momentary, mapped modules do not prove initialization, and absent windows do not
rule out loader failure or another process's dialog. No target text/paths, wait
chains, privilege changes, process reopening, suspension, console attachment,
window messages, kill, retry or timeout increase are added. Native observation
remains pending.

The next prepared discriminator adds
[SystemProcessInformation](https://learn.microsoft.com/en-us/windows/win32/api/winternl/nf-winternl-ntquerysysteminformation)
at those same two observation points. It bounds snapshot bytes, linked entries,
thread ranges and query attempts, then emits only the original process's thread
count, original primary thread's numeric state and its wait reason when waiting.
Original handle liveness and identity surround the query; malformed, unsupported
or failed snapshots remain unavailable and cannot change the launch result.
Names, addresses, reserved fields and unrelated thread data are not dereferenced
or emitted. This version-sensitive query supplies a momentary diagnostic fact,
not an exact stall cause or native acceptance. Native observation remains pending.
