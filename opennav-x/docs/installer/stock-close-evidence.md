# Application normal-close evidence

The original boat `Close` observer used `Get-Process`, then `CloseMainWindow`,
`WaitForExit` and a PowerShell `ExitCode` property read. The actual-application
fixture instead retained the handle from `Process.Start`. Those lifetimes were
different: [.NET Framework Process source](https://raw.githubusercontent.com/microsoft/referencesource/main/System/services/monitoring/system/diagnosticts/Process.cs)
requires a retained process handle in `ExitCode`'s `EnsureState(State.Exited)`.
`WaitForExit` only obtains and releases a temporary handle when one is not
already retained. A failed property read could consequently become an unavailable
PowerShell value, which the old comparison misreported as an application error.

The same defect existed in the installed `InteractiveJob` Close branch. Both
stock and installed close now use `Invoke-ReviewedNormalClose` in their already
shared `Common.ps1`. It retains the exact process handle before requesting
normal close, and rechecks the immutable PID/creation-time pair. It performs one
normal close request and a bounded wait. The explicit `get_ExitCode()` call must
return an integer; only a measured zero is success. Failure evidence distinguishes
whether close was requested, whether the wait completed, whether the handle was
retained and whether a numeric exit code is known. Nonzero codes are preserved.
Unknown codes remain unknown. There is no retry or forced termination.

The native marker fixture obtains a fresh `Get-Process` observer and covers normal
zero and nonzero exits, creation-identity refusal, and the original unretained
observer's failed getter. The same zero/nonzero cases also run the actual
`InteractiveJob.ps1` Close entrypoint against inert windows, without substituting
its function bodies or fabricating installation state. The actual official English/Swedish fixture also uses
the shared close primitive through a fresh observer and compares the measured
code with its independently retained starter object. Native qualification is
required before using the revised helper.

The earlier boat close result cannot recover a historical exit code. Normal
shutdown log markers, absent processes and absence of Windows crash events are
separate observations; they do not turn an unavailable exit code into zero.
Remaining chart helpers must still pass the existing cold-process guards before
any profile restoration or maintenance.
