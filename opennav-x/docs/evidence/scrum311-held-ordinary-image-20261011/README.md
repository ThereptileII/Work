# SCRUM-311 ordinary application creator custody

Selection: Jira comment 11326. Isolated baseline:
`560721d704f7895ff8b08eb15f269936bfb66d76`. This is Go launcher source and
focused test evidence, not Windows qualification or SCRUM-311 acceptance.

`ordinary()` passes the selected generation's managed `app/opencpn.exe` SHA-256
to `nativePlatform.start`. The existing receipt-bound private bootstrap direct
route passes that same expectation; its receipt/expectation checks are retained.
XNav, Legacy, Safe Mode, Later/offline, recovered and installed fallback ordinary
entries share this creator. Pending, selected-generation, transaction lock,
known-good, supervisor, `wait` and progress semantics are retained.

The creator opens the candidate as a single-link ordinary file with read sharing
only, hashes its actual handle within the existing 512 MiB bound, and obtains its
normalized NT physical name. Only local `\Device\HarddiskVolumeN` names on NTFS
are supported. It holds the true volume root and every physical ancestor above
the image, including ancestors above a SUBST root. Directories share read/write
but deny delete; the image denies write/delete. Each held directory must be an
ordinary same-volume directory, with stable native name and file identity.
Unsupported namespaces or unavailable inspection refuse launch.

The same acquired physical name supplies nonnull `CreateProcessW` application
and command-line image, and the physical parent supplies CWD, all through
`\\?\GLOBALROOT`. Creation uses `CREATE_SUSPENDED | CREATE_UNICODE_ENVIRONMENT`,
no inherited handles, and an explicit bounded environment with every
case-insensitive `SKAGER_UPDATE_*` variable removed. DOS drive letters and volume
GUID aliases are discovery inputs and are not reused as creation authority.

Before resume, returned raw process/thread handles corroborate PID, TID,
process-of-thread, repeated nonzero creation FILETIME, live process and native
image name. Held image/ancestor metadata, native names, link count and managed
hash are rechecked, and a physical reopen must match the held image identity.
The creator checks raw identity again, marks its attempt, and calls ResumeThread
once. Success requires previous suspend count exactly one. There is no wall-clock
interval assumption. QueryFullProcessImageName corroborates a native filename;
it does **not** directly retrieve the mapped image section's file ID.

All creator errors return failure with no alternate path, retry or fallback.
Pre-resume refusal may terminate only a successfully created suspended child
whose raw identity is owned and unchanged, with no resume attempt. Unproven
ownership or any resume attempt withholds termination. Cleanup never kills by
PID or name. Image/ancestor/process handles close after ordinary creation/resume;
this does not provide custody for the application's health lifetime.

`results.json` records exact local commands, toolchain, source hashes, results and
private binary hashes. Linux tests exercise the production state machine's order,
one attempt, cleanup, identity/metadata refusals, resume uncertainty/count
refusals, and creator failure across actual launcher entry routes. The existing
fake start now requires the managed image expectation for every ordinary entry.
Windows 386/amd64 cross-compiles check the actual native implementation and inert
fixtures, but do not execute them.

Native fixtures copy only the test executable as `opencpn.exe`. They cover the
actual `nativePlatform.start` physical image/CWD, stripped environment and an
inheritable parent handle that must be absent in the child; held image write and
rename denial; ancestor rename denial; absence of child execution before resume;
one resume; post-attempt termination refusal; wrong hash/hardlink refusal; and
unproven/mismatched raw identity termination refusal followed by owned suspended
cleanup. No vendor factory, application, shell, network, profile or device runs.

The separate SUBST remap control requires
`SKAGER_LAUNCHER_DISPOSABLE_SUBST_TEST=1` on a disposable hosted Windows runner.
It rejects LocalSystem/global namespace, requires a previously absent DOS letter,
verifies the single owned target, uses NO_BROADCAST, and removes only the exact
owned target on all exits, checking restored absence. After acquisition it remaps
the alias to harmless wrong bytes; the production owner must still launch its
held physical image, and an ancestor above the alias root must resist rename.
This fixture has not run locally and must never run on the boat.

Separate counterpart evidence reported by the coordinator: the PowerShell
creator [native run 38163791374](https://github.com/ThereptileII/Work/actions/runs/38163791374)
completed file/ancestor holding and actual SUBST remap, then its real
CreateProcessW call failed with Win32 error 87. This is not an executed Go result
or a demonstrated cause. The native signature/environment/flags and physical
GLOBALROOT application/CWD combination need inert diagnosis. In particular,
this Go implementation's physical CWD compatibility remains an assumption
awaiting that diagnosis and its own native positive control. No DOS/GUID fallback
or guessed change is authorized by that failure. A timed-out advisory review of
the counterpart does not provide acceptance evidence.

Remaining gates: coordinator review and tiny native Win32 execution, including
the opt-in remap fixture; actual OpenCPN GLOBALROOT image/CWD and loader
compatibility; rebuilt exact product bytes and appropriate qualification;
broader process/script/helper/module/loader/profile custody and updater
acceptance; controlled boat commissioning. No full application build, workflow
dispatch, push, boat operation or external payload review was performed here.

Primary API contracts inspected:

- [GetFinalPathNameByHandleW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfinalpathnamebyhandlew)
- [Windows file namespaces](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-file)
- [GetVolumeInformationByHandleW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getvolumeinformationbyhandlew)
- [CreateProcessW](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-createprocessw)
- [GetProcessTimes](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-getprocesstimes)
- [QueryFullProcessImageNameW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-queryfullprocessimagenamew)
- [ResumeThread](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-resumethread)
- [QueryDosDeviceW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-querydosdevicew)
- [DefineDosDeviceW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-definedosdevicew)

The pinned `golang.org/x/sys/windows v0.43.0` wrappers and Go 1.27.1 command-line
quoting implementation were also inspected. These contracts do not substitute
for the pending native controls or product compatibility result.
