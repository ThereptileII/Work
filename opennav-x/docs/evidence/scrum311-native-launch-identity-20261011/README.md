# SCRUM-311: retained supervisor image owner

Selection: Jira SCRUM-311 comment 11318. Base:
`c63dee40445f4b084468e3076c7d33a90eea118a`. This is an isolated source increment,
not native qualification or signed-update/boat acceptance.

## Implemented boundary

`UpdateImageOwner`, embedded in the existing `UpdateTransaction.ps1` assembly,
resolves the input local drive once, then acquires the actual local NTFS volume
root and every physical ancestor through `GLOBALROOT`. This includes parents
above a SUBST root. Directory handles deny delete/rename; the single-link image
handle denies write/delete and supplies the expected SHA-256. All held native
names, volume/file IDs, attributes, size and file link count are reread before
creation and at binding points. Actual handle-normalized native names are kept
separate from the original DOS audit spelling.

Creation uses non-null `CreateProcessW` application name, the held-derived
physical image/CWD, a Unicode environment, no handle inheritance, and
`CREATE_SUSPENDED`. The returned process handle's native image path, PID and raw
creation FILETIME are checked, and the held graph is rechecked, before one
`ResumeThread`. Native-path equality is corroboration, **not** a mapped-section
file-ID query. Within the stated file-mutation threat, pinned physical names and
the deny-write/delete image prevent the selected path from being substituted.
No GUID or mutable DOS drive mapping is used for creation.

The real `Start-SupervisedGeneration` path retains that owner through pipe
health verification and failure cleanup. Each phase keeps the existing pipe
ACL/client PID, session/challenge/generation/commit, ordering, length, EOF and
deadline checks. Owner-backed identity uses cheap metadata instead of reopening
and rehashing the executable per frame. Known-good receipt writing additionally
requires the live retained owner. Direct `Receive` compatibility remains only
for existing protocol fixtures; it cannot create a known-good receipt.

Normal close uses the creator's retained process handle and creation identity,
including already-observed exit on that handle. It never forcibly terminates a
possibly running application. Only this owner's never-resumed suspended child
can be terminated on a pre-resume mismatch; uncertain cleanup still refuses
fallback. Handles are disposed by the existing session finally path.

## Refusal and record compatibility

New pending fields are explicitly distinct:

- `nativeLaunchV1`: `none`, durably `armed` before creation, then `running` only
  with a recorded PID and `processCreationFileTimeV1`.
- `processCreationFileTimeV1`: raw creation FILETIME decimal spelling; historical
  `processStartTicks` keeps its former UTC-tick meaning and is never reinterpreted.

Current-host acquire/hash/argument refusal with `CreationAttempted == false`
retains existing safe rollback. Once native creation is attempted, uncertain
create/resume, a lost managed return or failed identity persistence denies
fallback/rollback. An interrupted armed record retains evidence and requires
manual recovery even if no application is visible. Historical attempted records
remain readable but cannot grant the new native cleanup predicate; they also
require explicit recovery attention. Unattempted historical records remain
eligible for their existing guarded recovery.

Fresh interrupted cleanup has no historical creator custody. It only narrows
normal close to the recorded PID/FILETIME plus a newly acquired expected-hash
image and matching native process path. Missing/mismatched/unqueryable identity
refuses signaling/recovery. These user-writable fields never become positive
commissioning authority. Go's existing `decodeRecord(..., strict=false)` accepts
additional fields while retaining UTF-8/duplicate/case-fold checks; no Go
production source was changed in this increment.

## Actual local results

PowerShell 7.6.6 on Linux, private `TMPDIR`, both exit **0**:

```text
pwsh -NoProfile -NonInteractive -File opennav-x/tools/test-update-transaction.ps1
pwsh -NoProfile -NonInteractive -File opennav-x/tools/test-native-update-image-owner.ps1 -PolicyOnly
```

[results.json](results.json) records exact arguments and stdout SHA-256;
[transaction-policy.log](transaction-policy.log) and
[image-owner-policy.log](image-owner-policy.log) retain output. These compile the
exact embedded native C# and test record/recovery refusals without calling
Win32. Native branches explicitly skip. No Windows or product result is inferred.

## Exact remaining native recipe

In disposable Windows CI, Windows PowerShell 5.1, run only:

```powershell
powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File opennav-x/tools/test-native-update-image-owner.ps1
powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File opennav-x/tools/test-update-transaction.ps1
```

The first script compiles two tiny x86 managed marker fixtures from the dedicated
source, never the application. It owns/removes its otherwise-unused local DOS
alias and all child handles; no real installation or preserved profile is used.
Prepared native cases are **pending**, not passing evidence:

1. Actual acquired multi-link image refusal and handle cleanup; ordinary
   single-link acquisition, concurrent write/ancestor-rename denial, pin release.
2. Remap the local DOS alias A→B after acquisition but before creation. Only A's
   hardcoded marker may execute; B has genuinely different bytes and must never run.
3. Actual supervisor creation→authenticated health→normal close after alias
   remapping, with DOS audit spelling and native identity kept separate.
4. Expected-hash/unsupported-stream refusal before native creation, and actual
   invalid-image `CreateProcessW` refusal after arming.
5. Exact-source test-assembly injection after the real `ResumeThread` call:
   uncertain return preserves the possibly running inert child and refuses
   success. There is no production injection callback or allow flag.
6. Fresh interrupted cleanup refuses wrong FILETIME and normally closes only the
   exact matching inert child.

The second script retains all existing protocol negatives and actual supervisor
fixtures. Added cases cover direct protocol health without owner being unable
to write known-good, durable armed interruption refusal, pre-create owner argument
refusal allowing prior rollback, and lost return/post-create persistence failure
retaining pending/live child without fallback. Existing corrupt-candidate,
failed-health, protected receipt and refused graceful-close assertions remain.

## Limits and follow-up

Go ordinary/fallback creation, Lifecycle loader self-test, helper/trust/installer
children, initial shortcut launch, and PowerShell `-File` script resolution are
outside this PowerShell supervisor increment. The actual OpenCPN loader,
module-path/CWD behavior under `GLOBALROOT`, Win32 ABI and complete native
protocol/supervisor fixtures still require qualification. The native filesystem
boundary excludes arbitrary same-token process injection and kernel/mount-manager
compromise. It does not hold the DLL/plugin closure, PATH/environment authority,
preserved profile, settings mutation/restoration or authenticated commissioning
broker. Bootstrap/trust/pending refusals and full signed boat gates remain open.
The earlier incompatible supervisor draft remains withdrawn in its original
isolated worktree; it was not reused or relabeled as acceptance.

## Source basis

Microsoft documents [GLOBALROOT and NT namespaces](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-file#nt-namespaces),
[retained final native names](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfinalpathnamebyhandlew),
[explicit native process creation](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-createprocessw),
[suspended creation](https://learn.microsoft.com/en-us/windows/win32/procthread/process-creation-flags),
and [native process image naming](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-queryfullprocessimagenamew).
These define the API boundary, not a native test result. The completed updater-only
Claude Opus 5 review of seven files at the base commit addressed the earlier
proposal; it remains advisory and does not prove this corrected implementation.
