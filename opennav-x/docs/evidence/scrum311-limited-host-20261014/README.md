# Genuine limited-primary host probe — 2026-10-14

[Run 38615961316](https://github.com/ThereptileII/Work/actions/runs/38615961316)
used exact helper `51ef8373d43ab2a32d2f6df712f72f188f1dcb67` in a disposable
GitHub-hosted Windows account. The original artifact's source/name, SHA-256,
831-byte length and every ZIP CRC were verified; the sanitized receipt is retained
here. Ephemeral account/SID/session authentication identifiers are deliberately
omitted. The full original report remains private evidence.

The original linked FULL token was actually an impersonation token at
SecurityIdentification (level 1), supplying the missing discriminating fact for
the earlier conversion refusal. This run did not convert or modify that token.
It used the genuine LIMITED PRIMARY logon token for the fixed read-only noargs
capability child. Native child identity, authentication context, image and original
process/thread handles passed the checks before its single ResumeThread, which
returned the expected previous suspension count of one.

The child did not exit within the fixed 60-second bound. The exact outcome is
`original-capability-child-timeout`, not successful desktop or customer acceptance.
The result does not establish the cause of the timeout. Cleanup preserved uncertain
process/account/profile state for destruction with the disposable job; it did not
kill or retry the child or change desktop ACLs, token levels, privileges or UAC.
No SKAGER product, installed service, boat profile or physical equipment was used.

The helper change passed 39 focused portable probe cases and 31 existing customer
launch cases before native dispatch; independent review found no concrete source
blocker. Those results do not supersede this native refusal. Actual ordinary-user
customer/UAC/installed lifecycle qualification still needs a usable authorized
Windows session. This result does not qualify that environment or close SCRUM-311.
The separate full product candidate remains `9b12442d` in run38615517743; this later
helper neither rebuilt nor relabelled that candidate.

## Subsequent standard-handle diagnostic contract

The maintained probe now compares each supplied stdin/stdout/stderr slot in the
original suspended child with its retained parent NUL/pipe object, before the
single resume. `DuplicateHandle` uses the original child as source and the parent
as destination, with `DUPLICATE_SAME_ACCESS`, no inheritance and no
`DUPLICATE_CLOSE_SOURCE`; `CompareObjectHandles` checks object identity. Only
booleans and numeric errors are retained. Observer duplicates join the existing
owned-handle cleanup, and original child identity is checked before and after
observation. A failed comparison refuses resume and defers the created child's
cleanup to the disposable job. No handle is injected into the child, and no
token, privilege, desktop/ACL, launch flags or 60-second budget is changed.

This is a prepared discriminator for missing diagnostic delivery, not evidence
that standard handles caused either retained timeout. Native observation and
actual interactive customer/UAC acceptance remain separate and pending.


## Fixed actual-token profile diagnostic channel

The subsequent native standard-handle observation proved all three supplied
numeric slots invalid in the original suspended child (error 6, no resume).
The maintained correction retains `CreateProcessWithTokenW`, `LOGON_WITH_PROFILE`,
the original LIMITED PRIMARY token, explicit desktop, original process/thread
checks, one suspended child/one resume, verified exit and the 60-second bound.
It supplies no standard handles and performs no handle injection, ACL changes,
privilege enablement, token conversion or fallback launch.

Only the literal `--disposable-profile-capability` worker mode changes delivery;
the existing noargs CLI remains unchanged. The worker resolves its destination
with `GetUserProfileDirectoryW` on its actual current process token. The parent
resolves it from the retained original LIMITED token and compares it with the
existing exact fresh-account SID/profile registration. Neither side chooses a
path from arguments or environment variables; no environment ownership is
inferred from profile loading or `lpEnvironment=NULL`.

The child exclusively creates one fixed basename with default inherited security.
Both sides retain no-reparse ancestor handles against rename. The parent retains
one original deny-delete file owner across bounded reads, checking full native
file ID, actual fresh SID owner, one link and append-only prefixes. No overwrite,
path-based replacement adoption or permission repair is allowed. Each fixed
marker and final JSON document is flushed. The first file marker proves worker
dispatch and successful diagnostic-file opening after module imports; it does
not prove earlier interpreter progress. Timeout/refusal retains only sanitized
bounded marker/count/document-presence facts. Capability still requires the
original child's verified exit 0, stable final file and actual-child identity
comparison; file bytes alone cannot supply acceptance. Diagnostic owners drain
before the existing own-account/profile cleanup, and uncertainty still defers
cleanup to disposable-job destruction.

This fixes the proven invalid standard-handle request. Native execution is still
required; it does not establish the earlier timeout cause, usable interactive
customer/UAC capability, installed product acceptance or SCRUM-311 completion.

Primary contracts: [CreateProcessWithTokenW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createprocesswithtokenw),
[GetUserProfileDirectoryW](https://learn.microsoft.com/en-us/windows/win32/api/userenv/nf-userenv-getuserprofiledirectoryw),
[CreateFileW](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilew).


## Disposable-host desktop provisioning correction

The fresh-account probe now uses `CreateProcessWithTokenW` with `lpDesktop=NULL`
to request Windows' documented per-user access provisioning for the caller's
inherited desktop. This changes OS-managed window-station/desktop permissions;
it is not an unchanged-ACL claim. Only this dedicated, exact-source,
workflow-dispatched disposable-account probe uses the choice. Installed customer
launch helpers retain their explicit desktop behavior.

Before account creation and immediately before launch/resume, the probe requires
the actual original elevated primary caller in a nonzero session on visible
`WinSta0`, with `Default` input and thread desktops, and reads the actual UAC
policy. Caller desktop/token and policy must remain unchanged after observation,
including timeout/refusal. A changed post-observation state blocks success and
is retained without masking an earlier failure. The successful child's native
report must preserve the same UAC policy and existing exact LIMITED primary
identity/session and real interactive desktop checks.

The fresh same-machine non-RID500 account, original OS token/logon, fixed worker,
actual-token profile/file custody, one suspended child, one resume, verified exit
and 60-second bound remain required. No manual DACL editing, policy write,
privilege enablement, token conversion, alternate launch, product action or
manual permission rollback is added. Uncertain child/account state still belongs
to whole disposable-job destruction. Normal OS per-user provisioning here is
host preparation, not a relaxation of installed broker/client authority.

One subsequent native observation is needed to test this documented setup
difference. It may resolve the no-entry symptom; it does not establish the
historical stall function. A usable LIMITED child's desktop alone does not prove
an interactive Explorer sign-in, usable original FULL primary custody, installed
mechanics or human UAC consent. Silent-elevation policy remains visible and both
acceptance flags remain false. No further automatic retry follows refusal.

Microsoft distinguishes these APIs: [CreateProcessWithTokenW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createprocesswithtokenw)
documents inherited-desktop permission additions for NULL/empty `lpDesktop`;
[CreateProcessWithLogonW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createprocesswithlogonw)
requires application-provided access on current Windows and qualifies automatic
additions as Windows XP behavior. The existing token-based route is preserved.
