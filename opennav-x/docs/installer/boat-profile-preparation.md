# Boat profile and managed-plugin preparation

These two maintenance tools prepare the actual Windows installation for a later
commissioning audit. Neither launches OpenCPN, changes connections, quarantines
plugins, sends vessel commands or changes remote-access services. They require
the exact supported stock OpenCPN 5.12.4 x86 executable, SHA-256
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`, its PE version
and architecture, and one interactive desktop belonging to the current SID.

All application/plugin helpers must be closed normally. No tool kills processes.
The `LOCALAPPDATA` environment must match the current account's known folder.
Custom plugin roots, redirected paths, overlapping recovery destinations and
unrecognized profile syntax are refused. Raw paths, plugin ownership manifests,
cached packages and chart/license metadata stay in private local evidence; do
not commit or upload the resulting recovery directory.

## Managed plugin cold backup

Run `tools/boat/backup-managed-plugins.ps1 -Workspace C:\XNav` after profile repair,
or pass `-ProfileCandidate <actual-same-directory-TMP>` before repair. The latter
accepts only the exact recovery pair described below; it uses the TMP solely to
determine whether Windows plugin-root configuration is unambiguous.

The backup includes the complete trees, including empty directories:

* The interactive account's `LOCALAPPDATA\opencpn\plugins`.
* The supported executable's `plugins` directory.
* The real shared profile's `plugins` directory, including `install_data` and
  cached plugin archives.
* The real profile's `ocpn-plugins.xml`, when present.

The tool records a flushed preparation journal before copying, hashes every
file, checks the full source inventory again, verifies every copied byte and
publishes the completed payload directory with an atomic rename. An incomplete
copy retains its partial directory and has no verified completion record.
The newly created recovery directory grants access only to the current SID,
administrators and SYSTEM. No original file or source ACL is changed.

This backup closes a gap in the earlier application/ProgramData recovery set:
managed plugin DLLs may live in the interactive account's local application data.
Possessing a backup does not qualify a plugin constructor or hardware output.

## Exact zero-filled INI repair

`tools/boat/recover-zero-profile.ps1` supports only the inspected 21,380-byte pair:

| File | Required SHA-256 |
|---|---|
| Entirely zero-filled `opencpn.ini` | `f84114bfa5c456c4b8f5f073141abb95537155f9f75d78dedc12db3ed5ab3867` |
| Valid same-directory `opencpn.ini~….TMP` | `a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9` |

There is no command-line switch to broaden these hashes or accept another build.
The TMP must pass strict UTF-8 INI parsing without duplicate keys/sections or
invalid control bytes. Its connection values are preserved exactly, including
any existing output configuration. Repair is **not** permission to launch.

1. `-Action Prepare -Workspace C:\XNav -Candidate <actual-TMP>` verifies the pair,
   snapshots the real profile, and durably preserves the corrupt original and
   exact candidate. It returns a prepared-record path and SHA-256. No profile
   bytes change.
2. Review that record. `-Action Apply -Workspace C:\XNav -Record <prepared-record>
   -ExpectedRecordSha256 <returned-hash>` rechecks the executable, SID, source
   paths, both fixed hashes, full profile inventory, original ACL and closed
   processes. A separate flushed apply journal precedes the same-directory
   native rename. The candidate bytes are copied without normalization or
   merging and flushed before publication. The new staging file receives the
   original's owner, group, DACL, creation time and supported attributes before
   replacement; its bytes and metadata are then checked independently.
3. `-Action Verify` with the same record/hash verifies exact recovered bytes,
   unchanged TMP, unchanged non-INI profile contents and preserved destination
   permissions. Apply performs these postconditions too.

The native disposable-file test exposed one Windows normalization: atomic
replacement changed the DACL control prefix from `D:` to `D:AI`, while owner,
group and every inherited ACE stayed identical. `AI` is the documented
[`DiscretionaryAclAutoInherited` control flag](https://learn.microsoft.com/en-us/dotnet/api/system.security.accesscontrol.controlflags?view=netframework-4.8.1),
value `0x0400`. This is distinct from the protection flag and from each ACE's
inheritance flags.

Before Apply, the recorded and current SDDL are parsed by Windows
`RawSecurityDescriptor` and their complete serialized descriptors must match.
After replacement, comparison may ignore **only** `0x0400` in the descriptor
control word. Owner/group SIDs, every other control flag, protection, DACL/SACL
content present in the descriptor, and every ACE's order, type, rights, flags
and identity remain exact. The comparator neither sorts/merges ACEs nor sets
permissions. Invalid descriptors fail closed. Raw before/after SDDL and the
comparison policy are retained in private verification evidence; they must not
be uploaded because they contain account SIDs. No permission repair or blanket
`Set-Acl` is used on the original profile.

Windows Server CI subsequently exposed a descriptor-size difference despite
the boat-PC disposable tests passing: the final replacement added explicit
copies of the inherited ACEs. That change is **not** accepted as equivalent
permissions. Windows
[`ReplaceFileW`](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-replacefilew)
documents DACL merging; even preparing identical staging permissions did not
prevent this Server behavior. Native publication therefore uses
[`MoveFileExW`](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-movefileexw)
with fixed `REPLACE_EXISTING | WRITE_THROUGH` flags (`0x9`). It submits one
same-directory rename of the fully verified staging file. There is no
cross-volume copy/delete fallback, destination deletion, retry that weakens
sharing checks, delayed reboot operation, or automatic rollback. Microsoft's
[file-security contract](https://learn.microsoft.com/en-us/windows/win32/fileio/file-security-and-access-rights)
states that default inherited descriptors are assigned on creation, not rename.
The final descriptor comparison remains exact apart from the already observed
`0x0400` marker; additional, removed or reordered ACEs still fail.

The helper creates a fresh permission object containing original owner/group/
access sections and applies it **only to the new same-directory GUID-named
staging file**. It also preserves creation UTC and ordinary Hidden, System,
Archive, Normal or NotContentIndexed attributes. The replacement content's
last-write time reflects staging, not the original content. Original bytes and
metadata are checked again immediately before publication and destination
bytes and metadata immediately afterward. No original, saved candidate, backup
or parent ACL is written. If staging permissions cannot be made identical, the
operation stops before replacement.

CI run `36273516935`, commit `62e28e5dfe42e96531f00dd88686634c167b0db8`,
found a pre-stage comparison failure between an audited metadata descriptor and
an ordinary descriptor. Its log lacked the audited value. The subsequent
same-audit-scope attempt passed desktop Windows temporary tests but **failed
Windows Server CI** and is superseded; consistent audit reads are not sufficient.

The decisive evidence is run `36274020524`, job `108493169274`, commit
`eb2ba9e1f64ea43c24e6360a466035b5aa41703f`. On the same original file, ordinary
`Get-Acl` returned three inherited access entries `(A;ID;FA;...)`, while
`Get-Acl -Audit` returned them as explicit `(A;;FA;...)`. Both descriptor control
words were `32772` (`0x8004`): this was **an ACE inheritance-flag difference**, not
a descriptor control-bit normalization. Copying the audited DACL produced a
staging DACL with three explicit plus three inherited entries and control
`33796` (`0x8404`). The strict comparator correctly refused that six-entry
staging descriptor before profile replacement. No duplicate or altered ACE is
accepted as equivalent.

The implementation now keeps these views separate. Ordinary
`Get-PreparationAccessAcl` supplies the owner/group/DACL baseline copied to
staging and compared before/after publication. An independent
`Get-PreparationAuditedAcl` probe is used **only** to reject SACL/audit metadata;
its DACL is never copied, normalized or treated as the preservation baseline.
The ordinary view is read again after the audit probe and must match exactly,
catching an intervening permission change. The audited value is retained only
as diagnostic evidence. Owner/group, every ACE flag/order/type/right/identity,
and the existing narrow post-stage control policy remain unchanged.

Microsoft's
[`Get-Acl` documentation](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.security/get-acl?view=powershell-5.1)
describes `-Audit` as obtaining the SACL, while
[`SECURITY_INFORMATION`](https://learn.microsoft.com/en-us/windows/win32/secauthz/security-information)
defines separate owner, group, DACL and SACL information requests. The
implementation follows that separation and does not infer access permissions
from the audit query's transformed DACL. Native regression coverage deliberately
reproduces the observed audit-view loss of `ID` on a temporary file; actual
inherited access entries must survive unchanged. The separate changed-owner
pre-stage refusal and actual audited-file refusal remain mandatory. CI-only
fixture diagnostics include both SDDL views/control words and the captured
ordinary baseline; no private boat account descriptor is printed by local tests.

Rename replaces a file object; it does not merge extra metadata. The narrow
profile tool therefore refuses readonly files, specialized attributes such as
EFS/compression/sparse/offline, multiple hard links, alternate named streams,
and audit/SACL metadata. A successful explicit audit-descriptor read is required;
insufficient permission to inspect it is an error. These cases need separate
review, not an automatic lossy conversion. File identity is not preserved by
staged replacement. The tool does not claim a storage-device power-loss
transaction: flushed candidate/backups/intent, write-through rename and explicit
postconditions provide recovery evidence, while the journal resolves an
interruption. Closed-process checks remain mandatory in each calling maintenance
transaction. Native exclusive or delete-sharing locks stop publication without
terminating a process or modifying the destination.

Any intervening user change stops the operation. The repaired INI becomes the
working baseline for subsequent commissioning undo; the corrupt original is
retained only for forensics. No error path automatically restores zeros over a
working profile. If interrupted after replacement but before the verification
record, run Verify and inspect the existing journal; do not repeat Apply blindly.
An original still containing zeros can be prepared again only after inspecting
any interrupted staging file and resolving the recorded failure.

## Validation boundary

`tools/boat/test-preparation.ps1` runs disposable native Windows filesystem
contracts in CI. Explicit `-IsolatedLocal` permits the same tests in a unique
temporary directory; it does not access actual boat/profile/registry state.
Linux PowerShell may run `-PortableContracts`, substituting only temporary-path
syntax while exercising the real hash, parser, copy, journal and atomic-replace
code. These checks cover identity disagreement, live helpers, unknown binaries,
recursive redirects, changed source inventories, exact-byte recovery refusal,
intervening edits, duplicate journals and atomic publication. Native tests also
check exclusive file locks and ACL preservation/privacy.

The 21 Linux portable contract groups pass, including all 639 non-exempt bit
changes in a bounded descriptor fixture. Native tests add actual SDDL comparisons,
negative owner/group/protection/ordered-ACE/SACL cases, real rename permissions,
an explicit protected owner/group/DACL fixture, creation time/attributes,
different-length restoration, readonly/named-stream/hardlink/audit refusal and
delete-sharing lock behavior. The revised native 34-group suite additionally
reproduces the observed audit-view DACL divergence and rejects an altered ordinary
owner before staging. Fresh native Windows PowerShell 5.1 qualification passed
all 34 groups, recorded in the private evidence file
`evidence/local/boat-beta2/native-preparation-access-audit-final.json`.
The commissioning suite passed 21 groups and launch verification passed 11
against the same production implementation. Windows Server 2022 CI subsequently
passed preparation 34, commissioning 21 and launch verification 11 at commit
`586df3875a8157e17b27da782b131420d9e6fbd6`,
[run 36274989439, job 108495883804](https://github.com/ThereptileII/Work/actions/runs/36274989439/job/108495883804).
The log confirms the ordinary and audited DACLs still differ on that runner;
the successful suite therefore exercises the corrected separation on the
previously failing platform. The six-suite tooling result is recorded in
[`beta2-windows-tooling-586df38.json`](../evidence/beta2-windows-tooling-586df38.json).
This qualifies the maintenance tooling only, not the product build, release,
physical hardware, or actual interactive boat launch.
CI failure reports include only disposable fixture
SDDL and stack context; local boat test failures do not print account descriptors.
The private
inspected pair passed its exact byte/hash checks and strict parsing (783 keys).
Disposable native validation and actual boat execution remain separate gates. The exact
authorized boat repair was already completed and verified using the prior
boat-qualified implementation. These new tests are disposable future-tooling
hardening; they do not repeat or alter that recovered profile.
