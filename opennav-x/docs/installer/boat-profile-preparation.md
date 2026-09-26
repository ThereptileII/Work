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
then found a pre-stage comparison failure:
metadata capture used `Get-Acl -Audit`, while the three staging checks used plain
`Get-Acl`. The logged ordinary descriptors matched; the captured audited
descriptor was absent from that failed-run log, so the precise differing bit is
**not established** by that evidence. The failure happened before staging ACL
application or profile replacement.

Native metadata capture and all staging descriptor comparisons now use the same
explicit audited query through `Get-PreparationAuditedAcl`. Microsoft's
[`Get-Acl` documentation](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.security/get-acl?view=powershell-5.1)
identifies `-Audit` as an additional SACL query; the maintained PowerShell
[`ProcessRecord` implementation](https://github.com/PowerShell/PowerShell/blob/v7.4.13/src/Microsoft.PowerShell.Security/security/AclCommands.cs#L743)
adds `AccessControlSections.Audit` to the owner/group/access request. Comparing
different query scopes is therefore avoided rather than presuming their
serialization is identical. This change adds no ignored control flag, does not
relax the strict pre-stage comparison, and leaves SACL refusal intact. Native
regressions enforce the query scope and prove that an altered audited owner is
rejected before either original or staging permissions change. CI-only fixture
diagnostics now include both ordinary/audited SDDL and control flags, as well as
the actual captured metadata baseline; private account descriptors remain out of
local boat-test output.

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

At this revision 21 Linux portable contract groups pass, including all 639
non-exempt bit changes in a bounded descriptor fixture. Native tests add actual
SDDL equivalence, negative owner/group/protection/ordered-ACE/SACL cases, and
the real rename permission check and an explicit protected owner/group/DACL
fixture. Added native cases verify creation time/attributes, different-length
reviewed restoration, readonly/named-stream/hardlink/audit refusal, and a
read-sharing handle that permits preflight but rejects the final rename while
retaining the exact stage and original. All 32 native PowerShell 5.1 groups
passed on the boat PC in a unique temporary directory. Private result:
`evidence/local/boat-beta2/native-preparation-movefile-final.json`.
Windows Server CI qualification of this rename change remains pending.
After the same-query-scope correction, all 21 portable groups and all 34 native
PowerShell 5.1 preparation groups passed. The related commissioning and launch
verification suites also passed 21 and 11 native groups respectively. These ran
only in disposable temporary trees; private results are
`evidence/local/boat-beta2/native-*-audit-scope.json`. Windows Server CI must still
qualify the same correction in a fresh published commit; earlier Server failure
and prior native 32-group results do not substitute for that gate.
CI failure reports include only disposable fixture
SDDL and stack context; local boat test failures do not print account descriptors.
The private
inspected pair passed its exact byte/hash checks and strict parsing (783 keys).
Native PowerShell 5.1 and actual boat execution remain separate gates. The exact
authorized boat repair was already completed and verified using the prior
boat-qualified implementation. These new tests are disposable future-tooling
hardening; they do not repeat or alter that recovered profile.
