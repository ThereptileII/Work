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
   processes. A separate flushed apply journal precedes the same-directory atomic
   replacement. The candidate is copied without normalization or merging.
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
`Set-Acl` is used on the profile.

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

At this revision 18 Linux portable contract groups pass, including all 639
non-exempt bit changes in a bounded descriptor fixture. Native tests add actual
SDDL equivalence, negative owner/group/protection/ordered-ACE/SACL cases, and
the real atomic-file-replacement permission check. The private
inspected pair passed its exact byte/hash checks and strict parsing (783 keys).
Native PowerShell 5.1 and actual boat execution remain separate gates. No real
profile was modified by these development checks.
