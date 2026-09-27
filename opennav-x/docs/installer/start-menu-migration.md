# Version-neutral Start-menu group

Beta 2 publishes `OpenNav X`, `OpenCPN Legacy`, `OpenNav Safe Mode` and
`Maintain OpenNav` in the **OpenNav X** Start-menu group. Optional Legacy/Safe
choices remain persistent. Setup's Finish launcher uses that group. The
maintenance window, progress and finish pages describe maintenance rather than
calling Repair an uninstall.

The internal `%LOCALAPPDATA%/OpenNavXAlpha1` root, HKCU uninstall key,
`OpenNavX.Alpha1.SideBySide.1` owner, state schema and immutable generations do
not change. Neither stock OpenCPN nor the navigation profile is modified by
this migration.

The target generation selects the group. Versions 0.2 and 0.3 use their original
`OpenNav X Alpha 1` group; Beta 2 and subsequent generations use `OpenNav X`.
Rollback therefore restores the group understood by the original retained
Beta 1 maintainer. The old executable and lifecycle script are not patched.
Updating forward again migrates the group again.

Before staging, publication, recovery or removal, both fixed groups must pass
ownership inspection. Every entry must be one of the four exact shortcut names,
be a real non-redirected file, and target a uniquely recorded owned generation
file with exactly the expected arguments and working directory. A matching
filename alone does not establish ownership. Foreign entries, modified
invocations, unknown generations, duplicate target records and redirected paths
refuse the action while preserving existing entries. Missing or damaged managed
executables remain repairable: shortcut ownership comes from their immutable
record, not a successful hash of the damaged resource.

The atomic state remains the transaction commit point. Publication creates and
verifies the complete target group before removing each verified old shortcut;
only an empty old group is deleted. An interruption at `after-shortcuts` leaves
both groups usable and the journal retained. Recovery reads the committed state
and finishes publication in the appropriate direction. Unknown entries found
after an interruption are preserved and must be inspected; recovery does not
claim or sweep them.

## Checks

`tools/test-installer-shortcuts.ps1` imports only actual lifecycle function AST
nodes. It uses real Windows WScript.Shell links, inert target files, a unique
temporary Programs tree and an isolated HKCU test key. The existing native
filesystem suite invokes it under both 64-bit and 32-bit PowerShell 5.1. Cases
cover clean install/removal, optional shortcuts, update, rollback and recovery
in both directions, foreign targets/arguments/working directories, unknown or
ambiguous ownership, unrelated entries, reparse refusal, and repair with a
missing owned executable. No installed application is launched by these tests.

The full native installer lifecycle gate additionally installs the genuine
accepted Beta 1 artifact, updates to Beta 2, rolls back to the unchanged Beta 1
generation, executes its original maintenance Diagnostics, and migrates forward
through an interrupted group publication and repair. It captures the actual
owned Beta 2 maintenance window and checks its neutral title, Repair default
and non-mutating Cancel. Existing stock/profile hash comparisons remain.

Local parser and source checks are preliminary. Native COM, NSIS wizard and
same-commit lifecycle qualification must pass in a later candidate; the running
8e780edc candidate does not contain this migration. No boat migration or finished
Beta 2 acceptance is claimed by this document.

NSIS's [UninstallCaption](https://nsis.sourceforge.io/Reference/UninstallCaption)
sets the maintenance executable's title; [Modern UI page settings](https://nsis.sourceforge.io/Docs/Modern%20UI%202/Readme.html)
set its progress and completion wording.
