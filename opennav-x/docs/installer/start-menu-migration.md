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

The target generation's immutable ownership record selects the group. New
generations record `shellLayout: OpenNavX.NeutralStartMenu.1` and use
`OpenNav X`. An absent marker selects the historical `OpenNav X Alpha 1` group;
an unknown, null or incorrectly typed marker refuses publication. Version alone
does not select a layout: exact early Beta 2 `8e780edc` is already version
`0.4.0-beta2`, but its original maintainer only knows the historical folder.
Rollback restores the group understood by that retained engine, including early
Beta 2 and Beta 1. Old executable, lifecycle and ownership bytes are not patched.
Updating forward writes a new marked generation and migrates the group again.
Marker validation occurs when the generation is read, so an unknown rollback
target refuses before the state or transaction journal is published. The
disposable lifecycle gate corrupts only its fixture's previous marker, verifies
that exact pre-commit refusal, then restores the saved fixture bytes.

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
The marker regression adds historical 0.4 publication, rollback, interrupted
rollback recovery, exact retained-generation bytes, cleanup and invalid-marker
refusal. Ten selection/refusal checks also run independently with `-PolicyOnly`
without filesystem, registry or COM changes.

The full native installer lifecycle gate additionally installs the genuine
accepted Beta 1 artifact, updates to Beta 2, rolls back to the unchanged Beta 1
generation, executes its original maintenance Diagnostics, and migrates forward
through an interrupted group publication and repair. It captures the actual
owned Beta 2 maintenance window and checks its neutral title, Repair default
and non-mutating Cancel. Existing stock/profile hash comparisons remain.
It additionally downloads the exact hash-pinned early Beta 2 `8e780edc` installer,
installs it, updates to the candidate, rolls back and invokes the unchanged old
Maintain executable for Diagnostics and Uninstall. Both groups must disappear,
and stock/profile bytes must remain unchanged. This is a regression fixture,
not an accepted release designation for the earlier candidate.

The exact migration source from `77551bc` passed **72 native COM checks in
64-bit PowerShell 5.1 and 72 in 32-bit/SysWOW64 PowerShell 5.1** on tooling
commit `db7a2fca319d827d9490a085344ec718d0405dbd`,
[run 36289266044](https://github.com/ThereptileII/Work/actions/runs/36289266044).
Both artifact digests match the API, upload logs and downloaded ZIPs; CRCs pass.
Private receipts are under `evidence/local/boat-beta2/caption-focus-db7a2f/`.

The explicit-marker fix followed source inspection of the verified actual
`8e780edc` source archive: its lifecycle script hard-codes the historical group.
The initial migration's version-based selection would therefore have produced
the wrong group after rollback to that build. The ten corrected policy checks
pass locally; new native COM and genuine early-Beta-2 lifecycle cases remain
required. Earlier 72-check COM results above do not qualify the marker fix.

Actual NSIS wizard and genuine Beta 1/early Beta 2 lifecycle qualification of the
new migration still must pass in a later product candidate. The completed
`8e780edc` installer gate passed its own 39 checks, but contains no neutral-group
migration. No boat migration or finished Beta 2 acceptance is claimed here.

NSIS's [UninstallCaption](https://nsis.sourceforge.io/Reference/UninstallCaption)
sets the maintenance executable's title; [Modern UI page settings](https://nsis.sourceforge.io/Docs/Modern%20UI%202/Readme.html)
set its progress and completion wording.
