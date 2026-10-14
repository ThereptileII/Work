# Original protected shortcut publication binding

SCRUM-27's original startup publisher now writes
`shortcut-publication-complete.json` into its fresh recovery directory only
after the normal, Legacy and Safe shortcuts have each completed a non-replacing
native rename. The original publisher keeps its menu, recovery, launcher,
published shortcuts, optional pre-existing normal shortcut backup and
pre-move `publication.json` under live native handles while it writes and checks
the completion record. The record binds the approved client request, original
generation, commit and package, launcher identity, final native menu and
recovery directory identities, all three fixed final link identities, and the
optional original normal backup. Each file identity includes its post-move
creation time, descriptor digest, length and content digest. The pre-move
manifest digest is included as a reference to recovery evidence, not as an
independent authority. The post-move creation times matter because NTFS may
tunnel a destination's former creation time into a moved file.

The completion file is created once, synced, read back through its original
native identity and held with the publication owner. Existing completion
files are never adopted or replaced. If the three moves succeed but any
completion step fails, startup remains uncertain; the moved links, original
backup and pre-move evidence remain available for explicit recovery. No
completion is inferred from the pre-move manifest or from file names alone.

`ParseApprovedStartupPublication` accepts only bounded canonical original
completion bytes matching a separately approved SHA-256 pin and the separately
approved original client request. Its result owns a private byte copy. An
adjacent hash, copied receipt, present file or administrator-supplied path is
not independent approval. Parsing establishes historical identity only. It
does not inspect current files, prove cold application/profile state, bind a
live broker witness, authorize restoration or grant any filesystem mutation.

For explicit service-only decommission, `MaintenanceOwner.BindStartupRestoration`
joins this independent original publication approval to an already-bound original
`DecommissionWitness` and the same independently approved original client request.
Binding opens the actual
current user's native KnownFolders and protected original launcher; every menu,
recovery, launcher, fixed link and optional original backup must match the approved
volume/file ID, creation time, descriptor digest, length and content hash. The
retained completion file must match the independently approved original bytes;
the pre-move manifest must retain its recorded digest. Missing, foreign, changed,
partial or colliding fixed objects refuse. Binding itself performs no move.

The baseline-only `uninstall-protected` action instead calls
`MaintenanceOwner.PrepareStartupRestoration` **before** its single broker
retirement request. Opening the original launcher afterward would fail because
its generation has moved. Preparation retains the original publication, menu,
recovery, links and launcher without mutation. A private anchor holds the original
generation, `app` directory and launcher with read access and delete sharing.
Native run [38465784927](https://github.com/ThereptileII/Work/actions/runs/38465784927)
proved that this is insufficient for directory rename: open descendant handles
cause `STATUS_ACCESS_DENIED`. That failed product evidence remains authoritative;
the earlier statement that delete sharing alone permits the move was incorrect.
Ordinary enrollment pin policy is unchanged.

The replacement source flow uses one authenticated two-phase maintenance pipe.
The broker acquires original DELETE owners and publishes the terminal fence and
durable started record before sending a borrowed `uninstall-fenced` witness.
This intermediate capability permits only original shortcut restoration and
drainage of the original `app`/launcher handles. The operator restores the links
while their original image is still open, validates the result and closes those
two descendant handles. It retains the original generation root, protected
parents, menu, recovery, link and service/process owners. The intermediate witness
expires when the callback returns and cannot authorize service removal.

The same broker transaction, locks, frozen profile and original DELETE owners
span the handoff. Only after successful same-conversation acknowledgement does
it rename the registration pair and generation and return `uninstall-prepared`.
`BindDecommission` requires the same binding and fence and the completed
restoration/drain. It rebinds the retained generation **root** to the fixed
`retired-<requestId>-<generation>` name. It does not reopen descendants or adopt a
replacement launcher. Protected ACLs, original parent/root ownership and the
broker transaction protect the drained interval; a bare file ID is insufficient.
Only the renamed root may rebind creation time for NTFS tunneling. Partial
restoration, failed close, lost continuation or mismatched final receipt leaves
terminal recovery state and forbids SCM mutation, retry or service-only fallback.

The existing focused native workflow selects all four new composed
retirement/restoration controls. Qualification must authenticate their actual
PASS/no-skip events from the original artifact. The explicit required-name
addition is retained in local `db355047` but was not published because the
current GitHub OAuth credential lacks workflow-edit scope; no existing gate was
removed or relaxed. The original joined run failed as recorded above. The corrected source
`d7be409d` passes native run
[38512820510](https://github.com/ThereptileII/Work/actions/runs/38512820510),
including all eight explicitly checked new native controls without skips.
[Authenticated component evidence](../evidence/scrum27-two-phase-uninstall-20261014/README.md)
records the original artifacts and limits. Full installed uninstall acceptance
remains pending; this component result does not supply that proof. Updated/pending lineage
and authenticated terminal cleanup remain unsupported by this baseline join.

For a bound service-only publication, `Decommission` retires the three fixed published links
to `retired-Skager.lnk`, `retired-Skager Legacy.lnk` and
`retired-Skager Safe Mode.lnk` in the original recovery directory, then restores
the optional original normal shortcut to `Skager.lnk`. Each source has retained
DELETE/read custody excluding other rename/delete handles. Non-replacing native
handle renames target the original retained directory; no source path can select
a replacement file. All destinations pass preflight, complete native custody and
the SAME live broker witness are rechecked around every move, and creation time
is rebound only after our own successful rename to account for NTFS tunneling.
No original normal backup means all three protected links are retired without
creating a replacement normal shortcut. Protected uninstall performs these same
moves during its intermediate callback, before generation retirement;
`Decommission` must not perform them again. Unrelated menu entries remain untouched.

The move uses `NtSetInformationFile(FileRenameInformation)` with the retained
destination directory and a fixed relative filename, following Microsoft's
[native rename contract](https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/ntifs/ns-ntifs-_file_rename_information).
The earlier Win32-wrapper form returned `ERROR_INVALID_PARAMETER` on the real
Win32 Windows Server 2022 gate [38437716546](https://github.com/ThereptileII/Work/actions/runs/38437716546).
That failed evidence remains preserved; replacing the call does not relax
non-replacement, original-handle custody or partial-failure checks.

Restoration completes before service disable, STOP and deletion marking, while
the original broker witness is still live. The restored/retired objects remain
under native custody and are rechecked before service deletion marking. Failed
binding revokes this owner, so it cannot fall back to service-only removal;
failed/partial moves permanently consume the attempt and preserve all recovery
objects. Close only drains handles, including after uncertainty. No file is
deleted, descriptor repaired, existing retirement target adopted or move retried.
Successful service results remain `marked-for-removal`, and owner-close errors
suppress success. The earlier explicit service-only API remains available when
no publication was bound; it is not full uninstall.

The final witness binds the authenticated broker's terminal fence and actual
application-exit/profile-cold observation. The protected two-phase transaction
retains the broker's profile freeze across shortcut restoration and generation
retirement. The broker releases it before returning the final witness. The
service-only shortcut transaction still occurs after that release; neither path
authorizes a profile mutation. Full
installed data-preservation acceptance, protected application cleanup, repair,
clean uninstall, native Windows execution and release qualification remain
separate gates. Temp-file tests do not qualify an installed menu or application.
