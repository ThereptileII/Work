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
generation, `app` directory and launcher with read access and delete sharing,
allowing the broker's DELETE owner to rename the generation without releasing
the launcher's original custody. Ordinary enrollment pin policy is unchanged.

Only `BindDecommission` with the authenticated same-broker uninstall witness can
finalize that anchor at `retired-<requestId>-<generation>/app/skager-start.exe`.
It checks the same retained handles, physical IDs, descriptors and launcher bytes;
path opens only corroborate their fixed aliases. Only the renamed generation
root may rebind creation time for NTFS tunneling. There is no launcher reopen,
alternative destination, adjacent-record adoption or binding retry. Preparation
or binding failure prevents shortcut and SCM effects. After successful binding,
the original shortcut restoration below still precedes service removal.

The existing focused native workflow selects all four new composed
retirement/restoration controls. Qualification must authenticate their actual
PASS/no-skip events from the original artifact. The explicit required-name
addition is retained in local `db355047` but was not published because the
current GitHub OAuth credential lacks workflow-edit scope; no existing gate was
removed or relaxed. Native execution of this joined correction and full installed
uninstall acceptance remain pending; Win32 cross-compilation and inert native
fixture source do not supply that proof. Updated/pending lineage
and authenticated terminal cleanup remain unsupported by this baseline join.

For a bound publication, `Decommission` retires the three fixed published links
to `retired-Skager.lnk`, `retired-Skager Legacy.lnk` and
`retired-Skager Safe Mode.lnk` in the original recovery directory, then restores
the optional original normal shortcut to `Skager.lnk`. Each source has retained
DELETE/read custody excluding other rename/delete handles. Non-replacing native
handle renames target the original retained directory; no source path can select
a replacement file. All destinations pass preflight, complete native custody and
the SAME live broker witness are rechecked around every move, and creation time
is rebound only after our own successful rename to account for NTFS tunneling.
No original normal backup means all three protected links are retired without
creating a replacement normal shortcut. Unrelated menu entries remain untouched.

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

The witness binds the authenticated broker's terminal fence and earlier actual
application-exit/profile-cold observation. The broker releases its profile freeze
before returning the witness, so this shortcut-only transaction does not claim
continuous current-profile custody and never authorizes a profile mutation. Full
installed data-preservation acceptance, protected application cleanup, repair,
clean uninstall, native Windows execution and release qualification remain
separate gates. Temp-file tests do not qualify an installed menu or application.
