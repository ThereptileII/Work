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

A later shortcut retirement/restoration transaction must obtain separate
authority and retain current native owners for every relevant file and
directory. It must compare the exact approved original identities with those
live owners, including all three fixed links and any original normal backup;
then it must use non-replacing operations and preserve evidence after an
uncertain partial move. The service may be removed only after that separate
shortcut transaction succeeds. This source slice does not implement that
transaction, repair, clean uninstall or installed Windows acceptance.
