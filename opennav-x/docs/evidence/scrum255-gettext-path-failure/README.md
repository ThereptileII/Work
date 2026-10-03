# SCRUM-255: preserve first native contract failure

[Run 37086572916 / job 111098021434](https://github.com/ThereptileII/Work/actions/runs/37086572916/job/111098021434)
failed on remote `abd4905b45b126e524e7989ce7f788ed1c821529`, before provider
acquisition. Artifact **11261350237** was downloaded independently: **2,407
bytes**, SHA-256
`7340c465ee0ddd43a8dcf86a7254185d173ad69e37c7ae5c6c3b5ec2a687cd4f`.
All four ZIP entries passed CRC verification and are retained here unchanged.

The 17 contracts reported three failures and six errors. The known pair fixtures
were rejected before their mocked probes, and retry fixtures reached
`chocolatey()` but failed its resolve-versus-absolute comparison. The fixtures
create those regular files in setUp. Thus rejecting legitimate canonical path
spelling is confirmed; the artifact did not retain raw/resolved TEMP paths, so
**Windows 8.3 expansion is a hypothesis until the next actual path diagnostics**.
No provider package or actual installed Gettext tool was exercised by this run.

The narrow correction inspects `lstat()` on the tool and every ancestor,
rejecting symbolic links and Windows `FILE_ATTRIBUTE_REPARSE_POINT` (including
junctions), and still requires an absolute regular file. Known Poedit and
Chocolatey roots, tool probes, hashes, no-PATH policy and all retry/failure guards
remain. A short filename alias is another spelling of the same file, not a link.
The x86 fixture compares the returned canonical directory to its canonical
expectation rather than another lexical spelling.

The same short proof job now retains raw/resolved TEMP diagnostics and exercises
`GetShortPathNameW`/same-file identity on an actual Windows alias, requiring a
real spelling difference. It also creates a disposable native directory
junction and requires its ancestor traversal to be rejected. There is no
acceptance shortcut if the runner cannot demonstrate the alias boundary.
Four portable checks cover canonical-spelling independence and rejected
leaf/ancestor reparse/symlink metadata.

Local result: **23 tests, 21 passed and two Windows-only cases skipped**. The
original 17 contracts remain. No native pass, provider acquisition, application
build or full candidate result is claimed at this correction. Root will publish
only this delta to the same focused native proof workflow.
