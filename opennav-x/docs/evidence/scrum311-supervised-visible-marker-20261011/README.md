# SCRUM-311 normal-close marker fixture correction

The retained [native run 38167390164](https://github.com/ThereptileII/Work/actions/runs/38167390164)
at exact candidate `6445d67e27c6b6f4254849556efea28248204711` remains **FAILED**.
The x64 supervisor artifact `11712727661` has SHA-256
`95b796a8bc582f31cba18633b4953e68b61f309c4b0599d503a1c0faff3b735e`,
matching the retained artifact API digest; its one member passed CRC.
`raw/owner.log` retains that member's **8,276 original UTF-16 bytes** unchanged.
The [receipt](results.json) records its length/hash, failed-candidate/current
source hashes and focused checks; [owner-decoded.log](owner-decoded.log) is a
trimmed UTF-8 reading copy with LF line endings.

The log has **57 PASS lines**, including the physical held-image application
selection, selected-name refusals, marker-A execution after SUBST remap, and live
`Matches` check. It then fails at the **first** `UpdateImageOwner.Close` with
`Owned child remains live; no forced termination or rollback.` The supervisor
health fixture comes later in the exact script and was not reached. No earlier
health exception being masked is evidenced by this record.

The marker used `ShowInTaskbar=false, Opacity=0`. Microsoft's .NET Framework
[MainWindowFinder source](https://github.com/microsoft/referencesource/blob/main/System/services/monitoring/system/diagnosticts/ProcessManager.cs)
excludes windows with an owner or without the visible style. The indexed
[Framework Form source](https://referencesource.microsoft.com/System.Windows.Forms/winforms/Managed/System/WinForms/Form.cs.html)
sets a taskbar owner on a top-level form when ShowInTaskbar is false and no
explicit owner exists. Microsoft also documents that
[MainWindowHandle is zero for hidden/non-taskbar processes](https://learn.microsoft.com/en-us/dotnet/api/system.diagnostics.process.mainwindowhandle).
Together these support a fixture-window discovery failure, rather than a defect
in retained image identity. The failing run did not record its actual HWND or
CloseMainWindow return, so native confirmation of this diagnosis remains pending.

The correction changes only the inert test form to `ShowInTaskbar=true,
Opacity=1`, preserving its tiny size, unowned top-level lifetime and normal
Application.Run message loop. After marker readiness the fixture refreshes its
actual Process object and requires a nonzero MainWindowHandle. It then retains
the original live-owner Matches assertion, exact `owner.Close($child,10000)`
call and child-exit assertion. No auto-close timer, alternate window signal,
production forced termination, retry or cleanup exception waiver was introduced.
The production UpdateTransaction.ps1 bytes are unchanged from candidate `6445d67e`.

Local Linux verification passed the existing **37 PolicyOnly checks**, including
exact embedded production C# compilation and direct parser/record refusals.
The exact edited marker C# syntax also parsed successfully through Roslyn;
this syntax check does not compile against Windows Forms or execute the marker.
Neither local check invoked a native API. Full native creation, normal-close,
health, uncertain-resume and interrupted-cleanup fixtures remain pending the
coordinator's next Windows run. This increment performs no native execution,
CI dispatch, application build, boat operation or external specialist review.
