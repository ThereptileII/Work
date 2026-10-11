# SCRUM-311 directory custody — native Windows, 2026-10-12

This is focused native component evidence, not an application release, MSVC
build, full authenticated service qualification or accepted update transaction.
The existing boat application, profile, connections and physical controls were
not changed. Tests operated on disposable inert files and empty directories.

## Observed failure and correction

The actual Windows/386 broker-store test at `d579c27b` renamed a directory while
its metadata-only handle was held. Omitting `FILE_SHARE_DELETE` did not establish
the claimed rename exclusion when the handle requested only metadata access.
The assertion was retained. Production directory owners now request
`FILE_LIST_DIRECTORY` in addition to their existing metadata rights, preserving
their sharing, reparse, identity, ACL and same-volume checks. Point-in-time
metadata corroboration is unchanged where another retained data-read owner
already supplies sharing exclusion.

The fix applies to the protected store (`aa9fe37c`), pipe image ancestry
(`31175848`), stock baseline ancestry (`11486212`), and ordinary/armed Go and C#
launcher ancestry (`cd85b15e`). Empty-directory fixtures avoid accidentally
crediting a held descendant executable with the directory owner's protection.
Tests require rename refusal while held and successful rename after release;
the strict fixtures require Windows sharing error 32. The store also refuses
acquisition when a conflicting delete-capable handle already exists.

Microsoft documents the distinction between metadata-only and data access in
the [sharing algorithm](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-fsa/8c0e3f4f-0729-49f4-a14d-7f7add593819)
and [share-access check](https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/wdm/nf-wdm-iochecklinkshareaccess).
This correction does not freeze DOS aliases, establish originally protected
provisioning, exclude earlier writable handles or prove DLL/loader custody.

## Exact-source results

| Check | Result and source |
| --- | --- |
| Broker store, pipe and stock directory checks | Native Windows/386 PASS: 11 selected top-level groups and 26 subcases at `11486212968d3f8e9e8277270c6def2c52d1c62b`. Three package executables ran on the boat PC; binary/source hashes are in the receipt. |
| Go armed-directory and marker checks | Native Windows/386 PASS: 3 selected groups and 4 subcases at `cd85b15e10aff7db1086a5e33a474cfc7f408da6`. |
| Installed launch-barrier fixture | Native Windows PowerShell 5.1 x64 and x86 PASS: 60 checks each against the `cd85b15e` production module. Inert executable filenames only; `applicationLaunched=false`. |
| Actual C# armed-barrier/image-owner directory opens | Native Windows PowerShell 5.1 x64 and x86 PASS: 12 checks each against the unchanged `cd85b15e` production module, using the standalone helper corrected in `054f9a79`. Includes exact sharing-error 32 and successful rename after release for both empty-directory owners. |

The standalone test's initial reflection call passed a PowerShell-wrapped path
to a .NET string argument and failed before the lock check. `054f9a79` makes the
path, handle and boolean argument elements explicitly typed, in both the
standalone helper and its equivalent disposable-CI owner fixture. Only the failed
helper was rerun; the passed Go and launch-barrier results were retained. The
broader CI-only owner fixture was not invoked on the boat or stripped of its
no-application guard. The dedicated helper creates no child process.

Raw logs, local usernames, desktop metadata and temporary paths remain private.
The curated receipt records test names/counts, source/binary hashes and original
log hashes. Intermediate failures remain retained locally: a remote wrapper's
argument/execution-policy errors, the actual pre-fix directory failure, and the
standalone reflection fixture's PowerShell argument-conversion error. None is
relabelled as a pass. The helper error occurred before its native lock check.

## Limits

The Go executables were cross-compiled on Linux and then **executed natively**
on Windows. This is distinct from native MSVC application compilation.
Originally protected service provisioning/trust, cross-account IPC rights,
installation/loader custody, crash reconciliation and exact-package Windows/boat
update acceptance remain unfinished. Existing source-only staging remains
disabled. No application build, CI dispatch, deployment, visual/endurance sweep,
physical command or publication occurred. SCRUM-311 remains In Progress.
