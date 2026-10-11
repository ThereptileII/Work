# SCRUM-311 native previous-restoration fixture — 2026-10-11

Exact source from `73625dbcab3cd4f77f9564576a51cc97cbe127bf` was copied
through the authorized `ssh boat` alias into a fresh isolated source-test folder.
All 17 source hashes were read back and matched local committed bytes before
execution. Both fresh Windows PowerShell 5.1.22621.5624 processes succeeded:
System32/x64 and SysWOW64/x86 each passed 106 assertions (212 total).
The full source hashes and results are retained in [x64.json](x64.json) and
[x86.json](x86.json). No source was changed during execution.

Thirteen disposable NTFS transactions exercise the exact previous-restoration
wrapper and real Inventory/Prepare/Apply/InspectRestore/Restore/preservation
engine. Native owner SID/session, Store handles and physical paths, private
journal ACLs, actual metadata/SACL access, installation lock, profile mutex,
file publication and permission checks remain real. Cases include unchanged
and reviewed changed-setting restoration, plugin quarantine/return, route-file
and DACL drift, pointer/root drift, contention, closed custody, replay, failed
journal publication and failed final cleanup after a consumed event.

The fixture explicitly substitutes a bounded **synthetic context**, synthetic
closed-process observation and a test-only baseline, after proving that the
unmodified public engine rejects that baseline without changing its profile.
Marked fault seams inject publication, pointer and cleanup failures. These are
native filesystem/operation results, not actual installed-context or process-
closure qualification. The raw report's `boatAccess=false` describes absence of
real boat-system/profile discovery; the test itself ran on the boat Windows PC.

Only disposable inert files were changed. No application was launched or closed,
normal profile/chart/plugin configuration accessed, physical command sent,
installed executable changed, remote-access service modified or reboot performed.
The test removed its disposable transaction trees before emitting success. The
isolated copied source files remain for reproduction.

The existing native Staging qualifier now invokes this fixture in a separate
fresh process for each architecture, with a separate retained log. Registration
syntax passed on Linux; this boat run is not a CI result. Full installed updater,
continuous loader custody, authenticated broker, actual no-update/Later/Now/
offline flows and release qualification remain separate outstanding gates.
