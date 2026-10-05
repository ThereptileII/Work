# Bootstrap-only installed launcher check

`tools/boat/run-xnav.ps1 -Workspace C:\XNav -UseStartupLauncher` is an explicit
opt-in to the installed `skager-start.exe --xnav` path. Without this switch,
existing boat launch behavior is unchanged. Use this command only after the
existing profile, plugin and read-only commissioning preparation and audit have
passed for the exact installed generation and interactive account. It leaves the
application open and performs no screenshot, endurance exercise or controls.

The helper requires startup-health version 1, no pending update and no
`app/update-trust.json`. It refuses even malformed pending/trust records. It does
not provision a root, select a feed, accept an offer or recover an update. Signed
update offers and rollback on the boat remain pending separate qualification.

The complete existing `Assert-ReadOnlyAudit` runs again in the interactive task.
The launch uses its audited working directory and PATH. The helper verifies and
holds the selected state, ownership, target, launcher, application and supervisor
files while launching. It retains process handles and binds the actual app to
the launcher's OS parent chain, creation time, account SID, session, image path,
SHA-256 and exact arguments. First startup must include the observed system
PowerShell `QualifyCurrent` child; a missed or ambiguous ancestor is a refusal.
An existing valid known-good receipt permits the direct launcher child path.
Window titles or PID numbers alone are not acceptance evidence.

Success requires launcher exit zero, the installed generation's authenticated
DPAPI startup receipt, the exact live application's responsive window and a fresh
initialization log marker. The receipt comes from the installed supervisor's
30-second continuous health check. The [human-wait protocol](startup-human-wait.md)
allows one authenticated navigation-warning decision interval without accepting
the warning automatically. The helper allows 690 seconds overall; dispatch allows
720 seconds, and its interactive task has a 15-minute bound. Other interactive
tasks retain their existing bounds. Failure does not
retry, kill or close the application. Inspect the interactive desktop and follow
the existing normal-close and commissioning restoration procedure before any
further launch or maintenance.

This startup receipt does not authorize navigation, mode changes, autopilot or
other post-launch controls. Their existing interactive audit and review contracts
still apply. `smoke-test.ps1 -UseStartupLauncher` also selects this launch path,
but retains that command's existing screenshot, observation and normal-close
behavior; use `run-xnav.ps1` for a startup-only handoff.

The focused disposable check is `tools/boat/test-startup-launcher.ps1` on native
Windows CI, or with explicit `-IsolatedLocal` on an otherwise closed local test
machine. It launches only an inert PowerShell child, never the installed app or
boat profile. Linux uses `-PortableContracts` and cannot qualify Windows process
observation. `test-boat-tools.ps1` runs and validates this check and embeds its
single JSON result as `startupLauncher`; existing maintenance checks remain
unchanged. The focused report includes its two exact helper source hashes and
keeps `installedBootstrap` and `signedOffersAndRollback` explicitly `pending`.
A helper test pass is not real boat startup acceptance.
