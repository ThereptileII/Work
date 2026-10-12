# Protected updater startup integration and native repairs — SCRUM-311

This is component evidence, not an accepted installed updater or Staging release.
The exact failed native run is retained in [failed-native-run.json](failed-native-run.json).
Jira remains the backlog; SCRUM-311 and the packaged security gates remain open.

Latest component run [38281497298](https://github.com/ThereptileII/Work/actions/runs/38281497298)
on exact `ea3c2bffa493915bf48b68834d6c1cc508ffb44c` passed Linux and native
startup interactions. Windows passed 703 top-level plus 2756 nested tests, failed
two top-level tests and skipped twelve. Fresh SCM enrollment was skipped, not
accepted. The downloaded original artifact and failures are recorded separately
in [manual-maintenance-native-run.json](manual-maintenance-native-run.json).
Historical installer transactions passed 47 inert checks; these do not establish
protected installation acceptance. `c8a4ab50` repairs permanent capability
revocation ordering and strengthens the actual inaccessible-directory fixture
without changing primary-token privileges. Native confirmation remains pending.

## Integrated source

- `f97c747a` adds authenticated selected-executor update offers and the protected
  prompt/Now/Later path. The broker rechecks exact consent when staging; prompt
  bytes and the actual native child identity are bound before resume.
- `56d0b477` adds explicit initial startup activation and normal-shortcut
  publication. Registration alone still does not select an application.
- `f8585655` joins actual service dispatch, startup relay and the explicit
  provisioner activation option. Existing selected-launcher relay remains a
  terminal handoff, not proof of successful ordinary application startup.
- `c2f55a19` suppresses qualification-only configuration, plugin/core log and
  startup-recovery bookkeeping writes, preserving the normal main-thread health
  checkpoint and ordinary OpenCPN persistence outside qualification.
- `daf9c3a4` refuses failed/crashed update prompts. Only explicit Later (0) and
  Now (10) exits are accepted; an unknown exit cannot silently become Later.
- `7e3c5e09` requires the existing native private-chart plugin-loader job before
  a Staging release can be recorded. Its focused delivery checks passed 25 cases.
- `c8b5a89b` selects the fixed system `windows.storage.dll` ShellLink factory,
  corroborates the original retained file identity/destination/security/bytes
  after its own non-replacing rename, and fixes native owner cleanup ordering.
  Generic pin validation stays strict; substitution regressions were added.
- `b92eaaa5` passes the required installation root in the actual native installer
  qualifier fixture. Its new regression fails with the original omitted argument
  and passes with the corrected caller; the focused portable armed-barrier suite
  passed 62 checks. Production C# was not changed.

Joined focused Linux race results before the native run were 71 top-level plus
304 nested cases for host/pipe/launcher and 44 top-level plus 134 nested cases
for enrollment/provisioner. These suites overlap other evidence and must not be
summed into a whole-product test count. Actual Windows/x86 component binaries
cross-compiled; cross-compilation is not native acceptance.

## Actual native results and diagnosis

[Run 38261504482](https://github.com/ThereptileII/Work/actions/runs/38261504482)
used exact source `daf9c3a4bca883f91a818b5eedf9c7c8ba7b6074`:

- Linux verification passed.
- Native startup update interaction harness passed. This harness does not
  establish protected SCM enrollment or installed broker execution acceptance.
- Windows Go component verification failed: 624 top-level and 2535 nested passed,
  8 top-level and 12 nested failed, 9 top-level skipped. Failure cases and retained
  original artifact sizes/hashes are recorded in the linked JSON.
- Profile capture refused a real directory MODIFIED notification for the inert
  fixture's `navigation` directory during initial capture. No notification was
  ignored. `3abc9bd4` moves watch arming after the candidate snapshot and requires
  a complete identity/metadata/security/namespace/digest validation under that
  active watch before accepting custody. Native confirmation remains pending.
- The native ShellLink factory refused the requested class; shortcut publication
  also encountered identity changes and fixture cleanup ordering failures.
  `c8b5a89b` repairs these paths, subject to a native rerun. Microsoft documents
  [the system ShellLink server](https://learn.microsoft.com/en-us/troubleshoot/windows-client/shell-experience/application-shortcuts-show-blank-icons)
  and [NTFS metadata tunneling after rename](https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/fltkernel/nf-fltkernel-fltgettunneledname).
  Tunneling explains a possible legitimate creation-time change; the exact
  failed run did not itself prove that mechanism. The repair corroborates the
  actual retained original handle and cannot adopt a same-byte replacement.
- The installer fixture's process-start failure was a missing required third
  installation-root argument in its caller. The production armed-session guard
  correctly refused the empty root. An 8.3 alias defect in that C# guard was an
  initial hypothesis, not the established cause; it does not justify changing
  the production guard.

## Replacement component run

[Run 38264329533](https://github.com/ThereptileII/Work/actions/runs/38264329533)
used exact source `307b66bc786f57584ea18cf8a6c7f7dfd240da11`. Linux and native
startup interactions passed. Native component results improved to **631 top-level
and 2548 nested passed; 2 top-level and 2 nested failed; 9 top-level skipped**.
Original artifact hashes and exact failures are in
[replacement-native-run.json](replacement-native-run.json). The prior failed
record remains unchanged.

The remaining shortcut assertion compared `RUNNER~1` with Windows' expanded
`runneradmin` target. `e87229ca` obtains the independently expected long path from
Win32 and retains exact target, arguments and working-directory assertions.
`182e5a99` repairs the security-mutation fixture: it changes a captured directory
DACL, independently observes that change and still requires capture refusal.
The captured ordinary file correctly denies write sharing; production remains
unchanged. `ca368ac4` restores actual safe top-level Lifecycle initializers in
the extracted fixture. Its portable regression reproduces the omitted
`CreatedRoot` failure and verifies the intended wrong-transaction refusal,
byte-identical selected/pending records and lock release after the repair.

The next native component run also explicitly opts into fresh fixed-name service
creation on a disposable hosted runner using the same-commit retained broker.
This tests real SCM principal/store initialization without changing the boat or
launching the navigation application. Fixture source/compilation alone is not a
native service pass. Replacement native execution of these repairs is pending.

## Maintenance component run and handoff producer

[Run 38267062370](https://github.com/ThereptileII/Work/actions/runs/38267062370)
used exact source `abce9b3233f975f34964900165caee796bf1dd13`. Linux and native
startup interactions passed. Authenticated startup/interruption records and
installer interrupted/history transactions also passed. The Windows component
suite passed 643 top-level and 2595 nested cases, with 2 top-level and 1 nested
failures, and 11 top-level skips. See [original artifact and member hashes](maintenance-native-run.json).
Actual fresh SCM enrollment was skipped after the prerequisite failure.

The actual maintenance process path comparison refused before completing image
binding. Windows can report the original short spelling while the retained file
handle reports its normalized name. `200a2832` opens the process-reported native
path and requires its canonical physical path, full file ID, creation time and
non-reparse status to match the original retained image; current process/token/
birth and live checks remain. Native confirmation remains pending.

The profile security test still failed in `SetSecurityInfo`, before production
validation. `560ec201` adds READ_CONTROL to its test-only WRITE_DAC handle and
corroborates actual protected DACL ACE bytes before the unchanged original
metadata/refusal assertions. This is a proposed fixture repair, not a proven
native pass or a change to production guards.

`ea4273d2`/`750af80b` join the initial baseline handoff producer. It preserves the
approved recovery ZIP byte-for-byte, compares the extracted tree, validates the
strict original client request and importer, bounds reads, checks cancellation,
propagates close errors and reads back output bytes. The output request digest
needs separate approval. Focused Linux producer checks and Windows/x86
cross-compilation passed; protected output-directory provenance, native producer
execution and actual enrollment/installed lifecycle remain separate gates.

## Remaining installed acceptance (unchanged)

The ordinary installed Setup still uses the older per-user lifecycle. It has not
yet joined the new protected operator enrollment, maintenance and uninstall
contract. Protected repair, manual rollback and decommission must preserve
authenticated history, trust floors, shared navigation data and other clients.
User-owned state/ownership JSON cannot authorize protected selection or deletion.
The preserved original shortcut bytes are recovery evidence, not independent
authority for a later restoration.

The combined native application/package, actual service principal/SCM path,
signed bootstrap and successor, no-update/Later/Now/offline cases, authenticated
startup and failure restoration, packaged networking/plugin/sharing gates,
installer lifecycle and exact candidate boat tests are still required. No full
application rebuild, boat/profile/service change, design review, endurance run,
physical hardware command or Production/public release occurred in this batch.
