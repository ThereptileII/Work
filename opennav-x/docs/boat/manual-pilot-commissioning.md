# SCRUM-313: explicit manual pilot child transaction

`tools/boat/commission-manual-pilot.ps1` is separate from read-only commissioning.
It never sends a pilot command, restores a plugin DLL, launches stock/Legacy/Safe,
changes a navigation source, or upgrades a package. The existing input-only launch
policy remains strict. Native qualification of these helpers precedes boat use.
The root coordinator alone handles the authorized physical six-command procedure.

Prerequisites are a qualified installed **manual-commissioning / contract1**
candidate, complete fresh parent read-only audit and applied plugin quarantine,
closed application/helper processes, and the existing enabled serial NMEA2000
COM8 connection. Other outputs, active/stored routes, custom plugin roots,
pending updates, unknown hashes and saved manual permission refuse preparation.
The normal existing `OpenNav/AlphaSettings` record must already exist. New/default
vessel settings must be established and independently preserved before this child.

The caller supplies exact commit, installed generation ID, executable hash,
ownership hash and package hash. Qualification is an independently reviewed JSON
file, supplied with its SHA256, containing owner
`OpenNavX.ManualPilotQualification.1`, matching `commit`, `executableSha256` and
`packageSha256`, fresh `reviewedUtc`, actual booleans `nativeSerialGatePassed`,
`defaultOffDiagnosticsPassed`, `retainedPluginsReviewedForBidirectional` all true,
and `shutdown` in the existing schema2 retained-plugin shutdown-review format.
These fields are an operator attestation bound to evidence, not measurements made
by this helper. Never manufacture them from successful queueing or a build log.
The shutdown review covers every exact retained plugin path/hash/source revision;
the bidirectional review must cover startup, idle and shutdown with COM8 writable.
AutoTrack stays outside all loader/search paths in its parent-owned quarantine.

Use the entry point's explicit actions in order:

1. **Prepare**: supply `Workspace`, `ExpectedCommit`, `ExpectedGeneration`,
   `ExpectedExecutableSha256`, `ExpectedOwnershipSha256`,
   `ExpectedPackageSha256`, `Qualification`, `ExpectedQualificationSha256`.
   The returned private `record` and `recordSha256` identify this child. It now
   owns `manual-pilot-active.json`, including before any connection change.
   Parent actions and input-only launch refuse while that marker exists.
2. **Apply**: supply `Workspace`, `Record`, `ExpectedRecordSha256`. A durable intent
   precedes exactly one COM8 IOSelect byte0→1, through the existing inverse-byte
   helper and native atomic publication/ACL checks. Nothing else is published.
   Repeat Apply is forbidden, including after an interrupted attempt.
3. **Launch**: same record arguments. It validates all owned runtime files against
   the pinned ownership inventory, parent proof/quarantine, complete generation
   and helper trees, profile bytes, SID/session and signed system PowerShell.
   Both dispatch and the actual limited interactive task repeat checks. The
   registered task definition is checked. Only `--xnav` is launched, once. An
   intent precedes start and a durable PID/start-time receipt precedes polling.
   The helper waits for fresh exact-build diagnostics showing both session flags,
   configured permission, simulation, replay, TRACK/WIND and route creation OFF.
   Timeout/failure retains ownership; it does not retry or kill the process.
4. In the product, inspect fresh identity and feedback. Configure exact `COM8`
   and an observed compatible NAME. Saving a new NAME requires another observed
   address claim; the refresh button sends one60928 request, rate-limited to5s.
   Save manual permission, then separately enable this session. The six types are
   STANDBY, AUTO, -1/+1/-10/+10. Wait for actual physical feedback and independently
   observe each response. Finish in physical STANDBY, disable the session and use
   **Return to display-only** before closing. TRACK/WIND remain unsupported.
5. **Close**: same record arguments. Normal close targets only the recorded
   executable/PID/start-time/SID/session, retains the process handle and records
   the measured exit code. Saved manual permission refuses Close. There is no
   force kill, repeated close attempt or inferred success. Manual closure/crash
   can instead proceed to Inspect once all relevant processes are absent.
6. **Inspect**: copies the actual closed profile, its complete tree inventory and
   key differences to new private evidence. Review all returned differences.
7. **Rollback**: supply the same record plus `Inspection`,
   `ExpectedInspectionSha256`, `ReviewedCurrentIniSha256`. It preserves every
   reviewed current byte except COM8's direction1→0. Pilot identity preservation
   is limited to the three exact pilot scalar fields, supported COM8/NAME, and
   display-only permission; all other AlphaSettings bytes must match. Ordinary
   current settings use the unchanged `Assert-SessionPreservationReview` rules
   on private validation copies with that pilot delta removed. Unknown changes
   refuse; no broad AlphaSettings admission or old-profile restoration occurs.
   Parent quarantine/ownership remains. Subsequent launch needs a fresh audit.

Recovery remains evidence-bound. Prepared or failed-before-publication children
can be inspected/rolled back without a launch. A rollback interrupted after its
intent or atomic publication resumes using the **same** inspection/hash arguments;
only the journaled before/after profile hashes are accepted. A completion record
written before marker deletion is also recoverable. Unknown partial files, changed
runtime/quarantine/profile trees and saved manual permission block mutation and
remain available for explicit review. Do not remove ownership markers manually.

The tooling does not authorize a second application launch, an automatic steering
retry, replay-derived feedback, or guessed pilot identity. The product still
requires current identity/epoch, permission, session, fresh physical mode/heading,
and feedback after that exact command's full serial write. Byte writes alone do
not establish pilot acknowledgement or real-world command success.

Focused inert gate:

- Native Windows CI: `powershell.exe -NoProfile -File tools/boat/test-manual-pilot-commissioning.ps1`
  with `GITHUB_ACTIONS=true`; local disposable Windows requires `-IsolatedLocal`.
- Linux PowerShell7: the same test with `-PortableContracts`.

Fixtures execute actual transaction/profile-publication and rollback code with
mocked OS identity, signing and process discovery. Windows retains real native
file metadata/ACL publication checks. No fixture starts a task/application,
executes a plugin, opens a COM port or sends a command. Actual task/scheduler,
product startup and boat feedback qualification remain separate gates.
