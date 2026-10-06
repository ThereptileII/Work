# SCRUM-313: explicit manual pilot child transaction

`tools/boat/commission-manual-pilot.ps1` is separate from read-only commissioning.
It never sends a pilot command, restores a plugin DLL, launches stock/Legacy/Safe,
changes a navigation source, or upgrades a package. The existing input-only launch
policy remains strict. Native qualification of these helpers precedes boat use.
The root coordinator alone handles the authorized physical six-command procedure.

Prerequisites are a qualified installed **manual-commissioning / contract1**
candidate, complete fresh parent read-only audit and applied plugin quarantine,
closed application/helper processes, and the existing enabled serial NMEA2000
COM8 connection. Route persistence must be exactly OFF. An empty stored
`ActiveRoute` or a syntactically valid GUID is retained verbatim; malformed route
IDs, other outputs, custom plugin roots, pending updates, unknown hashes and saved
manual permission refuse preparation.
The normal existing `OpenNav/AlphaSettings` record must already exist. New/default
vessel settings must be established and independently preserved before this child.

The pinned `model/src/routeman.cpp` constructor initializes no active route and
only activates the saved GUID when `g_persist_active_route` is true (lines110–119).
`navutil.cpp` reads and writes the GUID independently of that flag. Keeping an
inert saved GUID therefore preserves existing user data without activating it.
The manual child and completed-child preservation proof reject any change or
clearing of that stored value; this does not relax general route/source policy.

Startup acceptance and every manual UI observation/input additionally require the
actual top-level `route` diagnostic from `OpenCPNRouteReader`/`RouteProgressInput`:
`NoActiveRoute`, empty live route/waypoint IDs, zero waypoints, positive revision,
process-local revision scope and the exact normal-progress source. Missing,
awaiting, invalid or active route data cannot substitute for inactivity.
The completed observation's age at publication plus the diagnostic file's age
must total at most five seconds. `AssessRoute` does not age non-Valid states, so
a newly written file containing an old `NoActiveRoute` remains unacceptable.
Consumer reads may invalidate progress but never renew its observation timestamp. Diagnostics bytes, timestamp and SHA-256 are read
from one held, ordinary single-link file (maximum 4 MiB). Its Win32 read-sharing
mode denies concurrent write/delete/replacement; metadata comes from the held
handle rather than another path lookup. Startup evidence records that exact
snapshot digest. A replaced diagnostic path cannot lend its date to older bytes.
No route is activated, deactivated, erased or rewritten by these guards.

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
   configured permission, simulation, replay, TRACK/WIND and route creation OFF,
   plus a fresh explicit normal-progress `NoActiveRoute` observation.
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

## One-action manual UI helper

`manual-pilot-ui.ps1` is a separate opt-in UI path under the active manual child.
It does not extend `ReviewWindowNative`'s read-only action allowlist. Stage and
qualify the complete helper tree **before Prepare**: the child pins that tree and
refuses edits after preparation. Its candidate remains the exact previously
qualified package; a helper revision does not change the package revision.

Each invocation requires `-Workspace`, `-Record`, `-ExpectedRecordSha256`, an
explicit `-Action`, and a new 32-digit lowercase hexadecimal `-Nonce`. Every
invocation revalidates the signed current-user interactive host, active parent
and child, exact runtime/quarantine/tool trees, PID/start time/session/SID/executable,
current profile, no pending update, and fresh real-product diagnostics. It holds
both transaction locks. Ordinary session changes use the existing preservation
policy; only the already reviewed pilot binding/permission delta is separately
admitted. No actual profile bytes are written by this helper.

The helper requires the actual application/sheet to be in the foreground. It
returns only allowlisted native control metadata, compatible COM8 identities and
pilot diagnostics. Hidden, clipped, disabled, obscured or ambiguous controls are
refused. Expose a clipped configuration control using an explicit `ScrollDown`
or `ScrollUp` action; these operate the real page scroll buttons only while the
Autopilot configuration page is visible. No coordinate or arbitrary text target
is accepted.

Actions are individually selected; this list is a human procedure, never a
scripted command sequence:

- `Observe` records current controls and actual pilot mode, source, freshness,
  sequence, epoch, command ID/state and optional magnetic heading values/qualities.
  Missing angle values remain null. It never invents physical acknowledgement.
- `OpenSettings`, `PilotTab`, `PilotConnection` navigate to the real configuration
  page; `Advanced` exposes identity setup. `OpenPilot` or `BackToPilot` opens the
  manual drawer. Each navigation action is one native click.
- `OpenIdentity` opens the real identity sheet. `SetInterface -Value COM8` edits
  that field only. `SaveIdentity` can save an empty NAME to allow a subsequent
  `RefreshIdentity` request. Discovery remains the product's rate-limited
  non-steering address-claim request, never an automatic operation.
- After observing a real compatible claim, `SetName -Value <observed-name>` edits
  the NAME field. Both setting and saving a nonempty NAME require the exact unique
  compatible NAME from the actual configuration identity controls, a verified
  identity and recent (at most 30 seconds old) COM8 PGN 60928 at its displayed
  address. A heading PGN/address alone is insufficient. `SaveIdentity` is separate
  and returns product permission to display-only. Refresh again if the product
  needs a new claim after binding.
- With fresh matching physical mode feedback, `Permit` opens the permission
  sheet; `AcceptPermission` explicitly clicks **Save manual permission**.
  `Enable -ExpectedEpoch <observed-epoch>` opens the session sheet;
  `AcceptEnable -ExpectedEpoch <observed-epoch>` explicitly clicks **Enable manual
  control**. Both accept actions require their exact owned foreground dialog.
- `Standby`, `Auto`, `Minus1`, `Plus1`, `Minus10`, `Plus10` require the caller's
  current `-ExpectedEpoch` and `-ExpectedCommandId` from observation, actual fresh
  compatible feedback, saved permission and enabled session. `Auto` only opens
  its confirmation sheet; `AcceptAuto` with the same reviewed epoch/command ID
  explicitly clicks **Request AUTO**. Course changes require actual AUTO mode.
  Unresolved commands block further commands except explicit STANDBY preemption.
- `CancelIdentity`, `CancelPermission`, `CancelEnable`, `CancelAuto` click Cancel
  on their exact sheet. `Disable` can only switch an already enabled session off.
  `DisplayOnly` clicks **Return to display-only** before ordinary child Close,
  inspection and byte-inverse rollback. No plugin DLL is restored.

A durable nonce-bound intent precedes input. The helper sends one button
press/release pair or one field edit, never a sequence or retry. Its success means
only that UI input was dispatched; even a timeout may have delivered input.
Observe and inspect actual physical feedback plus the product's exact-command
confirmation before proceeding. Do not reinterpret a fresh sequence, a serial
write or a successful native click as command confirmation. TRACK/WIND and raw
NMEA/serial interfaces are absent.

`test-manual-pilot-ui.ps1 -PortableContracts` runs inert selector, state and profile
contracts on Linux PowerShell7. On disposable Windows CI, run without switches
under both supported PowerShell bitnesses. It additionally creates its own native
fixture, resolves and clicks one harmless counter button, verifies disabled-target
refusal and destroys its windows. It never attaches to a product or boat. Actual
wxWidgets accessibility, dialogs and physical feedback remain candidate/boat
qualification gates; these fixtures do not claim them.
