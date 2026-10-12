# Protected updater execution — SCRUM-311

This describes the joined implementation, not an accepted installed updater.
Jira remains the backlog. The historical [startup updater](secure-updater.md)
still describes the existing user-owned installed launcher. Protected normal
shortcut activation and Update Now are now joined in source, but the installed
Setup/maintenance lifecycle has not yet switched to that path or passed its
native service and boat acceptance.

## Actual service boundary

The SCM broker serves both fixed staging and execution endpoints from the same
protected Store and operation lock. A terminal endpoint cancels its sibling;
both must drain before Store custody is released. A joined cancellation/cleanup
error is not a successful stop. An undrained SCM shutdown retains owners until
the own-process service exits.

Execution intent is explicit: `start-selected` admits the independently approved
baseline executor. Empty history permits initial qualification after a second
locked empty-history check; complete history permits only terminal relay to the
selected executor. Pending/forked history refuses. `execute-session` qualifies a proposed generation;
`start-current` starts the independently selected version. Request IDs and stage
IDs are lookup data, never selection or launch authority. The original native
executor image, interactive token/session/logon, protected registration and
nonforking completed selection history must agree before any command.

An empty original history permits only explicit qualification of the independently
approved initial baseline. Registration alone does not select it. A signed
candidate requires the original completed generation stage seal and receipt,
exact consent/full policy/Setup/Recovery identities, and current signed-offer
eligibility against the actual selected predecessor. The live selected inventory
admits only the current transaction's newly reserved journal pair.

## Qualification and selection

1. Reserve a fresh one-use execution journal and capture the entire fixed
   ProgramData OpenCPN profile into its protected immutable original archive.
2. Commission nonmutating qualification against the original configuration and
   exact application-image capability. Retain native cold profile ancestry.
3. Record consumed spawn intent before the interactive creator is called.
4. Treat the reported PID only as a lookup. Independently open the actual process,
   verify birth, direct parent, token/session/logon and held original image,
   and retain its SYNCHRONIZE handle before waiting for health.
5. Authenticate the actual health-pipe peer as that SAME original child. Normal
   main-thread readiness, CONFIG provenance, minimum lifetime and receipt/EOF
   checks remain required. The launcher's redundant Observe command is not used.
6. Close/drain the original child and record selection only with current policy
   eligibility and original transaction custody still valid. Close the writer,
   reopen the actual protected history and verify the resulting selected head.
7. Relay to a fresh fixed selected launcher. Independently corroborate its native
   process and original executor image. It must authenticate a new `start-current`
   conversation before creating an ordinary application.

Qualification's core-output and third-party-plugin restrictions last for its
whole process. They are never cleared in place. A fresh ordinary selected process
uses normal OpenCPN configuration, plugin and navigation semantics. Local ordinary
startup does not require a reachable update server.

## Failed startup and offline recovery

New candidate launch and selection require current eligibility. Teardown and
recovery instead retain original peer, Store, generation, journal, profile and
previously qualified predecessor authority. A network outage cannot revoke the
ability to stop the original child and recover that predecessor.

Every consumed child receives at most one bounded close attempt. Actual original
exit must be observed before retaining restoration authority. A client report,
hash, PID lookup after custody is lost, successful transmission or arbitrary
absence scan cannot grant overwrite permission.

The restore provider checks the entire cold current profile against the original
archive. It durably records a bound restore intent before its first write.
Automatic writes cover only existing INI/log files and reviewed exact pinned
startup changes. Unknown navigation, connection, pilot, plugin, ACL, namespace or
unreviewed configuration changes refuse before effects. Partial effects retain
the original archive and permanent pending state; no blind retry occurs.

After verified restoration, the original selected predecessor must independently
pass commissioning, restricted startup health and actual close/drain before a
rolled-back terminal is recorded. The selected history is reopened again.
Cancellation, broken peer custody and ambiguous storage failures remain failures.

## Remaining acceptance boundaries

### Explicit operator maintenance prerequisite

The provisioner now accepts a separate, exclusive operation:

```
--maintenance=quiesce-maintenance
--broker-sha256 <independently-approved-installed-broker-hash>
--client-registration-request <original-approved-request-file>
--client-registration-request-sha256 <independently-approved-request-hash>
```

This is not an application startup option. The final operator tool and its
loader chain must be authenticated before elevation. It retains the fixed
original SCM service/configuration/security, protected image/request and actual
broker process/token/creation identity. Only one bounded STOP attempt is allowed;
both SCM stopped/zero-exit and the retained original process's zero exit must be
observed. Restart, changed custody, cancellation, timeout and cleanup uncertainty
remain failures. It never auto-starts a stopped service to invent drain evidence.
Initial approval uses the live trust clock; an expired bootstrap anchor refuses
without resetting floors or substituting caller bytes for completed registration.

**Quiescence is not repair, rollback or uninstall authorization.** Ordinary
application processes intentionally survive a successful startup conversation.
Stopping this broker therefore does not prove the application closed or its
profile is cold. Protected history/armed operations, original child/profile
ownership and the actual maintenance mutation transaction remain separate open
boundaries. The command closes retained owners and performs no application,
profile, shortcut, registry or protected-store mutation beyond stopping the
verified broker. Native Windows acceptance is still required.

### Explicit manual selection and retained ordinary application

Manual rollback is a different operation on the running protected broker:

```
--maintenance=manual-rollback
--request-id <fresh-32-lowercase-hex-id>
--expected-selected-head-sha256 <exact-current-head>
--target-generation <retained-32-lowercase-hex-generation>
--target-completion-sha256 <exact-ancestor-completion>
```

The separate fixed `SkagerUpdateBroker.maintenance.v1` pipe accepts bounded,
strict data-only records. It authenticates the actual SCM server and the original
interactive operator's elevated primary token, including an enabled, non-deny-only
Administrators group. The host obtains the expected client SID from protected
registration. Callers cannot supply a filesystem root, another client's identity,
an executable or a profile restoration command. A request is sent once. Loss of
the completion means the outcome is unverified, not permission to repeat it.

After normal selected startup, the service retains the SAME corroborated original
application process/token handles. A PID lookup or missing process-name scan cannot
establish exit. Live or uncertain predecessors refuse maintenance; missing custody
after service restart also refuses. A signalled original process remains retained
until the maintenance transaction consumes the observation. The bounded per-client
tracker and every maintenance operation share the engine lock. Drained service
shutdown releases query handles without signalling an ordinary application.

The host checks stock compatibility, current registration, original child exit,
legacy armed-marker denial and absence of other known profile writers. The native
transaction freezes CURRENT navigation data, reopens an exact qualified ancestor,
and appends a fresh permanent manual-selection journal. It neither restores a
historical profile nor resets metadata floors. Historical qualification is explicitly
historical; it is not fresh compatibility testing of the ancestor against today's
profile. No application starts and no hardware command is sent by this transaction.
The returned JSON is published only after protected selected-history readback and
successful closure. A partial reservation or failed readback remains uncertain.

The legacy armed-marker observation is denial-only. Repeated absence checks do not
prove a marker never briefly appeared between checks. Original token-derived folder
lookup must succeed; a service-user folder or caller-selected path is not a fallback.
Windows requires `TOKEN_QUERY | TOKEN_IMPERSONATE` for the original token's
known-folder query. Only the explicit elevated operator client temporarily changes
the already validated fixed service ACE from query-only to those two rights on
its SAME retained primary token. Ordinary startup keeps query-only access. No
token duplication, privilege adjustment or thread impersonation is performed.
The service opens an owned folder-query token from the SAME retained process
handle and proves its TokenID/logon against the retained query token. It closes
that handle within the checked callback. The client restores the exact original
query-only descriptor before releasing its query owner, including on cancellation;
observed descriptor/identity drift refuses a blind restore and never reports
successful completion. These are original token-object access rights, not a
filesystem authority or an alternate operator identity.
An abrupt operator-client exit cannot run in-process cleanup: a widened ACE can
remain on a shared primary token, and restoring its DACL cannot revoke handles
already opened by the fixed trusted service. This is not crash-proof revocation;
the grant itself never substitutes for broker authentication or transaction
authorization. Ordinary user startup never enters this grant interval.
Native rights, actual transport, installed lifecycle and recovery acceptance remain
required. This source join does not qualify repair/uninstall or migrate the older
LocalAppData installer layout.

### Protected in-app mode restart

A native Windows restart corroborates the actual process image, the requested
image and the independently opened process-reported native image by retained
physical identities, including canonical handling of 8.3 path spellings.
Native ProgramFilesX86 ancestry and both same-generation companions remain held
through helper creation. A protected application dispatches only its canonical
sibling `skager-start.exe --broker-current=<generation>[:legacy|:safe]`, never a
direct application restart. Exactly one closed mode argument is accepted; extra
profile, portable or arbitrary arguments refuse. The broker still owns selected
history and the original exited application capability. The helper's PID wait
provides scheduling only, not authority.

Protected restarts refuse an armed/invalid commissioning binding and any current
commissioning or `SKAGER_UPDATE_*` environment key. No qualification fence is
stripped. The original qualification-output latch also refuses at CompleteRestart
after startup has removed the original environment flag. Ordinary portable and
commissioning routing retain their existing argument/helper behavior. Focused
portable policy/lifecycle checks pass; native compile/runtime and installed
mode switching remain separate gates.

### Closed ordinary modes and installation ownership

The optional `startupMode` field accepts only `legacy` or `safe`; omission is
canonical XNav. Explicit empty/XNav, aliases, duplicate fields and arbitrary
arguments refuse. The mode is bound to the request, each applicable command,
report and final ordinary process. Candidate qualification always stays XNav.
Legacy/Safe selected startup bypasses the offer only, retaining the qualified
history, original image and current cold-profile gates.

The initial publisher creates fixed `Skager.lnk`, `Skager Legacy.lnk` and
`Skager Safe Mode.lnk` through the same approved baseline image. Only the closed
`--broker-start=<generation>[:legacy|:safe]` arguments differ. All destinations
are checked before publication; any existing Legacy/Safe entry refuses rather
than being adopted. The normal link retains its exact original byte backup.
A synced `publication.json` records independent original/staged identities and
hashes before non-replacing renames. Three moves are not atomic. Failure retains
evidence and uncertain state; it never authorizes automatic retry or cleanup.

The historical user-owned Lifecycle must refuse modification if any fixed
protected service, registry or machine namespace is present or ambiguous,
including partial enrollment. It cannot repair/uninstall protected ownership.
Read-only Diagnostics remain available. Native historical fixtures run before
fixed SCM enrollment; no service removal/reset is permitted to hide a collision.
Protected repair, decommission and migration require their own owner-scoped
transactions and acceptance; quiesce and manual selection do not authorize them.

### Release acceptance

- Native Windows execution of the joined service/provider/relay/restore path.
- Native execution of the bounded fresh-pipe handoff correction; the source
  waits only after actual busy-pipe evidence and authenticates the actual server
  before sending bytes, without connected replay.
- Native/installed qualification of joined protected normal-shortcut/startup
  admission and Update Now. The initial operator path is explicitly selected by
  `--activate-startup=true`, requires the actual approved current-user identity,
  and activates only the original newly provisioned service after registration.
  Existing installed Setup still uses the earlier supervisor; protected repair
  and decommission remain separate integration work. Joined manual selection
  requires native and installed acceptance before it is considered available.
- Exact build-selected sound-command serialization and additional legitimate
  startup profile normalization; new directories/navigation migrations are not
  licensed for automatic deletion or overwrite.
- Service-restart recovery: the actual Store/host readonly original archive opener
  is joined and permits inspection only. A serialized journal
  cannot recreate a process-local original closed-child handle or resume a
  partially attempted restore.
- Actual provisioning/SCM principal access, full Windows application/package and
  installer lifecycle, packaged security gates, and exact candidate boat tests.

No Production/public release, design review, endurance run or physical command is
authorized by this source integration. See the [integration evidence](../evidence/scrum311-execution-integration-20261012/README.md).
