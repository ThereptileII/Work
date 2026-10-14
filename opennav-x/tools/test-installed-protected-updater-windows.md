# Installed protected updater phase driver

`test-installed-protected-updater-windows.py` runs one real installed phase:
`no-update`, `later`, `now`, or `offline`. It does not build, publish, enroll,
repair, adopt, reset trust, change connectivity, seed a profile, force termination,
or clean up an installation. Run only on an explicitly disposable, interactive
GitHub-hosted Windows runner in `ThereptileII/Work`. Never run it on the boat.

Before running, independently authenticate two genuine fixture-free recovery
packages, their corresponding source archives, and their exact beta policies.
B1 must have a strictly newer supported product version and a different compiled
commit than B0; changing a filename, policy, or version declaration cannot qualify
reused application bytes. Both products need the actual compiled qualification,
startup-health and restart capabilities and the same committed public trust.
The same-commit B0 operator kit must have passed its existing exact source/build
verifiers and must pair with the B0 recovery/launcher prerequisite. An independently
approved original B0 client-registration request must already have been used for
explicit protected installation and initial qualification. This driver never
supplies installation authority.

The installed broker and staged diagnostic executable must include the real
`--maintenance=selected-history` endpoint (introduced by source commit
`6e182d0b`). The registered user running the driver must have an actual elevated,
nonimpersonated primary token for the diagnostic observations. Startup itself
now uses only that account's original linked LIMITED primary token through
`open_same_account_protected_startup`; an elevated `Popen` fallback is forbidden.
The protected image path is rebuilt from actual ProgramFilesX86, current SID and
the approved generation, with only `--broker-start=<generation>`. The original
limited-token logon identity and handles remain retained through prompt/app
observations. Actual prompt/app tokens must match that limited logon, SID and
session and be primary, medium-integrity and unelevated before UI input or
readiness acceptance. Explicit `WinSta0\\Default` avoids implicit desktop-access
additions. This code still needs native execution on a suitable signed-in
disposable host; injected-OS tests do not establish ordinary-user acceptance.
The native endpoint independently authenticates the actual client and
original protected selected history; printed diagnostic fields are observations,
not authority to mutate anything.

Use the broker's actual disposable profile at the Windows ProgramData known
folder plus `opencpn` (`brokerhost/execution_child_profile_windows.go` and
`execution_profile_capture_windows.go`), with nonempty navigation and explicitly
approved existing stable settings in `opencpn.ini`. A current `navobj.db` takes
precedence over `navobj.xml`; a stale XML backup cannot stand in for the database.
It must have no
active hardware/network equipment connections and no physical equipment attached.
The driver never enables control or clicks equipment controls. It accepts the
existing reviewed `status-only` or versioned `manual-commissioning` product build
contract; acceptance does not certify hardware output behavior. Close every
OpenCPN, launcher and update-prompt process before each phase. The driver refuses
any such existing process, even without a visible window.

## Joined installed journey

`installed_updater_journey.run_joined(initial, phases, pins, output, *, handoff)`
composes the existing initial B0 admission, all four phases and explicit protected
manual rollback to the original qualified B0 before the real Maintain uninstall.
Prepare `initial` through the B0 driver's `prepare`, and
`phases` through this driver's `prepare`, using independently approved original
declarations. Both must identify the same B0, client request, operator kit,
existing settings and original file identities. The callable does not supply
host authorization or approval, and must not run on the boat.

The operator's explicit `handoff(phase_name, expected_selected_head)` performs
only already authorized publication/connectivity changes outside the driver.
It returns an independently approved timestamp hash for each online phase and
exactly `None` for both `offline` and `manual-rollback`. At the latter handoff,
keep the already authorized offline condition through rollback and restored B0
startup; returning a timestamp refuses. Actual offline observers
still verify the configured endpoint before and after the rollback/startup step.
The journey creates no approval, publication or connectivity change.

One Setup/enrollment is followed by initial B0, no-update, Later, Now, offline B1,
explicit manual rollback, restored ordinary B0 startup/normal close, then Maintain.
The rollback sends the existing pinned provisioner's one-shot
`--maintenance=manual-rollback` request with a fresh request ID, the observed B1
selected head and the original qualified B0 generation/completion. Its exact
`manual-selected` result must match fresh protected history; the B0 version,
commit, completion and historical qualification/clean-close facts must match the
original admission. A new selected head distinguishes explicit rollback from
replayed original history. The current profile must remain preserved throughout.
The restored B0 launch has its own named observation and original limited-token
process checks; it is not recorded as a no-update or offline B1 phase. The helper
never retries or kills an uncertain maintenance process, including on its bounded
30-minute wait. A timeout can leave committed effects requiring operator review.

Every step gets a fresh native observer using the same original
SID, profile and registration. Generation-image owners drain before the next
handoff or retirement. The fixture's original broker query handle stays retained
and live through the phases, then closes before Maintain. The original profile
is compared before and after each phase, including between phases.

Pass an absolute `result.json` in a fresh evidence directory. Per-phase results
are create-only; partial failures remain recorded. A failure stops subsequent
steps and uninstall without retry or automatic cleanup. The separate
`manual-rollback/result.json` and joined result set
`nativeInstalledManualRollbackAcceptance` only after all rollback/startup checks
and original owner cleanup pass. Product and customer UAC acceptance remain
false. Repair, automatic rollback and functional stock startup after uninstall
are outside this composition. Portable ordering/refusal checks cannot qualify
native execution; a suitable signed-in disposable Windows host remains required.

## Approved fixture declaration

Prepare this declaration outside the driver. All paths must be absolute ordinary
single-link files with no symlink/reparse ancestors. Independently approve the
SHA-256 of the complete declaration, rather than treating a locally printed hash
as approval. Digests and commits use lowercase hexadecimal. Unknown declaration
fields, duplicate JSON keys, changed original file identities, and changed bytes
refuse. The two policy files are the original `releases/beta.json` bytes.

```json
{
  "schema": 1,
  "b0": {
    "version": "<genuine B0 version>",
    "commit": "<B0 40 hex compiled commit>",
    "recovery": {"path": "C:\\approved\\b0-recovery.zip", "sha256": "<64 hex>"},
    "source": {"path": "C:\\approved\\b0-source.zip", "sha256": "<64 hex>"},
    "policy": {"path": "C:\\approved\\b0-beta.json", "sha256": "<64 hex>"}
  },
  "b1": {
    "version": "<strictly newer genuine B1 version>",
    "commit": "<B1 40 hex compiled commit>",
    "recovery": {"path": "C:\\approved\\b1-recovery.zip", "sha256": "<64 hex>"},
    "source": {"path": "C:\\approved\\b1-source.zip", "sha256": "<64 hex>"},
    "policy": {"path": "C:\\approved\\b1-beta.json", "sha256": "<64 hex>"}
  },
  "operatorKit": {"path": "C:\\approved\\b0-operator-kit.zip", "sha256": "<64 hex>"},
  "clientRequest": {"path": "C:\\approved\\original-client-request.json", "sha256": "<64 hex>"},
  "trustSha256": "<64 hex exact shared public trust bytes>",
  "preserveSettings": [{"section": "Settings", "key": "<existing stable approved key>"}]
}
```

The driver verifies the recovery inventory, original PRODUCT_BUILD, qualification
sidecar, installer package identity, compiled source version and source trust
binding. It verifies the kit using the existing launcher/component verifiers,
without building or running a producer. It copies only the fixed kit closure plus
its B0 launcher verification prerequisite into a fresh evidence directory. The
original approved inputs are checked again before and after the actual phase.

## Explicit phase sequence

Use a fresh evidence directory for each invocation. First obtain the original
selected head through the independently authenticated diagnostic command; pass
that exact observation explicitly. Use the same original registration throughout.
There is no automatic retry, restart, enrollment, publication, or network switching.

```powershell
# Existing protected B0, with the approved B0 policy published.
python tools/test-installed-protected-updater-windows.py `
  --fixtures C:\approved\fixtures.json --fixtures-sha256 <approved-fixtures-sha256> `
  --evidence C:\acceptance\no-update --disposable-opt-in SCRUM-311-disposable-installed-protected-updater `
  --phase no-update --expected-selected-head-sha256 <observed-B0-head> `
  --timestamp-sha256 <independently-approved-B0-publication-timestamp-sha256>

# Separately publish/authenticate B1 through the existing operator procedure.
python tools/test-installed-protected-updater-windows.py `
  --fixtures C:\approved\fixtures.json --fixtures-sha256 <approved-fixtures-sha256> `
  --evidence C:\acceptance\later --disposable-opt-in SCRUM-311-disposable-installed-protected-updater `
  --phase later --expected-selected-head-sha256 <same-B0-head> `
  --timestamp-sha256 <independently-approved-B1-publication-timestamp-sha256>

python tools/test-installed-protected-updater-windows.py `
  --fixtures C:\approved\fixtures.json --fixtures-sha256 <approved-fixtures-sha256> `
  --evidence C:\acceptance\now --disposable-opt-in SCRUM-311-disposable-installed-protected-updater `
  --phase now --expected-selected-head-sha256 <same-B0-head> `
  --timestamp-sha256 <independently-approved-B1-publication-timestamp-sha256>

# Separately make the configured origin unreachable on this disposable host.
# Supply the newly observed B1 head from the successful Now result.
python tools/test-installed-protected-updater-windows.py `
  --fixtures C:\approved\fixtures.json --fixtures-sha256 <approved-fixtures-sha256> `
  --evidence C:\acceptance\offline --disposable-opt-in SCRUM-311-disposable-installed-protected-updater `
  --phase offline --expected-selected-head-sha256 <observed-B1-head>
```

Online phases pin the exact timestamp and its hash/length-linked numbered snapshot
and targets to the approved B0 or B1 policy, before and after application execution.
This observer does not implement a second signature verifier: the real broker
performs its existing signed-metadata and monotonic-floor checks. Offline requires
an actual connection/DNS/timeout failure at the configured HTTPS origin, before
and after execution. HTTP rejection, TLS failure, redirect or malformed successful
response refuses rather than counting as offline.

The driver launches only the exact original protected B0 launcher with its fixed
baseline argument. Later and Now require the actual pinned update-prompt process,
original native process identity, unique enabled expected button, and actual exit
contract. Now must produce a new selected B1 head with different original health
and clean-qualification-close facts and retain the exact B0 ancestor completion.
The real broker and real application produce all health/selection evidence; this
driver never supplies it. No-update, Later and offline retain the selected head.

Every phase observes a fresh real ordinary application process from the selected
protected generation, fresh deferred-initialization log evidence, an enabled actual
application window, normal close, and the original retained process's clean exit.
No forced termination is used, including after timeouts. The original image handles
remain retained throughout the phase and cleanup uncertainty refuses.

## Evidence limits and failures

Exit zero and `result.json` with `nativeInstalledPhaseAcceptance: true` mean only
that explicit phase passed. `nativeProductAcceptance` stays false: one phase is
not full release, installer, repair/decommission, boat, mode-switch, or standard-user
acceptance. All four original results and their exact fixture/harness pins must be
reviewed together; there is no automatic aggregate acceptance.

Preservation checks compare navigation GUIDs, names, descriptions, coordinates,
times, route waypoint order and track point order, plus hashes of the named
existing setting values. For two XML observations the complete canonical XML
must also agree. For two database observations all eight supported tables' row
values and HTML links must agree, excluding only import-assigned `created_at`.
SQLite file layout changes alone do not fail this comparison. Reads use an
immutable read-only connection; corrupt/incomplete schemas, NULL text payloads
unsupported by the native readers, orphaned links,
invalid order, redirected/linked files, and any journal/WAL/SHM sidecar refuse.
The driver never repairs, checkpoints, migrates or falls back from a current
database to XML.

XML-to-database migration is accepted only for the exact existing
`tests/fixtures/mode-persistence.gpx` bytes (SHA-256
`24be1789c06aa08cbca0cd470ece1fc664b8e5f74fac8bdf27b403314e5021dc`). The
Windows CRLF checkout form is separately pinned as
`90915745d6dae09851a6cdd5e87086b9f0152e52b6d959427b8f372bedff3627`;
only those two known byte forms are allowed. The fixture's entire
object set and ordered geometry must remain, with diamond symbols, the mark's
explicit visibility/shared flags, route visibility/planned speed 5, track
visibility and empty HTML-link tables. Other XML migrations refuse, even when a
reduced geometry projection agrees. The observer does not seed this fixture.
This follows the existing `profile-fixtures.py` controlled-fixture boundary; it
does not claim preservation of arbitrary XML extensions or every importer default.
In particular, the importer stores point `Type` as literal `type`; this scoped
fixture comparison does not claim that GPX `WPT` text survives unchanged.

The schema, nullability/defaults, primary/foreign keys and order conventions are
pinned to OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
[`model/src/navobj_db.cpp`](https://github.com/OpenCPN/OpenCPN/blob/37fd0cddb7334fe489e9f18aa163977a9c5c84f7/model/src/navobj_db.cpp):
`CreateTables` lines 67–174, route links 1185/1262 (one-based), track insertion
809–812/988 (zero-based), and point time/type bindings 1455–1458. Named additional
tables, indexes, views or triggers refuse; SQLite's internal primary-key indexes
are allowed. The portable database fixture uses the original `CreateTables` SQL,
not a reduced schema. Native migration against the selected application remains
required before this helper constitutes execution evidence.

Only hashes and store/controlled-fixture identifiers are emitted; no profile
contents or setting values are emitted. This is scoped preservation, not a claim that every profile file,
plugin, chart, connection, track file, or concurrent writer was qualified. Ordinary
startup may legitimately normalize settings; select genuinely stable settings
through independent review. The driver does not rewrite them to obtain a pass.

Selected-history proves original selected qualification and absence of a pending
transaction at observation. It does not report the broker's internal metadata-fetch
reason. An independently pinned reachable B0 publication plus unchanged selected
head qualifies observed no-update behavior, but cannot distinguish that internal
reason from a broker check refusal that safely kept B0. Offline likewise observes
configured-origin unreachability and correct retained selection; it does not invent
an internal offline status. A future reason-specific assertion must use actual
broker observations, not reinterpret unchanged history as proof of a fetch outcome.

Any uncertainty returns a sanitized refusal, preserves the new evidence/operator
stage and leaves original processes/state for operator inspection. It never adopts
or deletes an unknown namespace, suppresses a pending history refusal, or retries a
transaction. Native B0/B1 installed execution remains required; the portable inert
unit tests do not substitute for this gate.
