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
nonimpersonated primary token. This test does not cover unelevated ordinary-user
startup. The native endpoint independently authenticates the actual client and
original protected selected history; printed diagnostic fields are observations,
not authority to mutate anything.

Use an already initialized disposable profile with a nonempty `navobj.xml` and
explicitly approved existing stable settings in `opencpn.ini`. It must have no
active hardware/network equipment connections and no physical equipment attached.
The driver never enables control or clicks equipment controls. It accepts the
existing reviewed `status-only` or versioned `manual-commissioning` product build
contract; acceptance does not certify hardware output behavior. Close every
OpenCPN, launcher and update-prompt process before each phase. The driver refuses
any such existing process, even without a visible window.

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

Preservation checks compare exact before/after nonempty `navobj.xml` bytes and
hashes of the named existing setting values. No profile contents or setting values
are emitted. This is scoped preservation, not a claim that every profile file,
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
