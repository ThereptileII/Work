# Public-beta software release and updater incident runbook

Owner: coordinating release operator. Scope: SCRUM-77. This is an operator
procedure, not authorization to publish, sign, install, roll back, change hosting,
or contact users. It covers the software delivery and updater boundaries only.
Jira remains the incident/work backlog. Record the incident there before recovery.
The [delivery policy](../delivery-workflow.md) controls Staging, Production and
public-access authorization. Current public download access remains closed;
GitHub releases remain drafts. A successful exercise is not launch acceptance.

## 1. Stop the affected operation and identify the boundary

Assign one incident coordinator and one independent evidence reviewer. Record
UTC time, reporter, exact symptom, last successful operation and whether any
installer/application process is still running. Stop issuing new publish, signing,
install or retry requests for this candidate. Preserve the frozen candidate and
unrelated work. Do not cancel an executing installer, force-kill the application,
delete pending state, reset version floors, replace assets or clear trust caches.

Choose the first matching row; multiple boundaries can remain blocked together.

| Observed condition | Route | Immediate disposition |
| --- | --- | --- |
| Suspected stolen signing key, unexpected valid signature, changed trust root or compromised operator account | [S — Signing/trust incident](#s--signingtrust-incident) | Stop signing and publication; identity verification alone cannot clear the incident. |
| Unexpected commit, package hash, version, channel, tag, source or qualification | [W — Wrong candidate](#w--wrong-candidate) | Quarantine the selection; no overwrite or relabelling. |
| Installed update has a pending transaction, uncertain installer completion, failed health receipt or still-running candidate | [I — Installed update](#i--installed-update) | Preserve custody and pending records; do not start competing maintenance. |
| GitHub draft asset upload failed or its response was lost | [G — GitHub delivery interruption](#g--github-delivery-interruption) | Inspect the exact draft and original run before retrying. |
| TUF publisher interrupted, timestamp expired, or clients reject metadata | [T — Update metadata/publication](#t--update-metadatapublication) | Keep client rejection and monotonic state intact. |
| Qualified software regression without the conditions above | [R — Rollback or hotfix](#r--rollback-or-hotfix) | Freeze this rollout and assess exact prior generation versus new release. |
| Evidence does not distinguish these cases | [H — Hold](#h--hold-and-resume-record) | Collect evidence; an unknown result is not permission to repeat a mutation. |

## 2. Preserve the evidence set

Use a new access-controlled incident directory outside the checkout and public
hosting root. Keep original records unchanged; record SHA-256 for retained files.
Never collect signing seed envelopes, tokens, credentials, raw navigation data
or whole personal profiles into general CI logs. Private profile backups, when
needed for an authorized local recovery, remain access controlled separately.

Record these identities explicitly; do not substitute a branch name or “latest”:

- Product: full commit, embedded version, candidate ID, channel, GitHub release
  tag/ID, producer run ID/attempt and executable/package/source hashes.
- Evidence: `RELEASE.json`, `SHA256SUMS.txt`, `QUALIFICATION.json`, available
  `RETEST_SUPPORT.json`, original failed logs, completed required jobs and any
  `STAGING_PUBLICATION.json`; helper/retest/publication commits and runs separately.
- TUF, if configured: reviewed selection/policy hashes, public bootstrap/config
  hash, role key IDs (public only), metadata versions/expiry, local `state.json`,
  `public/timestamp.json` and the immutable files it references. Preserve the
  entire operator repository consistently after confirming no active writer.
- Installation, if involved: current/previous generation, ownership/state hashes,
  pending transaction identity, known-good/health and lifecycle records, actual
  process status, launcher/application hashes and stock OpenCPN identity.

### Read-only release inspection commands

Run from `opennav-x` in a reviewed checkout with Python and an already authenticated
GitHub CLI. Set `RUN_ID`, `ATTEMPT`, `TAG`, `PRODUCT_COMMIT` and `INCIDENT_DIR` to
reviewed values from the incident, not this document. The incident directory must
already exist and filenames below must be unused. Commands read remote state and
write evidence files; none publishes, dispatches or executes a package.

```sh
set -eu
set -C
: "${RUN_ID:?Select exact run}" "${ATTEMPT:?Select exact attempt}"
: "${TAG:?Select exact Staging tag}" "${PRODUCT_COMMIT:?Select full product SHA}"
: "${INCIDENT_DIR:?Select private new evidence directory}"
git rev-parse HEAD > "$INCIDENT_DIR/helper-commit.txt"
gh api "repos/ThereptileII/Work/actions/runs/$RUN_ID/attempts/$ATTEMPT" > "$INCIDENT_DIR/run.json"
gh api --paginate --slurp "repos/ThereptileII/Work/actions/runs/$RUN_ID/attempts/$ATTEMPT/jobs" > "$INCIDENT_DIR/jobs.json"
gh run view "$RUN_ID" --repo ThereptileII/Work --attempt "$ATTEMPT" --log > "$INCIDENT_DIR/run.log"
gh release view "$TAG" --repo ThereptileII/Work --json id,tagName,targetCommitish,isDraft,isPrerelease,assets > "$INCIDENT_DIR/release.json"
```

For a completed Staging delivery, authenticate and download to a **new/empty**
directory, then validate local immutable package/source/notices:

```sh
python3 tools/github_release_delivery.py fetch-staging \
  --repository ThereptileII/Work --tag "$TAG" --directory "$INCIDENT_DIR/release"
python3 tools/release_manifest.py verify \
  --directory "$INCIDENT_DIR/release" --commit "$PRODUCT_COMMIT"
sha256sum "$INCIDENT_DIR/release/RELEASE.json" \
  "$INCIDENT_DIR/release/QUALIFICATION.json" \
  "$INCIDENT_DIR/release/SKAGER-Beta2-Setup.exe" \
  "$INCIDENT_DIR/release/SKAGER-Beta2-source.zip" > "$INCIDENT_DIR/verified-release-hashes.txt"
```

A partial/failed delivery can legitimately refuse `fetch-staging`. Preserve that
refusal and inspect its original assembly/draft under route G; do not forge a
completed receipt or weaken the verifier. Local manifest verification alone does
not authenticate CI, qualify the release, validate TUF signatures or grant GO.
Logs may contain private environment details; review before sharing.

## G — GitHub delivery interruption

1. Determine whether compilation/qualification/assembly failed, or only transport
   failed. Failed product gates require their own fix/qualification. The retained
   publication continuation does not turn failed tests into passing evidence.
2. Inspect original run/attempt, exact candidate artifact ID/name/digest/size,
   manifest and qualification hashes, and draft ID/tag/target commit. Confirm the
   draft remains a prerelease and is not public. Retain all existing asset IDs,
   sizes and bytes; a timeout can hide a completed upload.
3. For a composed candidate with successful authenticated assembly and an existing
   matching draft, use the existing [retained publication workflow](../../../.github/workflows/skager-staging-publish.yml).
   The coordinator must review and commit the incident's exact selection in
   [staging-publication-request.json](../../tools/staging-publication-request.json)
   on `skager-staging-publish`. Its checked-in historical values are **not defaults
   for this incident**. An unrelated operator must compare every field to the
   original authenticated records before dispatch.
4. Only after route H approval, the coordinator dispatches the reviewed branch:

   ```sh
   gh workflow run skager-staging-publish.yml --repo ThereptileII/Work --ref skager-staging-publish
   ```

   The workflow invokes `github_release_delivery.py resume-staging` with the
   committed request and authenticated Actions execution. Do not simulate its
   `GITHUB_*` environment or invoke it as an ad hoc local bypass. Pin the actual
   dispatched helper commit/run/attempt in the incident and compare it with review.
5. Resume verifies existing assets and adds only missing ones. Missing draft,
   unexpected/replaced assets, byte conflicts or a conflicting prior transport
   receipt are **HOLD**, not instructions to delete/overwrite. The tool cannot
   reconcile those cases automatically. Non-composed failures follow the original
   workflow's verified retry path only after coordinator review; do not force them
   into the composed-publication request format.
6. Close this branch only when the continuation completed successfully and a new
   `fetch-staging` authenticates both original assembly and continuation, verifies
   unchanged bytes and retains the separate publication receipt. Preserve the
   original failed upload. Neither this operation nor its success promotes to
   Production or opens public access.

## T — Update metadata/publication

The [operator repository contract](staging-update-repository.md) is authoritative.
A local `publish` changes the private repository's public output; it does **not**
upload to hosting or change any installed client. Do not confuse GitHub delivery,
TUF metadata and installed-generation publication.

1. Distinguish stale client/host cache, timestamp/snapshot/targets expiry, root
   expiry, invalid signatures, wrong origin, missing immutable files and local
   publisher interruption. Preserve exact public bytes and acquisition time.
   Check UTC against an independently trusted clock; never move the clock backward
   or disable expiry to make verification pass. Suspected unauthorized signatures
   or trust changes go to S; unexplained mismatches go to W/H.
2. With no active writer, retain monotonic `state.json`, public files and the
   reviewed policy/selection. `operator.lock` is an OS-held writer lock: its file's
   existence alone does not prove a live writer. Do not remove lock/state files or
   copy an older state snapshot back to bypass a refusal.
3. If only online metadata expired or publication was interrupted, keys remain
   trusted, root is valid and state is unambiguous, approve **the identical policy
   and selection**, or a separately qualified higher product version. Interrupted
   publication consumes its reserved metadata version; the retry advances it.
   Equal product precedence with another commit/policy/selection and lower product
   versions are refused. Old immutable metadata must remain available.
4. After H approval, use the existing reviewed Linux operator executable. Replace
   paths and the selection pin with incident-reviewed values. Supply the three
   online role keys through the already approved secret-manager pipe to stdin;
   the pipe is environment-specific and is deliberately not invented here.

   ```sh
   skager-repository publish --repository /private/operator/repository \
     --assets /verified/retained-release --policy /reviewed/beta-policy.json \
     --selection /reviewed/selection.json --selection-sha256 REVIEWED_SELECTION_SHA256 \
     --keys-stdin
   ```

   No root key belongs in this operation. Do not use `initialize` as repair, put
   decrypted keys in arguments/logs, or change a reviewed policy to get past a floor.
5. Retain the `published` result and new `metadataVersion`; independently compare
   timestamp/version/hash links, policy identity and unchanged asset bytes. The
   repository switches `timestamp.json` last, after immutable dependencies.
   Any external deployment must preserve that order and old immutable files, then
   verify served HTTPS bytes and an actual authenticated client refresh against
   its existing retained state. No hosting deployment or standalone client-refresh
   operator CLI is implemented here: absent an approved deployment/verification
   procedure, stop at **local output prepared; serving/resume blocked**.
6. Expired root, missing/ambiguous state, interrupted initialization or required
   root rotation is **HOLD** for separate reviewed trust recovery. The current
   publisher does not implement root rotation. Export/validation can inspect
   public configuration, but cannot repair these conditions:

   ```sh
   skager-repository export-public-root --repository /private/operator/repository
   skager-repository export-public-config --repository /private/operator/repository
   skager-repository validate-trust --config /reviewed/public/update-trust.json
   ```

   `validate-trust` validates bootstrap configuration; it does not prove current
   timestamp freshness, served artifact availability or installed update health.
   Offline/rejected metadata normally leaves the installed app usable; pending
   transactions or local integrity failure must instead follow I.

## W — Wrong candidate

Keep the selected package, source, release record, signed policy, user instruction
and actual observations. Name the exact field that differs. Never rename a package,
edit a sealed manifest, replace a draft asset, re-sign accepted bytes or change the
commit under an existing version to make them agree. Record which clients, if any,
received the offer or installed the package; lack of reports is not proof of zero
exposure. If the mismatch is merely an unexecuted operator selection, select the
correct immutable candidate and repeat full authentication before H approval.

If wrong bytes were signed, served or installed, stop this rollout and involve
both release and security owners. A correctly signed unintended candidate is
still unintended. Determine whether S applies, preserve monotonic metadata floors,
and choose R. There is no implemented emergency revoke command; controlling an
external endpoint requires its separate authorized operations procedure.

## S — Signing/trust incident

Treat suspected TUF role-key, Authenticode certificate or operator-account
compromise as a security incident even if signatures verify. Record public key
IDs/certificate thumbprints, affected time interval, exact signed object hashes,
last independently trusted state, account/audit evidence and possible exposure.
Keep secret material out of the incident bundle. Stop using suspect keys and
coordinate authorized containment with their actual custodian; this repository
has no secret-manager, certificate-revocation or emergency hosting control tool.

TUF role keys and Windows Authenticode certificates are different trust systems;
replacing one does not remediate the other. Root rotation is not an available
publisher command. Do not mint a new bootstrap root in place, delete initialized
trust markers, reinitialize clients, or replay an old repository backup. Hold
until a separately reviewed recovery/rotation or migration procedure, trusted
custody, revoked/replaced credentials as applicable, exact affected releases and
client recovery plan are approved and qualified. See [signing](code-signing.md)
and [updater trust](secure-updater.md#trust-and-retained-state). Preserve the failed
and suspect records; successful signing with new material alone is not clearance.

## I — Installed update

Record process state before doing anything that may start the application.
A lost response or fifteen-minute installer deadline does not prove the installer
stopped. Never kill it to trigger rollback, launch another installer, or remove
pending/transaction/ownership records. The [updater supervisor](secure-updater.md)
recovers pending state through its guarded lifecycle before ordinary startup;
inability to close a candidate normally leaves pending state intact.
While an installer remains running or completion is uncertain, preserve process
and pending-transaction evidence and defer `Diagnostics` until completion is
confirmed and no maintenance writer is active. `Diagnostics` does not acquire
the transaction lock and cannot establish a consistent installation snapshot
during concurrent changes.

For an authorized diagnostic collection on the affected Windows account, use the
exact reviewed maintenance script and its sibling files from that generation.
Set `$ReviewedLifecycle` and `$IncidentReport` to verified local paths; the report
must be new. This writes a diagnostic report and logs, not a repair or rollback:

```powershell
powershell.exe -NoProfile -File $ReviewedLifecycle -Action Diagnostics -Report $IncidentReport
```

The tool is [Lifecycle.ps1](../../installer/windows/Lifecycle.ps1); its fixed root
is `%LOCALAPPDATA%\OpenNavXAlpha1`. Inspect the report's stock/file comparisons,
not only process exit code. Refused diagnostics or mismatches require H review.
Do not run a repository-head helper against an older installation as if its
source had been qualified for that generation. Corrupt or untrusted maintenance
code must not be executed merely to obtain a report.

Manual rollback requires exact current/previous identities, reviewed profile and
stock preservation, closed processes and the existing lifecycle safeguards.
This runbook deliberately grants no automatic rollback or arbitrary-generation
switch. Boat commissioning has additional generation/profile custody requirements;
route any boat case to [boat maintenance](boat-maintenance.md) and
[signed-update boundary](boat-signed-update-boundary.md) without applying this
software tabletop as permission to launch or modify the boat.

## R — Rollback or hotfix

| Evidence | Decision |
| --- | --- |
| One failed installed update; verified prior healthy generation and unchanged recovery authority; graceful closure and profile preservation proven | Coordinator may approve the existing guarded local recovery. It does not republish an older version. |
| Prior generation is unknown, unsafe, incompatible, tampered, or preservation cannot be established | HOLD; do not select it because it is older. Preserve evidence and assess an independently qualified recovery/hotfix. |
| Regression affects a distributed version; previous release is otherwise usable | Stop further rollout. Prefer a newly qualified hotfix with a strictly higher embedded version and exact source/policy; publisher/client floors prohibit advertising an older version as a normal update. |
| Suspected trust compromise | S governs first; a hotfix signed by the suspect authority cannot clear trust. |
| Only upload transport failed; exact product remains qualified | G/T as appropriate; avoid rebuilding or creating a hotfix for unchanged bytes. |

Every hotfix repeats affected software/security/native installer and source/license
gates for its exact bytes. Retained evidence may be reused only when its inputs and
scope are unchanged. Production and public-access instructions remain separate.
Recovery never restores an old navigation profile just to match an older binary.

## H — Hold and resume record

Before any mutating recovery, the coordinator writes a concrete proposal in the
incident: boundary/branch, verified cause versus hypothesis, exact inputs and
commands, expected changes, irreversible effects, required owner authorization,
independent reviewer and explicit stop conditions. The reviewer checks all of:

1. Original failed evidence retained; product, helper, policy and run identities
   remain distinct; any exposure is recorded as known, unknown or verified absent.
2. Required authentication, hashes, native qualification and profile/stock
   preservation evidence pass. Missing, expired, skipped or uncertain gates remain
   visibly open; a tabletop cannot close them.
3. No changed trust, metadata rollback floor, package content, public-access state
   or candidate identity is being hidden inside a “retry.”
4. Exactly one authorized operator executes the approved branch and records actual
   return values, new hashes/versions/run identity and independent postchecks.
5. A fresh authenticated fetch/refresh and applicable installed health/recovery
   evidence pass before declaring that boundary resumed. If the current tools
   cannot provide a required operation or proof, state **blocked** and assign the
   specific missing implementation/qualification in Jira. Do not invent a CLI.

Close the incident only after the reviewer records what resumed and what remains
blocked, with links to retained evidence and any required user decision. This does
not grant public-beta GO. Exercise this decision tree using the separate
[inert tabletop cases](release-incident-tabletop.md).
