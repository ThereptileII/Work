# Inert release/updater incident tabletop

Exercise for SCRUM-77 and the [incident runbook](release-incident-runbook.md).
All run numbers, selections and incident descriptions below are **invented**.
They are not instructions to access GitHub, use keys, execute an installer,
change a repository, contact anyone or perform boat work.

## Instructions for an independent operator

Read only the runbook and the case inputs first. Do not consult the facilitator
key until you have recorded your answers. For every case, return:

1. Primary runbook route and any secondary route, with the reason.
2. Evidence required before taking the next step, separating facts from unknowns.
3. Next permitted action/real command family, and whether it is read-only,
   requires approval, or is unavailable in the current implementation.
4. The operation you must not perform, exact stop conditions, and proof needed
   before declaring the affected boundary resumed.

Use placeholders when a real identity/path is not supplied. A missing identity
requires collection or HOLD, not a fabricated value. No oral project context is
needed: local command interfaces and boundaries are linked by the runbook.
Record the exercise date, runbook/helper commit, reviewer and case results in the
Jira evidence. A correct **blocked** answer passes a case when tooling or evidence
is absent. Passing the exercise does not qualify a real release or public launch.

## Case A — Draft upload response lost

A composed candidate's original compilation, required qualification and assembly
jobs succeeded. Upload then timed out. A private prerelease draft exists; two
assets appear present. A coworker proposes deleting those assets and rerunning
the build. You have the original run/attempt, artifact ID/digest/size, manifest and
qualification hashes, and release ID, but have not downloaded existing asset bytes.
No `STAGING_PUBLICATION.json` is listed. The checked-in publication request names
a different historical run.

## Case B — Prior continuation left a receipt

The candidate and product assets appear unchanged. A later publication run failed,
but the draft contains `STAGING_PUBLICATION.json`. A new continuation would create
a different receipt identifying its own execution. The new tool reports
`Existing transport receipt differs from this publication execution`.
There is no evidence that the existing receipt's execution completed successfully.

## Case C — Reserved metadata version and expired timestamp

An approved beta policy/selection remains exact and qualified. The publisher
reserved version 42 and then stopped before the timestamp switch. Public readers
still see version 41, whose timestamp has now expired. The root remains valid,
the three online keys are trusted, and the coordinator has independently verified
complete monotonic operator state with no active writer. A colleague offers to
restore the old `state.json` so the retry can reuse version 42. No authorized
hosting/deployment procedure or client-refresh acceptance is available yet.

## Case D — Expired root

Clients reject updates. The bootstrap root has expired; the operator offers all
four old seeds and proposes running `initialize` on the existing repository,
replacing clients' trust configuration and clearing their metadata cache. The
installed application has no pending update transaction and remains usable.
No independently reviewed root migration/rotation implementation exists.

## Case E — Correct signature, unintended commit

The signed beta policy advertises version `0.8.0-beta1`, commit B, while the
approved selection and corresponding source name commit A at the same version.
The signature is cryptographically valid. It is unknown whether any client has
accepted the offer. An operator proposes replacing the asset under the existing
URL with A and editing the manifest to match. No proof of key theft exists yet.

## Case F — Possible credential compromise

An unexpected, validly signed targets object appears. The online TUF role key may
have been exposed on an operator account. Separately, the Authenticode certificate
is still valid and a clean Setup signature verifies. A teammate concludes the
Windows signature clears the incident and suggests immediately signing a hotfix
with the same TUF keys. Root custody and client exposure are not yet established.

## Case G — Installer deadline with live process

Update Now was confirmed; the supervisor timed out waiting for the installer.
The installer process is still running, a pending transaction exists, and no
candidate startup receipt has arrived. The last known-good generation is recorded,
but no one has established graceful process closure or profile preservation.
The installation is a software test desktop, not a boat. A teammate proposes
killing the installer, deleting the pending record and running Rollback.

## Case H — Distributed regression and uncertain fallback

A qualified beta version introduced a navigation-data preservation regression
reported after distribution. The previous package is available, but its health and
compatibility with current profile state are not established. The operator proposes
advertising that older product version in new TUF metadata and restoring an old
profile backup. There is no current signing-compromise evidence. Production or
public-access approval for a new candidate has not been supplied.

## Facilitator key — consult only after answering

| Case | Required disposition and observable pass criteria |
| --- | --- |
| A | G → H. Inspect exact run/attempt and draft bytes; review a new committed incident-specific publication request. After approval, the real retained publication workflow can add missing assets without overwriting or rebuilding. Require successful continuation plus authenticated fetch and unchanged manifest/source hashes. Historical request values cannot be reused. |
| B | G → H, blocked. Preserve conflicting receipt and both execution identities. No receipt deletion, overwrite, forged environment or automatic retry. Independent reconciliation/approved recovery is missing; a locally consistent package does not prove continuation success. |
| C | T → H. Reject restoring old state. Approved identical-policy retry with the three online keys consumes a greater metadata version; retain previous immutable dependencies. Local `publish` can prepare output, but no endpoint/client recovery is claimed without approved serving and real refresh proof. The case remains blocked at that boundary. |
| D | T → H, blocked for separately reviewed root/trust recovery. `initialize`, cache clearing and bootstrap replacement are prohibited repairs. Current publisher lacks root rotation; exporting/validating config does not renew it. Do not classify an ordinary usable app as an installed pending-update recovery. |
| E | W → H, and S if investigation suggests unauthorized signing/trust. Stop the rollout; retain policy/selection/source hashes and unknown client exposure. Never replace sealed assets or the commit under equal version precedence. Reconcile the unintended publication and choose a qualified higher-version hotfix or guarded installed recovery as applicable. Signature validity alone does not establish the intended candidate. |
| F | S → H, blocked. Stop suspect signing/publication; establish custodian, role-key IDs, interval, audit records, root custody and exposure without collecting seeds. Authenticode cannot clear a TUF incident. Require separately approved and qualified containment/trust recovery; no implemented rotate/revoke shortcut may be invented. |
| G | I → H. Preserve running-process custody and pending records. Defer `Diagnostics` until installer completion and absence of a maintenance writer are confirmed; then use verified generation-specific code and review actual comparisons. Do not kill, replay install, clear pending state or manually switch pointers. Guarded recovery requires normal closure and exact preservation evidence; timeout is not proof of termination. |
| H | R → H. Stop rollout and investigate affected data. Unknown fallback blocks local rollback. Do not lower publisher/client floors or restore an old navigation profile. A qualified higher embedded-version hotfix must pass affected exact-byte gates, with channel/public-access authorization kept separate. |

A case fails if the operator invents a command, treats unknown evidence as a pass,
uses a historical request as the current identity, recommends resetting retained
state, conflates GitHub/TUF/installed publication, or claims live/public/boat
acceptance from this exercise. Record omissions and rerun only affected cases
after correcting the runbook. Preserve the first result as exercise evidence.
