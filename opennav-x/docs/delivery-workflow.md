# Versioned Staging delivery and Production promotion

Product-owner decision: 2026-10-04.
Workflow authority: [SCRUM-97](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-97).
CI implementation: [SCRUM-290](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-290).
Jira remains the sole backlog; this document records the delivery policy.

## Default channel and delivery authority

Development and ordinary delivery default to **STAGING**. Do not start Production
qualification or promotion without an explicit user instruction for the selected
candidate. Feedback, a successful Staging build or a request to continue ordinary
development does not authorize Production work.

Versioned **GitHub Releases** are the standard delivery record for both channels.
Each release identifies the channel separately from the immutable package version,
source commit, build/run identity, assets and SHA-256 hashes. Staging is a
prerelease; Production promotion reuses the exact selected package bytes and
corresponding source. A channel change does not rename or rebuild its embedded
version. CI artifacts retain intermediate logs and evidence; they are not the
normal delivery destination.

The repository is public, but public-beta payment/download access remains closed.
Create releases as **drafts** in both channels until the separate public-readiness
review and explicit user GO authorize opening access. A draft release requires
repository access and is not a public customer download. An instruction to
promote to Production alone does not authorize publishing that draft.

## Delivery sequence

Feature work → working Staging version → user feedback → frozen candidate →
Explicit user instruction → Production readiness checks → approved promotion.

Staging delivers small coherent batches frequently. Each delivered candidate
has a fixed version/build identifier, commit, package hash and source reference.
It must build, install and start, and pass checks relevant to its changes.
Known safety, security or navigation-data corruption defects cannot be excused
by calling a package Staging. Keep the previous working package recoverable.
New Staging development may continue while a frozen candidate is qualified.

Production promotion performs only the work needed to make that candidate ready
for distribution: relevant functional regressions, navigation/data validity,
security, supported OpenCPN compatibility, installer/update/repair/rollback/
uninstall and profile preservation, recovery, package integrity and corresponding
source/license compliance. Native Windows and relevant boat functional evidence
remain authoritative. Public launch also retains its legal, commercial and
human GO/NO-GO requirements; promotion is not permission to open public access.

## Design validation is explicit opt-in

Do not initiate or require design validation during promotion unless the user
explicitly requests it. This includes prototype/pixel comparisons, aesthetic
reviews, font/color/spacing refinement, design screenshot sets and design-only
DPI sweeps. Do not expand promotion into a redesign or require a fresh boat
screenshot review merely because a candidate is moving channels.

The same explicit-request rule governs design review during Staging work.
The prototype remains the design authority when implementing requested UI work;
this scheduling decision does not change the prototype or fabricate conformance.

Functional correctness remains separate: charts must load, commands must perform
their intended actions, relevant controls must be usable, and data/alerts must
not become misleading or inaccessible. Check a concrete functional defect at
its relevant resolution when needed. Do not use that exception to launch a
general appearance audit or a full visual matrix.

Record unrequested design validation as not requested, never passed. Historical
visual evidence and known design differences remain truthful. Older blanket
visual-promotion requirements and design-only acceptance criteria do not
automatically block promotion under this newer user decision. A mixed issue's
functional or safety defects remain release blockers independently of its
design-conformance work. Do not mark the whole issue Done without its evidence.

## Avoid repeated builds

Retain immutable application/package artifacts before downstream qualification.
Rerun a repaired test helper against those exact bytes where valid, recording
the package source revision and the separate helper revision. Rebuild when
application or package inputs change, not merely because a helper or channel
changes. Promote the exact qualified package; never silently substitute a
newly compiled executable. Channel identity must not require changing the
qualified binary's embedded version at promotion time.

Reuse evidence only when its relevant code, dependencies, environment and
package inputs are unchanged and recorded. Keep failures visible. Separate
affected downstream jobs so an installer test failure need not restart the
application build. Cancel superseded development runs, while preserving an
explicitly frozen qualification candidate. Verified dependency reuse is tracked
in SCRUM-225 and must retain source/toolchain/configuration integrity checks.

This decision does not re-enable endurance testing. The existing explicit
user-directed skip remains until changed; record it as skipped and disclose
any remaining release qualification gap. Do not silently start long tests.

## Workflow responsibilities

- `.github/workflows/opennav-baseline.yml`: ordinary Staging development checks,
  package creation and a versioned draft Staging GitHub Release.
- `.github/workflows/skager-production.yml`: manually selected exact-package
  readiness/promotion, only following explicit user instruction. Preserve the
  source Staging identity and hashes, reuse valid evidence and keep the resulting
  Production GitHub Release in draft pending separate public-access approval.
- `.github/workflows/opennav-prototype.yml`: explicitly requested design review.
  It is separate from ordinary Staging delivery and Production promotion.

The Staging workflow's ordinary push path selects integration/Staging branches
and code changes; documentation/evidence-only edits do not request delivery.
Manual dispatch defaults `extended_tests=false` and
`design_validation=false`. Neither switch authorizes unrequested design work or
changes the existing endurance-testing skip.

Staging release tags use
`skager-staging-<version>-run<runId>-attempt<attempt>-<sha12>`.
The Production workflow requires the selected Staging tag, exact confirmation
`PROMOTE <tag>`, and a reference to the user's Production instruction. The
resulting tag is `skager-production-<sameCandidateId>`. Record the original
Staging release alongside the Production record; the tag's channel changes,
while the candidate identity, package version, source and payload hashes remain
fixed. Keep the existing `SKAGER-Beta2-Setup.exe`,
`SKAGER-Beta2-Portable-Recovery.zip` and `SKAGER-Beta2-source.zip` asset names.

Release records also retain hashed `RELEASE.json`, `QUALIFICATION.json` and
`RETEST_SUPPORT.json` metadata for provenance, qualifications and the supporting
retest inputs. The separate CI support artifact is for installer retests against
retained package bytes; it is not the standard user download. Qualification
records must state skipped/unperformed gates and known failures truthfully.
Support inputs have a 90-day CI retention window. Promotion refuses missing or
expired inputs; it must not silently rebuild an older candidate. Restore the
verified original archive or select a newer Staging release. Intermediate inputs
include the run attempt so a retry cannot claim another attempt's evidence.

## Repository operation

The `staging` branch is the software development/delivery starting point.
The historical `opennav-x-beta2-ui` branch remains synchronized for existing
links and integrations; it still produces Staging, never Production by default.
The monorepo's `main` branch retains the boat firmware and registered workflow
definitions. Select **staging** in GitHub's “Use workflow from” selector when
manually requesting a software build or explicitly instructed promotion.

The small `skager-delivery-checks.yml` workflow tests delivery logic on Linux and
Windows without compiling OpenCPN or performing design review. Run this for
workflow/helper changes; do not create an application build merely to check them.

GitHub requires manually dispatched workflow files on the repository's default
branch. The reviewed workflow definitions are registered there; keep them in
sync when changing release workflows, then explicitly select
the software/Staging branch containing the matching helpers when dispatching.
Do not dispatch a stale firmware-only `main` checkout as a software promotion.
This registration is infrastructure work, not a Production promotion.

Release creation uses the job's scoped `GITHUB_TOKEN`, or the protected
`SKAGER_RELEASE_TOKEN` secret when the repository requires additional workflow
permission. GitHub requires Contents write and Workflows write for a release
target whose workflow files differ from the default branch. Never place that
token in source or logs; a permission failure must stop publication. See
[manual-workflow requirements](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow)
and [release API permissions](https://docs.github.com/en/rest/releases/releases#create-a-release).

These responsibilities define the SCRUM-290 implementation contract. Workflow
source changes are not evidence that a package was built, checks passed, a release
was created, or a boat candidate was qualified. No boat deployment or installed
configuration change follows from this policy update.

## Precedence and preserved evidence

This later user decision governs delivery channel selection, release publication
and the scheduling of design validation wherever older project specifications,
public-beta contract text, checklists or historical CI descriptions differ. The
original specification, public-beta contract, supplied HTML and historical
acceptance/failure records remain preserved. Their safety, functional, security,
source-compliance and data-preservation requirements continue to apply. Prior
visual acceptance does not transfer to a new candidate, and unrequested design
work must be recorded as not requested rather than passed.
