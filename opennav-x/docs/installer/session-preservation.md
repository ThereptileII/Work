# Preserve settings changed during active commissioning

SCRUM-310 adds an explicit **current-state preservation** path. It does not
extend the automatic startup-migration policy, establish the origin of saved
settings, authorize an application launch, or approve marine/network output.
The existing recovered root and completed baseline records remain immutable.
All inspection copies, settings, route identifiers, charts and review values
remain in private local evidence; do not put them in Git, Jira or CI artifacts.

Close the application and reviewed helpers normally before using this path.
Keep the active commissioning marker and its original installed generation until
restoration completes. Use `commission-read-only.ps1 -Action InspectRestore`
with the exact prepared record/hash to capture the closed profile and its
complete tree. A pre-close copy cannot authorize restoration.

An independent review must identify that exact parent and inspection, the
input-only and current INI hashes, and every changed key's exact old/new value
(including absence). Its schema is `1`, owner
`OpenNavX.SessionPreservationReview.1`, provenance
`current-user-state;origin-unverified`, `preservationOnly: true`, and
`launchPermission: false`. Set `reviewedUtc` at actual review time. Each change
has `key`, `before`, `after`, `origin: unverified`,
`decision: preserve-current`, and a concrete `reason`. Preparation refuses a
review older than 24 hours, duplicate/missing decisions or unmatched values.
These fields are deliberate review assertions, never generated approval.

`prepare-session-preservation.ps1` accepts `-Workspace`, `-Record`,
`-ExpectedRecordSha256`, `-Inspection`, `-ExpectedInspectionSha256`,
`-PreservationReview`, and `-ExpectedReviewSha256`. It freezes the current INI,
complete profile backup, review, profile/plugin ACL inventory and lineage in a
private `session-preservation` directory. The proposed recovery baseline differs
from the exact current INI by **only the original COM8 direction byte**. The
reverse transform is checked against the existing forward contract. Preparation
changes no live profile or plugin, and its proposal cannot authorize a new
commissioning session.

The initial bounded policy accepts reviewed display/viewport values, pinned
5.12.4 build markers, the observed texture/font shapes, base vessel model/display
settings without source/pilot/bridge bindings, exact Online AIS and WMM boolean
settings, and a stored route UUID while `PersistActiveRoute` stays `0`.
A stored UUID is preserved; it is neither cleared nor treated as proof of a
currently active or inactive route. Unknown keys, source priorities, connection
changes, arbitrary chart directories, plugin settings beyond the explicit WMM
flag, and embedded vessel source/control mappings are refused. The existing
hash-bound installed basemap-default proof is the only resource-path exception.
New cases require their own bounded policy and qualification.

Restore with the original `commission-read-only.ps1 -Action Restore` arguments
plus `-PreservationProposal` and `-ExpectedPreservationSha256`. Preservation and
migration-adoption arguments are mutually exclusive. Inspection/restoration and
preservation preparation share an exclusive transaction lock. The durable
restore intent pins both proposal kind/hash and exact target; omitting or
substituting the proposal on retry cannot restore an older baseline.

Every preservation restore, including a fresh inspection after interruption,
checks the original preserved profile snapshot, allowing only the already
published one-byte target. Later navigation files, INI, plugins, ACL or identity
changes cannot be accepted merely by taking another inspection. Original plugin
files return under the existing exact inventory/ACL checks. Completion and the
new `preserved-baseline.json` lineage are verified durably before the active
marker is removed. An interruption retains ownership and resumes the same
proposal; ambiguous plugin locations or different targets are refused.

The completed record is preservation evidence, **not launch permission**. Its
historical proof remains readable after a legitimate generation change. A new
commissioning Inventory/Prepare must explicitly select that baseline and obtain
a fresh source/plugin review. The independent launch audit must still establish
input-only connections, actual route-output safety and plugin behavior; saved
Online AIS/WMM flags and retained settings do not grant those permissions.

Focused checks:

- `test-session-preservation.ps1 -PortableContracts`: disposable byte, review,
  lineage, drift and lock contracts; mocked ACL/identity boundaries are explicit.
- `test-commissioning.ps1 -PreservationFixture`: native disposable actual
  entrypoints, full backups/ACL checks, interruption and generation transition.
  Outside CI, this requires `-IsolatedLocal`. It never runs OpenCPN or hardware.
- Existing baseline-adoption and commissioning checks remain required.
