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
hash-bound installed basemap-default proof and the explicit WMM case below are
the only resource-path exceptions.
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

## Observed WMM save-time location

The additional explicit preservation case admits only
`Directories/WMMDataLocation` from the supported stock application's
`plugins/wmm_pi/data/` to the **current parent generation's**
`app/plugins/wmm_pi/data/`. Both values must exactly match wxFileConfig's escaped
backslashes, including the trailing separator. Old generation A to new generation
B is a distinct future case and is currently refused; this is not general
cross-update profile migration support.

Pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`
`plugins/wmm_pi/src/wmm_pi.cpp` derives this location from shared application data
in `LoadConfig`, leaves the preference read disabled, and writes the derived
location in `SaveConfig`, including during `DeInit`. The magnetic model loader
independently opens `WMM.COF` below the same shared-data directory. Preserving the
written value does not enable WMM or grant launch/output authority.

Closed inspection freezes a separate `wmmResourceProof`, including the original
ownership manifest bytes/hash, parent generation/state/commit, exact before/after
locations, and the three files `WMM.COF`, `wmm_live.svg`, and `wmm_pi.svg`. Both
stock and installed trees must contain exactly these regular files, match the
reviewed Windows checkout hashes, and match unique managed-file ownership entries.
Additional files, redirects, altered bytes, missing ownership or arbitrary paths
are refused. The proof remains historical evidence after that generation retires.

The same exact resources and original installation are checked again before
proposal publication and each live preservation restoration boundary. Each
changed INI key still needs its own explicit decision. Legacy baseline restore
cannot erase this delta, and automatic migration adoption still refuses it.
Native disposable preservation tests exercise this case with documented inert
resource-byte bindings; the portable contracts separately lock the production
resource hashes. Neither test accesses the boat.

The Windows resource hashes use the exact pinned Git source after its Windows
`core.autocrlf=true` checkout conversion. Read-only evidence confirmed both the
validated stock and installed copies are byte-identical to that conversion:

| Resource | Bytes | SHA256 |
| --- | ---: | --- |
| `WMM.COF` | 4647 | `b766a66b3438b91f01a037ab9cf24c3e48dd3bbf32b00ddc8328bf99291aa805` |
| `wmm_live.svg` | 4735 | `044064c5a0af3fc3d41fb884155f8dc3a7638b6de375af722f7862546481267f` |
| `wmm_pi.svg` | 10039 | `194f32ab7a0e257920500f67449ad244b4eaca13be6759d2cb4ed31646e0a617` |

`WMM.COF` has 93 CRLF sequences and `wmm_pi.svg` has 112; `wmm_live.svg`
has no line terminators. Runtime checks hash the original bytes directly.
They do not normalize newlines or admit the Linux LF variants.
