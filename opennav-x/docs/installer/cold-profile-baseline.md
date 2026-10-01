# Preserve a changed, closed boat profile for commissioning

The normal boat profile may change after an earlier completed commissioning
baseline. Its current bytes are user state. A prior baseline is recovery
evidence, not permission to replace those bytes. This separate cold-preservation
path records the current profile without launching OpenCPN, changing connections,
moving plugins, or attributing the changes to an OpenCPN startup.

The operator starts only with a completed predecessor `adopted-baseline.json` or
`completed-cold-baseline.json` and its exact SHA-256. The predecessor and current
normal profile must belong to the same interactive account. The approved stock
5.12.4 Win32 executable and any installed generation must pass the existing
identity checks; OpenCPN and known helper processes must be closed and no
commissioning transaction may be active. The new run lives under `C:\XNav\runs`
with private account/Administrators/SYSTEM access.

`tools/boat/capture-cold-baseline.ps1 -Action Capture -Workspace C:\XNav
-PredecessorRecord <completed predecessor> -ExpectedPredecessorSha256 <hash>`
copies the entire current profile and a predecessor INI into a private run. It
hashes every copied file, records the exact normal INI and ACL, verifies that
the current output-capable COM8 setting has the existing one-byte input-only
transform, and rechecks the full live profile, plugin trees, process/installation
identity and transaction marker before publishing `capture.json`. It leaves the
live profile and plugin bytes untouched. A planned or interrupted copy is not a
baseline and must be preserved for inspection.

An independent reviewer compares `predecessor.ini` with the captured
`profile-backup/opencpn.ini` privately. The review is a JSON document with
`schema: 1`, `owner: "OpenNavX.ColdProfileReview.1"`, the exact `captureSha256`,
`predecessorRecordSha256`, `beforeSha256`, `afterSha256`, `reviewedUtc`,
`provenance: "pre-existing-current-user-state;origin-unverified"`, and
`preservationOnly: true`. Its `changes` array has one exact entry per changed
key: `key`, `before`, `after`, `origin: "unverified"`, and a specific `reason`
for preserving the user state. Values and paths remain private. This review
does not assert which program or user made a change.

The current bounded policy covers only the reviewed display/viewport/AUI,
interface mode, Dashboard size/log, plugin-window position, version marker,
ownship display coordinate and Swedish menu-font shapes. It rejects changed
source/connection/chart paths, plugin enablement, route/control configuration,
missing/unknown keys, ambiguous profile syntax, invalid values and unreviewed
changes. Source review for plugin startup remains a separate requirement.

After review, run `tools/boat/capture-cold-baseline.ps1 -Action Complete
-Workspace C:\XNav -CaptureRecord <capture.json>
-ExpectedCaptureSha256 <hash> -Review <review.json>
-ExpectedReviewSha256 <hash>`. It rechecks the live profile, backup, whole
plugin inventory, stock/installation identity, ACL, closed processes and no
active transaction before copying the exact review and publishing
`completed-cold-baseline.json`. A partial review copy makes this capture
unusable; preserve it and start a fresh capture. Never overwrite or replay it.
The completed record is a hash-bound preservation lineage, not launch approval.

Use the completed record/hash as `-BaselineRecord` and
`-ExpectedBaselineSha256` for a **new** `commission-read-only.ps1 -Action
Inventory` and `-Action Prepare`. The usual independent source/binary review
of every candidate plugin, one-byte input-only transaction and short-lived
actual-profile launch attestation still apply. A later restore returns the
exact captured output-capable baseline, preserving all reviewed user bytes.
Do not launch automatically after restoration. Existing post-session baseline
adoption remains restricted to its active parent transaction.
