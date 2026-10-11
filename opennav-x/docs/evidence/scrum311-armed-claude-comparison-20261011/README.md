# SCRUM-311 independent Claude / Codex comparison — 2026-10-11

The user requested an independent Claude investigation and comparison. The
restricted review wrapper completed a read-only **Opus 5** parent review of 17
explicitly selected source, test and contract files. Model metadata was verified;
selected files did not change after capture. Their exact hashes are retained in
[source-receipt.json](source-receipt.json). No credentials, boat data or private
transcripts are included here.

This was a source review of proposed installed armed-session denial changes,
not a full application qualification or a complete signed-update broker. The
snapshot combined committed Go `a95208a9d2e57a1d75e752e11ddd434146675a63`,
PowerShell `3ed41d9b2483b80b7cd91ffd49f60767d1173be8`, and the root agent's
uncommitted store/test/contract bytes. The parent ended near its budget ceiling
before either requested child specialist returned. No independent child review
result is claimed. Codex compared the report with the full source and final
PowerShell amendment `e1ac0bedcdd46ad73f728f05e8cd5c645633e395`.

## Claude findings and Codex dispositions

| Reported concern | Verified disposition |
| --- | --- |
| AllowMissingRoot on Update permits existing armed installation mutation | Not established. Existing present ancestors reach the fixed marker lookup and presence throws before mutation. Update already supports fresh-root initialization; a genuinely missing root must pass strict absence checks after creation, before and under transaction.lock, before owner.json and recovery. The permissive fresh-root check is not an existing-root authorization. Native behavior remains pending. |
| Go tests do not cover native marker semantics | Snapshot limitation, not missing test implementation. `armed_barrier_windows_test.go` already tests exact FILE_NOT_FOUND, other error identities, real absent/present leaf types, missing parents and requested managed roots. It was not included in Claude's 17-file scope. Cross-compilation is not native execution; execution remains pending. |
| An ambiguous lookup permanently bricks startup/maintenance | Overstated. Ambiguity denies while it persists; resolving a transient lookup failure allows checks again if the marker is absent. A retained armed marker intentionally has no automatic disarm/recovery authority. The four-stage broker and authenticated recovery remain explicitly unimplemented, so no complete updater/recovery acceptance is claimed. |
| localPath might emit backslashes and remove final creator guard | Rejected by direct source: `records.go` localPath replaces backslashes with forward slashes before deriving layout. The raw arbitrary-directory creator seam remains deliberately separate from installed launcher entry. |
| Bootstrap/trust/progress might lack creation-boundary checks | Scope limitation, not an established defect. Their defining files were omitted. Full source includes guarded bootstrap/trust paths and progress behavior; entry and existing mutation checks must still pass. No broad all-entry native qualification follows from review. |
| Diagnostics report could alias into an armed root despite textual prefix exclusion | The final PowerShell amendment removes the prefix exception and requires proven absence before every report/summary write. This was independently corrected before Claude's report; armed diagnostics remain console-only. No new duplicate patch required. |

## Additional Codex finding: held image versus mutable drive alias

A separate read-only Codex trace found a real identity mismatch not identified
by Claude within its budget. Go's ordinary image owner can retain executable A
and a compatible physical application spelling after the original requested
DOS/SUBST alias changes to B. The final wrapper still queries that original
lexical installation root. It can therefore accept B's absent marker while
preparing/resuming held image A, whose root marker is present. This is not the
same as ordinary arming after a final valid check. Earlier entry checks still
refuse a marker visible through the alias, and the cooperative same-root
transaction lock prevents normal store arming during ordinary startup custody.
No complete stationary-marker-at-entry exploit or native reproduction is claimed.

PowerShell's per-call physical drive capture closes an additional within-call
alias remap gap, but does not by itself bind a later call to an already-held image.
The required targeted Go correction is to derive and validate the final marker
root from the held native image and exact managed-generation suffix, under its
existing ancestor custody, rather than recapture the possibly changed alias.
Implementation and focused regression coverage were delegated in the same
isolated Go worktree. This finding remains open until that correction and its
required native qualification pass.

## Verification boundary

The comparison used source/control-flow analysis and existing focused agent
results. It did not run a full application build, native Windows acceptance,
boat deployment, design review, endurance suite or physical actuator command.
No Production promotion, public publication, signed broker enablement or Jira
Done transition is authorized by this record. The root agent owns integration
and the combined build decision.
