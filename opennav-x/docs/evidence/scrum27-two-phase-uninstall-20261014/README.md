# Two-phase protected uninstall — native component evidence

Exact joined source `d7be409dc5c3abc31b82315510891adf53de4486` passes
[focused run 38512820510](https://github.com/ThereptileII/Work/actions/runs/38512820510),
attempt 1. Linux: **104 top-level / 298 nested** cases pass, no skips/failures.
Windows Server 2022, GOARCH=386: **139 top-level / 356 nested** cases pass, no
failures. Three existing non-Windows-boundary cases intentionally skip on Windows;
none of the eight specifically required new native controls skipped.

Both original artifact ZIPs were downloaded and checked against the provider's
exact length and SHA-256. Every member passes CRC validation. `results.json`
records their identities, counts, skipped names and the eight explicit native
PASS requirements. The unchanged workflow already selects these names; a separate
required-name workflow edit remains unpublished because the current OAuth
credential lacks workflow-edit scope. No existing gate or assertion was weakened.
The exact broker source-closure gate also passes on both jobs. Cross-compilation
and local tests are supplemental, not the source of native acceptance.

The former failure [38465784927](https://github.com/ThereptileII/Work/actions/runs/38465784927)
at `8a706e80` remains a product failure. Windows forbids renaming an ancestor
while app/launcher descendant handles are open, regardless of delete sharing.
The new positive native test demonstrates that original shortcut restoration and
child-handle drainage precede the move, with the original root and broker DELETE
owner retained. Refusal cases preserve original identity, partial restoration,
changed location, expired/cancelled ownership, failed drainage and no-SCM effects.
Protocol cases additionally check expected-input arming, borrowed-witness
revocation, no capability-mutex re-entry and refusal of the old single-phase API.

Independent source review found no remaining concrete pipe/operator issue. A
final witness transfer deadline check was corrected before integration. The
protected action remains bounded to 90 seconds, with a 30-second intermediate
exchange; failed post-fence work is terminal and cannot retry or fall back.

This is **component evidence**. It does not establish full installed uninstall,
terminal cleanup, updated/pending-generation lineage, repair/update/rollback,
new OpenCPN package qualification, physical boat behavior or public launch.
No application was rebuilt or deployed and no boat state was changed by this gate.
