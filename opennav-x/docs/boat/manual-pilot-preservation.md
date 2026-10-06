# Preserve a completed manual pilot binding through parent restoration

A manual child can finish with a real COM8 translator binding saved as
`display-only`. Its rollback returns COM8 to input-only and leaves the parent's
plugin quarantine active. The ordinary parent preservation policy deliberately
refuses a newly added pilot identity. A completed child may now supply a narrow,
explicit evidence exception; it does not authorize equipment commands or launch.

After closing and rolling back the manual child, obtain a fresh parent
`InspectRestore` result and an independent per-key preservation review as usual.
When preparing preservation, additionally supply all four arguments:

```powershell
-ManualChildCompletion 'C:\XNav\runs\<selected-manual-child>\rolled-back.json' `
-ExpectedManualChildCompletionSha256 '<exact SHA256>' `
-ManualChildInspection 'C:\XNav\runs\<selected-manual-child>\inspection-<id>.json' `
-ExpectedManualChildInspectionSha256 '<exact SHA256>'
```

These are additional arguments to `prepare-session-preservation.ps1`; its
existing parent record, parent inspection and independent review hashes remain
mandatory. Select the precise child inspection used by that child's rollback.
Nothing searches for a recent child or infers approval from a directory name.
Supplying fewer than all four arguments refuses the operation.

The parent inspection must contain exactly the completed child's rollback
profile bytes. An active child, later profile edits, another parent, another
original generation, changed evidence, a non-COM8 binding, unsupported NAME,
unknown pilot fields or saved manual permission refuses preservation. The
child's prior input and final settings must match in every opaque byte after
removing the three narrowly parsed pilot scalars. Earlier settings differences
still pass the existing parent policy, and every actual parent-profile change
still needs the existing explicit independent review.

The proof binds the selected completion and inspection hashes, the prepared
child and exact parent transaction, the child input/output bytes, rollback
intent and reviewed current bytes, and the completed one-byte inverse. It
retains `display-only`; session enablement is never restored. Parent restoration
continues through the existing exact proposal and normal plugin restoration
procedure. It does not launch the application. Future commissioning needs fresh
inventory and source/launch review.

The preservation proposal records the explicit child selection. Keep that
child's entire private evidence directory alongside the existing parent and
preservation directories. Proposal rereads and future baseline lineage checks
revalidate the full selected chain. Historical validation requires the original
parent/context/generation identity to agree inside the retained evidence, but
never requires that old generation, old tools or old active marker to remain
installed. Missing or changed evidence refuses the lineage; no generic pilot
identity exception is added to ordinary settings preservation.

Focused inert coverage is in `tools/boat/test-manual-pilot-preservation.ps1`:
use `-PortableContracts` on Linux, or disposable Windows CI / explicit
`-IsolatedLocal`. These checks exercise evidence, profile policy and historical
lineage using fixture files. They do not qualify native ACLs, real processes,
ports, equipment feedback or boat operation.
