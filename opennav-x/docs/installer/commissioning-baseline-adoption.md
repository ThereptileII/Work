# Preserving reviewed startup migration between commissioning sessions

A stock 5.12.4 first start can legitimately update the older profile's version,
notice and display persistence. Returning the temporary COM8 connection and
quarantined plugins must not silently discard those reviewed changes.

The default commissioning baseline remains the separately recovered, 21,380-byte
INI with SHA-256 `a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9`.
Existing transactions retain that exact recovery target. A different baseline
requires an explicit, completed adoption record; neither a current file nor an
unfinished proposal substitutes for the original pin.

## Review and prepare

1. Close the exact reviewed application normally. Keep its commissioning
   transaction active. Do not restore it or start another application yet.
2. Run the existing `commission-read-only.ps1 -Action InspectRestore` with the
   prepared record and hash. Inspect its private saved INI, complete profile
   inventory and exact before/after differences.
3. Independently write a private migration-review JSON containing one entry for
   **every** changed key relative to the prepared `input-only.ini`. Bind the
   exact prepared record, inspection and both INI hashes. The tool does not
   generate approvals from observed changes.

The review schema is:

```json
{
  "schema": 1,
  "owner": "OpenNavX.ProfileMigrationReview.1",
  "parentPreparedSha256": "<prepared.json hash>",
  "inspectionSha256": "<inspection hash>",
  "beforeSha256": "<prepared input-only.ini hash>",
  "afterSha256": "<inspected closed-session INI hash>",
  "reviewedUtc": "<UTC review time>",
  "changes": [
    {
      "key": "Settings/ConfigVersionString",
      "before": "<exact previous value, or null if absent>",
      "after": "<exact observed value>",
      "sourceRevision": "37fd0cddb7334fe489e9f18aa163977a9c5c84f7",
      "sourceBoundary": "<inspected source file/function>",
      "reason": "<why this exact normal write is accepted>"
    }
  ]
}
```

The closed policy admits the existing bounded display/viewport rules, an exact
qualified version marker, the acknowledged navigation notice, explicitly
reviewed locale persistence, and the already reviewed AUI/Dashboard deltas.
Locale is **never changed automatically** or converted to English for testing.
Any locale delta needs its own exact per-key review. Missing/removal, unknown
keys, source/connection changes, chart-path changes, plugin enablement and
control configuration are refused, except for the one source-proven obsolete
font removal described below. Normal defaults outside this policy require
a separate source-specific implementation and tests before adoption.

### Observed official startup normalization

A preliminary **live** boat INI copy identified three representation/migration
cases. It is evidence for these narrow policy rules, not a closed-session
inspection or permission to adopt the current file:

- Quoted coordinate pairs are accepted only for existing latitude/longitude
  display keys. Pinned `navutil.cpp` formats these with `%10.4f` padding, and
  `wxFileConfig` quotes the string. One balanced outer quote pair is decoded for
  the existing finite/range checks; stored bytes are preserved. Quotes in source,
  path or connection values are not normalized.
- `Settings/GPUTextureMemSize` may change from exactly `128` to `64` only across
  the observed official 5.12.2/2025-08-01 to 5.12.4/2025-09-12 version markers,
  with `Settings/OpenGL=1` preserved. Pinned
  `OCPNPlatform.cpp::Initialize_3` lines 634–646 explicitly assigns this texture
  budget during the GL-capable upgrade path. This setting is not proof of a
  rendered chart or current graphics backend.
- Only the exact obsolete `Settings/MSWFonts/sv-00c6075a` English `Menu` record
  may be removed, preserving `Locale=sv`, `LocaleOverride=sv_SE` and both existing
  translated Swedish menu records byte-for-byte. Pinned
  `FontMgr.cpp::ScrubList` removes current-locale descriptions absent from the
  translated candidate list; the Swedish catalogue translates `Menu` to `Meny`.
  `navutil.cpp` lines 2543–2553 rewrites the surviving font list. Other font
  removals, replacement edits and different obsolete values remain refused.

Each case still needs its exact per-key source review bound to a fresh normal
**post-close** inspection. Tests cover malformed quotes/out-of-range coordinates,
incorrect version/budget/GL state, changed locale, altered/missing translated
fonts, unrelated deletion and omission of the explicit removal approval.
The removal review must contain an explicit JSON `null` for the new value;
omitting that field or retaining the font key with an empty value is refused.

The official executable `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`
contains `5.12.4-0+37fd0cd` and `2025-09-12`; its permitted marker is exactly
`Version 5.12.4-0+37fd0cd Build 2025-09-12`. Pinned Windows product builds use
`Version 5.12.4+37fd0cd Build YYYY-MM-DD`, with a valid, nonfuture date. These
forms follow the pinned CMake version generation; a version string alone does
not authorize an executable or installation.

Prepare the private proposal using `prepare-baseline-adoption.ps1` with
`-Workspace`, `-Record`, `-ExpectedRecordSha256`, `-Inspection`,
`-ExpectedInspectionSha256`, `-MigrationReview` and `-ExpectedReviewSha256`.
This repeats the cold process, account, installation, whole plugin-tree,
profile-inventory and ACL checks. It copies the reviewed evidence privately and
builds a proposed baseline by reversing only the existing COM8 direction byte
from `0` to `1`. All other bytes, including encoding, line endings and migrated
values, remain identical. The exact inverse is checked against the existing
forward input-only transformation. Preparation changes no live file.

## Restore with explicit adoption

Use the original transaction and inspected-current-byte arguments for
`commission-read-only.ps1 -Action Restore`, adding:

```powershell
-AdoptionProposal <private-adoption-directory/proposal.json> `
-ExpectedAdoptionSha256 <exact proposal hash>
```

The existing durable replacement primitive preserves the profile's native file
metadata. Only after the exact target profile and complete original plugin
inventory are verified does restoration publish `adopted-baseline.json`, then
remove the active marker. Its immutable lineage binds the original prepared
transaction, reviewed saved bytes, explicit review, proposal and completed
restoration. The original files and original recovery baseline remain intact.

Restoration returns the original output-enabled connection setting and plugin
files; it **never launches OpenCPN**. Prepare a fresh read-only transaction before
any subsequent commissioned launch.

If interrupted, retain the active marker and journals. Run a fresh
`InspectRestore` with the **same** adoption proposal/hash, review its exact
current bytes, then retry Restore with those new inspection arguments and the
same proposal. Restoration intents lock the target: omitting or substituting the
proposal cannot reset the migrated profile. Unexpected files, modified evidence,
changed ACLs or changed profile bytes refuse recovery instead of being erased.

## Rebind the next generation

Pass the completed `adopted-baseline.json` and its hash as `-BaselineRecord` and
`-ExpectedBaselineSha256` to both new `Inventory` and `Prepare` operations.
Inventory must cover the actual current stock/installed/managed roots. Write a
fresh source-review plan bound to that new inventory; the previous generation's
plan is not reused automatically. Apply and the normal launch audit retain all
existing safeguards, including exact input-only bytes and plugin quarantine.

The adoption record is configuration lineage, **not launch permission**. It
cannot approve plugins, renew a source review, arm a restart broker or authorize
hardware output. The same user/profile identity is required across lineage,
and lineage depth is bounded.

## Validation

The portable suite exercises exact byte reversal, Swedish locale preservation,
closed per-key review, rejected source/output/chart changes, incomplete or
changed lineage, and immutable restoration targets. Existing cold launch,
installed runtime and stock-review policy suites remain required.

On disposable Windows, `test-baseline-adoption.ps1` also runs the actual
commissioning entrypoints with `-AdoptionFixture`: preparation without mutation,
failure immediately after atomic adopted-INI publication, refusal to change the
durable target, inspected recovery, exact DLL restoration, and a subsequent
fresh Inventory/Prepare/Apply/Restore using the adopted profile. Both Windows
maintenance workflows require this suite. Native transaction qualification and
actual migrated boat-profile acceptance remain pending until their own evidence
is recorded; no application or physical equipment is used by these tests.
