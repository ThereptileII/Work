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
control configuration are refused, except for the exact source-discovery append
and obsolete font removal described below. Normal defaults outside this policy require
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

### Subsequent pinned startup texture minimum

After the boat's September 28 restart, the cold profile preserves navigation,
charts, source configuration and plugin state. Its display changes include the
reverse texture-budget transition, `64` to `128`. Pinned
`navutil.cpp::MyConfig::LoadMyConfig` calls `LoadMyConfigRaw` and then applies
`wxMax(128, m_iTextureMemorySize)` when GL expert mode is false (lines 554–560).
`UpdateSettings` writes that value at line 2168. The earlier upgrade's 64MB
selection is therefore not the persistent normal-start minimum.

The adoption policy admits only the observed 64→128 value with unchanged
`OpenGL=1`, unchanged false/default expert mode and unchanged pinned
`Version 5.12.4+37fd0cd Build 2026-09-27` marker. It still requires an independently
reviewed exact per-key record and closed-session hashes. Other budgets, expert
mode changes, GL changes and different version markers are refused. Thirteen
additional portable/native contract cases cover this boundary and preservation
of the normalized profile through the one-byte connection restoration. This
changes commissioning tools only; application and upstream code are unchanged.

The official executable `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`
contains `5.12.4-0+37fd0cd` and `2025-09-12`; its permitted marker is exactly
`Version 5.12.4-0+37fd0cd Build 2025-09-12`. Pinned Windows product builds use
`Version 5.12.4+37fd0cd Build YYYY-MM-DD`, with a valid, nonfuture date. These
forms follow the pinned CMake version generation; a version string alone does
not authorize an executable or installation.

### Observed variation-source discovery at normal close

The private post-exit INI with SHA-256
`5f5f9c4e2c09e9e45d74f78b3b338289e37d8168338f0d59a16c5dd3472bebe4`
adds one entry to `Settings/CommPriority/PriorityVariation`. The prepared
input-only INI (`8fa87a4550a645521f0833155371f497c4631577d354466d41808f7fea7fe0fe`)
has five entries in this exact order: `nmea2000 COM8:105;127250`, then N2K
addresses 243, 35, 33 and 49 for PGN 127250. All five entries, their spacing and
their final separators remain unchanged. The only permitted addition is the
exact sixth token `N2k device address: 204 ; PGN: 127250|`.

Pinned `model/src/comm_bridge.cpp` explains this write:

- `HandleN2K_127250` (line 675) calls `EvalPriority` for a successfully decoded,
  non-unavailable variation value.
- `GetPriorityKey` (line 1230) formats the N2K payload's source-address byte and
  PGN using the observed spelling.
- `EvalPriority` (line 1269) appends an unseen source at `map.size()` **before**
  deciding whether its priority permits updating the active value.
- `GetPriorityMap` (line 475) serializes priority order with `|` separators;
  `gui/src/ocpn_frame.cpp` (line 1910) saves the maps during normal close.

The reviewed action history contains only warning acknowledgement, resizing,
capture and normal close, with no priority editor or plugin API action. This
supports automatic source discovery as the explanation. The saved token does
not prove the source became active, establish the device's identity, or validate
its data. The priority editor and plugin API can also write priority maps, so
the INI difference alone cannot prove how it arose. A new fallback source is
navigation configuration, not a visual setting.

Adoption therefore accepts only this exact five-to-six-entry transition and
still requires the independent per-key review and whole-file hash bindings.
Raw values are checked without whitespace normalization. Reordering, replacing
or removing existing entries, another address/PGN, multiple appends, duplicate
records and changed connection, route, plugin or other priority settings remain
refused. Normal armed restart continues to reject this change; it cannot grant
itself an adoption review. After explicit adoption, the preserved six-entry map
can pass the existing unchanged-configuration checks.

Restoration reverses only the temporary COM8 direction byte and preserves the
reviewed discovery along with the other accepted core writes. These tests do
not complete the boat transaction: all application/helper processes must be
closed and the fresh whole-profile/plugin/ACL inspection must pass first.

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
