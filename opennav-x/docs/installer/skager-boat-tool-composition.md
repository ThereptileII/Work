# Coherent SKAGER boat review tooling — SCRUM-258

The application candidate at local `78eccb8b7f21b260ded57d3ba763f884d60c8180`
has current SKAGER window/menu selectors but lacks the completed cold-baseline
reader already qualified and used on the boat. The preserved boat checkout
`20765cf374da5a1ab47dff0b3b427cd24f595455` supports that baseline but expects old
OpenNav captions. Neither source alone qualifies the combined review sequence.

This tools-only composition restores nine production files and four existing
tests byte-for-byte from its exact local mapping
`7658f36c50140c35a44678b75675ad9b4e77e526`. Prior native run 36919418416 passed all
ten tooling jobs; downloaded artifact 11191068356 has SHA-256
`d031f870b1968247a44f2888a2cb21b74a0ba9d7867d449c14ae66c64359f684`.
That evidence establishes provenance, not a new composed-source native pass.

Restored production closure:

- `ColdBaseline.ps1`, `capture-cold-baseline.ps1`, and the completed-cold reader in
  `CommissioningBaseline.ps1` preserve exact full-profile bytes, predecessor,
  explicit review, identity, metadata and private ACL checks without launch permission.
- `RestartCommissioning.ps1` pins `ColdBaseline.ps1` in each new session's exact
  dependency inventory. Existing sessions are never migrated or rebound.
- `ColdChartHelper.ps1`, `recover-cold-chart-helpers.ps1`, `ChartHelperShutdown.ps1`,
  `stop-chart-helper.ps1`, and `stop-orphan-chart-helper.ps1` retain the shared
  per-process one-attempt ledger, exact orphan identity and normal fixed transport.
  Both older stop paths reserve the global attempt before their legacy locator.

Current SKAGER review selectors, policies, tests and dual historical/current
portable-archive handling remain unchanged. The old `test-boat-tools.ps1` is not
copied: it would remove current mixed-root and case-root refusal coverage.
No application code, storage identity, frozen build cache or boat file changes.
The exact accepted/current source closure is recorded in
[the evidence summary](../evidence/scrum258-boat-tool-composition/summary.json).

## Focused evidence and required native gate

Ten existing portable suites passed 945 checks/groups: cold-baseline 36,
cold-helper-policy 21, cold-helper-workflow 33, chart-helper-attempts 20,
baseline-adoption 127, commissioning 12, restart-commissioning 309,
broker-fixture-contracts 24, review-window 230 and restart-window-review 133.
Individual reports retain their substitution and no-boat limits. The broker
fixture additionally copies/hashes the cold reader and rejects its missing
transitive module. Current selectors compile without invoking native windows.

The existing actual native Prepare/Arm fixture now requires the fresh session's
exact cold-reader hash. Before starting its inert marker parent, it exercises
actual Arm refusals for changed cold-reader bytes, an omitted old dependency
inventory and a different tool directory. Each must fail with the specific
session-identity error and no Arm intent. It restores exact fixture bytes before
the existing successful Arm/Collect path. This fixture parses and passed
independent source review; actual Windows execution remains pending.

Publish only the composed tooling branch `skager-boat-review-composition` to run
`.github/workflows/opennav-boat-tools.yml`. Its native maintenance job includes
the four restored suites. Existing native restart transport, broker/Prepare/Arm
and guarded window jobs qualify the complete session and selector boundary;
the existing shortcut, display, stock-window and source-verification jobs remain
intact. The workflow builds only small marker/helper fixtures, never OpenCPN or
its dependency stack. Retain all downloaded artifact hashes and exact source/run
identity. Maintenance evidence is `development-boat-tooling-<sha>`; native
Prepare/Arm evidence is inside `development-commissioning-broker-<sha>`.

## Conditional deployment and review sequence

All paths and variables below come from verified private records. This document
does not authorize execution, invent an audit or substitute a current filename
for a package hash.

1. Verify the exact candidate's downloaded Setup/recovery/source bundle hashes,
   native package/installer/display/chart gates and same-run restart attestation.
   `PRODUCT_BUILD.json` must identify the fixture-free installed product, disabled
   hardware output, exact executable/helper hashes and restart protocol 1.
   A pending-endurance artifact remains development review only.
2. Preserve the existing qualified boat checkout. After this composed revision's
   native gate, use its accepted archive/manifest with existing
   `stage-review-tools.ps1 -Workspace $W -Archive $Zip -ArchiveSha256 $ZipSha
   -Manifest $Manifest -ManifestSha256 $ManifestSha -Commit $ToolCommit`.
   Verify returned staging identity. Use one coherent staged directory for the
   whole new session; never change pinned files or toolDirectory afterward.
3. Use preserved qualified `update.ps1 -Workspace $W -Setup $Setup -Sha256
   $AcceptedSetupSha -ExpectedCommit $Candidate`. The supported stock executable,
   closed processes, absent active transaction, cold recovery, Setup bytes,
   installer result and installed ownership must pass. This does not launch.
4. From the composed tools, run `commission-read-only.ps1 -Action Inventory
   -Workspace $W -BaselineRecord $ColdRecord -ExpectedBaselineSha256 $ColdSha`.
   The already completed baseline SHA is
   `543f63c4583140917208a54dac2419540bc418b43143e74a939c618da29d1470`;
   its path remains private. Do not use the older default baseline from historical
   documentation. Review the final installation's complete plugin/helper trees.
   `Prepare` requires the same baseline selection plus `-Plan/-ExpectedPlanSha256`;
   `Apply` requires the returned `-Record/-ExpectedRecordSha256`.
5. Independently issue a fresh exact installed-build/profile/plugin/output audit
   and retained-plugin shutdown review. Use `RestartCommissioningPrepare.ps1
   -Workspace $W -ShutdownReview $Review -ShutdownReviewSha256 $ReviewSha`, then
   `run-mode.ps1 -Workspace $W -Mode XNav -RestartSessionRecord $Session
   -RestartSessionSha256 $SessionSha`. Retain actual launch request/result and
   native process creation identity. These operations repeat the existing guards.
6. Use `review-window.ps1` with the exact `-LaunchResult/-ExpectedLaunchSha256`,
   `-ExpectedCommit` and one fixed action per call. Inspect real chart content,
   Day/Dusk/Night, measured bounds/DPI and honest unavailable data. For mode
   transitions, Arm one exact parent/mode, wait for listening readiness, and use
   `review-restart-window.ps1 -Action RequestMode` with exact session, parent,
   Arm and readiness hashes. Collect the same completed Arm; use `-Action
   ReviewChild -CompletionFile/-ExpectedCompletionSha256 -ReviewAction Capture`
   for the proven successor. Review SKAGER → Legacy → SKAGER → Safe → SKAGER.
   Legacy/Safe support Capture/Resize only. No ordinary close counts as an
   in-application restart, and no refusal is bypassed by renewing hashes.
7. Close normally with `stop.ps1 -Workspace $W -ProcessId $VerifiedPid`; verify
   application/helper closure and collect owned completed broker tasks. Run
   `InspectRestore`, review its private exact diff, then `Restore` with prepared
   and inspection record hashes plus `-ReviewedCurrentIniSha256`. Verify preserved
   profile/plugin/recovery identities and inactive commissioning. Restoration
   restores original outputs and grants no automatic launch.
8. Only after replacement charts/modes/recovery pass may exact identified old
   portable/download copies be retired through `retire-portable.ps1` or
   `retire-download.ps1` with accepted ownership hashes. Recheck external shortcut
   ownership. Preserve all installed recovery generations, stock application,
   navigation data, credentials and remote access. Unknown material remains.

Native qualification, downloaded proof, hash-verified staging and actual fresh
commissioning are still open; the portable result does not close SCRUM-258 or
qualify an application release or physical boat operation.
