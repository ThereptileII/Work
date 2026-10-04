# SCRUM-273 — duplicate source-cache publication closeout

**Recommendation: Done for SCRUM-273's bounded source-preparation repair.**
Every issue-specific criterion below is supported. This is a documentation-only
review at `5d19314fe51d7192ea3cfc2fa55d1674c559f406`; it does not qualify the
full application, corresponding-source release, TLS, installer or boat.

[Issue SCRUM-273](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-273)
and all three comments were read (10798, 10801, 10919; pagination total 3,
`isLast=true`). Comment 10801 retained Testing pending the corrected full
candidate's normal preparation/package gates; selection 10919 authorizes an
audit and **does not waive that hold**. The hold is supported by the actual full
private-adapter preparation/package evidence at `15452e5`, described below.
The issue describes this source acquisition/private-package boundary; it does
not require terminal application recovery/installer packaging. Root must retain
Testing if it interprets 10801 as requiring that broader terminal gate. No Jira
transition was made here.

## Criterion matrix

| Criterion | Evidence and finding | Verdict |
| --- | --- | --- |
| Preserve original failure and identify the mechanism | [Original audit](../scrum273-native-9ec-cache-failure/README.md), report and unchanged first-failure log retain run **37135421013**, remote **9ec668981b9aa86bcd643b84c6f18cfa002b3f74**, local **9f72d0a797868d84efd9441022220a9b2ca52918**. `temporary.replace(cached)` raised WinError 5 for `7d5393a1212da0841e28e2f74883cb40fbd75a1f`. Both `COPYING` and `COPYING.gplv2` expect that 15,170-byte blob; the unchanged lock has 14 duplicate identities. The old eight-worker per-path fetcher allowed competing publishers. The exact OS interleaving was not traced. | Pass |
| Deterministic duplicate-key regression and preserved concurrency | The exact six-case test script is bound to successful native run **37135967220** below. Forced cache misses plus a second-publication refusal require exactly one download/publication for three destinations, including a gitlink path; all destination/cache bytes must match. Distinct identities rendezvous at a two-worker barrier. The [retained old-code negative control](../scrum273-source-cache/original-negative-control.log) fails with the intended duplicate-publication PermissionError. | Pass |
| Tamper and identity rejection; errors remain visible | Cases 3–6 below reject corrupted cache/network bytes, enforce each duplicate destination's own expected length/hash, and propagate publication PermissionError. `fetch_sources` validates every grouped expectation before publishing or writing any destination in that group. No permission suppression, unverified cache fallback or global serialization was added. | Pass |
| Native short preflight proceeds beyond source acquisition | [Completed native audit](../scrum272-native-714-compile/README.md): remote **71470d07d8dd9ac6a3050e2b64093e0892473c97**, local **1c4d817c373b85838bc7c1b5d97d11bd026b3454**, job **111240444202**. Original report says `passed=true`, inventories 219 original/219 patched private files, and records ten x86 objects from corrected core/private configurations. The preflight ran the six cache cases before acquisition; the actual [log](../scrum272-native-714-compile/source-cache-tests.log) says `Ran 6 tests … OK`. | Pass |
| Comment 10801: corrected full candidate's normal preparation/package gates | [Full 15452e5 audit](../scrum272-native-154-expired-fixture/audit.json) records the actual private package passing frozen verification against production resources. Retained transcripts and first-build receipt bind normal preparation, actual DLL/source package creation and installation on both fixture/production passes to **96c0c270 → 15452e5**, run **37136712793**. The cache/verification functions are unchanged across this source and the proven/current revisions. Actual package/source/resource records were rechecked below. | Pass for the affected private preparation/package boundary; terminal application packaging remains unproven |
| Mandatory package/source closure remains | Source and caller inspection below confirms exact inventories, per-blob verification, patch rederivation, same-job dependency identity, package verification and installed corresponding-source inclusion remain mandatory. No guards or assertions were weakened by the cache repair. | Pass — preservation, not current package acceptance |

## Six recorded cases

These are the six test methods in the unchanged
[`tools/test-ocharts-source-cache.py`](../../../tools/test-ocharts-source-cache.py),
whose exact Windows checkout bytes match the successful native report. The
native log uses six dots rather than named verbose output; the bound script
identifies the cases. They were not rerun for this review.

| Test method | Required result recorded by the six-case pass |
| --- | --- |
| `test_duplicate_paths_have_one_cache_publication` | One download and publication, three exact destinations, one exact cache entry. |
| `test_distinct_blobs_remain_parallel` | Independent downloads cross a two-worker barrier and produce both expected files. |
| `test_tampered_cache_is_rejected_without_download_or_overwrite` | ValueError; no download, no destination, corrupt cache left untouched. |
| `test_network_tampering_is_rejected_before_publication` | ValueError; no destination or cache publication. |
| `test_every_duplicate_path_expectation_is_checked` | Conflicting duplicate-path length rejected before destination/cache publication. |
| `test_publication_permission_error_is_not_hidden` | PermissionError propagates; no destination is written. |

## Binding to the frozen/current source

[source-binding.json](source-binding.json) records read-only comparisons between
proven local `1c4d817…`, full-preparation local
`96c0c2705aacae511b6f8c22afaa6618dbabceff`, frozen
`0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4`, and review base `5d19314…`.
`safe_path`, `blob_ok`, `fetch_sources`,
`apply_patches`, `verify_prepared`, `verify_producer_dependencies`, `prepare`
and `package_dll` have identical function text. The complete cache test, source
lock, package verifier, native-preflight caller, package-preview caller and
private CMake configure gate are also unchanged. The preparation script's only
differences from the proven revision expand `PATCHES`/`LOCAL` for later chart
work; their current contents still participate in `INPUTS`, receipt comparison
and source packaging. This does not transfer native acceptance to those changes.

The retained native ZIP was rehashed: **536,550 bytes**, SHA-256
`49c8ddee500f6f881099010dac432a24eff5f492224a3a8a0e3f987624174572`;
all **171** entries passed CRC inspection. Its report and cache-test log match
the committed evidence byte-for-byte. The report's relevant source-input
records match the proven Git files after exact LF-to-CRLF checkout expansion.
The earlier full 61-input/219-source/object audit remains linked above; this
closeout rechecks the cache boundary rather than repeating that entire audit.

## Actual full-candidate preparation/package hold

The [retained full-run audit](../scrum272-native-154-expired-fixture/README.md)
is an application-job failure, not a pass of terminal packaging. Its unrelated
expired-certificate fixture failure occurred after the affected private source
preparation/package steps. [Original transcript excerpts](full-candidate-transcript-excerpts.log)
record exact preparation at XNav line 17743, DLL creation at 17967, and DLL plus
corresponding-source installation at 21200/21202. The production transcript
records the same installation at 3681/3683, after the mandatory reuse checks in
the frozen `Build-PrivateOCharts` caller. Both passes reached 139/139 tests.

[full-candidate-package-binding.json](full-candidate-package-binding.json)
retains the original first-build receipt and read-only comparisons. That receipt
is emitted only after `verify_prepared`, same-job dependency comparisons and
`verify-ocharts-adapter-package.py` succeed. It binds run **37136712793**,
attempt **1**, `windows-integration`, remote
**15452e512fd073090b1a4cea7c9010eac0874118**, and the exact frozen build script.
Its records match the actual manifest, **1,328,640-byte** private DLL (SHA-256
`9af282c0e24ccc6ce91e314eb710d915331e932e9a597cb91957089023eb8e41`),
and **5,632,547-byte** corresponding-source ZIP (SHA-256
`99ee27c9b70129cc12ac0b24ab676bad29f57b0f01a1d6509e0f0537384f622a`).
All 40 archived product inputs match frozen source; all 219 originals match
locked Git blob identities/lengths. The source ZIP passes CRC inspection.
The package's chart manifest/header match actual production resources, whose
five recorded resource files also match. The retained outer artifact matches
its previously audited SHA-256, and the production transcript matches the
committed gzip byte-for-byte. The prior independent audit explicitly records
the frozen static package verifier pass; that verifier was not rerun here.

This supplies actual successful private preparation/package evidence for 10801,
not merely unchanged guard code. The preparation receipt's own bytes are not
archived; its identity is recorded in the retained first-build receipt. Terminal
product recovery/installer packaging, real-host module loading, TLS acceptance
and boat qualification remain unproven; none is inferred from the static package.

Mandatory caller chain, inspected at the review base:

- [`prepare-ocharts-adapter.py`](../../../tools/prepare-ocharts-adapter.py):
  `prepare` verifies producer dependencies and finishes with `verify_prepared`;
  that verifier checks the closed inventory, input receipts, every original
  blob, patch rederivation and overlays. `package_dll` verifies again and includes
  original license blobs plus current recipes/patches in corresponding source.
- [`build-pristine-windows.ps1`](../../../tools/build-pristine-windows.ps1):
  the unchanged `Build-PrivateOCharts` body checks complete same-job identity,
  verifies prepared inputs on both first/reuse passes, compares dependency
  outputs, and calls the package verifier. The
  [private CMake entry](../../../cmake/ocharts-adapter/CMakeLists.txt) also makes
  prepared-input verification fatal before target configuration.
- [`verify-ocharts-adapter-package.py`](../../../tools/verify-ocharts-adapter-package.py):
  exact source/binding/input identity, dependency receipts, resources, DLL/PE
  identity, ZIP inventory/CRC, every product input and every upstream blob are
  checked. `installed_source_bundle` verifies the installed DLL/source/runtime
  and compiled trust header; the unchanged
  [`package-preview.py`](../../../tools/package-preview.py) calls it and includes
  its verified source bundle in the distribution's corresponding-source archive.

The guarantee remains **one publisher per locked identity within one fetch
invocation**. Unrelated processes sharing a cache are outside this repair.
The short native report explicitly leaves package/runtime/TLS acceptance false.
Frozen local `0a52a6c…` → remote `17ab044a1e5222dc71791ac8118454219efe8734`,
run **37164360050**, remains the separate in-progress candidate at this review's
selection snapshot. No new test, build, CI poll, code change, boat action or
release qualification was performed.
