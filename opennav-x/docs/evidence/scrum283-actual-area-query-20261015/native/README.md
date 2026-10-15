# SCRUM-283 actual area query: native x86 evidence

**Passed** on exact harness `2f9c41cbd3699f8d1bb7b0aee60650a74e558a29`, implementing `f70c0eb8a488c3d0ff337f608325d51fccf95755`. [Original run 38740153247, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38740153247), [job 116439435152](https://github.com/ThereptileII/Work/actions/runs/38740153247/job/116439435152), completed 2026-10-15. The selected query-only branch did not rerun the already-green 288-exposure fixture and did not build the application.

| Native variant | Machine | Compile/run | Actual query cases | Checks |
| --- | --- | --- | --- | --- |
| Original private | I386 | 0 / 0 | 29 | 113 |
| Repaired private | I386 | 0 / 0 | 29 | 125 |
| Polygon-bypass negative | I386 | 0 / 1 expected | Rejected at inside-bounds/outside-polygon case | Negative |

The original missing-callback `require-repair` run also exits 1 as required. All 29 direct-query outputs and both actual query bodies agree between original/repaired variants. The repaired fixture connects the actual query through the private callback to `_UDWHAZ03`, isolated-danger output and DisplayBase promotion. Synchronous borrowed object/list lifetime and removed/restored area behavior pass.

Artifact **11975036987**, 184,836 bytes, SHA-256 **`7f856f42d0fa686b6af77bc10a9919db42893a0fca779a3ba4323e09547357a3`** was downloaded once after authenticating run/attempt/job/head and artifact metadata through GitHub. Download size, API digest, ZIP CRC, unique safe member paths and file types were verified. The original archive, API responses and job log remain private; this directory contains only sanitized receipts, hashes and test logs. `receipt.json` retains their exact identities and the 13 original artifact-member hashes.

Windows Server 2022 image `20261012.333.1` used MSVC `14.44.35207`, Hostx86/x86. Each produced fixture PE was checked as I386 by the exact helper. The existing workflow authenticated the three locked wx 3.2.8 archives before extraction; helper and workflow inventories agree on **1,612 files**, and both before/after checks passed. This establishes the consumed wx fixture boundary, not qualification of the separate maintained TLS SDK. No SDK payload or fixture executable is included here.

`input-binding.json` independently maps all 11 native harness hashes and 10 pinned core hashes to the exact Git bytes, explicitly accounting for Windows CRLF checkout. Three patch identities match exactly. All 219 original private-source files match their locked Git blob identities and all 219 derived files match the exact local production-patched source. Every extracted-method hash agrees with the earlier exact-commit Linux fixture, and every native staged-input hash matches those retained fixture bytes or explicit CRLF expansion. `source-inputs.json` preserves the original native per-file hash inventory. `fixture-identities.json` preserves per-variant staged-input, method and executable hashes while omitting runner-specific compile paths. No vendor source bodies, chart data or binaries are copied into this evidence.

The source methods are real; the minimal chart class, object construction/attributes, context wiring, mariner setting and explicit pre-tessellated geometry are fixtures. The inherited first-match, plain-before-symbolized and LINE/AREA reference-point semantics are preserved. This does not establish arbitrary line/polygon intersection, tessellation generation, chart ingestion, whole private DLL ABI/lifecycle, software/GL rendering, private real-chart parity or boat/installed-product acceptance. Those gates remain open. The downloaded evidence was audited without re-executing tests or binaries, dispatching retries, changing the boat or publishing a release.
