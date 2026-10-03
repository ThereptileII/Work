# SCRUM-272 / SCRUM-273: native trust-probe compile audit

[Run 37135967220](https://github.com/ThereptileII/Work/actions/runs/37135967220),
job `111240444202`, passed on 2026-10-03. Remote
`71470d07d8dd9ac6a3050e2b64093e0892473c97`, tree
`09c767e7c3aca1f5bf8c238bc35dad48977a596e`, maps to local
`1c4d817c373b85838bc7c1b5d97d11bd026b3454`.

Downloaded original artifact `11278945798` is 536,550 bytes, SHA256
`49c8ddee500f6f881099010dac432a24eff5f492224a3a8a0e3f987624174572`.
All 171 ZIP entries passed CRC verification. The original ZIP is retained at
`/home/standard/Projects/X-nav-worktrees/scrum273-native-proof-audit/.local/native-trust-714.zip`;
the extracted archive and full completed job log are retained alongside it.

Six deterministic source-cache checks and 18 path plus six absent-input checks
passed. The unchanged native path control reproduced the original
`Invalid character escape '\a'` at the actual trust-probe target source lists.
Both corrected core/private configurations then succeeded with MSVC
19.44.35229.0, Windows SDK 10.0.26100.0, Win32 and `/MD`. Four actual production
source projects compiled ten objects. Core wxCurl reports eight deprecation
warnings, zero errors; the other three compile commands report zero warnings
and errors. No assertions were relaxed.

Independent artifact checks:

- All 61 local source/workflow/recipe inputs match the exact local revision:
  11 directly, 50 after exact Windows CRLF checkout conversion.
- All 219 private originals were reconstructed from cached pinned Git blobs,
  checking Git blob hash and length. Applying this revision's exact patches
  reproduces all 219 recorded derived source hashes without normalization.
- All 154 core inputs were reconstructed from pinned OpenCPN Git objects and
  the nine ordered current patches. Recorded Windows bytes match precise CRLF
  checkout expansion. All ten archived translation-unit copies match their
  corresponding reconstructed source or exact local probe source.
- All 1,611 SDK input hashes match the prior independently staged locked SDK.
  The two wx source archives and curl archive were rehashed against current
  locks; curl header bytes were also compared directly from its archive.
  The wx archive extraction itself was not repeated in this audit.
- Four retained `.vcxproj` files match report hashes and their exact expected
  `ClCompile` source sets. Production `/MD` remains; no `NOMINMAX` or forced
  include was introduced. All ten object lengths and SHA256 values match the
  report; each actual COFF machine field is `0x014c` (x86).

`audit.json` records independent comparisons. `native-report.json.gz` preserves
original report bytes. The original failing configure and corrected configure/
compiler logs are retained verbatim. Original cache-failure evidence remains
under `scrum273-native-9ec-cache-failure`.

This qualifies the bounded cache/path/configuration and actual object compilation
repair only. The runner invokes `ClCompile`, not `Link`; CMake's compiler-detection
executables are not application or trust-probe runtime evidence. Maintained
producer, executable link, real TLS, application, package and boat acceptance
remain separate gates. No full build or rerun was dispatched by this audit.
