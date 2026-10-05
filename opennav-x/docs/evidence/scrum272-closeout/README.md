# SCRUM-272: native trust-probe path repair closeout

Evidence-only audit on 2026-10-04, based on
`5d19314fe51d7192ea3cfc2fa55d1674c559f406`.
[SCRUM-272](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-272)
and all four comments (10796, 10800, 10802, 10917; complete pagination) were
read live. Comment 10917 selects this bounded audit and explicitly separates
full corrected-candidate package/boat qualification.

**Recommendation: the configure repair meets its own acceptance criteria and
can move to Done after root review.** No Jira transition was performed. This
does not close parent SCRUM-211, TLS runtime, package, installer, or boat gates.

## Acceptance matrix

| Issue criterion | Retained evidence and independent closeout check | Result |
| --- | --- | --- |
| Preserve the original failure | [Original production audit](../scrum259-native-442-trust-configure/README.md), original compressed transcript and `first-error.txt`: run `37126951293`, remote `442960ba55277845e171f9b95a38838f66c23981`, local `8a0ed1f646e2551a55639c1cc3fb609cc2652464`. Production passed 139/139 CTests before `Invalid character escape '\a'` in native source paths. No TLS assertion ran. | Met |
| Drive-letter/backslash paths configure correctly at the boundary | Repair `9f72d0a797868d84efd9441022220a9b2ca52918` normalizes all six path-valued inputs before preparation verification, wx discovery and target expansion. [Focused native logs](../scrum272-native-714-compile/README.md) retain the original failure control and successful corrected core/private configuration with real `D:\a\...` inputs, including spaces in SDK/private paths. Retained `paths.log` records 18 drive/UNC/POSIX and six absent-input checks. | Met |
| Actual downloader and private wxCurl sources remain selected | Four archived `.vcxproj` files were read directly and their hashes rechecked against the original report. Both downloader projects compile integrated `model/src/downloader.cpp` plus the actual probe; private wxCurl compiles locked/patched private `base.cpp`, `http.cpp` plus the actual wxCurl probe. Core wxCurl uses its actual integrated copies. All ten reported object hashes/lengths were rechecked from the archive and all machine fields are `0x014c` (x86). | Met |
| TLS assertions unchanged | The original executable target block is byte-for-byte identical to the extracted `Targets.cmake`. Both probe implementations, TLS server, Windows TLS runner and all three relevant TLS patches have identical Git blobs across the repair parent, repair, focused proof and later full candidate; identities below. Production prepared-source verification, maintained curl import-library existence, `/MD`, native trust path, source selection and lack of TLS-test bypass definitions remain intact. Both CMake fragments enter the private preparation input closure. | Met |
| Focused native configure/probe gate succeeds before a full candidate | [Run 37135967220](https://github.com/ThereptileII/Work/actions/runs/37135967220), job `111240444202`, local `1c4d817c373b85838bc7c1b5d97d11bd026b3454`, remote `71470d07d8dd9ac6a3050e2b64093e0892473c97`, tree `09c767e7c3aca1f5bf8c238bc35dad48977a596e`: original control fails, both corrected configurations succeed, and four actual source projects compile ten x86 objects using MSVC `19.44.35229.0`, SDK `10.0.26100.0`. This precedes the combined candidate selected in comment 10802. | Met |
| Exact corrected full candidate remains required for package/boat qualification | Preserved as a separate mandatory gate. The narrow runner only invokes `ClCompile`; it does not qualify linking, TLS, maintained producers or packaging. Its preparation/import-library boundary omissions cannot be used as full-production acceptance. Later production evidence below closes only the previously unobserved actual private configure/link boundary. | Preserved; package/boat qualification remains open |

## Exact artifacts and later production corroboration

The focused artifact `11278945798` was rehashed directly from the retained
original ZIP: 536,550 bytes, SHA256
`49c8ddee500f6f881099010dac432a24eff5f492224a3a8a0e3f987624174572`.
It contains 171 entries. The existing [independent audit](../scrum272-native-714-compile/audit.json)
records the original CRC checks, 61 exact local inputs, 219 reconstructed private
originals/derived files, 154 core inputs and 1,611 SDK records. This closeout
rechecked archive identity, the four project hashes/source lists and ten actual
object identities; it did not rerun those reconstruction checks or any tests.
Original ZIP:
`/home/standard/Projects/X-nav-worktrees/scrum273-native-proof-audit/.local/native-trust-714.zip`.

[Run 37136712793](https://github.com/ThereptileII/Work/actions/runs/37136712793),
job `111243795980`, local `96c0c2705aacae511b6f8c22afaa6618dbabceff`, remote
`15452e512fd073090b1a4cea7c9010eac0874118`, tree
`3c53cff0332dfc09babb8e5b8daa7986f77a9c84`, later reached the actual private
production path. Its [retained transcript](../scrum272-native-154-expired-fixture/first-error.log)
records successful configuration at 18:13:40 UTC and both real probe executable
links at 18:13:47 UTC. The [original prerequisites receipt](../scrum272-native-154-expired-fixture/private-probe-prerequisites.json)
identifies `wxCurlSourceKind=verified-private-ocharts`, the actual source and
probe hashes, runtime DLL hashes and prepared-source receipt hash. This is
stronger evidence for the repaired production boundary than the compile-only
wrapper alone.

That full run still **failed** at 18:13:49 UTC while generating the expired
certificate (`expired-index.txt` could not be parsed), before CA import, server
startup or any trust case. No TLS pass follows from the configure/link pass.
The [existing audit](../scrum272-native-154-expired-fixture/README.md) retains
artifact `11281990645` identity and the original failure. Probe executables were
cleaned up, so this is retained log/receipt evidence, not a replayable payload.

## Source identity checks

The following Git blobs are identical at `9f72d0a^`, `9f72d0a`, `1c4d817` and
`96c0c270`; this verifies that the path repair and its two historical proofs
did not change the TLS assertions or relevant patched production behavior.

| Path | Git blob |
| --- | --- |
| `tools/downloader-trust-probe.cpp` | `89648d0b6dc6069447cf28dd31aaa9bd544424aa` |
| `tools/wxcurl-trust-probe.cpp` | `e6cc4adaae2fa64c1aac48ad34a4505bce0ab964` |
| `tools/downloader-trust-server.py` | `45ea5950dfbe8f8728cbd93e56aab8eb1ba183f1` |
| `tools/test-downloader-trust-windows.ps1` | `fe584270cf6528e2e2f21db3e11e63df84735035` |
| `patches/opencpn-5.12.4-download-trust.patch` | `b84516d2486602334e0d7eada245ff6cd376b4ba` |
| `patches/opencpn-5.12.4-wxcurl-trust.patch` | `2236e8ed4195df1525228d2ec11f2f09c2e17faf` |
| `patches/ocharts-wxcurl-trust.patch` | `248ce8cff01363ddfa01cf54f7ba8d044c61b347` |

The three path-repair CMake files are also byte-identical from `9f72d0a` through
both historical proofs, audit base `5d19314` and frozen local candidate
`0a52a6c`: `CMakeLists.txt` blob `387b4c5f39b1717dd037341824f64ded7edc6489`,
`InputPaths.cmake` blob `b676bdb440f171aa243e5edb9ade200dd288c0d9`, and
`Targets.cmake` blob `d501b3871c63ccf9ec4b3eee8a9a431b7a971f0d`.

The six probe/server/patch entries above remain identical at frozen `0a52a6c`
and root `5d19314`. The Windows TLS runner has one later, separate SCRUM-277
fixture correction (`fc87ec9`): creating the expired-CA index with a zero-byte
`[IO.File]::WriteAllBytes` call instead of `Set-Content` writing an empty line.
The exact diff from `96c0c270` to `0a52a6c` contains only that replacement;
the TLS assertions and owned-trust cleanup body are unchanged. The runner is
identical between frozen `0a52a6c` and root `5d19314`. This source comparison
does not claim that the corrected fixture or current TLS runtime has passed.

## Remaining qualification boundary

No missing gate remains for this specific normalization/configure repair. Actual
core/private TLS suites, owned-certificate cleanup, full exact-candidate
application/package/installer acceptance and boat acceptance remain mandatory.
The current frozen local `0a52a6c` / remote `17ab044` / run `37164360050` is
context supplied by root; this audit did not poll it or infer its outcome.
No source edits, new tests, build/test reruns, CI dispatch, boat action or Jira
mutation occurred. Only this documentation was added for independent root review.
