# SCRUM-219: extraction repair resolved; downstream acceptance remains open

Evidence-only audit on 2026-10-04, based on
`407d3f750f7f21d6cf3eae306ad9ab4772f1f384`.
[SCRUM-219](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-219)
and all nine comments were read live (10247, 10250, 10251, 10255, 10256,
10257, 10274, 10441, 10927; complete pagination).

**Recommendation: retain Testing.** The extraction implementation and focused
native proof are complete, and the actual integrated producer crossed the
repaired boundary. Comments 10256 and 10441 explicitly retain required downstream
application/package proof before closure. Selection comment 10927 authorizes
this audit; it does not waive that requirement. No new extraction work or repeat
test/build is justified by the inspected evidence.

## Acceptance matrix

| Requirement | Exact retained evidence and closeout assessment | Result |
| --- | --- | --- |
| Diagnose the original pre-CMake stall honestly | Original `e9737d6bad9f3eb3db71877bce0816f59c62b0e7` (local `f9396889740e142e6a5c622eaada3a892ec0e348`), run `36850600318`, job `110331054362`, cancelled after 180m31s. Its curl log ends at the zlib prerequisite import report with no CMake configure output. OpenSSL 347 files/4,283 tests and zlib 13/13 passed. The original cancellation does not establish which operation stalled or the extractor's internal cause. [Original record](../scrum-219-curl-preparation-stall.md). | Met with explicit uncertainty |
| Disposable native reproduction with exact source, actual tools, timestamps, bounded waits and stdout/stderr/error evidence | Run `36873257468`, local `736361b120d2a00264415bfb51f7d59294a54929`, published `56a66a012da34c3e68afd8a9d5fc603b9a0d46ba`: selected Windows system `tar.exe`/bsdtar 3.8.4 timed out after 120,005ms extracting the locked archive; its retained stdout/stderr are empty. Stages record PID, deadline, tool hashes and failure. Separate CMake comparison exited 0 in 936ms. This proves a reproduced native extraction failure, not the original uninstrumented stop. [Reproduction receipt](../scrum-219-native-extraction-reproduction.json). | Met |
| Fix the reproduced cause with focused native regression evidence | Implementation `40893680dffb40c68f9d1961097e8d1b834f9f00` resolves CMake and uses its extraction command instead of system tar. Corrected local `f8ba5e9f82472ee5dad385db1fc78b42997f853c`, published `f1e9616738aaaf23d29e4d815f344404f94da659`, run `36874980766`: exact producer extraction command passed in 767ms, exit 0, CMake 3.31.6. Full in-run source inventory includes all 4,406 files and dotfiles. Closeout independently compared every path/SHA256 to the retained locked archive: no missing or mismatched file. [Equivalence receipt](../scrum-219-native-extraction-equivalence.json). | Met |
| Actual integrated producer crosses the boundary | Run `36969350849`, job `110721360745`, remote `0e9ec666ab27262a0c246f87f2e0fa5c7ede9fd0`: actual retained curl producer log records extraction from `06:20:55.4829790Z` to `06:20:56.1779153Z` (~695ms), then CMake configuration, compilation and 1,569/1,569 upstream tests. The later import-library filename failure belongs to SCRUM-224. [Integrated record](../scrum-224-0e9-import-library-failure.json). | Met |
| Repair remains in the frozen candidate without weakening locks/gates | The CMake tool-selection line and three extraction/log lines are byte-identical at corrected `f8ba5e9`, frozen `0a52a6c` and audit base `407d3f7`. Diagnostic script and curl lock blobs are identical. The entire current producer matches between frozen/root; later changes to the historical producer concern other documented boundaries and do not remove its nonzero all-pass upstream-test, native ABI or runtime checks. | Met for extraction scope |
| Required downstream application/package proof retained by comments 10256/10441 | Neither the short preparation proof nor the inspected `0e9` artifact qualifies the application/package. The latter lacks a successful curl producer manifest and ends before application/package acceptance. The current full candidate is still an ongoing qualification in root's handoff; no final job/package result is asserted. | **Open; retain Testing** |

## Independently inspected retained bytes

The following original ZIP sizes and SHA256 digests were rechecked in this audit;
their actual native stages/logs were read without executing their contents.

| Artifact | Bytes | SHA256 |
| --- | ---: | --- |
| Original cancelled job `11166793278` | 312,975 | `1544cdd80ec2ce789175923f364da7c6c4743eda998b96d02df4fcefcc823766` |
| Native reproduction `11168675603` | 7,447,656 | `24b711d798b6aad2ac7920027b42f0a1ff8c1d26cb2991a1471434e02b4a724a` |
| Corrected native proof `11168459240` | 7,619,925 | `34b63bd41fc1331454eeadfdadf1e2081418c0cf39a64ef92b1eef04dd9b7c87` |
| Integrated `0e9` producer `11213290312` | 11,482,710 | `b5e2bd96d04cb6f66dee6e01f8a986fc7982dd1f632ad099b52ec053d4b409ea` |

Original archive locations:

- `/home/standard/Projects/X-nav/evidence/local/e973-native-tooling/11166793278/artifact.zip`
- `/home/standard/Projects/X-nav-worktrees/scrum219-curl-preparation/evidence/local/native-probe-36873257468/artifact.zip`
- `/home/standard/Projects/X-nav-worktrees/scrum219-curl-preparation/evidence/local/native-probe-36874980766/artifact.zip`
- `/home/standard/Projects/X-nav-worktrees/scrum224-qualification-evidence/evidence/local/0e9-failure/windows-integration.zip`

The retained source archive at
`/home/standard/Projects/X-nav-worktrees/scrum219-curl-preparation/evidence/local/curl-source-inspection/curl-8.22.0.tar.xz`
is 2,953,092 bytes, SHA256
`f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7`.
The corrected native `source-files.json` contains 4,406 records, SHA256
`4ff792d70f4f233e8b343a6c7a16b397ca76bb91d7813f19b525ca98b2b4a707`.
Their complete path/hash comparison passed. The first reproduction's 4,399
uploaded files alone were insufficient because seven dotfiles were omitted;
the corrected in-run inventory resolves that earlier evidence limitation.

## Frozen-source binding and remaining boundary

At `f8ba5e9`, frozen `0a52a6c` and root `407d3f7`, diagnostic script
`tools/test-curl-preparation-windows.ps1` has Git blob
`0ad433a5ee675b20b3a724a4968338ca3440d04b`, and the curl lock has blob
`ce06e15e46e227319798c7e9949f03736917f078`. The unchanged extraction invocation
is `Invoke-Checked $CMake @('-E','chdir',$BuildRoot,$CMake,'-E','tar','xf',$Archive)`.
The entire current producer is blob `1e942fac66bce94ccd4e99846c745fbba82fb23b`
at both frozen/root revisions. The bounded diagnostic retains process-tree
termination and bounded redirected-output draining; production extraction
does not acquire a new per-process timeout from that diagnostic proof.

Root's handoff states that current `17ab044` / run `37164360050`, native job
`111325149619`, crossed integrated modes and maintained-TLS AIS while
fixture-free production was live. That is progress context, not a terminal
job or package result; this audit did not poll it. Required downstream
application/package acceptance remains unresolved here. Security, installer,
release and boat gates remain intact. No tests, builds, new downloads, source
edits, CI polling/dispatch, Jira transitions or boat actions occurred.
