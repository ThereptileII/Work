# SCRUM-211: retained bccdbb1 native production failure

[Run 37177738716](https://github.com/ThereptileII/Work/actions/runs/37177738716), attempt **1**, exact candidate **bccdbb11cef827d3b63731fe1e874bc72bf47d2d**, concluded **failure**. This bounded audit preserves the original failure evidence; it establishes no package eligibility, Windows overall pass, application crash or boat acceptance.

Unchanged [artifact 11295952643](https://github.com/ThereptileII/Work/actions/runs/37177738716/artifacts/11295952643), `windows-production-failure-bccdbb11cef827d3b63731fe1e874bc72bf47d2d`, is retained as `original-artifact.zip`: **177,280 bytes**, SHA-256 **7ea2d15d0f033596bcb21824650437754c02f2bb0fb6baecfb90abbb9d2dbdb5**. Local bytes independently match the authenticated API digest. All **13 members** passed CRC and safe-path checks. `api.json` retains artifact/run provenance; `audit.json` records member hashes and derived findings.

| Original evidence | Finding |
| --- | --- |
| `windows-production-tests.xml` and original CTest rows | **139 unique native Windows production cases**, all run/passed; zero failures, errors, disabled or skipped cases. Original log reports **19.31 s**. This is **139, not the separate Linux suite's 147**. |
| `owned-ca-import.process.json` | Exit **0**, `timedOut: false`, elapsed **0.3414442 s**. Progress timestamps bracket the invocation by **0.344875 s** (approximately 0.345 s); the subsequent trust-import completion marker is present. |
| `progress.jsonl` | Valid fixture server reports start on port 51490. First `downloader-valid` process starts at **06:36:45.1922942 UTC**, reaches its 30-second limit, and records failure at **06:37:15.2150026 UTC**. |
| `downloader-valid.process.json` | `timedOut: true`, limit **30 s**, elapsed **30.0144378 s**, exit **−1** following harness termination of the owned process tree. This is not evidence of a spontaneous application crash. |
| Downloader stdout/stderr | Each retained file contains **two bytes, CRLF only**: no diagnostic text. No successful downloader result was recorded. |
| `prerequisites.json` | Retains source/probe/runtime identities and `wxCurlSourceKind: verified-private-ocharts`. The compiled probe and DLL hashes are runner observations; their binaries are not included for independent rehashing. |

The frozen runner calls the first Downloader `valid` case before its first private WXCURL case. This failure therefore **does not supply a private WXCURL trust-matrix result**; compilation and prerequisite identity cannot substitute for those runtime results. The previously completed production tests remain valid within their own scope.

## Source hypothesis, not a diagnosed cause

Frozen local candidate `1a6733a1cbcc817aa0f13fa5acc41aac62a54146` has no explicit wx application or log-target initialization in `tools/downloader-trust-probe.cpp`. Its patched Downloader retains warning logging on transfer failure. In wxWidgets **3.2.8**, [default logger creation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/common/log.cpp#L539) selects `wxLogOutputBest` when there is no application object, and that logger delegates to `wxMessageOutputBest`. The [Windows output implementation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/common/msgout.cpp#L104) can enter a blocking MessageBox when usable application stderr traits are absent.

That source path is a **plausible probe-harness explanation**, not proof that this process displayed a MessageBox or that a particular network error occurred. The archive contains no native window inventory, stack or emitted diagnostic establishing the blocked call. Native proof is still required. Source identities and primary-source blob references are recorded; no change or rerun was performed by this audit.

The original progress/prerequisite/process records and stream files are retained unchanged under `ocharts-private-wxcurl-trust-windows/`. `production-excerpts.log` contains only numbered summary/failure lines; full logs and the original XML remain inside the unchanged archive. No full logs were separately copied, no tests/builds/CI were dispatched, and no boat action occurred.
