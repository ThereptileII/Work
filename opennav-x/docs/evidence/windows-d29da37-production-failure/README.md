# d29da37 Windows production gate failure

Candidate `d29da372af86c01cb2817f932891fd9408d882fe`, frozen local
`615118f351abdea7b79a042db63e58fe0a635e0a`, existing run `37184477492`.
The run mapping was supplied by root; the original production log independently
names the candidate SHA. No fresh CI query, test, build or boat operation occurred.

The unchanged [original artifact](original.zip) is 176,609 bytes, SHA256
`9584f33daae2082dfcaaff7c334192c54b6e520613067481049dae64a862d301`.
All 13 entries pass CRC, unique safe path and non-symlink checks.
[audit.json](audit.json) records every original member's size/hash. Small reports
are copied unchanged; both full original logs remain in the ZIP. The
[terminal excerpt](terminal-excerpt.txt) identifies original log line ranges.

| Stage | Original evidence | Outcome |
|---|---|---|
| Production tests | `windows-production-tests.xml`, corroborating transcript | **139 distinct cases passed**, zero failures/errors/skips; logged total 19.26s. This does not include the subsequent trust matrix. |
| Native probe preparation | Transcript links `downloader-trust-probe.exe` and `wxcurl-trust-probe.exe`; `prerequisites.json` | Both probes built. Actual integrated Downloader source/header and verified private o-charts wxCurl sources are recorded, alongside preparation/curl manifest and 72 runtime DLL hashes. |
| Certificate fixtures | Transcript signs valid localhost, wrong-host, expired and untrusted leaves | Fixture generation completed before import. |
| Owned trust import | `owned-ca-import.process.json`, progress | Exit0 in **0.2698864s**, not timed out; LocalMachine Root import `after` recorded at08:56:16.0670734Z. |
| Valid TLS server | Progress | Published localhost port60607 at08:56:16.2650048Z. |
| First Downloader case | `downloader-valid.process.json` | Exit **−1073741819 / 0xC0000005** after **1.2674955s**; `timedOut=false`, configured deadline30s. This is an access-violation exit, not the earlier timeout. |
| Last observed probe work | Original stderr | Curl initialization → download begin → download returned / HEAD begin → reported filesize36 bytes → **HEAD returned**. |
| Case assertion | Original stdout is only CRLF (2 bytes), terminal failure | Required `download_ok` field absent. Harness throws `valid probe output omitted download_ok`; production step terminates exit1. |

The stage receipts are under
[ocharts-private-wxcurl-trust-windows](ocharts-private-wxcurl-trust-windows).
The last progress record is the first `downloader-case/valid` failure at
08:56:17.5635821Z. There is no successful `valid.json`, no wxCurl case execution
receipt and no `summary.json`: **zero completed trust-case reports**. The frozen
source runs private wxCurl valid only after Downloader valid succeeds, followed
by redirect/downgrade/file/partial-transfer, wrong-host/expired/untrusted,
write-exception/unrelated-directory/initial-HTTP and trust-removal coverage.
None of those later cases is established by this artifact.

The returned GET/HEAD stages and filesize line do not establish complete payload,
structured-result or TLS matrix acceptance. The native process exit is confirmed;
its faulting instruction, stack and cause are **not established**. No dump or
exception stack is retained. Do not label this a wxWidgets, curl, destructor,
logging or TLS implementation defect from these bytes alone.

Prerequisite binary/DLL hashes and the recorded production executable SHA256
`3a4d81dc7a4136d4aee940474ddabe84d8a128ffbf9f1f6baa64b5b94c8fac0a`
are runner observations; this failure artifact contains no binaries for an
independent rehash. The harness has a finally-cleanup path, but there is no
successful cleanup summary here; no independent cleanup acceptance is claimed.

**No eligible installer/package is established.** The required production trust
gate failed despite the preceding 139 tests passing. This artifact has no Setup
or portable package and grants no install, launch, chart-review or boat acceptance.
Preserve this failure while root pursues a bounded diagnosis; no cause or fix is
claimed by this documentation-only audit.
