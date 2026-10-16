# Original 854078a5 native producer outcomes

Candidate `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`, version
`0.4.0-beta2.4`, passed its original [Windows producer job
116536443214](https://github.com/ThereptileII/Work/actions/runs/38765418126/job/116536443214)
in run `38765418126`, attempt `1`. All 34 recorded job steps succeeded.
This receipt records original component and producer checks. The original
[downstream qualifier 116562831922](https://github.com/ThereptileII/Work/actions/runs/38765418126/job/116562831922)
remains failed; these outcomes do not qualify installed startup or delivery.

The authenticated artifact `11992784515` contains 65,383,721 bytes, SHA-256
`14d14df0f80e20f7b40877bca167d527914c900c5f1b092db31c0cc8ca5b8bb9`.
The custody owner verified all 18,125 member CRCs. This read-only extraction used
that retained archive without another download, full CRC pass or test execution.
The original 2,879,992-byte producer log has SHA-256
`6d405c136ce05dca73402d3c33ca5d6d3640aa584eef0498e377739bdebc8818`.
[receipt.json](receipt.json) binds the selected original members, hashes and line
references. Large originals remain private; no vendor binaries or profile data
are committed here.

| Original scope | Observed result | Authoritative member or producer-log lines |
| --- | --- | --- |
| Fixture and fixture-free CTest | The same 146 ordered names passed twice: 292 executions, zero failures, skips or disabled cases. | `windows-xnav-tests.xml` and `windows-production-tests.xml`, lines 2–6; log 6474 and 10652. |
| Peer unavailable model | Four cases passed in each CTest execution, covering empty transfer refusal, no object serialization, retained-version clearing and key refusal. | Both XMLs, testcase lines 727, 743, 759 and 775. |
| AIS native loopback | Eight provider lifecycles, 18 TLS/transport scenarios and 251 actual session checks passed. The declared session minimum is 178. These are separate nested scopes, with no combined total. | `ais-native-runtime/provider.log`; `transport.log:19`; `session.log:1`; minimum in `report.json:14`. |
| Core Downloader/wxCurl trust | 12 Downloader cases (3 accepts/9 refusals) and 12 wxCurl cases (4 accepts/8 refusals) passed; cleanup verified. | `downloader-trust-windows/summary.json:2–4` and its case arrays. |
| Private o-charts wxCurl trust | The 12 core Downloader scenarios passed again; 12 cases exercise the separately bound private wxCurl source. Cleanup verified. These are not 48 distinct scenarios across the two invocations. | `ocharts-private-wxcurl-trust-windows/summary.json:2–4`; both `prerequisites.json` files identify the source kind. |
| Real-host private chart module | Four expected outcomes passed, including invalid-option and wrong-original refusals; profiles unchanged. Factory, plugin Init and chart rendering were deliberately unexecuted. | `ocharts-real-host-module/summary.json:32–53,450`; `module.json:2,22,35–36`. |
| Peer CLI and native response buffer | Exact source/executable/run CLI success receipt verified; buffer CTest passed 1/1. | `windows-peer-cli-receipt.json`; log 6527; `peer-buffer-windows.log:23–25`. |
| Fixture-free loader/restart | Loader passed with four required resources present, fixtures disabled, no profile initialization or plugin loading. Restart capability probe accessed no profile and started no child. | `production-package-selftest.json:4–24`; `production-restart-selftest.json:2–6`. |
| Producer helper checks | AIS gate 12 passed; packaged curl source-patch reproduction 1 passed; compiled-recovery checks 7 passed. | Log 528–530, 6515–6517 and 10854–10856. |

Both retained OpenSSL consumer observations passed. Historical dependency
producer suite results are not counted as new executions here. The PE dependency
audit records 88 entries and zero missing dependencies; the branding receipt is
a data-only application/setup inspection, not installer execution.

The producer records `trackedSourceDirty=true` for compiled recovery at log
10866. This outcome receipt does not replace the separate retained source
inventory/audit or describe that checkout as clean. It keeps the immutable
product revision separate from this later documentation commit.

The producer explicitly defers desktop qualification at log 6488 and 10834.
PluginHandler is capture-only in this artifact. Final installed lifecycle,
startup, fresh retained plugin qualification and boat acceptance remain separate
gates. Design validation was disabled; endurance was not exercised and retains
the user-directed skip. No skipped or pending gate is represented as passed.
