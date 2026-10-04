# Native maintenance contracts for bccdbb1

[Job 111363797680](https://github.com/ThereptileII/Work/actions/runs/37177738716/job/111363797680), **Native boat recovery and commissioning contracts**, completed successfully on Windows Server 2022 at **2026-10-04 04:44:35 UTC**. Authenticated job/artifact records and the original checkout log bind remote commit **bccdbb11cef827d3b63731fe1e874bc72bf47d2d**, run **37177738716**, attempt **1**. The workflow invokes the twenty maintenance suites in native **Windows PowerShell 5.1**, then the Python navigation-copy suite, and refuses completion if a suite fails.

Unchanged [artifact 11293129075](https://github.com/ThereptileII/Work/actions/runs/37177738716/artifacts/11293129075), `boat-maintenance-bccdbb11cef827d3b63731fe1e874bc72bf47d2d`, is retained as `original-artifact.zip`: **25,160 bytes**, SHA-256 **8c4be1c6b09d4e2faaff466d7733fcf0c1afc27d5567ff8b499c1df68bfc604a**. Local bytes match both the authenticated API digest and original upload log. All **21 members** passed CRC and safe-path checks; the archive contains exactly the expected suite logs, with no duplicate/traversal/absolute/drive paths or symlinks.

| Recorded suite | Passing checks reported |
| --- | ---: |
| boat-tools | 31 |
| official-upgrade-policy | 86 |
| preparation | 34 |
| commissioning | 21 |
| commissioning-launch | 11 |
| review-window | 233 |
| review-staging | 12 |
| source-checkout | 16 |
| ais-credential-import | 17 |
| restart-commissioning | 313 |
| restart-aui-persistence | 77 |
| stock-review | 90 = 78 policy + 12 transaction |
| stock-chart | 61 |
| startup-log | 16 |
| renderer-log | 66 |
| baseline-adoption | 178 = 127 policy + 25 native stock + 26 native installed-resource transactions |
| broker-fixture-contracts | 29 |
| restart-window-review | 187 |
| stock-welcome | 149 |
| installed-welcome | 114 = 85 policy + 29 runtime-fixture |

These are **twenty passing PowerShell suite receipts**, with check/assertion counts rather than unique independent test counts. Nested transaction checks already contribute to their parent totals; several suites invoke the same commissioning fixture helpers with different options. No combined unique-scenario total is claimed. Separately, `maintenance-navigation-copy.log` records **11 unique Python tests**, each `ok`, terminal `OK`, in **3.422 seconds**, with zero failed/skipped tests. Member hashes and reconciled count components are in `audit.json`.

## Native evidence and limits

Preparation and commissioning exercise actual Windows disposable filesystem behavior: atomic replacement, locks, ACL/stream/hard-link refusals, journal interruption/recovery, exact-byte restoration and preserved fixture data. Baseline adoption additionally records both native stock and installed-resource transactions. Source-checkout uses disposable Git fixtures with `networkAccess: false`; credential import reports `native: true` and `productionCredentialAccessed: false`.

Native interop compilation is recorded for review/stock policies. **Compilation does not mean the corresponding desktop action ran.** Restart commissioning explicitly has `nativePipeExecuted: false`; restart-window review has `nativeWindowExecuted: false`; review-window has `nativeActionsExecuted: false`; stock-chart has `nativeApisInvoked: false`. Stock-welcome does execute a read-only desktop metadata probe, but reports `actualStockModalAcceptance: false`; installed-welcome reports `windowsApisInvoked: false` and `actualInstalledModalAcceptance: false`. Renderer/startup suites use synthetic logs; the renderer receipt explicitly denies hardware-acceleration acceptance. The actual application was not launched for these maintenance suites; no boat or hardware command acceptance is implied.

Source mapping uses frozen local **1a6733a1cbcc817aa0f13fa5acc41aac62a54146**. All 21 entrypoint files independently match that commit; their SHA-256 hashes are retained. The entire `tools/boat` Git tree matches frozen tree **aadbdce396d7f1cf2428fa39b02c3cff8854ab4c**. Workflow SHA-256 is `f52c3e7a7a350cd09a1cc67b1b47ae36bbcd6b7be59c14d864b515d91d35898e`; its exact remote/local byte equality was established in `../contracts-bccdbb1/`. The broader publication mapping is root evidence, not re-created here, and no runner-side script-byte attestation is invented.

`api.json` retains selected authenticated provenance. `audit.json` records original member hashes, every suite count, native/disposable scope flags and source identities. `job-excerpts.log` retains numbered checkout, invocation, final test and upload lines. Full decoded job log remains in this worktree's ignored `.local/boat-maintenance-bccdbb1/original.log` (**159,065 bytes**, SHA-256 `e3afd7911aacf1f8d75434f50e025e652e1c3a4302b73dac4d83dd265aa4ad6d`); original ZIP and extracted logs are beside it.

No tests, builds, application launch, CI dispatch or boat operation were performed by this audit. **This establishes the completed native maintenance-contract job only; application-package, native visual, actual modal/restart process and physical boat gates retain their separate evidence requirements.**
