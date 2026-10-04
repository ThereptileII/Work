# Windows 55ef51e — native passes, installer lifecycle assertion failure

**The native job failed and this candidate is not package-qualified.** [Job 111428607683](https://github.com/ThereptileII/Work/actions/runs/37199379614/job/111428607683), run **37199379614**, attempt **1**, completed at **2026-10-04 13:46:42 UTC**. Exact candidate **55ef51e4944e8570f6a391dad20b0db8447a7443**, frozen local source **e768b06bf5c130038abfd6e1166da0078037cb5b**. The real-host module, production trust, recovery and independent display gates passed; the installer lifecycle failed at a deliberately damaged-package case. No overall installer, package, boat or release acceptance follows.

Original artifact **11305580491** is **84,042,991 bytes**, SHA256 **`747e38e8d88c4e49fca6b6efb181101db10824ac87cf55f6e0be80011e8bec7a`**. Authenticated API size/digest match downloaded bytes. All **20,127 ZIP entries** pass CRC and unique relative/case-fold path, no-traversal/drive/backslash/encryption/symlink/special-file checks. [api.json](api.json) retains terminal job and artifact records.

Ignored original: `/home/standard/Projects/X-nav-worktrees/bccdbb1-contracts-restart/.local/windows-integration-55ef51e/original.zip`. Selected unchanged reports are extracted under its sibling `extracted/`, preserving `evidence/local/` and `build/` paths. [selected-original-reports.zip](selected-original-reports.zip) is a new documentary subset of **299 unchanged report/stream/cache payloads**, not the original artifact; [audit.json](audit.json) records each hash. No binaries, certificates, charts or profiles are committed here. No tests, builds, reruns, captures or boat actions were performed by this audit.

## Exact native results

| Gate | Recorded evidence |
|---|---|
| Fixture and production CTest | **139 + 139** executions, identical **139 unique** case identities; zero failures/errors/disabled/skipped. Production cache has route scenarios, fixtures and pilot loopback OFF; fixture cache has them ON. |
| Real-host private chart module | Four expected exits **2 / 0 / 0 / 1** for option-without-selftest, default, module, wrong-original. Actual module resources/imports/binding/status/point-style/unload passed; profiles unchanged. |
| Module stage trace | Eight explicit markers: begin, resources verified, load/bind begin, load/bind returned, binding status queried, point style queried, unload begin, **unload returned**. Wrong-original was refused with `original plugin identity is unsupported`. |
| Module scope | No factory call, plugin Init, original vendor DLL execution or rendering; child process creation blocked; point style explicitly unavailable before Init. Original report keeps `nativeProductAcceptance=false`. All 21 recorded source identities match frozen Git/Windows CRLF bytes. |
| Production default loader | Exact candidate, fixtures false, `INSTALLED PRODUCT`, `status-only`, resources present, plugins/profile initialization false. The separate `installer-loader-selftest.json` is the earlier fixture-enabled developer report and is not used as production proof. |
| Product output policy | Five native simulated-loopback checks passed; zero received bytes, sent commands and wire output, no transport failures. No physical pilot qualification. |
| Production/private TLS matrix | **12 Downloader + 9 private o-charts wxCurl cases** passed; respectively 3 accepted/9 refused and 3 accepted/6 refused. |
| Separate public TLS matrix | **12 Downloader + 9 core wxCurl cases** passed with the same acceptance/refusal counts. These are 42 process executions across two matrices, not 42 unique scenarios. All 42 process receipts show expected exit 0/1, no timeout, console cleanup completion; both owned trust cleanups verified. |
| Native AIS | Actual logs: **178 session checks**, **18 TLS/transport PASS rows**, **six provider/server lifecycle pairs**. Exact run/attempt/commit; all 15 retained runtime files and same-job receipt independently rehashed. Product, WFP and boat acceptance remain false. |
| Exact extracted recovery ZIP | **Seven checks passed**, **1,015 files verified**, fixtures OFF/status-only, normal-profile audit count 0. The report binds executable and restart helper hashes and protocol 1; it is not installer lifecycle acceptance. |
| Preview and crash recovery | Preview 19 checks passed. Three recovery-repeat reports each contain five checks; these are repeated fixture scenario executions, separate from the seven production recovery checks. Screenshot review remains required. |
| DPI | Actual GetDpiForWindow/wxDpi **96 / 120 / 144** at 100% / 125% / 150%; interactions passed and display restored to 100%. Native visual review is still required. |
| Public ENC | Software passed. Requested OpenGL was **rejected by the host**, and upstream software fallback passed; hardware GL remains open. No private-chart or boat GPU qualification. |

## Installer failure and retained hash bindings

`installer-lifecycle.json` records **34 completed checks** before `installer-41-damaged-package.json` failed its expected assertion. That operation refused Update with **`Missing app-local PE import wxbase32u_net_vc14x.dll`**, naming `app/opencpn-cmd.exe`. The preceding corrupt-ZIP and interrupted-extraction checks completed. This is the deliberately damaged-package refusal boundary, not evidence that the clean recovery or Setup payload omits the DLL. The lifecycle remains failed, and subsequent checks were not completed.

The earlier compact failure artifact **11304975518** is retained ignored at `/home/standard/Projects/X-nav-worktrees/bccdbb1-contracts-restart/.local/windows-installer-failure-55ef51e/original.zip`: **571,655 bytes**, SHA256 **`10b2b26391bab2ce33631f8891f29ecee087c6531a0486dcdd650a258fcf8b6c`**. API digest/size, all 121 CRC/safe paths, and byte equality of **all 121 members** against their final-artifact counterparts were independently checked.

The original native reports agree on these identities:

| Reported payload | SHA256 |
|---|---|
| Recovery ZIP | `18fbab979dbc2f73042b62a18fce12594152d89eb939f103efcc67b678f21c33` |
| Setup, 47,266,310 bytes | `735d035261438ce29c3e00893bf91395db8f6411750b86a8bb33e4ec79e4cfb7` |
| Product executable | `20b454904958dfb86c3689d63a3486b3ed4e14a297295e19aa2582243ba9ee3f` |
| Restart helper | `322a6873cd0a3d36f6b09217307b6c5e089b8a91972256ba0ca9a2dfb196e49b` |
| Private adapter | `73647285f59bcf3575e0adc9abfa4d8812da4a4732718acb3ff439924ed37ba7` |

These are **original report measurements**, not independently rehashed delivery payloads: this evidence archive contains neither the recovery ZIP, Setup nor product executable. The recovery/build, module host and PE-brand reports agree on the product executable; installer preservation and PE-brand reports agree on Setup. No eligible candidate-delivery artifact was produced after the failure.

The endurance policy and later delivery steps were skipped after installer failure; there is no native `soak/results.json` in this archive. This is not an endurance pass or the separate Linux skip receipt.

Existing unchanged 1280×800 images were extracted for root review under the original archive's sibling `review/`: `recovery-navigation-day.png`, `recovery-navigation-day-restored.png`, `chart-software-01-loaded.png`, `chart-software-06-returned.png`. The first pair belongs to production recovery; the second to disposable public-ENC fixture rendering. This audit makes no screenshot, font, individual symbol-family, private-chart or boat-display acceptance claim.
