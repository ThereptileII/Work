# Native Windows 17ab044 artifact audit

**Partial gates passed; the Windows job was cancelled and no installer is
eligible.** The private chart TLS trust operation did not complete. This audit
does not turn that cancellation into an overall Windows, package or boat pass.

Exact candidate `17ab044a1e5222dc71791ac8118454219efe8734`, run `37164360050`,
attempt 1, native job `111325149619`. Original artifact **11293866599** is
**66,715,913 bytes**, SHA256
`5895a38e5bfd474d2969d831a9fa7fe922a5eaf8b7d7194643382aa1303d1ed8`.
This audit independently rehashed the locally retained archive; its identity
matches root's authenticated GitHub metadata. The archive contains 17,974 entries.

| Completed evidence | Finding |
| --- | --- |
| Fixture-enabled application tests | 139 unique XML cases, all run; zero failures, errors, disabled or skipped cases. |
| Production application tests | Another 139 unique XML cases, all run; zero failures, errors, disabled or skipped cases. Production CMake cache has route scenarios, test fixtures and pilot loopback all OFF; fixture build has all three ON. This does not imply the later production/package gate completed. |
| Actual native AIS session | Original `session.log` explicitly records **178 checks passed**. |
| Actual TLS/transport | **18 PASS rows** and final success summary, including invalid trust/hostname, redirects, compressed overflow, fragmentation and receive-bound cases. |
| Actual provider lifecycle | Six provider/server passes: normal IPv4, disable/enable, credential change, open-disable/enable, open-credential-change and normal IPv6. These are actual loopback executions, not requested counts. |
| Exact runtime/source | All 15 retained AIS runtime executables/DLLs rehash to the report; same-job dependency receipt also matches. Recorded wrapper hash matches frozen local `0a52a6c` with Windows CRLF. Report identifies exact run/attempt/candidate. Both transport/provider executables record maintained SSL/crypto/zlib imports; OpenSSL reports 3.5.9, VC-WIN32. |
| Native fixture UI | `preview-results.json` records 19 checks and `passed; screenshot review required`. Higher DPI remains explicitly unexercised in this report. |
| Private chart TLS | The retained trust directory contains prerequisites only; no completed result. The [original timestamped excerpt](native-trust-delay-excerpt.log) ends after fixture certificates at 02:11:25 UTC, before cancellation at 04:16:57 UTC; the import/first-probe boundary is not yet resolved. |

The AIS report explicitly retains `productAcceptance`, `wfpAcceptance` and
`boatAcceptance` as false. Loopback success does not establish live-provider,
Windows filtering, installer or boat acceptance.

## Native images available for review

The original 1280×800 [day navigation image](preview-01-navigation-day.png) shows
coastline/basemap geometry and the native SKAGER shell with clearly labelled DEMO
values. The [returned XNav image](preview-09-returned-xnav.png) shows coastline
geometry with live inputs explicitly unavailable. Both are from the disposable
fixture-enabled preview tree, not a qualified installed product.

These are coarse basemap views, not completed ENC/private-chart symbol or depth
rendering evidence. A separately inspected production route-label PNG is a
component drawing fixture, not an application chart view. Native typography,
full chart/symbol comparisons, DPI and boat-display acceptance remain separate.

`audit.json` records the bounded independent checks and original-file hashes.
`selected-original-reports.zip` is a new documentary subset of 11 unchanged
reports/cache files, not the original GitHub archive. The two images are retained
byte-for-byte. No binaries, charts, private keys or configuration profiles were
copied into this evidence directory. No builds, tests, downloads, CI actions or
boat operations were performed; earlier endurance work is outside this audit.
