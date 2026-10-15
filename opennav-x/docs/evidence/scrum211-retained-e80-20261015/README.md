# Retained e80 candidate custody and native PluginHandler result — SCRUM-211

The Windows producer succeeded for frozen product
`e80cb962e1a32d98bb75b964dca6eb516f7a5e23`,
[baseline run 38744236920 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38744236920/attempts/1),
job `116461918332`. The baseline later failed in Windows qualification. These
receipts authenticate the original bytes; they grant no installed-product,
release, boat or public-access acceptance. This documentation commit is separate
from the product and does not relabel its source.

## Original custody

[custody.json](custody.json) records exact API artifact identities, original ZIP
sizes and SHA-256 values, complete member CRC checks, source/package inventories,
and hashes of the private audit receipts. Each original was downloaded once.
The original API responses, logs and binary archives remain private; none is
included in this evidence change.

| Original artifact | ID | Bytes | Verified ZIP members |
| --- | --- | ---: | ---: |
| Compiled recovery | `11981264862` | 428,133,658 | 2 |
| Plugin guard capture | `11981229778` | 131,376,585 | 1,588 |
| Windows build evidence | `11983241024` | 65,382,852 | 18,125 |
| Staging build | `11983131179` | 1,541,595,806 | 2 |
| Failed Windows qualification | `11983762287` | 17,427,852 | 4,625 |

The nested compiled recovery inventory authenticated all 17,121 payload files,
including 9,266 product entries against the frozen Git blobs and modes, with
explicit Windows checkout line-ending handling. The sealed staging inventory
authenticated 4,578 files. Its unchanged qualification field is `not-run`.
The source ZIP authenticated 13,669 files and five bundled dependency sources;
the recovery ZIP authenticated 1,046 files and the fixture-free product identity.
Source parity includes the exact Git blobs for two integrated symlink texts.
The omitted product `.gitkeep`, reference-only integrated gitlink and separately
retained workflow recipes are explicitly distinguished in the receipt.

| Existing package leaf | Bytes | SHA-256 |
| --- | ---: | --- |
| `SKAGER-Beta2-Setup.exe` | 187,480,564 | `37e925de13d2ed02466b40c234c8483290eb58b87ad808d6bd202d8fd4976082` |
| `SKAGER-Beta2-Portable-Recovery.zip` | 193,909,393 | `e2182746356a1d4bc6808d9699de336f9ea364785de7beca16228621bf6dcffc` |
| `SKAGER-Beta2-source.zip` | 453,511,078 | `a9ce8efe33d44128ff26cc045803e60a450885ce5a536e45db7cb896aa0bfa74` |

## Native gate attempts preserve the refusal

The independently reviewed capture inventory pin is
`670e86e7b4fd8c047cb8449c22a2d75d9faa703454474e700f416f5d26a7b79e`.
All 1,585 captured payload files and 11 runtime DLLs were authenticated. The
candidate producer ran on Windows image `20261004.326.1` and selected the older
SDK from run `38735490052`, source
`bfbfdca7573c14f4623d2b8b42159abedb99e351`.

The retained PluginHandler workflow ran with helper
`1537a14dc70b33a4ca33d9cceb78013c7dd2e92b` and that same producer/pin:

| Gate run 38756849052 | Job | Allocated image | Result |
| --- | --- | --- | --- |
| [Attempt 1](https://github.com/ThereptileII/Work/actions/runs/38756849052/attempts/1) | `116497424567` | `20261012.333.1` | SDK binding refused; native tests skipped |
| [Attempt 2](https://github.com/ThereptileII/Work/actions/runs/38756849052/attempts/2) | `116499163250` | `20261012.333.1` | SDK binding refused; native tests skipped |

Both authenticated the original producer capture and all 99 immutable consumed
Git blobs/modes. Both selected the newer SDK from run `38618335277`, source
`68c835175bdbc441e57aa1201db8f6ca95a7d1e7`, according to the actual runner image.
The first failure in both was `Bind retained capture to the authenticated
dependency SDK`, with the original exception:

```text
ValueError: Candidate TLS input differs from authenticated producer: runtime/libcurl.dll
```

[plugin-refusals.json](plugin-refusals.json) binds each original log and result
artifact, including the complete CRC checks. The second attempt was one explicit
coordinator-authorized failed-job rerun. No checks were weakened, no application
was rebuilt, and this worker initiated no retry. Neither attempt executed the
native loader/PluginHandler qualification. The earlier native result on the
newer SDK is not transferred to this candidate.

## Exact captured-runtime native pass

[Run 38761760564 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38761760564/attempts/1),
job `116514570605`, passed with helper
`a57098f01053baf283610efe8a9c883b6715904a` against the same frozen producer and
independently reviewed inventory pin. The original artifact `11986286491` is
12,754,835 bytes, SHA-256
`0e8d040a900943fea907a062f22146b2af14acd2df550a5e5547a2188fe97e4a`;
all 5,284 member CRCs passed. Original API/log/report hashes and the complete
thirteen outcomes are in `retainedNativeSuccess` in [custody.json](custody.json).

| Native PluginHandler case | Outcome |
| --- | --- |
| Valid HTTPS; HTTPS redirect | Both installed the two-entry fixture archive |
| HTTP downgrade; local-file redirect; initial HTTP | All refused before archive open; installed files and records unchanged |
| Interrupted but extractable archive | Refused before archive open; installed files and records unchanged |
| Untrusted; redirect to untrusted; expired; redirect to expired | All refused before archive open; installed files and records unchanged |
| Wrong host; redirect to wrong host; trust removed | All refused before archive open; installed files and records unchanged |

All thirteen cases passed. The ignored-download-failure mutant was rejected
because it entered archive installation after a failed download. The HTTP
control was reachable, forbidden HTTP requests were zero, and owned trust/server
cleanup was verified with no cleanup errors. The same maintained workflow also
passed 58 native loader groups, 17,625 deterministic export-forwarder checks and
282 TLS byte-policy checks. It executed neither a vendor DLL nor a plugin
factory; loader, rendering and lifecycle limits remain explicit in the original
summary.

The allocated Windows image was **20261004.326.1**. Both host and subject SDKs
were the exact older producer `38735490052/1`, source
`bfbfdca7573c14f4623d2b8b42159abedb99e351`, original artifact `11976566007`,
ZIP SHA-256 `b41dd68dee3ec29b10aa1cedbde93df258140624f5c127932d1cdecfc104a831`.
Both roles bind manifest
`67ec64a1752cd23e42535c6739e6d60e8b33f05e5ae9eb9f95da3bad6e1c434f`
and toolchain `1cc7c6a0b2579b14f9c04c7f0ae83a5260bc335c941db0f1ae2d5d8433b91f03`.
The new helper separates current-host SDK reprobes from immutable captured
subject authority. This run exercised the **older-host/older-subject** combination;
it provides no native acceptance of a newer-host/older-subject combination.

Independent read-only review compared the embedded inventory and all mappings
with the previously authenticated 1,585-file original capture. Normal and mutant
runtime hash maps match all eleven captured DLLs. All 99 consumed Git blobs/modes
were checked: 98 were unchanged, and the only consumer change was the exact
reviewed `test-plugin-download-guard-windows.py` old/new blob pair. The one
helper-only module `plugin_subject_sdk.py` has its exact Git blob and LF/Windows
checkout hashes recorded; neither allowed probe source changed. All 17 local
input hashes and the immutable workflow hashes matched. The three retained
production source files match the final source archive after explicit CRLF-to-LF
conversion; runtime byte acceptance uses no normalization.

## Captured runtime versus the final package

Eight of the 11 captured runtime DLLs match the final recovery package exactly.
All four maintained TLS DLLs (`libcurl.dll`, `libcrypto-3.dll`, `libssl-3.dll`,
`zlib1.dll`) match. Packaging replaces `concrt140.dll`, `msvcp140.dll` and
`vcruntime140.dll` with different sizes and hashes; the complete before/after
identities are recorded in [custody.json](custody.json). No claim of metadata-only
equivalence is made for these three MSVC DLLs.

This pass authenticates the precise captured source/runtime inputs, including
the four TLS DLLs. It does not establish byte identity or installed acceptance
for all final packaged runtime files. The separate
[full retained qualification](../scrum22-retained-e80-20261015/README.md) preserved
a passing UI/functional prefix but failed during installer trust initialization.
The original baseline remains failed; no release, boat or public-access
acceptance is granted. Neither earlier SDK-binding refusal is overwritten.
