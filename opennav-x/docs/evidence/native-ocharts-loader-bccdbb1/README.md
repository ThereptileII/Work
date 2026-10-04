# Same-candidate native o-charts loader prerequisite

**The bounded native loader prerequisite passed; live chart/plugin and boat
acceptance remain open.** This SCRUM-15/17 audit uses the unchanged original
artifact **11293353341** from [run 37177738716](https://github.com/ThereptileII/Work/actions/runs/37177738716),
[job 111363797689](https://github.com/ThereptileII/Work/actions/runs/37177738716/job/111363797689),
remote `bccdbb11cef827d3b63731fe1e874bc72bf47d2d`, mapped frozen local
`1a6733a1cbcc817aa0f13fa5acc41aac62a54146`.

`original.zip` is **17,496 bytes**, SHA256
`4f4f1023c471b8a9f83369e832c2ba60f52ab967ae01079768a50887afd9dea1`.
The connector's artifact metadata agrees on ID, run, source SHA, length and
digest. All **36** ZIP members passed CRC, safe-path, case-uniqueness and
non-symlink checks before extraction. `api.json` retains connector metadata and
successful completed job steps. `audit.json` binds every original member.

| Original result | Independently checked observation |
| --- | --- |
| Native Win32 loader | **38 distinct PASS groups**, agreeing with `native-tests.json` and `summary.json`. These cover actual locking/hashing, changed inputs, path/reparse refusal, missing exports, bind/status rejection, bound-module lifetime and callback/fallback mechanics. |
| Exact source | All **15** recorded source/input hashes and sizes match frozen local `1a6733a` with Windows CRLF checkout bytes. This includes the actual loader/fallback implementations, binding definitions, harness and inert fixture source. The recorded standalone `skager-ocharts-loader.yml` input is also matched; this run executed the corresponding job in `opennav-baseline.yml`. |
| Python preparation contracts | Original logs show **17** producer-prefix tests, **6** source-cache tests and **17** adapter-preparation tests, each ending `OK`. No rerun occurred in this audit. |
| Build wiring | Original log reports parse/order, first-build and exact-reuse checks plus **17 rejection cases**. This log explicitly does not claim native adapter compilation. |
| Fixture boundary | Five harmless DLL variants are reported. Original event logs show attach/bind/status/detach behavior; receipt flags keep vendor and plugin-factory execution false. |

The invalid-PE and missing-export errors in `native-tests.log` are expected
negative cases with corresponding PASS groups. The final unload-refusal case
uses an injected reserved **non-module** address: it does not prove refusal to
unload a genuine loaded module. The report explicitly keeps
`nativeProductAcceptance=false`, `vendorExecuted=false`,
`pluginFactoryExecuted=false` and `productAcceptance=false`.

This artifact does **not** qualify the real adapter's OpenCPN import ABI,
renderer initialization, full-app Standard/Safe selection, plugin Init/DeInit,
encrypted-chart rendering, licensing, or physical equipment. Vendor helper and
closed-library boundaries are unchanged. Reported executable/runtime/fixture
and vendor binary hashes remain runner observations: those binary payloads are
not in this archive and were not independently rehashed here. Exact packaged
DLL identity, dependency/helper closure and actual native/boat runtime remain
separate requirements.

The seven adjacent report/log files are byte-for-byte copies from the original
ZIP for convenient review. No application source, test, build, CI execution,
remote session or boat state was changed by this evidence-only audit.
