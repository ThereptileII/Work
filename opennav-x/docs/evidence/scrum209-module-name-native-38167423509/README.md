# SCRUM-209 native module-name compatibility audit

[Run 38167423509](https://github.com/ThereptileII/Work/actions/runs/38167423509),
[job 114552775936](https://github.com/ThereptileII/Work/actions/runs/38167423509/job/114552775936),
at exact commit `6445d67e27c6b6f4254849556efea28248204711`, attempt 1,
**passed** on the disposable Windows Server 2022 runner. The retained native
source/fixture gate compiled with **MSVC 19.44.35229.0, Win32/x86**, passed
**17,602 export-forwarder reader checks**, and completed **51 loader groups**.
Run/job/artifact ownership, native summary and native receipt agree. The resource
Python identity job was skipped; it is not a passing result.

The changed `Win32PluginTlsImports.h` refuses reserved device stems before
sibling lookup and retains original module-name case for the sibling callback,
lowercasing only the deduplication key. Its exact native bytes are verified in
`audit.json`. The changed forwarder-reader test was compiled and executed by
MSVC: its new reserved-module, ordinary-name and mixed-case callback assertions
are included in the 17,602 native checks. These are deterministic reader and
callback checks; they do not establish behavior on a native case-sensitive
filesystem or a new Windows loader bypass.

The **9,885 import-reader checks were not compiled or executed by this native
job**. They remain the separately retained earlier Linux GCC ASan/UBSan result
in `raw/prior-portable/`. All five checked source identities in that prior
receipt match this candidate's Git objects, including the changed import test
and module-name header. The old/fixed seven-row portable lookup probe and its
original logs are retained without rerunning them. Native and portable counts
are separate; neither count is a loader-group count.

The loader still reports **51 groups, not 53**. Their names and order exactly
match [the previous native 51-group gate](../scrum209-forwarder-native-38164763565/README.md).
The complete native `loader_test.cpp`, fixture CMake, `OriginalPluginTlsGuard.h`
and `Win32PeImports.h` are unchanged against that gate. The new reader cases
therefore supplement the retained loader assertions without replacing or
weakening them.

The first-map control proves the private helper was absent from the application
directory and not preloaded, then maps it while the production guard retains
original/sibling write and delete locks. The passing source assertions require
the mapped path, file identity and hash to match the inspected file, recheck the
locks during mapping, unload both modules, preserve hashes, release the locks,
restore the process current directory and verify the DLL-directory setting is
unchanged. The retained native observation is:

- Private filename: `original\helper_unmapped.dll`.
- Volume serial: **771785911**; file index high:low: **65536:11645**.
- SHA-256: `42cc280d1e35c36e7b9a627e08b41b462dc59f11b217ffcac11d1c11ddca8e99`.
- Event bytes: exactly `attach\ndetach\n`.

The guarded benign named/ordinal forwarders likewise retain original/sibling
write/delete locks through resolution, compare mapped path/identity/hash,
confirm unload, unchanged hashes and released locks, and restore search state.
Legacy/unsupported cases remain refused before their original/sibling DllMain;
their separate unguarded private inert probes establish loadability without
relaxing production grammar. The multi-dot probe again resolves the exact
private filename `forward.clean`, value **409**; this does not establish a
general forwarder separator rule. The final invalid-module-handle group passed
with error **126**, using an injected non-module address rather than a genuine
loaded-module unload failure.

`audit.json` verifies all **24 native summary source identities** against the
exact dispatched Git revision after LF-to-CRLF conversion, retaining both
lengths and hashes. Against the preceding native 51-group source set, only the
module-name header and forwarder-reader test differ. The build log records
**zero errors and 17 C4702 unreachable-code warnings**, all in inert
`fixture.cpp` targets. The native test diagnostics include expected negative
load/import failures; the final native receipt and all 51 completed groups pass.

All nine constructor receipts match the immutable source allowlist and the
summary fixture after-hashes/lengths, including their reserved metadata offsets,
121-byte reservations and unchanged-other-bytes assertions. The archive retains
receipts rather than PE binaries; this audit checks exact constructor source
and receipt consistency and does not independently reparse native binary bytes.

Artifact **11712742676**, already downloaded by the coordinator, is **29,664
bytes**, with API/local SHA-256
`7673bc0234588cc6e7089cdf1e23d4fb37b62bff6aeafd949944dab59704042b`.
All **50 ZIP members** pass CRC, unique case-fold path and bounded plain-text
entry checks. `raw/` retains those members, three terminal API responses, the
job log and twelve unchanged prior portable evidence files: **66 byte-exact
records**, **250,470 bytes** total. `audit.json` inventories their paths, lengths
and hashes; `.gitattributes` preserves original bytes. No ZIP or PE binary is
committed.

This passes the bounded native source/fixture mechanics gate only. The native
product/vendor/plugin-factory acceptance fields remain false. It does not close
SCRUM-209, qualify a production package or real adapter, establish external
loader/dynamic/helper closure or general module identity, or accept full app
fallback, encrypted charts, licensing, maintained TLS/certificates, installer
lifecycle, source/notice compliance or boat behavior. Earlier failed evidence
remains retained. This audit only inspected retained records and local Git
objects: no download, rerun, build, native execution, CI dispatch, network,
Claude review, product-document change, deployment or boat operation occurred.
