# SCRUM-211 input-capture preparation, 2026-10-11

Selected in Jira comment 11330. Isolated source base:
`8e93c5cccf1aa5033f2201325c68b8f3f20c3dd0`. This evidence covers only the new
CI-only capture helper and its portable contract tests. Existing native runner,
workflows, product source and project status are unchanged.

The [capture contract and exact producer invocation](../../architecture/plugin-guard-input-retention.md)
explain the closed SDK/runtime projection, actual production CMake binding,
authenticated TLS comparison and mandatory independent external inventory pin.

Focused checks:

```text
python tools/test-retain-plugin-guard-inputs.py
python tools/test-plugin-download-guard-windows-tests.py
```

Run from `opennav-x`, with a private temporary directory. Observed stdout is
retained unchanged in `capture-tests.txt` (29/29, saved from the completed tool
result without an extra rerun) and `runner-contract-tests.txt` (19/19, redirected
directly); `checks.json` records their hashes and the source hashes examined.
Tests use inert source files and
mock only native metadata inspection/CI adapters; they never load DLLs, run a
compiler/application, download inputs, modify certificate trust or contact the
boat.

Capture tests cover runner-schema/copy compatibility and independent pin
enforcement; exact archive choice with both import filenames present; required
runtime dependency closure and exclusion of unrelated DLL/profile/PDB files;
producer/Win32/wx/CMake/header mismatch refusals; authenticated TLS mismatch;
missing/unresolved/driver imports; file/directory case aliases, links and mocked
reparse refusal; byte/count/report limits; source and retained-copy mutation;
fresh output; final authority recheck before publication; the real dumpbin
selection/parser adapter with an inert mocked subprocess; and inspector/tool
mutation. The adapter test proves the command is metadata-only `/DEPENDENTS`,
not candidate execution. It does not prove native Windows parsing/loading.

No native capture has run. No actual SDK inventory has been reviewed or pinned.
Native PluginHandler guard execution, application/plugin qualification,
installed runtime recovery, source compliance and signed/boat acceptance remain
open. The helper's generated hash and informational receipt cannot close them.

## 2026-10-13 retained B0 import finding

The failed prepackage B0 run [38442801510](https://github.com/ThereptileII/Work/actions/runs/38442801510)
at `2c61369080c358d5ef9b8952d6f2881b52891b13` retained artifact
`11840836434`. Its outer ZIP SHA-256 is
`fa0357a26605f29403bde6602cdedf5b91f8ac10345956ad44f4cc5ae201a8a7`;
the inner `COMPILED_RECOVERY.zip` SHA-256 is
`c4ff53f1441efbd2dfc69f2f452ece340ff95a31862001883ca95f343a8ca70e`.
The retained `build/production-install/libcurl.dll` is 638,464 bytes, SHA-256
`238b82c6ff7463edd5189fea337e8876094d4380843fa93d600302e56bc4c8c0`.
Read-only PE import inspection (`llvm-objdump -p`) showed its import
`WLDAP32.dll`. Recursing through the nine required production runtime seeds
found ten installed DLLs, including `liblzma.dll`; `WLDAP32.dll`
was the sole import outside the existing approved Windows system names,
API-set names and installed DLLs. It was absent from the retained production
install. The capture helper and native source-slice runner now share the narrow
plugin system-DLL list with `wldap32.dll`. Capture refuses a production-install
copy of that name, and the runner refuses a retained runtime mapping for it;
the probe's binary directory receives only checked retained DLLs.

The focused inert-file capture suite passes 37/37 on Linux, including the
observed B0 `libcurl.dll` import set, unknown-import refusal and local/retained
`WLDAP32.dll` impostor refusals. These tests do not execute the B0 binary.
Native capture and native plugin behavior remain pending.
