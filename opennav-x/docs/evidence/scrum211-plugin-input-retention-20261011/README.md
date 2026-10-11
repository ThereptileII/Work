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
