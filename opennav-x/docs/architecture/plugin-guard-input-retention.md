# CI-only PluginHandler SDK/runtime retention

SCRUM-211 selection: Jira comment 11330, 2026-10-11. This adds capture for the
existing [native source-slice runner](plugin-download-guard.md). It does not wire
that runner, qualify the application, or change trust. No build, source download,
DLL load, application launch, certificate operation or network request is part
of capture. The native producer must already have built its production install.

`tools/retain-plugin-guard-inputs.py` runs only in the GitHub-hosted Windows
`windows-integration` job in `ThereptileII/Work`. Explicit producer SHA, run ID,
attempt and Win32 architecture must match the live environment; checkout HEAD,
tracked source and production `OpenNavBuild.h` must match that SHA. A local,
boat, other job, stale source or substituted architecture refuses.

The helper calls the existing `native.dependency_inputs(root, bundle,
provenance)` before capture and again before publication. The authenticated
maintained producer remains the authority for curl headers/import and the four
maintained TLS DLLs. The existing runner's `bind_tls` compares the captured
bytes against that authority. Capture never creates replacement bundle
provenance or calls a dependency reprobe/build/download.

| Inventory input | Actual producer source |
| --- | --- |
| `curlInclude`, `curlImport` | Authenticated `build/windows-curl-8.22.0/install/include/curl` and `lib/libcurl.lib` |
| `archiveInclude` | `build/integration-source/cache/buildwin/include/archive.h` and `archive_entry.h` |
| `archiveImport` | Exact `LibArchive_LIBRARY:FILEPATH` from `build/production-windows/CMakeCache.txt`; only that cache's `archive.lib` or `libarchive.lib` |
| `wxRoot`, `wxLib` | Exact production wx 3.2.8 root, `lib/vc14x_dll`, and `mswu` configuration from the same Win32 CMake cache |
| Runtime | Required runner DLLs and their required same-install DLL imports, directly inside `build/production-install` |

Archive include selection must also match `LibArchive_INCLUDE_DIR:PATH`. There
is no library-name preference or fallback when CMake selected another input.
wx retention projects its include tree, immediate import `.lib` files and
`mswu` setup headers. It excludes wx SDK DLLs, PDBs, debug setup directories and
unrelated SDK files. Header/resource suffixes have an explicit allowlist;
unexpected include contents refuse rather than copy opaque data. This records
actual selected wx/libarchive bytes; it does not independently authenticate
their provenance. That review is still necessary before an external pin.

Runtime begins with the existing runner's `DLLS`, including the maintained TLS,
wx base/core, archive and MSVC DLLs. Capture derives `dumpbin.exe` from the
single production `CMakeCXXCompiler.cmake` x86 MSVC selection and records its
hash and selection-file hash. It checks each selected DLL is Win32 and reads
`dumpbin /DEPENDENTS`, following imports only to direct DLLs in that same
production install. The runner's existing system-DLL/API-set exceptions apply.
An unresolved import, non-allowlisted driver, missing required DLL or ambiguous
compiler selection refuses. The candidate is never executed. No directory
sweep copies plugins, executable files, profiles, credentials or private keys.

The output directory must be new, have plain existing ancestors, and sit
outside the producer's `opennav-x` root. Input paths reject links, reparse
points, hardlinked selected files and case aliases, including selected parent
directory names. Limits are 20,000 retained files, 40,000 inspected entries per
bounded walk/parent check, 128 runtime DLLs, 256 MiB per file, 2 GiB total data
and 8 MiB per JSON report. Each metadata inspection has a 30-second timeout and
rejects output exceeding 1 MiB. Files are streamed, compared with their acquired
metadata/hash, copied exclusively, and hashed again at the destination. Capture
then repeats selection, runtime closure, source identities/hashes, production
controls, host/SHA checks and dependency authentication before publishing.
These are snapshot-consistency checks, not a lasting native custody claim.

Output is:

```text
plugin-guard-inputs/
  data/sdk/{curl,archive,wx}/...
  data/runtime/*.dll
  inventory.json
  capture.json
```

`inventory.json` has exactly the existing runner's five fields: `schema`,
`candidateCommit`, `inputs`, `runtime`, `files`. Each file record is only
`bytes` and `sha256`. There are no receipt extensions inside the inventory or
files outside its closed SDK/runtime projection. `capture.json`, outside the
data root, records source-path mappings, producer SHA/run/attempt, authenticated
dependency authority, production controls, inspector identity and DLL imports.
It explicitly reports `captured-not-qualified`, requires an independent pin,
and denies native/product/boat acceptance. A failed/partial output is not reused;
without final `inventory.json` it is not a completed capture.

After the existing production build, the producer invocation is:

```powershell
python opennav-x/tools/retain-plugin-guard-inputs.py `
  --root opennav-x `
  --producer-commit "$env:GITHUB_SHA" `
  --producer-run-id "$env:GITHUB_RUN_ID" `
  --producer-run-attempt "$env:GITHUB_RUN_ATTEMPT" `
  --architecture Win32 `
  --dependency-bundle "$env:GITHUB_WORKSPACE/dependency-bundle" `
  --dependency-bundle-provenance "$env:GITHUB_WORKSPACE/dependency-provenance.json" `
  --output "$env:GITHUB_WORKSPACE/plugin-guard-inputs"
```

The coordinator owns producer wiring and upload of the complete directory.
An independent reviewer must audit the actual producer artifact, SDK origins,
runtime imports and closed inventory, then supply its externally held SHA256.
The generated `inventorySha256`, or a hash merely fetched alongside the same
artifact, cannot approve itself. A later runner takes `--retained-root
plugin-guard-inputs/data`, `--retained-inputs plugin-guard-inputs/inventory.json`
and that independently reviewed `--retained-inputs-sha256`, plus its existing
authenticated maintained bundle arguments. It still performs its own native
reprobes, compilation, TLS/caller assertions, mutation control and cleanup.

The focused [portable evidence](../evidence/scrum211-plugin-input-retention-20261011/README.md)
uses inert files and injected read-only import results. Actual Windows capture,
artifact audit, independently pinned native runner execution, production
PluginHandler routing/ABI and installed/boat acceptance remain pending. This
capture is neither a distribution nor a qualification pass.
