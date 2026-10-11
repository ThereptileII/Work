# Read-only declared-root plugin inventory

SCRUM-209 groundwork, built on the standalone `ReadWin32PeImports` reader.
This is not wired into runtime loading, CMake, CI, installation or release
acceptance. The issue's native plugin/import closure, exact loaded runtime,
TLS/certificate, packaging and installer lifecycle requirements remain open.

`tools/win32_plugin_inventory.py` accepts explicit plugin/executable roots and
optional search directories. For every import it lists case-insensitive name
matches in the importing file's directory and the explicit directories. It
does not rank directories or choose between duplicate candidates. Both normal
and delay edges are retained, including repeated descriptors. Every discovered
candidate is visited recursively, including all ambiguous candidates; cycles
terminate through the visited-path set.

Example from a development shell (paths are illustrative):

```text
python tools/win32_plugin_inventory.py --root C:/Review/plugins/example_pi.dll --search-dir C:/Review/app
```

The tool compiles only `tools/win32-pe-import-probe.cpp` into an owned temporary
directory with the configured C++ compiler (`--cxx`, `CXX`, or `c++`/`cl`). It
executes that parser, never an inspected DLL/EXE. Captured target bytes are
hashed and sent unchanged through the parser's binary stdin. No target paths
are passed to the parser. The JSON report goes to stdout; compiler messages
go to stderr. The tool does not write inventory files or modify inspected
files, configuration or plugin state.

Each successfully read file has an absolute path, byte count and SHA-256,
including malformed/non-x86 files which subsequently fail parsing. Unreadable
or oversized files have explicit errors without an invented identity. Parser
executable and source hashes identify the reader used. Normal/delay edges
carry candidate paths linking to those file records, or explain why candidate
inspection was prevented by a budget.

Edge states are `unique-candidate`, `ambiguous`, `unresolved`,
`unsupported-name`, and `unavailable-search`. Path-shaped/reserved-character
imports are reported rather than followed. An unavailable search directory
cannot be silently treated as no match. Network/device paths, symlinks and
reparse paths are refused. File metadata is checked around each read and again
after traversal; directory metadata is checked around listing and afterward.
Observed changes make the report incomplete. These checks are not an atomic
filesystem snapshot or protection against adversarial concurrent replacement.

Budgets bound nodes, edges, captured bytes, individual files and entries per
directory. Exhaustion yields explicit errors and an incomplete report, never
silent sampling. The default budgets are recorded in the JSON.

`inventory-observed` / exit 0 means the declared static graph was observed with
one candidate per edge, all nodes parsed and no observed errors. Otherwise the
result is `inventory-incomplete` / exit 1. Neither result establishes the
Windows loader's selected path or a secure/complete runtime closure. Every
report explicitly contains `runtimeEnforcement: false` and
`loaderClosureAccepted: false`.

Unmodelled inputs include process PATH, already-loaded modules, OS system
directories, API-set resolution, SxS/manifests, export forwarders, import thunks,
dynamic LoadLibrary/GetProcAddress calls and child processes. Unused siblings
are not swept. A dependency which is not present in declared directories is
unresolved even if Windows could obtain it elsewhere; directory ambiguity is
reported even if Windows would apply a search preference. No trust, ABI,
signature or legacy-TLS policy is inferred from inventory success.

## Focused validation

`python tools/test-win32-plugin-inventory.py -v` compiles the actual reader once
and runs disposable filesystem fixtures. Coverage includes recursive normal
and delay edges, cycles, exact identity, Unicode/spaced paths, distinct and
identical-byte ambiguities, case collisions, missing OS/API-set dependencies,
path-shaped imports, missing/malformed/non-x86 files, unavailable directories,
symlinks, resource limits, mutation detection, unchanged target bytes and JSON
CLI exit behavior.

Linux focused validation passed. One read-only historical `chartdldr_pi.dll`
control (SHA-256
`81d6a61afdf2ce407dae614218e883a0ecfaef0fd5f12d314efc7edb716fb59c`)
produced one hashed node and 15 unresolved edges when given its retained
profile/plugin directories, which contain no matching dependencies. Its result
was correctly incomplete; this does not qualify that old package.

Native Windows/MSVC and Windows filesystem behavior remain pending. This
tooling does not replace the future runtime gate or its installed-host tests.
