# SCRUM-209 forwarder fixture repair: native link failure retained

[Run 38162532440](https://github.com/ThereptileII/Work/actions/runs/38162532440),
[job 114538271808](https://github.com/ThereptileII/Work/actions/runs/38162532440/job/114538271808),
at `f5ebd02095bd6858802e48adc141cd2a31033550` **failed**. The actual Win32 MSVC
build reached nine forwarding targets, then reported 17 warnings and 18 linker
errors in 22.34 seconds. The eight forwarding original DLLs and `forward_helper`
each produced LNK2001 and LNK1120. Named targets reported unresolved
`fixture_value`, `legacy_tls_fixture`, `dll.fixture_value`, or
`clean.fixture_value`; ordinal/indirect cases reported `forwarded_probe`.
The 51 native loader groups and native pure-forwarder test were not executed.
The separate preparation/build-wiring checks passed, including 17 rejection
cases; these do not establish native loader acceptance.

`raw/` retains byte-exact API receipts, the full native compile log and command,
configure log/command, failed runner summary and the compact preparation logs.
`artifact-verification.json` verifies artifact `11710439296`: 15,038 ZIP bytes,
API digest and locally computed SHA-256 both
`69287fdf4f935d06c0f75e26a1a3085daee40cb7b0b6d18da5da2df469dfef68`,
and CRC success for all 14 ZIP members. The ZIP and binaries are not committed.

The nine `.def` files in `source-derived-defs/` reconstruct the deterministic
`file(WRITE)` strings in that exact commit. They were **not present in the
downloaded artifact**. The base CMake file converted to Windows CRLF matches the
failed runner's recorded 7,072 bytes and SHA-256
`31cf06a5a0f9a96c583b9f2816f92a1fc7367dafed9dcf470cac570dff444a23`.
The full compile log identifies these generated definitions and their errors.
Although Microsoft's [EXPORTS reference](https://learn.microsoft.com/en-us/cpp/build/reference/exports?view=msvc-170)
describes named/ordinal forwarding, this evidence does not establish why the
linker interpreted these definitions as unresolved symbols.

The repair constructs **inert PE export metadata** after an ordinary MSVC local
alias build. It does not test MSVC's emission of forwarding syntax. The fixed
nine-file allowlist supplies the same exact targets as the native cases. Each
seed must contain the original long reserved export name and a unique
`forwarded_probe` EAT slot, with distinct ordinals pointing at the same local
placeholder code. The reservation is replaced by a shorter still-sorted export
name followed by a NUL-terminated forwarding target; only the probe EAT RVA is
redirected. The string including NUL remains wholly within the raw-backed
export-directory interval, as required by the [PE export-address-table specification](https://learn.microsoft.com/en-us/windows/win32/debug/pe-format#export-address-table).

The constructor refuses unexpected architecture, signatures/checksums, section
or table overlap, writable/executable export data, an overlapping DLL name or
other data directory, changed placeholders, shared probe ordinals, existing
forwarders and repeated construction. It checks complete file size and every
byte outside the two bounded metadata ranges remains identical. Executable
sections and ordinary compiler-produced import data are retained. It validates
all nine inputs before the first write and persists failed/partial receipts.
Native runs must retain `fixture-generation.json` with hashes, offsets and exact
targets for all nine files, then use those resulting hashes for unchanged-byte
checks throughout the loader tests.

Portable verification passes **15 focused test methods**, including all nine
targets, malformed inputs, all-input prevalidation, repeat refusal, a simulated
partial write failure with retained failed receipt, and untouched unrelated
files. `portable-constructor-tests.log` is the actual output.
`portable-construction.json` and `portable-reader.log` record nine synthetic
PE32 before/after pairs consumed by the unchanged production `ReadWin32PeImports`:
each original has no forwarding declaration, and each constructed image yields
exactly its expected raw target with no ordinary/delay module declarations.
`portable-reader-probe.cpp` is the tiny compiled read-only probe. The synthetic
bytes are data models, not executable Windows fixtures. Compilation used
`g++ -std=c++17 -Wall -Wextra -pedantic -Iopennav-x/src` with private temporary
storage; the first attempt encountered the host `/tmp` quota and the identical
compile succeeded with the private `TMPDIR`. The runner's Python sources also
pass syntax parsing.

No production header, loader assertions, 51-group receipt, ambiguous-grammar
policy, actual Windows `GetProcAddress` positive control, or retained old-source
negative control changes. All code execution and module identity/unload checks
still belong to the native loader gate. **Native compilation, construction and
Windows loadability of the repaired seeds remain pending.** No application,
vendor factory, boat, deployment or release qualification is claimed.
