# SCRUM-209: reserved module names and original-case sibling lookup

This bounded change is based on coordinator revision
`3d13f1e93f40099df20cc75edb8d77755b9a270a`. It changes
`Win32PluginTlsImports.h` and adds focused assertions to the existing import and
forwarder tests. The coordinator recorded Jira selection 11338 and owns review
disposition and integration.

The closure now refuses ASCII DOS device stems before `read_local`: CON, PRN,
AUX, NUL, COM1–COM9 and LPT1–LPT9, case-insensitively, including extensions,
multiple extensions and spaces immediately before the first dot. CONIN$ and
CONOUT$ are also conservatively refused. Existing invalid-name and unsupported
forwarder-grammar refusals remain in place. The PE reader already rejects bytes
outside ASCII 32–126 for both import names and forwarders; the superscript digit
aliases cannot enter this closure through that reader.

The callback now receives the original module spelling, with `.dll` appended
only for extensionless names. Its lowercase deduplication key remains unchanged.
The 256-node and 256 MiB graph budgets, legacy TLS policy, and held-handle
callback contract are unchanged. `OriginalPluginTlsGuard.h` and the PE reader
are unchanged; the former already builds its sibling path from the callback's
exact module string.

The naming basis is Microsoft's [file naming documentation](https://learn.microsoft.com/en-us/windows/win32/fileio/naming-a-file),
including reserved stems with extensions and superscript digit aliases. The
lookup correction follows Microsoft's [case-sensitivity documentation](https://learn.microsoft.com/en-us/windows/wsl/case-sensitivity):
case-sensitive directories can distinguish differently cased filenames. The
coordinator verified these primary references; this task made no network request.
The refusal of spaces before the extension and CONIN$/CONOUT$ is deliberately
conservative.

## Portable verification

Both changed test programs were compiled once with GCC, C++17, `-O1 -g -Wall
-Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer` and executed
once with `ASAN_OPTIONS=detect_leaks=0`, `UBSAN_OPTIONS=halt_on_error=1`. All four
compile/run exits were zero; no sanitizer diagnostics were emitted.

| Focused test | Passing checks |
| --- | ---: |
| `win32_pe_imports_test.cpp` | 9,885 |
| `win32_pe_forwarders_test.cpp` | 17,602 |

Existing test assertions were retained. The additions include 240 reserved
normal/delay cases and 48 reserved named/ordinal forwarder cases, each checking
zero callbacks and unchanged PE bytes; unsupported forwarder forms also retain
zero-callback refusal. They cover all listed ASCII device stems, ordinary names
that share their prefixes, superscript import-name encodings rejected by the
reader, and exact-case TLS-bearing sibling lookup through normal, delay and
forwarder paths. Existing alias/cycle, malformed-input and budget checks still
pass.

`portable-lookup-probe.cpp` uses a synthetic PE fixture and an exact-string fake
reader. The same seven-case probe was compiled and run against the exact base
header and the changed header, without running either whole test suite again:

| Probe | Base observation | Changed observation |
| --- | --- | --- |
| `CoM1.dll.backup` normal import, `NUL   .dll` delay import | One callback each; no refusal | Zero callbacks; reserved-name refusal |
| Explicit `MiXeD.DLL`, normal and delay | Requests `mixed.dll`; fake sibling missed; no refusal | Requests `MiXeD.DLL`; sibling read; legacy TLS refused |
| Extensionless `MiXeD`, normal and delay | Requests `mixed.dll`; fake sibling missed; no refusal | Requests `MiXeD.dll`; sibling read; legacy TLS refused |
| Ordinary `COM10.dll` | One callback; no refusal | One callback; no refusal |

The raw compile output files (empty on success), run outputs and compiler
identity are retained byte-for-byte. `verification.json` records source/log
hashes, compile commands, observed exits and derived probe checks. Compiler
outputs and the extracted baseline header remain in ignored local scratch;
no executable or binary fixture is committed.
The compiler's raw version output includes a final blank line; it is preserved
despite the corresponding `git diff --check` notice. Source and derived evidence
pass the whitespace check.

## Inference limits and remaining gates

This demonstrates the portable callback lookup correction and pre-read refusal.
It does not demonstrate a native case-sensitive `LoadLibrary` bypass, device-open
behavior, or general Windows loader closure. Case-insensitive deduplication is
retained; application/system/PATH resolution, dynamic loads and child processes
remain outside this bounded sibling proof. No reparse policy or handle ownership
code changed.

Native Win32/MSVC qualification of the changed production header, actual plugin
loading and held-file behavior remain pending with the coordinator. No native
process, COM port, full application build, CI run, boat operation or external
review was started by this task. Prior native evidence qualifies its recorded
revision only; these portable checks do not qualify a product package or close
SCRUM-209.
