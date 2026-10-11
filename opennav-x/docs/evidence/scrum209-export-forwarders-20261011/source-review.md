# SCRUM-209 declared export-forwarder increment

Selection: Jira comment 11322. Base: coordinator
`c63dee40445f4b084468e3076c7d33a90eea118a`. The previous 43-group native pass
at `e615446e2c1f547fd2e01d87bcf0621da0c8500a` is retained unchanged; it does
not cover export-only dependencies. Material issue comments 11287, 11296,
11297 and 11311 remain applicable limitations and evidence.

The pure reader previously examined only normal/delay import descriptors.
`old-policy-control.cpp` is the exact synthetic export-only regression compiled
against the base reader/policy before edits. Its log records the expected exit
1: a declared `libeay32.legacy_tls_fixture` dependency was admitted. It is a
source-policy failure control, not proof that any actual plugin executed or
loaded a legacy DLL. Reproduce with base headers in an isolated checkout:

```sh
TMPDIR="$PWD/opennav-x/evidence/local/.tmp" g++ -std=c++17 -Wall -Wextra -Werror -Iopennav-x/src opennav-x/docs/evidence/scrum209-export-forwarders-20261011/old-policy-control.cpp -o opennav-x/evidence/local/old-forwarder-control
opennav-x/evidence/local/old-forwarder-control
```

The implementation follows Microsoft's [PE export-address-table specification](https://learn.microsoft.com/en-us/windows/win32/debug/pe-format#export-address-table).
An EAT entry inside the export data-directory range declares a forwarder;
its ASCII string and terminating NUL must both stay in that range. The parser
checks complete raw backing, bounds the directory at 1 MiB, EAT at 65,536
entries, forwarders at 4,096, individual strings at 4,095 bytes plus NUL and
combined import/forwarder text at 1 MiB. Arithmetic uses wide checked extents.
Malformed inspected metadata returns no partial import or forwarder lists.
Zero/hole and nonforwarded EAT entries are not dependency declarations; they
are not newly validated as code or proof of image validity.

`Win32PeImports.modules` keeps its original normal/delay meaning. The trailing
`forwarders` collection holds original raw declarations. The TLS policy accepts
only one module/symbol separator, with a nonempty symbol or decimal ordinal
1–65,535. Module names reach the same legacy-basename policy and bounded
same-directory graph as imports. Multiple-dot/explicit-`.dll` declarations
are unsupported grammar and are refused before load; this does not label them
invalid Windows images or invent a Windows separator rule. Missing sibling
modules still leave external loader search outside this proof.

`gcc.log` and `clang.log` retain exact compiler/environment/commands and passing
results: 9,366 existing import-reader checks and 17,496 focused forwarder checks
under each compiler with ASan/UBSan. LeakSanitizer is disabled on this ptrace
host; address/undefined checks remain enabled. Coverage includes direct aliases,
named/ordinal and mixed import/forwarder chains, benign and absent siblings,
cycles/deduplication, unsupported grammar, directory/string/EAT/section/RVA
boundaries, output/name/node budgets, truncation and deterministic mutations.
Runner Python syntax and diff whitespace also pass. No full application build
or broader suite was run.

Prepared native scope is 51 loader groups (43 retained plus eight forwarder
groups), plus the pure forwarder test compiled by MSVC. The native README
describes exact controls. Microsoft's [MSVC EXPORTS reference](https://learn.microsoft.com/en-us/cpp/build/reference/exports?view=msvc-170)
documents the named and `other_module.#ordinal` definitions used by these
fixtures. Simple benign named/ordinal forwarding must resolve
the inspected sibling under live original/sibling guard handles, with mapped
path, pre-mapping file identity, hash and write/delete lock checks. Legacy and
unsupported declarations must be refused before original/sibling initialization;
unguarded harmless controls establish loadability. Ambiguous benign probes
record actual Windows behavior without weakening policy. Native source and
fixture identities remain required by the runner receipt.

Native compilation/execution has not been performed. No workflow dispatch,
push, vendor factory, full application, boat or physical operation occurred.
This increment changes production headers, so an application package rebuild
and exact-artifact acceptance are required for product qualification. Root
coordinates integration and the existing tiny native gate. Broader loader
search/dynamic/helper paths, full application fallback, maintained TLS and
certificate behavior, installer lifecycle, source/notice compliance and boat
acceptance remain open; this does not close SCRUM-209.
