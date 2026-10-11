# Standalone Win32 import metadata reader

SCRUM-209 metadata reader. The Windows original-plugin fallback now uses it
through `Win32PluginTlsImports.h` for a narrow direct/delay legacy TLS refusal
(see [the boundary](../docs/architecture/windows-native-dependency-integration.md#original-plugin-pre-load-refusal)). The reader itself consumes
caller-supplied bytes and performs no filesystem, environment, registry,
network, process or DLL-loading operations.

`ReadWin32PeImports` returns the original ASCII spelling and normal/delay kind
of each module descriptor, preserving duplicates. Both RVA-based and legacy
VA-based PE32 delay names are supported. Imports are optional; no private
exports, wx import, module extension or module allowlist is required. The reader
reports path-shaped and legacy TLS names rather than deciding their policy.

Success means only that the bounded module-name metadata was parsed. It does
not validate import thunks, exports, executable code, signatures, whole-image
loadability, dependency resolution, ABI compatibility or trust. It cannot see
later dynamic loads. No inspected PE is executed. Unknown/malformed metadata
fails with an empty result list, never a partial list presented as complete.

Limits are explicit: 128 MiB input, 96 sections, 1 MiB per descriptor directory,
4096 descriptors total, 4095 bytes per name and 1 MiB aggregate name bytes.
Only x86 PE32 is supported. Directory ranges must have complete contiguous raw
backing within one section or the headers. Ambiguous overlapping sections,
virtual-only backing and unsupported delay attributes are rejected. These
restrictions still require native representative-plugin qualification; the new
runtime caller is not accepted for public release from portable tests alone.

The existing `ChartModulePe.h` is not reused: its private adapter export/import
allowlists and blanket delay-import rejection would reject ordinary plugins.
This reader keeps parsing separate from that policy. Complete runtime closure
still needs a reviewed Windows dependency-resolution contract, file identity
and replacement protection, enforcement at every load/reload entry point and
native installed-host evidence. This work makes no loader-closure claim.

## Focused checks

Run `python tools/test-win32-pe-imports.py` from `opennav-x`. It compiles only
`tests/win32_pe_imports_test.cpp` in a temporary directory. GCC/Clang uses
warnings as errors; `--sanitize --cxx clang++` adds ASan/UBSan. A native MSVC
developer shell can run the same command with `--cxx cl`; native execution has
not been performed for this increment.

Fixtures include ordinary plugin module names, normal and both delay formats,
missing imports, duplicates, path-shaped names, header RVAs, legacy TLS names,
malformed/truncated/overlapping metadata, resource ceilings and deterministic
single-byte mutations. On Linux, both GCC and Clang ASan/UBSan passed. Leak
checking was disabled for the sanitizer run because the host uses ptrace;
LeakSanitizer cannot operate under it. No leak-check pass is claimed.

Optional `--pe <file>` arguments add read-only real binary controls, never
loader execution. Four retained Win32 plugin artifacts from historical
`4b9d1340016193dd707ff78bf5e67eebaa52a54e` were read locally; their complete
normal-import name sequences matched independent GNU objdump output and their
SHA-256 values were unchanged afterward:

| Plugin | Bytes | Imports | SHA-256 |
| --- | ---: | ---: | --- |
| chartdldr_pi.dll | 615424 | 15 | 81d6a61afdf2ce407dae614218e883a0ecfaef0fd5f12d314efc7edb716fb59c |
| dashboard_pi.dll | 862208 | 16 | 2cf70dc85f7e13ca8bfd9ce33f7d4a489c5d4d4e14d5d7a9ef946a9b64115e27 |
| grib_pi.dll | 1338880 | 21 | aa4cd0b57cc2cb022b0391c77ac3baaa1301032a5917e2199538b4a8cdc43b20 |
| wmm_pi.dll | 4570624 | 13 | 65f324af15f512310fe7b1606651a5d803c6a154f71677ca71e545bd54a29097 |

These are parser compatibility controls, not acceptance of those old packages,
their dependencies, or a current application revision. Native MSVC validation
of the new narrow runtime guard remains separate work.
