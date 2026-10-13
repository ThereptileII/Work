# Windows maintained dependency integration

SCRUM-209 connects the reviewed OpenSSL 3.5.9, zlib 1.3.2 and curl 8.22.0
source builders to the disposable Win32 integration build. It does not change
the pristine build, package an installer or modify an installed OpenCPN tree.

After the pinned upstream dependency bootstrap,
`build-pristine-windows.ps1 -Integration` builds OpenSSL, then zlib, then curl.
The zlib headers, import
library and DLL are copied into the disposable `cache/buildwin` only after
their hashes and byte sizes match the zlib producer manifest. The curl builder
then independently verifies both dependency manifests and inputs, builds and
tests curl, proves its Win32 import closure with `dumpbin`, and publishes the
matching full header set, import library and DLL to that cache.

The narrow `model/cmake/Curl.cmake` patch preserves the pinned imported-target
layout. It removes only the legacy CA bundle and OpenSSL 1.0 DLLs from that
disposable tree's CMake install list. The maintained curl manifest must already
show imports of `libssl-3.dll`, `libcrypto-3.dll` and `zlib1.dll`, and reject
`ssleay32.dll` and `libeay32.dll`, before application CMake is invoked. The
existing pinned CMake rules install the OpenSSL 3 DLLs and zlib DLL; the curl
rule installs the maintained libcurl DLL.

After installation, orchestration copies the three producer manifests into the
install root and rehashes all four installed DLLs against their manifest
records. It fails if either legacy TLS DLL appears. It never deletes legacy
DLLs from a user installation. The installer now also scans the complete
staged application after preserving existing plugins and other additions.
Install/update/repair refuse a candidate containing either legacy TLS DLL,
including nested or case-varied names, before self-test and state publication.
Original plugins, user files and the prior active generation remain intact.
Exact recorded rollback is a recovery operation and deliberately retains its
original bytes; this does not qualify the older runtime for public release.
The candidate gate also parses bounded x86 PE normal and delay imports for
executables, plugins and their resolved local dependencies (including modules
with non-DLL extensions). Missing application runtimes and legacy TLS imports
are rejected even when the old DLL itself is absent. OS dependencies resolve
through Windows' known x86 system directory, not a caller-supplied environment
path. This is a static import check; it cannot qualify arbitrary plugin
`LoadLibrary` behavior or replace native plugin/runtime acceptance.
Auxiliary non-x86 DLLs retain their import-scan exception, but non-x86
executable helpers are refused: the 32-bit host can start a separate process,
and the x86 parser cannot qualify that process's dependency closure. This
refusal preserves the original files and applies before candidate publication.
The added PE32+ executable fixtures have portable layout validation; executing
their guard assertions under PowerShell, including native Windows, is pending.

The actual installer functions pass deterministic PowerShell parser/closure
fixtures on Linux. Native PowerShell 5.1 in both host bitnesses, retained-plugin
install/update/repair refusal, exact rollback and real package qualification
remain required.

The existing early native private-chart module check also compares its actual
mapped `libssl-3.dll` and `libcrypto-3.dll` paths and observed byte identities
against the installed OpenSSL build manifest. Missing/duplicate modules, a
different directory even with matching bytes, changed hashes, or inconsistent
output/cache provenance fail the check. The manifest and installed DLLs must
remain unchanged across the observation. This adds no new application entry
point and leaves dependency producer inputs unchanged. Portable negative
fixtures cover the comparison; exact-candidate native execution remains pending.
This is early-host loader evidence, not ordinary external-plugin session or
TLS-traffic qualification, and post-exit file hashes are not in-memory hashes.

Source identities remain in `tools/windows-openssl.lock.json`,
`tools/windows-zlib.lock.json` and `tools/windows-curl.lock.json`. Native output
is retained by the producer scripts under `evidence/local`, including
`windows-openssl-native-output.log`, the zlib 1.3.2 evidence directory and
`windows-curl-native-output.log`; the outer integration transcript and curl
orchestration log retain sequencing and failures.

This change has PowerShell syntax and patch-stack validation on Linux only.
Native MSVC compilation, upstream producer tests, application link/import
closure, runtime TLS, installer and release acceptance still require native
Windows evidence. SCRUM-211 separately governs application certificate and
redirect policy; changing the source-built dependency alone does not establish
secure download behavior.

## Early Poedit prerequisite

SCRUM-255 now validates both Poedit Gettext tools before any expensive native
preflight/dependency work, using bounded acquisition through the existing
Chocolatey provider only when the build explicitly permits installation.
Exact files are reprobed before CMake, which receives absolute tool paths.
See [the Gettext boundary and focused proof](windows-gettext-prerequisite.md).
This prevents a masked stock `win_deps.bat` package failure from consuming the
maintained dependency build first; it does not bypass those dependency gates.


## Original plugin pre-load refusal

SCRUM-209 comment 11287 identified roots outside the staged installer app:
pinned OpenCPN `PluginPaths::InitWindowsPaths` puts `GetWinPluginBaseDir` first,
and `AbstractPlatform::GetWinPluginBaseDir` resolves configured or per-user
plugin directories. `PluginLoader::LoadPlugIn` is the common initial/reload
entry point. Its existing integration patch invokes
`LoadPluginWithPresentationFallback`, including the ordinary original load
when no presentation adapter applies or an adapter cleanly refuses.

On Windows that original-load branch now constructs `OriginalPluginTlsGuard`
before `wxDynamicLibrary::Load`. It reads the selected plugin while holding a
read handle without write/delete sharing across the load call. The production
byte policy refuses direct and declared-delay imports of `libeay32.dll` or
`ssleay32.dll`, including case, qualified path, extension-less and trailing-dot
aliases. Unknown/malformed/non-x86 metadata, unreadable files and files outside
the reader's 128 MiB limit refuse the original load. The ordinary OpenCPN
unloadable-plugin path receives the failure, with the detailed refusal logged.
No plugin/profile file is rewritten or deleted. This runs for all discovered
original paths without copying the upstream configuration resolver into Setup.
The current bounded declared-import/forwarder walk inspects both the original
plugin directory and the actual host executable directory. The latter is
obtained from `GetModuleFileNameW(nullptr)`, not inferred from the plugin path.
Both candidates are inspected even when a same-name sibling exists; one shared
queue traverses chains alternating between the two roots. Candidate handles and
native directory identities remain retained through original-module custody.
Extension-less names resolve to `.dll`; case-folded names plus directory
identity deduplicate repeated declarations and cycles. The walk refuses
unsupported names, unreadable or redirected candidates, malformed x86 PE and
exhausted node/byte budgets. Missing candidates are outside these two roots,
not evidence that Windows cannot resolve them elsewhere. No process-wide DLL
search setting, loader flag, original discovery path or file is changed.

The private presentation adapter retains its separate compiled exact-byte
identity/lock/compatibility gate. Successful adapter selection does not execute
the original and does not invoke the new original-import policy. The pristine
OpenCPN Legacy executable is unchanged. Ordinary maintained TLS, vendor,
optional/delay and import-free plugin metadata remains permitted by this
narrow policy; passing it is not a statement of plugin compatibility or trust.

This is **partial enforcement**, not closure of SCRUM-209. It does not traverse
system/PATH/redirected or already mapped dependencies, dynamically selected
DLLs or executable helpers, validate activation manifests, or prove arbitrary
loaded-module identities. Declared export forwarders retain the separately
documented conservative grammar and PE bounds. It
does not stop previously mapped code or provide a hostile-filesystem sandbox;
path/ancestor redirection and filesystem replacement semantics need separate
native qualification. Exact bytes of a known-good plugin are not an approval of
its future dynamic behavior. The installer guard and read-only inventory/module
observer retain their distinct existing scope.

Focused portable checks exercise the actual reader and policy for normal and
both delay formats, aliases, malformed/foreign inputs, ordinary controls and
unchanged input bytes. The local graph tests cover two-hop indirect TLS imports,
cycles, repeated names, malformed/missing siblings and the node bound. The
native harmless load controls in
`tests/windows_ocharts_loader` additionally compare successful unguarded loads
against refusal before original DllMain, repeated loads, clean adapter fallback
and lock lifetime. The 42-group predecessor passed focused native run
[38146240771](https://github.com/ThereptileII/Work/actions/runs/38146240771)
at source commit `840e1932fd2531f508d43459c89d39bce98fb496`,
but its benign helper was explicitly preloaded. The 43rd group uses a
unique, previously unmapped benign helper without an executable-directory
duplicate. A scoped process current-directory search change allows the actual
native original load while an explicit production guard holds both inspected
files; it checks write/delete sharing violations and the mapped sibling path,
volume/file index and hash, then unloads and restores process state. That new
first-mapping control passed all 43 groups at `e615446e` in
[native run 38159170047](https://github.com/ThereptileII/Work/actions/runs/38159170047).
It is direct
guard-class/native-load evidence, not a full application fallback qualification.
The first attempt, native run
[38158753749](https://github.com/ThereptileII/Work/actions/runs/38158753749) at
`1f756424`, compiled and passed the prior 42 groups but stopped before new-group
mapping at an incorrect zero-capacity `GetDllDirectoryW` fixture assertion.
The bounded string-query repair then passed the exact-revision run above;
the failed attempt remains retained without being interpreted as a product
loader failure. Source and native receipts are linked from
[the scope record](../evidence/2026-10-11-scrum209-local-import-closure.md).

Before acceptance, qualify the exact integrated native candidate with default
per-user, configured, portable and
app-local roots; initial/reload and successful/refused adapter paths; existing
supported plugins and OpenCPN unloadable diagnostics; observed normal/delay/
transitive/dynamic/helper identities; TLS/certificate failure behavior; and
install/update/repair/rollback/uninstall preservation. Broader loader enforcement
and native/boat acceptance remain open. This source change does not qualify the
frozen Staging artifact or authorize public beta.


### Original-module inspection custody (SCRUM-209, 2026-10-12)

The original-load fallback transfers its inspected-file owner to the OpenCPN
plugin container. Custody now spans later factory-export forwarding, declared
delay imports and checked unload/detach callbacks. File protection ends only
after successful checked module unload. Failed unloading retains the module and
pins; destruction of an uncertain container deliberately keeps them until process
exit and logs the uncertainty. This does not enforce arbitrary dynamic loads or
prove a complete Windows dependency closure.

Container layout uses a narrow `SKAGER_PLUGIN_CUSTODY` usage requirement on the
shared model interface, consistently visible to model, GUI and headless readers.
It must not depend on source-local `OPENNAV_X`. Pristine builds keep their original
layout/destructor. [Source, independent review, focused Linux probes and pending
native cases](../evidence/scrum209-plugin-custody-20261012/README.md) distinguish
this repair from earlier native evidence, which is not relabelled.
