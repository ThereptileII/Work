# SCRUM-259 native host-loader guard gate

This small MSVC Win32 project compiles the actual `OChartsModuleLoader.cpp`,
`PluginPresentationLoader.cpp` and inline `PluginPresentationFallback.h` with
locked wx 3.2.8. It neither builds nor launches OpenCPN. The workflow is manual;
adding it does not dispatch a run.

The accepted vendor `o-charts_pi.dll` (SHA-256 `99edcfd4…a5a5b875`) is extracted as
one exact regular member of its locked public vendor archive. It is only read,
hashed and copied into private negative-control directories. It is never passed
to a factory, `LoadLibrary` or the production fallback helper. No vendor helper,
license, chart, profile or private data is extracted or used.

Five tiny locally compiled fixture DLLs provide known exports, deliberate missing
exports and controlled binding/status errors. Their factory/destructor exports
terminate the test with failure if accidentally called. The positive binding
fixture uses the real copied `BindingState` and verifies the complete UTF-8 path.
It also tries real write/delete opens on the original during binding, proving
that the production read lock prevents both. These fixtures do not qualify the
real adapter's import ABI or renderer.

The first 38 grouped checks cover:

- Unavailable package, actual worker thread, Safe, empty verified resource path
  (the Standard/style-ineligible boundary), and unrelated original filename.
- Wrong original/adapter bytes or declared length, missing adapter, relative/UNC
  paths, actual NTFS junction parents, conflicting file locks, and changes during
  the compatibility inspection window. Compatibility is a controlled explicit
  callback input; the actual real-plugin import ABI check remains separate.
- A hash-matching invalid PE, over-capacity UTF-8 binding path, four missing
  exports, bind refusal and seven malformed/pre-initialized status responses.
- The actual copied-status validator, success with one module, no replacement of
  an occupied destination, lock release after return, and clean unload events.
- The complete actual fallback helper with null/forwarded callbacks, failed
  original load after prior cleanup, a clean rejected adapter before benign
  original fallback, and residual-module refusal. Every original used in these
  fallback tests is a harmless locally built fixture, not vendor bytes.
- Explicit invalid-handle fault injection: a reserved, inaccessible, non-module
  address is attached as the handle; the real Win32 `FreeLibrary` failure must
  retain ownership and block fallback. Scope cleanup detaches it and releases
  the reservation. This is not evidence of a genuine loaded-module unload
  failure, and does not execute the reserved address or mock any Windows API.

`tools/test-ocharts-loader-windows.py` accepts only disposable native Windows CI.
It records exact source/runtime/executable/fixture identities and fails on source
or runtime drift. It builds only this small project and uses no charts/network
service or plugin constructor. Downloads are limited to the locked wx SDK,
pinned picosha2 header and accepted public archive. Only logs, copied DLL events
and identity receipts are uploaded; vendor bytes and private case copies stay in
the disposable runner scratch directory.

Local preparation evidence: Python syntax passed, and the exact cached archive
was read with the real extraction function: only its 2,088,960-byte DLL was
extracted, its accepted hash matched, and its PE header verified as I386/PE32.
The 42-group predecessor passed focused native run
[38146240771](https://github.com/ThereptileII/Work/actions/runs/38146240771)
at `840e1932fd2531f508d43459c89d39bce98fb496`. The receipt retains source,
fixture and runtime hashes and explicitly sets product acceptance false.
It includes two direct/delay and two same-directory indirect-import controls;
its benign indirect control explicitly preloads the helper. That result does
not prove first dependency mapping while inspection owns the sibling handle.
The 43-group predecessor adds the first-mapping control below.
Those groups passed at `e615446e2c1f547fd2e01d87bcf0621da0c8500a` in
[native run 38159170047](https://github.com/ThereptileII/Work/actions/runs/38159170047).
The receipt records `unmappedSiblingGuardControl: true` and product acceptance
false. No boat action was performed for this increment.

The first 43-group attempt,
[38158753749](https://github.com/ThereptileII/Work/actions/runs/38158753749) at
integrated revision `1f756424`, compiled and passed all prior 42 groups. The new
group stopped before original load at the fixture's process DLL-directory
assertion. Its zero-capacity `GetDllDirectoryW` query incorrectly treated a
required buffer size as an actual nonempty path, so this failure establishes
neither a nonempty override nor a product loader failure. The failed run is
retained as evidence. The repair reads a bounded output buffer, distinguishes
an empty string from an API error, logs the actual path/length and rejects a
genuine nonempty override without changing it. After the scoped current-directory
change, it also verifies the queried DLL-directory string is unchanged. It
never calls `SetDllDirectory`, preserving the distinct empty-string/NULL states.
The repaired 43-group control passed the exact-revision native run above.

API/source references: [GetDllDirectoryW return contract](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-getdlldirectoryw),
[SetDllDirectoryW empty-string and NULL semantics](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setdlldirectoryw),
and [pinned wx 3.2.8 native load wrapper](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/dlmsw.cpp#L185)
(ordinary `LoadLibrary`, with no search-state setter in this wrapper).

Remaining product gates: actual package/adapter import ABI, full application
Standard/Safe selection, real plugin lifecycle, encrypted-chart rendering,
licensing behavior and physical hardware. The mechanics tests do not replace
these gates or certify a usable chart package.


## SCRUM-209 original plugin refusal extension

Two additional harmless DLLs import a locally compiled `libeay32.dll` stub,
through real normal and MSVC delay-import tables. They are loadable without the
guard, with attach/detach event evidence. The production fallback must refuse
both before original DllMain, on repeated attempts and after a clean adapter
refusal, even while the stub is already mapped. The test checks original bytes
and lock release afterward. No real obsolete TLS library or vendor code is
executed. Source receipts now include both guard headers and the PE reader;
fixture receipts include the stub and both importing modules. These new native
controls passed in the 42-group predecessor above. They do not exercise actual OpenCPN root
discovery, dependency resolution, helpers or arbitrary dynamic loads.

The indirect controls add an original DLL importing a locally built helper.
One helper imports the harmless legacy-name stub; the other is benign. The
original has no direct legacy import. The first is refused before its
DllMain; the second remains loadable using an explicitly preloaded benign
helper. This tests the same-directory disk graph and already-mapped control,
not first mapping, general Windows loader search order or plugin helper processes.

The additional `indirect_unmapped` original imports a unique inert
`helper_unmapped.dll`, built below `Release/unmapped` so the executable directory
has no duplicate. The test verifies that the helper is unmapped, then temporarily
sets only its own process current directory to the private original directory
for dependency search. It preserves the prior current directory and restores it
on success and exception unwinding; it changes no system search configuration.
While an explicit production `OriginalPluginTlsGuard` is alive, it checks actual
write/delete sharing violations for both inspected files, performs the first
native original load, and verifies the mapped helper's full path, volume/file
index identity and SHA-256 against the inspected sibling. It repeats the lock
checks after mapping, unloads the original and dependency, and verifies unchanged
bytes and released inspection handles. The native log records the mapped path
and identity; the exact receipt adds `unmappedSiblingGuardControl: true`.

This control directly exercises the production guard class and native load.
Existing fallback groups cover the fallback helper separately. The passing new
control establishes this one benign first-mapping/inspection-handle case;
it does not qualify general loader search, arbitrary plugins, full application
fallback integration or SCRUM-209 closure. Source receipts include the new inert
helper source, and fixture receipts include both the new original and helper.

## SCRUM-209 declared export-forwarder extension (native pending)

The project now requires **51 groups** and additionally runs the focused pure
forwarder suite under MSVC. The eight new loader groups use only locally built
inert DLLs: direct named and ordinal `libeay32` forwarders, an ordinal `ssleay32`
alias, an indirect sibling forwarder, benign named/ordinal forwarders, and two
benign ambiguous-grammar probes. The [MSVC EXPORTS syntax](https://learn.microsoft.com/en-us/cpp/build/reference/exports?view=msvc-170)
documents both named and ordinal forwarding definitions. Every fixture's exact raw forwarder string must
be emitted by MSVC and observed by the production reader. There are no ordinary
dependency import descriptors for those forwarded modules.

The production fallback must refuse declared legacy or unsupported-grammar
graphs before original or forwarded sibling DllMain, including repeated attempts
and a clean adapter refusal. The indirect sibling has an explicit attach/detach
marker. Private harmless legacy stubs provide unguarded loadability comparisons;
they are explicitly preloaded from the case directory to avoid the older
fixtures' application-directory duplicate. No obsolete TLS implementation runs.

For simple benign `module.symbol` and `module.#ordinal`, the actual production
guard owns original/sibling handles through both native load and
`GetProcAddress`. The previously unmapped sibling must resolve to its inspected
private full path, pre-mapping volume/file-index identity and SHA-256; the test
invokes only the known inert integer export. Write/delete locks are checked
before and after resolution, and unload, unchanged bytes, released handles and
restored process search state remain required. New helpers live below
`Release/forwarders`, outside the executable directory.

Multiple dots and explicit `.dll` spellings remain **unsupported policy
grammar**, not a declaration that Windows rejects those images. Two unguarded
benign probes expose both plausible symbol/module alternatives using distinct
integer values. Native logs record the actual resolved path, identity and value;
neither probe relaxes the production policy. The current conservative subset
must not be broadened based on a guessed separator rule.

The reader follows Microsoft's [PE export-address-table specification](https://learn.microsoft.com/en-us/windows/win32/debug/pe-format#export-address-table):
only EAT RVAs inside the export-directory interval declare forwarders, and each
ASCII string including NUL must fit inside that interval. Ordinary EAT entries
remain outside this dependency inspection; their acceptance is not executable
code or full export-table validation. The declarations are recorded separately
from normal/delay imports, then checked by the same TLS policy and bounded local
graph. All list results are discarded on malformed inspected metadata.

Portable checks and the retained old-policy failing fixture are recorded in
`docs/evidence/scrum209-export-forwarders-20261011`. The new native groups and
grammar observations have **not run**. This remains declared local-graph
enforcement only; external loader roots, arbitrary dynamic/helper loads, full
application/adapter integration, general mapped identity, TLS/certificate and
installer lifecycle acceptance remain open. No vendor factory, chart, boat or
physical operation is part of this gate.
