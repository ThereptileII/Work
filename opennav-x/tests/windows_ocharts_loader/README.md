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

The 38 grouped checks cover:

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
The native project and 38 groups are **prepared, not run** until a native workflow
receipt exists. No CI dispatch or boat action was performed in this task.

Remaining product gates: actual package/adapter import ABI, full application
Standard/Safe selection, real plugin lifecycle, encrypted-chart rendering,
licensing behavior and physical hardware. The mechanics tests do not replace
these gates or certify a usable chart package.
