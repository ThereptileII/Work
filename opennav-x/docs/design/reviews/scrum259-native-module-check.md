# SCRUM-259: early private-module diagnostic

This is an explicit maintenance check in the real installed, fixture-free
`opencpn.exe`. It does not qualify chart rendering, licensed-chart recognition,
normal plugin initialization, helpers, boat hardware, or release readiness.

## Inspected load boundary

The pinned o-charts source is
`c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`, selected by
`tools/ocharts-adapter-source.lock.json`. The native recipe's source closure is
`cmake/ocharts-adapter/CMakeLists.txt`: plugin translation units, owned binding
and alpha helper, API-17 import library, cpl/dsa/wxJSON/iso8211/tinyxml/geoprim/
pugixml/s52plib, and wxCurl.

| Pinned source boundary | Observed consequence |
| --- | --- |
| `src/o-charts_pi.cpp:295–302` | `create_pi` constructs the plugin; never call it here. |
| `src/o-charts_pi.cpp:480–552` | Constructor creates an event handler/private directory, loads configuration and considers EULAs. Merely discovering a disabled plugin is unsuitable. |
| `src/o-charts_pi.cpp:558–748` | `Init` establishes helper paths, calls `IsDongleAvailable` at 651, then initializes S-52. Never call `Init` here. |
| `src/fpr.cpp:92–96` | `IsDongleAvailable` executes the vendor helper. It is beyond this diagnostic's permitted boundary. |
| `src/ochartShop.cpp:3917` | Global `wxStopWatch` starts a clock; other inspected nontrivial globals are strings, arrays/maps/vectors and wx class/event metadata registrations. |
| `libs/wxcurl/src/base.cpp:929–932` | `curl_global_init` is inside an explicitly called initialization method; no global call was found. |
| Owned `ChartPresentationAdapter.cpp:18` | Global binding owns storage/mutex only. Font statics and S-52 construction are inside functions. Bind/status copy data and do not initialize the renderer. |

No custom DLL entry point, CRT initializer hook, global wxModule instance, or
module-load-time helper/profile/host API call was found in that pinned compiled
source closure. Standard CRT and dependency DLL initialization still runs on
LoadLibrary; this is not a claim that arbitrary DLL loading has no side effects.
The executable hash/size is compiled from the verified package, not supplied by
a user option. Before loading, the diagnostic enforces the Windows child-process
prohibition policy. An unsupported/refused policy stops the check. The policy
setter is resolved dynamically only in the explicit check, so ordinary startup
gets no new hard kernel import or process policy.

## Implementation

`--opennav-self-test <new absolute report.json>` keeps its original resource-only
behavior. Adding `--skager-chart-module-check <absolute o-charts_pi.dll>` selects
the module check before normal profile, plugin and transport startup. Supplying
the companion option alone exits 2 through the same early path. The existing
self-test startup hook is unchanged.

The original DLL is only the locked, read/hash identity input. The private DLL
path is fixed beside the host. `LoadOChartsModule` retains its original-name,
local path/reparse, hash, byte-count, main-thread, compiled-package and two-pass
lock checks. There is no original fallback and no PluginLoader construction.
The portable PE32 parser checks x86 DLL shape, bounded imports, the permitted
SDK/system import set, actual host and wx core imports, exactly four named
exports, and refuses delay imports or forwarded exports. LoadLibrary in the
real host then resolves the actual imported functions; a parser alone cannot
prove this ABI compatibility.

The diagnostic verifies every installed generated resource against the compiled
manifest, loads/binds/queries the private module, copies its pending-init status,
records the actual loaded path, and checks unload plus absence of both module
handles. It never obtains a plugin instance or calls either factory or Init.
`plugins_loaded=false` continues to describe plugin instances; the separate
`chart_module.module_loaded` field describes the explicit transient DLL load.
The copied state/reason and negative identity case provide observable evidence;
factory/profile facts also rely on the audited early-only source boundary.

## Focused checks and native call recipe

Local checks passed: 20 actual C++ PE parser positive/refusal cases, and linked
real InstallerSelfTest/ChartModuleCheck objects exercising default behavior,
companion-option-only refusal, report overwrite/relative-path refusal and the
non-Windows explicit-check refusal. The Linux helper branch does not prove the
Windows loader. Python wrapper syntax checked; no native execution performed.

The root integration must add `src/integration/ChartModuleCheck.cpp` to the
existing application target and existing native changed-unit list. It uses only
wx/JSON, generated `XNavChartResources.h` and `SkagerOChartsPackage.h`, picosha,
existing module/binding headers and kernel APIs; no App or PluginLoader headers.
No new library beyond the existing host closure is required.

After the explicit private build has installed the production fixture-free
host, the existing disposable native job may run:

```powershell
python tools/test-chart-module-windows.py --install production-install `
  --evidence evidence/local/ocharts-real-host-module
```

Use a new evidence directory outside installation/discovery paths. The wrapper
requires Windows Actions. By default it uses the existing changed-unit locked
fetch routine to acquire the fixed archive under `build/ocharts-identity-cache`;
`--vendor-archive` can supply an existing archive only if its exact lock matches.
It extracts only the original DLL under that cache's `extracted` directory,
outside evidence and application discovery, and never installs or uploads it.
Evidence contains its hashes, not the vendor binary. It runs the actual host
with a 30-second bound for four cases: misplaced option, unchanged default,
positive module check, and a changed-original-byte refusal. It checks exact
candidate identity, fixture-free build, pending-init status, resource/hash/size,
actual loaded path, host imports, child prohibition and unload, then verifies
original, host, adapter and normal profile bytes unchanged. Preserve all reports
and command/stdout/stderr records. A compact source receipt includes the actual
Actions workflow, the wrapper/loader/early-entry sources, and relevant trust
inputs. At the post-bind boundary Toolhelp records up to 512 actual mapped
module paths; snapshot failure or an oversized inventory refuses the check.
The wrapper hashes those observed files after process exit and retains the
runtime receipt. This distinguishes selected runtime paths from an installation
inventory; it is not an in-memory code-integrity measurement. No runtime/vendor
binary is copied into evidence. No chart, license, vendor helper, profile fixture
or physical output is needed or accepted by this wrapper.

Required native compile, exact real-host module proof, and later normal private
renderer/boat acceptance remain open. API-17 here names the linked host import
ABI; it does not change the original plugin's declared API 1.11 or claim that
Init was tested.
