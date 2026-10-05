# SCRUM-259: pinned o-charts private presentation boundary

Read-only independent source audit, 2026-10-03. No application/plugin changes, builds, CI, boat actions or private-chart reads.

## Finding

The current SKAGER core presentation factory does **not** select resources or typography for the pinned o-charts private S-52 renderer. With ordinary installed working-directory contents, o-charts loads the stock shared `s57data/chartsymbols.xml` and its atlases. A working-directory override can alter that result, but is an uncontrolled process-wide precedence rule, not SKAGER integration. This establishes the source-level coverage gap; it is not a claim of newly observed boat pixels.

Do not interpret the core's `SKAGER presentation v1 / pinned symbols` status as proof that a plugin chart uses that presentation. MBTiles are outside this vector-resource mechanism; baked raster labels/symbols cannot acquire the vector style through these hooks.

## Exact source and dependency identity

Pinned plugin: [`bdbcat/o-charts_pi`, c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8](https://github.com/bdbcat/o-charts_pi/tree/c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8).

The cached directory is not a Git checkout. I compared its file contents with the exact GitHub tree using Git blob SHA-1 framing, rather than inheriting the enclosing repository's HEAD:

| File | Verified Git blob |
|---|---|
| `Plugin.cmake` | `c6b3200cde359d2dccdf253045435089ea945c3a` |
| `CMakeLists.txt` | `9050f108a8567a58f8975556db433aadf7c385fe` |
| `src/o-charts_pi.cpp` | `2dc83c6a6eda949773bc440dc8c83f8b20aeb67f` |
| `src/eSENCChart.cpp` | `02e3d21f580fa1bc5cca38ebf097d169358bd80a` |
| `libs/s52plib/CMakeLists.txt` | `b469184f9bb7aafa7de52bf4b19e018522cc172b` |
| `libs/s52plib/src/s52plib.cpp` | `72356a432ace97dec3cb05eb060effa1a9908786` |
| `libs/s52plib/src/chartsymbols.cpp` | `30ae2477b4b8ba748a8cd6f046079e8607b8151f` |

`opencpn-libs` is gitlink [`38762f4e7b8faf39133cc40efc99b12b5200f04b`](https://github.com/leamas/opencpn-libs/tree/38762f4e7b8faf39133cc40efc99b12b5200f04b). Its recursive tree has API 1.17 headers/import libraries but no S-52 renderer implementation. `Plugin.cmake:158–159` instead adds **plugin-owned** `libs/s52plib` and links `ocpn::s52plib`; that directory's CMake file creates static `S52PLIB` from its own sources. Top-level `CMakeLists.txt:46–50` separately links the API import target. Therefore the plugin's s52plib constructor is not the core constructor patched by SKAGER.

## Loader and actual paint chain

- `o-charts_pi.cpp:748`: plugin initialization calls `init_S52Library` before chart rendering.
- `o-charts_pi.cpp:2639–2657`: registrar/CSV paths derive from `GetpSharedDataLocation()+s57data`; the private `ps52plib` constructor receives the stock shared `s57data/` directory. Existing instance returns early.
- `s52plib.cpp:258–276,1120–1170`: constructor builds its own tables, calls its own `LoadConfigFile`, then preloads object classes from the stock shared CSV location.
- `chartsymbols.cpp:679–729`: loader gives `chartsymbols.xml` in CWD precedence; otherwise loads beside the constructor path. No manifest verification or strict-path option exists. It also returns true after a failed XML parse when the file exists: a new integration must not treat current `m_bOK` alone as sufficient validation.
- `chartsymbols.cpp:736–810`: raster atlases load relative to that same selected directory.
- `o-charts_pi.cpp:2661–2670`: plugin-data XML patch code is commented out; the matching `ChartSymbols` header has no active `PatchConfigFile` method.
- `o-charts_pi.cpp:900–1080`: `OpenCPN Config` messages synchronize visibility, mariner settings, symbol style, text factors and scales, not XML/atlas paths or core font resolver callbacks.
- `o-charts_pi.cpp:1118–1124`: palette change selects the private library's color scheme.
- `eSENCChart.cpp:2606–2805` and `6256–6346`: active GL/software drawing calls private `ps52plib->RenderArea.../RenderObject...`. Core PI style/state queries still synchronize settings; they do not redirect paint. An older core-renderer path at 1959–1999 is disabled by `#if 0`.
- Core source 78eccb8 `ChartPresentation.cpp:151–179` creates and configures only its returned core library. `ocpn_plugin_gui.cpp:382–384` still returns the ordinary shared directory.

## Smallest safe cooperative integration proposal

1. Keep all existing shared-data/plugin-data API meanings, CWD and stock files unchanged. Add an explicit, versioned **opt-in plugin presentation negotiation** using the existing plugin-message ABI (or a separately versioned dedicated API), with an exact supported plugin/resource identity. Unmodified or unsupported plugins continue stock rendering. This requires a qualified plugin change; there is no current stock-plugin preference that accomplishes it.
2. Before creating the private library, resolve a dedicated SKAGER resource directory only when the host's actual selected mode/style is active and the manifest/assets validate. Keep registrar CSV paths stock. Add a plugin-local strict resource loader that cannot fall back to CWD, reports parse/atlas failure and falls back atomically to the unchanged stock resource set.
3. Resource mapping covers colors and resource-defined glyphs only. Core geographic/light typography and sounding font resolvers do not exist in this private library. Full visual matching requires bounded ports of those hooks into the private painter, with the same custom-font/semantic guards; do not claim resource selection alone fixes fonts.
4. Preserve current Day/Dusk/Night callbacks and mariner preferences. Style switching needs an explicit lifetime plan: chart objects retain LUP/rule/text/bitmap/GL caches. `UpdateLUPs`, `ClearRenderedTextCache`, `ResetPointBBoxes` and `FlushSymbolCaches` exist, but do not by themselves prove replacing a live library is safe. Initial construction before charts exist is the narrow first boundary; live switching requires coherent chart teardown/rebuild or separately qualified cache invalidation. Do not silently accept a stale style.
5. Report the actual selected renderer/resource identity separately for core ENC and o-charts. Preserve Standard/Legacy/Safe fallback and validate both paint paths. Licensing, decryption, helper processes, chart bytes and user profile content are outside this integration.

Reusing the core PI object-render APIs would require adapting the plugin's own decoded object/context ownership and replacing its active paint path; this is materially broader than a private resource/typography integration. Global shared-path substitution, plugin-data substitution, CWD changes or stock-resource overwrite are not suitable shortcuts.

## Evidence and limits

Downloaded exact trees/API/library source remain in private `.local/ocharts259-audit/` in the `scrum257-final-captures` worktree; all fetched library files were checked against their exact tree blob identities. Existing cached plugin source was only read. No binary/source correspondence beyond the supplied accepted pin was newly established. Actual pinned Windows plugin compilation, installed ABI/load, normal licensed chart rendering, theme/style transitions and boat recognition remain required before claiming coverage.

## Exact-original DLL shadow-load assessment

A host-selected, SKAGER-owned adapter for one hash-qualified original o-charts DLL is compatible with the inspected path model **in principle**, with these boundaries:

- Keep discovery identity in `PlugInContainer::m_plugin_file`. Core `gui/src/pluginmanager.cpp:4302–4312` returns this field from `GetPlugInPath(this)`. Plugin `o-charts_pi.cpp:562,575–577` uses it to locate the original Windows `oexserverd.exe` beside the original DLL. Returning the adapter path instead would relocate helper lookup and violate the intended boundary.
- `model/src/plugin_loader.cpp:1530–1536` currently stores that original path then loads the same path. A separate verified actual-load path can be selected at this shared load boundary while retaining original metadata. This must also cover reload: `UpdatePlugIns`, line 816, invokes `LoadPlugIn(pic->m_plugin_file,pic)` directly.
- Keep original `m_plugin_filename`, modification-time tracking, enabled-config key and duplicate suppression (`LoadPluginCandidate:496–622`). Register only one plugin instance/module and existing chart class names. Loading both original and adapter can duplicate private globals and wx chart registrations; a second separately discoverable plugin is not the safe model.
- Existing compatibility checking at line 587 inspects the original binary. The actual adapter also needs exact manifest/hash and Win32/wx/import compatibility validation before its `wxDynamicLibrary::Load`. In the Windows implementation this check parses imports without loading the original module (`CheckPluginCompatibility:1298–1345`); it does not force simultaneous original/adapter loading.
- `GetPluginDataDir` (`model/src/base_platform.cpp:269–293`) resolves named configured plugin-data roots, independently of `m_library`'s actual load path. Keep that behavior and original plugin name unchanged. No source requirement to relocate plugin data was found.
- `g_pi_filename` has a historical helper `-z` use in `eSENCChart.cpp:398`, but that function is enclosed in `#if 0` at 354–424. It is not evidence of an active helper requirement to execute the original DLL.
- Record both identities truthfully: original discovery DLL/path/hash and actual adapter DLL/path/hash. Existing metadata and logs based on `m_plugin_file` alone would otherwise obscure the code actually executing. Immutable package ownership must cover the adapter and dependencies; normal plugin update/reload must re-evaluate the original hash and reject unsupported composition.

A dynamically queried additive host C ABI is a maintainable alternative to message negotiation: a versioned bounded request/result that exposes only the selected verified presentation resource contract, with no C++ `s52plib` pointer crossing the DLL boundary. The adapter still owns its private library and strict-path/font-hook integration. Unsupported hosts or refused identities must preserve the original stock-plugin path. Core Standard/Legacy mode selection must not inadvertently execute the styled adapter, and Safe must retain upstream plugin-skip policy.

No intrinsic helper-path blocker was found in this bounded source inspection. This does not establish safety of a built adapter: Windows import resolution when loaded from a different directory, CRT/wx ABI, exported factory/destructor pairing, chart cache ownership and unload, original helper licensing operation, or plugin-manager update behavior still require exact-source native proof. Keep the adapter branch and this qualification separate from frozen 78eccb8/61a0a783.
