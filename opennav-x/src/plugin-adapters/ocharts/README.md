# SCRUM-259: private o-charts presentation port

This overlay and `patches/ocharts-skager-presentation.patch` target only
`bdbcat/o-charts_pi` commit `c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`.
The plugin vendors its own S52 library; the `opencpn-libs` gitlink
`38762f4e7b8faf39133cc40efc99b12b5200f04b` supplies API 17, not this renderer.
Preparation/build/package qualification is separate from this source port.
The adapter must never be named `*_pi.dll` or be independently discovered.

## MSVC runtime compatibility

The standalone private target uses the pinned host's
`_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR` definition for all its C++ objects. The
host already uses this policy in OpenCPN 5.12.4 `CMakeLists.txt:431–440`; the
private build must not silently use newer inline mutex construction against
the host's older staged runtime. Microsoft documents the constructor/runtime
compatibility boundary in its [VS 2022 17.10 STL release notes](https://github.com/microsoft/STL/releases/tag/vs-2022-17.10).
This preserves the existing ABI and binding locks; it does not replace any
installed runtime or make an older runtime a supported general deployment
choice. The recovery packager separately supplies the licensed current x86
toolchain runtime.

The first actual host/module check in candidate `ecf7e0c` exited with an access
violation before its report. The [bounded native BindingState comparison](../../../docs/evidence/scrum259-native-binding-runtime/README.md)
reproduces the unguarded first-lock access violation and completes the guarded
binding lifecycle against the same exact staged CRT. It does not locate the
earlier full-host fault instruction or rule out an additional defect. Actual
host import/bind/status/unload and boat chart acceptance remain separate gates.
The early-only module diagnostic now retains flushed stderr stages through
the inherited Windows pipe without opening a log dialog or a user profile.

## Presentation boundary

The patch changes one renderer initialization call, preserving its stock CSV
registrar, helper, decryption, license and plugin-data paths. The private library
receives the host's copied binding only after the host qualifies both DLLs.
Binding has no renderer side effects, refuses malformed UTF-8, non-absolute paths,
wrong version/size, nonzero reserved/trailing bytes, repeat binds and binds after
initialization. Status returns copied scalars, never a pointer or refreshed state.

The adapter verifies the five resources against its compiled
`XNavChartResources.h`, initializes only from that directory (never CWD), and
verifies the bytes again before selecting the library. The strict parser rejects
missing/invalid XML, incomplete required sections, missing Day/Dusk/Night tables,
foreign atlas names and PNG decode failures before installing tables. Hashes, not
this structural check, define the accepted symbol/lookup contents. No GL textures
are allocated by the validation step. Hashes before/after loading detect changed
bytes but are not a claim of adversarial resource-race protection.

A refused owned presentation is destroyed; a fresh stock library is constructed
from the unchanged shared `s57data` directory, also without a CWD override. An
unbound adapter uses the original stock constructor behavior. The host must use
the original plugin for Standard/Legacy/Safe and unsupported identities. Switching
chart style requires the agreed cold restart. Fallback status reports presentation
selection, not chart-license validity or successful chart rendering.

## Typography port

The eight-file patch is a three-way transfer from the pinned OpenCPN base to the
accepted core presentation implementation, retaining the plugin's renderer/API
and coordinate differences. `SKAGER_OCHARTS_ADAPTER` scopes the private fields and
paint hooks; do not define `OPENNAV_X` for the plugin. All objects that include
`s52s57.h` must use the same definition to preserve private class layout.

Geographic OBJNAM classification, tracking/native shaping fallback, opacity and
whole-string texture invalidation use the existing shared helpers. Generated
LIGHTS descriptions use the same exact three suffix classifiers, font/ink/halo
raster and cached SW/GL paint, preserving description strings and visibility.
The pinned private `s52cnsy.cpp` emits those suffixes at lines 1643–1652. Soundings
reuse the exact-size font resolver and depth atlas sizing change; their digits,
units, placement, color, special marks and display preferences stay upstream.
There is no new decluttering policy or fabricated chart geometry.

API 17 exposes `GetFontLegacy`, which returns the first matching locale/ChartTexts
entry, whereas the core renderer selects the default entry. The LIGHTS guard
preserves the private renderer's existing request: system point size +2 (+1 on
macOS). It only owns a returned factory-equivalent normal system font with black
ink. This prevents changing default creation and preserves custom first-match
appearance. With multiple stored font entries, the private plugin can therefore
retain stock LIGHTS text when the core styles it; no new host font query or
preference mutation is introduced. Geographic-name and sounding typography retain
the already selected style-ownership policy.

## Focused evidence, 2026-10-03

- The patch applied cleanly to all eight exact source files and reverse-applied
  byte-for-byte to the original verified source.
- `tests/binding_test.cpp` passed malformed input/no-copy, repeat/late bind,
  copied ownership, stock/unbound and side-effect-free status cases.
- `tests/resource_test.cpp` used the actual generated five-file resource set.
  Exact hashes/XML/PNG decode passed; a changed byte, corrupt/missing dusk PNG,
  incomplete XML, foreign atlas path and empty document were refused. Expected
  PNG errors are retained in the test log.
- GCC compiled `s52plib.cpp`, `chartsymbols.cpp`, `DepthFont.cpp`,
  `o-charts_pi.cpp` and `ChartPresentationAdapter.cpp` against the exact private
  plugin/API headers in both production `ocpnUSE_GL` + `ocpnUSE_GLSL` and
  compatibility GL configurations. These units contain the software path too;
  this is compilation evidence, not a running renderer capture.
- Initial diagnostic compilation omitted upstream `__OCPN_USE_GLEW__` and
  exposed a GL include-order failure. Restoring that actual Plugin.cmake flag
  fixed it. The X11 `Status` macro collision was corrected by naming the private
  method `ReadStatus`. No upstream GL behavior was changed.

Reproduce the loader-boundary tests on Linux with the prepared source tree, the
matching generated resource directory (including `XNavChartResources.h`), and wx:

```sh
python3 src/plugin-adapters/ocharts/tests/run_tests.py \
  --plugin-source /absolute/prepared-plugin-source \
  --resources /absolute/generated-resources \
  --output /absolute/private-test-output
```

`--wx-config` and `--wx-prefix` support the existing private wx sysroot. The
runner compiles only the copied binding and actual resource validation tests; it
does not load a plugin, create chart objects, launch an app or access a boat.
The independent preparation recipe owns production build commands and Win32
export aliases. Native MSVC DLL link/export/ABI, host selection/fallback, actual
encrypted-chart SW/GL Day/Dusk/Night and Windows font/DPI evidence remain required.
No package, release, licensing or boat acceptance is implied by this port.

The accompanying host review identified an over-capacity Win32 path conversion
that must be checked before constructing wxString; the root owner corrected it.
Host explicit read locks bracket load/bind/status, not the later `create_pi` call.

SCRUM-267 adds four observation-only activity calls at Init entry/completion,
DeInit and plugin destruction. The separate `skager_chart_point_style_v1` export
copies the actual private renderer getter only while that renderer is active,
valid and selected, on the application thread. It borrows the pointer for that
single read and never retains it. Failed initialization, fallback, absent renderer,
DeInit/destruction and off-thread queries cannot expose a table. Existing binding
and status v1 layouts/reserved fields are unchanged; package verification now
requires the additional export. The focused runner also checks this observation
and its host-side decoder, without launching any renderer.
