# SCRUM-267 — private renderer effective point-symbol table

This separate increment extends the existing private presentation patch for
pinned o-charts `c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`. It does not change
shared chart preferences, plugin configuration assignments or lookup semantics.
Core OpenCPN policy and diagnostic fields are a separately owned part of SCRUM-267.

## Boundary

The private `s52plib` has a default-false per-instance
`m_presentationSimpleSymbols` member. `EnablePresentationSimplifiedSymbols()`
only enables it. `GetEffectiveSymbolStyle()` returns SIMPLIFIED when enabled,
otherwise the unchanged `m_nSymbolStyle`. There is no persistence setter,
global mode flag or shared ownership added.

`ChartPresentationAdapter.cpp::CreateChartPresentation` enables the policy only
inside the successful `m_bOK` branch **after the second compiled-resource
verification**. Unbound/refused-resource/failed-initialization fallback constructs
a fresh stock library with the default-false flag. No caller can select a style
by changing an arbitrary profile or trusted-resource hash.

The eight actual private reads now use the getter:

| Patched private source | Use |
|---|---|
| s52plib.cpp:634 | UpdateMarinerParams: S52_MAR_SYMPLIFIED_PNT |
| eSENCChart.cpp:2563 | DoRenderRectOnGL point table |
| eSENCChart.cpp:2741,2794 | DoRender2RectOnGL point and text passes |
| eSENCChart.cpp:2988 | DCRenderText point table |
| eSENCChart.cpp:4568 | BuildRAZFromSENCFile initial LUP selection |
| eSENCChart.cpp:6740 | GetObjRuleListAtLatLon selection/query table |
| eSENCChart.cpp:7263 | GetLightsObjRuleListVisibleAtLatLon query table |

The renderer continues to use its existing RAZ arrays, conditional logic,
visibility, table lookups and feature attributes. Boundary-table selection is
unchanged. The point-table getter is not applied to area boundaries.

`init_S52Library` creates the verified library then calls `LoadS57Config`, which
calls `UpdateMarinerParams` at private `o-charts_pi.cpp:2444`. Therefore the
constructor's initial Paper parameter is replaced by the effective value before
normal rendering. The existing `GenerateStateHash` includes all S52 mariner
parameters, including this effective point-table parameter. Its algorithm is
unchanged. Host/config field assignments at private `o-charts_pi.cpp:977,2407`
and `s52plib.cpp:289,9899,10003` remain byte-exact. The historical `LIGHTS03`
reference to `m_nSymbolStyle` in `s52cnsy.cpp:941` is inside a block comment and
is intentionally untouched.

## Verification

- Exact original blobs verified against the existing source lock. Both final
  private patches apply from those originals, and the entire resulting source
  tree equals the source used for the checks/compiles.
- Full-tree reverse proof: seven eSENC reads, one mariner read and the exact
  header methods/member reverse to the pre-increment patched source; all other
  files, including configuration handling and conditional symbology, are equal.
- 40 C++ checks execute the extracted actual inline methods and actual
  `UpdateMarinerParams` against the **real private `s52utils.cpp` mariner store**.
  Paper/Simplified saved values, both area boundary styles, repeated host writes,
  idempotent enable and fresh stock/fallback instances are covered. Removing the
  effective override fails the unchanged fixture as expected.
- Actual private `s52plib.cpp`, `eSENCChart.cpp` and product
  `ChartPresentationAdapter.cpp` compiled independently with real headers and
  GL/GLSL defines. These are single-object Linux checks, not a linked adapter.
  The historical s52-only compiler recipe initially lacked eSENC's TinyXML
  include and TIXML_USE_STL dependency. Retained setup failures document this;
  adding the exact pinned `libs/tinyxml/CMakeLists.txt` public include/definition
  produced the passing eSENC compile. No header substitutes or permissive flags.

No application, vendor DLL, helper, licensed chart, transport or boat was run.
No profile was read or written. This does not qualify native Windows, licensed
chart rendering, actual mode/style cycles or boat recognition. Those remain
SCRUM-267 and SCRUM-259 acceptance gates.

Reproduce the focused source/method check using the two privately prepared trees:

```sh
python3 tools/test-ocharts-symbol-style.py \
  --source /private/after/source --before /private/before/source \
  --wx-prefix /path/to/wx/prefix --output /private/fixture-output
```
