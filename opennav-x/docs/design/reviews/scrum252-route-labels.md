# SCRUM-252: eligible route waypoint name labels

Implementation based on application `1356fd1603aacbea04d7081d16331e9a181180bb`
and pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
This is a bounded visual change, not route, native Windows, or boat acceptance.

## Canonical reference

The immutable HTML SHA256 remains
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
Chromium 145.0.7632.6 computed styles and unmodified first-label captures are in
`docs/evidence/scrum252-route-labels/prototype-computed.json` and the three
`prototype-label-*.png` files. The later CSS override wins: 10px, normal 400
weight, normal tracking, inherited Segoe UI Variable Display / Segoe UI / Arial /
sans-serif. There is no text halo. Background is 25px high, radius 5px, stroke
1px `#708e791a`; width is `min(170, UTF16(name).length * 6 + 20)`.
The text starts 10px inside the background with baseline waypoint-y + 34px.
First-label x is -18; last-label x is -width-14; background y is +18.

| Theme | Fill | Text | Border RGB / alpha |
| --- | --- | --- | --- |
| Day | #f7f8f0 | #233e3e | #708e79 / 26 |
| Dusk | #243a40 | #e1e5d8 | #708e79 / 26 |
| Night effective | #101a20 | #8e9888 | #576f5e / 26 |

Night values include the **ancestor** chart-canvas `brightness(.78)`, which
changes RGB but preserves alpha. The browser capture's most frequent Night
background pixel is exactly (16,26,32). No other Night palette is changed here.
Prototype SVG viewport scaling and native glyph rasterization differ from
OpenCPN; the mapping uses nominal prototype pixels as DIPs, consistent with the
route markers. It does not pretend the prototype's illustrative locations are
real coordinates.

## Production boundary and ownership

`ChartRouteLabel.cpp` receives the existing SCRUM-242 copied eligible ordinal.
Every existing route/icon/selection/active/special/MOB/shared/repeated/layer/
range-ring/anchor-watch/route-style/Standard/Legacy/Safe fallback remains in
place. Label eligibility additionally requires a visible nonempty actual name,
factory-equivalent black Marks font, factory point-local derived font, and
unchanged (-10,+8) name offsets. Custom font/colour/offsets remain stock. Saved
factory-equivalent preferences are eligible; intent identical to factory values
cannot be inferred. No configuration is rewritten.

The prototype labels only its illustrative first/last points. Production keeps
**all existing name visibility decisions**, including interior names. Eligible
interior names use the first-point placement; this is an explicit adaptation,
not permission to hide a navigation label. Empty, multiline/control-containing,
overlarge, unsupported-font/raster, or unsupported-scale inputs fall back to the
whole stock label; names are never abbreviated, truncated, or rewritten.

The stock font, name extents, offsets, route model, projected position, and
selection/hit semantics are unchanged. Both painters union tight nontransparent-pixel bounds
covering the whole card **and full untruncated native text**, including the
prototype's long-name overflow beyond its 170px background. Texture origin and
power-of-two upload size are separate; transparent texture padding never enters
paint/hit/cull bounds. GL unions the
already-scaled label after upstream icon scaling, avoiding double scaling. Its
early geographic reject invalidates while the eligible or previous styled cache
exists, so style/size/theme/stock transitions cannot keep stale bounds.

A transient point-owned shared cache contains the native shaped whole-string
raster. It has no serialized fields or global pointer-keyed ownership. The
existing `m_iTextTexture` and deletion lifecycle retain GL ownership; an explicit
owned/current distinction invalidates changed pixels and deletes styled pixels
before returning to the stock texture builder. Replacement uploads are allocated
and their actual dimensions verified before replacing an existing stock texture.
A failed upload clears styled ownership and remains on stock until appearance
changes, preventing repeated stock-texture deletion/recreation. Software uses the same RGBA
bitmap. Unchanged pans do not rerasterize or upload a texture. Native complex text
and font fallback stay a single shaped run; no character-by-character splitting.

Inputs are bounded to 256 Unicode scalars, scale .25–4, measured width <=2048 and
height <=128, and <=65,536 power-of-two raster pixels. A single RGBA image,
native bitmap, or texture is at most 256 KiB, excluding platform bookkeeping;
upload has one temporary buffer of the same bound. Default Westhaven occupies
128x64 pixels. Only an eligible <=99-point active route can acquire new styled
caches. Encountered fallback releases CPU rasters and the GL texture on the next
GL paint. Hidden points follow the existing waypoint lifetime/resource policy.
This is a bounded allocation design, not a measured boat frame-time claim.

## Focused evidence

- `chart_route_label_test` uses the actual production raster and font guard:
  **93 checks pass**, all three themes, first/last placement, full long-name
  bounds, custom font/colour fallback, native accented/Arabic/CJK/combining/
  symbol text, 150% scale, atomic invalid-input fallback, and cache retention.
  `route-labels.png` is the inspected native fixture; it is not a live chart.
- 10,000 repeated cache hits per theme retained the same raster/texture state;
  observed 24.1 / 22.2 / 22.7 ms total.
  These microchecks exclude route eligibility scanning, the chart, and GPU work.
- Actual `ChartRouteLabel.cpp` and patched `route_point_gui.cpp` compiled with
  production flags/includes in **software and GL configurations** (4 objects).
  Source and object hashes plus exact commands are in `verification.json`.
- All **nine ordered patches** apply to the pinned source using an independent
  temporary index; both changed upstream files byte-match the resulting tree.
- All 7 focused Windows chart-preflight tests pass. The new helper is the 17th
  real unit in its existing allowlist; dynamic header/resource identity remains.
- The native fixture is a version-controlled CMake target run by the existing
  integrated/production Linux and pristine Windows scripts. No CI dispatched.

Native Windows fonts/DPI, integrated real-route screenshots for both renderers,
selection/theme transitions in a running application, pathological real fonts,
and boat readability/performance remain open. GL here was compiled, not visually
accepted. Root owns the combined integration and qualification. No frozen root
source/cache, boat, real navigation input, or chart data was modified.

## Follow-up review correction

Root review of `392d246` identified stale GL ownership on failed allocation and
transparent power-of-two padding in paint bounds. The follow-up separates raster
origin/quad from exact nontransparent pixel bounds and makes upload replacement
atomic. Its production failure method is exercised with an owned styled texture,
then a recreated stock texture: only the owned texture is deleted, flags clear,
and a repeated failure preserves the stock texture. Tests also check every
painted pixel is contained while both transparent POT dimensions are excluded.
Known upload failure is retained across unchanged cache hits and reset on a
changed appearance. Hardware allocation failure itself was not induced.
