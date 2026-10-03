# SCRUM-264: actual IHO S-64 symbol rendering, Linux 9632421

Sixteen original 1280 × 800 screenshots passed the bounded collector: three scenes, SKAGER and Standard, software and Mesa OpenGL, with an additional Night comparison for isolated-danger/safe-water. All twelve corrected application sessions exited normally (actual debugger exit events report code 0). Every full image and the source-position crops were inspected at native size. No new rendering defect was observed in the scoped ordinary lateral, isolated-danger, safe-water, cardinal or short-range LIGHTS13 artwork.

This is development evidence from official presentation-test geography. It is not operational ENC, native Windows, physical GPU, private o-charts or boat acceptance. Standard is the same executable's stock-resource reference, not a historical pre-change SKAGER image.

## Exact inputs

- Clean source: `9632421f701c5ec74d7a1360bc713c0034faf9af`.
- Installed ELF: `55b5f9f707be6188bf283c4ece5e992caa43352d77c4a7639a195f60d52e27a3`.
- Chart manifest: `4af8f1245a59d2280fd74b563fd24f98401dfeb4ff8cdabf927352e5df856fc6`.
- GB4X0000.000: 945697 bytes, `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- OpenGL: Mesa 26.2.2-arch1.1, llvmpipe (LLVM 22.1.8, 256 bits), actual OpenGL 4.6 compatibility profile. This executes the GL path through a software renderer.

`inputs/` retains the build owner's source/install seals and the independent post-capture executable, manifest, fixture and clean-source recheck. The collector verifies all nine applied patch results and source/resource identities before launch. It uses the existing staged build without rebuilding or modifying it. Exact layout-header hashes and compiler identity are retained with the post-capture check.

The fixture is the official IHO S-64 Edition 3.0.3 unencrypted presentation-test cell, retained from the official test dataset. See the existing source/rights audit `docs/design/reviews/scrum256-official-cardinal-test-source.md` and the full feature inventory `docs/evidence/scrum264-public-enc-scenes/`. No public-domain or unrestricted redistribution claim is made. Original chart bytes and generated SENC files are not included here; local evaluation screenshots, selected feature metadata and file hashes are retained. The copied fixture was mounted read-only and remained byte-identical.

## Image and selected-rule map

Each filename below exists under both `software/` and `opengl/`. Matching JSON holds actual viewport, source-point crop bounds, live-input status and selected-rule evidence. Full original PNGs are retained, with 72 × 72 crops anchored to the actual raster painter coordinates.

| Scene images | Source features | Actual SKAGER / Standard selection |
| --- | --- | --- |
| `s64-lateral-{SKAGER,Standard}-Day.png` | S7 green cone RCID 219; S6 red can RCID 224 | XNLAT013 / BOYLAT13; XNLAT024 / BOYLAT24; both raster. Co-located red LIGHTS11 remains vector. |
| `s64-safe-isolated-{SKAGER,Standard}-{Day,Night}.png` | BOYISD RCID 454; Fairway BOYSAW RCID 1389 | BOYISD12 and BOYSAW12 raster. Co-located white LIGHTS13 becomes raster in SKAGER; Standard retains vector. |
| `s64-cardinals-{SKAGER,Standard}-Day.png` | North RCID 82, east 4, south 72, west 10 | BOYCAR01/02/03/04 raster; all four existing approved cardinal shapes actually reach the painter. |

`collector/scenes.json` records exact original attributes, coordinates, lookup IDs/RCIDs and related LIGHTS/TOPMAR. Source geometry is preserved. `S57Obj::Index` in the trace is the OGR feature index, not source RCID; coordinate and class matching establishes identity. The ordinary stem/head artwork visibly replaces stock filled shapes. Isolated-danger double spheres, safe-water sphere and small white-light circle/rays remain visible in Day and subdued Night. Cardinal head arrangements are visible with existing chart overlays.

Both styles deliberately use persisted Simplified symbol table 76 to compare artwork at the same lookup boundary. Actual in-process effective table and lookup table are also traced as 76. This does not independently qualify the separate default Paper-to-Simplified selection policy. The raw report's older `symbol_table_receipts.scope` describes only the persisted-config receipt; the separate actual trace records provide the additional runtime lookup proof.

## Preserved failures and read-only trace correction

The first software and GL attempts failed before screenshots because the optimized executable did not expose the DWARF parameter `this`: `Variable 'this' not found.` Their original reports, command logs and trace errors are retained in `initial-trace-failure/` and `diagnosis/`. These are diagnostic failures, not observed product paint failures. Forced cleanup produced a null debugger exit code; no normal-exit claim is made for those first attempts.

The bounded correction uses the exact mangled symbol entry before its prologue, after checking actual ELF64 x86-64 SysV disassembly. It reads argument registers and bounded process memory only. A tiny standalone layout program compiled against the actual production headers and ChartPresentation compile definitions/include paths supplies offsets; it constructs no chart/navigation objects. The first helper link lacked wx libraries and is retained; linking the actual wx core/base libraries corrected that helper-only failure. No application rebuild occurred.

The trace calls no inferior function and writes no model state. It validates pointer ranges, names, coordinates and final canvas dimensions. RenderSY and RenderRasterSymbol probes are capped and disable themselves after requested geometry is observed. Both are disabled before the settled screenshot; the application continues normally. Debugger stops affected initial rendering timing, so these are not performance or endurance measurements. The executed collector, corrected and initial probes, layout source/result, compile receipt and actual disassembly are included.

## Controls and retained differences

Fresh isolated profiles, displays 247/248 and private process namespaces are used. The fixture is the sole chart database entry; the actual quilt, viewport, 1014 × 566 canvas and exact presentation status are checked. Controlled loopback RMC travels through the real decoder, with fresh LIVE/Measured values checked and pilot/control disabled. These simulated sensor values are explicitly test input, off the official test-chart viewport; chart objects are not injected. Theme changes use the actual controls. Exact depth-water ink and actual requested symbols are required, and shutdown requires normal application exit. Original profiles remain locally; this evidence includes configuration/log/GL receipts and complete profile file hashes, excluding chart/SENC/cache binaries.

The cardinal scene has a retained all-round yellow light circle near image position (596,342): approximately 24-pixel radius in software versus 65 in GL. This difference is present in **both Standard and SKAGER**, outside the changed short-range LIGHTS13 raster. The full four cardinal images preserve this counterevidence; no renderer-wide equality or all-light conformance is claimed. Existing magenta warning/radio symbols, central physical lighthouse/beacon artwork and this long-range circle remain stock. OverZoom warnings in the close lateral/isolated views remain visible.

Preferred-channel actual-chart coverage remains open: GB4X0000 has 22 ordinary red/green BOYLAT and GB4X0001 only two ordinary BOYLAT, with no preferred-channel categories/bands. BOYSPP and generic fixed-beacon replacement remain open. Related physical TOPMAR records are retained; these Simplified default TOPMAR rules have no separate paint instruction. This does not establish all TOPMAR combinations or authorize inferring physical equipment. Night here covers isolated-danger/safe-water only; no Dusk, hot-return, sector/directional-light, all preferred-channel alias, Windows or private-plugin rendering claim is made. No additional product changes, application build, CI or boat action was performed.

## Reproduction and integrity

`collector/` contains exactly the executed scripts. Restore the isolated cache symlinks and the read-only official fixture described in the preparation evidence, then use the wrapper with the sealed expected identities. The recorded commands use phases `s64-963-software-abi` and `s64-963-opengl-abi`, renderer `software` / `opengl` and displays `:247` / `:248`. The supplied Python is `/home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3`.

Logs use deterministic gzip; original file hashes and byte sizes are in each `complete-local-file-inventory.json`. `SHA256SUMS` covers every retained file other than itself. No screenshots were repainted, resized or retouched.
