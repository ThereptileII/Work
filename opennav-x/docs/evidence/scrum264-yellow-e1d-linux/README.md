# e1d0136 actual yellow buoy and fitted-head proof

Exact application source `e1d01367692f7b8c866be369417e68c836c5d563` completed a 29-step incremental `opencpn` build and install. No full147 or separate resource suite was repeated. Installed ELF: `8f5d3b898636c6f4461c0b1898569462ead32ff0403d7c9ab11f462947d24c08`; resource manifest remains `ffa75c9d097a143e58aa13f9300b04bebac0836ab849bd4284401df1866f4b2a`. The failed33 tree and original d5 remain preserved.

Actual official IHO S-64 test geometry—not an operational nautical ENC—now produces separate `XNSPPY01` BOYSPP254 and `XNSPPT01` TOPMAR257 raster calls at the same507,283 canvas anchor. Actual lookup/effective table is76. The head painter follows the unchanged visibility, source-slot, attribute and unique-platform guards; no model mutation, visibility override or chart injection was used.

**SKAGER software and OpenGL Day/Dusk/Night/Day-return pass.** Both entire chart regions return exactly to Day pixels, with no mask. Software passed and native-size originals were inspected before OpenGL was run. Full1280×800 GL Day/Dusk/Night originals and the pair crop were inspected: the fitted X remains distinct above the body without clipping. The collector records requested scale0.6 and explicitly checks OpenCPN's observed actual0.5826126536. Core saved/effective diagnostics remain76; the absent private adapter remains explicitly unavailable.

## Separate retained Standard GL failure

Standard software retains original `BOYSPP11` and exact Day-return. Standard GL retains original `BOYSPP11`, but its light-description text moves14px left on Day-return:503 changed pixels, chart-relative bbox526,280–595,299. The glyph pixels are identical after translation. The whole-chart assertion remains failed and its originals are retained.

Exactly one isolated, no-build Standard-only GL control on frozen33 reproduced the identical503-pixel shift. Every corresponding whole chart at Day/Dusk/Night/Day-return is byte-for-byte equal between33 ande1, with no mask. This proves that the observed shift predates the terminator repair; it does not waive Standard behavior or establish a whole-suite pass.

Read-only source review identified a consistent stock mechanism: RenderT_All initially measures spec-font `X` into avgCharWidth; Standard ASCII GL overwrites that width using cached-font `M` only on cache creation. Theme clearing can create a fresh text object while preserving that font cache. The original light rule uses xoffs2. This source inference is separate from the observed exact14px translation and does not claim copied runtime font-metric values.

Original failure and controls are preserved. Native Windows, actual private DLL/host and boat acceptance remain separate pending gates. No CI, boat, hardware output or source modification occurred during these captures.
