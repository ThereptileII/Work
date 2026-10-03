# Combined Linux chart capture: a3e8477

The exact combined candidate built and installed successfully, and **147/147 existing regression tests passed**. The actual software and Mesa OpenGL collectors completed **16 captures and four normal application exits**. A separate strict comparison found one unresolved defect: **OpenGL SKAGER's hot Day return changes the chart-selector border**. This evidence does not qualify that hot-return behavior as passed.

Source: `a3e84771652c920479517f0d16a1dd6133c440d2`.
Installed ELF SHA-256: `dfb404933197888e137eef709d2866d23205e973543b3ab8cd1335654ab75a95`.
Resource manifest SHA-256: `c92bcdcfb19d2a4f0578b62f30d94eaf3ff59c3fb1705caa86a65e27a9f3309f`.

## Build and isolation

The previously qualified private `scrum259-linux-6dafd29` worktree advanced to the exact clean source above. One incremental configuration/build/install ran, with **193 Ninja steps**, followed by the existing test suite once (**21.95 seconds**). It used two build jobs and a fresh private test HOME/runtime. The broader recompilation is recorded rather than described as only the small UI/resource edit set. The five unchanged standalone drawing fixtures were not repeated.

The capture process independently reconstructed all **nine pinned integration patches**, checked actual upstream working bytes, verified the generated source header, all installed manifest file hashes, and pristine Standard resource hashes. The loader self-test and every captured diagnostic confirmed the exact executable commit and developer fixture capabilities. `staged-inputs.json`, `frozen-inputs.json`, `receipt.json`, `seal.log` and compressed build/test logs retain those receipts.

The existing 78eccb8 cache stayed read-only. During capture the complete host filesystem, private source/build/install and cache were read-only; only the collector output and isolated `/tmp` were writable. Software used X display `:237`; GL used `:238`. Fresh SKAGER and Standard profiles received only bounded simulated loopback RMC at 47.6,-122.36 through the actual input path. Public US5SEAFL ENC source hashes match the historical 78eccb8 capture exactly. No model pointer injection, Demo activation, boat access, hardware output, CI dispatch or endurance run occurred.

The GL log records an actual OpenGL 4.6 compatibility context, Mesa 26.2.2 and llvmpipe LLVM 22.1.8. This is not physical GPU validation. No private o-charts adapter was loaded or exercised.

## Retained checks and results

Each renderer retains the original actual ENC/quilt, requested renderer, measured/fresh input, Day→Dusk→Night→Day controls, water/land content, wordmark, wrapped-edge rejection/negative control and clean process exit checks. All sixteen complete images and corresponding diagnostics are retained under `software/` and `opengl/`; normal exits were checked after each of the four successful profiles.

The historical chart comparison uses `(80,68)-(1094,634)` and wordmark `(8,8)-(174,58)`. Its **only chart exclusion** is the changed toolbar at `[883,1072) × [545,597)`, exactly 189×52 pixels. All four actual owned control bounds plus their source-defined 4-pixel gutters, and the actual titled X window geometry, independently agreed with that rectangle. Each renderer retains `toolbar-only-mask.png`. Full images remain unmasked. Clock, input-age and primary navigation caption pixels outside those historical rectangles are retained but were never part of their equality oracle.

- All **eight Standard historical chart comparisons** have **zero differing pixels outside that toolbar**. Complete read-only accounting independently checked the GL images after the fail-fast return assertion; it does not replace that failure with a pass.
- All wordmark comparisons are exact. Software SKAGER and Standard hot Day returns are exact. GL Standard's hot Day return is exact.
- **GL SKAGER's hot Day return fails:** 1,270 pixels differ within `[83,686) × [614,632)`, the chart-selector/piano border. The dominant 1,192 pixels change from `(7,7,7)` to `(83,100,95)`. No broader mask, tolerance, altered assertion, new capture or app rebuild was used to hide this.

The source clue is pinned `gui/src/piano.cpp`: the software pen uses `GetGlobalColor("CHBLK")` at line 153, and the GL texture-building pen does so at line 304. SCRUM-261 changes Day CHBLK to the new neutral role. The initial GL border retains the former value, while theme return uses the new value. An initial cached texture predating library selection is a plausible cause, **not yet a proven lifecycle diagnosis or repair**.

The first agent status message mistakenly reported the GL comparison as passed because a shell command continued to a successful log search after Python failed. The original `opengl/historical-comparison.json` and log correctly recorded failure throughout. The independent reviewer caught it; this report and receipt explicitly correct the claim. `complete-image-accounting.json` records every image comparison, including that failure.

## Actual paint evidence

Fixed sample rectangles were chosen from the historical image before viewing the new output; no color search moved the probes to passing locations. Original images, enlarged comparison crops and pixel transition receipts retain:

- FERYRT01 repeat `(104,168)-(130,185)`: software has seven solid pixels changing from `(197,69,195)` to exact Day area ink `(156,134,150)`.
- Plain cable boundary `(90,236)-(230,245)`: 102 solid pixels make that same exact transition.
- Wreck annotation/mark `(742,415)-(775,439)`: 28 solid pixels change from `(7,7,7)` to exact Day neutral `(83,100,95)`; antialias transitions are also recorded.
- The east tower sample `(1049,279)-(1068,308)` remains pixel-identical, showing that the change does not blanket-recolor all dark marks.

Both the collector owner and independent reviewer inspected original images. The reviewer inspected all 16: the ferry repetition and cable boundary remain distinguishable, muted mauve is visible in each theme, and softened Day neutral marks remain readable. Software solid area cores are Day `(156,134,150)`, Dusk `(184,160,177)`, Night `(113,99,110)`; GL Day's observed `(155,133,149)` is its existing renderer quantization. Dusk/Night neutral samples remain unchanged. The toolbar's former conspicuous pale outline is absent in these new captures.

Bright magenta rings, information/restriction ink and unchanged stock lighthouse/wreck geometry remain outside the scoped resource changes. Actual Seattle chart density/geography is not a paint defect. This fixture contains no CATCAM/cardinal objects. Native Windows, physical GPU, adapter-rendered charts and boat recognition remain separate acceptance work.

## Preserved first attempt

The first software attempt stopped before a screenshot at the former exact diagnostic string. Final source `ChartPresentation.cpp:444-445` now appends `OChartsPresentationStatus()`, and its default at `OChartsPresentation.cpp:80` is `o-charts: no SKAGER adapter loaded`. Actual diagnostics and logs showed verified SKAGER resources, not fallback.

Only the exact full expected string for both SKAGER and Standard was updated to include that explicit absent-adapter state. Core presentation selection, resource checks and all existing predicates remained strict. One fresh software retry followed; GL ran once with the corrected string. `first-attempt-status-contract/` retains the original collector, report, trace, diagnostics and application log. That log records normal frame/application cleanup; the failed capture report correctly remains `clean_exit: false` because its lifecycle did not complete, and its cleanup process exit code was not separately recorded.
