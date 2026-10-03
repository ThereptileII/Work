# SCRUM-264: named supplementary IHO review scenes

Collector-only change based on `fa4278e`, selected in Jira comment 10797. No application source, chart data, renderer, package or existing image oracle changes. No application launch, build, CI dispatch or boat operation was performed.

`capture-native.py --iho-s64 <GB4X0000.000> --iho-scene yellow|lateral|cardinals` now selects one bounded source-backed scene. `yellow` remains the default. A scene flag without the exact IHO-cell argument is refused; arbitrary coordinates and chart inputs are not introduced.

| Scene | Latitude, longitude | Requested / recorded actual px/m | Source inventory |
| --- | --- | --- | --- |
| yellow | −32.3471615, 61.169588 | 0.6 / 0.5826126536 | Existing BOYSPP 254 / TOPMAR 257 pair |
| lateral | −32.5186315, 61.0216421 | 0.3 / 0.3000000119 | BOYLAT 219/S7 and 224/S6; retained original scene also visibly contains 228/S5 and its green light |
| cardinals | −32.37658945, 61.0300087 | 0.12 / 0.1199999973 | BOYCAR 82/4/72/10, respectively north/east/south/west |

The two additions load the exact retained `scrum264-public-enc-scenes/scenes.json` (Git/LF SHA-256 `1da8b7c03d1bbbd35296531a2c7ea01c3c5aa9261d9b9e9336f6e6c3e3cfb1b3`) and exact historical actual-canvas diagnostic receipts. Those receipts supply observed scales, not equations or fabricated model objects. Both canonical LF and actual checkout hashes are recorded, allowing Windows CRLF conversion without accepting changed content. Selected feature attributes and source lookups are copied from the locked inventory into the capture provenance; they are expected source mappings, not proof of newly selected painter aliases.

The input remains the exact official IHO S-64 cell SHA-256 `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`: presentation-test geography, **not an operational nautical ENC**. Its source and staged bytes remain checked after shutdown. Single-cell quilt, 1014×566 canvas, exact center/recorded scale, no follow, Simplified table 76, actual requested renderer, application/package identity, copied-package integrity, fresh diagnostics and complete Day-return checks remain mandatory.

Yellow still runs the original exact head/body pixel probes in every theme. New scenes explicitly record `pixel_proof: null` and `visual_acceptance: review-required`; they require inspection of the original images. They do not acquire a glyph oracle from the existence of atlas resources. No existing failure is waived: the known historical yellow Standard OpenGL Day-return mismatch still fails the unchanged assertion.

## Optional package-review selection

The already audited target record may add:

```json
"iho_review_scenes": ["lateral", "cardinals"]
```

The existing yellow and Seattle matrix stays first and unchanged. Omission keeps the previous matrix. Only these two unique additions are permitted; software remains required, actual OpenGL fallback remains a failure, and execution remains serial and fail-first under the unchanged 30-minute cap. Each additional scene runs XNav and Standard, Day → Dusk → Night → Day. Both additions therefore add four application sessions per selected renderer. The duration of this expanded native matrix is not measured; the cap is not a performance claim. If an earlier original guard fails, later scenes will not run.

No generic `_bcngn`/`_slgto`, preferred-channel, real ORIENT-positive light, physical light-support tower or initialized private o-charts canvas coverage is claimed. The unchanged yellow/Seattle views and these two additional views cannot establish complete family acceptance.

## Focused validation

Seven focused Python cases passed: three named-selection/source-lock/dispatch cases and the existing four actual retained-yellow pixel/negative/viewport/whole-return cases. The latter retain twelve actual theme/style/renderer image probes, missing-head/body refusal and the known Standard GL return failure. New viewport fixtures combine unchanged historical chart observations with the current observer schema explicitly for guard testing; they are not new runtime evidence. Syntax checks for both Python tools, YAML parsing and actual PowerShell parser checks of all workflow blocks passed. No broad suite was run.
