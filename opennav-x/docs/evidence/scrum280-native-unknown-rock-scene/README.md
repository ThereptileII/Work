# SCRUM-280: optional native package unknown-rock view

Adds only `--iho-scene unknown-rock` to the existing package collector and its
workflow allowlist. Root may later select `iho_review_scenes: ["light-fog",
"unknown-rock"]` after independently auditing an eligible exact package. No
artifact target is created, no workflow dispatched and no application launched.
The two-optional-scene maximum, existing scene choices, yellow pixel oracle,
package/source identities, fixture-free requirement, disposable-copy isolation,
renderer truth, Standard controls, exact Day return and clean exit are unchanged.

The new choice binds existing immutable 45d evidence, without copying or editing
it:

- Scene inventory: `docs/evidence/scrum279282-45d-linux-canvas/collector/scenes.json`,
  Git/LF SHA256 `daab3f5b1723d5f51838646c55bc26c526519dffd252cd7e54e2adc0d62235fc`.
- Actual software Day observation:
  `docs/evidence/scrum279282-45d-linux-canvas/output/capture-45d73e8-s64-rocks-software-r2/s64-rocks-SKAGER-Day.json`,
  Git/LF SHA256 `07dd81c88df6ab8b61d7d9be04fdd2312fcf6b0d02602578c529b0bc45773962`.
- Official IHO cell `GB4X0000.000` remains locked to
  `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- Camera: latitude −32.3766581, longitude 61.03512875; requested 0.6 ppm,
  retained actual 0.5826126536, unchanged 1014×566 native chart canvas.

The retained source has UWTROC RCID1/2/3, WATLEV3 and absent VALSOU; actual
45d Linux painter traces selected UWTROC03 for these three. The nearby awash
RCID38 retains WATLEV5 and absent VALSOU; its actual Linux selection was
ISODGR51, not UWTROC04. Both observations remain historical Linux evidence.
The new collector records original attributes, not candidateRaster assumptions,
and explicitly has no native glyph/selected-alias pixel oracle. Native
screenshots still need review; source/camera presence alone is not selection
proof. No chart attributes, safety depth/contour or original geometry is changed.
This is official presentation-test geography, not operational nautical data.

`tests/recovery_capture_inputs_tests.py` passed **28 tests** locally. New checks
exercise the actual retained viewport, reject wrong/reversed coordinates,
scale/follow/quilt changes and scene mismatch, permit normal checkout CRLF,
and reject one-byte edits to either locked inventory or observed viewport.
Existing package refusals and retained-image checks also remain passing.
`focused-tests.log` is the original output. No Windows runtime, app build,
full regression suite, CI, network acquisition or boat operation occurred.
This tooling stays separate from candidate 17ab044 until explicitly reviewed.
