# SCRUM-275/276 optional actual-package scenes

Tooling-only increment on `bbccf16`, selected in Jira comment 10829. No new native
images, application build, CI dispatch or boat operation occurred. An eligible
independently audited package target is still required; this change does not
create one or accept an older application as implementing these features.

Two optional `--iho-scene` names extend the existing disconnected package
collector. The default yellow/Seattle matrix, exact yellow pixel proof, package
and copied-profile checks, truthful software/OpenGL checks, timestamps and full
Day-return equality are unchanged. Both new scenes are **review-only**: retained
images must be inspected, and source attributes do not prove selected symbols or
successful paint. The workflow captures both SKAGER and Standard in the existing
Day/Dusk/Night/Day cycle, stops at the first failure and retains its evidence.

| Name | Center (latitude, longitude) | Actual co-located source objects |
|---|---|---|
| `light-fog` | -32.3760351, +61.0307025 | LIGHTS32 white, nominal 20 nm, no sector attributes; FOGSIG33 |
| `sector-rwg` | -32.4843207, +60.9650566 | LIGHTS620 green 257–275 degrees / 6 nm; LIGHTS1881 red 295–300 / 6 nm; LIGHTS1882 white 275–295 / 8 nm |

The sector scene retains all three independent source records; their ranges and
bearings are not combined. Those three have no co-located structure/topmark or
orientation/special/visibility/status/quality fields. The original decoder fields
are in `scenes.json`, including light characteristics and feature identifiers.
The stock LIGHTS06 conversion and actual conditional selection remain the app's
responsibility. This scene can review the new CA fan and mixed-color central
location point together, without hiding independently charted structures.

The cell is the already retained official IHO S-64 **presentation test dataset**,
not an operational nautical ENC: `GB4X0000.000`, SHA256
`c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
It was re-read directly using pyogrio 0.13.0 / GDAL 3.12.4, `UPDATES=APPLY`, and
Shapely 2.1.2. All Point layers were inspected for exact co-location, not merely
selected LIGHTS rows. Null fields/empty lists were omitted; remaining attributes
are copied from the decoded cell. No feature data was created or modified.
The source fixture is retained in the earlier
`scrum264-final-light-captures/.local/colored-light-capture/fixture` workspace.

Both views request 0.6 pixels/metre and require exactly 0.5826126536 in actual
canvas diagnostics at the existing 1014×566 chart size. This is the existing
canonical IHO scale calibration, not a claim of newly observed sector-scene
geometry. LIGHTS32 at that center/scale has separate c24 software evidence in the
275 canvas work; this collector adds no Windows, OpenGL or 276 coverage by itself.
Any actual scale, coordinate, quilt or renderer mismatch fails without retry or
loosening the check. Reversed longitude/latitude is explicitly rejected.

The optional target field can be `"iho_review_scenes": ["light-fog", "sector-rwg"]`.
The existing limit remains **two** unique optional scenes from the now four-name
allowlist, preserving the serial 30-minute job boundary and software requirement.
Existing `lateral` / `cardinals` remain selectable. No arbitrary coordinates or
new remote operations were exposed.

Validation: five focused `NamedIhoScenes` tests passed, including unchanged yellow
routing, existing source locks, new attribute tampering and reversed-coordinate
refusal. Python syntax and diff checks pass. The first local test launcher lacked
Pillow; the existing bundled Python/Pillow runtime ran the checks successfully.
No source or assertion was changed for that environment issue.
