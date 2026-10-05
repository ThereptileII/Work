# Final combined Linux chart and wordmark captures

The 16 first-attempt captures passed on 2026-10-03 local time: SKAGER and Standard, each Day → Dusk → Night → Day, using the software renderer and the actual OpenGL drawing path. All 16 original PNGs were visually inspected. Four application sessions exited normally. There were no failed attempts, retries, relaxed predicates, image edits, or replacement screenshots.

This is a bounded Linux developer visual gate for SCRUM-235/236/253/254, not Windows, release, route-label/card, or boat acceptance. The scene contains no active route. A separately reported software waypoint/card probe failure remains outside this gate and is not resolved by these captures.

## Exact identity

- Source: `f0976cc65ea63d3ed6f60ac53ec38a856d11b66b`.
- Executable SHA-256: `97c6ee19043a3fe6bd66a099451c26c8a0f5e2f3cf77b8e2fe626f2ab3e38e3e`.
- Generated chart manifest SHA-256: `7c1f9e41f7dbd99b322a5c1cc9a4c0b73b8ad3ba3505c7da89f05ddc5e4abe39`.
- Pinned OpenCPN source: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
- Five generated chart resource identities and the relevant chart, typography, wordmark, theme and patch source hashes are in each renderer's `report.json`; `independent-verification.json` records a separate read-only source, resource, screenshot and pixel audit.

Both collectors checked the clean frozen source, independently reconstructed all nine patches in a temporary private Git index, checked exact staged executable/resource bytes, and verified runtime loader and screenshot diagnostic identities. The cache and original prior cache were read-only throughout. The wrapper grants writes only to its own preparation/output directory and private temporary directory.

The executable honestly reports `DEVELOPER TEST BUILD`, test fixtures enabled, and `test-loopback-only` output policy. The four development CMake options remain ON as recorded in `staged-inputs.json`; no production qualification is inferred. Loader self-tests did not initialize a profile or load plugins. Real capture sessions used fresh owned temporary profiles, no Demo/replay/scenario activation, and no pilot command or physical connection.

## Observed result

The approved SKAGER wordmark has no rectangular matte in any theme. Each of the 16 images passes 700 empty-background pixel probes, with six separate main-letter columns and three APP columns. Night blends exactly into header RGB `(12,17,21)`. Source identity guards still protect the unchanged approved geometry. The pointer was moved to `(5,5)` before settling and capturing, avoiding tooltip occlusion without altering image bytes.

The updated ordinary surfaces and built-up fill appear in the real public ENC view. `XNBUA` now shares `LANDA`; counts below intentionally combine land and built-up pixels and do not independently prove the geometry of either feature class.

| Renderer / theme | Water RGB / pixels | Land and built-up RGB / pixels |
| --- | --- | --- |
| Software Day | 213,229,229 / 352260 | 238,238,226 / 102499 |
| Software Dusk | 52,79,89 / 352565 | 78,97,93 / 102523 |
| Software Night | 14,23,28 / 352923 | 29,41,37 / 102578 |
| OpenGL Day | 212,228,228 / 357245 | 237,237,225 / 103783 |
| OpenGL Dusk | 52,79,89 / 358134 | 78,97,93 / 103809 |
| OpenGL Night | 14,23,28 / 358279 | 29,41,37 / 103840 |

The OpenGL Day one-channel-step difference is the pinned area-fill conversion `round(c*255/256)`; the collector checks it explicitly. Night retains brighter sounding and safety ink against the dimmed surfaces, while ownship and its COG predictor remain visible. This scene is not an exhaustive safety-symbol or route-state qualification.

For every style/renderer pair, the complete chart rectangle `(80,68)-(1094,634)` and wordmark rectangle `(8,8)-(174,58)` are pixel-identical between initial Day and return to Day. Independently, all six Standard Day/Dusk/Night chart rectangles are pixel-identical to the exact successful `1356fd1603aacbea04d7081d16331e9a181180bb` baseline. SKAGER Day/Dusk water colors remain unchanged; the intentionally changed built-up and depth roles are not incorrectly compared to the old colors.

The strict 3,232-pixel top/bottom edge-strip equality probe passes for every image; deliberately supplying the top strip as the bottom is rejected in all 16 cases. No tolerance was added. Shared flat-water pixels are expected; only a completely repeated strip indicates the tested wrap defect. Bottom strips additionally require nonuniform chart content.

Standard retains its pinned dense stock text, large depth-unit label and dark stock Dusk/Night palette. OpenGL shows its light-sector arc differently from software; identical Standard baseline rectangles establish that this comparison view did not newly change that rendering. These limitations are retained in the originals, not cropped away or described as new fixes.

## Environment and input

The public NOAA `US5SEAFL.000` real ENC is active in the quilt (chart type 5), centered at 47.6, -122.36, scale 0.150000006 pixels/metre. Source ENC file hashes are in each report. An owned realtime loopback RMC sender supplied that fixed public location, SOG 3 kn and COG 90° with current timestamps; reported source ages ranged 33–241 ms. The input is controlled simulation, despite the navigation adapter correctly labeling fresh received NMEA values `LIVE` / `Measured`. No private boat data, hardware, replay, Demo provider, or pilot commands were used.

Actual OpenGL uses `llvmpipe (LLVM 22.1.8, 256 bits)`, OpenGL 4.6 compatibility profile, Mesa 26.2.2-arch1.1, under Xvfb. This exercises the GL rendering implementation, not physical GPU or Windows driver behavior. Fontconfig resolves Segoe UI Variable Display, Segoe UI and Arial requests to Liberation Sans on this Linux host; source policy hashes and matches are recorded. Native Windows glyph metrics remain a separate gate.

## Retained evidence

`software/` and `opengl/` contain eight untouched PNGs each, corresponding full diagnostic JSON, the report, loader and controlled-input evidence, and launch/application/collector logs. The PNG hashes were rechecked after byte-for-byte copying. `files.sha256` inventories this evidence. Only benign Pillow deprecation warnings appear in collector/audit stderr.

The complete owned temporary profiles and caches remain at `/home/standard/Projects/X-nav-worktrees/scrum253-night-canvas/.local/final-capture-prep/output/capture-f0976cc-{software,opengl}/`; bulky generated SENC and profile databases are not copied into Git. The collection scripts are retained under `collector/` as run, including absolute isolation paths and expected-identity argument requirements. Do not rerun them from this evidence directory without setting up an isolated output location. The separate `verify-evidence.py CACHE_PATH` is read-only and checks the retained results against the staged exact cache and original baseline; its captured output is provided.
