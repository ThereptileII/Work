# SCRUM-264: public ENC scene audit and Pier 57 before capture

This receipt records real chart data and actual application drawings before the
SCRUM-264 glyph increment. It does not qualify the new glyphs or all marine-mark
families. No production source, original fixture, existing coast collector,
frozen cache, boat profile or hardware was changed. No build or CI was run.

## Source and update identity

`cell-summary.json` contains the exact byte lengths, SHA-256 values, DSID and
counts. `features.json.gz` is the deterministic compressed full feature audit;
`inspect-public-enc.py` reproduces the read-only GDAL inspection. It used
pyogrio 0.13.0 / GDAL 3.12.4 with explicit `UPDATES=APPLY`. Independently reading
with `IGNORE` and `APPLY` changed US5SEAFL edition 2 and US5SEAFK edition 1 from
UPDN 0 to UPDN 1. All source bytes were hashed before and after reading and
remained identical. The application was given the same exact `.000` and `.001`
files, with fresh disposable profiles and SENC caches.

| Cell | BOYLAT | BOYISD | BOYSAW | BOYSPP | BOYCAR | BCNCAR | LIGHTS | TOPMAR |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| NOAA US5SEAFL | 0 | 0 | 0 | 6 | 0 | 0 | 19 | 0 |
| NOAA US5SEAFK | 0 | 0 | 0 | 1 | 0 | 0 | 2 | 0 |
| IHO S-64 GB4X0000, official test data | 22 | 1 | 8 | 23 | 11 | 6 | 151 | 50 |

The retained nearby real NOAA cells do not provide the missing buoy classes.
The S-64 cell is an official display test fixture, not real-world geography;
none of its proposed scenes was launched here. No new chart was downloaded.

US5SEAFL input hashes:

- `.000`, 841005 bytes: `7e474bea96e7a84c6f3ee6b44ff7db4288029fa47ac2a285594e18469dbf518c`
- `.001`, 8709 bytes: `504440d201453548aae160c4c5340d55d480bd39cd57061b7fc03262968e062d`

## Bounded scenes and expected source branches

`scenes.json` supplies exact attributes, positions and pinned lookup records for
five proposed views. Only the first was captured. Coordinates below are
latitude, longitude; scale is the application's viewport pixels-per-metre input.

| Scene | Center | Scale ppm | Expected Simplified glyphs |
| --- | --- | ---: | --- |
| NOAA Pier 57, actual capture | 47.60605, -122.34281 | 0.6 | BOYSPP11 and LIGHTS13 |
| NOAA traffic separation buoy T | 47.5759311, -122.4512408 | 0.6 | BOYSPP11 and LIGHTS13 |
| S-64 lateral test objects S7/S6 | -32.5186315, 61.0216421 | 0.3 | BOYLAT13 and BOYLAT24 |
| S-64 isolated/safe-water test objects | -32.54495755, 61.03466285 | 0.25 | BOYISD12 and BOYSAW12 |
| S-64 four cardinal test objects | -32.37658945, 61.0300087 | 0.12 | BOYCAR01/02/03/04 |

Pier 57 feature extent is longitude -122.3428703 to -122.3427489 and latitude
47.6059633 to 47.6061367. BOYSPP RCID 23 and 24 are named warning lighted buoys
B and A: BOYSHP 4, CATSPM 27, COLOUR 1,11, COLPAT 1, SCAMIN 29999.
Their Simplified lookup id 1058 / RCID 31110 selects BOYSPP11. The data does not
assert a yellow X topmark; no TOPMAR feature exists in this cell. A generic
lookup must not be misreported as additional physical attribute evidence.

Colocated LIGHTS RCID 39 and 43 have COLOUR 1 (white), LITCHR 2 and no VALNMR,
CATLIT, sectors or orientation. Pinned lookup id 1131 / RCID 31183 invokes
LIGHTS05. In the pinned `libs/s52plib/src/s52cnsy.cpp`, LIGHTS05 delegates to
LIGHTS06 (lines 1012–1032); LIGHTS06 defaults missing VALNMR to 9 (1353), takes
the non-sector flare branch (1431), and `_selSYcol` maps white/yellow/orange to
LIGHTS13 (407–420). The emitted flare angle is 135 degrees. This is no proof of
all-round circle or sector-arc rendering. These lookup conclusions are source
analysis, not an instrumented trace of the renderer's emitted instruction.

## Actual before-capture result

Both software and OpenGL runs passed all eight scene checks on their first
attempt and exited normally. Each saves SKAGER and Standard Day, Dusk, Night
and Day-return: 16 original 1280 × 800 images, matching diagnostics, full
reports, controlled-input receipts and logs. All file bytes are inventoried in
`sha256.json`. The retained profile subsets include exact configuration and
application logs (raw logs and GL-capability JSON are losslessly gzipped); complete private profile contents are inventoried, not
silently omitted from provenance.

- Exact application commit: `a3e84771652c920479517f0d16a1dd6133c440d2`
- Installed ELF SHA-256: `dfb404933197888e137eef709d2866d23205e973543b3ab8cd1335654ab75a95`
- Chart manifest SHA-256: `c92bcdcfb19d2a4f0578b62f30d94eaf3ff59c3fb1705caa86a65e27a9f3309f`
- OpenGL: actual llvmpipe LLVM 22.1.8, Mesa 26.2.2, compatibility profile.
- Build: developer test fixture, loopback-only output policy; no Demo, replay,
  pilot command, private boat data or production fallback.

The collector checks exact source, all nine patched upstream inputs, executable,
resource and stock-resource identities; exact software mode, style, theme,
viewport center/scale/size; one real US5SEAFL quilt reference; fresh owned
loopback input; known controls; and clean exits. The source is read-only through
bubblewrap; only the new collector output directory and private `/tmp` are
writable. The two renderers use separate profiles and displays 237/238.

The disposable profiles explicitly request **Simplified**, `nSymbolStyle=76`,
and must retain that value after clean exit. Current diagnostics do not expose
the in-process symbol-table enum, so this is a checked configuration boundary,
not a direct enum observation. The former coast scene used Paper, value 82;
those screenshots do not establish Simplified buoy coverage.

This separate collector requires over 1000 pixels of each expected water and
land/built surface in its close view. It deliberately does not claim the old
coast view's density, edge-wrap negative control or historical rectangle
comparison. The existing strict coast collector was not edited. The old 1356
report remains only an exact historical identity anchor. Original screenshots
are unmodified; no synthetic mark was injected.

Independent inspection of the actual software and GL SKAGER Day/Night images
confirms both Pier 57 buoy/light pairs are visible around image coordinates
(584,343) and (592,360), with labels and the chart's ordinary overlap. The
Night stock glyphs are subdued; this receipt does not declare them improved or
qualify a new design. Existing brown piers, 148 DIP header logo and GL piano
return behavior belong to this frozen before source. Other saved themes and
Standard images remain available for independent comparison; no all-pixel
Day-return equality or broader route acceptance is claimed.

Linux drawings do not establish Windows font metrics, native DLL behavior,
physical GPU rendering or boat acceptance. Missing real-world family coverage
remains explicit.

## Reuse for the later comparison

The executed collector and wrappers are preserved verbatim in `collector/`.
Private runnable preparation is at
`/home/standard/Projects/X-nav-worktrees/scrum264-real-enc-scenes/.local/pier57-capture`.
Its `inputs/{app,upstream,build,install}` links address the read-only isolated
`/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29` cache; exact
`frozen-inputs.json` and `staged-inputs.json` copies are included. Use a new
output phase, fresh profiles and explicit final source/ELF/manifest identities
for a later run. Do not overwrite these before outputs. The forthcoming
124 DIP wordmark requires a deliberate corresponding component-bound update
in a new collector revision; the captured script's 148 DIP checks stay intact.

Executed command pattern, once per renderer (`software` on `:237`, `opengl` on
`:238`), with the matching historical renderer report:

```sh
SKAGER_CAPTURE_PYTHON=/home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  .local/pier57-capture/run-capture-isolated.sh pier57-a3e-software \
  --display :237 --renderer software \
  --expected-commit a3e84771652c920479517f0d16a1dd6133c440d2 \
  --expected-exe-sha256 dfb404933197888e137eef709d2866d23205e973543b3ab8cd1335654ab75a95 \
  --expected-manifest-sha256 c92bcdcfb19d2a4f0578b62f30d94eaf3ff59c3fb1705caa86a65e27a9f3309f \
  --baseline-report /home/standard/Projects/X-nav-worktrees/skager-product-fidelity/docs/evidence/skager-chart-1356fd1-linux/software/report.json
```
