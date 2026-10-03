# SCRUM-264: actual red/green light aliases on 5c05eb5

Both bounded lateral runs passed on the first attempt: twelve original screenshots, four normal application exits and no weakened assertions. The real red and green short lights use compact circle/rays in SKAGER, while Standard retains its stock flares. Each renderer passed exact whole-chart Day → Night → Day equality for both styles, and exact historical963 Standard Day chart equality without a mask. No application build, product change, CI or boat action was performed by this capture task.

## Identity and scope

- Clean source: `5c05eb55c15b67d4014df55896d95452e9cc9d64`.
- Installed ELF: `0e7f0500f79f8ca917b3ba0853ce3c205d68e6e1beaffd06377283950e32fd73`.
- Chart manifest: `beff73cae53211c4ef0b0c3fa2ed83e5290680f19f48781646e1f1d0d4f88e53`.
- Official IHO S-64 GB4X0000.000: `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
- Renderer paths: actual software and Mesa llvmpipe OpenGL; not a physical GPU or native Windows.

The build owner supplied the sealed inputs after build/install and 147/147 tests. This task independently verified them, all nine applied patches, staged resources and stock resource locks before launch, and rechecked source/executable/resource/fixture hashes afterwards. `inputs/` retains those receipts. The only compilation here was the small actual-header layout program used to read process fields safely; no application object or shared cache was changed.

This is official presentation-test geography, not an operational nautical ENC. Original chart and SENC bytes are excluded. Source rights/provenance remain documented in `docs/design/reviews/scrum256-official-cardinal-test-source.md` and `docs/evidence/scrum264-public-enc-scenes/`.

## Genuine source objects and actual render boundary

The original lateral viewport is unchanged: latitude −32.5186315, longitude 61.0216421, scale 0.3 pixels/metre. Existing S7 green BOYLAT219 and S6 red BOYLAT224 remain. Added coverage is the genuine S5 green BOYLAT228 and its explicitly linked LIGHTS227, both at latitude −32.5146737, longitude 61.017694. LIGHTS227 is COLOUR4, LITCHR2, SIGGRP(1), SIGPER5; the visible description is Fl G5s. The red source is LIGHTS223 at −32.5134656,61.0240359, COLOUR3 and the same five-second character. The earlier curated scene omitted S5; its green light was already visible, not newly fabricated or inserted.

The exact original cell was reread using pyogrio/GDAL. Runtime traces independently confirm both lights' attribute lists contain `catgeo,COLOUR,LITCHR,SIGGRP,SIGPER`, with no ORIENT. The scene metadata and green-source receipt retain source attributes and coordinates. OGR object index is RCID minus one for these records and is explicitly distinguished from RCID.

For both renderers the actual method-entry trace observes original LIGHTS11/12 with vector definition86. The SKAGER raster entry then observes XNLIT011/012 with raster definition82 at the same real point; red canvas(587,78), green(375,126). No inferior function calls or object writes occur. The actual-header receipt establishes offsets; exact symbol-entry breakpoints disable after the requested geometry is observed (at most 43 calls per probe), before the settled screenshots. Debugger stops affect initial timing; these are not performance measurements. Exit events are all code0.

The Standard raster probe intentionally only requires buoy draws, so its empty light-raster list is not an independent negative proof. Standard retention is established by the original vector-entry records, visibly stock flares and the strict complete historical Day image comparison. The collector/report is retained as executed. Its inherited `scope` wording says three scenes; this run actually covers one viewport with three selected buoy records. Similarly, the persisted-symbol receipt wording does not supersede the separate actual table76 trace.

## Original images and comparisons

Under each of `software/` and `opengl/`:

- `s64-lateral-SKAGER-{Day,Night,Day-return}.png`
- `s64-lateral-Standard-{Day,Night,Day-return}.png`

All are 1280×800 originals. Each has actual diagnostics; three 72×72 source-point crops retain S7, S6 and S5, with bounds anchored to the painter's real coordinates. Every unique Day/Night view and representative native-size red/green composition crops were inspected. Return images are pixel-identical across the complete chart rectangle `[80,68,1094,634)` and retained in full. No broad tolerance or exclusion mask was used.

`historical963/` contains the original Day SKAGER and Standard reference bytes. Standard Day is identical across the entire chart in each renderer. There is no historical Night baseline claim. For SKAGER Day, `before-after-diff.json` records 470 changed chart pixels in software and 404 in GL, all within the genuine red/green light neighborhoods. There are no other chart-pixel changes in this scene.

At native size, the old long flares are gone in SKAGER. The red can/green cone class heads, colored stems, real names/characters and small centered light rings/rays remain. Night marks are subdued and still visually identifiable on this display; this does not qualify physical boat recognition. The existing OverZoom warning remains. Buoy/light co-location is intentionally composed; circles are not removed merely to resemble an isolated prototype icon.

## Controls and remaining acceptance

Displays 247/248, fresh private profiles and a read-only fixture/build are isolated. Controlled loopback RMC uses the actual decoder; fresh LIVE/Measured values are required, and pilot/control remain disabled. It is explicitly simulated sensor input, not live boat navigation. The actual chart, source file, viewport, dimensions, software/GL state, presentation status, water ink, theme controls and normal shutdown are checked. Complete local profile inventories and selected configuration/log/GL receipts are retained without chart/cache binary payloads. Logs use deterministic gzip; all retained files are hashed in SHA256SUMS.

This run proves ordinary red/green no-ORIENT rendering and the stated Standard/return comparisons. Real ORIENT-positive chart rendering, private o-charts rendering, native Windows, physical GPU and boat acceptance remain open. White-light/Pier57/hatch evidence belongs to the separate coordinated collector; it is not claimed by these images. No preferred-channel, sector/directional, long-range CA or all-family acceptance is inferred.
