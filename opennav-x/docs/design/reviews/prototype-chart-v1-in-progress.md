# XNav presentation v1 — development review, not acceptance

Reference: immutable prototype SHA-256
`b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
Linux captures use the real pinned OpenCPN canvas and public NOAA US5SEAFL ENC.
No boat/private charts or artificial chart objects were used.

Initial implementation changed only eleven palette roles; symbols, lookups and
conditional navigation instructions stayed byte/semantically identical.
The first coastline-only capture exposed upstream's lazy S-52 initialization.
The correction verifies the packaged palette at mode selection, independently
of whether a vector chart has been loaded. Actual ENC initialization verifies
the resource hashes again before constructing its presentation library.

The corrected software ENC pass retains its upstream quilt identity and more
than twenty distinct interior colors through Day → Dusk → Night → Day. It
closes normally. This proves rendering continuity, not navigational/visual
acceptance. The executable in `evidence/local/prototype/chart-v1-enc-xnav-pass3`
has SHA-256 `fb8ac6efb3c6821faf3ca448d2fac944c82346e9949a76d5e09cdb42baf392a2`;
it contains uncommitted chart work on local `f6d0cb2`, so a committed rebuild
and native replacement evidence are required.

Observed, still unresolved:

- Water/depth-area colors follow the derived palette; built-up land uses the
  independent stock CHBRN role and still appears gold in Day.
- Stock chart text hierarchy/density and some very large area labels differ
  visibly from the reference. Check the native renderer before changing fonts.
- Night's original dark symbol/text palette was designed for a darker stock
  background. Contrast against the new backgrounds needs explicit hazard review;
  do not accept or deploy this presentation as navigation-ready.
- Native chart selector strip, rectangular floating surfaces, compass glyph,
  chart pane border, rail hierarchy and footer remain visible mismatches.
- Route, ownship, selected objects and local/online AIS restyling remain open.
- The Linux GL path renders chart content but some floating control child
  glyphs are occluded. A diagnostic "visible" flag does not pass this visual
  check; software/GL control rendering needs correction.

The subsequent corrected coastline-only palette capture passes exact prototype
land/water pixel checks for Day/Dusk/Night/Day and exits normally. The Standard
ENC comparison also retains actual detail. Linux GL uses llvmpipe/Mesa 26.2.2;
its successful content checks are not physical GPU evidence.

Next evidence: exact-commit XNav/Standard comparison, software/OpenGL, native
Windows, resource-failure fallback, style and Legacy/Safe restart cycles, and
physical boat review. No PASS is entered in the conformance record.
