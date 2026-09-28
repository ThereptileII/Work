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

## Native pass at ea869a9 (local 2a3c52)

Downloaded artifact SHA-256
`5b79f9d62e12ebcdf4050958cc956aff6faedf9447b99e948d4c049aba32a0f4`
and ZIP CRC/size verify; executable SHA-256
`d86aee8ba3acfd175bc07e380ad9a21d38bd1014519f0705aad9fc23a03faa36`.
The 102 integrated tests, 369 resource checks, 18 adversarial TLS cases and
actual-provider lifecycle pass. This is development evidence, not acceptance.

Reviewing the native public-ENC Night image confirms the dark chart-label/hazard
contrast concern. The floating chart controls are absent in the image despite
visible diagnostic flags. The coastline sequence fails after an older Day
diagnostic snapshot is read; ENC return-Day snapshots still say Night. Do not
count these as passed theme cycles. Wait for the requested observed mode and
verify actual control pixels before retaining replacement evidence. Fix the
owned native surface stacking and compare again on both renderers.

## Second general-ink pass

The working tree on `827e221` passes 377 resource checks, 120 integrated tests,
and four software plus four Linux GL public-ENC captures with clean shutdown.
Software Dusk/Night review confirms readable general monochrome chart text.
The unchanged baked dark raster symbols remain dim; they do not use the XML's
CHBLK RGB values, so blind RGB replacement would be unjustified. Day's general
ink remains pinned because prototype muted text loses shallow-area contrast.
Urban fill, density/large labels, chart selector, route/ownship and full symbol
review remain unfinished. Linux GL uses llvmpipe. Native/boat replacement is
still required; [local evidence](../../evidence/prototype-ais-navigation-chart-ink-local.json)
does not assert visual or navigation acceptance.
