# SCRUM-378 symbol and light-sector source batch

20 October 2026. Base `3959aeb7`; pinned OpenCPN 5.12.4
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

Implementation commits: `17b202f3` (neutral ink), `dd68b1e8` (core/private
raster integration), `3dc6377a` (expanded sectors), `87446a10` (artwork/audit).
The containing integration commit wires generator and private-header inputs.
[`source-inputs.json`](source-inputs.json) records the checked source bytes.

## Results

| Check | Result |
| --- | --- |
| Pinned catalogue | All 1,105 effective entries and 75 shadowed definitions inventoried |
| Scalable resources | 43 names / 129 scheme entries; 15 additional native redraws |
| Source artwork | 210 original/flattened CSS render pairs identical; 387 size/scheme renders |
| Native artwork anchors | All 15 exact anchors and actual painted extents checked |
| Resource generation | Passed; existing five XML/atlas/RLE output hashes unchanged |
| Neutral paint policy | 1,001 assertions passed |
| Actual core raster dispatch / wx painting | 62 checks passed with final 129-entry table |
| Actual private raster dispatch / wx painting | 62 checks passed with the same final table |
| Scale negative control | Removing the Prepare percentage multiplier is detected |
| Actual expanded-sector software / Mesa GL methods | 158 checks passed |
| Existing compact-sector core regression | 576 checks passed |
| Light-sector palette | Passed six independent SVG color fixtures and Night brightness cap |
| Complete core patch stack | All 21 patches apply and independently reconstruct |
| Private renderer patch sequence | Applies with exact reconstructed source equality |
| Complete changed native translation units | `s52plib.cpp` and `s57chart.cpp` pass Linux syntax checks |
| Private adapter preparation inventory | 17 tests passed |
| Windows changed-unit helper policy | 7 tests passed on Linux; not native Windows execution |

The final native raster checks use generated table SHA-256
`11065bbfb0bb82bc642697d183819152471240dc1a9f046b5472a0c9d4096836`.
Their actual source hashes and commands are in
[`raster-checks.json`](raster-checks.json).
The earlier renderer-worker result used a preliminary table; it is superseded
by this final-table pair, not added to the test count.

Source/resource receipts and sector method hashes accompany this document.
The full generator ran before the final SVG-only anchor/fill refinements; the
final table, source reproduction and glyph renders were rechecked without
regenerating unchanged PNG atlases. The generated-header C++ syntax check and
final-table wx checks cover that final table.

## Review corrections

- Native HPGL geometry must map its own anchor to the existing bitmap pivot,
  rather than centering a tight view box. Four minor beacons remain native
  because a correct fit would materially shrink their mark.
- Hazard depth/drying/land fills retain their pinned categorical colors.
  Neutral object outlines do not flatten those roles.
- Independent review found a fractionally clipped `OBSTRN18` outline. The fit
  now reserves its actual pen extent; serialized painted bounds and all nine
  affected size/scheme renders were rechecked.
- The private renderer compared integer cached width with fractional requested
  width. Cache identity now checks both actual integer dimensions.
- The menu-to-renderer scale path exists. The owner's no-effect observation has
  not been reproduced on the boat; identified rendering defects are corrected
  without claiming that all causes of that observation are proven.

## Visual samples and limits

These are **source artwork contact sheets**, not application screenshots.
Each cell compares enlarged atlas artwork with final-size SVG rendering.
Coordinator review inspected Day and Night; all three sheets are retained:

- [Day](symbols-DAY_BRIGHT.png)
- [Dusk](symbols-DUSK.png)
- [Night](symbols-NIGHT.png)

The sheets do not apply the runtime 12% aid-size boost or Very Light neutral-ink
policy. They verify artwork/sharpness, not final chart composition.
The rendering component checks cover those runtime policies separately.

No full application build, CI run, native Windows result, private DLL runtime,
real ENC/o-charts comparison, GPU performance claim or boat operation belongs
to this batch. Unsupported/specialized symbols retain their documented native
fallback. The temporary GL uploads and new sector raster work still need
performance review in a future requested candidate.

Installed `bf489683` / Beta 2.6 is unchanged. SCRUM-378 remains Testing pending
the next candidate's native and boat validation.
