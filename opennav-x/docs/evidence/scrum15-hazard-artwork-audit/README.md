# SCRUM-15: supplied rock and wreck artwork boundary

Read-only inspection on `48c2f8b8d5d4103bcbaacc45a18c4e6a238d4c52`.
No application/resource changes, builds, tests, CI actions or boat access.
`source-inventory.json` retains exact source paths/hashes, all three complete
symbol definitions, point lookup consumers and nine original/effective tile hashes.
`prototype-computed.json` records a read-only Chromium inspection of the unchanged
supplied HTML at 1280×800/DPR1; external requests were blocked. This is source/CSS
inspection, not application canvas or native Windows acceptance.

**A narrow resource implementation is possible without changing classification.**
Replace only these final effective symbol portrayals in the owned SKAGER resource
set, preserve the conditional procedures, and leave the standard resource set and
unmapped symbols intact. These names are not equivalent to all rock/wreck objects.
There is no supplied textual contradiction between their three symbol definitions
and the prototype's three intended meanings. However, changes of shape, fill,
opacity and size require visual recognition/overlay acceptance; a bitmap change
alone is not proof of preserved visual communication.

## Exact supplied portrayal

Authoritative `docs/design/prototype/index.html`: final CSS at 116–118, artwork at
1203–1205, tone/graphic at 1230–1233; actual chart point size at 1169. The immutable
HTML hash is `b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
All three use no fill, round caps/joins, 1.2-unit strokes and group opacity 0.8,
scaled by **27/32** on the chart. Final chart-text stroke colors are Day `#687b7a`,
Dusk `#adbbb1`, Night `#758579`; stroke size before device scaling is 1.0125 pixels.
The prototype counter-rotates/zooms these point groups to retain screen size.
Do not copy its fictional coordinates, optional labels or hit/selection circles.

| Symbol | Supplied geometry | Existing bitmap / pivot |
|---|---|---|
| UWTROC03 | Diagonal X from ±4; four radius-0.7 filled dots on axes at ±7. Geometric box ±7.7. | 13×13 / (6,6), atlas (950,307), RCID2249 |
| UWTROC04 | Axial plus at ±5; four radius-9 corner arc segments, dash `1 3`. Geometric box ±8. | 13×11 / (6,5), atlas (973,307), RCID3164 |
| WRECKS05 | Outlined hull, mast/pennant and side marks; lower wave path with local opacity 0.55 (effective group product 0.44). Geometric box x−11..12, y−7..10. | 19×13 / (9,6), atlas (35,358), RCID2038 |

The last two supplied shapes cannot fit their old bitmap boxes at exact prototype
scale, including stroke. An implementation must allocate measured bounds and
preserve the geographic anchor through a matching pivot; squeezing/clipping into
old cells is not exact fidelity. Geometry boxes above exclude stroke expansion.

## Actual conditional and lookup boundary

Core source is the pinned `libs/s52plib/src/s52cnsy.cpp` under the prepared source
path retained in the inventory. Later matching strings at lines 4953–6119 are in
the disabled legacy block beginning at 3932; they are not additional live cases.

- `OBSTRN04` (1655–1863) handles UWTROC and OBSTRN, but emits **these rock names
  only for point UWTROC**, after `_UDWHAZ03` has not supplied isolated danger.
  UWTROC03: `VALSOU` absent and `WATLEV=3` (1820).
  UWTROC04: known `VALSOU≤20` with `WATLEV=4 or 5` (1759), suppressing the sounding
  in that existing branch; or absent `VALSOU` and missing/default `WATLEV` (1813,
  1823), except `WATLEV=2` uses LNDARE01 and `WATLEV=3` uses UWTROC03.
  Known deeper water and other known-depth states retain DANGER51/52 and soundings.
- `WRECKS02` (3239–3437) emits WRECKS05 only for a point with absent `VALSOU`, no
  isolated-danger substitution, and both `CATWRK`/`WATLEV` present: specifically
  `CATWRK=2,WATLEV=3`, or its residual default. `CATWRK=1,WATLEV=3` selects WRECKS04;
  categories 4/5 and water levels 1/2/4/5 select WRECKS01. Missing either attribute
  does not automatically produce WRECKS05 in this implementation—do not broaden it.
  Known depth retains DANGER51/52, `Wk`, sounding and QUASOU7's WRECKS07 overlay.
- `_UDWHAZ03` (521–614) retains safety-contour/associated-depth-area evaluation,
  ISODGR51 substitution and DisplayBase promotion. `CSQUAPNT01` (2177–2223) retains
  PA/PD/REP/question-mark overlays from QUAPOS. None is part of the replacement.
  Area outlines/fills, foul-ground cases, priority, radar behavior, SCAMIN and
  display/visibility policy must stay upstream-controlled.

Point lookups are UWTROC Simplified1276/31328 and Paper2509/30580 → OBSTRN04;
WRECKS Simplified1298/31350 and Paper2531/30602 → WRECKS02. CATWRK3 has earlier
FOULGND1 lookups. Six direct `$CSYMB` references also share the three names:
Simplified1680/31732,1681/31733,1693/31745 and
Paper2953/30981,2954/30982,2966/30994. Thus a final-symbol replacement affects
both SKAGER point tables and `$CSYMB`, not only a single Simplified lookup.
If Paper must remain stock, an additional scoped alias/selection seam is required;
changing only the shared dictionary cannot claim that narrower scope.

Original private source also has the live branches at OBSTRN04:1792,1846–1856 and
WRECKS02:3452–3460, at the exact path/hash in the inventory. This confirms separate
private ownership; it does not qualify its native renderer.

## Meaning and remaining evidence

The pinned/effective definitions and all nine Day/Dusk/Night bitmap tiles are
currently identical for these names. UWTROC03 and WRECKS05 retain CHBLK over DEPVS
filled danger geometry in their original vector definitions; UWTROC04 is CHBLK.
The supplied designs remove that solid shallow-water fill. The new WRECKS05 hull
and mast could be confused with the retained visible-hull WRECKS01; the new
UWTROC04 plus/dashed surround could be confused with the old uncertain-depth rock
language. These are specific recognition risks to compare, not evidence that stock
artwork is intrinsically required or that the prototype has reclassified data.
The fictional label “Rock awash” must not become a new assertion for upstream's
missing/default-water-level cases.

`chartsymbols.cpp:388–447,615–640` defaults `preferBitmap=true`; these definitions
have bitmaps and no override, so normal XML loading chooses raster **despite**
`<definition>V</definition>`. Keeping HPGL unchanged preserves source fallback,
but that fallback would remain visually stock. Resource replacement should not
rewrite HPGL, global CHBLK/DEPVS, conditional rules or neighboring danger symbols.

Required future proof is tightly targeted: exact SVG/theme/alpha derivation and
unrelated-resource inverse; actual core/private loader dimensions/pivots; original
conditional-output equivalence across depth/water-level/missing attributes and
isolated-danger cases; real software/GL Day/Dusk/Night/Day-return scenes showing
these three alongside WRECKS01/04, ISODGR51 and positional-quality overlays where
present. Retain depth text, visibility and selection behavior. Native Windows and
boat readability remain open. No such tests or captures were run in this audit.
