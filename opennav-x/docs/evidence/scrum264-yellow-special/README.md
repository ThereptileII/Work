# SCRUM-264: yellow special-mark body and explicitly fitted X

Implementation base: `d5d71356d806ea8c3518644d10728a24f1334d1d`.
This closes the supplied yellow special-mark artwork implementation gap without
assuming that every yellow buoy carries an X. The frozen d5/b8 executable,
build, chart cache and CI run remain untouched.

## Exact classification and draw boundary

The immutable `seamarks.json` special entry maps BOYSPP11 to yellow and says
“yellow X topmark when fitted.” `chart-marker-art.js` supplies the stem, yellow
segment, crossbar, water-filled position circle and separate X path. The new
source-locked derivative splits these exact paths at the same 27/32 scale;
`immutable-source-check.json` independently verifies original manifest hashes.
Night brightness .78 is applied exactly once; no new palette or sizing rule.

- **XNSPPY01**, RCID60013, tile(692,1160,24,28), pivot(12,14): body only.
  A verified SKAGER instance, Simplified table and actual selected BOYSPP11
  are required. Typed BOYSHP3/4/5/6/8 and exact single COLOUR6 are required.
  Unknown/missing shapes, other colors, any COLPAT, ORIENT or direct TOPSHP,
  malformed/duplicate relevant attributes and unsafe alternate list types
  retain stock. These known shapes already share this Simplified class glyph;
  the slender supplied stem is not a new physical pillar classification.
- **XNSPPT01**, RCID60015, tile(756,1160,24,28), pivot(12,14): supplied X only.
  It is drawn for an actual point TOPMAR with typed TOPSHP7 and exact COLOUR6,
  only through original empty Simplified lookup1262/RCID31314. Any populated
  instruction/rule or attribute-specific lookup retains upstream behavior.
  The head requires exactly one eligible same-chart floating platform at the
  exact x/y position, using pinned TOPMAR01/_atPtPos equality. Missing context,
  ambiguous platforms, non-finite position or more than 4096 platforms refuses
  the added head. The shared buoy predicate excludes CATSPM15/52, whose earlier
  lookup branches choose BOYSUP02/BOYDEF03, and rejects malformed lists.

The added head call is after existing ObjectRenderCheckRules and DC setup in
both actual DoRenderObject implementations. Category, SCAMIN and object
visibility remain upstream-owned; no second navigation-processing pass occurs.
Its original default instruction remains empty. A missing/invalid body or head
alias preserves no-draw behavior. Existing TOPMAR15/16 exceptions, Paper,
conditional topmarks, LIGHTS, labels, priorities and geographic positions remain
unchanged. Original symbol resources and all lookup nodes are untouched.

RenderRasterSymbol receives stable library-owned Rules directly: no shared Rule
mutation, synthetic rule-chain insertion, copied cache ownership or escaped
stack metadata. The normal raster path owns scaling, clipping and object bounds.
Core INST is a wxString value; API-17 private INST is a pointer. The shared guard
handles each actual ownership form, requiring a valid empty string.

## Focused evidence

- **490 resource assertions:** both aliases/metadata, original BOYSPP11 and
  all BOYSPP/TOPMAR lookups, exact path/color separation, all six owned raster
  tiles and whole preceding d5 XML/all-three-RGBA-atlas inverse pass. Every
  neighboring byte and RLE remains unchanged. Pivot/extent/neighbor mutations
  are rejected. Existing whole-resource tests now know only these two additions;
  the broad suite was not rerun.
- **43,962 actual-method assertions:** real pinned loader, PNG crop,
  RenderSY, DoRenderObject, FindBestLUP and lookup comparator run with fixture
  objects. The accepted predicates are checked against all actual Simplified
  BOYSPP lookup rows, including category precedence. The tests cover explicit X,
  absent/other topmark, original no-draw, instance/Paper/missing alias fallback,
  visibility rejection, null DC (GL branch dispatch), native DC, original Rule
  identity, unique/duplicate/other-chart platforms and bounded lists.
- Seven actual-method mutation controls reject bypassing existing light/buoy
  guards and the new fitted-head or visibility ordering. Core/private head and
  RenderSY bodies agree after normalizing only their compile macro. This is
  **not** full chart or GL painting: visibility policy, projection and painters
  are explicit fixture boundaries, with recorded draw calls. The actual
  existing visibility predicate is not replaced in the product.
- Actual core/private enabled s52plib and private integration-disabled objects
  compile. All nine core and both private patches apply; final patched bytes
  equal the compilation inputs. Sixteen adapter preparation tests pass with
  the new shared header in its copied/hashed dependency closure.
- All three contact sheets were inspected at native size. Body alone has no
  invented head; the separate explicit head composes into the supplied X mark.
  Night is subdued and tiny; this is not boat recognition/readability acceptance.

The final fixture uses explicit atlas positions; slot724/RCID60014 remains
reserved for the independent generic-beacon work. The original diagnostics are
retained: the first private compile exposed pointer-vs-value INST ownership;
a new negative fixture supplied an invalid string ORIENT to upstream's numeric
getter; and a new XML inverse test initially moved the bitmap element instead
of restoring its metadata in place. These were corrected without changing the
upstream parser, weakening guards or suppressing failures.

## Real source candidate and remaining validation

`actual-source-pair.json` retains genuine official IHO S-64 test records:
BOYSPP254 (BOYSHP3, COLOUR6, CATSPM27) and co-located TOPMAR257 (TOPSHP7,
COLOUR6), latitude **−32.3471615**, longitude **61.169588**. This is presentation
test geography, not an operational nautical ENC. The previously decoded GDAL
record was reread and the unchanged `.000` independently rehashed; no new chart
was downloaded or object fabricated. Runtime platform uniqueness still needs
the real chart-context check. Other known source pairs include261/262 and648/651.

Actual software/OpenGL canvas captures of this new two-object composition,
private encrypted-chart rendering, native Windows and boat readability remain
required. Independent TOPMAR visibility can legitimately leave only the body
visible; missing/ambiguous source evidence never gains an X. Yellow can/cone
BOYSPP15/25, unsupported shapes/colors and Paper retain their original distinct
portrayals. No full application build, broad regression suite, CI dispatch,
boat action or claim of all-family visual acceptance occurred.
