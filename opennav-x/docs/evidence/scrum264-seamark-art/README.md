# SCRUM-264: bounded classified buoy and light artwork

Base: a3e84771652c920479517f0d16a1dd6133c440d2. The preceding full-family audit is
8ebcf6e03502a79b351c40b94086acffbddc0242 and remains the mapping/gap record.
This increment implements **eleven** authored assets, not the whole symbol
family. No app painter, navigation, upstream source, chart data, profile, helper,
licensing or original resource is changed.

## Exact boundary

* Eight aliases XNLAT013/014/023/024, XNCAN072/073 and XNCON066/067 redirect only
  uppercase marine BOYLAT Simplified lookup ids 1029–1044. All sixteen full
  RCID/type/table/attribute/instruction identities are checked. Source lookup
  count and order stay fixed; all original symbol nodes remain intact. RCIDs
  60001–60008 must not collide with any RCID anywhere in the input XML.
* BOYISD12 and BOYSAW12 receive isolated 24 × 28 bitmap bounds, pivot (12,14).
  Their exclusive Simplified lookup selections remain byte-equivalent.
* LIGHTS13 gets the exact prototype circle and four rays at 25/32 scale and
  `prefer-bitmap` **no → yes**. This is an explicit render-path change, not just
  a tile repaint. Old HPGL, color references and vector bounds are retained.
  `_selSYcol` still selects it for white/yellow/orange short-range flares; red,
  green, unknown, two-color, long-range/all-round CA, sectors and directional
  question-mark decisions stay upstream-owned. The raster remains upright,
  matching the prototype; it does not interpret the old flare angle as a new
  physical orientation. No sector geometry/range is imported from the mockup.

Each owned slot has a verified transparent two-pixel moat; all complete XML
bitmap rectangles are checked for overlap. New slots are x=244..564 step32,
y=1160, size24×28. All other atlas pixels, including invisible RGB, are preserved
except already separately authorized neutral/anchor/service/cardinal changes.

## Narrowed scope and retained failures

Initial generation refused LIGHTS13's pinned `prefer-bitmap=no`. That refusal is
retained in `first-generation-refusal.log`; the exact preference exception was
then reviewed and authorized. The strict reverse validator rejects reverting
that preference, moving its pivot, changing vector HPGL, duplicating a node or
lookup, changing classification, or redirecting the real white/orange selector.

The planned BOYSPP11 replacement was **not implemented**. Every existing user
omits COLOUR, and actual NOAA US5SEAFL Pier 57 buoys use white/orange. Painting a
yellow/X glyph there would lose real meaning. Eight new color-qualified lookup
clones would need a separately proved lookup-order/attribute-parser boundary.
The two proposed generic BCNGEN01 aliases were also withheld: no proof excludes
an inferred physical head overlapping a real TOPMAR. Existing four cardinals
are retained unchanged. Unmapped Paper Chart and fixed-beacon variants remain
explicit gaps. The persisted upstream Paper Chart default is a separate issue;
these buoy changes need an effective Simplified style to appear.

## Evidence and limits

`DAY_BRIGHT-native-contact.png`, `DUSK-native-contact.png`, and
`NIGHT-native-contact.png` were inspected at native 1× size. Can/cone heads,
preferred-channel bands, isolated-danger spheres, safe-water sphere and light
circle/rays remain visibly distinct. They are small supplied prototype glyphs;
this inspection does not establish boat sunlight, distance or DPI readability.
Night owned input colors receive .78 once; no global safety palette is dimmed.

`loader.json` records **30,500** focused actual pinned-loader checks, including
all eleven names, RCIDs, raster selection, bounds/pivots and all three decoded
atlas tiles, compared with independently rasterized SVG. It executes the actual
`_selSYcol` function for 15 single/two-color and all-round cases. The actual
private adapter `ValidateOwnedPresentation` also accepts this generated XML and
all three atlases. GL texture rectangles are checked without a GL context.

The private o-charts source's ProcessSymbols, BuildSymbol, GetImage and
GetGLTextureRect bodies are byte-identical to those executed. Its pinned
chartsymbols.cpp Git blob 30ae2477b4b8ba748a8cd6f046079e8607b8151f is verified
against the adapter source lock. Its independent LoadRasterFileForColorTable
reads the table-selected PNG from configFileDirectory and does not restrict
symbol names; the existing private patch supplies the verified directory and
strict no-CWD parser. This is table/loader source proof, **not** private DLL
execution or actual encrypted-chart rendering acceptance.

No full app build, CI, boat action or cache mutation was performed. Actual
software/OpenGL before/after chart scenes, native Windows/private adapter, and
physical-display recognition remain required. Parent owns combined validation.

The final affected checks total **65,671**: 45,116 seamark/negative checks,
1,462 anchor, 2,649 service, 8,708 cardinal, 1,255 Day-neutral and 6,481 complete
XML inverse checks. Existing Dusk/Night PNG goldens remain unchanged after
reversing only the eleven new independently proved slots. The whole-resource
suite was not repeated; its affected existing functions and unchanged XML
inverse block were run against the single generated output.

Two bounded harness/oracle corrections are retained. Reading the manifest from
JSON produced lists while the existing Day test expects the generator's RGB
tuples; the runner restored those tuples. The Day-only test also had whole
Dusk/Night PNG goldens; it now reverses the newly proved slots before comparing
the same golden hashes. Neither correction changes product output or weakens
an atlas exclusion beyond the exact eleven rectangles. Anchor/service/cardinal
checks were not repeated after they passed. See `affected-first.log`,
`affected-day-oracle-first.log`, and `affected-remaining-pass.log`.

Reproduce resource checks with `tools/test-seamark-resources.py --source` pointing
to pinned `data/s57data` and `--generated` to this generator's output. The
existing `tools/verify-anchor-loader.py --seamarks` compiles actual loader and
conditional methods; `--private-source` verifies the separately pinned private
loader bodies. `loader.json` retains the exact compilation inputs and hashes.

## Combined integration check

Root integration `ebebd3a` combines the structural fill/outline changes with
these eleven glyphs. One complete `chart_presentation_resources_tests.py` run
passes **66,093 assertions**, including whole-tree inverse rules, unchanged
resource exclusions, deterministic generation, Windows checkout normalization
and malformed-resource refusal. The merge preserves both sets of explicit
paint exceptions. See `combined-resource-check.json` for exact input hashes.
No full application or Windows result is implied.
