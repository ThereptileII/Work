# SCRUM-308 — classified light-support tower ink

The source inspection establishes a supported cause class, not the identity of
an object on the boat chart. The LIGHTS alias path handles `LIGHTS11/12/13`.
The conservative CA point inventory refuses independently co-located `LNDMRK`
structures so that a generic light marker cannot replace or obscure a physical
tower. Neither path styles a separate light-support tower. The existing
SCRUM-284 all-round outline already applies and is unchanged.

The unchanged final prototype provides a circle and four rays for
`lighthouseArtwork()`. It does not provide physical-tower artwork. Its historical
v7 tower screenshot is not the final executable design. This correction is a
narrow **prototype ink-role extension**, not a claim of exact tower-artwork
conformance: the original tower representations intentionally remain intact.

The pinned public S-57 dictionary defines `CATLMK17` as tower and `FUNCTN33` as
light support. Only Simplified LNDMRK lookup 1147/RCID31199 with `TOWERS01`, or
1140/RCID31192 with `TOWERS03` and `CONVIS1`, can select owned aliases. The latter
remains a distinct conspicuous tower. The runtime additionally requires one
exact category and function, valid attribute types/counts, and matching symbol.
Absent CONVIS or explicit CONVIS2 is accepted for the ordinary tower; conflicting,
unknown, duplicate, or multiple values retain stock. Any ORIENT, STATUS, QUAPOS,
or QUASOU presence also retains stock, including malformed values.

The loader replaces earlier vector definitions with the final RCID342 raster
entries. The owned aliases retain those effective **14×26** shapes, every alpha
byte, pivot **6,22**, spacing and ordinary raster scale behavior. `XNLTWR01` uses
the existing XNGEO chart-text outline and DEPMD fill; `XNLTWR03` uses the distinct
XNBLO mark-black silhouette. Those roles follow the current Day/Dusk/Night
palette, including the existing single Night brightness adjustment. A pinned
per-coordinate two-ink transfer retains the ordinary shape's baked edges;
conspicuous geometry and transparency remain distinct. No pixel classification
runs at render time. No generic LIGHTS glyph is added above a tower.

The added atlas slots are `(980,1160,14,26)` and `(1004,1160,14,26)` with a checked
transparent moat. Original source symbols, tiles, lookups, priorities, labels,
light sectors, range bands, characteristics and co-location refusals remain
unchanged. The resource generator verifies input hashes, effective definition
order, alias collisions, slot bounds and a complete inverse XML comparison.
The verified presentation flag gates runtime selection; Standard, Legacy, Paper,
unknown objects and unavailable/invalid alias resources keep the stock symbol.

Focused verification uses public pinned resources and deterministic fixture
objects only. `tests/chart_light_tower_resources_tests.py` checks exact role
transfer, complete alpha, original tiles/lookups, source/alias mutations and
optional previous-resource inverse comparison. The shared actual-loader gate
`tools/verify-anchor-loader.py --seamarks` runs the pinned lookup selector and
RenderSY bodies, exercising ordinary/conspicuous objects, theme returns and
scale factors, and proving resource immutability and all refusal paths. It can
compare the core/private source bodies. This is not a full canvas, GL atlas
upload, physical GPU, native Windows or boat result; those remain integration
qualification gates. No claim is made that the boat's observed symbol was one
of these supported objects.
