# All-round lights: supplied artwork boundary

Read-only SCRUM-15/264 review at local `1b1fde26478802959d0642b41efc03cf4ecf4952`.
This records a visible difference; it does not accept complete chart conformance.

The immutable prototype's `SYMBOLS.md:25` defines the fictional Långskär sector
light. `src/chart-symbols.json:155–188` supplies green, white and red sectors
spanning 164–236 degrees and illustrative compact/expanded radii of 44/340.
`src/light-sectors.js` has no separate all-round portrayal. The circle-and-rays
central point in `src/chart-marker-art.js:22` is distinct from a range ring;
SCRUM-275 already reuses that point for eligible lights.

The real IHO lighthouse scene contains LIGHTS RCID32, a white light with nominal
range 20 nm. Pinned OpenCPN `s52cnsy.cpp` selects all-round portrayal for this
non-sector light in `LIGHTS06` (around line 1415). `_selSYcol` (around line 432)
uses nominal-range bands plus a color offset, producing
`CA(OUTLW,4,LITYW,2,0,360,17,0)`. Its circle is **not** a geographically scaled
20 nm range boundary. The original code itself describes this circle as an
OpenCPN extension; this record makes no independent S-52 compliance claim.

The original core/private `RenderCARC` paths retain their geometry and renderer
scaling. `src/integration/ChartCaFan.h:59–60` deliberately excludes absent,
sub-one-degree and full-circle sector sweeps from SCRUM-276's compact sector
paint. Consequently, the actual Day/Night lighthouse screenshots still show
the outlined upstream circle around the prototype central point. The Night
circle also retains its upstream yellow/brown appearance.

There is no supplied full-circle design to transplant. Removing the circle,
substituting the fictional sector radius, or filling it with a sector wash
would be an unsupported design extension. A separately selected refinement
could modernize its paint while retaining classification, radius bands,
visibility and renderer scaling, but it must not be presented as an exact
copy of supplied all-round artwork. The same distinction applies to physical
towers for which the prototype supplies no corresponding custom body.

No application changes, tests, rendering, native build or boat actions were
performed for this review. The source findings were independently checked
against the immutable prototype and pinned OpenCPN implementation. Root also
viewed the existing exact `27e93e4` software/OpenGL Day/Night captures; those
captures qualify only their separately recorded building-paint increment.
