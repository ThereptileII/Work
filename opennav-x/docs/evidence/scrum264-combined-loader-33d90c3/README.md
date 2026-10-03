# Combined SCRUM-264 symbol loader verification

Source: `33d90c38cd1a628c21d0ca159647012d98649d2e`.

**46,918 actual pinned loader/method checks passed** against the combined resource
set, including all 17 owned tiles in Day, Dusk and Night. Explicit atlas positions
include yellow body692, generic beacon724 and fitted X756. The head-specific
minimum coverage assertion now selects by symbol name, independent of list order.

The independently checked manifest is
`ffa75c9d097a143e58aa13f9300b04bebac0836ab849bd4284401df1866f4b2a`.
All five manifest payload hashes and sizes matched. All seven generated files,
including the header and manifest, were copied byte-exact into a private directory
before execution. `resource-seal.json` records each identity; generation was not
repeated and the originating resources remained read-only.

The core chart patch is byte-identical to the preceding yellow-special proof.
The only private patch delta changes plugin lifecycle status in o-charts_pi.cpp;
no exercised S52 method changed. Every field of the extracted render-method
receipt equals the preceding `scrum264-yellow-special` receipt. The seven earlier
mutation controls therefore remain applicable and were not repeated. The retained
positive-only wrapper executes the frozen collector unchanged through its positive
fixture; it omits only its final mutation loop. No assertions were weakened.

This executes actual pinned symbol parsing, PNG crop, lookup selection, light
conditional and render-dispatch methods with fixture objects and recorded painter
calls. Core/private shared loader and render bodies are compared. The new generic
resource is now covered by the same native PNG/reference comparisons as the other
tiles. This is not a real chart canvas, actual GL draw, private DLL lifecycle,
Windows or boat display test. No full app build, broad suite, CI or boat action was
performed. The focused fixture completed on its first run with no new failure.
