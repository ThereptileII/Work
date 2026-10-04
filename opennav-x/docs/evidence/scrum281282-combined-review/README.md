# Combined supplied-symbol source integration review

Reviewed product source `00ffa88eccb321d5230002f1a1621045abed85b1`, then
added the parent-requested SCRUM-283 third private patch from `af69c5e`.
The local follow-on commit differs from that root snapshot only in an unrelated
`docs/upstream-patches.md` append; application, patch, helper and recipe bytes match.
No source correction was needed. This is source composition and Linux object
compilation evidence, not a native Windows, DLL load, app canvas or boat result.

- All nine core patches apply in order to pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
  Independent temporary-index reconstruction equals the prepared complete tracked
  source tree `9f027c926ec61da84fa118af7b53a61d24dede4c`.
- All three private patches apply to the verified original cached plugin inputs
  at `c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`. Independent reconstruction matches
  every changed file. The cache contains 217 verified source-lock blobs; two
  untouched license files are absent, as explicitly recorded. This check does
  not claim a complete redistributable 219-file private source package.
- Core and private `draw_lc_poly` bodies are byte-identical to reviewed isolated
  SCRUM-281. Core and private `CreatePatternBufferSpec` bodies are byte-identical
  to isolated SCRUM-282, including its corrected private guard. Both headers and
  the shared CA uploader header are identical to those reviewed versions. Each
  painter include occurs once; the appended patch sections compose cleanly.
- Final `eSENCChart.cpp`, `s52cnsy.cpp` and `s52s57.h` are byte-identical to the
  SCRUM-283 agent's independently compiled files. The adapter-only callback is
  appended to the private chart context. No API17 file is modified; hashes of
  the existing read-only API17 headers used by compilation are retained.

Both complete combined `s52plib.cpp` translation units compiled successfully at
`-O3` using the existing audited Linux wx 3.2 / GL dependencies. The core object
was compiled once. The initial private check reused the SCRUM-281 recipe; review
then strengthened its command to read **all actual adapter target definitions**
from `cmake/ocharts-adapter/Targets.cmake`, including `__OCPN_USE_CURL__`, empty
`DECL_EXP`, and configured API version `1.17`. That complete unit passed, and was
recompiled once after the new SCRUM-283 context header. No warnings/errors were
emitted by these full-unit commands. They do not assert MSVC or `-Werror` coverage.

Final preprocessing uses those compiler arguments. It confirms core `OPENNAV_X`
and private `SKAGER_OCHARTS_ADAPTER` are mutually exclusive, with GL/GLSL enabled,
and both fishing/cable calls are actually present in each compiled branch.
The private callback header does not change this renderer object's emitted bytes:
final object SHA256 `8dd64ac84df916ba760004689a9ea044f51dc2d45f3402ba0d0cffaf567c9243`.
Core object SHA256 `685196611681ad30e3f6a720d9ffef42a6f4a6bfe80534c533a7f86ef4148778`.

Receipts include exact commands, source/helper/patch hashes, original cache
verification scope, independent method equality and object hashes. Core command
was reconstructed from the unchanged retained command-building script after the
private-only run replaced its command log; the original successful core compile
log and object are retained. Existing generated resource headers and platform
configuration from the read-only audited Linux builder supply compile inputs;
this does not qualify the new combined generated resources, which the parent
checks separately. Original 554-case painter and pixel fixtures were not rerun.
No full application/dependency build, renderer execution, CI or boat operation
was performed. Actual combined canvas, native Windows and boat gates remain open.
