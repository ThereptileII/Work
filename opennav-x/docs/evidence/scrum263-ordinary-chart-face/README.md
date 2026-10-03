# SCRUM-263 ordinary ENC chart face

Private base: `ccc0faa`. Jira selection: comment 10737.

The immutable final `.chart-label` and `.chart-symbol-label` rules explicitly
specify Segoe UI. The latter's later override changes size/tracking, not family
(`docs/design/prototype/index.html:12,104,118`). This differs from the UI's
inherited Segoe UI Variable Display stack. Previously ordinary TX/TE inherited
the saved/system ChartTexts face; a Windows mismatch was reachable but not yet
measured. This change enforces the explicit family without claiming all chart
text now matches prototype geometry or the boat's actual font substitution.

`ChartTextFace.h` selects installed Segoe UI, then Arial, else an empty policy
meaning preserve the original template. Enumeration is cached once per module;
it does not occur while drawing. Only the successfully verified core/private
library constructors set the copied instance face, before text caches exist.
Standard, Legacy, failed verification and unbound private libraries leave the
policy empty. Chart style switches already require an application restart;
there is no live face-policy mutation or borrowed-font invalidation here.

Both real RenderT_All paths retain their original ChartTexts getter and original
FindOrCreateFont call. For ordinary text only, the optional owned face is tried
with those exact unscaled size/family/style/S52-weight/underline inputs. A null
or invalid candidate retains the original font pointer. Existing FontMgr owns
the returned stable cached font and content scaling. No saved face, size, color
or profile byte is written. S52 body normalization, user size contribution,
minimum size, effective weight policy, TX/TE content, color, offsets and
visibility/declutter rules are unchanged. The original computed average width
is retained; natural string extents use the chosen face as usual.

The exact GeographicChartName and IsGeneratedLightDescription predicates
exclude designated geographic/generated-LIGHTS roles even when their resolver
returns null (custom LIGHTS preferences or failed specialized raster/font). A
non-null specialized result likewise remains authoritative. Existing tracking, opacity, sounding and special
raster handlers are unchanged. Ordinary labels retain their original software
and GL rendering paths and font-pointer cache keys. No global UI font changes
or font binaries are introduced.

Focused verification:

- All nine core and both private patches apply in independent source copies.
- The source-extracted actual core and private font-establishment blocks each
  pass 202 assertions each, plus four bypass mutations each: verified style,
  non-null specialized font, null geographic role and generated LIGHTS fallback.
  Cases cover ordered family availability, empty fallback, custom sizes/styles,
  all three S52 weights, valid/null/invalid owned fonts, cached-pointer reuse,
  preserved specialized fonts (including null custom/failed handler fallbacks),
  positive ordinary LIGHTS OBJNAM/ORIENT TE, and unchanged configured-font bytes. Fixture
  template/cache storage is supplied locally; this is not a full chart render.
- Actual core s52plib, core ChartPresentation, private s52plib, private
  ChartPresentationAdapter, and private integration-disabled s52plib compile.
- Sixteen private preparation checks pass with the new copied-header closure.
- The existing integrated ui_font_resolution_test gains an ordinary-family
  probe. On Windows it checks installed-family choice and actual selected-HDC
  GetTextFaceW with retained size/style/weight. It is already run by the existing
  optional drawing group; no new workflow is added. Linux lacks both candidate
  faces and correctly reports original-template fallback. Linux cannot qualify
  Windows substitution, full ENC label drawing, DPI or the boat display.

No full application build, broad suite, CI dispatch, remote action or shared
builder/cache write occurred. The component fixture links existing frozen
ccc0faa UI libraries read-only and writes its own objects/executable privately.
Compact exact-source receipts follow separately; Windows/boat acceptance remains
open rather than inferred from this Linux proof.
