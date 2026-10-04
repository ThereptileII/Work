# bccdbb11 native visual evidence: scoped review

SCRUM-15/279/283, with existing SCRUM-263/264/265 typography, classified-symbol
and structural-paint requirements. Read-only review of original retained native
Windows evidence; no new test, build, render, chart download or application run.

Frozen local `1a6733a1cbcc817aa0f13fa5acc41aac62a54146` maps to published
`bccdbb11cef827d3b63731fe1e874bc72bf47d2d`, run `37177738716`.
Root supplied artifact `11295783837` identity. This audit independently checked
its original 77,133,138-byte ZIP, SHA256
`aac540c4cb27c3340b2284ea04708184be4ba5cf34106bbe344d8dc4802adb5a`,
all 19,697 entries' CRCs, unique safe paths and absence of symlink entries.
[audit.json](audit.json) maps every retained original member to its archive path,
size and digest. The original ZIP remains in the main worktree's private local
archive, not duplicated here. Five PNGs and three JSON reports/manifests are
retained byte-for-byte. The log excerpt identifies its original line ranges.

## What was actually seen

| Request / evidence | Demonstrated | Remaining boundary |
|---|---|---|
| [Real ENC, SKAGER Day](chart-software-01-loaded.png) versus [Legacy](chart-software-05-legacy.png) | Cream land, pale blue water, muted gray/green shore and geographic lettering replace the broad mustard/brown Legacy appearance. SKAGER header and chart controls are visible. | Different UI/viewport extents and a later Legacy route/dashboard fixture mean this is not a pixel-matched difference test. Harbor symbols/buildings are tiny and densely overprinted; individual feature attribution and readability are not established. |
| Supplied buoy/light artwork | Inspected the unchanged prototype Windows navigation reference, `chart-marker-art.js` and `seamarks.json`, and the original private Day atlas. Its added bottom strip visibly contains slender classified buoy bodies/topmarks, light circles/rays, anchor, marina and hazard motifs consistent with the supplied artwork. | An atlas proves available paint, not actual ENC selector coverage, recognition or private chart rendering. The Seattle view does not isolate every buoy color/shape, red/green ordinary light, marina selector, fishing polygon or associated-depth hazard case. |
| [Name painter](chart-names-production.png) and [light labels](chart-lights-production.png) | Original native component images show Day/Dusk/Night geographic lettering, water-name spacing/italic treatment and haloed light text. Accents and Arabic fallback are visible. | These are explicitly offline painter fixtures, not native chart-object captures. The name fixture deliberately supplies Arial (`tests/chart_name_text_test.cpp`); it is not proof that the full application uses Arial. Light-label text does not prove light symbol/sector geometry. |
| Native production typeface | Original log records Segoe UI Variable Display unavailable, Segoe UI available; production 11px/400, 23px/650, 48px/400 and ordinary-chart policy resolve to Segoe UI, confirmed through the GDI face. | This supports the installed prototype fallback policy on this runner. It does not qualify the boat's installed fonts, every label role or every DPI. |
| [Smaller logo](skager-wordmark-production.png) | Native before/after component sheet shows 148 and 124 DIP at 100/125/150% across three themes. The full ENC screenshot visibly uses the compact header logo. Frozen `SkagerWordmark::HeaderWidthDip` is 124 and `Shell` uses it. | Approved SKAGER branding replaces the reference's historical OpenNav logo; it is not an exact original-logo claim. Boat legibility/placement remains open. |

Also directly viewed original `chart-software-02-zoom.png` and
`chart-opengl-01-loaded.png`; their hashes are recorded without duplicating them.
The latter's filename describes the request, **not the actual renderer**.

## Source/resource and renderer checks

- [charts-results.json](charts-results.json) reports native Windows, NOAA
  `US5SEAFL.zip` SHA256
  `b027e029dc7b76595381d89e3718145eb5069e5a03917e78bada4f8ebe8c84e4`,
  a disposable rendering fixture. Both requested phases have
  `runtime.chart.opengl_enabled=false`, including adjacent/returned cell states.
  The OpenGL phase explicitly records host rejection and software fallback;
  real GPU/OpenGL remains unqualified. This is real ENC geography with controlled
  input/route fixtures, not a live boat session.
- Both phases report core SKAGER presentation available, effective point style
  76 versus saved 82, and `private_ocharts.available=false` / no SKAGER adapter
  loaded. None of these screenshots is a live o-charts view.
- All five payloads in the original core, production and private resource sets
  validate against the identical [resource manifest](chart-resource-manifest.json),
  SHA256 `cecfd92eff1b2c9a9c2aa2e64e9a16968a84bdcdee14b77eba337a9277c07185`.
  This includes XML, RLE and all three atlases. All 46 input hashes/sizes in the
  [private package manifest](ocharts-package-manifest.json) match frozen source
  bytes or their exact native CRLF form. This is source/resource identity, not
  successful private runtime or dependency/helper qualification.
- Neutral structural paint is selective: manifest `XNSTR` owns 14 lookup IDs,
  `XNSHR` owns six outline IDs, with widths/selectors/labels unchanged. Day
  structural fill `[238,238,226]` and shore `[175,191,174]` agree with supplied
  prototype land/shore. Global `CHBRN`, hazard meanings and unowned symbols are
  retained. Brown seen in a stock atlas or Legacy view is therefore not evidence
  that the structural correction was omitted. Low-contrast structure recognition
  still requires feature-specific native/boat review.
- Frozen `ChartCaAllRound.h` retains the narrow ordinary-light full-circle
  extension, with original radius/center and unsupported cases falling back.
  The supplied light artwork is a point/rays glyph plus sector treatment, not a
  physical tower drawing or a supplied ordinary full-circle asset. This capture
  set does not establish complete all-round/classified-light acceptance.
- Original DPI report measures 96/120/144 DPI at 100/125/150%; source hash and
  compact observations are in `audit.json`. Those measurements do not make this
  five-image selection an exhaustive DPI visual review.

## Acceptance remains open

This evidence advances native software visual review only. It does not close
SCRUM-15/279/283 or the related typography/symbol/structure acceptance gates.
Unmapped physical tower variants, white/orange Dusk fallback, unsupported light
cases, and the private associated-depth line/area limitation remain as documented
in the [integration review](../../design/reviews/scrum279-283-symbol-integration.md).
No new defect is inferred merely from the stock fallback symbols or atlas.

The same candidate's production trust gate **failed**:
`downloader-valid exceeded the 30-second native probe limit`, after successful
LocalMachine trust import and server startup. The original terminal failure is
preserved in [production-observations.txt](production-observations.txt).
This candidate is not a passed production/package/boat release. Actual private
charts, helper/dependency closure, physical GPU, boat fonts/display recognition,
and package/runtime qualification remain separate required evidence.
