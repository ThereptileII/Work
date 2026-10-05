# Retained native Windows visual review — 15452e5

This read-only SCRUM-15/263/264 review uses original artifact **11281990645** from
[run 37136712793](https://github.com/ThereptileII/Work/actions/runs/37136712793),
remote `15452e512fd073090b1a4cea7c9010eac0874118`, exact local
`96c0c2705aacae511b6f8c22afaa6618dbabceff`. The candidate **failed** production
certificate-fixture setup. Its screenshots remain diagnostic evidence, not an
eligible package or release/boat acceptance. No application was launched, built
or changed for this review.

The [receipt](../../evidence/scrum263-264-native-154/receipt.json) binds the original
65,648,702-byte ZIP (SHA256 `1fa9660cbf99b68893cd96b49c6fee22d92dd9f4a4d0d39f3660f970cba43f3b`),
all 18,441 CRC entries, nine verbatim PNGs, original reports/log, and immutable
prototype references. The complete ZIP remains in the private
`scrum273-full-154-watch/.local/windows-integration-154.zip` evidence directory.

## Findings

| Review point | Actual observation and boundary |
| --- | --- |
| Approved compact logo | The two-line SKAGER/APP artwork is visible without clipping in the retained 1280×800 Day/Dusk/Night, 125% Day, 150% Night and 1920×1080 Day images. `src/ui/SkagerWordmark.h:20` fixes the header width at **124 DIP**; `Shell.cpp:112–129` draws the approved embedded PNG. The component matrix separately records 148-before/124-after drawings at three themes/scales. It is not a main-app screenshot. The prototype's historical OpenNav wordmark is superseded by approved SKAGER artwork, not a branding target to restore. |
| Font selection | [Recorded HDC output](../../evidence/scrum263-264-native-154/recorded-font-and-wordmark.txt), original production log lines 3778–3785, says **Segoe UI Variable Display unavailable**, **Segoe UI available and selected**, and `GDI-face=Segoe UI` for 11/23/48-pixel UI probes and the ordinary-chart policy at **96 DPI**. This matches the prototype's allowed fallback stack. It does not establish every live chart label's face or HDC selection at 120/144 DPI. No new font-selection defect is established by these pixels. |
| Composition/DPI | Full “Instruments” remains visible. At 1920 Day the rail grows and chart gains room; the logo remains compact. At 125/150% on the 1280-pixel desktop the effective logical viewport is smaller, with compressed/ellipsized content; this is not evidence of an incorrect scale factor. The receipt records actual Windows/wx DPI pairs 96/96, 120/120 and 144/144. Only Day was retained at 1920, so no 1920 Dusk/Night claim is made. |
| Neutral structures | The loaded Seattle ENC has neutral beige land and subdued small shoreline structures, not a conspicuous broad brown structural fill. This scale and scene do **not** individually resolve all BUISGL/FLODOC/MORFAC/PONTON variants or ruined-structure hatching. Main DPI images are coastline basemaps, not structural-ENC proof. SCRUM-265 and later class-specific work remain subject to their exact-scene gates. |
| Chart tools | Native Night has a fine, subdued outline, also present in the canonical Night reference. Effective HTML `.floating` explicitly specifies `#6b8b801c`; `FloatingSurface.cpp:83–97` paints that border and `#68837730` separator. Thus a visible outline alone is **not** a newly confirmed mismatch or evidence of the earlier bright GTK frame. |

## Concrete remaining differences

1. **Persistent stock chart-selector band:** the teal strip around screen
   `(91,606)–(683,624)` in both actual ENC images is absent from the prototype's
   primary composition. SCRUM-246 previously changed its palette; contextual
   presentation remains a separate known layout gap. Hiding real selection,
   quilt or unavailable-chart state without a working replacement is not a fix.
2. **Remaining chart-art differences and incomplete symbol coverage:** small,
   crowded Seattle labels/marks and guarded stock light circles differ from
   the deliberately sparse illustrative prototype. The zoomed magenta circle
   is the previously source-audited dumping-ground boundary, not evidence that
   all magenta should be recolored; the bounded SCRUM-262 paint proposal was
   withheld after shallow-fill visibility review. Information/hazard marks,
   real soundings/units and sector/orientation cues must retain meaning. These
   screenshots do not resolve each lateral/cardinal/yellow-topmark/light class,
   nor justify declaring SCRUM-264 fully conformant. Actual IHO and private
   chart/boat comparisons remain distinct evidence requirements.

## Correction: upstream notification control

The initial version of this review incorrectly grouped the separate chart bell
with unresolved stock-style defects. Follow-up against root `1f349a2`, the
[SCRUM-271 contract](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-271)
and all six material comments (10777, 10781, 10786, 10788, 10792, 10795) corrects
that conclusion. SCRUM-275 concerns chart lights and does not own this control.

The original `9d98` image has a filled cyan bell in an approximately 37-pixel
grey outlined square. The retained `154` image instead shows the already
integrated **44-DIP dark rounded surface and cyan outline prototype bell** at
roughly `(946,101)–(990,145)`. Its source `NotificationButtonBitmap.h` is
byte-identical between frozen local `96c0c270` and reviewed root `1f349a2`
(SHA256 `b1e62496a93d658efefb94d1da6198f257bd1466199f7a883d7084b74c029c03`).
The [271 design record](scrum271-notification-style.md) documents a deliberate
notification-specific adaptation: prototype alert target 44 DIP/radius 9,
centered 22-pixel bell path, theme background, and explicit cyan/amber/red
severity ink. It does not copy the illustrative prototype badge or invent a count.

The applied notification patch changes bitmap creation, DPI invalidation and
successful-style alpha handling. Pinned `ChartCanvas` still obtains upstream
count/maximum severity, shows/hides the control accordingly, routes its hit to
`NotificationsList`, and preserves acknowledgement by GUID. The shell entry's
presence does **not** establish duplicate delivery of the same notification.
The contract explicitly prohibits simply hiding this independent upstream
notification access or replacing it with an unrelated shell alarm.

Thus this Day software image supports the intended styled appearance; it does
not establish a new unresolved SCRUM-271 style defect. Native all-theme and
severity behavior, acknowledgement, DPI hit bounds, actual GL and boat review
remain open. This correction changes interpretation only: original images,
archive identity and the earlier pre-repair `9d98` review remain unchanged.
No application change, build, test execution or Jira write was made.

The canonical 1280×800 [Day](../prototype/reference/windows/navigation-day.png)
and [Night](../prototype/reference/windows/navigation-night.png) references
were inspected, along with immutable HTML and earlier native review. Real
Seattle geography, missing route/input state, chart units and hazard density
are not replaced with the prototype's fictional content. The 125% image's
“Change chart orientation” tooltip is a captured hover state, not a permanent
part of the design.

Both requested renderer phases in `charts-results.json` actually report
`opengl_enabled=false`; the log says OpenGL is not usable. Core effective
Simplified 76 versus saved Paper 82 is recorded, while private presentation is
unavailable. Therefore this artifact supplies **native software** observations,
not native OpenGL or private o-charts acceptance. Screen-level conformance,
physical DPI readability and boat acceptance remain open.
