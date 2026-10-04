# Prototype conformance — in progress

## Current qualification and review boundaries

The active build is identified in [status.md](../status.md) and its exact
publication record. The test-only `0da2c643` replacement follows the
[55ef51e native results](../evidence/windows-55ef51e/README.md): actual chart-module
binding, application tests, TLS, AIS, recovery, DPI and public ENC checks passed,
but the installer harness stopped at the earlier correct missing-import
rejection. No eligible package or completed boat acceptance follows.
The [original native visual review](reviews/native55-chart-font-logo.json)
records image hashes and scope: neutral structural palette, smaller wordmark,
Day/Dusk/Night text roles, and actual production recovery with no Demo controls.
Hosted Windows lacks Segoe UI Variable Display and measures Segoe UI fallback
at 96 DPI; no 144-DPI actual font-face proof or boat-font acceptance is inferred.
Requested hosted OpenGL fell back to software. Native private-chart, close
symbol-family and boat GPU/display acceptance remain pending.
The candidate also retains the 124-DIP
SKAGER wordmark, prototype font stack, classified buoy/light artwork, neutral
structural paint and compact ordinary sector fans. See the
[historical supplied-artwork audit](reviews/scrum264265-current-artwork-coverage.md)
with the [later integration review](reviews/scrum279-283-symbol-integration.md):
its earlier unimplemented rock/wreck/marina/fishing/cable entries have been
superseded, while unmapped variants remain. Catalogue availability does not mean
every stock glyph has a redesigned counterpart.

[Actual combined326 Linux canvas evidence](../evidence/scrum275276-326-linux-canvas/README.md)
qualifies 24 software/Mesa captures and retains the separate failed elevation-label
Day return. The [corrected 5bb focused actual GL gate](../evidence/scrum268-5bb-linux-canvas/README.md)
then passes 12 images and three exact Day returns. That small chart-text cache fix
is now included in the replacement Windows source.
Root reviewed full Day/Night/Standard images and independently verified 116
retained files. No Windows/boat or complete-screen row advances from Linux proof.
Those retained images still show the original all-round circle and stock overzoom
warning, alongside classified fallback symbols. Their later paint corrections
below do not retroactively change these historical captures or accept the screen.

The remaining generic brown building point now uses the bounded neutral
`XNBLDG01` alias. Its [actual 27e software/Mesa review](../evidence/scrum265-27e-linux-canvas/README.md)
passes eight captures and both exact Day returns. All observed changed chart
pixels lie within the two affected building tiles; original symbol geometry,
classification and conspicuous/Paper variants remain unchanged. The Night
generic-versus-conspicuous contrast still needs native/boat recognition review.

The prototype does not supply a replacement physical lighthouse tower or
all-round range circle; [the historical source boundary](reviews/scrum264-all-round-light-boundary.md)
records that absence. It does not mean the later ordinary-circle paint extension
below is still unimplemented. The supplied rock/wreck/marina/fishing paths
and cable waveform were integrated at `45d73e8` and are included in current
`bccdbb11`, with [source-locked combined resource proof](../evidence/scrum279-283-combined/README.md)
and [optimized core/private object review](../evidence/scrum281282-combined-review/README.md).
They were absent from the former `4ddf1f3` candidate. The
[actual 45d Linux canvas review](../evidence/scrum279282-45d-linux-canvas/README.md)
now retains 40 captures, ten exact Day returns and ten clean sessions. It proves
UWTROC03 selection and source-backed cable coverage, while sampled awash rock
and wreck correctly select ISODGR51. UWTROC04/WRECKS05 and the absent exact
marina/fishing-area selections are not thereby qualified. This supersedes the
older audit's unimplemented status for these exact glyphs, not their conditional,
recognition or visibility limits. Private hazard-query repair retains separate
native/private-chart and boat gates.

The integrated [SCRUM-284 all-round paint](../evidence/scrum284-all-round-light/README.md)
is now a thin, outline-only extension of the prototype sector-arc roles. It
preserves original circle geometry and range-band scaling, with unsupported
states stock; no exact supplied all-round or physical-tower asset is claimed.
The [SCRUM-286 overzoom treatment](../evidence/scrum286-overzoom-warning/README.md)
uses the existing 12px/550 warning-callout typography without changing the
upstream trigger or translated warning. It too is an explicit design extension,
with stock fallback. Actual-method software/Mesa checks and optimized compilation
cover these increments; SCRUM-286 also retains two exact fixture Day returns.
The [actual fc6348a light comparison](../evidence/scrum284-fc6348a-linux-canvas/README.md)
passes 16 captures and four exact Day returns, with all eight Standard chart
controls unchanged. The [98d2c45 warning application review](../evidence/scrum286-98d2c45-linux-canvas/README.md)
also passes 16 captures and four exact Day returns; only the warning region
changes and all Standard chart controls stay identical. Native Windows/DPI and
boat readability remain separate gates. The approved
124-DIP wordmark and prototype font stack remain included, without implying
accepted font selection on every target. No complete glyph, Windows or boat
screen row is marked PASS from these bounded results.

## Native review inputs for the current candidate

This is the existing SCRUM-15/263/264/265 review, not new implementation scope.
Use the current candidate's original `evidence/local/` members and report identity
when they are available; the filenames below are existing collector outputs,
not a claim that the in-progress run has produced or passed them.

| User request | Exact existing outputs to inspect | Required interpretation |
| --- | --- | --- |
| Smaller logo and typeface, main shell | `preview-01-navigation-day.png`, `preview-09-returned-xnav.png`, `preview-results.json`; `dpi-100-01-navigation-day.png`, `dpi-100-navigation-dusk.png`, `dpi-100-02-navigation-night.png`, `dpi-125-01-navigation-day.png`, `dpi-150-02-navigation-night.png`, `dpi-100-1920-navigation-day.png`, `dpi-results.json` | Check the approved 124-DIP SKAGER/APP artwork, clipping, hierarchy and actual DPI. Preview/DPI basemaps do not qualify ENC glyphs. The 1920 capture is Day only. |
| Native face and component paint | `skager-wordmark-production.png`, `chart-names-production.png`, `chart-lights-production.png`, `windows-production-Win32.log` (the same collectors also emit `*-xnav.png` and `windows-xnav-Win32.log`) | Wordmark/name/light PNGs are component drawings. Read the actual selected/GDI font and DPI in the log; a correct font request alone is insufficient, and these images do not replace application review. |
| Symbols and brown structural paint in real ENC | `chart-software-01-loaded.png`, `chart-software-02-zoom.png`, `chart-software-04-restored.png`, `chart-software-05-legacy.png`, `chart-software-06-returned.png`; corresponding `chart-opengl-*` files and `charts-results.json` | Bind chart identity and effective presentation to the report. Verify actual `opengl_enabled`; a filename is not GL proof. Seattle views do not resolve every guarded light, structure or supplied glyph, and do not provide the missing private/IHO all-round scene coverage. |

The [15452e5 retained receipt](../evidence/scrum263-264-native-154/receipt.json)
already binds nine original PNGs, including the six DPI views listed above,
loaded/zoomed software ENC and the production wordmark drawing. Its
[font excerpt](../evidence/scrum263-264-native-154/recorded-font-and-wordmark.txt)
records Segoe UI fallback at 96 DPI. These are historical references, not current
candidate pixels. The [17ab044 review](../evidence/windows-17ab044/README.md)
retains the two preview views above, but no completed native ENC/private symbol
qualification. Neither record closes physical boat readability or same-machine
HTML/native typography comparison. Unsupported semantic light variants,
unmapped towers and the white/orange Dusk fallback remain explicitly open.

## Retained earlier qualification history

The former frozen candidate was local `0a52a6c` / published `17ab044`
([CI](https://github.com/ThereptileII/Work/actions/runs/37164360050)). Its
[complete source mapping](../evidence/skager-symbols-0a52-publication.json)
independently verifies all 6,658 mapped entries. The retained
[Linux audit](../evidence/linux-17ab044/README.md) and
[native Windows audit](../evidence/windows-17ab044/README.md) preserve their actual
results and limitations. Windows cancellation at the subsequently reproduced
CurrentUser trust prompt did not invalidate the earlier partial passes or make
that candidate an eligible package. Historical images remain unchanged.

The former frozen Windows candidate was local `48c2f8b` / published `4ddf1f3`
([CI](https://github.com/ThereptileII/Work/actions/runs/37155858878));
[its complete source mapping](../evidence/skager-parent-context-publication.json)
is verified. Application, interaction, DPI and public ENC gates passed, but
native qualification failed at the AIS wrapper's generated-project traversal
before AIS compilation/runtime. SCRUM-285 owns its focused correction. The
parent-context proof and exact same-job reprobes passed. The preceding `63c1029`
failed before AIS runtime because its verification caller omitted the original
tool PATH prefixes; this was not an observed application crash.

The preceding frozen candidate was local `96c0c27` / published `15452e5`
([CI](https://github.com/ThereptileII/Work/actions/runs/37136712793)); its
[publication receipt](../evidence/skager-combined-96c0-publication.json) binds the
exact mapped source. The 2026-10-03 18:07 UTC native job observation confirms
integrated modes, chart gestures, repeated recovery and the complete fixture UI
suite passed. The production application and 139 tests passed, then expired-certificate
fixture creation failed before TLS assertions. The [original transcript](../evidence/scrum272-native-154-expired-fixture/README.md)
is retained; SCRUM-277 owns the bounded setup repair. Package and boat
qualification remain open.
It includes the smaller logo, prototype fonts, classified symbols and neutral
structural paint below, plus the bounded notification-bell refinement. The
preceding `442960ba` run failed private trust-probe configuration after the
product build; the narrow path/cache repair passed its separate native gate
before this replacement was dispatched. The corrected
[preview timing predicate](../evidence/scrum270-preview-validity/README.md)
preserves intentional unavailable route/arrival data during waypoint transitions.
The earlier `9d98a500` native run passed application compilation, 139 tests, the fresh
five-export private DLL/resource audit, DPI/touch and public ENC checks. Its
[renderer receipt](../evidence/scrum270-native-9d98-preview-failure/renderer-proof.json)
shows that the requested OpenGL phase used software fallback; native hardware
OpenGL remains unqualified. Its
[retained failed preview](../evidence/scrum270-native-9d98-preview-failure/README.md)
blocked product packaging. Actual GDI probes selected the prototype's Segoe UI
fallback for UI and ordinary chart text; the hosted runner lacks Segoe UI Variable
Display. Geographic italic canvas-face observation, private renderer runtime and
physical boat review remain open. The Arial geographic painter fixture is not
evidence of the production selected face. No screen-level row advances.

The candidate includes
the supplied generic beacon, classified yellow buoy and fitted topmark, the
124-DIP approved SKAGER logo, prototype font choices, and neutral structural
paint. The [current publication receipt](../evidence/skager-combined-96c0-publication.json)
verifies the entire mapped source tree. A separately reproduced and repaired
Windows resource-generation mismatch passed its targeted native gate before
the preceding build was started. Product runtime, actual private-renderer loading and
physical boat review remain pending; no screen-level row advances.

The [actual yellow-pair canvas review](../evidence/scrum264-yellow-e1d-linux/README.md)
passes SKAGER software and OpenGL Day/Dusk/Night/Day-return, including the
canonical empty-instruction repair caught by the preceding failed capture.
The [generic-beacon comparison](../evidence/scrum264-generic-beacon/README.md)
proves exact supplied artwork at the resource boundary, not an actual private
chart. Core and private symbol-table observations are now separate copied
diagnostics; the private observation still needs native runtime evidence.
SCRUM-268 retains the inherited Standard OpenGL light-label shift and its
unchanged-source control. It is neither hidden by a tolerance nor claimed fixed.

The preceding full candidate was local `d5d7135` / published `b8cfbf8`
([CI](https://github.com/ThereptileII/Work/actions/runs/37114216075)). It adds the
final explicit Segoe UI Land-face choice while Water retains its inherited
stack. The preceding native 70-private/23-core checks pass with exact audited
inputs. No screen-level row advances until actual native and boat review.

Final source `9dee9b1` / published `aa95750b` closes the ordinary TX/TE font
policy gap with a verified-SKAGER-only Segoe UI/Arial face. It preserves existing
sizes, weights and designated geographic/LIGHTS handlers, their fallbacks, and
Standard/Legacy preferences. [Focused source evidence](../evidence/scrum263-ordinary-chart-face/README.md)
passes; final integrated native face and boat rendering remain pending.

Current local `ccc0faa` / published `1c3e32d6` adds a narrowly classified
[white/orange pillar derivative](../evidence/scrum264-white-orange-pillar/README.md).
Its focused resource/loader/object checks and [16 integrated Linux captures](../evidence/skager-chart-ccc0faa-linux/README.md)
pass; Dusk deliberately retains the original symbol after the brighter
orange trial lost color recognition. This is an explicit remaining difference.
The [focused native private-object run](https://github.com/ThereptileII/Work/actions/runs/37112484119)
passes the CMake path repair and all 70 actual private objects, with an
[independently audited artifact](../evidence/scrum259-native70-ccc0/README.md). Neither
compilation nor individual symbol proof advances a screen-level acceptance row.

Frozen local `5c05eb5` / published `159cbeff` has a fresh integrated Linux build,
147 passing regressions and 24 original real-chart images covering the corrected
[short red/green light aliases](../evidence/scrum264-colored-lights-5c05-linux/README.md)
and [neutral construction hatch](../evidence/skager-chart-5c05eb5-linux/README.md).
Software/Mesa OpenGL, Standard comparisons and entire Day-return checks pass.
Original light vectors remain stock when an ORIENT attribute exists; no encoded
direction is discarded. The [native 23-unit compile gate](https://github.com/ThereptileII/Work/actions/runs/37109776549)
passes at the exact mapped revision, but is not application or visual acceptance.
The unchanged font/header inputs retain the qualification boundaries below.

The [remaining family inventory](../evidence/scrum264-remaining-symbol-boundaries/README.md)
distinguishes supplied custom SVGs from stock catalogue entries. No direct custom
physical-tower or classified fixed-beacon artwork exists in the prototype. The
white/orange special-purpose derivative is included in ccc0faa with explicit
class guards; it is not part of the earlier 5c05 images. Long-range/sector
lights and other unmapped families remain open, subject to navigation meaning.
Windows/private-renderer/physical boat acceptance remains pending.

The preceding local `9632421` / published `d2787c6` has an integrated Linux build,
147 passing regressions and [32 real NOAA ENC captures](../evidence/skager-chart-9632421-linux/README.md).
All whole-chart Day returns are exact in software and Mesa OpenGL, closing the
retained a3e selector reproduction. All sixteen Standard historical comparisons
remain unchanged within the existing toolbar exception. Actual diagnostics
prove that the effective SKAGER Simplified table does not overwrite the saved
Paper preference. Every captured header matches the approved 124-DIP component.
An additional [16 IHO S-64 captures](../evidence/scrum264-s64-9632421-linux/README.md)
prove actual classified symbol/raster selection with twelve clean exits. Ordinary
red/green short flares (corrected in the later source above), classified long-range
lights and central physical lighthouse artwork visibly remain stock in those
historical images; no all-light matching is claimed.
These are bounded Linux results, not Windows, private o-charts or boat acceptance.

The combined batch includes eleven classified prototype marine/light assets,
fourteen structural land fills, six shoreline outlines, and the smaller logo.
The UI already requests the prototype's exact ordered font stack. Actual native
face selection remains a Windows gate; a read-only boat inventory confirms all
three named families exist but does not prove which face the application uses.
Unmapped special-purpose buoys and fixed beacons remain explicit gaps. Hazard
patterns and classified long-range/sector lights retain their navigational meaning.
No screen-level acceptance row is advanced by partial symbol or resource proof.

The preceding full native run failed producer-manifest resolution before any
application compilation. Its producer-root/header-closure repair passed the
[exact native prerequisite proof](../evidence/scrum259-native-prefix-proof/README.md).
The [complete replacement run](https://github.com/ThereptileII/Work/actions/runs/37106815245)
passes maintained dependency producers but stops on a private-renderer CMake
Windows-path escape before private DLL or application compilation. The exact
[failure receipt](../evidence/scrum259-private-configure-d2787/README.md) remains
retained; a focused configure/object repair gate is in progress. Windows
screenshots, actual private renderer and physical boat comparisons remain required.

The latest correction batch adds exact ferry/cable-area paint (SCRUM-260),
40,482 role-proven Day neutral pixels (SCRUM-261), and the native chart-toolbar
border/divider plus full Instruments caption (SCRUM-14). Their focused source,
pixel and resource evidence is linked from status.md. All screen-level rows
remain Pending until the combined application and physical display are reviewed.
Remaining off-palette wreck pixels and other stock chromatic roles are explicitly
not represented as matched by these bounded corrections.

The replacement application is local `dfa7b72`, published `c9ff4ae2`. Its
integrated Linux build, five drawing fixtures and 147 regressions pass; all 23
changed native compilation units pass separately. The full Windows run
37097634494 stopped before application compilation on a deterministic line-ending
preparation check. Its bounded correction now passes the separate native
14/16/38-case gate in run 37098440931 at published `5d6c5cf4`
([downloaded evidence](../evidence/scrum259-native-final/README.md)).
Neither result qualifies a Windows application rendering or icon. The prior
78eccb8 chart images below remain their own unchanged evidence; they do not
exercise the new private o-charts renderer required by the boat's licensed
vector collection. Actual native and boat chart comparisons remain required.

The preceding chart-capture source is `78eccb8` (published `61a0a783`). It includes exact
classified cardinal artwork and the actual-active-waypoint name card, together
with the prior land/Night/service/branding corrections. Its integrated Linux
build and [native seventeen-unit preflight](../evidence/scrum-247-native-chart17-final/README.md)
pass. The [sixteen real-ENC comparisons](../evidence/skager-product-fidelity-78eccb8-linux/README.md)
and [software/GL route/card comparisons](reviews/scrum252-257-final-78eccb8.md)
also pass their bounded gates. That full native run subsequently failed the translucent geographic-name
painter; the separate correction and replacement are recorded in status.md. No screen-level acceptance row is changed by
compilation or a subset of chart states. The public NOAA comparison cell contains no cardinal objects, so those
images cannot establish actual ENC cardinal recognition.

Exact `f0976cc` now has [sixteen real-ENC comparisons](../evidence/skager-product-fidelity-f0976cc-linux/README.md)
with corrected land, effective Night surfaces and matte-free approved SKAGER
artwork. Software and Mesa-GL theme cycles pass; Standard remains pixel-identical
to the retained baseline. The [route comparison](reviews/scrum252-integrated-f0976cc.md)
initially fails the software name-card/circle/understroke check while the GL
route scenario passes. [Direct fixture mutations omitted normal repaint
notifications](reviews/scrum252-fixture-repaint.md); an ordinary repaint shows
the correct software objects. The narrow fixture correction retains every
navigation assertion and still needs a fresh integrated capture. It does not
establish chart conformance acceptance.
The observed low-contrast active-waypoint name now has a bounded
[prototype-card correction](reviews/scrum257-active-name.md), preserving the
upstream active-point symbol and blinking; fresh rendering proof remains required.
Native Windows runtime and boat review remain open.

Frozen source `1356fd1` (published `9fcd3db`) now has [sixteen corrected real-ENC
captures and independent review](reviews/skager-chart-1356fd1-linux.md), covering
software/Mesa-GL through Day → Dusk → Night → Day. The repeated-edge
defect is absent under the strict negative-control-derived probe, with four
clean application exits. Two actual upstream route scenarios pass their 26
checks and exact route-ink probes. This closes the observed Linux reproduction
of the framebuffer defect; it does not close native Windows or boat gates.
Dense harbor labels, small light-text readability and special-state route symbols
remain visual review items. The logo's Night background has a focused correction
in `8accdd9`, followed by waypoint labels, effective Night chart colors and the
GTK recapture repair. Their combined native/boat rendering is not qualified.
All screen-level rows below remain Pending.

The October3 integrated chart review now includes exact-source software
captures after the LIGHTS and sounding-font changes at
[`19c900a`](../evidence/skager-chart-19c900a-linux/receipt.json).
Standard chart pixels remain identical to the prior e14de49 control in all
three themes. SKAGER labels follow the prototype's smaller hierarchy, but 8px
light descriptions remain difficult to read in this Linux capture, especially
Night, and dense harbor overlaps persist. Native Windows/font/DPI and physical
readability remain open; this is not an accepted conformance exception.
SCRUM-250 separately repairs the confirmed GL theme-time framebuffer-size defect;
the software captures cannot qualify that correction.

Authority: [immutable v8 prototype](prototype/index.html), hash in
[manifest](prototype-original.json). The Beta `79a95c4` deployed when this design
lock was introduced predates it and was **not visually conformant**. Current
deployment identity belongs to the exact-revision evidence in
[project status](../status.md); the historical statement is not a fresh boat
inventory. No screen is accepted by this record. Functional Beta evidence
remains valid only for its original scope.

| Screen family | Layout | Type | Color | Spacing | Components | Interaction | Chart | Boat 1280×800 |
|---|---|---|---|---|---|---|---|---|
| Navigation Day/Dusk/Night | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Passage / waypoint | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| AIS list / target / details | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Instruments | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |
| Propulsion / energy | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |
| Autopilot | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Anchor | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Alerts / health | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Settings / display / sensors | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Diagnostics / system | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Radar unavailable | Pending | Pending | Pending | Pending | Pending | Pending | N/A | Pending |

Each PASS requires linked exact-commit reference/current/diff, measured geometry,
resolved fonts, interaction results, a corrective second capture and a boat
review. No averaging across views or broad image tolerance. Physical rendering,
touch, ENC, OpenGL/software and 100/125/150% remain separate gates.

The October 2 follow-up restores the supplied rail metric icons and the Display
theme selector's missing enclosing track. Focused component reviews retain
their limited scope: [rail icons](reviews/scrum14-rail-metric-icons.md) and
[Display track](reviews/scrum-216-theme-track-local.md). These changes are being
developed separately from frozen candidate `c95d3a0`; its running qualification
cannot qualify the newer UI. The same isolated batch adds saved-route/waypoint
Search and the observed Chart presentation drawer, with floating Layers and
Settings entrypoints. Managed/unsupported prototype rows remain truthful and
non-interactive; the [chart control contract](chart-presentation-controls.md)
records navigation-correctness adaptations. All screen-level rows above remain
Pending.

SCRUM-100's [horizon correction](reviews/scrum-100-horizon.md) now has exact
Day/Dusk/Night component reference/current/diff evidence, owned-data freshness
and action guards, and real pointer/keyboard checks. Local geometry passes;
Linux text rasterization differs visibly. Native Windows and physical boat
acceptance remain pending, so the Navigation row above remains Pending.

The boat was unreachable at the September 28 stage transition. It reconnected
and its interrupted read-only commissioning transaction was subsequently
[closed and independently verified](../evidence/boat-prototype-reconnect-20260928.md).
The earlier Beta remains installed and closed. New deployment/capture requires
a fresh read-only audit against the adopted baseline. Do not infer physical
acceptance from Linux reference generation or the previous Windows suite.


The user-authorized final identity is SKAGER. The original prototype remains
unchanged design evidence; current customer-facing branding uses the approved
SCRUM-89 wordmark and Windows icon derivative. This explicit identity change
supersedes the prototype's development wordmark. The combined SCRUM-235–238
batch has focused development checks but still requires exact-source native
Windows and boat comparison; all screen-level acceptance rows remain Pending.
