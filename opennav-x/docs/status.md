# SKAGER status — 2026-10-03

## Current combined candidate: SKAGER chart fidelity and branding

The next correction batch includes the exact prototype
[ferry/cable-area ink](evidence/scrum260-area-ink/README.md),
[Day neutral marker ink](design/reviews/scrum261-day-neutral-ink.md), and
[floating chart-control border and complete navigation captions](evidence/scrum14-floating-caption/README.md).
These preserve navigational classifications, symbol geometry, soundings,
Standard resources and control hit areas. The Day mask changes only 40,482
unambiguously owned neutral pixels; off-palette wreck bitmaps remain unchanged.
The [final copy audit](design/reviews/scrum236-final-copy-audit.md) also closes
remaining operator/diagnostic product labels. Customer-facing identity is
SKAGER with the approved Jira artwork; immutable evidence, compatibility IDs
and required OpenCPN attribution remain. Focused checks pass, but fresh combined
application captures, Windows packaging and boat acceptance are still required.

### Preceding integrated candidate and corrected Windows prerequisite

The replacement source is `dfa7b721ef6eca3f084f77e95dd8e2adc20bde4b`,
published exactly as `c9ff4ae2c110d234807a326bc30c25084eb42a5d` (2,849 mapped
blobs/modes verified). Its [integrated Linux build/install](evidence/scrum259-linux-dfa7b72/README.md)
passes all five original drawing fixtures and **147/147** regressions. The
shared-model callback link correction and the initial isolated test-environment
failure are retained with their passing evidence. The
[corrected native preflight](evidence/scrum259-native23-corrected/README.md)
passes all **23 actual Win32 compilation units** with independently verified
downloaded objects. Its earlier missing-header failure is retained separately.

The [full replacement run 37097634494](https://github.com/ThereptileII/Work/actions/runs/37097634494)
stopped at its Windows private-loader prerequisite, before application
compilation: the preparation test expected deterministic LF output, but inherited
Windows Git settings produced CRLF. Thirteen of fourteen preparation cases
passed; the native loader did not execute, so its expected artifact was absent.
A bounded production preparation correction now passes its
[separate native gate](evidence/scrum259-native-final/README.md), run
37098440931 at exact published `5d6c5cf44cb72f8166b2ab67115bccd568336e35`:
14 preparation cases, 16 build-wiring refusals and 38 actual harmless-DLL
loader groups. The downloaded artifact and source identities were independently
verified. Git patch output now explicitly stays LF regardless of inherited
Windows settings. This does not qualify the private renderer DLL, application,
charts or final package; no application crash is inferred from the original run.
Other Linux/native jobs continue independently. The Windows real-module,
final-icon, installer, chart and boat gates remain open.

The [fresh read-only boat check](evidence/scrum17-readonly-20261003/README.md)
confirms unchanged validated stock OpenCPN, installed Beta 1, profile and recovery
state, with no navigation/helper processes. SSH and Tailscale remain healthy;
RustDesk's service is running, but interactive connectivity was not exercised.
No candidate was installed and no older generation was removed.

**Boat chart presentation gap (SCRUM-259):** the fresh read-only inventory found
581 `.oesu` files in the first configured chart root and two MBTiles files in the
second. Both roots are available; no chart contents, private paths or licence
material were exported, and the profile remained unchanged. The accepted
o-charts source constructs and renders with its own S-52 library from the stock
shared resource directory. Current core `CreateChartPresentation` does not
intercept that construction. Consequently the core NOAA captures below cannot
qualify SKAGER presentation on the boat's o-charts collection. The exact plugin
resource/API boundary has now been inspected under SCRUM-259. A separately named,
exact-source private renderer adapter and fail-closed host boundary are implemented;
the source/package recipe has passed focused preparation checks. Native ABI,
private downloader trust, complete application and boat rendering gates remain
open. The [actual Win32 loader boundary](evidence/scrum259-native-loader/README.md)
passes all 38 native groups at published `4609f31dba3e970802e6ba3fbe3438e129ab9aad`.
The next candidate also includes an explicit early real-host module check,
which blocks child-process creation and never invokes the plugin factory or
initialization. It has not yet passed natively. The installer gate now checks
the actual application's and Setup's PE icon frames and product metadata
against the approved Jira artwork; native final-binary results remain open.
No boat plugin or stock resources have been changed. Raster MBTiles
cannot receive object-level vector styling. [Availability evidence](evidence/boat-chart-types-20261003/summary.json).

The preceding chart-capture application source is `78eccb8b7f21b260ded57d3ba763f884d60c8180`,
published exactly as `61a0a7838b56ad841bb458af6fc62651464bdafe` (2,472 mapped
blobs/modes verified). Its [integrated Linux build and staging pass](evidence/skager-chart-78eccb8-linux-build/README.md).
This combines the prior palette/service artwork, the [four classified cardinal
glyphs](design/reviews/scrum256-cardinal-glyphs.md), and the [actual active-waypoint
name card](design/reviews/scrum257-active-name.md). Active-point symbol/blinking
and navigation semantics remain upstream-owned. SKAGER naming and the approved
Jira logo are wired into the product and Windows resources; compatibility IDs,
original source evidence and required OpenCPN credit are retained.

The [actual native Gettext prerequisite proof now passes](evidence/scrum255-gettext-native-pass/README.md):
25 contracts, successful bounded acquisition, exact-path tool identities and a
real UTF-8 catalog operation. Both earlier focused failures are retained.
The [final seventeen-unit native preflight](evidence/scrum-247-native-chart17-final/README.md)
passes with independently verified downloaded objects and resources. All
[sixteen final ENC comparisons](evidence/skager-product-fidelity-78eccb8-linux/README.md)
pass, and Standard pixels remain identical. The [software and actual-GL route
runs](design/reviews/scrum252-257-final-78eccb8.md) pass all 26 original assertions,
hot themes, active-name cards with preserved icon blinking, stale controls and
clean shutdown. A card-covered GL pixel sample required a documented collector
correction; its original failure and missing-stroke negative controls remain.
No application correction or weakened navigation assertion was involved.

The exact published candidate entered the [full Linux/Windows qualification
run 37088759582](https://github.com/ThereptileII/Work/actions/runs/37088759582).
Windows job 111105527254 built and installed the application, but failed the
offline geographic-name painter at 03:16:18 UTC: water-label pixels exceeded
the required alpha bound. That candidate remains unqualified. The isolated
[native diagnosis and correction](evidence/scrum243-windows-alpha/README.md)
now pass all 2,452 original Windows assertions in run 37094223771 at exact
`7687ce53c780bdfd53db30bb063f9bcc8f6404b7`. GDI+ grayscale coverage restores
translucent text without changing bounds, colors, opacity or any opaque pixels.
The original failed artifact and paired images remain retained. A complete
replacement application/package has not yet been qualified. Linux qualification
continues in the background. No
full release, physical GPU, boat visual pass or obsolete-version retirement is
implied. Whole-chart prototype conformance remains open.

Boat review preparation found a tooling integration gap: the already qualified
cold-baseline recovery checkout predates SKAGER window selectors, while the
application candidate's tools lack that completed-baseline reader. SCRUM-258
[combines those previously reviewed boundaries](installer/skager-boat-tool-composition.md)
without changing the frozen application. Its exact local `3da0e563de40f71fdb5c31df89c9ec65a04826b5`
is published as `ffa2de31ae02ca826ec5d3604f57a8c619035bb1` on the separate
`skager-boat-review-composition` branch: all 2,484 local blobs/modes match and
eight unrelated repository files remain unchanged. Independent source review and
945 focused portable checks pass. All ten jobs in [native tooling run 37090716935](https://github.com/ThereptileII/Work/actions/runs/37090716935)
now pass, with [six original artifacts independently verified](evidence/scrum258-native-tooling/README.md),
including actual dependency refusals and successful Prepare/Arm/Collect. It
built only small test helpers, not another OpenCPN application. At 03:13:19 UTC,
all 117 tools were staged separately and independently rehashed on the boat;
[the staging receipt](evidence/scrum258-native-tooling/boat-staging.json) confirms
unchanged stock executable, profile, installed state and qualified source checkout.
SSH, Tailscale and RustDesk remain running. No new tools were executed and no
application was launched or installed.

## Previous correction batch: chart labels, Night, land and service symbols

Local `f0976cc65ea63d3ed6f60ac53ec38a856d11b66b` is frozen and published as
`9a4231f45d978321c612a6a1de66d735d531bbaa`. Its combined Linux application
[built and staged successfully](evidence/skager-chart-f0976cc-linux-build/README.md).
The [seventeen-unit native preflight](https://github.com/ThereptileII/Work/actions/runs/37085103227)
passed, with downloaded object/source/resource identities independently verified.
It is compilation evidence only, separate from the preceding full candidate.
[Sixteen real-ENC captures](evidence/skager-product-fidelity-f0976cc-linux/README.md)
pass software/OpenGL Day → Dusk → Night → Day, with four clean exits. Exact
Night/land colors, matte-free logo and unchanged Standard chart pixels were
checked. The [active-route comparison](design/reviews/scrum252-integrated-f0976cc.md)
passes all 26 original assertions and hot-theme label checks in OpenGL, but
the first software capture lacks the expected name card, numbered circle and
route understroke. The [diagnosis and repair](design/reviews/scrum252-fixture-repaint.md)
trace this to direct test-scenario mutations missing the redraw notification
used by normal route commands. An ordinary theme repaint on the same executable
shows the correct cards, markers and current geometry. Only the test scenario
was corrected; all 26 navigation assertions remain intact. A fresh capture of
the corrected scenario remains required. No release or whole-chart conformance
is inferred from these captures.

This batch includes theme-aware approved SKAGER artwork, guarded route-point
name cards, effective prototype Night chart colors, the GTK chart-control
recapture fix, exact pilot-boarding/radar-beacon artwork, and the correction of
built-up-area fill to prototype land. It retains navigation classifications,
hazard contrast, Standard resources and Legacy/Safe boundaries. No new boat
installation or old-generation retirement has occurred.

## Previous full candidate: integrated failures retained

The frozen implementation is local
`1356fd1603aacbea04d7081d16331e9a181180bb`, published as
`9fcd3db54ee6913cc144ecfc09ec2152078a6eff`. On October 3 at 00:02 UTC it was
promoted once to the existing `opennav-x-beta2-ui` qualification branch, after
the targeted Windows failures were fixed and verified. Later local evidence and
corrections are separate from this frozen executable.
The exact-source [full run is 37080314681](https://github.com/ThereptileII/Work/actions/runs/37080314681).
Its Linux and native Windows contract jobs pass. The integrated Linux job built
the application and passed all 147 CTest cases, then stopped on the resource
scenario's obsolete `OpenNav` log-prefix expectation. The
[one-line correction and downloaded failure](evidence/scrum235-linux-resource-log/README.md)
retain all resource/profile assertions; a focused same-executable rerun passes
SKAGER, Legacy and Safe with three clean exits. The full job remains failed.
The native job also stopped before application compilation: Chocolatey's Poedit
download returned HTTP 504, which the upstream batch masked until the later
gettext check. [Downloaded failure evidence](evidence/scrum255-gettext-failure/README.md)
is retained. SCRUM-255 now owns early prerequisite verification and bounded
recovery; no blind full-job rerun is started. This is not an observed application
crash. Application/package/installer/rendering/endurance acceptance remains open.

The narrow SCRUM-255 repair checks both Poedit tools before expensive builds,
retries only bounded package-manager failures, and verifies their identities
again before configuration. Its [separate native prerequisite run](https://github.com/ThereptileII/Work/actions/runs/37086572916)
stopped in the offline contracts, before acquisition: the lexical-path guard
mistook canonical Windows path spelling for redirection. That guard is being
corrected to detect actual reparse components; the failed run is retained.
No full application build was started for this failed short proof.

The next local increment includes theme-aware compositing of the approved logo
(`8accdd9`): original asset bytes and letter geometry remain unchanged, the
baked-in rectangular matte is removed, and Night uses the prototype's reduced
brand ink. [Nine focused component drawings](evidence/scrum-236-theme-wordmark/README.md)
cover three themes and 100/125/150% device sizes. They do not qualify native DPI.
SCRUM-252 covers the remaining waypoint-name labels; SCRUM-253 covers the
confirmed difference between raw Night chart tokens and the prototype's
chart-only brightness rule. These changes are not included in run 37080314681.

The combined next increment now includes waypoint name cards (`559a070`),
effective Night surfaces and owned overlay ink (`fcc79aa`), and the confirmed
GTK owner-recapture repair (`1e0a7f1`). Focused source/raster checks pass; the
waypoint cache includes the two review corrections: failed GL replacement
keeps stock ownership intact, and transparent texture padding does not enlarge
the label's hit/cull bounds. The combined `6dd6665` Linux application linked
successfully in 98 incremental steps. This is build evidence, not visual or
Windows acceptance. Night hazard ink, safety contours and soundings deliberately remain
brighter to preserve the measured contrast gates. The [symbol source audit](design/reviews/scrum15-symbol-source-audit.md)
separates exact artwork candidates from decorative examples that would change
charted meaning. SCRUM-254 owns the narrow pilot-boarding/radar-beacon artwork
increment, now integrated as `585fd0f`: both exact prototype assets pass the
pinned native loader and resource-isolation checks. The built-up-area fill is
also being corrected from the earlier shore-color policy to the prototype's
land role. A final combined capture build and native preflight follow these
bounded changes. No newer combined executable is qualified yet.

- [Native sixteen-unit preflight](evidence/scrum-247-native-chart16/README.md)
  passed MSVC Win32, including the actual GL canvas and sounding renderer.
  Downloaded objects, source identity and archive integrity were independently
  verified. This is compilation evidence, not Windows product acceptance.
- The frozen Linux integrated build linked successfully after 104 build steps.
  Sixteen real public ENC captures cover SKAGER and Standard in software and
  Mesa OpenGL, each through Day → Dusk → Night → Day, with four clean exits.
  The strict repeated-edge check now passes; the older failing pictures remain.
  [Captures and independent review](design/reviews/skager-chart-1356fd1-linux.md)
  retain their precise identities and limitations. Two actual upstream route
  scenarios also pass 26 checks each, exact projected
  route-paint samples, stale handling and clean exit. These do not qualify boat
  GPU, Windows fonts or every route/AIS visual state.
  The stale-position GL route image omits floating chart controls visible in
  its software counterpart. SCRUM-228 traced this to the delayed owner recapture
  and now has a [focused repair](design/reviews/scrum228-recapture-repair.md),
  including an actual native stacking negative control. The newer integrated
  application still needs to demonstrate the correction.
- The [fresh read-only boat audit](evidence/boat-readiness-20261003.md) found
  remote access running, no application processes and matching stock/profile/
  recovery hashes. The connected display reports 1920×1080 with configured
  150% scaling; actual application DPI and the requested 1280×800 comparison
  remain unmeasured. The accepted Beta1 installation remains unchanged.

The candidate includes customer-facing SKAGER naming, the approved Jira logo
and Windows icon derivatives, chart/name/light/sounding typography, bounded
route/waypoint/healthy-AIS presentation, the chart framebuffer repair and the
scoped submarine-cable color. **Visual conformance is not accepted:** harbor
label density, small light-description readability, several special-state
symbols and the newer wordmark's integrated rendering still require review or
correction. Standard fallback and navigation meaning remain mandatory.

Full native application, fixture-free packaging, installer/recovery, DPI,
physical display and boat acceptance remain open. No replacement is installed,
no older generation is removed and no physical command is sent by this batch.
The previous candidate's [Linux job and three-hour endurance passed](evidence/scrum-224-linux-baseline-4c597955/README.md)
with both 147-case suites, 1,080 samples and 90 dropout recoveries. This older
source's pass cannot qualify the frozen candidate or these newer changes.

## Previous: SKAGER identity and prototype chart refinements

The latest user instruction requires the approved SCRUM-89 SKAGER artwork and
customer-facing name throughout the product. The combined development branch
now includes native captions/logo (SCRUM-235), Windows application/setup/maintenance
icons and SKAGER distribution/guide names (SCRUM-236), fractional default-route
foreground (SCRUM-237), and geographic chart-name typography/ink (SCRUM-238).
The integrated source is local `1a02ae083501eb05feeef7879e6ee450c076be69`,
published equivalent `280d5e2e28570fed4f2b84eaafc104bad9ee7c1f`. It also includes
the bounded healthy COG predictor (SCRUM-239). Its integrated Linux build and
six real-ENC software captures passed with two clean exits; see the
[visual review and exact evidence](design/reviews/skager-chart-integrated-1a02ae0.md).
Further focused integration now includes onboard AIS bodies (SCRUM-240), the
bounded route understroke (SCRUM-241), verified default waypoint markers
(SCRUM-242), geographic tracking/opacity (SCRUM-243), and the effective
prototype anchorage glyph (SCRUM-244). All nine combined upstream patches apply
to the pinned source. The first combined source `e14de49f444c56de28465416a83337838ad84413`
passed its Linux integrated link and twelve real-ENC software/Mesa-GL captures,
with four clean exits. The [comparison record](design/reviews/skager-chart-e14de49-linux.md)
retains exact identities and two initial collector failures. SCRUM-243's
text-overlap/DPI-bound correction is included. Visual inspection nevertheless
found a theme-transition OpenGL edge-wrap defect in both SKAGER and Standard;
SCRUM-250 is repairing the confirmed framebuffer/viewport mismatch. These
images therefore do not pass the OpenGL visual gate. They contain no active
route or AIS targets and do not qualify those painters. Later integrated
source includes light descriptions (248), sounding typography (249), and the
selector's deferred-fallback correction (246). Native and boat qualification
remain open; older Windows evidence does not qualify these newer changes.
Internal configuration/protocol identities,
existing user data, immutable prototype and required OpenCPN attribution remain
intact. See [native identity](architecture/skager-native-branding.md),
[packaging identity](installer/skager-branding.md),
[route foreground](design/reviews/scrum237-route-foreground.md) and
[geographic names](design/reviews/scrum238-geographic-names.md).

The preceding candidate `4c597955f96647a9c7b837139aa2732f2c3a3fa5` is in
[run 37063131823](https://github.com/ThereptileII/Work/actions/runs/37063131823).
At 22:12 UTC on October 2, integrated Windows job 111026476044 failed to link
`floating_surface_test.exe`: its console target expected `_main`, whereas
`wxIMPLEMENT_APP` supplied the Windows GUI entry point. This is an automated
fixture link failure, not an observed SKAGER crash. SCRUM-245 isolates the
entry-point repair and native lifecycle proof, including a required reproduction
of the original linker error, before another full candidate. No behavioral
assertion is removed. Failure artifact 11254867923 has SHA-256
`82e07a88d0e822a0556fcaa4eebc39ad7a33ca8e4d2a7491a9531097b077861b`.
Linux contracts passed 92/92 and Windows 89/89; 13 other jobs passed. The latest
22:25 UTC observation still has Linux endurance running, not accepted as a
three-hour pass. Windows UI, fixture-free packaging and installer steps after
the failed link were skipped and cannot be claimed passed.
The isolated SCRUM-245 correction then passed native MSVC Win32 in
[run 37073219984](https://github.com/ThereptileII/Work/actions/runs/37073219984),
commit `e67f70e7dafc31b6da9d34d5eab8690a9eab09c8`: the old entry point reproduced
the exact linker errors, and the fixed executable passed all 12 unchanged
Windows lifecycle checks. The downloaded artifact's size/hash/CRC, 130 source
inputs, executable and 11 runtime DLLs were independently verified; see
[the retained receipt](evidence/scrum-245-floating-entry/native-37073219984/verification.json).
This closes that fixture defect, not SCRUM-224 or full product qualification.
Long-running qualification continues in the background while these isolated
visual changes receive focused checks. No redundant full candidate was started
for the individual branding or paint changes.

Prototype conformance remains open. The e14de49 captures show the corrected
COG endpoint, geographic tracking and two vector-selector colors; labels and
several chart-object states still differ. Route/waypoint/onboard-AIS appearance,
native Windows DPI, actual boat GPU and boat-display comparisons remain gates.
The bounded implementations retain explicit special-state and geometry fallbacks.

The first focused native chart preflight [37076177181](https://github.com/ThereptileII/Work/actions/runs/37076177181)
stopped before compilation: Git's repeated-section mode handling rejected
`chcanv.cpp` under Windows file-mode configuration. The exact nine-patch failure
was reproduced; private pinned-index application and independent final-tree
verification fix it without changing the patch contents or tamper guards.
Combined local `19c900afdfca4c3272715c0f998b8941ec45c8df`, published equivalent
`6f22f688cc1156c8c1e2395c854fff1a38398338`, includes all nine verified patches and
15 actual chart production units. The corrected [native preflight37077779848](https://github.com/ThereptileII/Work/actions/runs/37077779848)
failed on the preflight helper's missing `OCPN_Sound.h` include path. The complete
production include set and actual generated sound configuration corrected that
helper; [failure evidence](evidence/scrum-247-chart-preflight/native-37077779848/receipt.json)
is retained. The subsequent sixteen-unit preflight above passes. This short
compile gate cannot substitute for a full application,
fixture-free package, installer, Windows UI or boat acceptance.
No physical
actuator commands or boat-install retirement have been performed by this batch.

## Previous: chart refinements and corrected candidate preparation

The previous stale-snapshot and touch failures below are retained historical
evidence. The corrected source now has two verified development gates:

- [Exact Linux preview](evidence/scrum-224-cf07197-linux-preview/coherence-identity.json)
  at local `cf0719749482e62d48daed02515f2b6c6d242b67`, published equivalent
  `e0e0e9ef71714932ab69d533d1f86cbcd48e33b3`: the 129-check freshness regression
  and one full preview pass. The preview took 213 seconds, retained 44 captures,
  exercised 60 pointer actions and all eight scenarios, and observed seven
  clean application exits through XNav, Legacy, Safe and direct Legacy return.
  Real coastline remains visible in the reviewed final return. This run used
  process/signal tracing; it does not retrospectively identify the older747
  SIGKILL cause. The interrupted build wrapper, brief overlapping Ninja and
  explicit object/cache recovery are recorded, not hidden.
- [Downloaded native touch proof](evidence/scrum-229-cf44-native-touch/verification.json)
  for `cf44e938a3539af6e97a03346542f04f3e7895a4` passes actual 125% Windows
  input. The original Edit target gives zero scroll. Three native body-hit pans
  move 480 px, leave Save and the lower action fully visible, and an exact tap
  activates the action once without saving or changing fields. The initial
  test's unsupported forwarded-message-count assumption was removed; native
  hit, movement, settled geometry and state assertions remain strict. Artifact
  hashes, 273 recorded input entries, 29 objects, 12 runtime DLLs and both test
  executables were verified. This is a component proof, not full-app DPI or
  physical touchscreen acceptance.

After those checks, the exact `cf44e938` source was promoted once to
[full run37057756273](https://github.com/ThereptileII/Work/actions/runs/37057756273).
Its Linux and Windows portable contract jobs found a missing shared-helper
binding in the older geometry test (91/92 and 88/89 CTest entries passed).
This is a test-harness `NameError`, not an application crash. The downstream
Windows integrated job was skipped; running upstream/Linux jobs remain intact.
The two-file repair at local `3b944213` / published `16187f407754a3dbe137d03311518e27445b03d7`
passes all 24 retained cases locally and in [native short run37058880373](https://github.com/ThereptileII/Work/actions/runs/37058880373)
under Windows cp1252. No product or behavioral assertion was removed.
Chart refinements are being batched separately before another full candidate.
No replacement package or boat deployment is yet qualified.

## SCRUM-15 / SCRUM-231 / SCRUM-232 — user chart feedback confirmed

The user's screenshot concern is valid: the active XNav chart presentation
still differs visibly from the immutable prototype. Functional chart tests do
not qualify chart appearance. The retained FFE ENC view uses XNav resources,
but its large mustard built-up areas retain stock CHBRN. Inspection of the
hash-pinned public US5SEAFL cell identifies the Seattle and West Seattle
BUAARE polygons. SCRUM-231 changes only their fill through a dedicated XNav
paint role using the prototype shore neutral. CHBRN also colours hazards and
light sectors and remains intact for those uses. [Matched local ENC evidence](evidence/scrum-231-built-area-local.json)
now confirms the correction in all three themes and pixel-identical Standard
chart regions. The strengthened resource guard passes 3,484 checks and leaves
the captured generated resources byte-identical. Native Windows, OpenGL and
boat acceptance remain open.

SCRUM-232 separately tracks the stock red default ownship versus the prototype
chevron, preserving OpenCPN position, heading, accuracy, user-icon and scaled
ship semantics. Heavy labels and chart-symbol/route/waypoint/AIS styling remain
open. Some illustrative prototype danger symbols omit required chart meaning;
they cannot be copied indiscriminately. No primary chart view is visually
accepted. The chart corrections are isolated from the frozen stability
candidate. The combined refinement source now includes:

- Dedicated built-area fill, with matched Day/Dusk/Night real-ENC captures
  and byte-identical Standard chart regions (SCRUM-231).
- Shared prototype ownship chevron at the existing OpenCPN position/rotation,
  retaining stock custom, scaled and low-accuracy paths. The three changed
  production units compile; 488 focused painter checks pass, including the
  Windows max-macro hazard and shared software/GL DPI factor (SCRUM-232).
- Available owned online AIS names, exact prototype baseline and theme ink,
  bounded placement and stale/lost label removal. 58 core checks and the updated
  26-check painter pass; the production overlay compiles (SCRUM-233). Independent
  review found OpenCPN's 500-pixel text-measurement clamp. The correction omits
  saturated metrics; its regression exercises a real 1,024-pixel name reported
  as 500 pixels. Exactly-500-pixel labels are conservatively omitted too.

The combined local `25297e3` / published `6e0117de` source completed one
136-step integrated Linux link and a short real-ENC Day/Dusk/Night run with a
clean exit. [Retained identity and captures](evidence/scrum-15-25297-integrated-chart/identity.json)
bind the exact executable and public chart. Root review confirms the neutral
built areas and east-facing prototype chevron. Controlled loopback input was
fresh; it is not real boat-data evidence. These captures precede the label
saturation repair and contain no online targets, so cannot qualify that repair.

The same comparison still shows prominent text, dense soundings, stock hazard
symbols and a red upstream course predictor. This is not prototype conformance.
The [remaining presentation review](design/reviews/scrum15-chart-comparison-20261002.md)
records the differences and source boundaries. SCRUM-234 now corrects a
separately confirmed GL-only online AIS stern-notch defect by changing only
the polygon's cyclic starting vertex. The original fails two topology checks;
the correction passes all three, including 1,326 interior comparisons and
unchanged software path geometry. Its production object compiles. This is
source/topology proof, not an actual OpenGL capture.
Windows, actual GL, physical boat review and remaining chart-symbol/label/route
refinements are still open. No replacement package is qualified yet.

## SCRUM-224 / SCRUM-228 / SCRUM-230 — clean GTK fix verified locally; stale snapshot blocks preview

The coherent Linux build at `a3edcaef2904bf1450e306f3e086164479ec7f6f`
contains the approved GTK floating-surface fix without temporary diagnostics.
Its short viewport check passed actual zoom/palette button activation, stable
chart center, correct painted positions and coastline/layout checks.
[Exact build and failure evidence](evidence/scrum-224-a3edcae-local-preview.json)
binds executable SHA-256
`cb195d7356f5891aa2c4831b64f3bc8f33f0fe9e4fc4670c696200858b404985`.
The prior747 and671 binaries and evidence remain preserved.

The one traced full preview then failed the existing stale-data assertion.
At5038ms, the exported snapshot marks samples stale and omits route distance,
but still includes an aging arrival-SOC estimate. The retained
[UI screenshot](evidence/scrum-230-a3edcae-stale-ui.png) already shows unavailable
values and “Battery SOC stale”; this is an inconsistent snapshot observation,
not proof that stale advice remains visible. SCRUM-230 is In Progress. No
assertion was relaxed and no rerun was performed.

Lifecycle checks were not reached. The original747 missing-window/SIGKILL
uncertainty remains open; its observed descendant was not proven to be the
restart child. Process tracing can alter timing and grants no waiver. This is
not a full preview, native Windows or boat pass, and no new full candidate was
dispatched by this local verification.

## SCRUM-224 / SCRUM-229 — ffe native build passes; preview and 125% touch gates fail

The frozen [ffe run](https://github.com/ThereptileII/Work/actions/runs/37041103887)
has completed its native job. [Downloaded terminal evidence](evidence/scrum-224-ffe-native-final.json)
verifies artifact11246267901 (43,226,505 bytes, SHA-256
`50ab9ff8307f85b792c9f1ee8ddb5fe771cb96c23527b1402af5c1e1459d853e`).
Native compilation and138/138 integrated tests passed, as did installed peer
and loader checks, pointer navigation/route gestures, three recovery sequences
and the public ENC/plugin gate. The requested OpenGL phase actually used
software fallback; this is not actual GPU/OpenGL acceptance.

The preview stops at the obsolete Settings → Display → Chart presentation
path already corrected in the prepared747 helper. The native125% DPI run
separately fails to reach the lower Vessel Preferences action by touch. Its
unverified pan start falls on the editable battery-capacity field; SCRUM-229
tracks a strictly checked scroll-body gesture and native replay. The100%
sequence completed,125% remains failed,150% was not reached. Fixture-free
product, installer, package-security, native endurance and boat gates were not
qualified; no replacement full candidate or boat deployment has started.

## SCRUM-224 / SCRUM-228 — local preview repair verified; two bounded failures retained

The coherent Linux build at `74761400862bc1d25102b2b741d14247dd343cdc`
verifies the corrected visible Diagnostics selection, all eight data scenarios,
60 pointer actions and XNav → Legacy with coastline content.
[Retained preview evidence](evidence/scrum-224-747-local-preview.json) records
the subsequent Legacy → XNav failure: an OpenCPN-named descendant received
SIGKILL before the expected replacement window/startup log appeared. Its empty
observed argv does not establish which forked child it was. The sender/cause is under
investigation; this is not a full preview or lifecycle pass. An earlier
interrupted run remains separate. No new full candidate was dispatched.

A separate three-input check of that same executable confirms an initial
Linux floating-chart-control visibility/geometry inconsistency after startup
resize. The chart receives the zoom-labelled inputs while controls are not
painted; palette input still works. [Evidence and images](evidence/scrum-228-747-local-viewport.json)
retain the actual input trace and unchanged scale. SCRUM-228 tracks the fix.
This does not explain the older c95 scale drift or establish a Windows failure.
The frozen ffe native build continues; package/boat promotion remains blocked
by its Linux preview failure. Boat files, profiles and remote access are unchanged.

## SCRUM-224 / SCRUM-14 / SCRUM-226 — combined focused Windows proof passes

Published `ffe4ecf11f43a916098fefe8f17071c63867a838`
(local `f220b31391815663c442d1f8dcbe98a54c03333c`) passed
[native run37039246758](https://github.com/ThereptileII/Work/actions/runs/37039246758):
**469 component checks** (Settings250, Search85, Chart46, Energy88). The strict
Layers and drawer geometry checks and runtime staging guards also passed.
[Downloaded proof](evidence/scrum-224-ffe-focused-native/verification.json)
verifies exact artifact11241214936,259 unique source/patch/lock inputs,
31 compiled objects,12 runtime files,four executables and30 canonical captures.
The immutable Search reference job was intentionally skipped.

Native review confirms the System intro/row order, truthful disabled states and
visible recovery/Advanced links. Title/subtitle ink remains2px/4px above the
canonical reference, and row font metrics remain unmatched; this is partial
System refinement, not whole-view visual acceptance. Search/Chart retain their
previous appearance (only Shell clock pixels differ). The Energy header repair
passes actual pointer-close/reentry after scrolling and resizing; both retained
header screenshots show the action in its correct upper-right position.

After the downloaded focused evidence passed, the exact same `ffe4ecf` commit
was promoted once to [full qualification run37041103887](https://github.com/ThereptileII/Work/actions/runs/37041103887).
It is frozen. The prior88 Linux layout-oracle failure remains retained below;
its native job subsequently failed the same obsolete Layers oracle after successful compilation and138 tests. Waiting for that already-disqualified
candidate's terminal native result is no longer a prerequisite to starting the
corrected candidate; no run was cancelled or restarted. No package or boat
promotion follows from the component-only pass. Exact full-shell DPI,
dependency/security, installer, recovery and physical-display gates remain open.

The exact ffe Linux job then failed in the preview interaction check after the
repaired Layers check, navigation/input, recording, recovery and user-flow
checks passed. [Downloaded failure evidence](evidence/scrum-224-ffe-preview-selector-failure.json)
shows a hidden previous Diagnostics control counted alongside the visible
Preferences action. No crash was reported. A focused selector repair is being
verified before any new candidate; the native job continues unchanged. This
candidate is not eligible for boat deployment or package-security probing.

The older c95 Linux job completed:146/146 fixture-integrated tests and the same
146/146 fixture-free product tests passed. Its downloaded
[three-hour evidence](evidence/scrum-224-c95-linux-final.json) contains1080 samples,
zero sustained handle/thread growth, resident-memory median change−106496 bytes,
and2.58% of one CPU core. The final soak image is all water after substantial
viewport scale/longitude drift; it is explicitly **not chart-content acceptance**.
Numeric endurance does not qualify ffe, native Windows, or the boat display.
Boat/profile/remote access and public payment/download access were unchanged.

## SCRUM-227 — disposable AIS outage boundary verified

The separate [native run37047212685](https://github.com/ThereptileII/Work/actions/runs/37047212685)
passes on exact source `97142aa1a99ed789380274f20ae7a5c5349d18aa`.
[Downloaded evidence](evidence/scrum-227-disposable-native-proof.json) verifies
all four source inputs,three Win32 executables and both IPv4/IPv6 phases.
Existing marker connections were interrupted, retries were blocked and traffic
recovered after both normal helper exit and forced termination. Independent
control traffic stayed connected without errors. Fresh independent queries
confirmed the uniquely owned filters and sublayer were removed in both phases.

This helper can target only inert loopback marker executables in a disposable
hosted runner. It does not target OpenCPN,AISStream or boat traffic. The initial
RPC type compile failure and strict readback refusal were retained; the latter
was documented Windows INDEXED metadata, now explicitly requested and compared
exactly without masking unknown flags. Actual-product binding and live
AIS loss/aging/reconnect remain open;SCRUM-227 is not Done. Product candidate,
boat installation, remote access and hardware-control policy are unchanged.

## SCRUM-224 / SCRUM-213 — native focus repair proven; integrated layout oracle correction required

Published source `88c141ba9fee5c28e12b5f4ac0f7e7550d1a8ca6`
(local equivalent `688bb713db885dbe78ca6db861382ce248c231f7`) passed the
[focused native run](https://github.com/ThereptileII/Work/actions/runs/37032588707):
230 Settings + 85 Search/Shell + 46 Chart checks, plus the corrected strict
large-desktop geometry oracle. Downloaded artifact `11238311244` was verified
against its SHA-256, 247 source inputs, 31 objects, 12 runtime files, three
executables and 23 canonical captures. Combined Linux Settings passed 181
checks with 14 captures; this does not replace integrated Linux qualification.

The retained `60929a5` negative control reproduces the actual failure:
restoring focus to Advanced battery model scrolls Sensors out of its pointer
target. Explicit Preferences reopening now focuses its current visible tab
before resetting scroll. The unchanged native pointer checks and before/after
trace prove stable geometry and real Sensors selection. The independent
1920-width failure was an obsolete test expectation: the immutable prototype
and native product require 460 DIP at the large-desktop breakpoint. No tolerance
was widened and no release gate was removed.

[Verified native evidence](evidence/scrum-224-88c-focused-native-proof.json) and
[scoped visual review](design/reviews/native88-focused-review.md) cover the
Search row/focus outline, shared Windows title/border, Chart supporting text
and System flow corrections. Original HTML is unchanged. Whole-view conformance,
other DPI/Back-header states and boat acceptance remain open.

After this evidence passed, the **same commit** was promoted to
[full Linux/Windows qualification run 37033702537](https://github.com/ThereptileII/Work/actions/runs/37033702537).
The candidate remains frozen. Its integrated Linux job failed before navigation-object interactions because the strict chart-layout oracle still listed the six pre-Layers floating controls. The actual new44×44 Layers button matches the immutable prototype. [Verified failure evidence](evidence/scrum-224-88c-linux-layout-failure.json) retains the real screenshot and layout. The isolated correction requires the exact seventh control and preserves one-pixel geometry bounds and arbitrary-overlay rejection; captured negative/positive replay plus nine targeted rejections pass. No application geometry changed. Native Windows continues independently; **this run is not eligible for boat deployment or security-package probing**. Full product/dependency, installer/recovery, native DPI and boat gates remain unaccepted.

The same commit's [native recovery subset](evidence/scrum-17-88c-recovery-tooling.json) has21 passing maintenance reports,384 marker-process checks,51 actual broker checks and56 Prepare/Arm/Collect checks, with downloaded bytes/source identities verified. These disposable tests do not imply real-profile or application acceptance.
The older c95d failed native evidence remains preserved; its already-running
Linux endurance continues independently. The boat installation and remote
access were not modified; older-install retirement remains gated by a known-good
replacement. No public payment/download access was opened.

## SCRUM-224 — c95d native UI gates fail; short prototype proof passes

The frozen `c95d3a0` [full run](https://github.com/ThereptileII/Work/actions/runs/37017351644)
passed native compilation and all 138 integrated tests, installed peer CLI,
staged loader, pointer route gestures and repeated crash recovery. It then
failed the Sensors Preferences pointer check and the 1920-wide Preferences
width assertion. [Verified failure evidence](evidence/scrum-224-c95-native-ui-failure.json)
records both artifacts and exact scope. The latter oracle expects 432px where
the immutable prototype and actual native drawer require 460px. The former
shows body scrolling after foreground activation and remains under investigation.
No product package, installer, installed security probe or boat promotion is
accepted. Linux endurance continues independently. Narrow reproductions and
corrections precede any further full candidate build.

Separately, `29ea06a358ed24cc29eb2be74b67b234a3ecaa6e` passes
[focused native run 37026807977](https://github.com/ThereptileII/Work/actions/runs/37026807977):
190 Settings, 82 Search/Shell and 46 Chart drawer checks (**318 total**), plus
compilation of all 28 production UI objects and the complete NavigationActions,
NavigationObjects and SettingsStore units. The downloaded artifact, 247 source
inputs, 31 objects, three executables/runtimes and 23 captures were verified.
[Exact proof and limitations](evidence/scrum-224-29ea-focused-native-proof.json)
remain component-scoped. Display track colors/geometry are supported; known
other Settings differences and physical boat acceptance remain open. This pass
does not qualify the full candidate or the later local System-flow correction.

## SCRUM-14/15/216 — bounded prototype follow-up; native proof pending

Published follow-up `34b8509` reached native CMake setup in
[short run 37025085247](https://github.com/ThereptileII/Work/actions/runs/37025085247),
which rejected an obsolete include directory before compilation or execution.
The [retained artifact and all-path audit](evidence/scrum-224-prototype-native-setup.json)
identify the narrow correction: remove `gui/src/s57/include`, absent from the
pinned source tree; retain strict checks for all 27 actual directories and all
production headers/macros. Only this short proof is retried. No application
crash, dependency qualification, Windows UI pass or boat acceptance is inferred.

Retry `9b0766b` / [37025995465](https://github.com/ThereptileII/Work/actions/runs/37025995465)
compiled both complete navigation-bridge units and the production UI library.
It stopped linking the standalone Search test because its Windows console
entrypoint was missing. Both new fixtures now use the established explicit
`main` → `wxEntry` pattern. Their local checks still pass (82 + 46); only the
short native proof is retried. Product sources and `c95d` remain unchanged.

Separate from frozen candidate `c95d3a0`, the October 2 follow-up restores rail
metric icons and the Display selector track, adds saved-route/waypoint Search,
and connects the native Chart presentation drawer to floating Layers and
Settings → Navigation. OpenCPN owns the chart preferences; commands display
observed readback, and unsupported/managed layers have no invented switch.

The combined Linux component build passed. Search and Shell entrypoints passed
82 checks, including rejected-action readback, Escape/Close and a short-chart
overlap boundary; the chart drawer passed 46 checks. Two synthetic Shell captures
and exact local source/executable identities are recorded in
[scoped evidence](evidence/scrum14-prototype-combined-linux/verification.json).
Earlier component-only Display and rail reviews retain their original scope.

A separate short native proof compiles the complete changed navigation bridge
and production UI, then runs the Settings, Search and Chart components. It does
not rebuild dependency producers, qualify an installer, or launch on the boat.
Windows and boat conformance remain pending. The full `c95d3a0` candidate remains
unchanged: at 15:04 UTC its Linux endurance step and native integrated build were
still running; no product artifact or boat replacement is accepted here.

## SCRUM-224/214 — native encoding proof passed; c95d replacement qualifying

Published replacement `c95d3a091bb2a0ce0e19d0146ab28dc848ef5290`
(local `24d0a5a`) passes [native Python-only run 37017228040](https://github.com/ThereptileII/Work/actions/runs/37017228040):
Windows Python 3.12.10 with default cp1252 completes all 24 tests in 0.303 seconds.
The GitHub comparison against `2060` confirms only the geometry test file and
its small Python-only workflow changed; product code is unchanged. Both AST
source reads now explicitly use UTF-8, with a regression for legacy Windows
decoding. Geometry, clipping and touch assertions remain intact.

The preceding `2060` full run failed Windows contracts at 86/87 tests: implicit
cp1252 decoding corrupted the Unicode minus label in the geometry harness,
failing thirteen subcases. Native integration was skipped. This is a test-source
encoding defect, not a product crash. Other Linux jobs from that revision remain
running; no endurance result is inferred. The earlier 190-check Settings
component proof remains valid within its recorded scope.

After the native Python proof passed, the integrated branch advanced to exact
`c95d`. [Replacement run 37017351644](https://github.com/ThereptileII/Work/actions/runs/37017351644)
is in progress. Full native application, CLI, DPI/visual, installer/recovery and
endurance acceptance remain open. See the [failure and replacement proof record](evidence/scrum-224-c95d-native-python-proof.json).
No product, boat or release acceptance is claimed.

## SCRUM-224/98 — retained 2060 focused native proof

Published source `2060f8f6b078db6e74e6f8b111e7439073ca1bce` (local `af33a05`)
passes [focused native run 37014950505](https://github.com/ThereptileII/Work/actions/runs/37014950505).
The downloaded artifact matches its API length and SHA-256; all 867 ZIP entries
pass CRC/path checks. Native Settings completes 190 checks, fourteen captures
and ordered interaction steps 0–23, including root reopening. The executable,
twelve runtime DLLs and fourteen captures match their retained hashes. All five
changed objects and 26 UI objects compile; both legacy macro negative controls
fail as intended. The native home-directory excerpt proof passes, but does not
execute the actual installed CLI. See the [scoped proof](evidence/scrum-224-2060-focused-native-proof.json).
All 256 retained source/input records correspond to exact local `af33a05`:
seven are byte-identical and 249 match the expected Windows CRLF checkout.

The preceding `f2704c3` probe failed because the Settings test's recurring timer
re-entered its interaction sequence. That failure remains retained. The `2060`
correction changes only `tests/settings_drawer_test.cpp`; production Settings
code remains unchanged.

After this proof, the integrated branch advanced once from `488f` to exact
`2060`. [Full candidate run 37015958242](https://github.com/ThereptileII/Work/actions/runs/37015958242),
created at 13:52:44 UTC, later failed Windows contracts and skipped native
integration as described above; separate Linux jobs remain running. Actual CLI, complete DPI/fullscreen,
chart/plugin, installer/recovery, runtime closure and native endurance gates
remain open. Captures still require visual review; this is neither product nor
boat acceptance. Publication to `ThereptileII/Work` was explicitly authorized
on 2026-10-02; the repaired source and earlier qualification evidence are
published. No boat deployment or retirement has occurred.

## SCRUM-224 — compile repaired; three native qualification failures isolated

Frozen `488fbbdf5161e986c986773187b1008a7f8684f5` has completed
[run 36984898997](https://github.com/ThereptileII/Work/actions/runs/36984898997):
14 jobs passed, native integration failed, publication was skipped. Application
compilation/linking now pass. Windows passes 138/138 application tests, native
chart/route pointer flows, repeated crash recovery and the public ENC/software
and OpenGL chart gate. These are not complete Windows product acceptance.

The native failures are now source-grounded: the peer CLI refusal test runs
after GUI processing has created the normal profile directory; root Preferences
reopening retains a scroll offset that hides its section tabs; and the 1920px
DPI oracle expects 61px navigation although the immutable HTML and actual
native rendering require 69px. Local repairs preserve the unknown-profile
refusal and strict UI assertions. The Preferences component passes 179 checks
and its no-reset negative control fails; 23 geometry tests distinguish the
prototype width breakpoint and reject deliberately incorrect sizes. The first
[focused run 37013831216](https://github.com/ThereptileII/Work/actions/runs/37013831216)
subsequently failed on test-timer reentrancy. Its retained failure and the passing
replacement proof are distinguished above; full candidate qualification remains
pending.

Linux integrated qualification passed: 146/146 tests in each fixture and
fixture-free configuration, plus the actual three-hour trip. The latter records
1,080 samples, 540 UI actions, about 1.1 MB resident growth, no measured
handle/thread growth and 3.21% of one CPU core. This is Linux evidence, not a
substitute for native endurance or boat acceptance. Both artifacts have verified
size, SHA-256 and ZIP CRC. See the
[terminal qualification record](evidence/scrum-224-488f-terminal-qualification.json).

No fixture-free Windows package or installer was produced; those dependent
stages were skipped. The boat is unchanged, old installations are retained,
and prototype fanout remains held. On 2026-10-02 the user explicitly authorized
publication of the reviewed source repairs and non-sensitive qualification
evidence to `ThereptileII/Work`. Fresh native acceptance remains required.

## SCRUM-224 — earlier focused compile proof and prerequisite results

Frozen candidate `488fbbdf5161e986c986773187b1008a7f8684f5` (local `548a376`)
passes [the focused native compile](https://github.com/ThereptileII/Work/actions/runs/36984238840).
The verified artifact reproduces both original Windows macro failures, then
compiles the four corrected networking units, production UI target and full
SettingsStore. All 31 retained I386 objects match their recorded hashes; this
covers 14 of the 15 changed XNav UI/integration source files. See
[exact-source proof](evidence/scrum-224-native-changed-units-488fbbd.json).
The first short probe stopped at a CMake path-escaping error before compilation;
the corrected probe passed without rebuilding the maintained dependencies.

The [integrated run](https://github.com/ThereptileII/Work/actions/runs/36984898997)
subsequently verified full application compilation/linking, then failed the
three later checks described above. The new early compile gate detects the
original macro failure before expensive producer suites. No boat installation
or retirement occurred.

Eleven early artifacts from this run have verified API sizes, SHA-256 and ZIP
CRC, including the same-run restart prerequisite receipt. Windows pristine
passes 60/60; Linux pristine retains its two named upstream failures (64/66).
These are prerequisites, not integrated product acceptance; see
[early evidence](evidence/scrum-224-488f-early-gates.json). The earlier 09:02 UTC
running observation is superseded by the terminal record above. Neither job
was restarted or cancelled to collect it.

The same candidate's completed contract logs confirm 90 Linux / 87 Windows
CTest passes, 43 dependency contracts per platform and ten additional restart
executions per platform, with no failures in those suites. See
[contract evidence](evidence/scrum-224-488f-contracts.json). These results do not
replace integrated application, installer, UI or boat qualification.

## SCRUM-22/27 — installer preservation checks prepared in parallel

The unsupported-build smoke assertion previously checked the supported sibling
directory rather than the rejected installation. Test-only revision `0fc08f9`
now snapshots the actual rejected tree and pre-existing seeded profile, verifies
the exact rejection reason and checks that installation/shortcut folders remain
absent. It records Setup, package-manifest and test-source hashes. Installer
product code is unchanged. This correction is separate from frozen `488f`;
native proof against its eventual verified package remains pending.

[The recovery coverage audit](evidence/scrum-27-real-release-coverage-audit.json)
confirms genuine accepted Beta 1 and genuine early Beta 2 installers are used
by the existing update/rollback harness. Historical 45-check recovery evidence
is retained separately from pending candidate acceptance. There is no need to
rerun another complete application build for this audit or the test correction.
Neither issue is Done and no boat retirement has occurred.

## SCRUM-24/25 — signed release policy passes native tests

The isolated update branch at `a3a7bac04a3bdda8ddd485450a7dab8aedbd2c4d`
passes [Linux and native Windows/386](https://github.com/ThereptileII/Work/actions/runs/36939227135).
Each platform passes 19 top-level tests and 30 subtests, without failures/skips,
plus `go vet`; both downloaded artifacts have verified length, SHA-256 and CRC.
The policy distinguishes upgrades, same release, downgrades, conflicting source
identities and unsupported OpenCPN installations. Strict JSON and Windows path
checks reject the case-alias and reserved-name defects found during review.
The read-only composition binds policy identity to actual signed TUF metadata;
no installer executes and no startup updater is enabled. See
[exact native evidence](evidence/scrum-24-release-policy-native.json).
Root/key/cache custody and transactional recovery remain open. The boat candidate
is still separately frozen at `45b8a9d`.


## SCRUM-25 — isolated signed-update verifier passes native feasibility

The separate verifier branch at `24bd013f4891638f0e59411836c67f271a66a645`
passes [Linux and native Windows 386](https://github.com/ThereptileII/Work/actions/runs/36936782032).
Both downloaded result artifacts have verified size, SHA-256 and ZIP CRC.
Each platform passes six top-level tests and nine subtests without skips, plus
`go vet`. The tests use real Ed25519-signed metadata generated only in temporary
fixtures and reject altered signatures/content, expired or older metadata,
wrong channels/identity, oversized data and cross-origin redirects.

This is a feasibility result, not a shipping updater. It neither executes a
package nor modifies the application or installer. Trusted-root deployment,
cache protection, application release/downgrade policy, dependency licensing and
transactional installer handoff remain open; see
[the scoped evidence](evidence/scrum-25-verifier-feasibility.json).
SCRUM-24 now owns the deterministic release-policy prerequisite. Product
qualification continues independently on frozen `45b8a9d`.


## Current replacement — verified early gates

All ten native boat-tooling jobs at exact `45b8a9d` pass. Twenty-seven
available early artifacts have independently verified lengths, SHA-256 and
all 1,237 ZIP entry CRCs. Supplemental AIS passes seven tests per platform;
the Linux network fixture passes 18 TLS/transport scenarios and 30 offline
checks. Prototype references capture 63 states per platform, without implying
native application visual acceptance. Pristine Windows passes 60 tests;
pristine Linux retains its two classified upstream failures out of 66 and is
not release acceptance. Integrated Linux endurance and the three native product
build jobs remain in progress at this observation; installation remains withheld.
See [the exact early evidence](evidence/scrum-224-45b8-early-gates.json).

## Current replacement — native certificate path repair

The application candidate is now frozen at
`45b8a9d2809d0d34171010ad1c00f4a3cd056a09` (local `9c4740f`).
The previous `37978e` composition and object-workflow builds failed before curl
configure because a Windows-backslash script path made MSYS Perl look for
certificate inputs in the output directory. The narrow caller repair uses the
verified MSYS Perl explicitly and passes the same slash form as upstream CMake.

[Focused native run 36935540968](https://github.com/ThereptileII/Work/actions/runs/36935540968)
passes the new actual-script path regression and retained host/certificate/tool
checks. Its downloaded artifact has verified length, SHA-256 and all 20 ZIP CRCs.
See [the scoped proof](evidence/scrum-224-45b8-native-path-pass.json).
Fresh [integrated](https://github.com/ThereptileII/Work/actions/runs/36935775514),
[prototype](https://github.com/ThereptileII/Work/actions/runs/36935778459) and
[native tooling](https://github.com/ThereptileII/Work/actions/runs/36935773834)
runs target that exact commit; they are not yet accepted. No deployment or old
installation cleanup has occurred. A read-only boat heartbeat at 22:24 UTC
confirmed zero OpenCPN processes and all three remote-access services running.


## SCRUM-218 — complete native diagnostic log retained

The separate read-only collector at `c42ff4c97ee71cfe84bab54873ee5dac18f15510`
passes [run 36932209206](https://github.com/ThereptileII/Work/actions/runs/36932209206)
and ten focused refusal/security tests. It retained all 1,738,273,051 bytes of
the cancelled b48 prototype job's log within the explicit 2 GiB / eight-minute
limits. The downloaded archive and uncompressed log have independently verified
hashes and lengths. No running product job was modified or restarted. This is
diagnostic evidence, not application acceptance; see
[the collection record](evidence/scrum-218-complete-native-log.json).

## Previous candidate qualification — 37978e

The previous frozen candidate is `37978e26517af810092ec2bc554b199d36a705d2`
(local `2a3c2c6b88945a50a7b552fd41c37658747ac5a4`). Its
[integrated run](https://github.com/ThereptileII/Work/actions/runs/36929977632)
passes 90 Linux and 87 native Windows portable CTest contracts, plus ten
repeated restart handoffs on each platform and the supporting policy suites.
All ten [native tooling jobs](https://github.com/ThereptileII/Work/actions/runs/36929976171)
pass. Fourteen downloaded early artifacts have verified size, SHA-256 and ZIP
CRC records, including 238 actual COM shortcut migration checks on each host
architecture, 22 maintenance reports, native broker/Prepare/Arm checks, and
supplemental AIS tests. See [the exact-candidate early evidence](evidence/scrum-224-37978e-early-gates.json).

The [prototype run](https://github.com/ThereptileII/Work/actions/runs/36929980191)
has passed its supporting checks, but native composition job `110596574723`
failed before curl configure: MSYS Perl received a backslash script path,
and the certificate generator searched the output directory for source inputs.
The producer correctly rejected the empty certificate. The separate object-flow
job failed at the same preflight; its independently verified artifact confirms
the same error. The integrated Windows job also failed at the same preflight; all three
native failure artifacts have verified hashes and ZIP integrity.
See [the verified failure record](evidence/scrum-224-37978-certificate-path-failure.json).
The replacement caller-path repair has passed its focused native gate; its
full qualification remains open as recorded above. No existing generator or
TLS assertions were weakened.
No artifact is approved for boat deployment. The previous b48 Linux job has
completed successfully, including 146 fixture and 146 fixture-free test cases,
the 10,800-second soak, loader checks and mode/navigation/chart fixtures. See
[its bounded evidence review](evidence/scrum-224-b48-linux-complete.json).
This older Linux result is not acceptance of this replacement revision. The boat remains
unchanged and old installations are retained until a replacement is known-good.

## SCRUM-224 — native test-host producer binding verified; product gates remain open

The focused producer binding run at exact commit `fcd5adfca9e4ffdda00eddd40a48b2d85b433587` completed successfully in [run 36929342744](https://github.com/ThereptileII/Work/actions/runs/36929342744), job 110594459325. Its 13,192-byte artifact (SHA-256 `5d17f69c19d7c4f763d4e7277565708318f5b5f54ce77b1dd73b502d8b729db0`) passes all 19 ZIP CRC checks. The native Win32 CMake cache binds curl tests to MSYS2 Perl; wrong-host and tampered-cache cases refuse, and restored facts pass. Local 13+14+9 checks pass. This qualifies the test-host binding diagnostic only, not the complete upstream curl/HTTP/TLS suite or application, UI, installer, or boat acceptance. See [SCRUM-224 native binding evidence](evidence/scrum-224-native-test-host-binding-pass.json).

## SCRUM-224 — native curl test-host proof; frozen application gates remain open

The frozen candidate `b48bf4a8f12f98c459d79aa08805141ac49e0306` still has two
cancelled native prototype jobs in
[run 36901029053](https://github.com/ThereptileII/Work/actions/runs/36901029053);
their UI gates were skipped. Bounded retained logs show Strawberry Perl reaches
a curl test-runner state where Windows cannot resolve `exec`, curl fails to
start its HTTP server, and test 1 remains active.

Four focused native cases on source commit
`622157fed6f2a3a9c5aeb476e502941789790498` use unchanged curl 8.22.0
`runner.pm` and `servers.pm`. Strawberry Perl reproduces readiness and server
launch defects; MSYS2 Perl reports `cygwin`, passes readiness and starts a
disposable marker process. This confirms the test-host choice is material to
these modules. The [pinned Windows workflow](https://github.com/curl/curl/blob/01346829096c61b372692f6dc43ffa778c6caccd/.github/workflows/windows.yml)
selects MSYS2 for curl tests while the product target remains native Win32 MSVC.
See [host-probe run 36926615433](https://github.com/ThereptileII/Work/actions/runs/36926615433).

The follow-up [run 36927000029](https://github.com/ThereptileII/Work/actions/runs/36927000029)
also passes certificate generation and CA/host/key verification using the
existing patched `genserv.pl` under MSYS2, with explicitly selected host OpenSSL
3.6.4. No additional generator patch was needed; this does not qualify the
pinned production OpenSSL 3.5.9. The earlier b48 composition artifact also
shows the pinned OpenSSL 3.5.9 certificate setup generated local test material
and verified `test-localhost.crt` before the runner failure. Neither focused
probe is a full HTTP/TLS or curl-suite pass. Earlier failed selector and
host-label checks remain preserved in the
[SCRUM-224 evidence](evidence/scrum-224-native-test-host-pass.json).

The separate [integrated run 36901026915](https://github.com/ThereptileII/Work/actions/runs/36901026915)
still had Linux integrated and native MSVC XNav/Legacy/Safe jobs in progress at
21:14:07Z. Full application, UI, installer and boat acceptance remain open.

## Current qualification and boat cleanup — 2026-10-01

SCRUM-222 cold-profile preservation tooling at
`034545be03295352a1c509d8979e4a3eb5bf64b6` passes all ten native Windows
tooling jobs in [run 36911321279](https://github.com/ThereptileII/Work/actions/runs/36911321279).
Downloaded artifacts were independently size/hash/ZIP checked. The cold suite
passes 40 native disposable groups, including actual private ACL refusal and
restoration plus normal commissioning/restore; see
[native preservation evidence](evidence/scrum-222-native-cold-baseline.json).
This qualifies the exact tooling, not the frozen application or boat acceptance.

The boat source checkout was updated to that exact tooling commit without
changing the installed application. An earlier SCRUM-222 cold-profile capture
refused before backup publication because two known-hash o-charts decoder
helpers remained from September 29 despite OpenCPN being closed. The current
user INI remained intact; at that point no launch or cleanup occurred. SCRUM-223
tracks separately reviewed normal local shutdown for these exact orphan
processes outside an active commissioning transaction. Existing closed-process
checks remain mandatory.

SCRUM-223's first native maintenance attempt at commit
`363fee0cc37a70b09d41799638b0ab7935041411` is preserved as a failure: 24 of 25
reports passed, while the isolated cold-helper workflow fixture rejected its
CRLF source substitution before native fake transport. The corrected test-only
source at `20765cf374da5a1ab47dff0b3b427cd24f595455` passes all 10 native jobs
and all 26 reports in [run 36919418416](https://github.com/ThereptileII/Work/actions/runs/36919418416);
the 30,731-byte artifact digest and ZIP integrity were verified. The 33-check
workflow substitutes process, ACL, transport and ledger observations. Separate
checks include 45 native pipe-suite checks (10 disposable fake-pipe cases) and
23 native disposable-ledger ACL checks; none invokes the vendor helper or
accesses the boat. See [SCRUM-223
native evidence](evidence/scrum-223-native-cold-helper.json). On the boat, the
reviewed capture covered two helpers, 2,031 profile entries and three plugin
trees with 80, 40 and 80 entries. The two fixed one-shot local requests
completed at 20:25:28Z; independent verification at 20:25:56Z confirmed both
retained process identities and exact pipe peers, then measured exit code 0 for
each. The post-close inventory found zero OpenCPN processes and zero chart
helpers. The profile was unchanged at SHA-256
`d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6`, and the
stock executable remained at its reviewed SHA-256. All three remote services
were running. There was no application launch, profile change, plugin move,
force termination, reboot or physical output. The required post-close SCRUM-222
cold Capture succeeded and its exact record was downloaded and hash-checked.
Independent exact-key review and native Complete now pass: all 19 changes are
preserved, including the current display and saved-position state, without
attributing their origin or treating saved coordinates as live data. The completed
record is hash-bound to the unchanged 21,492-byte profile and predecessor; see
[actual cold preservation evidence](evidence/scrum-222-boat-cold-preservation.json).
Fresh source/plugin review and input-only commissioning remain mandatory before
remote application launch. Installation, retirement of the old copy and boat
acceptance remain pending.

The frozen full candidate is `b48bf4a8f12f98c459d79aa08805141ac49e0306`
(local mapped commit `0053ed507b03c79877fd6bb96c0d761fb9f68ac9`). Its narrow
native tool-facts and version-probe regression passed under both Windows
PowerShell 5.1 and PowerShell 7 in [run 36900544171](https://github.com/ThereptileII/Work/actions/runs/36900544171);
both artifacts were independently verified. The probe covers real x86 tool
facts, stream stability, refusals and restored verification. The same-commit
Linux and Windows contract jobs now pass (90 and 87 CTest cases respectively,
plus the separately recorded Python, native and repeated lifecycle checks).
The [Windows tooling run](https://github.com/ThereptileII/Work/actions/runs/36901026982)
passes all 10 jobs; verified actual COM shortcut reports pass 238 checks on
each Windows host architecture. See the [contract evidence](evidence/scrum-217-b48-full-contracts.json)
and [shortcut evidence](evidence/scrum-215-b48-native-shortcuts.json).
The [integrated](https://github.com/ThereptileII/Work/actions/runs/36901026915)
run remains in progress; two native Windows prototype jobs in [run
36901029053](https://github.com/ThereptileII/Work/actions/runs/36901029053) were
cancelled during build/regressions before downstream UI gates. No full candidate
acceptance is established. This focused pass does not qualify the full dependency
producers, dependency reuse, application, package or boat.
See [native stream-pass evidence](evidence/scrum-217-native-stream-pass.json).

Downloaded same-commit recovery evidence now verifies 21 native maintenance
suite reports, 51 guarded-broker checks and 56 Prepare/Arm/Collect checks.
These use disposable files and marker processes, not the boat or OpenCPN UI;
see [recovery tooling evidence](evidence/scrum-17-b48-recovery-tooling.json).
The pristine upstream Windows build passes 60/60 tests. The pristine Linux
comparison builds but reproduces its two documented upstream failures (64/66);
it is explicitly not an all-tests-pass result. Integrated application tests
still must pass without exceptions. See [upstream baseline evidence](evidence/scrum-217-b48-upstream-baselines.json).

An earlier frozen candidate, `141265ea2fa8796c9570065034f31575292ff49e`
([integrated run](https://github.com/ThereptileII/Work/actions/runs/36892709388),
[prototype run](https://github.com/ThereptileII/Work/actions/runs/36892710519)),
was **not qualified**. Windows contracts failed on nonportable path fixtures;
the dependent integrated Windows application job was skipped. The prototype
composition job separately failed before OpenSSL compilation because its child
PowerShell could not resolve `Get-FileHash`. These remain historical SCRUM-217
integration failures, not acceptance of the certificate repair or product.

The isolated receipt-fixture correction at
`899af50a1a4623a08f71d199d146a1cecd1959c2`
([focused run](https://github.com/ThereptileII/Work/actions/runs/36894286613))
passes the 15 receipt, 9 reuse and 4 staging tests on native Windows. The
13-test evidence suite failed at OpenSSL retained-log summary parsing;
Linux passed all 41 focused tests. A subsequent CRLF parsing repair at
`5a0015a2911a0b643c761df9657e6a3c6ab31e7c`
([focused run](https://github.com/ThereptileII/Work/actions/runs/36895232260))
passes all 42 tests on both native Windows and Linux: receipt 15, evidence 14,
reuse 9 and staging 4. Downloaded artifacts were independently size/hash/ZIP
checked. Original byte hashes and contradictory-failure rejection remain
enforced. The separate PowerShell producer repair progresses past the original
missing-module failure. Its diagnostic run at
`d6b05c8554c671e7504b797d15cc33404e226495`
([native run](https://github.com/ThereptileII/Work/actions/runs/36898370390))
passes Windows PowerShell 5.1 but fails PowerShell 7: only NMAKE's merged
stdout/stderr ordering changes the recorded first line and output hash. The
retained captured/observed records have identical executable identities,
environment and exit codes. Bounded separate-stream capture was subsequently
implemented and passed its real-process regression under both interpreters;
strict identity and output comparisons remain required. See the
[stream-order failure evidence](evidence/scrum-217-native-stream-order-failure.json)
and [subsequent narrow pass](evidence/scrum-217-native-stream-pass.json).
The narrow interpreter gates subsequently passed on the frozen b48 candidate;
its broader native qualification is still in progress as noted above.
No new boat deployment or removal of older installations has occurred.

The published candidate `f1e2cde8fcbf92826d648007b267cc0f5320aa55`
([integrated run](https://github.com/ThereptileII/Work/actions/runs/36875827855))
is not qualified: its Linux user-flow test stopped at the Create waypoint
Name field, which measured 46 pixels against the unchanged 48-pixel touch
minimum. The integrated build, 146 unit tests and preceding mode, navigation,
route-progress, marine-input and object checks passed; later chart, endurance
and fixture-free Linux gates did not run. All three native application jobs also failed at curl test certificate
generation (SCRUM-221); their earlier tooling successes do not qualify the application.

A later bounded Windows certificate probe passed at [run 36891804473](https://github.com/ThereptileII/Work/actions/runs/36891804473),
job `110468992463`: the locked original reproduced the expected filename failure,
the pipe-free helper passed 5/5 focused tests, and CA/host generation took
717 ms. Certificate, key and chain checks also passed with host OpenSSL 3.6.4. The
corresponding-source reproduction passed. This is not pinned OpenSSL 3.5.9,
full curl/application qualification, or boat acceptance; the three application
job failures remain part of the unqualified candidate history. See the
[SCRUM-221 evidence](evidence/scrum-221-native-certificate-pass.json).

SCRUM-220 fixes the shared editor minimum without changing the prototype
Preferences token: single-line editor heights are 48/52/56 DIP at 100/125/150%
interface scale. Local implementation `8a938eccfe8ca05392d6ebe5e404ad1f702a76e5`
has passed a real wxGTK modal component probe and an integrated Linux build,
install and unchanged eight-group waypoint/route interaction smoke. Both Name
fields measure 488×48 and navigation-database persistence checks pass. Exact
replacement full-suite Windows/boat evidence remains required. See the
[regression review](design/reviews/scrum-220-waypoint-touch-regression.md).
The boat still has no new SKAGER replacement installed; old versions and
recovery backups remain intact pending a known-good installation.

The replacement integration worktree retains the reviewed Display preferences,
1920×1080 geometry and native CMake extraction repair (SCRUM-216/213/219), then
adds the touch repair and explicit same-job dependency reuse (SCRUM-220/217).
The reuse receipt, producer evidence, tamper refusal and staging suites pass
41 focused cases locally; Linux and native Windows CI now invoke them. Reuse
is restricted to a successful fixture build in the same job and never replaces
downstream product/security/installer tests. The SCRUM-221 helper-level native
certificate probe now passes; the pinned producer/full curl suite still needs
integration validation.
Complete replacement Linux, native Windows and boat gates remain required.
No website work, boat installation or physical hardware output is included.

The corrected frozen candidate `e9737d6bad9f3eb3db71877bce0816f59c62b0e7`
has not qualified. Its [prototype run](https://github.com/ThereptileII/Work/actions/runs/36850600318)
reached terminal cancellation in both native build jobs. The downloaded,
hash-verified composition artifact shows OpenSSL completed 347 files / 4,283
tests and zlib passed 13 tests, then curl preparation produced no configure
output before the 180-minute job limit. This is a separate failure from the
earlier zlib URL mismatch below. SCRUM-219 investigates the exact preparation
boundary in a disposable native probe; no root cause or replacement acceptance
is claimed. See [retained evidence](evidence/scrum-219-curl-preparation-stall.md).
A short native probe now reproduces a stall in Windows system tar on the exact
locked curl XZ archive; CMake extracts those same bytes in 936 ms. SCRUM-219
has now passed its corrected native extraction probe in 767 ms, with all
4,406 source paths and SHA-256s identical to the locked archive. The focused
repair is validated; full-product acceptance is not inferred from that result.
The earlier e973 full integrated Linux job has passed, including its real
three-hour endurance and fixture-free build. Its native job remains separately
under observation; this is not acceptance of the new revision. No
replacement was installed on the boat and obsolete copies remain intact until
the new installation and recovery modes are known-good.

The zlib source URL correction is published as
`c85db9d308c4a0ae1bd527bb7634b559c0fdb8e6` (local
`5957f14a26675b29950423556a9813a6faa36c10`). Its
[native source-only run](https://github.com/ThereptileII/Work/actions/runs/36843124074),
job `110306680157`, passed. The downloaded artifact was independently checked
against GitHub's artifact digest; its source record confirms the expected and
observed zlib archive are both 1,502,830 bytes with SHA-256
`bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16`.
The [combined native run](https://github.com/ThereptileII/Work/actions/runs/36843282070)
and [prototype run](https://github.com/ThereptileII/Work/actions/runs/36843282641)
are for that same exact commit. The prototype run's native composition/object-flow
jobs stopped at curl's zlib manifest identity guard; the verified failure artifact
is from composition job `110307179535`, not the combined run. Its zlib producer
recorded the reviewed GitHub release URL, while two curl consumers still
required the earlier zlib.net URL. The downloaded native artifact was
hash-checked before review; OpenSSL and zlib
built, and zlib's 13 tests passed before this guard. The subsequent local
consumer correction aligns the URL in the PowerShell builder, Python package
verifier and notice provenance while retaining archive/hash/size/signature
locks. A self-contained producer/consumer guard now runs before the costly
native dependency build, while all later real build checks remain in place.
Its actual producer-manifest, focused PowerShell 7 and Python checks pass; corrected
native application/package qualification has not run. See the
[consumer closure record](evidence/scrum-209-zlib-consumer-closure-20261001.json).
The combined run also reached a terminal native-job failure; that separate
failure is not attributed from the reviewed prototype artifact. These runs do
not establish a qualified fixture-free application/package,
installer, boat or release result. The earlier source failure/pass record is in
[zlib source URL evidence](evidence/scrum-209-zlib-source-url-correction-20261001.json).

The preceding prototype candidate
`7464a00d1071cd019c51821b7c858f5b1f989837` failed its native source guard in
[run 36841633753](https://github.com/ThereptileII/Work/actions/runs/36841633753),
job `110301869566`, before application testing. Its retained source report
records expected size/hash 1,502,830 bytes / `bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16`
and observed size/hash 11,975 bytes /
`d5b43f44bbe15f74bae56414ae917f7b2ed13ab0df81bff0b845a100efda5681`.
The response body was not retained or characterized. No cause beyond the
observed identity mismatch is claimed.

The earlier frozen security/display candidate
`99b0a58b08c5742a7e31341071eabd0124530e4a` (local
`325ccd3596d368a6aaf0787bdf4b4e673b13e881`) also had an incomplete
[full native/Linux run](https://github.com/ThereptileII/Work/actions/runs/36832799208)
and [prototype run](https://github.com/ThereptileII/Work/actions/runs/36832798681).
Its prototype object-flow job failed the zlib archive lock check; that older
artifact did not expose the observed size or hash. Keep this distinct from the
later `7464a00` report above. The current corrected runs remain partial results.
Native portable contracts pass 86/86; the actual Windows PowerShell 5.1
filesystem and historical shortcut suites pass in both bitnesses. The earlier
`322aecdf8cd51fd6ea951ae0caf6b0bde5cefaba` candidate failed because Windows
translated a disposable peer marker to CRLF; the replacement writes canonical
bytes and preserves the strict consumer check.

The user authorized leaving the tested SKAGER build on the boat PC and retiring
obsolete OpenNav X copies and shortcuts (SCRUM-17). A read-only inventory
confirmed the supported stock OpenCPN hash, the accepted Beta 1 installed
generation, older owned generations and duplicate historical Start-menu groups.
No application, profile, chart or recovery file has been removed. Replacement
qualification and boat checks precede retirement; recovery material is retained.
The [fresh cold backup](evidence/skager-boat-cleanup-readiness-20261001.json)
verified 2,123 application and 2,008 profile files against the original bytes;
it is readiness evidence, and no cleanup or software install has occurred.

SCRUM-215 adds versioned SKAGER shortcuts and installer/maintenance captions in
a separate subsequent increment. Both historical layouts remain supported for
rollback. Its subsequent source is published as
`010fe057dcbe4aa85d14f8734d2cd1113646cdd1` (local `6d640e1`), with every
mapped blob/mode/type verified. All nine jobs in the
[short native tooling run](https://github.com/ThereptileII/Work/actions/runs/36837038651)
pass, including 238 actual COM/historical-lifecycle checks in each PowerShell
bitness. The rollback handoff removes verified SKAGER links before exposing
the old maintenance engine, preventing orphan links after an interrupted
rollback. Full application packaging, NSIS lifecycle and boat checks remain
required; SCRUM-215 is Testing. This is not part of the frozen `99b0a58`
candidate and its tests do not qualify that different source.

The subsequent SCRUM-214 Vessel Preferences form is integrated locally at
`2174501` from `d534649`. Its five native fields save the vessel display name,
explicit model assumptions and edited stock S-52 safety depth. Blank chart depth
keeps the current chart value; missing model inputs stay unconfigured. Local
focused results are 124 component checks with 12 captures, seven persistence
checks and the actual OpenCPN integration syntax check. The Night capture was
reviewed and an independent preservation review found no defect. A subsequent
isolated Linux integration configured embedded source commit `ab485998` (while
the worktree advanced to documentation-only `f7f5b9f`). Its pinned-source
application, settings-driver and test targets built 720/720; the remaining
install dependencies built 287/287. The integrated settings tests passed 7/7,
peer-unavailable tests 4/4 and isolated 1280×800 component driver 124 checks.
The installed developer build passed its loader/resource self-test and XNav,
Legacy and Safe startup plus IPC quit. One concurrent Safe capture was killed
under local memory pressure while Legacy launched; its sequential rerun passed.
See the [local integration record](evidence/scrum-214-vessel-linux-integration-20261001.json).
This is development evidence for the embedded source, not a new candidate,
native Windows, boat or release qualification.

The subsequent form-spacing refinement and capture-harness update pass a
separate 130-check Linux component run with 12 captures. Its five input bounds
and Save button match the independent prototype rectangles exactly in that
component environment; screenshot review found the small paired captions
visible and unclipped. The revised Linux capture harness passed four responsive
cases against the earlier installed `ab485998` developer app, including lower
Save visibility and vessel-profile entry. That run validates the harness flow,
not the refined geometry in an integrated app. See the
[refinement record](evidence/scrum-214-vessel-form-refinement-20261001.json).

A fresh read-only boat inspection at 08:49 UTC confirmed no OpenCPN process,
the exact supported stock 5.12.4 x86 executable, and running SSH, Tailscale and
RustDesk services. Windows currently reports 1920×1080. This is readiness
inventory, not product, physical-touch or new-display acceptance. No old
software or user navigation data has been removed.

## Display coverage and current priority — 2026-10-01

The user added **1920×1080** as a supported target alongside the primary
**1280×800** prototype layout (SCRUM-213). The native DPI harness now exercises
the large fullscreen chart, four primary rail values, alert accessibility and
Passage/Traffic/Settings/Alerts panels, then verifies return to 1280×800.
Existing 100/125/150% DPI checks remain. Source syntax checks pass; native
captures and boat-display acceptance are still pending. See the
[qualification record](design/reviews/1920x1080-qualification.md).

Website development waits until the software is complete and tested on the
boat PC, following the user's latest sequencing instruction. Jira remains the
sole backlog. Read-only boat readiness confirmed connectivity and the supported
stock executable; it is not acceptance of the current development candidate.

## Security integration in progress — 2026-09-30

The integrated source was published as `7a7392cc6ef15594db6ccd6c330e615f2db33f5d`
(local `b3ee47884b8455adee11f2915bfe95e85d7848db`). All 1,337 mapped
entries were checked by blob hash, mode and type before publication; eight
unrelated entries and seven unrelated branches were preserved. Native/Linux
[run 36818668852](https://github.com/ThereptileII/Work/actions/runs/36818668852)
and prototype [run 36818667685](https://github.com/ThereptileII/Work/actions/runs/36818667685)
failed their native dependency build before application qualification:
zlib build logging reported a Windows file-sharing violation after MSVC setup.
The exact locked file was not identified by that log. The notice-byte regression
gate passed on native Windows and Linux; application packaging/installer
acceptance is not established. A follow-up removes repeated batch log opens
and records each build stage, without retrying or suppressing failures. SCRUM-208/211 are Testing, not Done. SCRUM-209 is In Progress after
review found that preserved plugin/addition files could restore legacy TLS DLLs
into an otherwise maintained candidate. Candidate-only refusal now preserves
those original files and the previous installation. It also audits normal and
delay PE imports after preserving plugins, including transitive local modules.
Linux PowerShell fixtures pass; native install/update/repair refusal and rollback
tests have been added but require exact-candidate native execution.

SCRUM-212 now implements an explicit unavailable local-peer boundary instead
of inventing a new pairing protocol. Inbound application startup and outbound
UI/model paths are being contained consistently across XNav, Legacy and Safe.
Stock OpenCPN and its isolated server-model tests remain unchanged. This is a
subsequent change requiring separate qualification; it has not been deployed.

At local `f5efdb3`, the reviewed nine-patch Linux integration passes 140/140
tests (22.93 seconds), all five mode-cycle startups and actual CLI peer-key
refusal checks. Later installer/display test increments require their own
candidate qualification; these results do not imply native or boat acceptance.

SCRUM-208/209/211 remain launch blockers. The maintained Windows dependency
chain now has separate source builders for OpenSSL 3.5.9, zlib 1.3.2 and curl
8.22.0, joined only in the disposable integration build. Source hashes, licenses,
manifest consistency, installed DLL hashes and the absence of legacy OpenSSL
runtimes are package gates. zlib's generated wrapper passed 14 upstream Linux
tests; PowerShell manifest interoperability and tamper rejection passed. These
checks do not qualify the native Windows binaries or installer lifecycle.

The actual pinned Downloader accepted an untrusted owned test certificate.
The integration patch now requires verified HTTPS for public catalog/plugin
downloads, rejects insecure redirects, stages path downloads and propagates
network failure before plugin extraction. Its real Linux transport harness
passes 12 cases, including both GET/HEAD rejection and stream exceptions.
The actual WXCURL harness now passes 13 Linux cases, and the native trust gate
covers both download implementations. The peer response buffer and allocation
guards pass exact-source tests. Peer authentication/credential remediation is
tracked separately as SCRUM-212; no complete application TLS remediation is claimed. See the corresponding
architecture and evidence records.

The combined eight-patch integration at `3cb637b` passes its Linux build and
136/136 integrated tests (19.66 seconds). The focused Online AIS pointer flow
passes after the semantic capture wait correction `8bdb445`. These are local
development results, not Windows qualification.

The earlier published f752dae→5652c01 candidate remains retained negative evidence. Its prototype capture
failed looking up the Online AIS Enabled control. Downloaded, hash-verified
evidence shows a stale pre-transition diagnostic snapshot; the actual button
is fully visible. The bounded semantic wait correction retains all assertions.
The full Linux run, including three-hour elapsed trip, passed. The full native
run passed both 128-test suites, OpenSSL upstream tests, DPI/touch and charts,
but recovery packaging failed its exact OpenSSL notice-hash check. Installer
and native endurance were skipped. A disposable Git checkout reproduced the
notice failure with `core.autocrlf=true`. Commit `ba30f9b` preserves third-party
notice bytes using Git attributes; the regression check confirms the original
OpenSSL/curl/zlib license hashes and retains a failing unprotected control.
The package hash requirement remains unchanged. The changes above are a subsequent integration
candidate and require their own exact-revision results. These results do not qualify a boat deployment or public release.

## SCRUM-208 / SCRUM-40 / SCRUM-98 bounded candidate evidence — 2026-09-30

Frozen local revision `f752daeb0fc07e2abb878c0c43a4b64760f22101` has a
published candidate `5652c01aa7dc450cdfc081a981945b0cc6f6905d`. The local
integrated suite passes 136/136, and the Auth0 PostgreSQL CI job
`110049568910` in run [36762862390](https://github.com/ThereptileII/Work/actions/runs/36762862390)
passes all 31 tests, including fresh, reverse and reapplied migrations. The
full native run [36762862383](https://github.com/ThereptileII/Work/actions/runs/36762862383)
and prototype run [36762860209](https://github.com/ThereptileII/Work/actions/runs/36762860209)
are not accepted: prototype composition failed at the Enabled-control lookup,
and the full native run failed its notice-hash packaging gate. The three-hour
Linux run and native prototype object-flow job passed. No release or native
acceptance is inferred.

The source-publication proof maps all 1,289 prepublication entries and
preserves eight unrelated entries (proof SHA-256
`f3fe14b1c93a06d71096a1774bb2a9dd7f3ac21aebb5a1ca6d0fdc84614f897b`). The
PowerShell interop directory is an inert fixture-only check and does not qualify
installed or packaged DLLs. SCRUM-208 remains a Highest security launch blocker.
SCRUM-209's legacy curl/OpenSSL closure remains open, and SCRUM-211's
certificate-validation bypass finding is an additional open security blocker.

The earlier remote run [36742212299](https://github.com/ThereptileII/Work/actions/runs/36742212299)
on `19cf4f70dc4d74789a44b4abe2302241e193d72d` passed its installer step but
failed the retained DPI step; native endurance was skipped. That run's own log
does not print the active DPI scale, so its full-run scale remains unconfirmed
without artifact `11115071251`; the separate retained prototype evidence must
not be transferred to it. Its Linux integrated job subsequently passed, including
the real three-hour elapsed trip; the native failure still blocks acceptance.

## Next combined native candidate — 2026-09-30

The compact rail now fits measured number/unit bounds above status metadata
(`58ce8b2`, `3092417`), addressing the retained 150% overlap. The primary
prototype sizes remain where they fit. Native screenshots are still required;
local syntax validation does not qualify the design.

The OpenSSL source-build boundary is merged (`3cbd7e5`, `bb294f3`). Package
validation (`789cdbf`, `fbc991a`) now checks the actual Windows build manifest,
installed and final packaged DLL hashes/architecture, source archive and
notices. It includes the unchanged verified source tar in corresponding source.
Focused checks pass: 8 OpenSSL package, 11 source archive and 6 AIS probe cases.
The source-build compiler/upstream-test and native regression gates are pending.

SCRUM-209 separately tracks the confirmed legacy curl closure: the current
dependency bundle carries libcurl 7.58.0 importing OpenSSL 1.0.2n. Its maintained
replacement, zlib and trust-policy qualification remain launch blockers.
Updating the major-version-3 libraries alone does not close that issue.

## Account session foundation — 2026-09-30

SCRUM-40's Auth0 login/session foundation is merged at `99f6a71` / `745cc09`.
Exact merged revision `4bd4c36dae43b9b2ab9e5039c687f531b6c605d5` passes
31/31 tests on isolated PostgreSQL 18.6: 11 database-integrity and 20 account
cases. The check includes concurrent one-time callbacks, signature and claim
validation, redirect refusal, bounded responses, session rotation, CSRF logout,
and fresh/reverse/reapplied migrations. The disposable database was stopped
and removed. [Evidence](evidence/scrum-40-auth-local.json) records exact logs;
the [session contract](architecture/auth0-session-foundation.md) records the
boundary. CI now includes both suites, but publication/native and staging
acceptance remain separate. Authentication stays disabled by default; real
tenant, recovery/MFA, abuse controls, privacy/security review and deployment
are not complete.

## Native repairs and security dependency review — 2026-09-30

The pointer-scroll activation repair is merged at `61cdd4c`: it waits for the
actual foreground window within the existing deadline, then rechecks geometry,
window identity and pointer obstruction. Focused delayed/denied activation,
overlay and movement cases pass; native Windows verification remains open.
The horizon row-stretch correction is merged at `fa9a595`. Its isolated Linux
component passes 232 checks, including Return action-identity cancellation.
The strict Linux HTML comparison still fails on platform-specific row height;
the corrected 53-pixel native height matches the retained Windows reference,
but only a fresh Windows comparison can qualify it. See
[retained component evidence](evidence/scrum-100-row-stretch-local.json).

SCRUM-208 is a Highest security launch blocker: retained Windows dependency
DLLs identify as OpenSSL 3.0.5, with no reviewed backport evidence. Official
OpenSSL 3.5.9 source has passed published checksum and isolated signature
verification; a source-built Win32 integration replacement is in progress.
No installed/boat DLLs have been changed. Native TLS, package/import, plugin,
installer and exact-source qualification remain required. The existing stock
OpenCPN compatibility allowlist has not been expanded.

## Active native candidate and database foundation — 2026-09-30

Local `f2152da67f9e6525fd0d5dcc28d127266ccca879` is published as
`19cf4f70dc4d74789a44b4abe2302241e193d72d`. Its [full native/Linux run](https://github.com/ThereptileII/Work/actions/runs/36742212299)
passes the standalone contract suites: 86/86 Windows and 89/89 Linux.
The [prototype run](https://github.com/ThereptileII/Work/actions/runs/36742210448)
passes the native footer component (198 checks), but its strict horizon
comparison fails on 45-pixel event controls versus the HTML grid row's 53 pixels.
The retained 150% DPI suite also fails on immediate foreground activation
observation before scrolling; the pointer hit is inside the intended viewport.
Both verified archives and [exact negative evidence](evidence/scrum-98-native-19cf-negative.json)
are retained. Neither run has overall acceptance. The integrated Linux build previously passed 136/136
upstream tests. Those counts describe different test suites, not a combined
release result. The branch/commit concurrency correction `267c72d` is local and
awaits publication; it does not undo the preceding run's automatic cancellation.

The reviewed SCRUM-94 database schema and CI definition are merged through
`d0105a1`. Its exact local PostgreSQL 18.6 loopback-TCP check passes all 11
integrity cases, migration drift, fresh migration, reverse to 0001, and reapply
through 0003. The disposable cluster was stopped and removed. See
[exact schema evidence](evidence/scrum-94-schema-local.json) and
[database contract](architecture/web-database-foundation.md). CI, backups,
restore and staging gates remain open. SCRUM-40 now owns the bounded Auth0
login/session implementation; no public service has been deployed.

## Next native verification candidate — 2026-09-30

The selected fullscreen test correction is implemented in `882d497`; the
unchanged real pointer checks now get additional scroll-occlusion evidence
(`1634d5d`). Full passage maps only to the supplied Day Passage reference
(`b7e768c`); reprocessing the verified native captures produces all 34 exact
comparison sets, still pending visual acceptance.

The shared custom button now handles Return before wxMSW dialog navigation
without claiming Tab or arrow keys (`c270502`). Its isolated Linux component
passes 164 checks including actual pointer, normal/held Enter, Space, disabled
input and Tab to a real destination. Initial local failures were a Wayland/Xvfb
environment mismatch, retained separately; the corrected diagnostic uses the
same X11 environment as the existing harness. See [keyboard evidence](evidence/scrum-98-footer-keyboard-local.json).
A new exact native Windows run is required. The 150% Preferences obstruction
is instrumented, not yet diagnosed or repaired; no acceptance is inferred.

## SCRUM-98 / SCRUM-100 replacement native evidence — 2026-09-30

Exact local `491c29b` maps to remote `11a97233fa7ad4d1b83e346642c9bc75dd0a30d9`.
The local integrated build/install and 136/136 tests pass. The native integrated
build and 128/128 tests pass, then actual pointer input correctly rejects
Source health: its midpoint hits the Windows taskbar clock. The harness forced
a decorated 1280×800 frame over a 1280×800 desktop's reserved taskbar area.
The selected repair uses OpenCPN's supported fullscreen startup for that exact
test desktop, retaining real input, containment and chart/layout assertions.

Two separate prototype failures remain: the footer component's Enter callback
assertion (after 185 checks), and the retained DPI suite's covered Preferences
scrolling surface at 150%. The latter completed 100/125%; the additional
prototype-only DPI suite passed all three scales, which does not override the
failed suite. Their causes are not inferred from the navigation taskbar result.
The Full passage screenshot also needs its existing HTML Passage state mapped
into comparison. Original HTML and comparison strictness remain unchanged.

[Verified archives and exact negative evidence](evidence/scrum-98-native-11a97233-negative.json)
retain all three failures. Prototype run
[36726644584](https://github.com/ThereptileII/Work/actions/runs/36726644584)
finished with six successful and two failed jobs; full run
[36726657879](https://github.com/ThereptileII/Work/actions/runs/36726657879)
had not completed its elapsed-time endurance gate when the later push for exact
candidate `19cf4f70dc4d74789a44b4abe2302241e193d72d` reused the branch-only
concurrency group and automatically cancelled that run's remaining endurance
step. This workflow now scopes concurrency by branch and exact commit, while
still cancelling duplicate runs of the same revision. Future validation must
observe a separate candidate run through its terminal endurance result; the
original intermittent exit-255 failure remains open. No release or boat
acceptance is claimed.

SCRUM-31's [web architecture decision](architecture/public-beta-web-commerce.md)
is complete at `95d5a12`. SCRUM-94 now owns the next bounded database-schema
increment; authentication, payment services, deployment, backups and staging
remain separate implementation/acceptance work. Public access stays closed.


## SCRUM-98 native negative evidence — 2026-09-30

The exact local revision `8eaaba6a3268f61c5d7cd89bd8825f78464c6156` was published as remote revision `32174cfb716a77c75b03efa0ac368d3e04ce9231`. Full CI run [36615777880](https://github.com/ThereptileII/Work/actions/runs/36615777880) recorded 13 successful jobs, two failures and one skipped job. Linux job 109568284851 failed to compile `TestEarlyStartupTrace.h:100` because the fortified write triggered `warn_unused_result` under `-Werror`.

Windows job 109570349159 built and passed all 128/128 unit tests, then `tools/smoke-navigation.py` failed in `footer_health_action` to find a fully visible `Source health` pointer target. The screenshot shows the target rendered; obstruction, parent geometry, and hit-test rejection are unknown, so the root cause is unproven. Release remained skipped and the original exit-255 failure remains open.

The focused compile fix `84c7c13` covers 11 terminal-only scenarios per policy; no logs were retained and native verification remains pending, so it is not a broad pass. Native artifact evidence, including the 1,175,724-byte ZIP with SHA-256 `2d5e7336b669a243cc2f2dd176495ecc679b43d81dc2fad32a7f35b32b6f9027`, CRC/path verification, and scoped publication proof, is recorded in [SCRUM-98 native evidence](evidence/scrum-98-native-32174-negative.json). No accepted or Done claim is made.

## Current stage: Jira-driven paid public beta

[Navigare / SCRUM](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole backlog. The complete [public-beta contract](public-beta-contract.md)
now governs the application, distribution, website, commerce, support and
release gates. The approved public identity is SKAGER / SKAGER App; domain
control and legal clearance remain separate gates in SCRUM-89. Public payment
and download access remain closed pending the final human GO/NO-GO review.

The reviewed Windows marker-reader and horizon-header repairs are committed in
`2345abe`, combined with Linux-only, opt-in startup observation in `4b1f301`.
The actual isolated diagnostic application builds successfully; all three
observation-policy suites pass eleven scenarios each. One marked-file preview
run records 44 captures and seven successful process exits, with complete
initialization/parser/log stages for every process. This verifies the diagnostic
but does **not** close the original intermittent exit-255 failure. The source
revision, precommit embedded build label and binary hash are recorded separately
in [diagnostic evidence](evidence/scrum-98-startup-observation-local.json).
The [trace contract](startup-observation.md) preserves normal product and native
Windows entry behavior. Fresh native Windows reader, component, DPI and
installer gates remain required; SCRUM-98 stays In Progress and SCRUM-100 stays
Testing. Existing CI endurance runs are preserved on their own branches.

The combined portable build caught a further test-reader compilation defect
before publication: GCC 16 defaults to C++20, whose `u8string` element type
cannot concatenate directly with `std::string`. The reader now preserves the
UTF-8 bytes explicitly, with a non-ASCII path regression. Both explicit C++17
and C++20 builds pass 74 marker checks, and all 88 configured portable suites
pass. Native sharing semantics remain pending; no failed candidate ref was
published. See [failure and repair evidence](evidence/scrum-98-marker-utf8-local.json).

Exact replacement `ced99ac379e5622c21e48c83772b43aa7f2e8df6`
(equivalent local `3fb6635`) is **not qualified**. Its Windows portable suite
passes 84 of 85 tests, then fails the concurrent fixture-marker reader; a
separate passing 376-check/24-case process invocation does not waive that
failure. Test-only readers now allow Windows DELETE-handle sharing while
continuing to deny open writers; native execution remains pending. The native horizon screenshot
fixture also fails to compile because its Windows branch lacks the explicit
`wx/dcmemory.h` include; that narrow correction requires a replacement build.
See [retained marker evidence](evidence/scrum-98-marker-sharing-negative.json).

The same local source passes all 136 integrated tests and its first mode cycle,
selected-navigation interactions, recording, both route styles and public ENC
checks ([exact local summary](evidence/scrum-98-integrated-3fb6635.json)).
The portable preview then reproduces Legacy → XNav failure. The new
pre-teardown evidence observes the replacement child exiting 255 before a third
startup/window, after Legacy exits successfully. The cause remains under
early-startup investigation. One bounded process trace passes with all seven
application exits successful; that timing-altered diagnostic does not repair or
supersede the [original failure](evidence/scrum-98-preview-3fb6635-negative.json).
See the [limited diagnostic result](evidence/scrum-98-preview-3fb6635-diagnostic.json).
Follow-on pilot, production and Full passage checks stopped at the first
failure. The concrete failures remain retained; implementation status is
recorded above without transferring acceptance across revisions.

Replacement `701149f` (equivalent local `826a44f`) remains **unqualified**.
The local integrated build passes 135 tests; recording now observes the exact
1280×34 footer. Its mode cycle, selected navigation, both route styles and
public ENC software/OpenGL checks pass. The portable preview subsequently
fails Legacy → XNav after 39 captures: Legacy saves the requested mode and
exits, but no third startup is logged. The original failure remains retained;
the restart child has not yet been conclusively diagnosed. Local OpenGL uses
llvmpipe and does not qualify the boat GPU.

The same exact native candidate passes its fixture-free MSVC build but exposes a separate
Windows commissioning-fixture sharing failure while reading a child readiness
file. The fixture made its final name visible before closing the writer. The
replacement publishes checked, closed markers atomically and adds 70 portable
checks plus native exclusive-handle assertions; production restart guard code
is unchanged. The actual failed OS lock holder was not captured, and Windows
replacement remains mandatory. CI Linux also fails a Chart transition in the
mode harness; that evidence is being investigated separately. See the
[marker development record](evidence/scrum-98-marker-publication-local.json).
These failures are not waived by the passing component or local mode tests.

The terminal `701149f` prototype run has six passing/two failing jobs; its full
run has eleven passing/three failing/two skipped jobs. All 19 archives verify.
Windows integrated suites pass 127 tests; the actual status-only product probe
records zero outgoing bytes. The standalone footer fixture fails on an entirely
blank image, while the actual product footer and coastline render. Both footer
and horizon fixtures now explicitly size their parent host and record native
parent/child/hit geometry before the original assertions; Linux passes
153/208 checks. The retained 150% DPI test also used an 80-DIP rail minimum that
conflicts with the immutable compact layout's 72-DIP rule. Its replacement uses
that exact breakpoint; four-value visibility, clipping, touch and alarm checks
remain. The rest of the 150% suite remains unexecuted. See
[exact terminal evidence](evidence/prototype-native-701149-development.json),
[host-fixture evidence](evidence/scrum-99-component-host-local.json) and the
[compact-layout oracle](evidence/scrum-98-dpi-compact-oracle.json).

One instrumented repeat of the portable preview passes all seven process exits
and 44 captures; the original restart failure remains unresolved. The preview
harness now retains per-launch output and pre-teardown process/window evidence.
No retry is added and no successful repeat is presented as a product repair.

Candidate `88b7bbc` fails qualification: both prototype MSVC jobs stop at the
new footer component's missing console entry point, and Linux recording stops
on a real 1279px footer inside a 1280px frame. Its 135 integrated Linux cases
and 83 Windows portable suites pass; native component captures were not reached.
The narrow entry-point repair follows the other component executables and
passes 98 local checks/ten captures. Independent wxAUI probes reproduce the
fixed-dock trailing spacer; the replacement makes only the footer proportional
and scopes/restores XNav's zero sash metric through theme changes. The recording
gate still requires the full 1280px width. Replacement native evidence remains
mandatory. See [retained failure evidence](evidence/prototype-native-88b7bbc-development.json)
and its [terminal archive record](evidence/prototype-native-88b7bbc-final-development.json).

The same local binary passed XNav → Legacy → XNav and Safe startup, then failed
its final normal startup once. One instrumented repeat passed; the original
failure remains unresolved/intermittent. Mode gates now retain every launch's
stdout/stderr, early exits and failure window/process inventory. No recovery
fix is inferred from the repeat. The retained prior installer also exposed
fixture rejection being preempted by the output-policy rejection. Both guards
remain mandatory; fixture identity is now checked first with strict boolean
typing, and 48 actual PowerShell validator checks pass. Full installed lifecycle
and replacement Linux/Windows gates remain open.

SCRUM-100 implements the prototype horizon's measured fractional columns,
heading, separators, Full passage and guarded contextual actions. Its owned
model passes 81 checks, native Linux component passes 132 checks/14 captures,
and eight related regressions pass. Day/Dusk/Night reference/current/diff
retain visible Linux text differences. Fresh/stale position controls NOW;
route revision and local AIS identity are revalidated at activation. New
integrated pointer checks remain pending with the combined source. Windows,
DPI and boat conformance are open; no Navigation screen is accepted. See the
[horizon review](design/reviews/scrum-100-horizon.md) and
[development evidence](evidence/scrum-100-horizon-local.json).

SCRUM-19 now enforces status-only XNav equipment output at the final transport
boundary, independent of saved permissions. Passive pilot feedback remains
available. The separate non-installable command-test build is restricted to a
verified local TCP peer. Package/install/update checks require an executed
status-only declaration; exact historical rollback/repair remain recovery-only
and explicitly unqualified. Portable denial checks and 27 actual installer
policy checks pass. Linux replacement passes 134 integrated tests, the actual
product probe records zero outgoing bytes despite saved permission and fresh
compatible feedback, and 94 component checks produce nine captures. Visual
review found and corrected a misleading enable-control promise: the product
now visibly says status-only / equipment control unavailable in Day and Night.
See [local policy and presentation evidence](evidence/scrum-19-status-only-local.json).
Native Windows replacement remains mandatory.
Stock OpenCPN/plugin output remains independent and still requires the boat
profile/plugin audit. See the [hardware boundary](st4000-beta-contract.md#public-beta-output-restriction--scrum-19).

Prototype conformance, chart presentation and Online AIS remain in progress
under SCRUM-14/15/16. SCRUM-19 addresses unqualified XNav physical output;
SCRUM-98 tracks the native installer/chart and DPI gate repairs. SCRUM-97 has
committed the Jira workflow and project-goal changes, but its required complete
issue-to-acceptance cycle remains open. No issue is accepted from compilation
or component checks alone.

SCRUM-99 replaces the unrelated navigation footer with the native HTML status
bar: exact three-group composition, owned position/course, navigation quality,
and a source-health action. No cross-track distance is invented; XTE stays
unavailable. Recovery remains visible through Settings → System → Interface
& recovery, plus the existing Ctrl+Shift+S shortcut. Focused Linux validation
passes 44 footer provenance assertions, 49 retained source-health assertions
and 98 component/input checks with ten actual captures. The combined source
passes all 83 portable suites. Exact reference/current/
diff crops retain Linux font-metric differences; Windows and boat conformance
remain pending. See the [footer review](design/reviews/scrum-99-status-footer.md).

The `4b9d134` native replacement passes both 126-case MSVC suites and the
fixture-free product's actual zero-output pilot probe. Its prototype DPI gate
then finds two System captions: the footer action and the page heading. The
retained 100% frame has all four rail values and eight recovery actions visible;
it is not a completed DPI gate. SCRUM-99 now exposes and tests the distinct
footer panel geometry instead of inferring it from that caption. Replacement
native runs must still pass; no layout, chart, touch or persistence check is
waived. See [retained exact native evidence](evidence/prototype-native-4b9d134-development.json).
The current combined candidate also includes the status-only wording
correction. The older local fixture rebuild was deliberately stopped before
integration to avoid qualifying an intermediate revision; it is not counted as
a passed build.

Source Health now uses the prototype disclosure sheet with independent onboard
and Online AIS status, per-measurement quality and protected AISStream key
settings. Incomplete GPS pairs and estimated readings do not appear as healthy
measured connections. See the [Source Health review](design/reviews/prototype-source-health-in-progress.md).

Native `f934f3b` / prototype run `36536723961` finishes seven jobs passed and
one failed. The full run `36536897649` finishes fourteen passed, one failed and
one skipped; publication is withheld. Linux product and fixture suites each
pass 133 tests; their MSVC counterparts each pass 125. Native evidence includes
151 own-key checks/twelve captures, 49 Source Health checks/six captures,
33 primary product captures plus four Online AIS captures, and 19 retained
preview groups/50 captures. Linux completes the actual three-hour endurance
run. Windows endurance was not reached.

The installer passes eighteen checks before applying XNav chart colors to
Legacy. Retained pixels show real coastline in the pinned Standard palette;
the independent exact Standard oracle passes. The replacement checks each
mode against its own independently defined palette and adds the actual
XNav-to-Legacy transition capture. Its portable suite increases to 54 exact
palette checks, including blank, wrong-style and wrong-light rejection, while
retaining eighteen geometry rejection checks. The DPI gate confirms the exact
Preferences scroll endpoint at 100%, then encounters outdated action captions.
The replacement uses the current drawer actions and reopens Settings for each
destination. A further correction requires actual touch scrolling and a fully
visible, exact native hit target for Battery & reserve; all nineteen observation
and touch-helper regressions pass locally. Full native replacement at 100/125/150% and the installer lifecycle
remain mandatory. No failure is waived and no boat deployment is accepted.
Six downloaded archives are hash/size/CRC/path verified. See the
[exact native development evidence](evidence/prototype-native-f934f3b-development.json).

Earlier `60a9e8b` / prototype run `36532671500` finishes seven jobs passed and
one failed; its full run `36532672143` is also terminally failed. The earlier
Preferences observation failure and all retained negative evidence remain in
[native development evidence](evidence/prototype-native-60a9e8b-development.json).
The later `f934f3b` evidence above supersedes the pending-run status, without
transferring acceptance across builds.

The equivalent local `60a9e8b` tree passes 132 product tests, 132 fixture tests,
151 AIS component checks/twelve captures, all route gestures, and eight preview
checks/44 captures without extra X11 focus helpers. The corrected late-resize
guard remains restricted to visible XNav transient surfaces. See
[local corrective evidence](evidence/prototype-transient-raise-local.json).

Native `d8316d8` / run `36530182434` passes the complete 29-state product
capture, AIS own-key checks, software ENC unit/palette checks, and development
100/125/150% probes. Seven jobs pass; the retained fixture test still selects
the duplicate Configure instruments action. All nine archives verify. No
visual conformance or deployment acceptance. See [development evidence](evidence/prototype-native-d8316d8-development.json).

Native `28b5e9c` passes 151 AIS component checks and all twelve captures,
including own-key Day/Night, cancel, replacement, failure and removal. The
separate protected import suite passes seventeen checks. All eight prototype
archives verify; six jobs pass and two fail. The product Off screenshot confirms
the action while its preceding diagnostic still says enabled. Replacement input
checks await the semantic result. The fixture Preferences transition remains
on Sensors; the replacement waits for the destination page and activates only
the exact target-owned native surface before clicking. Neither failure is waived.
The subsequent `7018c7f` full run passes both corrected 78-test portable suites;
its integrated gates finish with the failures below. No deployment qualification or visual
conformance PASS. See [native own-key evidence](evidence/ais-own-key-native-28b5e9c.json).

Full run `36528305413` / `7018c7f` passes 132 Linux and 124 MSVC unit
regressions, but qualification remains negative. Windows chart gestures pass;
the preview chooses the rail's repeated Configure instruments caption, and the
DPI context check counts floating toolbar actions as context actions. Exact
product-control and accessible-identity selectors replace those assumptions.
The local fixture preview passes eight checks/44 captures after owned-surface
focus settling. Pointer evidence traces the Linux Passage library failure to the underlying
timeline during OpenCPN’s one-second deferred resize raise. A narrow transient-
surface guard now passes the complete local route gestures; replacement
qualification is still required. Legacy/Safe retain their original raise.
See [retained negative evidence](evidence/prototype-native-7018c7f-negative.json).

The user-supplied AISStream key workflow now has explicit own-key guidance, a
masked 48px input, disabled empty/invalid Save, and no automatic enablement.
Cancel/Escape never replace a saved key; confirmed removal disables Online AIS.
An Escape defect in modal/drawer dispatch was found and fixed. Linux passes
151 AIS component checks/twelve captures, 132 product and 132 fixture tests, and retained
Pilot/Passage/Settings component regressions. All tests use fake credentials.
Native key-entry checks pass; broader replacement gates remain open. No key is distributed with builds.
See [credential UI contract](online-ais.md) and [corrective evidence](evidence/ais-own-key-ui-local.json).

The first real AISStream service probe on the boat passes: subscription confirmed,
41 peak targets, 54 accepted reports, zero rejected; clean disable/cache clear.
The user-authorized credential is verified in the interactive user's Windows
Credential Manager. A bounded local pipe crosses the SSH/desktop logon boundary
without a plaintext key file, key argument, key log or remote-access change.
Thirteen portable import checks and seventeen native checks pass. The isolated
probe uses qualified `c86c6a7`; it does not launch OpenCPN or qualify the latest
product UI. Live target selection, internet-loss/reconnect and physical display
review remain open. See [live evidence](evidence/boat-ais-live-c86c6a7.json).

Native `e83df7d` passes the nine component suites, including Radar's 64 checks
and Pilot's 77. The product composition check stops because Radar's diagnostic
screen coordinates were compared directly to client-relative HTML coordinates
(8px frame / 31px caption offset). Reviewed native pixels retain the scope.
The replacement normalizes by the measured client origin and retains the exact
one-pixel geometry bound. Its Linux corrective capture passes; replacement
Windows geometry and full qualification are still required. The completed full
run also rejects stale user-flow selectors and a Safe chart check expecting
XNav colors. Downloaded Safe pixels show stock coastline content; replacement
uses the existing pinned stock palette oracle. The fixture Display check
mistook a nested Configure instruments action for the shell action. All 21
archives verify; no failure is waived and deployment remains withheld. See
[exact negative evidence](evidence/prototype-native-e83df7d-in-progress.json).
No conformance PASS.

Radar now has a native prototype focus composition, with a fixed scope and an
independently scrolling control column. It explicitly withholds returns,
heading/range and scanner controls without a validated receive/control source.
Linux corrective captures pass 64 component checks, both integrated suites
pass 132 tests, and the actual product captures all three Radar themes. The
first layout recursion and subsequent scope/gradient differences are retained
and corrected. Review then exposed a blank Night scope during whole-page theme
rebuilds. The persistent component now passes exact fixed-palette interior
checks through the application theme cycle. See [Radar review](design/reviews/prototype-radar-in-progress.md)
and [local evidence](evidence/prototype-radar-local.json). Windows/boat pending.

Native `a1077b0` / prototype run `36520947790` passes 7/8 jobs, including
124 MSVC tests, 53 Alert checks, corrected Pilot modal repaint, software ENC
comparisons and development DPI. Full qualification `36520970749` passes
13 jobs but stops in Linux/Windows loopback pilot tests still addressing the
old full-page controls. A separate Windows preview input assertion also stops
the retained fixture run. All 22 artifacts verify; deployment is withheld.
See [negative qualification evidence](evidence/prototype-native-a1077b0-failed.json).

Updating the loopback test exposed a real clipped safety-confirmation dialog.
The correction sizes it from the application owner and wraps all text to its
available width; 77 component checks cover consent/cancel/mode confirmation.
The local actual OpenCPN TCP pilot loopback again passes all command/feedback,
timeout, stale Standby and reconnect checks, with no physical hardware. Chart,
plugin and endurance harnesses now target prototype entries, retaining their
existing navigation, performance and resource assertions. Replacement native
qualification remains mandatory; no screen or live AISStream gate is accepted.

Alerts now uses the prototype notification drawer, retaining the real chart and
existing episode-specific acknowledgement semantics. No illustrative encounter
is fabricated. Linux passes 132 integrated tests and two 53-check component
runs; the corrective run verifies removal of native grey button corners.
The replacement application pass retains 26 primary and four AIS-settings
captures. Fixture regressions pass all eight check groups and 44 captures,
including readable GPS acknowledgement amid other critical conditions and
mode/persistence restarts. Native Windows and boat conformance remain open. See the
[alert review](design/reviews/prototype-alerts-in-progress.md) and
[contract](alert-presentation-contract.md).

Pilot replacement `0ce6835` / run `36518416053` finishes 7/8 jobs; both MSVC
suites pass 124 tests. Actual native composition, software ENC/units and
100/125/150% development interactions pass. All nine archives verify. The
retained preview incorrectly assumes 800px client height in a captioned window;
replacement checks apply the exact HTML rules to measured client geometry.
Reviewed Pilot component images also expose incomplete repaint after modal
confirmation despite green interaction checks. The replacement invalidates
visible owned children after modal destruction and requires all labels in
captures. No visual or boat PASS. See
[negative native evidence](evidence/prototype-native-0ce6835-failed.json).

Native `403a4e8` / run `36515856835` passes both MSVC 120-test suites and
six component suites, including Anchor's 34 checks. Six of eight jobs pass.
The capture guard rejects the new Anchor owned surface, and retained preview
expects Instruments to cover its intentionally retained timeline. Downloaded
native evidence identifies both assertions; replacements recognize the exact
owned header signature and exact responsive timeline boundary without relaxing
occlusion or geometry. Eight archives and 109 PNG hashes verify. See
[negative evidence](evidence/prototype-native-403a4e8-failed.json).

The native prototype Autopilot drawer now replaces the old full-page controls.
An owned presentation boundary requires measured magnetic pilot feedback and
retains explicit enable/confirmation, pending/timeout and manual-Standby rules.
Linux passes 132 integrated tests and two 61-check/seven-image component runs.
The initial actual-product Day capture is incomplete; a corrected capture gate
requires title and all eight labels and awaits semantic page publication. Its
replacement retains a complete actual drawer in all three themes. Fixture
regressions pass all eight scenarios, 44 captures and mode/persistence restarts.
Exact native Windows replacement and boat review remain pending.
See [pilot review](design/reviews/prototype-pilot-in-progress.md) and
[control presentation](pilot-presentation-contract.md). No boat commands sent.

Anchor's completed Linux fixture run passes 128 tests, five shared-profile mode
phases, selected navigation input, 26 route checks, 26 object captures and all
eight preview scenarios with 44 captures. See
[retained regression evidence](evidence/prototype-anchor-regressions-local.json).

Native `314e0677` / run `36513102807` passes both 115-test MSVC suites,
113 Preferences component checks and the exact 100/125/150% development drawer
probe. Downloaded Windows pixels show complete tab labels. Six of eight jobs
pass overall: composition stops on an old diagnostic publication after Close
(the retained image shows the restored chart), and the broader fixture test
hits Windows' taskbar over System. Replacement harnesses wait for the semantic
close publication and leave desktop room around the unchanged 1280×800 capture.
No assertion is waived. Eight artifact digests/CRCs and 108 capture hashes are
retained in [negative evidence](evidence/prototype-native-314e067-failed.json).

The prototype Anchor drawer is implemented locally using owned OpenCPN watch
observations. New provenance/history and pinned-Mercator projection tests bring
the production Linux suite to 128/128 (31.17s on the corrective pass); the isolated native component
passes 34 checks and five captures, including confirmation cancellation and
stale GPS. Two integrated passes each retain 24 product captures; the second
confirms all three Anchor themes and Close restoring the chart. The fixture
build and full lifecycle/input regressions pass; native
replacement captures remain required. See the
[anchor review](design/reviews/prototype-anchor-in-progress.md) and
[data contract](anchor-presentation-contract.md). The boat is reachable with
OpenCPN closed; no prototype deployment, hardware output or visual acceptance.

The compact Preferences correction passes exact drawer rectangles against the
independent HTML at all four Linux resize checkpoints. Production and fixture
suites pass 123/123 (19.74s and 19.25s); 45 local component/product/layout PNGs
are retained across the text and workspace corrections. Windows tab painting
now uses the same natural DirectWrite metrics as its layout. Native replacement
and boat review remain required. See [correction evidence](evidence/prototype-drawer-correction-local.json).

The wider Linux fixture regression now passes with prototype navigation:
44 screen captures, all eight failure scenarios, alert acknowledgement/recovery,
and repeated XNav/Legacy/Safe restarts preserve seeded navigation/configuration.
The integrated suite passes 123/123 (19.56s). Thirty exact chart-ink checks and
23 status-summary contracts retain fail-closed coverage while replacing obsolete
Beta menu/bottom-toolbar assumptions. Updated native mode, fixture, recovery and
DPI harnesses remain pending execution; no test gate was removed. See
[regression evidence](evidence/prototype-regression-harness-local.json).

Native `e0b6a16` / run `36510429598` passes seven of eight jobs: both integrated
suites 115/115, five component suites, four exact route-ink cases and the new
100/125/150% development interaction probe. All eight artifacts, 88 native PNGs
and 144 reference PNGs verify. The main capture fails because older canonical
metadata lacks the newly checked tab selector. Reviewed pixels also show clipped
tab labels and incorrect compact Preferences bounds. Replacement code paints tab
text with matching DirectWrite metrics and uses measured compact drawer geometry;
independent geometry assertions remain strict. This is not visual acceptance.
See [negative native evidence](evidence/prototype-native-e0b6a16-failed.json).

The lower vessel-profile group now completes the measured sidebar spacing:
Settings matches all three reference positions within one raster pixel, the
32px profile action opens Vessel settings, and the short desktop layout hides
it as the HTML does. Missing vessel identity stays unavailable. Two corrective
Linux passes retain 66 images; fixture-free and fixture-enabled builds each
pass 123/123 after the SDK identifier correction. Capture policy now passes
218 checks and admits only the exact optional profile action. Windows and boat
review remain pending. See [local evidence](evidence/prototype-profile-rail-local.json).

The native replacement `72df728` / run `36509226412` stops during both MSVC
integrated builds: the SDK `small` macro collides with a compact-pilot local
variable after DirectWrite headers are included. Six other jobs pass; all eight
artifacts verify. The correction renames the variable without changing SDK
definitions or UI/data behavior. No runtime/DPI/typography success is inferred.
See [failed native build evidence](evidence/prototype-native-72df728-failed.json).

Preferences text-layout correction is ready for native replacement validation:
Windows uses fractional DirectWrite advance measurements to retain the HTML's
six-tab first row; independent capture checks all eight exact rectangles. Linux
rebuild/install passes 123/123 and the Settings component 102 checks; 12 component
and 21 actual-product images are retained. No Windows font PASS is inferred from
that Linux result. See [local evidence](evidence/prototype-preferences-text-local.json).

Run `36506983264` / `8e73ed0` is retained as a negative development run (7/8
jobs). Its new actual-DPI probe exposes the old rail losing Radar at 150%; the
compact correction follows in `8b6eab7`. The 125% Settings target is also too
small, so the replacement gate uses each measured prototype height rather than
a shared lower bound. A separate Preferences Close capture failure remains
unproven: the harness now verifies/activates the specific native input surface
and retains failed-state evidence without retrying input. All eight artifacts
and fourteen DPI images verify. See [negative evidence](evidence/prototype-native-8e73ed0-failed.json).

The replacement Windows development run `36506092362` / `f13075a` passes all
eight jobs: both integrated suites 115/115, five component suites including
102 Preferences checks, 50 native capture-guard cases, actual product captures,
object workflows and all four unchanged exact active-route ink cases. All nine
artifact hashes/CRCs, 87 native PNGs (including route images) and 120 reference
PNGs verify. This closes the fixture repaint and rail/catalog harness failures;
it does not qualify the complete product or boat. Preferences tab wrapping still
differs visibly from the Windows HTML. See [development evidence](evidence/prototype-native-f13075a.json)
and [preceding negative run](evidence/prototype-native-466d1c6-failed.json).

The compact-shell correction now follows measured HTML desktop breakpoints.
Before correction Settings was compressed at 1024×640 and absent at 853×533.
Both corrected Linux resize passes preserve the chart and all four rail values;
the second also reaches lower Preferences actions by actual scrolling. Returning
to 1280×800 restores primary geometry. There are 20 retained compact captures,
21 canonical product captures and 123/123 passing tests (18.74s).
Actual Windows DPI and boat acceptance remain pending. See
[compact evidence](evidence/prototype-compact-layout-local.json).

Additional responsive prototype references now measure the unchanged HTML at
125/150% equivalents in twelve physical 1280×800 captures. A separate native
Windows development probe will test actual DPI, visible rail controls/readings,
chart content and touch-operated Preferences/theme/Close in the fixture-free
application. The existing full release suite stays mandatory. Linux rebuild
passes 123/123 in 20.17s and all five design-contract tests. Compact-shell
migration is implemented locally; Windows execution and boat review remain pending. See
[responsive review](design/reviews/prototype-responsive-in-progress.md) and
[reference evidence](evidence/prototype-responsive-reference-local.json).

The Preferences migration now preserves the chart behind the prototype's 432px
drawer, with eight sections and shared native sensor links. Three corrective
component passes retain 36 images (100/100/102 checks); the latest integrated
Linux build passes 123/123 in 20.46s. The isolated route fixture now passes its
26 progress and ten exact Day/software ink checks after the repaint request.
Capture policy passes 216 checks; 50 native marker cases await Windows.
The fixture-free product also passes 123/123 in 19.66s and its actual loader
self-test. Twenty-one product captures pass after scoping the rail identity
assertion to the prototype sidebar (Preferences has its own Radar tab). Two new
marker cases now appear in the fixture's explicit catalog; 217 pure guard checks
pass. See [corrective product evidence](evidence/prototype-settings-product-local.json).
Inline forms, remaining section
content, non-default DPI and boat review are still open; this is not Settings
visual acceptance. See [review](design/reviews/prototype-settings-in-progress.md)
and [local evidence](evidence/prototype-settings-local.json).

Windows development run `36503170626` / remote `6091c23` passes seven of eight
jobs. Both integrated suites pass 115/115; AIS/Passage/Instruments/Energy widget
checks pass 71/26/41/44. Actual-product captures and the corrected object workflow
pass. The new active-route ink check fails on its first Day/software image:
the route is not painted despite valid copied upstream projections. The fixture
omitted the explicit canvas repaint used by OpenCPN's normal route manager;
the replacement schedules that same repaint without requesting navigation
processing. Unchanged pixel assertions and replacement Windows results remain
required. All nine artifacts, 62 native PNGs and 120 reference PNGs verify.
See [negative evidence](evidence/prototype-native-6091c23-failed.json).

The Energy screen's next prototype pass corrects primary card proportions,
numeric hierarchy, battery graphic, forecast rows and power tiles. An owned
presentation boundary withholds predictions from mismatched/stale observations
without inventing sensor values. All 123 integrated Linux tests pass after the
correction; two offline Energy passes each pass 44 checks/seven captures, and
the shared-card Instruments regression passes 41 checks/five captures. Fourteen
new presentation assertions cover provenance and failure states. Lower Explore
pace/calibration UI and native/boat qualification remain open; no conformance
PASS is claimed. See [review](design/reviews/prototype-energy-in-progress.md)
and [evidence](evidence/prototype-energy-local.json).

Windows development run `36500844253` / remote `3a90eff` closes the actual-product
capture refusal: twelve primary and four additional guarded captures pass.
Both native integrated suites pass 114/114; AIS/Passage/Instruments components
pass 71/26/41 checks. Seven of eight jobs pass overall. The object-flow test
reads a previous Route detail diagnostic publication immediately after Settings
returns to Navigation; its screenshot shows the correctly restored chart and
North control. The corrected harness waits for a later Navigation publication,
then applies the same strict geometry/chart assertions. Its Linux object flow
passes, including both settings-return cases and persisted navigation objects.
All nine artifact hashes/CRCs, 55 captured PNGs and 120 reference PNGs verify.
Native route-ink tests were not reached; replacement Windows and boat gates
remain pending. See [evidence](evidence/prototype-native-3a90eff-failed.json).

The current active-route presentation increment copies the immutable prototype's
Day/Dusk/Night route ink into OpenCPN's existing software/GL route rendering.
Standard, selected routes, geometry and progress semantics remain upstream-owned.
Eight Linux render cases pass 26 route-progress checks and ten exact projected
stroke checks each; 16 captures are retained. The rebuilt integrated suite passes
122/122 (19.81s), plus five prototype contract tests and 212 capture-guard checks.
OpenGL uses the pinned renderer's exact RGB/256 conversion. Windows and boat
qualification are pending; broader overlay conformance is not claimed. See
[route-ink evidence](evidence/prototype-active-route-ink-local.json).

Latest Windows development run `36498559587` / remote `42255ef` passes seven of
eight jobs: 114 integrated tests, all component suites and independent ENC captures.
The strict actual-product capture refuses the new rail's unexpected structural
caption. Source inspection of the pinned wxWidgets 3.2.8 confirms default panel
names become native captions. The correction explicitly clears labels only on
the decorative rail divider and drawer containers; capture predicates remain
unchanged. All eight negative-run artifacts and 27 manifest PNGs verify. See
[negative evidence](evidence/prototype-native-42255ef-failed.json). Replacement
Windows capture and boat deployment remain pending.

Replacement `299c728` / run `36496486152` passes the corrected Windows
Instruments rendering check (41 checks/five captures), all 114 integrated tests,
48 native marker-window cases and 212 pure guard checks. Seven of eight jobs
pass. The actual-product capture guard refuses Passage after three successful
navigation theme captures; its child stderr was not retained by the original
CI launcher. The replacement retains that refusal explicitly; no guard has
been bypassed. All eight artifacts and 30 manifest-listed PNGs verify, with
six partial navigation PNGs retained separately. See
[negative native evidence](evidence/prototype-native-299c728-failed.json).
No package or boat acceptance is claimed from this run.

The committed rail/scroll correction `da17dab` has a fresh integrated Linux
configure/build/install and 122/122 passing tests (20.59s). Its Windows
qualification remains pending.

The next native Passage increment now follows the prototype drawer instead of
replacing the chart with a full-page route summary. It copies accepted route and
SmartNav values, matches the exact energy input publication and withholds values
on GPS loss, stale data or revision mismatch. Route commands retain their
existing OpenCPN identity/confirmation gates. Three successful component passes
and two full-product passes retain 41 verified PNGs, including the corrective
icon-background fix. All 121 integrated Linux tests pass after the corrections. Offline component
checks: 26; new provenance assertions: 25. Full product checks include Passage Day/Dusk/Night and Close retaining the
1014×566 chart viewport. Windows development run `36491018722` now passes all
seven jobs (113 integrated tests, 26 Passage component checks); all eight
downloaded artifact hashes/CRCs and 48 captured PNG hashes verify. The exact
local source commit passes 121/121 Linux tests in 19.52 seconds. See
[native evidence](evidence/prototype-native-6a75c05.json). Boat and visual
acceptance remain pending. Saved-passage
and waypoint flows still need prototype migration. See
[review](design/reviews/prototype-passage-in-progress.md) and
[local evidence](evidence/prototype-passage-local.json).

The next Instruments migration is in progress: native prototype wind/heading
and tile composition, retained horizon, copied assessed readings and preserved
instrument selection. Directional graphics require real coherent heading/wind;
COG cannot stand in for heading. All **122/122 Linux tests** pass after the
corrections (19.27 seconds), including 26 new presentation assertions. The
non-installed widget passes 36 checks/five captures; two corrective full-product
runs pass the complete theme/Close cycle and existing AIS settings flow. There
are 42 verified PNGs across the retained corrective runs. The first review's
displaced compass text and unavailable-line collision are corrected and tested.
[Design review](design/reviews/prototype-instruments-in-progress.md) and
[evidence](evidence/prototype-instruments-local.json) record the required
navigation-meaning differences, earlier negative runs and pending Windows/boat
gates. The later rail/scroll correction below removes its older Up/Down chrome.

The Instruments Windows increment `1a175e3` / run `36494702427` is **not
accepted**: 114 integrated tests, object interactions and actual-product/ENC
captures pass, but the isolated Instruments widget capture was blank because
its parent panel was not explicitly sized on Windows. An explicit host sizer
and ancestor-containment assertion correct the fixture; unchanged strict pixels
pass on Linux (41 checks/five images). Replacement Windows evidence is pending.
All seven downloaded artifacts were hash/CRC verified; 38 successful captures
and five negative widget images are retained. See
[failed native evidence](evidence/prototype-native-1a175e3-failed.json).

The boat display-review guard is being qualified for the prototype rail and
known owned surfaces. It retains exact installed executable/session/launch
identity and rejects unrelated or changed windows. Linux policy suites pass;
native marker-window and actual-product capture qualification remain pending.
No new prototype product has been launched aboard, and no actuator action was
added. See [guard review](design/reviews/prototype-boat-capture-guard.md).

A further Instruments correction removes its old permanent scroll toolbar and
restores the exact main-rail button widths/group separation. All 122 Linux tests
pass (19.33s); 16 captures pass exact rail measurements and actual pointer-wheel
access to lower readings/back to top. Windows and boat gates remain pending;
the lower vessel-profile group is still an explicit mismatch. See
[corrective evidence](evidence/prototype-rail-scroll-local.json).

Replacement `c86c6a7` / [run 36485693077](https://github.com/ThereptileII/Work/actions/runs/36485693077)
passes all seven development jobs. All eight downloaded artifacts, their ZIP
CRCs, 29 captured PNG hashes and the probe's 17 binaries were independently
verified. The native PowerShell launcher now records the required negative
exit code 4. An isolated read-only boat probe then ran and shut down cleanly:
`credential_missing`, no subscription and zero targets. OpenCPN remained
closed. This is a negative live-service result, not AIS or product acceptance;
the SSH logon has no usable credential. No key or vessel identity was collected.
See [native evidence](evidence/prototype-native-c86c6a7.json) and
[boat probe evidence](evidence/boat-ais-probe-c86c6a7.json).

At local `820cbf9410870554e93df9555296ef7a852e589d`, a fresh integrated
configure/build/install and all **120/120 Linux tests** pass in 19.90 seconds,
including the corrected scale legend backing. Native depth-label run
`36486349316` and scale-placement run `36487339813` both pass all seven jobs.
All eight artifacts per run and 41 recorded product/component PNG hashes per
run verify. Native Feet/Day, Fathoms/Night and scale Day/Night were reviewed;
unit semantics and scale placement work, while broader chart design remains
open. See [depth native evidence](evidence/prototype-native-7b2a6fd.json) and
[scale native evidence](evidence/prototype-native-ebe7010.json). None of these
incremental gates qualifies the unchanged old boat installation as the new
prototype product.

Latest local chart increment replaces only XNav's large depth-unit emboss with
the prototype metadata typography/placement, using OpenCPN's unchanged actual
quilt/single-chart resolver. Twenty real-ENC captures cover software and
llvmpipe Day/Dusk/Night, all three supported depth units and Standard fallback;
all PNG hashes verify and 120 integrated Linux tests pass. Standard retains its
original emboss. These are development results, not native/boat acceptance.
See [depth-unit evidence](evidence/prototype-depth-units-local.json).

The next local scale-bar pass preserves OpenCPN's distance/unit computation and
moves its legend beside Follow Boat. All 120 integrated tests pass before the
backing-only visual correction; twenty software/llvmpipe/coastline captures
retain their hashes. The first pass exposed an ENC sounding behind the legend;
a small neutral backing corrects that ambiguity. The recorded HTML content gap
and actual painted bounds now pass without overlap. This safety-legibility
exception, native Follow Boat width, chart selector, density and full chart
review remain documented; native/boat acceptance is pending. See
[scale evidence](evidence/prototype-chart-scale-local.json).

Current local chart review: the final HTML cascade now supplies all marker
tokens, including eight roles omitted by extraction from its earlier stylesheet.
The immutable HTML is unchanged; 4 design-contract tests pass. A bounded neutral
Dusk/Night sprite derivation passes 388 resource checks and eight real-ENC
software/llvmpipe captures. Review exposed black cached text in the GL path;
renderer parity and visual acceptance remain open. See the
[ink investigation](design/reviews/chart-ink-contrast-investigation.md).

Native development run `36478655922` at remote `335b14e` (local `bf47479`)
passes 112 integrated tests, 71 AIS component checks, transport/provider tests
and twenty product/ENC captures, but **fails overall**. The new actual-object
Windows flow reached the final waypoint edit with an obsolete System menu path;
the probe package separately refused the pinned bundle's old `vccorlib140.dll`
against the explicitly selected licensed CRT. Both negative logs/artifacts are
retained. Replacement fixes use the current Settings → Waypoints entry and
recognize that CRT family without relaxing unknown-library rejection. No probe
package or live boat AIS result is accepted from that failed run.

Replacement `9a5162b` / run `36481625379` closes the Windows object-flow failure
and passes its 112 integrated tests, 71 component checks and product/ENC captures.
Probe packaging verifies 17 x86 binaries and imports, then fails before launch
because a copied Python environment dictionary looks up mixed-case SystemRoot.
The replacement normalizes Windows environment keys and explicitly removes the
development credential; six portable packaging tests cover this boundary.
Run `36483619726` / `1954c19` verifies all 17 binaries, clean-PATH launch and
source/ZIP construction. Its final Windows PowerShell 5 negative probe check
fails because Start-Process loses the fast child's exit code (null). No failed
package is deployed. The replacement owns the .NET process handle through exit,
drains both streams asynchronously, preserves the deadline and rejects unknown
exit status; CI also asserts the recorded exit code explicitly. The boat is
reachable with OpenCPN closed. A new SSH presence check did not list the saved
AIS credential; this differs from the earlier presence check and is not yet a
credential-read or service result.

Live AIS remains unverified. The local rail revision `00ff457` passes 120
integrated Linux cases plus actual object and synthetic navigation input flows.

The main rail's corrective Linux pass now checks the four measured HTML row
bounds and compact pilot-summary bounds at 1280×800. Typography uses the final
48px/−3px numeric rule; observation age and source state remain actual data.
The card cannot infer a pilot mode from missing feedback. Twelve fixture-free
captures retain Day/Dusk/Night and AIS settings behavior. Populated sensor,
native Windows and boat replacement gates remain outstanding.

The user's September 28 instruction supersedes the older visual baseline:
safety/navigation correctness → supplied v8 HTML → extracted prototype spec →
older brief/image → current Beta. The native architecture and all existing
data, control, installer and Legacy/Safe contracts remain in force.

The supplied HTML and all 113 companion files are committed unchanged. HTML
SHA-256: `b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
Original ZIP/extracted/Git bytes were verified. See
[prototype specification](design/prototype-spec.md),
[component map](design/prototype-component-map.md),
[interactions](design/prototype-interactions.md) and
[chart inspection](design/xnav-chart-style.md).

Sixty canonical renders on each of Linux and native Windows cover twenty
actual states across Day/Dusk/Night using pinned Chromium, offline loading and
a fixed clock. Windows resolves Segoe UI; Linux resolves Liberation Sans.
Both reference artifacts and all Windows PNG hashes were verified. See
[evidence](evidence/prototype-reference-7925b160.json).

Native shell migration now follows the 68/80/186/132/34px composition. Two
Linux capture/review passes exposed and corrected undersized GTK font ems,
selected navigation state and energy-card composition. Eight lossless
reference/current/diff sets retain visible mismatches without masks or broad
similarity tolerances. The actual coastline canvas remains stock presentation.
The integrated build at local `545feca` passed 110/110 tests; the first run
conflicted with the simultaneous capture on upstream's fixed REST port, so the
complete gate and captures were repeated serially. Native Win32 MSVC compilation and 102/102 integrated unit regressions also
pass at remote `8357239` (local `a6a9259`). Its eight captures exposed occluded
chart controls; the capture job failed during secondary command-line quit.
Follow-up z-order/shutdown fixes await replacement native evidence. The next
run at `83569f0` hit a Windows `max` macro collision in the new transport test
driver. The correction builds at `d92e187`; 102/102 integrated native tests and
6/6 portable AIS tests on each platform pass, including protected credential
storage. The new native fragmented-overflow transport test then failed: bounded
reading stalled while OpenSSL retained decrypted bytes. A resume-before-poll
fix now passes 18 hostile-frame scenarios and the actual-provider lifecycle on
Linux and native Windows at remote `ea869a9` (local `2a3c52`). Its native MSVC
build and 102 integrated tests also pass. Downloaded artifact hashes, sizes and ZIP CRC
were verified; see [replacement run evidence](evidence/prototype-native-ea869a9.json).
The native capture job still fails: input/capture timing reads an older light-mode
snapshot. Downloaded ENC images also expose absent floating controls and weak
Night contrast. Neither a successful screenshot command nor `visible=true`
qualifies these views. Replacement state-driven and painted-pixel checks are in progress.
The replacement at remote `e51a7df` (local `37b36dc`) now passes all six
development jobs: native build, 102 integrated tests, AIS contracts on both
platforms, hostile TLS/provider tests and native composition/ENC capture.
Downloaded artifact digests, sizes, CRC and each recorded PNG hash verify.
Controls are visibly painted and real zoom input changes the upstream scale;
observed Day/Dusk/Night/Day retains ENC content and exits cleanly. This closes
the capture/control failure, not visual acceptance. Night hazard/text contrast,
rail composition and primary sheets remain mismatches. See
[replacement evidence](evidence/prototype-native-e51a7df.json).
No screen is accepted; see
[conformance](design/prototype-conformance.md) and the
[review log](design/reviews/prototype-native-shell-in-progress.md).

Online AIS now has owned target/cache/provider contracts, onboard precedence,
aging, bounded viewport subscriptions and reconnect policy. The foundation
passed native Win32 MSVC and Linux tests at `087318c`; downloaded JUnit artifacts
verify ([evidence](evidence/prototype-ais-foundation-087318c.json)). Its pure JSON
codec adds 100 deterministic checks and the session adds 116. The portable
suite now passes 75/75. The actual provider passes a local TLS lifecycle test
covering prompt subscription, viewport replacement, reconnect and resubscription.
The protected Windows credential adapter and Linux environment-only adapter
are implemented; isolated native Credential Manager tests pass at `d92e187`.
The opt-in bounded TLS/compressed WebSocket boundary passes 18 loopback
adversarial scenarios on Linux and native Windows. The worker provider lifecycle
passes on both platforms. XNav-only product/provider wiring and native Traffic
drawer are now in development validation; online chart overlay and live
AIS acceptance remain pending. No service connection or target count is claimed.
The application-thread Online AIS preference/credential boundary and copied
viewport normalization add ten deterministic integrated cases. Linux passes
120/120 with these additions. Opt-in, failed-save disable, replay isolation,
credential separation, worker rejection and pinned LLBBox antimeridian/world
bounds are covered. Fixture-free Linux captures exercise the native settings
path: default OFF, missing-key withholding, OFF, theme changes, Back and Close.
Two failed capture passes exposed native sheet stacking, followed by a correction
and retained paint-latency checks. Windows replacement and populated-target
interaction remain required. See the [AIS drawer review](design/reviews/prototype-ais-drawer-in-progress.md).
The replacement native run at `3db9704` (local `98f3646`) passes all six
development jobs, 112 integrated Windows tests, six AIS contracts per platform,
and twelve product captures including Online AIS settings. Downloaded digests,
CRC and PNG hashes verify. Traffic and Night settings were visually reviewed;
heading mojibake was identified and corrected in the next increment. See
[native evidence](evidence/prototype-native-3db9704.json).

Supplemental online chart marks now have an owned, bounded, independently tested
presentation model and two narrow paint/context-selection hooks into OpenCPN.
Thirty-nine new assertions cover identity/provenance, local precedence, position
validity, directional withholding, retained-state aging/expiry and antimeridian
coordinates. Portable tests pass 76/76 and Linux integrated tests 120/120.
The fixture-free product/settings capture still passes with no synthetic data.
Replacement native [run 36471022893](https://github.com/ThereptileII/Work/actions/runs/36471022893)
at `ad4913f` (complete local `97665f5` mapping) passes all six jobs, 112 integrated
tests and seven AIS suites on each platform. All six downloaded artifacts verify
their API digests, lengths and ZIP CRC. Twenty product/ENC captures verify their
PNG hashes; Traffic Day and settings Night were reviewed, confirming the UTF-8
heading fix. See [native evidence](evidence/prototype-native-ad4913.json).
Populated-target, actual symbol/software/GL and live boat evidence remain
pending; the compiled overlay is not declared accepted.

A separate, non-installed native component executable now exercises populated
AIS list/card interactions without placing fixtures in the product. The first
pass exposed unsupported wxGTK Tab-state querying; input-modality event tracking
replaces that query. Actual painted captures then drove a second pass matching
the prototype's status pills, small units, 16px statistic gutter, callout and
right-aligned detail rows. Fresh OpenCPN estimated CPA/range values are accepted
as estimates and withheld after their own dependency expiry; online CPA/TCPA
remain unavailable. Replacement native Windows and boat review are still gates.
The completed local correction passes 120 integrated tests, 69 component checks
and all three immutable-design checks. Nine painted component captures and
exact drawer reference/current/diff sets are retained as development evidence.
Replacement native `079ec95` now passes all six jobs, 112 integrated tests and
69 populated component checks. Six downloaded artifacts and nine component
image hashes verify; Day/Night were reviewed. Typography/border/shadow mismatches
remain. See [evidence](evidence/prototype-native-079ec95.json).

A separate non-installed read-only AIS commissioning probe links the production
TLS/provider/credential boundary. It never loads OpenCPN, profiles or plugins;
its bounded run reports only aggregate counts and fixed connection-state labels.
Linux integrated build and 30 offline CLI/privacy checks pass, as do nine source
archive tests. Native packaging/closure and live service evidence remain pending.
Only credential-entry presence has been checked aboard; no successful connection
or live target count is claimed yet.
The first probe package attempt at `d79fac6` passed native build, 112 tests,
24 offline probe checks, transport/provider and UI captures, then correctly
failed on conflicting app-local MSVC CRT copies. No probe ZIP was uploaded.
The correction explicitly chooses the licensed toolchain CRT (as the main
packager does), still rejects other ambiguous dependencies, and passes four
portable resolver tests. Added exact OpenSSL dependency notices and a bounded,
hash-verifying boat invocation script. Replacement native packaging is pending.

The actual-model object flow now validates retained AIS summary/report age,
native drawer scrolling and return-to-chart selection. It also exposed and
fixed the timeline being omitted when Advanced Settings rebuilt the chart.
Linux passes all 27 object scenario groups, seven actual pointer actions,
26 captures, both exact settings-return layouts, coastline and navobj storage.
The obsolete compact-card Details hop was replaced by the prototype's direct
target drawer; no underlying route/waypoint/AIS semantic check was removed.
The dedicated widget suite now passes 71 checks and the established integrated
command passes 120 tests. A separate native object-flow development job was
added; the full existing release regression workflow remains mandatory.
See [local evidence](evidence/prototype-ais-navigation-chart-ink-local.json).

The second chart-ink pass preserves Day contrast and improves Dusk/Night text.
Resource checks pass 377; actual public ENC software/llvmpipe captures retain
content through all themes. Baked raster symbols still need contrast work,
and no chart presentation is accepted yet.

XNav-owned S-52 presentation is being implemented as separately packaged,
hash-verified resources derived from the pinned baseline. Its resource tests
preserve all symbols, lookups and conditional navigation semantics while
checking deterministic palettes and depth-role distinctions. The integrated
build, actual ENC palette review, Standard fallback and renderer/mode cycles
remain gates. Linux now passes the integrated build and 110/110 tests, 369
resource checks, 18 adversarial TLS scenarios and the actual provider lifecycle.
Corrected coastline palette captures and public ENC XNav/Standard captures
retain content through Day/Dusk/Night/Day with clean exits. The Linux OpenGL
capture uses Mesa llvmpipe, not a hardware-GPU acceptance result. The review
records remaining chart-text/night-contrast and overlay differences; see
[chart review](design/reviews/prototype-chart-v1-in-progress.md).
AISStream must not
inherit the local Signal K client's disabled TLS validation. The prototype's
newer vendor symbol snapshot cannot replace the pinned resources blindly.

Boat SSH returned on September 28 at 18:36 UTC. The interrupted commissioning
transaction is now closed: normal shutdown of the exact orphan chart decoder,
fresh cold inspection, narrow adoption of reviewed stock-return preferences,
and restoration all passed. All five quarantined plugin DLLs have their original
hashes again; stock executable, navigation database and chart database remain
unchanged. No application/helper process or active commissioning marker remains.
The original connection direction is restored, without launching the application.
Recovery tooling replacement [run 36469956631](https://github.com/ThereptileII/Work/actions/runs/36469956631)
passes all nine jobs, including 178 native adoption checks. See the
[recovery evidence](evidence/boat-prototype-reconnect-20260928.md).
Any next application launch requires a fresh read-only audit/preparation against
the newly adopted baseline; expired launch/restart receipts cannot be reused.
The deployed application still uses the previous Beta UI. No physical actuator
commands are authorized or sent.

## Prior Beta 2 development and deployment evidence

**Beta 2 development is in progress; its current development package is installed
on the boat, but it is not qualified for release.**

The user's Desktop feedback has been read completely and recorded in
[boat Beta 1 feedback](feedback/boat-beta1-feedback.md). The approved design
reference was the Beta 2 baseline before the new HTML design lock. Work separates fixture-enabled CI
executables from the installed product, refines shared visual components and
navigation workflows, and adds repeatable boat deployment and maintenance tools.
The accepted Beta 1 results below remain historical evidence, not Beta 2 gates.

### Boat reconnected and interrupted session recovered — September 28

SSH/Tailscale/RustDesk are available again. Windows booted at 06:35 UTC;
OpenCPN's log records a clean application exit before that restart. The last
unacknowledged Menu/staging requests left no corresponding run records. No
action was blindly replayed. Stock OpenCPN, the installed `7827acb` executable,
navigation database and chart database retain their exact recorded hashes.

The closed-session review found eleven persisted display/position/GL changes
relative to the prepared input-only profile. Pinned `LoadMyConfig` explains
the non-expert texture minimum of 128MB after loading the earlier upgrade's
64MB setting. The narrow adoption extension passed 125 portable cases and the
nine-job native [tool run 36388235873](https://github.com/ThereptileII/Work/actions/runs/36388235873)
at `ad08061eb46d63401a5565ae561bb8f780cfae88`; all nine artifact API/upload/
download hashes, sizes and ZIP CRC agree. A separately staged qualified copy
preserved the interrupted session's source dependencies during restoration.

All five quarantined DLLs and the original connection direction are restored.
Independent byte comparison confirms the adopted INI retains every other
post-session byte. Its SHA-256 is
`9dfc3fb94a5de45047f785aa0d922ea3c990b8d58522a61d9ec5047ed7d328f3`.
The active commissioning marker was removed only after verified completion.
No application or physical command was launched. The source checkout then moved
to qualified tool commit `ad08061e`. A new deployment/commissioning session is
required; the previous four-hour restart session is expired.

Both complete application runs (`7827acb` and `79a95c4`) finished all sixteen jobs
successfully. The latest `79a95c4` candidate has 110 integrated Linux tests,
102 integrated Windows tests, 45 installer lifecycle checks, all three DPI
scales and actual three-hour endurance passes on both platforms. Its final
package, native and Linux evidence match API/upload/download digests and ZIP
integrity. All 999 portable files and 5,101 corresponding-source files verify.
The product is fixture-free and retains the approved Win32 plugin ABI.

Setup SHA-256 `b281362f6bf6d51410aba1491e16049d4c7bf243f6602a2bd7a3d0342ab09c4d`
has been verified on the boat. Update passed: all 1,269 files in the 1,292-entry
cold profile inventory remain byte-identical, including both navigation and chart
databases. The installed executable matches the exact package (`aee52bd2…`),
and stock OpenCPN is unchanged. Boat mode/maintenance/screen acceptance and remaining
Beta 1 retirement are still open; CI success alone is not Beta 2 acceptance.

The exact `79a95c4` product launched with a fresh read-only audit. A Windows
network prompt was inspected and its proxy closed without granting access;
the composited sheet remained visible. The subsequent fixed-size review exposed
a tooling bug: restoring the maximized frame first selected a normal rectangle
partly outside the monitor, and the containment guard stopped before repositioning.
No chart/pilot action was sent. The repair applies the already tested stock-window
recovery policy to the distinct XNav identity, retains strict capture/input guards,
and adds eight disposable native resize cases. Boat deployment awaits native
qualification; the running product and restart-session dependencies are unchanged.

### Reconnection required after the resumed session

At 14:44 UTC, local Tailscale is Running but the boat peer is offline with
last-seen 08:17:38 UTC. A fresh SSH probe times out. The `79a95c4` installation
and pre-launch profile preservation are verified; the present process state
is unknown. The last window action restored the maximized frame and refused
before fixed placement. No action is replayed and remote services are unchanged.

The four-hour session created at 07:11 UTC is expired. On reconnection, inspect
the exact failed request/result, process/start identity, clean-exit log and
active commissioning journal before normal close/restoration. Do not reuse
its launch or mode receipts. Five DLL quarantines and the input-only connection
setting remain part of the unclosed transaction; do not run maintenance first.

Resize tool commit `9c3b7bb7b1518be306bd751788b01912ee1543dd` maps all 716
local source blobs/modes exactly and preserves the pin plus eight unrelated
repository blobs. [Native tool run 36438174776](https://github.com/ThereptileII/Work/actions/runs/36438174776)
passed all nine jobs. All nine artifacts match API/upload/download digests,
sizes and ZIP CRC. Thirty-one actual display cases pass, including eight new
resize cases; fourteen mode cases remain passing. The repair is not deployed
while the boat is offline. Local checks pass: 178 display-policy/compilation checks,
133 restart-window checks and 13 handoff tests. No product rebuild or boat
visual acceptance follows from these tooling tests.

The following September 27 checkpoints are historical; the recovery above
supersedes their offline/active-commissioning state.

### Current boat deployment — 09:36 UTC

The independently downloaded `7827acb` development package was updated on the
boat at 09:09 UTC. Setup SHA-256 is
`7bfe4ba1c31a68304edd04f261ac8bee3f7c93a2612f3bbbb3c1da1b0b076e71`;
installed executable SHA-256 is
`56f435d08904ec5a0b8a840fe13625b9476353e7c97a8d2368f16c100b5672e5`.
All 904 real profile files compare identically before/after the update, and the
approved stock executable is unchanged. Full platform endurance remains open.

A fresh read-only commissioning transaction and guarded-restart session preceded
the 09:14 XNav launch. Its five quarantined plugin copies and input-only serial
change remain active during this review and must be restored after normal close.
Both queued Windows network prompts were cancelled without granting access.
The exact native 1280×800 frame at 150% DPI now shows real chart content, clear
Center, all four rail values and no floating Dashboard. Three zoom-out actions
retain chart content. These observations are scoped visual checks, not complete
boat acceptance. The actual monitor remains 1920×1080; physical 1280×800/touch
acceptance must not be inferred.

The new pan review found the tooling's diagnostic-path mismatch: the installed
product writes under `opennav-logs`, while the helper looked at the profile root.
The correction adds native wrapper cases for a decoy root file, missing/stale
installed diagnostics and wrong commit. Tooling `f44f2b4` passed all nine native
jobs in [run 36310215706](https://github.com/ThereptileII/Work/actions/runs/36310215706),
including 23 actual display cases. All nine downloaded artifacts match
API/upload hashes, sizes and ZIP integrity. It is not yet deployed.
The earlier main-window refusal was traced to a native hover tooltip after
cancelling the network sheet; a bounded pointer-only move cleared it. No failed
zoom/capture request sent input. Product/restart tools were not modified aboard.

### Boat connectivity interruption — 09:50 UTC

Tailscale reports the boat peer offline with last-seen 09:50 UTC; local
Tailscale remains Running/online. SSH and Tailscale probes time out. No remote
access settings were changed. The last confirmed XNav display was Diagnostics;
a subsequent Menu request returned no remote receipt. Do not repeat it or infer
process exit until the actual journal/process is inspected after reconnection.
The commissioning transaction remains active and must be restored only after
normal application close and reviewed profile deltas. Mode/maintenance/old-copy
retirement gates remain open. [Partial display review](design/reviews/beta2-boat-7827-review.md).

The staging command for a separate review-tool copy did not return a receipt;
no archive or staging script was uploaded. A read-only run-directory check is
required before retrying it. A versioned staging helper is being native-tested
so newer qualified display tools can coexist with the frozen restart dependencies.

### Current boat review tools

`07da8e307f174646f4fd2722ec84bda511f4bcfd`,
[run 36303832915](https://github.com/ThereptileII/Work/actions/runs/36303832915),
passed all nine native jobs. Downloaded artifacts match upload/API SHA-256,
sizes and ZIP integrity. Twenty display cases and fourteen mode cases pass;
chart pan records one exact key pair or refuses without input. Source checkout
on the boat passed at 07:51 UTC, with no application change or launch. Actual
boat pan, touch and replacement visual acceptance remain open.

### Native review refinement

The retained native screenshots exposed an ambiguous autopilot subtitle. It now
states “Manual commands require feedback confirmation”; this is a requirement,
not a claim that feedback has arrived. [Twenty-screen review](design/reviews/beta2-native-9f592-review.md).
The fixture-free Linux build/install passes. A separate complete CI candidate
will qualify this refinement together with the bounded chart-pan tooling.
No release or boat visual acceptance is claimed from the old screenshots.

### Current replacement validation

The current full replacement is `79a95c4f39063c20ca4d5a98c3147080d9813077`,
[CI 36311718794](https://github.com/ThereptileII/Work/actions/runs/36311718794).
All 710 mapped blobs/modes match local `3e3c628`; the pinned manifest and eight
unrelated repository blobs are preserved. This includes the direct cleanup
deadline repair and Windows PowerShell assembly load needed for isolated
review-tool staging. Application and installer source still match `7827acb`.
The separate native tool run is
[36311719759](https://github.com/ThereptileII/Work/actions/runs/36311719759).
The tool run completed successfully: nine native jobs, 12 staging cases, 23
display cases and 14 mode-window cases. All nine artifact API/upload/download
hashes, sizes and ZIP integrity agree. [Tool evidence](evidence/beta2-review-staging-tools-79a95c4.json).
The full application run remains unfinished; no newer helper has been deployed
during the boat connectivity interruption.

The earlier `a5b290e52ebeb83bda75c1f7008879c398548dc3` candidate in
[CI 36307149573](https://github.com/ThereptileII/Work/actions/runs/36307149573)
failed Windows installer cleanup and cannot qualify the release.
All **706** mapped blobs/modes match local
`9e0d0c77b5ba2513cc7355da80f38e03ddf631b6`; the pin and eight unrelated blobs
are unchanged. It merges the refined candidate history without rewriting either
branch. Application/installer code is unchanged from `7827acb`; the installer
harness now waits for actual relocated cleanup and preserves failed fixtures.
Six timing/lifetime tests pass locally and both contract jobs pass natively.
The verified earlier failure is documented in the
[maintenance completion review](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The native installer reached 45 checks, but its final direct-engine uninstall
hit a separate 120-second harness limit. Its prior relocated uninstall completed
in 150.406 seconds. The final direct wait is being aligned with the same bounded
600-second deadline; twelve timing/lifetime cases pass locally. This candidate
is not accepted. Its subsequent native DPI tests passed at 100%, 125% and 150%.
Eighteen verified original screenshots were individually reviewed, including
alert/rail coexistence, System, night surfaces and mode chart content; see the
[scoped DPI review](design/reviews/beta2-native-a5b290-review.md). High-DPI expanded
pages still scroll; physical touch and the complete boat sequence remain open.
Both complete replacement platform gates and boat validation remain pending.

`7827acb7c8b0d708285bd26a4c48d545dd64d139` is running the complete gates in
[CI 36304661282](https://github.com/ThereptileII/Work/actions/runs/36304661282).
All **699** mapped blobs/modes match local
`a883f3fb7c729773e323d9234d95dda1f62fcacf`; the pinned baseline and eight unrelated
repository blobs are preserved. It includes the autopilot wording refinement,
bounded chart-pan tools and the DPI harness correction below. No replacement
product had been deployed at the earlier checkpoint; see the current deployment above.

The boat tooling checkout moved to `7827acb` at 08:36 UTC after its native tooling
gates passed; no application changed or launched. At 08:48 UTC, cold checks still
show the exact `8e780` executable, unchanged stock/INI/navigation database, no
application/helper and no active commissioning. The remaining Beta 1 portable
folder and its three download files match their expected hashes. Retirement is
prepared but awaits the replacement's mode/recovery checks; no files were moved.

`608756` remains historical diagnostic work. Its harness added an input-evidence
dictionary which source review subsequently found shadowed the screenshot Path;
it cannot qualify the release. The corrected function reaches screenshot and
touch-close checks at all scales with a mocked native boundary, but only the
new native run can establish actual Windows behavior. The short-lived `7ac585`
run was superseded by `7827acb` through normal same-branch CI cancellation.

The preceding `9f592` application candidate is **not accepted**: its full Windows
installer matrix passed, but the DPI gate timed out on its first physical chart
context gesture. The bounded harness repair verifies the foreground process and
actual chart HWND geometry before sending one click; no retry, scale or layout
assertion is removed. [Failure and replacement scope](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The boat remains unchanged while the new exact-commit run proceeds.

### Previous full candidate (Windows DPI failed)

`9f59209914f57ff97af7e184b201752b8f422f0a` ran the native gates in
[CI 36299835767](https://github.com/ThereptileII/Work/actions/runs/36299835767).
All 694 mapped blobs/modes match local `c72061459b8c532c0f6aaad5846c61a52231ea2f`.
It includes the exact upstream manager type, Dashboard presentation, maintenance
Cancel/paint-readiness harness repairs and complete resource-review dependency
closure. The corrected Linux build and all 27 object-workflow groups pass;
replacement native UI and release gates remain open. It is not deployed.

The native integrated application build and both **71/71** contract suites now
pass. Eleven prerequisite artifacts have been independently downloaded and
checked against their API/upload hashes, byte sizes and ZIP integrity. Linux
completed its functional/UI checks and entered the three-hour elapsed-time trip
at 06:45:57 UTC. Windows passed both **102/102** integrated/fixture-free CTest
runs, all **45** installer lifecycle checks, seven production recovery groups,
eight actual user-flow groups and software/OpenGL chart/plugin checks. The DPI
failure withheld the development and release packages; native endurance did not
start. Downloaded full native evidence matches API/upload SHA-256, size and ZIP
CRC. The earlier Linux trip continues as historical evidence, not acceptance.

Before the next boat launch, source inspection found that the restart tooling
rejected separately reviewed stock and bundled DLLs sharing a basename. A
[per-copy shutdown identity](installer/restart-plugin-copy-identity.md) repair
passes 309 portable policy checks and 17 copied-dependency checks. Tooling
candidate `506a70f5c49cf6d9335be31891817bf88ebc120e` is in
[native run 36301213829](https://github.com/ThereptileII/Work/actions/runs/36301213829),
matching all 696 mapped blobs/modes at local `b7c916d90bcb4c0c086516e838696a53e9acf194`.
It does not change the application candidate or cancel its endurance gate.
That tooling passed all nine native jobs and was checked out on the boat without
an application change. The subsequent owned-Legacy-window refinement is also
qualified and is the current source checkout, described below.

The preceding boat tooling was `3552a3ce2c2d78b8c46ec2a94639cd62d060bee6`,
[run 36301703492](https://github.com/ThereptileII/Work/actions/runs/36301703492).
All nine native jobs and downloaded/hash-verified artifacts pass. Fourteen
actual mode-window cases include the preserved floating Legacy instruments;
obscured menus, foreign/unowned windows and unexpected XNav/Safe overlays remain
refused. [Scoped evidence](evidence/beta2-legacy-window-review-3552a3.json).
The boat application is still the closed `8e780` development build. Its current
904-file profile inventory is recorded privately for independent update checks.
The 07:16 UTC cold preflight independently rechecked the stock/application,
INI, navigation database and chart-list hashes: unchanged, no running OpenCPN
or commissioning transaction, and SSH/Tailscale/RustDesk still running.

The earlier cold restoration used qualified tooling `1439b5b2dac669090b5de4ef956eaffa0f42c542`,
[run 36299584290](https://github.com/ThereptileII/Work/actions/runs/36299584290).
All nine jobs and all downloaded/hash-verified artifacts pass, including 163
native baseline-adoption cases and actual broker/Prepare/Arm checks.
[Scoped tooling evidence](evidence/beta2-resource-adoption-1439b5.json).

### Superseded Dashboard type candidate

`ee475f0b5dbef6ea9a5ff5778f3238665f47d944` was tested in
[CI 36298899492](https://github.com/ThereptileII/Work/actions/runs/36298899492).
All 692 mapped blobs/modes match local `bd6d6511da3a6a3d843b9ef1c20d4eb847d0307e`;
unrelated repository files and the pinned OpenCPN revision are unchanged.
It includes the Dashboard presentation refinement and the two scoped harness
repairs below. Its separate branch preserves the preceding Linux elapsed-time
run while testing this exact new product. No replacement boat deployment yet.

The native product link failed on the new Dashboard bridge’s incorrectly declared
layout-manager type. It is corrected to the actual pinned `OCPN_AUIManager`;
[failed evidence and correction](design/reviews/beta2-plugin-workspace.md).
No failed build was deployed.

The first installed boat session has closed normally with a retained native
handle and measured exit **0**. No chart helper remained. The navigation database
is byte-identical. Four final INI changes have been inspected; a narrow
[installed resource adoption](installer/beta2-installed-resource-adoption.md)
check has passed all native tooling gates. The closed transaction is now
restored: all five quarantined DLLs and the original connection byte are back,
the four independently reviewed migrations are preserved, and no application
was launched. [Boat completion record](evidence/beta2-boat-first-xnav-8e780.json).

### Previous full candidate (Windows failed)

`edd8da0a4bd386fbb2dbd249289b9f09eeae8dc3` is the replacement full candidate,
matching all 680 mapped blobs/modes at local
`3e0b17dcbb353793a5c135c18774a1e91a6a3b02`, in
[CI 36295867893](https://github.com/ThereptileII/Work/actions/runs/36295867893).
It downloads both immutable historical installers before clearing the read-only
Actions credential. The previous run is superseded after its native installer
harness failure; its unfinished Linux endurance is not an accepted soak.
The native integrated and fixture-free builds passed, as did chart/plugin checks.
The installer exercised genuine historical update/rollback, then the maintenance
Cancel test incorrectly required exit 0 instead of NSIS's documented user-cancel
exit 1. The DPI harness read Diagnostics before its first paint established the
scroll extent. Both failures and narrowly scoped test repairs are recorded in
[the harness review](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The complete Windows gate failed; no replacement release artifact is accepted.
The same run's Linux gate completed successfully at 08:32 UTC: both 110/110
CTest suites pass and the trip measured 10,800.19 seconds with 1,080 samples and
540 UI/dropout actions. Its downloaded evidence matches API/upload SHA-256, size
and ZIP integrity. [Historical Linux record](evidence/beta2-historical-linux-edd8da0.json).
This does not qualify the failed Windows candidate or the newer replacement.
The historical product has not been deployed.

The next local increment suppresses only registered bundled Dashboard panes in
XNav, restoring their original workspace in Legacy. This follows the actual
boat observation of large floating Legacy instruments obscuring the chart.
Linux integrated build, **110/110** CTest cases, **27** wx object groups and both
software/OpenGL chart cycles pass. Native replacement and boat review are pending.
[Presentation contract and coverage](design/reviews/beta2-plugin-workspace.md).

### Superseded credential-order candidate

`a896fb5c5fa935b5daa757087ebedc4cd531c1d3` was the previous full candidate,
matching all 664 mapped blobs/modes at local
`5dd114c00cf07ceef8b32e7d6ec978e33d5d9e50`, in
[CI 36292727131](https://github.com/ThereptileII/Work/actions/runs/36292727131).
Native contract, maintenance, transport, broker, display, bilingual stock-tool,
integrated/fixture-free builds, DPI and chart/plugin gates passed. The installer
gate passed initial installation/rollback, then failed because the harness had
already cleared its Actions credential before fetching the second pinned prior
package. The download ordering is corrected locally; genuine early-package
rollback and complete endurance remain replacement-candidate gates.
It includes the explicit historical-layout rollback fix and narrow observed
profile-migration rules.

The superseded `fa27637eb30150b472acd631cd28e6426d43b9fe` ran in
[CI 36291671816](https://github.com/ThereptileII/Work/actions/runs/36291671816).
All 655 mapped blobs/modes match local
`357d4fee21bc6a1c3211646f230b89feadb036a0`. It includes the corrected endurance
palette action and unchanged-value layout caching. It is not accepted and will
not be released: subsequent source review found that an early 0.4 Beta 2 package
still used the historical Start Menu layout. The new explicit layout marker
preserves that package's immutable maintenance engine during rollback. Native
x86/x64 COM tests pass; a complete genuine-package rollback test remains a gate
for the next product candidate.

### Retained development review package (not qualified)

`8e780edc34f68abd693a5d5f6aecdb3ba05a75c4` was exercised in
[CI 36287991989](https://github.com/ThereptileII/Work/actions/runs/36287991989).
All 630 tracked blobs/modes match local `81ec667e1d3740b36b99eaf6a1ed7526b7edd044`;
CI fetches and verifies the unchanged pinned OpenCPN source. This replacement
adds the exact-version installer caution flow, reviewed profile-migration
preservation, complete broker fixtures and component-local dim-mode hover hints.
The local contract suite passes **71/71**, integrated Linux CTest **110/110**,
and actual wx object workflows **25 groups / 25 captures**.
The native functional suite, fixture-free recovery package, installer lifecycle,
100/125/150% DPI/hover checks and chart/plugin gates passed. Their development
review bundle and full native evidence have been downloaded and hash-verified.
The Windows endurance harness then failed before its first sample because it
still selected the removed Light caption. The next source uses the existing
palette action and verifies Day/Dusk/Night. This candidate has no accepted
Windows endurance result. Its Linux elapsed-time run was cancelled with 327
retained samples and has no accepted endurance result. The hash-verified setup has now completed its first boat installation with
exit 0. Independent before/after inventories confirm the complete normal profile
is identical, and stock OpenCPN retains its validated hash. A fresh installed plugin/data audit preceded the read-only launch. The actual
1280×800 frame now shows real licensed chart content, ownship and saved marks.
Live N2K wind, depth, STW, water temperature and tanks are observed; battery and
motor data remain unavailable and dependent energy estimates are suppressed.
A saved floating Dashboard still obscures the left chart, so strict clear-overlay
acceptance remains open. The Windows firewall prompt was cancelled, with visual
confirmation; no Allow action was used. This development bundle is not a
qualified release. [First boat review](evidence/beta2-boat-first-xnav-8e780.json). [Initial installation](evidence/beta2-boat-first-install-8e780.json).

### Previous candidate (not qualified)

`60cd054712c6a930147247970121a959d48e03bf` was exercised in
[CI 36284246369](https://github.com/ThereptileII/Work/actions/runs/36284246369).
All 591 tracked source blobs and modes match local source
`f123bf5c3d33d979409a8f32418653ac56c2d413`; the pinned upstream gitlink is unchanged.
The local portable contract suite passes **70/70**, and the sequential integrated
Linux regression suite passes **110/110**. This candidate includes the
startup-log rotation fix, preserved plugin workspace, tighter instrument spacing,
fresh waypoint selection/permissions, and truthful route Undo eligibility.
The expanded Linux object workflow passes 24 groups with 25 captures, and the
actual-pointer workflow passes eight groups with nine captures. These local
results do not replace native package or boat acceptance.

The same candidate's native fixture-enabled functional suite, fixture-free
recovery-package launch/mode smoke, DPI and chart/plugin checks have passed.
Its installer gate failed when the accepted Beta 1 executable displayed the
expected version-change navigation caution during the real update sequence.
Candidate clean install, chart startup and first-install rollback had passed;
the remaining lifecycle and native endurance steps are not accepted. The
[narrow harness repair](installer/beta2-version-transition-notices.md) retains
the actual captured warning and explicit Agree action, with exact owned-version
checks. The expanded local portable suite passes **71/71**; a replacement native
candidate is now running above. The superseded candidate's Linux three-hour
elapsed-time trip began at 01:26 UTC; it is not an accepted endurance result.

The separate exact-revision commissioning tooling has passed its native marker
transport, full broker and actual Prepare/Arm/Collect gates; downloaded evidence
and upload hashes are verified. See [the scoped qualification](installer/commissioning-restart-qualification.md).
Actual installed-app launch, plugin shutdown, in-app mode transitions and boat
screen review remain outstanding. Later UI-driver tooling is qualified separately;
it is not part of the current candidate's acceptance evidence.

### Boat prerequisite — official 5.12.4 upgrade verified

Read-only inspection found **OpenCPN 5.12.2-0+b69f44c / x86**, executable SHA-256
`2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48`.
The user authorized a backed-up upgrade. The official visible Upgrade has now
completed, and the installed **5.12.4 x86** executable matches the validated hash
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
All 539 profile files remained byte-identical. Complete third-party plugin files,
the RTL-SDR registration and its original uninstaller were preserved; no recovery
copy-back was needed. The original cold backup remains verified and retained.
The wizard's Upgrade/reset/summary/Finish screenshots were reviewed; Run and
Show were unchecked, and OpenCPN was not launched. Tailscale, SSH and RustDesk
remain running. [Upgrade evidence](evidence/beta2-boat-stock-5.12.4-upgrade.json).
This closes the stock-version prerequisite; Beta 2 still requires its own gates.

A stock-only commissioning inventory matched all nine reviewed plugin DLLs
and their source evidence. The stock session used a journaled one-byte input-only change and four
temporary DLL quarantines. Both have now been restored after closed-session
inspection, preserving the independently reviewed startup migrations. The exact official stock
executable launched with empty arguments on the interactive desktop, preserving
Swedish locale. Its complete first-start GPL/navigation caution was captured,
downloaded, hash-verified and reviewed. The separately native-qualified warning
tool acknowledged it once after two consecutive complete reviewed-image matches;
modal dismissal and the current-launch startup-finalized marker are confirmed.
[Read-only preparation](evidence/beta2-boat-stock-readonly-preparation.json),
[actual warning review](evidence/beta2-boat-stock-warning-review.json), and
[qualified settling tools](evidence/beta2-warning-settling-4fa62e.json).

The first view showed water at a close 100m scale
and a Windows Update notification. No Windows restart or update action was taken.
The later capture refused the overlapping notification. The fixed-size resize
then exposed a partially offscreen saved normal window and refused before changing
its size. Native-qualified replacement tools have now recovered the boat window
to an exact 1280×800 visible frame at DPI144 without changing display settings.
The exact launch log records an Intel UHD OpenGL4.6 canvas context. This is not
proof of chart rendering or hardware acceleration. No physical command has been
sent. A private diagnostic image now shows the **real detailed nautical chart**,
ownship and saved marks. Its overlapping floating Dashboard pane still fails
the strict clear-overlay capture rule; the diagnostic image is not substituted
for that automated gate. [Native tool qualification](evidence/beta2-stock-chart-5013db.json)
and [actual boat observations](evidence/beta2-boat-stock-first-chart.json).
Both configured chart directories are currently accessible, and the chart
database still matches its pre-upgrade hash. A preliminary live INI comparison
found seven startup changes. After normal close was requested, the log records
complete clean shutdown and OpenCPN is no longer running, but the close checker
did not retain a native process handle and cannot establish its exit code.
The chart plugin also restarted its local helper during the final redraw;
cold restoration correctly refused while that helper remained. Native-qualified
cleanup subsequently verified its exact pipe server/process identity and issued
one source-defined local shutdown packet; its retained handle reported exit 0.
This does not reconstruct the earlier parent application exit code. The post-application-exit navigation database
is byte-identical to the baseline and passes integrity/raw-storage comparison.
The final INI has 13 changed keys, including ordinary resized-window persistence
and an appended source-priority observation. A complete cold inspection and independent exact per-key review now pass.
Restoration reversed only the temporary connection byte, restored all four
quarantined DLLs and published the verified adopted baseline. No application
was automatically launched; the next installed session requires a fresh review.
The configured N2K port was absent from the earlier inventory; at 04:06 UTC the
read-only Windows inventory reports **Actisense NGT (COM8)** present and healthy,
alongside USB Serial COM4. Port presence does not establish sensor freshness or
accuracy. Tailscale, SSH and RustDesk remain running automatically. A
closed-copy navigation-database integrity/content baseline is also
recorded privately using the [byte-preserving audit](installer/navigation-preservation-audit.md).

The actual shared profile's `opencpn.ini` was already zero-filled at first
inspection (21,380 bytes; last written 2026-09-20). A same-sized nonzero temporary
INI from the same write interval is preserved as a potential recovery source.
After the official upgrade, the exact verified temporary copy was restored with
an atomic replacement. The corrupt original and candidate remain backed up; all
other profile contents and owner/group/permissions were verified unchanged.
Connection values were not edited and no application was launched. A separate
30,851,001-byte managed-plugin/metadata backup is verified.
[Profile recovery evidence](evidence/beta2-boat-profile-recovery.json).
The original cold, hash-verified recovery set remains complete: 2,123 application files and
539 profile files (1,318,897,252 bytes). The earlier incomplete attempt remains
separate and is not an accepted backup.
A second cold recovery set now preserves verified stock **5.12.4** plus the
recovered working INI: 2,123 application files, 539 profile files and
1,318,975,173 bytes. Every copy and final source inventory matched; independent
executable/INI hashes also match. The original pre-upgrade set remains retained.
[Working-state backup evidence](evidence/beta2-boat-stock-working-backup.json).
Private chart/profile content remains on the boat PC or in ignored local
inspection evidence, never in the repository.

The normal profile includes an output-capable NMEA 2000 connection and enabled
pilot plugins. The active temporary commissioning transaction makes that
connection input-only and quarantines the four reviewed output-capable or
unqualified plugin DLLs before the stock launch. No physical control command
has been sent. Third-party outputs are addressed independently of OpenNav's
own control switch. Saved user diagnostics show prior live
GPS, heading, wind and depth; this is not current-session hardware acceptance.
Reported desktop mode is 1920×1080 and the saved application DPI is 144;
1280×800 physical-display acceptance remains outstanding.

The old Developer Preview portable folder has been retired by an atomic move
into the boat-local recovery archive, with ownership hash checked before and
after and a durable recovery journal. No profile/chart/user file was deleted.
Beta 1 remains available until a known-good replacement can be installed. Six
obsolete download ZIPs (five identical accepted Beta 1 archives and one accepted
Developer Preview archive, 912,982,707 bytes) have also been moved into versioned
boat-local recovery storage. Every ZIP matched its accepted release SHA-256
before and after the atomic move; durable records retain the original location.
No unrelated Desktop content is included in feedback documentation.

### Beta 2 implementation and validation under way

- Production defaults to `XNAV_ENABLE_TEST_FIXTURES=OFF`; deterministic sources
  remain in separate test code. Package/installer self-tests must reject a
  fixture-enabled executable.
- Alerts share the fixed top status area; the primary rail has four visible
  values without a narrow scrolling viewport. Center and the current palette
  have explicit labels. System uses a page rather than a tall popup.
- Route/energy, instruments, settings and manual pilot layouts are being
  refined against the reference. Native and boat visual review are pending.
- Integration fixes cover chart-layout restoration after actual settings
  reconfiguration, whole-metre anchor labels, safe cleanup of XNav-owned anchor
  marks and copied chart context/Go To actions.
- Late-created input-only loopback GPS and actual AIS decoding pass 17 grouped
  Linux object/integration checks in the earlier iteration. The expanded suite
  now passes 21 groups with 18 screenshots, including copied waypoint context,
  compact chart cards, settings return, active-route arrival/completion/deletion
  and stale modal selections. Earlier UTC+2 testing exposed and fixed AIS observation
  clock conversion that made fresh targets appear two hours stale. Four added
  integrated clock tests pass in UTC and UTC+2. Sensors now distinguishes no AIS
  reports, current reports and stale/lost reports using actual target timestamps;
  decoder existence alone does not imply reception. Boat reception remains pending.
- Latest local portable contract suite passes **67/67**, including AIS reception
  health, native-frame/chart-layout geometry, current route-summary selection
  and pairing diagnostic layout observations with native window bounds. The integrated Linux build
  passes **110/110 individual CTest cases** under the prescribed sequential
  invocation (excluding the duplicate upstream aggregate). The separate
  fixture-free Linux product previously passed 110/110 and five loader/resource
  self-test checks. Seven actual-pointer workflow groups pass with eight
  screenshots: orientation, context dismissal, waypoint create/Go To/stop, route
  point entry/Undo, cancellation and named save with a read-only database audit.
  [Chart workflow and route-lifecycle review](design/reviews/beta2-chart-workflows.md).
  These are local development results, not exact-release qualification. Native
  and boat execution of the expanded workflows remains pending.
- Beta 2 installer wizard, versioned maintenance and boat scripts are implemented
  but their new native lifecycle gates have not yet run.
- First CI candidate `15e5a265` exposed a Windows-only line-ending assumption in
  a source-archive test. The corrected candidate `a0af22c248f8a81bb8068ccf3f64bd921478060f`
  was exercised in [CI 36266809347](https://github.com/ThereptileII/Work/actions/runs/36266809347).
  Linux and Windows contract jobs pass. Native MSVC integration builds and all
  102 integrated CTest cases pass, followed by mode/persistence checks. Its
  navigation UI smoke stopped at an old source-caption assertion after alerts
  moved into that header slot; the replacement asserts the critical-position
  alert and retained stale ages explicitly. Six native frame/mode screenshots
  were reviewed in [the first native review](design/reviews/beta2-native-a0af22c.md).
  Complete Windows, packaging and boat gates remain pending. No Beta 2 release
  acceptance is implied; later candidates below supersede it.
- Follow-up source `ee720380ac72b7459f5ff0538acecb9c2b650180`
  ([CI 36269508823](https://github.com/ThereptileII/Work/actions/runs/36269508823))
  matches all 493 local tracked source blobs and file modes. Its Windows boat-tool
  fixture exposed .NET Framework ZIP backslash entries; the fixture now emits
  the same slash paths as real Python packaging while rejection of unsafe ZIP
  paths remains tested. The correction passes 21 isolated native Windows checks.
  Linux build/CTest and navigation, recording, route, marine, Signal K and pilot
  transport gates pass, but the object workflow stopped at compact waypoint
  Details. This candidate was not qualified. The chart-card layout/focus
  correction and repeated local checks below supersede that failed interaction.
- The authorized official-stock upgrade uses a separate reviewed visible-wizard
  driver, not the OpenNav installer or silent replacement. Native PS5.1 passes
  86 policy/helper checks, 21 filesystem groups and 23 profile-preparation groups.
  The bare OpenCPN registration is explicitly identified as the inspected RTL-SDR
  plugin, with complete value/type and uninstaller preservation. Temporary-file
  recovery tests confirmed Windows adds the DACL AutoInherited metadata bit;
  ownership, protection and every ordered ACE must remain exact. The independent
  real-profile recovery subsequently passed 25 native groups and was applied with
  a separate journal, exact candidate hash and complete preservation checks.
- Candidate `06f3bc32879cff4b1822ff53710388b1146d05a7`
  ([CI 36271371549](https://github.com/ThereptileII/Work/actions/runs/36271371549))
  includes the corrected ZIP fixture, X11 pointer-target observations and native
  boat maintenance tests. All 506 local tracked source blobs/modes match the
  published tree. The object harness passed two additional local 17-group runs;
  this candidate was superseded before full qualification.
- Follow-up `a6c15b22c2344a69437e4ef7d9d738fe3f1aed50`
  ([CI 36272268284](https://github.com/ThereptileII/Work/actions/runs/36272268284))
  exposed a Windows Server file-replacement ACL merge in the disposable profile
  preparation suite. It is not an accepted build. Boat Windows 11 recovery had
  already passed exact permission and content verification; the affected helper
  was subsequently hardened for both environments without loosening permission checks.
  Maintenance suites now run in a separate mandatory native CI job, so MSVC/UI
  validation can proceed concurrently. Final publication still requires every
  maintenance suite, and adds the reversible commissioning transaction tests.
- Boat-local disposable PowerShell 5.1 checks now pass 32 profile-preparation,
  21 commissioning transaction and 11 launch-verification groups, plus the
  existing 21 boat-tool groups. Portable counterparts pass 21, 12 and 11 groups.
  Real commissioning has not yet been applied. Its one-byte input-only change,
  reviewed plugin quarantine, interrupted restoration and complete helper-file
  launch verification are independently tested; no application or physical
  command was launched during these checks.
- Product candidate `62e28e5dfe42e96531f00dd88686634c167b0db8`
  ([CI 36273516935](https://github.com/ThereptileII/Work/actions/runs/36273516935))
  matches all 515 committed local source blobs/modes. Both contract jobs pass;
  native MSVC and 102 integrated CTest cases pass, followed by selected-input,
  Signal K and recording checks. The Windows pilot interaction gate stopped at
  an unavailable course-button caption and is under investigation. Linux reached
  the required elapsed-time stability test. Neither platform is yet accepted.
  The maintenance job exposed a Windows Server security-descriptor difference:
  `Get-Acl -Audit` can omit inherited ACE flags in its DACL view. The corrected
  implementation uses the ordinary owner/group/DACL as the preservation baseline
  and the audit view only to reject unsupported SACL metadata. It never copies
  the transformed audit DACL or relaxes ordered-ACE checks. The corrected split
  passes 34/21/11 boat-local temporary-file groups and the same native Windows
  Server suites. Tooling commit `586df3875a8157e17b27da782b131420d9e6fbd6`
  passes all six maintenance/review suites (266 groups) in
  [CI 36274989439](https://github.com/ThereptileII/Work/actions/runs/36274989439).
  This qualifies that tooling revision only, not the application.
  The native window
  review helper passes 93 policy/compilation groups on Linux and Windows;
  actual UI actions remain unperformed. Both restored chart directories and
  the 442,271-byte Windows chart database are present; rendering is still pending.

- Candidate `c8ff99eaf448147a17c02d99f3a71d430763a618`
  ([CI 36275173742](https://github.com/ThereptileII/Work/actions/runs/36275173742))
  includes explicit UTF-8 conversion for native degree/temperature captions and
  readable alert actions. All 528 committed local blobs/modes match the remote
  tree. Its native maintenance job passes, including 16 new source-checkout
  groups. The exact source was retrieved into the managed boat workspace without
  changing or launching the application.
  [Source-only evidence](evidence/beta2-boat-source-c8ff99ea.json).
  Native MSVC and **102/102 integrated CTest cases** passed; the loopback pilot
  interaction also passed, and two native screenshots confirm correct degree
  glyphs. The object gate then failed before its first chart screenshot because
  the harness had left the application at its default **896×532** while testing
  for a 1280×800 layout. This is not an accepted application build. The harness
  now explicitly sizes the native window before validation and checks chart
  dominance, four visible rail values and control separation against actual
  frame/client geometry. It retains coastline checks and captures failure
  evidence without resizing a pending modal. The replacement passed the local
  21-group object run; its native rerun remains required.
  [Native review](design/reviews/beta2-native-c8ff99ea.md) and
  [partial native evidence](evidence/beta2-windows-c8ff99ea-partial.json).
- Tooling-only commit `6f9ee027518307ac373d1080edf38534b1af1ff7` passes
  **288 checks across seven suites** on native Windows Server 2022 / PowerShell
  5.1 in [CI 36275798350](https://github.com/ThereptileII/Work/actions/runs/36275798350).
  This includes source-checkout and launch guards in addition to profile recovery,
  reversible commissioning and window-review policies. All 533 source blobs
  match the published tree. [Tooling evidence](evidence/beta2-windows-tooling-6f9ee02.json).
  This is maintenance-tool qualification only: no application or hardware command
  was launched, and it does not qualify the current product or boat deployment.

- Candidate `6160d3e4bcd924853462f96a32f1b502a72a2884`
  ([CI 36277024981](https://github.com/ThereptileII/Work/actions/runs/36277024981))
  passes native MSVC, **102/102 integrated CTest cases** and the expanded
  21-group navigation-object suite. Later Windows checks fail and no product
  package is accepted or deployed. Pointer diagnostics stop updating after a
  North-to-Course interaction; the Linux reproduction continues updating.
  Bounded, opt-in tracing is added only to the fixture build to distinguish a
  stalled event loop from diagnostic publication failure on the native rerun.
  No speculative navigation behavior change or timeout relaxation is made.
  Separately, the preview helper expected the old destination caption; the
  plugin/route chart checks expected the old flat Settings and route-save flow.
  Those checks now follow the actual UI while retaining geometry, plugin paint,
  route identity and persistence assertions. The 150% rail failure paired
  pre-resize diagnostics with current HWND bounds; the saved native image and
  subsequent independent bounds show all four values fitting. The new barrier
  synchronizes observation times before the same containment/touch assertions.
  [DPI investigation](design/reviews/beta2-dpi-observation-6160.md).
  [Failed native evidence](evidence/beta2-windows-6160d3e4.json) and
  [four-image review](design/reviews/beta2-native-6160.md) preserve the exact
  downloaded artifact; none of these partial results implies acceptance.
  All Linux functional CI steps passed; its three-hour endurance step was later
  canceled by the superseding candidate and is not accepted. The separate fixture-free Linux product passes
  110/110 integrated cases, five loader checks and seven synthetic-data exclusion
  groups. These results do not replace the failed native or pending boat gates.
- Tooling-only `5b597bff0df7295a0ab1f3edbc3d45217582b56d` passes **340 checks
  across seven native Windows suites** in
  [CI 36277973803](https://github.com/ThereptileII/Work/actions/runs/36277973803).
  This extends fixed, read-only waypoint/AIS selection policy; no physical UI
  action or hardware command was executed. Automatic in-app restart remains
  outside the independently audited cold-launch procedure.
  [Tooling evidence](evidence/beta2-windows-tooling-5b597bf.json).

- Candidate `12100a74ff619b7268a6e20902bd9d0de3b53b39`
  ([CI 36279275114](https://github.com/ThereptileII/Work/actions/runs/36279275114))
  passes 67/67 portable contracts on each platform, native MSVC and 102/102
  integrated CTest cases, 340 maintenance checks, all 100/125/150% DPI checks
  and both chart phases. The OpenGL-requested phase used the verified upstream
  software fallback; hardware OpenGL remains open. Rail/alert bounds and
  Legacy/Safe return coastlines pass at each scale. Actual ENC chart switching,
  route creation/editing and plugin-manager paint pass. Pointer Course-up still
  stops periodic updates after the callback has returned; fixture autopilot
  interaction also fails. Packaging, installer and native endurance were skipped.
  This is a failed development candidate, never deployed.
  [Evidence](evidence/beta2-windows-12100a74.json) and
  [four-screen review](design/reviews/beta2-native-12100.md).

- Replacement `b565553d284e91f9aea552b3993428206de5247d`
  ([CI 36281109419](https://github.com/ThereptileII/Work/actions/runs/36281109419))
  passes both 67-contract jobs, native integration, the Course-up pointer gate
  and the complete fixture UI suite. Its fixture-free native build, 100/125/150%
  DPI/touch checks and chart/plugin gates also pass. Software basemap rendering uses a
  copied viewport instead of invalidating the live quilt during paint. Native
  diagnostic JSON is read explicitly as UTF-8. Extracted portable testing fails
  because its startup observer counts earlier initialization markers after
  upstream log rotation; the actual Safe-to-XNav window has coastline content.
  The corrected observer requires a fresh startup/finalization sequence and
  retains chart/clean-exit checks. Installer and native endurance were skipped;
  the candidate has not been deployed. [Failure evidence and repair](evidence/beta2-portable-b565-log-rotation.json).
  The downloaded full native evidence archive matches the API and upload-log
  SHA-256 `a548bac982aa8406b960584dd88b8a52071e04d546a5949da8ba2e0f78e78cb2`.
- A separate plugin-workspace repair restores the normal upstream perspective
  while preserving only XNav-owned temporary panes. It uses exact manager and
  window identity. Local fixture and production builds pass, along with 110
  integrated cases, 22 object groups and software/OpenGL chart checks including
  actual floating Dashboard captures. The native and boat gates remain open.
  [Workspace review](design/reviews/beta2-plugin-workspace.md).
- The optional commissioning restart boundary passes 886 protocol checks and
  359 native assertions across 24 marker-only process scenarios. This is a
  deliberately armed, one-use recheck of the saved profile, plugin trees and
  original process identity before launching a replacement. Normal unarmed
  mode switching is unchanged. The full broker, actual Prepare/Arm procedure,
  packaged application capability and boat transitions require separate gates;
  these standalone results do not authorize an older helper or application.
  [Native transport evidence](evidence/commissioning-restart-fe397.json).
- Current local contracts pass **70/70**, including packaged guard capability
  and startup-log rotation. Guard integration passes 110/110 in both fixture and
  fixture-free Linux builds; the separate actual loader test preserves the
  normal profile. Instrument whitespace refinement retains value fonts and
  data-quality labels and passes the full Linux UI/scenario smoke.
  [Instrument/energy viewport review](design/reviews/beta2-instrument-density.md).
- Actual full restart-broker native tests now pass six marker-only cases and 48
  assertions, including refused output/plugin changes and uncertain consumed
  permits. The scheduled-task policy still needs a native replacement: observed
  Task Scheduler uses an account name for its principal and null for no triggers.
  Both representations must be verified without weakening exact SID/action
  identity. Actual Prepare/Arm/Collect and real application transitions remain
  pending. The separate stock-after-uninstall review tool has portable coverage
  but has not launched stock OpenCPN on the boat.

## Accepted Beta 1 baseline

**Beta 1 software qualified: all ten CI jobs passed, downloaded release verified, native visual review complete.**

Beta 1 (`0.3.0-beta1`) is the software package for desktop and staged boat
commissioning. It is not approved for navigation or production use. Physical
boat acceptance is separate. No autonomous steering is implemented.

Qualified application/source commit: `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a`.
[CI run 36137990012](https://github.com/ThereptileII/Work/actions/runs/36137990012).
[Acceptance and download verification](evidence/beta1-a3e6e08-accepted.json).

OpenCPN remains pinned to **5.12.4 /
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`**. Windows retains the supported
**x86 application/plugin ABI on Windows x64**. The `win64` suffix describes the
host. The pristine upstream checkout is unchanged; the reviewed integration
patch is documented in [upstream patches](upstream-patches.md).

## Download and first test

[Download OpenNavX-Beta1-Windows](https://github.com/ThereptileII/Work/actions/runs/36137990012/artifacts/10876531379).
Extract that outer Actions archive to find:

- `OpenNavX-Beta1-Portable-win64.zip`
- `OpenNavX-Beta1-Setup.exe`
- `OpenNavX-Beta1-Test-Guide.md`
- `OpenNavX-Beta1-Boat-Commissioning.md`
- `OpenNavX-Beta1-source.zip`
- `SHA256SUMS.txt`

Extract the complete portable ZIP to a short writable folder, then run
`Run-XNav-Demo.cmd`. Confirm DEMO and real coastline/chart content. Portable
modes use an isolated profile. Before Setup, back up the normal OpenCPN profile
and follow the [desktop guide](beta/OpenNavX-Beta1-Test-Guide.md). Setup uses
the normal shared profile and runs beside the untouched original executable.
Choose Update for an existing Alpha installation. Alpha's historical install
directory/Start-menu folder remain to preserve upgrade continuity.

## Exact-revision qualification

| Gate | Result |
| --- | --- |
| Portable contracts | 60 suites on each platform; ten additional restart repetitions separately |
| Linux integrated | 106 cases; live input, route, object/AIS/anchor, recording, pilot, recovery, UI and chart gates |
| Native Windows MSVC | 98 cases; same functional boundaries, extracted portable launch and three recovery cycles |
| Installer | 29 real lifecycle checks, including accepted Alpha upgrade and failure recovery; 39 filesystem checks in each 32/64-bit PowerShell 5.1 host |
| Display | Real 1280×800 at 96/120/144 DPI; mouse, injected touch tap/pan, fullscreen, Night and mode returns |
| Charts/plugins | Public NOAA ENC plus deterministic coastline checks; Dashboard, GRIB and WMM; Windows software fallback, Linux software/llvmpipe OpenGL |
| Endurance | Three actual hours each; Linux 10,800.175 s / Windows 10,800.109 s; 1,080 samples and 540 page actions each |
| Download/review | Five release hashes, 1,009 portable file hashes and 4,814 source ZIP entries verified; 40 native Windows and three Linux images individually reviewed |

Repeated tests are not counted as additional distinct cases. Native Windows
images are authoritative; injected touch does not establish physical touchscreen
acceptance. Hosted Windows rejected hardware OpenGL, so target GPU testing
remains open. Exact measurements and screenshot hashes are in the evidence. Sustained resident
growth was 184 KiB on Linux and 2.97 MiB on Windows; Windows private growth was
3.73 MiB. Linux descriptors/threads and Windows handles/GDI/USER had zero median
growth. Mean CPU was 1.65% / 1.10% of one core respectively. Maximum page
observation was 1.25 s / 1.73 s, including the 1 Hz diagnostic observation delay;
these are not paint-latency measurements or a universal leak-free claim.

## Implemented Beta product

| Area | Implementation and boundary |
| --- | --- |
| Navigation/shell | Real chart; touch controls, follow/orientation/measure, contextual pages, configurable rail, Day/Dusk/Night, fullscreen, persistent alerts and Legacy/Safe restart. |
| Routes/waypoints | OpenCPN-owned storage/progress, guarded basic creation/edit/activation/reversal, next point and remaining-distance snapshot. Advanced/protected cases remain in Legacy. |
| Marine data | Selected OpenCPN navigation plus NMEA 0183, 15 supported N2K PGNs and own-vessel Signal K; physical NAME, source precedence/age/cadence and explicit invalid/unavailable/uncertain state. |
| Propulsion/battery | Standard marine acquisition plus explicitly bound boat adapter; independent producer-expiry contract; high-voltage 127751, SOC 127506, gear/regen and motor-temperature meaning where configured. No PC Leaf CAN decoder. |
| Energy | Explicit capacity/reserve/sign/auxiliary assumptions, empirical curve import, filtered speed/power calibration export, advisory route energy/range/arrival SOC with quality and blocking reasons. |
| Commissioning | Per-item health/PGN/instance diagnostics, bounded opt-in recording, deterministic REPLAY with historical ages and controls disabled, privacy-limited field diagnostic ZIP. |
| SmartNav/AIS | Turn/timeline/energy advice, OpenCPN CPA/TCPA/alarm context, modern target selection/card/detail and expiring chart highlight. No autonomous command path. |
| Anchor/alerts | Normal OpenCPN anchor watch, distance/history/depth/wind/battery and a persistent actionable alert layer. No invented drag prediction or automatic standby. |
| Manual pilot | ST4000 adapter, exact identity binding, bidirectional TCP Actisense transport, six manual commands, feedback/timeout/rate limits and every-start OFF. Simulator retained; TRACK/WIND unavailable. |
| Hazards/radar | Tested bounded corridor/provider architecture and unavailable/uncertain semantics. No accepted live ENC corridor provider or Pathfinder display/control adapter. |
| Robustness | External JSON/input bounds, malformed/stale/reconnect tests, two-failure startup Safe fallback, serial-discovery resource ownership and installation failure recovery. |
| Distribution | Hash-gated side-by-side per-user Setup; real Alpha update, immutable generations, repair/rollback/uninstall; stock executable/shared profile preserved; isolated portable recovery. |

Contracts: [marine input](marine-input-contract.md), [boat producer](boat-propulsion-contract.md),
[recording/calibration](recording-replay-contract.md), [energy](energy-model.md),
[pilot](st4000-beta-contract.md), [AIS](ais-beta-contract.md),
[alerts](operational-alerts.md), [display](display-beta-contract.md),
[robustness](beta-robustness.md), [installer](installer-transaction-contract.md).

## Stable foundation and feedback

The reported Legacy → XNav blank-chart regression remains closed by `16dbaf7`
and repeated exact-release coastline/ENC/mode-return checks. The active-leg
console overlap was closed in accepted Alpha `08bc92f`. The user's Alpha
acceptance and deliberate deferrals are recorded in [the Beta plan](beta1-plan.md).
The immutable remaining-route contract retains normal upstream active-point
range plus subsequent stored legs; no independent navigation calculation.

Earlier accepted increments and rejected candidates remain in
[development history](status-history-beta-development.md) and [evidence](evidence/).
Failed runs are not release acceptance.

## Post-build visual review and deliberate deferrals

- Diagnostics retains an old **ALPHA 1 / NOT FOR NAVIGATION** warning caption.
  The separate Beta 1 version, commit and compiler fields are correct. This is
  a cosmetic label issue, recorded for the next feedback-driven increment.
- At 150% the transient System popup partly covers the Alerts button. Critical
  alert text remains visible; press Escape or click outside to dismiss the
  popup before opening Alerts. This presentation refinement is deferred.
- With larger DPI or an alert, use Up/Down or pan to reach lower rail/page
  values. The heading card can be partly outside the viewport even at 100%
  with an alert; configure the rail order to prioritize desired instruments.

These acceptance notes follow the build; they do not change the downloaded
binaries or their bundled guides. Of the 78 automated DPI captures, a subset
is included in the 40 individually reviewed Windows images. Fullscreen was
1920×1080; normal window gates were 1280×800.

## Remaining physical/product gates

Follow [boat commissioning](boat-commissioning.md): read-only first, propulsion
comparisons/calibration next, pilot status next, then individual deliberate
commands in a secured safe environment, and supervised underway trials last.
The C6 producer patch compiled but was not flashed. The complete PC → gateway
→ ESP32 → SeaTalk → ST4000 path still needs physical feedback/loss validation.
Physical touch, target GPU/navigation PC, broader plugins, radar hardware, live
ENC corridor integration and at-sea operation remain open. Beta is unsigned;
native/Legacy dialogs can remain bright. [Known limitations](beta/KNOWN_LIMITATIONS.md).

Stop at Beta 1 for user/boat feedback. No production-release work or autonomous
steering is authorized by this acceptance.
