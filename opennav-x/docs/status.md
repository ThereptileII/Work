# OpenNav X status — 2026-09-29

## Current stage: prototype design lock, chart presentation and Online AIS

Source Health now uses the prototype's disclosure sheet with separate onboard
and Online AIS status, per-measurement quality and a direct path to the user's
protected AISStream key settings. Incomplete GPS pairs and estimated readings
no longer appear as healthy measured connections. Linux passes 133 product
regressions, 133 fixture regressions and 49 component checks/six captures, with
two actual-product captures and retained comparisons. The retained preview
passes eight groups/44 captures and AIS own-key regressions pass 151 checks. Native replacement and
boat review remain open. See [Source Health review](design/reviews/prototype-source-health-in-progress.md).

Native `60a9e8b` / run `36532671500` finishes seven jobs passed and one
failed. The product passes 124 MSVC tests, 151 own-key checks/twelve captures,
29 product captures, software ENC/theme/unit checks and 100/125/150% development
DPI probes. The fixture job passes chart gestures, shared-profile mode switching
and all eight preview groups/44 captures, then fails the Preferences scroll
stability check at 100%. No failure is waived. The replacement pairs native HWND
bounds with a later diagnostic observation before the unchanged endpoint check;
ten portable observation tests pass, native replacement is pending. All nine
archives verify. Full run `36532672143` remains pending. Deployment is withheld.
See [native development evidence](evidence/prototype-native-60a9e8b-development.json).

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
