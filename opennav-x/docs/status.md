# OpenNav X status — 2026-09-25

**Alpha 1 (`0.2.0-alpha1`) is accepted as the software baseline. Beta 1 is in development.**
The user reports Alpha manually tested and authorizes real-boat integration and
release hardening. No navigation certification or physical boat acceptance is
claimed. [Beta plan and feedback disposition](beta1-plan.md).

The exact Alpha download is commit `08bc92fc778591a3978e594a54208d672ff9158f`,
[run 36092747986](https://github.com/ThereptileII/Work/actions/runs/36092747986).
All nine jobs pass. The downloaded artifact, its four release-file hashes,
1,000 portable file hashes, source archive, build identity and x86 ABI were
verified. Twenty-eight native screenshots and three Linux screenshots were
reviewed. [Acceptance evidence](evidence/alpha1-08bc92f-accepted.json).

OpenCPN remains pinned to **5.12.4 / `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`**.
Windows keeps the supported **x86 application/plugin ABI on Windows x64**.
The `win64` download suffix describes the host, not a new plugin ABI.
The pinned checkout remains pristine; reviewed patches apply to a disposable
integration worktree. See [upstream patches](upstream-patches.md).

## Stable foundation

The reported Legacy → XNav blank coastline regression is closed at
`16dbaf72d7923742a5acaadf0fae42ee490e7cf6`,
[run 36047288188](https://github.com/ThereptileII/Work/actions/runs/36047288188).
All eight jobs pass: 67 Linux / 57 Windows integrated cases, ten portable suites
per platform and ten additional restart repetitions. Seven Linux and nine native
chart-content checks pass across startup/mode cycles; all nine replacement
native captures were reviewed. The downloaded ZIP's 997 file hashes and exact
executable match the tested build. [Foundation evidence](evidence/windows-foundation-16dbaf7-review.json).

The immutable remaining-route contract was previously accepted at `954b450`:
normal OpenCPN active-point range plus subsequent stored route legs; separate
position/observation ages and route/revision/point identity; invalid, stale,
ambiguous, edited or transitional state never becomes a valid zero arrival.
[Contract](route-progress-contract.md), [review](evidence/windows-954b450-review.json).

## Beta development

Candidate `92a695f` / run `36130393824` is **not accepted**. The native
portable package passed and all new installer failure/recovery cases preserved
stock/profile, but the missing-DLL loader fixture left a Windows System Error
dialog covering the original OpenCPN welcome button at the final post-uninstall
check. The installer now directly launches the self-test with a scoped inherited
noninteractive error mode and restores the parent setting. The strengthened
fixture requires prompt loader failure without modal residue. Two native
error-mode restoration checks supplement the existing filesystem suite.
Exact replacement qualification remains mandatory.
The 150% Menu endpoint repair passes; the later route-page check exposed an
outdated fixed-height test threshold with the new visible alert strip. The test
now verifies the actual available center bounds and unobscured content.

Candidate `0d4bcb7` / run `36127208962` is **not accepted**: both builds/core
functional gates passed, but a fresh GitHub API check found the extracted Windows
portable interaction step failed. An earlier cached status feed was stale.
The long endurance stage must now run only after all earlier Windows gates pass;
portable/DPI failure captures upload immediately. Superseded qualifications are
cancelled without publishing a package. Exact replacement evidence is required.
The downloaded job log identifies a test-visible page-label collision after Alpha
wording removal: product and preview panels shared a prefix, invalidating the
strict overlap assertion. Product panels now have a distinct role label; the
visibility/overlap assertion remains, and Navigation checks that both panel roles
are hidden. The one-time job which retired three known failed runs is removed;
normal branch concurrency now prevents superseded endurance jobs accumulating.


Release qualification is still in progress. A Linux allocation profile identified
an upstream libudev serial-discovery leak; the reviewed integration patch now
owns/frees contexts, scans and devices. Three actual-reference tests cover normal
and failed discovery. Earlier short endurance passes are not treated as the
required three-hour release result. Native permission-fixture comparison now
checks exact ACE bytes/protection while allowing Windows' observed
`SE_DACL_AUTO_INHERITED` bookkeeping flag. Candidate `25e96f6` remains unaccepted
because its string-only DACL assertion failed; recorded ACEs were identical.


The Beta packaging/display candidates `106b7a3` / run `36122795750` and
`8236ee5` / run `36123576221` are **not accepted**. The longer endurance
harness exposed an incorrect test expectation: the unavailable-instrument demo
retains GPS/route data, so only dependent energy advice must disappear. The
replacement requires that distinction explicitly and exercises repeated cycles.

Hardening revision `f562f3805f1296fa5123fc0cbe2a684d6c0b4c15`,
[run 36121533524](https://github.com/ThereptileII/Work/actions/runs/36121533524),
passed 60 portable suites on each platform, 103 Linux / 93 Windows integrated
cases and actual Signal K, N2K, recording, pilot, AIS and recovery gates. It is
**not accepted**: native 150% scaling exposed a menu endpoint focus jump, and the
installer permission fixture failed before testing the actual denied operation.
The replacement focuses the scroll viewport without selecting a child; native
tests require the last menu action to remain fully visible at all three scales.
The permission fixture now uses the native .NET Framework DACL API, saves/restores
the exact DACL and retains error details. Two early native filesystem checks
verify actual directory denial and restoration before the installer matrix.

Beta packaging targets `0.3.0-beta1`, exact Beta download names and separate
desktop/boat guides. The installer upgrade fixture is the hash-verified **actual
accepted Alpha Setup**, not a relabeled current executable. Alpha installation
identities remain stable. The Energy card includes approximate ETA from the
existing tested arrival model. Exact-commit native acceptance and three-hour
endurance on both platforms are still required; no Beta release is accepted yet.

Release hardening adds bounded pre-parser Signal K validation, malformed marine
input cases, control-byte rejection in source identities, seven additional
installer failure/recovery checks and an actual elapsed-time resource harness.
The two-minute Linux harness checkout passed; it is not the required three-hour
release result. Short development CI selects 120 seconds. The release candidate now selects
10,800 seconds on each platform; named publication depends on every gate. [Robustness contract](beta-robustness.md).

Display revision `84f7122` / run `36117089450` is **not accepted**: compilation,
portable tests, recording and pilot checks passed, but AIS runtime gates exposed
selection being cleared by sub-millisecond clock-conversion jitter and an
advisory fixture racing the upstream alarm timer. Stable per-report observation
epochs and a continuing isolated fixture repair those issues; the Linux actual
object/card/chart gate now passes. Native replacement is mandatory. The Linux
System popup test also now supplies the focus and resize settling normally
provided by a window manager; explicit Escape handling releases its pointer grab.
The full local preview passes with the popup visible. No failed run is promoted.
The replacement local gates pass **60 portable suites / 103 integrated cases**,
actual AIS selection/advice and real Signal K input/failure/recovery. Signal K
now reuses OpenCPN's RapidJSON dependency after the loopback gate found wxJSON
rejecting valid escaped Unicode source labels. [Local evidence](evidence/beta-hardening-local.json).

Night/touch hardening now uses shared gesture/button scrolling without native
scrollbars, preserves full instrument-card height and keyboard focus, reserves
space for global alerts around sheets, and exposes Pilot plus a fixed manual
STBY control. The software basemap uses OpenCPN's own night land/water palette.
The System popup is compact enough for the 150% workspace. Local 60 portable
suites / 99 integrated cases, 13 primary night-surface checks and actual
recording/replay pass; final pilot/native results are recorded separately.
Native tests now scroll actions into view, check card height, exercise touch
pan, full-screen return and System popup bounds at 100/125/150%.
[Display contract](display-beta-contract.md), [local evidence](evidence/beta-display-local.json). Windows remains authoritative;
this paragraph is development progress, not Beta release acceptance.

The AIS increment at `ac6fafa` / run `36114659033` passed both 60-suite contract
jobs but its native build stopped before compilation on a CRLF-converted patch
context line. The replacement preserves valid unified-diff prefixes and uses
one LF-normalized stream for check, application and exact-source verification.
No hunk is ignored, and the failed native run is not accepted.


AIS target cards now select/center the existing chart target with an expiring,
owned selection. A shared observation epoch fixes live AIS advisories being
incorrectly rejected as future data. SmartNav suppresses route-dependent advice
on current GPS loss, incoherent sources, stale fixes and route transitions;
newer coherent GPS fixes remain usable with fresh upstream route progress.
Hazard provider failures and excessive/untrusted results degrade explicitly.
The existing ENC query APIs remain insufficient for a complete corridor; no
safe-route claim is made. Local **60 portable suites / 99 integrated cases**,
actual AIS selection/advice, preview/mode and manual-pilot checks pass.
[Contract](ais-beta-contract.md), [chart/radar boundary](beta-chart-radar-boundaries.md),
[local evidence](evidence/beta-ais-advice-local.json). Native replacement pending.

Run `36112601192` (`5b08227`) passes native compilation and the official
prerequisite wizard, but is rejected at recording interaction: an asynchronous
Ctrl+A selected/deleted a filename prefix during the test driver's WM_CHAR
stream. The driver now selects synchronously, retains normal filename change
notifications and exact full-path verification. No artifact from that failed
run is accepted, and no product data or file validation is weakened.


Recording/replay/calibration export, physical N2K NAME identity and the manual
pilot loopback path are accepted as a software increment at `9a1872f`,
[run 36106701545](https://github.com/ThereptileII/Work/actions/runs/36106701545).
All nine jobs pass: 47 portable suites on each platform, 98 Linux / 88 Windows
integrated cases, actual native export and pilot feedback/reconnect checks,
22 installer lifecycle checks, charts and 100/125/150% DPI/injected touch.
Eight new native captures were reviewed; artifact hashes were verified.
[Acceptance](evidence/beta-recording-pilot-9a1872f-accepted.json). This supersedes
the earlier recording/SDK failures below; boat expiry and field ZIP remain
separate pending increments. No physical acceptance is claimed.

The first Beta increment adds standard 127751 voltage/current, 127489 coolant,
127493 gear, other tank instances and source cadence/invalid-input diagnostics.
Accepted at `a3717673674fdcd7ad8126ea3f24e354139b13d3`,
[run 36096824548](https://github.com/ThereptileII/Work/actions/runs/36096824548):
97 Linux / 87 Windows integrated cases, 31 portable suites on each platform,
TCP N2K loss-of-data smoke, mode/chart/recovery/installer/DPI gates pass.
The three new native live/stale/unavailable captures were reviewed.
[Evidence](evidence/beta-input-a371767-accepted.json).
Recording/replay, calibration export and the commissioning overview now pass
38 portable suites, the Linux integrated build/97 existing cases, actual recording
UI, N2K loss-of-data and mode-cycle checks. Native Windows acceptance is pending.
[Recording contract](recording-replay-contract.md),
[local evidence](evidence/beta-recording-local.json).
The first native recording attempt `6114f1e` compiled but rejected two portability
cases (canonical TEMP path aliases and a text-mode test fixture). Corrections
pass all 38 local portable suites; native replacement evidence is required.
[Failure/repair record](evidence/beta-recording-6114f1e-failure.json).
Replacement `c123d47` passes all 38 contract suites on Linux and native Windows;
native MSVC and all 87 integrated cases also pass. Its actual recording/replay/
stale/pause/rewind/stop checks pass, but calibration Save produced no requested
file. The gate remains rejected while file-picker interaction is corrected and
additional failure evidence is collected.
[Failure record](evidence/beta-recording-c123d47-failure.json).
The interaction test also exposed and fixed first-open pane wrapping and focus
after replay changes; native scaling remains a required gate. [Source inspection](beta-boat-source-inspection.md)
records the actual producer contract and its retained-data freshness limitation.

The next internal increment adds a copied-data ST4000 protocol adapter, explicit
device binding, feedback/timeout/isolation rules and anti-repeat commands.
All 44 local portable suites and 97 integrated Linux cases pass, including commands checked by the actual
hash-pinned boat firmware parser. It is not yet connected to the application's
live transport and does not claim physical acceptance.
[Pilot development contract](st4000-beta-contract.md),
[local gate](evidence/beta-pilot-protocol-local.json).
Both native and Linux contract jobs now pass the 44-suite protocol increment at
`2de6238`, including the actual firmware parser. Full runtime acceptance still
depends on the pending recording export correction.

A source inspection and native JSON review exposed synthetic OpenCPN address
labels incorrectly used as physical NAME, splitting one battery across PGNs.
The repair observes real 60928 claims, keeps SOC and V/I on one copied pack
identity, clears reassigned samples and rejects duplicate NAME. Local 45 portable
suites, 98 integrated cases and actual TCP identity/change/conflict/loss checks
pass; recording/replay still passes locally. Native qualification is pending.
[Identity evidence](evidence/beta-n2k-identity-local.json).

The live pilot increment now uses an existing bidirectional OpenCPN TCP/Actisense
complete-PGN ASCII connection. Exact observed NAME binding and saved permission
remain separate from session enablement; control starts OFF. The actual loopback
UI/transport test exercises AUTO, all four heading steps, STANDBY, missing-feedback
timeout and same-driver reconnect without any physical hardware. New read-only
network provenance hooks invalidate old identities and queued observations.
Other transports remain status-only; TRACK/WIND output is unavailable. Native
integration and final replacement visual evidence remain mandatory before Beta
acceptance. The recording dialog gate at `11c0a5c` reached Save but its screenshot
helper incorrectly attempted to resize a native file dialog; the helper now
preserves the dialog dimensions. No export pass is claimed from that failed run.

Native pilot run `36104989528` at `07f4248` rejects the integrated build because
the Windows SDK's COM `interface` macro expands a new C++ field name. Portable
contracts on both platforms pass but do not include that SDK header context.
The replacement renames the field to `interface_id` without changing serialized
configuration keys and adds `windows_sdk_macro_contract`. Native replacement
build/runtime evidence is still required; the failed build is not accepted.

The boat mapping increment binds an actual marine NAME, maps the inspected motor
temperature field, suppresses its virtual SOC fuel tank, and decodes regeneration.
Its reviewed boat-side firmware patch supplies independent 2500-ms sensor expiry
and a v2 marine freshness contract. The PC withholds dependent predictions from
v1/unverified input. The full sketch compiles for XIAO ESP32-C6, with no flashing.
Local 51 portable suites (including both actual firmware oracles), 99 integrated
cases and real OpenCPN loopback expiry tests pass in development; the final source
and native replacement gates remain open. [Contract and firmware procedure](boat-propulsion-contract.md), [local evidence](evidence/beta-boat-expiry-local.json).

Beta field-report development adds a bounded local diagnostic ZIP with copied
source health, numeric assumptions, adapter state and a transition journal.
Position/device/route identities and arbitrary files are omitted by default;
recording inclusion requires explicit selection and consent. The System popup
now reports actual pilot enablement instead of the obsolete unconditional OFF
label. All 54 portable suites, 99 Linux integrated cases and actual UI ZIP export/recording regression pass. [Local evidence](evidence/beta-field-report-local.json). [Privacy/export contract](field-diagnostic-bundle.md). Native acceptance
is pending. Boat run `36107966976` (`2272759`) compiled the ESP32-C6 firmware but
Windows preparation rejected a CRLF patch checkout. The replacement normalizes
only the temporary patch and verifies the exact patched source hash; a local
CRLF reproduction passes. The failed native run is not accepted.

Energy diagnostics now identify the specific blocking input and distinguish
current, aging, estimated and unavailable input quality without claiming forecast
accuracy. Local 55 portable suites, 99 integrated cases, all preview scenarios/
mode cycles and actual boat expiry input pass. Native replacement acceptance
remains required. [Energy contract](energy-model.md),
[local evidence](evidence/beta-energy-quality-local.json).

The boat-expiry/field-ZIP replacement `2da6791` passes both 54-suite contract
jobs, 99 Linux / 89 native integrated cases and actual boat/ZIP/replay checks.
Five new native captures were reviewed. The complete run remains **rejected**:
the stock prerequisite wizard exposed a partially built Finish page before its
completion text. Energy run `6cc9b9e` exposed the corresponding destination-page
race in the separate prerequisite job. Automation now waits for stable required
controls while preserving all destination/hash/resource/registration checks.
[Failure and scoped visual evidence](evidence/beta-boat-field-2da6791-review.json).

Operational alert development adds an always-visible condition strip, actionable
list, episode-scoped acknowledgement and independent sensor-loss/pilot/AIS/anchor/
energy conditions. It cannot acknowledge upstream alarms or emit a command.
Anchor now also displays battery SOC. Local 58 portable suites, 99 integrated
cases and actual page/acknowledgement/recovery scenarios pass; native replacement
acceptance remains pending. [Alert contract](operational-alerts.md), [local evidence](evidence/beta-alerts-local.json).

## Implemented Alpha product

| Area | Current implementation and boundary |
| --- | --- |
| Shell/navigation | Real chart canvas, touch controls, follow/orientation/zoom/measure/context entry, source health, configurable rail, full-screen and Day/Dusk/Night. Legacy and Safe remain available. |
| Routes/waypoints | Owned catalogs, guarded activation/deactivation/reverse, basic chart creation/edit, waypoint name/description and confirmed isolated deletion. OpenCPN owns model, storage and progress; protected/advanced cases retain Legacy. |
| AIS/anchor | Existing OpenCPN target/CPA/TCPA state in modern cards/list; no independent collision logic. Existing anchor-watch range/alarm with owned history, depth/wind and human controls. |
| Instruments/sources | Selected OpenCPN position/SOG/COG; normalized N2K/NMEA0183/Signal K sensor inputs through the existing bus, precedence/pinning, provenance, observation age and unavailable/stale states. No second transport stack or PC Leaf CAN dependency. |
| Energy | Live-ready battery identity/sign/capacity/reserve configuration, strict optional speed/power curve and tested snapshot consumers. Dependent estimates suppressed for missing/stale/invalid inputs; net battery and motor power remain distinct. |
| SmartNav | Owned route steps, next turn, timeline, energy and existing AIS context advisories. Tested future chart-corridor abstraction; live chart hazard query remains unavailable. No absence-of-hazard safety claim. |
| Autopilot/radar | Manual pilot interface, explicit DEMO simulator, capability/status/feedback/timeout logging, global disable. No live output adapter or SmartNav steering path. Radar capabilities/status abstraction; live radar unavailable. |
| Settings/diagnostics | Shared-profile validated settings, source policies, empirical curves, explicit propulsion mappings, display selection and advanced OpenCPN access; build, mode, age, provenance, model and plugin/chart diagnostics. |
| Recovery | Two unfinished XNav starts select Safe before optional modules; retry evidence retained. Deferred-notice repair passes three native recovery cycles; see evidence below. |
| Distribution | Portable isolation and Alpha labels/source/license packaging; native NSIS per-user side-by-side setup, immutable generations, repair/update/rollback/uninstall and fault-recovery engine implemented. Full native lifecycle and downloaded release validation passed at `08bc92f`. |

Contracts: [sources](vessel-source-contract.md), [marine input](marine-input-contract.md),
[energy](energy-model.md), [SmartNav](smartnav-alpha-contract.md),
[hardware](hardware-adapter-contract.md), [settings](alpha-settings-contract.md),
[explicit propulsion mapping](propulsion-source-mapping.md),
[recovery](startup-recovery.md), [installer](installer-transaction-contract.md).

## Accepted internal gates and current evidence

Core source/energy gates passed at `0e65cd5`; advisory/adapter contracts passed
at `82efd08`; marine codecs/loopback passed at `785aa45` with 81 Linux / 71
Windows integrated cases and 22 portable suites per platform. These are internal
milestones, not an Alpha release. [Marine acceptance](evidence/alpha-marine-785aa45-gates.json).

Current candidates retain **90 Linux / 80 native Windows integrated cases**
and **30 portable suites per platform**, with ten additional restart
repetitions. The source, object/AIS/anchor, synthetic data and energy regressions
remain enabled. Their platform-specific totals must not be added to repeated
runs as if these were distinct tests.

The recovery-notice ordering repair passes at `fe37250` on both platforms,
including three independent native forced-crash → Safe → human XNav retry
cycles. Each requires actual modal dismissal, an enabled parent and visible
coastline. Three corresponding images were reviewed.
[Recovery acceptance](evidence/recovery-fe37250-gate.json).

Linux passes software and llvmpipe OpenGL with two hash-pinned public NOAA ENC
cells, real quilt-reference switching, overlays and mode returns. Native
Windows passes chart creation, point dragging, zoom/pan/follow, cell switching
and software fallback. The hosted Windows driver rejects hardware OpenGL;
physical GPU validation remains open.
[Chart review](evidence/windows-chart-1460c05-review.json),
[Linux chart evidence](evidence/alpha-two-cell-chart-local.json).
No blank/all-water image is accepted as known chart-content evidence.

The strengthened native plugin-manager gate passes at `93a8491`: Dashboard,
GRIB and WMM load, the list and Ok/Cancel/Apply controls are fully painted,
and actual Cancel dismissal succeeds. The image was reviewed.
[Plugin acceptance](evidence/windows-plugin-93a8491-review.json).
Native 100/125/150% DPI and injected touch checks also pass on this candidate;
final packaged-revision screenshots still require review.

The exact official OpenCPN prerequisite is installed by its visible native
wizard and verified against executable/resource/uninstall-registration
postconditions. Its narrowly observed post-completion exit 1223 is documented;
Alpha Setup and maintenance still require exit zero.
[Prerequisite gate](evidence/installer-stock-8ae303c-gate.json).

The complete native installer lifecycle now passes at
`7bc36e426a55926045ea1aece0ebe96ef9417863`,
[run 36088505518](https://github.com/ThereptileII/Work/actions/runs/36088505518),
with **all nine jobs successful**. This covers real wizard install/cancel,
registry discovery, unsupported-hash/ownership refusal, distinct prior-version
update, owned-file repair, first-install and prior-generation rollback, both
injected transaction failures, diagnostics, conventional uninstall and launch
of untouched official OpenCPN with coastline and shared fixtures preserved.
Custom Unicode harmonic sources survive generation changes and uninstall.
Thirty portable suites pass on each platform; 28 native filesystem/loader-wrapper
checks pass in each PowerShell 5.1 host. The actual application lifecycle remains
separate from the helper fixtures.

The downloaded native evidence archive was hash-verified. Ten native images were
reviewed, including restored stock, installed mode return, real ENC chart/edited
route return, plugin manager, Setup and 125/150% layout. The public compatibility
manifest now admits **only the tested stock executable SHA-256**:
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
[Qualification and review](evidence/alpha-installer-7bc36e4-qualification.json).
Earlier failure records remain under `docs/evidence/`; none is release acceptance.

Visual review of the manifest-bearing `32a6564` candidate found the native
active-leg console overlapping the XNav rail during real route activation.
It is **not accepted for delivery**, even if its earlier automated checks pass.
The replacement suppresses only that native widget in XNav and adds a real
route/widget regression which fails before the fix. Legacy/Safe keep the original
callback; route calculations and output processing remain unchanged.
[Visual finding](evidence/alpha-console-32a6564-review.json).

## Alpha delivery and Beta gates

The final console repair passes the exact packaged revision on Linux and Windows,
including 26 real route-state checks and the widget visibility assertion.
[Download Alpha](https://github.com/ThereptileII/Work/actions/runs/36092747986/artifacts/10847446708):
the outer artifact **OpenNavX-Alpha1-Windows** contains
`OpenNavX-Alpha1-Portable-win64.zip`, `OpenNavX-Alpha1-Setup.exe`, the test guide,
source archive and `SHA256SUMS.txt`. Earlier candidates are superseded.

Beta must retain all Alpha gates and add live commissioning, recording/replay,
feedback-confirmed manual control, failure/security tests, several-hour soak,
and exact packaged-revision Windows acceptance. Beta is not yet delivered.

The installer intentionally integrates **beside** the original OpenCPN instead
of replacing its executable. Installed modes use the normal shared profile;
portable modes retain an isolated profile. See [architecture decisions](installer-alpha-design.md).

Physical N2K/boat propulsion validation, live pilot output, radar integration,
chart-corridor queries, physical touch/target GPU performance, target navigation
PC and at-sea trials remain explicit open hardware/product gates. No hardware
success or navigation approval is implied by a simulator or hosted CI result.
[Known limitations](alpha/KNOWN_LIMITATIONS.md),
[manual test guide](alpha/OpenNavX-Alpha1-Test-Guide.md),
[physical procedures](physical-validation.md).

Historical accepted preview records and unsuccessful Alpha candidates remain
in [the status archive](status-history-2026-09-25.md) and [evidence](evidence/).
