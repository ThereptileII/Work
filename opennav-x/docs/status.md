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
