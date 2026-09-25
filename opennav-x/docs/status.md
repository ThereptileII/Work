# OpenNav X status — 2026-09-25

**Alpha 1 (`0.2.0-alpha1`) has passed native installer qualification.
The final manifest-bearing download is undergoing release validation.** It is
not approved for navigation. The final installer/portable artifact must pass
the same-commit gates and download verification below.
The user accepted the Developer Preview direction and authorized the complete
[Alpha stage](alpha1-plan.md), including the Alpha installer.

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
| Distribution | Portable isolation and Alpha labels/source/license packaging; native NSIS per-user side-by-side setup, immutable generations, repair/update/rollback/uninstall and fault-recovery engine implemented. Full native lifecycle qualification passed; final release artifact validation remains open. |

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

## Remaining Alpha delivery gates

1. Build the manifest-bearing release commit on Linux and native Windows;
   retain every existing functional, recovery, portable, DPI/chart/plugin and
   installer lifecycle gate.
2. Download the actual `OpenNavX-Alpha1-Windows` artifact and verify its portable
   ZIP, Setup, source archive, guide and SHA256SUMS against the packaged commit.
3. Review exact final native screenshots and record the accepted revision/run.
   Candidate artifacts do not replace the final named download.

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
