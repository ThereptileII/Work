# OpenNav X status — 2026-09-25

**Alpha 1 (`0.2.0-alpha1`) is implemented and undergoing native qualification.
It is not released or approved for navigation.** The final installer/portable
artifact must not be published until the remaining gates below pass together.
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
| Distribution | Portable isolation and Alpha labels/source/license packaging; native NSIS per-user side-by-side setup, immutable generations, repair/update/rollback/uninstall and fault-recovery engine implemented. Full lifecycle qualification remains open. |

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

The actual Alpha preflight, registry discovery and unknown-hash/ownership refusal
pass. Full installation remains under qualification. Inspection reproduced
rejection of the bundled `ca@valencia` locale path, now corrected without
weakening traversal/reparse protections. At `2803773`, **28 actual filesystem
and loader-wrapper assertions pass in each native PowerShell 5.1 host (x86 and
x64)**. The loader fixture is explicitly a fast wrapper test; it does not replace
real installed OpenCPN acceptance.
[Payload-path reproduction](evidence/installer-locale-fe37250-reproduction.json),
[fast native gate](evidence/installer-filesystem-2803773-gate.json).
At `2803773`, clean native wizard installation, installed coastline launch and
first-install rollback now pass. The later launch exposes a persisted tide-data
path into the deleted generation. Installed resource defaults now resolve to the
verified original stock resources, preserving all custom selections. The new
contract passes 30 portable suites on Linux and native MSVC. The dedicated
post-locale hook also passes actual Linux XNav/Legacy/Safe harmonic loading with
Unicode paths, removed generations and a retained custom source list. The full
native lifecycle qualification remains pending. Its `264b5a1` check compared
the long Windows path with a short-name alias; the replacement requires
filesystem identity rather than spelling equality.
[Alias evidence](evidence/installer-path-alias-264b5a1-failure.json).
[Resource lifetime failure and repair](evidence/installer-resources-2803773-failure.json).
At `931490f`, install, update, repair, rollback, both interrupted-transaction
recoveries, diagnostics and conventional uninstall pass with stock/profile
preservation. The final untouched-stock launch awaits acknowledgement of the
normal upstream version-change safety notice and its chart/data checks.
[Stock-return evidence](evidence/installer-stock-return-931490f-failure.json).
The complete lifecycle, exact final artifact and public allowlist remain gated.

## Remaining Alpha release gates

1. Retain the repaired native recovery and accepted plugin-manager
   paint/click gates (including upstream Ok capitalization); retain
   the passed chart-edit/switch/software-fallback and Linux OpenGL coverage.
2. Complete actual official-stock installation plus Alpha install, prior-version
   update, repair, rollback, interrupted transaction, uninstall and restored
   stock launch with exact hashes and shared-profile fixtures unchanged.
3. Review the exact candidate's native screens, 100/125/150% DPI and installer
   screenshots; retain measurable chart content and plugin checks.
4. Populate the compatibility allowlist only with the exact verified stock
   executable after its native lifecycle passes. It is currently **empty**.
5. Pass every Linux/native job on the final commit; download and verify the
   actual portable ZIP, Setup, source archive, guide and SHA256SUMS artifact.
   No Alpha download has yet passed this acceptance gate.

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
