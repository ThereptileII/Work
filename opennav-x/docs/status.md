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
| Recovery | Two unfinished XNav starts select Safe before optional modules; retry evidence retained. Exact native intermittent close failure below remains under investigation. |
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

Subsequent candidates retain **90 Linux / 80 native Windows integrated cases**.
`33447f2` passes 28 portable suites per platform, native configurable displays,
actual 100/125/150% DPI/touch, public NOAA ENC loading/zoom/pan/follow, plugin
manager and actual three-point route creation. Four relevant native images were
reviewed. Its route-edit automation used a double-click-like gesture; the new
press-drag-release gate still requires native acceptance.
[Native review](evidence/windows-alpha-33447f2-review.json).

Linux passes both software and llvmpipe OpenGL switching the actual quilt
reference between two hash-pinned public NOAA ENC cells, including overlays,
mode returns and performance sampling. Windows OpenGL/full chart gate remains
pending. No blank/all-water screenshot is accepted as chart evidence.
[Local chart evidence](evidence/alpha-two-cell-chart-local.json).

Native Windows PowerShell 5.1 passes **24 actual filesystem assertions in each
32/64-bit host** at `452e26b`, after correcting .NET null-string binding during
atomic replacement. Native NSIS Setup and the separately compiled prior-version
fixture build at `4203854`; the official prerequisite installer returns 1223
before any Alpha lifecycle. Explicit ShellExecute elevation also returns 1223 at `4181962` despite a
verified administrator token. Initial stock UI capture is now required to
identify the abort; Alpha remains per-user/as-invoker.
[Filesystem gate](evidence/installer-filesystem-452e26b-gate.json),
[prerequisite failure](evidence/installer-4203854-prerequisite-failure.json).

`9981643` passes all 80 native compiled tests but its object-scenario reader races
Windows report replacement. The bounded access-only reader replacement passes
**29 Linux portable suites** and the actual object/AIS/anchor scenario; its native
rerun is pending. Corrupt evidence and product failure assertions still fail.
[Failure and replacement](evidence/windows-json-publication-9981643-failure.json).

A native Safe → XNav recovery timeout at `54029a3` remains unresolved. The
replacement logs close boundaries, captures failed windows and requires three
independent actual forced-crash/recovery cycles. No automatic test retry hides
failure. [Failure record](evidence/windows-recovery-54029a3-failure.json).

## Remaining Alpha release gates

1. Resolve/validate the native recovery and chart-edit/switch/OpenGL gates.
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
