# Beta 1 — real boat integration and release hardening

Authorized by the user's Beta request on 2026-09-25. Continue the accepted Alpha
architecture and pinned OpenCPN 5.12.4/x86 ABI on Windows x64. Stop after Beta
artifacts and commissioning guides; no automatic production stage. No SmartNav
control path. Physical acceptance is separate from software acceptance.

## Alpha feedback disposition

| Input | Classification | Disposition |
| --- | --- | --- |
| User: Legacy → XNav lost charts | Bug | Closed by accepted foundation `16dbaf7`; exact Alpha `08bc92f` repeats portable/installed/Legacy/Safe chart-content checks. |
| Final Alpha review: active-leg console covers rail | Bug / usability | Fixed and red-tested; exact `08bc92f` Linux/native route tests and screenshots accepted. |
| User: Alpha completed and manually tested | Acceptance, no additional defect list | Record only supplied statement; no invented physical/DPI/at-sea results. |
| Review: bright native scrollbars/title frame and basemap land at night | Visual refinement / operational usability | Beta night hardening before release. Advanced Legacy dialogs remain an explicit escape hatch. |
| Physical touch, hardware GPU, N2K propulsion and pilot | Hardware-dependent validation | Software fixtures and bench procedures now; no physical success claim. |
| Autonomous steering, advanced radar fusion, production certification | Deferred feature | Excluded. |

No other Alpha feedback file was found in the project. Later feedback is steering
for this stage. Serious navigation regressions must be fixed before acceptance.

## Ordered increments and gates

1. **Boat/source inspection and live acquisition.** Pin inspected firmware
   revisions; map actual marine PGNs, reference/instance/sentinel semantics,
   text state and source health. Keep selected OpenCPN GPS ownership. Add bounded
   diagnostics/rates, input rejection, source-loss scenarios and commissioning
   bench UI. Test real OpenCPN bus boundaries on both platforms.
2. **Recording, replay and calibration.** Opt-in bounded normalized recordings,
   strict parser, deterministic replay visibly separate from live/control,
   speed/power calibration exports/imports and energy confidence/reasons. No
   implicit raw bus log or position export. Retain empirical curve assumptions.
3. **Manual pilot adapter.** Inspect actual translator and OpenCPN send path;
   target configured interface/device, display-only by default, deliberate enable,
   STANDBY/AUTO/±1/±10, feedback confirmation/timeout/no retry storms. TRACK/WIND
   only with verified capability. Simulated transport contract and native UI
   checks; physical status/control commissioning remains open.
4. **Operational advice and alarms.** Turn and timeline failure handling; AIS
   selection/highlight; ENC future-corridor query boundary with explicit coverage
   and uncertainty; radar plugin feasibility/status; Anchor history/battery and
   clear alarm layer above sheets. No charted/measured-depth conflation.
5. **Recovery, privacy and release robustness.** Bounded diagnostic bundle with
   explicit recording selection, malformed network/config/file inputs, installer
   locked/corrupt/interrupted/permissions/dependency failures and real stock
   coexistence. Preserve immutable-generation rollback and shared profile.
6. **Native usability and endurance.** Night/fullscreen/resize, 100/125/150% DPI,
   injected touch and documented physical touch gate, plugin compatibility,
   software/OpenGL fallback and several-hour simulated trip. Measure CPU, memory,
   handles, UI update latency and mode-restart survival. Keep existing tests.
7. **Exact Beta delivery.** Native MSVC and Linux same-commit gates, installer
   lifecycle, downloaded/hash-verified `OpenNavX-Beta1-Setup.exe`,
   `OpenNavX-Beta1-Portable-win64.zip`, desktop/boat guides, source and checksums.
   Review native screenshots. No successful package publication if tests fail.

Every code increment remains buildable. Contract/unit gates are supplemented by
actual runtime/bus/lifecycle/UI evidence; compilation alone is not acceptance.
Existing counts are 90 Linux / 80 Windows integrated cases and 30 portable suites
per platform, plus GUI/synthetic/recovery/installer checks recorded separately.

## Decisions retained

- Marine data uses OpenCPN connections/bus; no desktop Leaf CAN decoder.
- Read-only copied route progress and AIS; core owns calculations and objects.
- PC receipt time cannot prove a bridge refreshed its underlying sensor.
- Per-user side-by-side installed generations preserve exact stock executable;
  normal installed profile is shared. Portable remains isolated. Beta must test
  upgrade from Alpha without renaming away or losing existing installation state.
- Missing radar/ENC capability remains unavailable, with the precise limitation
  documented. Missing data is never replaced with simulation in live mode.
