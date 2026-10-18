# SCRUM-378 — prototype refinement and existing boat comparison

2026-10-18. User-requested visual refinement; source-only development remains in
force. Jira is the backlog. This record describes evidence and implementation,
not a separate roadmap or visual acceptance certificate.

## Reference and observed environment

Compared immutable `prototype/index.html`, its CSS and canonical Windows
`navigation-day.png` / `instruments-day.png` with the user-supplied Personalise
popup screenshot and the running boat through RustDesk on workspace 2.
The prototype remains unchanged. The popup reference shows four position
selectors under “Your four essentials” and Restore essentials.

Boat capture metadata identifies installed commit
`9feae59ce0ef1f54107c3c51f920085297ea7bc8`, **1920×1020, 120 DPI**. This is
approximately 1536×816 logical pixels, not the 1280×800 target. Native captures
used the reviewed, hash-verified existing capture helper against running PID6980.
The Windows Start menu initially covered the application; that capture refused
and was retained as a failure. After dismissing that menu, captures succeeded.
The application was not restarted, replaced or qualified as stable.

Private chart screenshots remain outside Git under the task's `.tmp` directory:

| Capture | UTC | SHA-256 |
|---|---|---|
| Navigation, installed baseline | 2026-10-18 20:04:03 | `168de033637198061a8037a1fbf9225c94d5c906463e052ff96dc6d07e69c84e` |
| Instruments, installed baseline | 2026-10-18 20:09:25 | `5756b078ac29f968851e52a0c08034363a245fcacd67b95a0fe9afc864fc37ab` |

No proprietary chart material, coordinates, credentials or unrelated desktop
content are committed. Real observed apparent wind/heading/depth help explain
presentation gaps, but this inspection is not sensor commissioning.

## Requested changes and source response

| Request | Source response / remaining evidence |
|---|---|
| 1. Very Light chart | Added opt-in uniform normal-water fills and faint ordinary contours; safety contour, drying and hazards preserved. Native/boat paint pending. |
| 2. Colors and chart scale | Prototype palette retained; shared core/private chart display scale and overlay sizing. Not all raster image content is scalable/restylable. |
| 3. “Light” lighthouse | Existing exact LIGHTS/qualified support aliases reviewed. Particular northern feature still unclassified; no speculative tower substitution. |
| 4. Vessel chip | Refreshes configured name/initial and opens Vessel settings before presentation. Navigation row height budget retains the shortcut at the bottom. |
| 5. Rounded surfaces | Shared native shape for owned drawers, context/route sheets and credential/setup dialogs; stock Legacy/system windows remain native. |
| 6. Rail detail/proportion | Value-font bounds govern graph placement; subtle source/age footer restored; units/quality retained and decorative content yields before values. |
| 7. Display percentage | Immediate persisted 100/125/150% changes, failure rollback, UI size and chart invalidation wiring. Native DPI behavior pending. |
| 8. Select values/presets | Four position selectors, duplicate-safe changes, Restore essentials and up to eight named presets; old overflow preserved in Instruments. |
| 9. Active-route header | Actual active route name beside logo, including when distance is unavailable; no fabricated distance or destination substitution. |
| 10. Forecast hover/copy | Forecast badge consumes its own hover/click so native chart-edge cursor does not appear; forecast/stale/source/time remain, redundant advisory sentence removed. |
| 11. Wind vectors | Lighter prototype cyan with restrained opacity; denser bounded geographically sampled field. No synthetic measured wind. |
| 12. Instruments compass | Valid coherent apparent-wind fallback explicitly labeled when true direction is unavailable; no fabricated true wind. Source-age/quality remains visible. |
| 13. Subtle lines | One physical-pixel theme separators join top/navigation/rail surfaces, without global chart-border chrome. |
| 14. Live visual inspection | Existing navigation/instruments and supplied reference compared. New-source screenshots and full visual acceptance deliberately unperformed. |

## Additional concrete findings

The installed rail showed a 3m “safety” marker beside below-transducer depth.
The source now withholds this comparison because chart safety depth and measured
transducer depth have different reference datums. The live instrument compass
had no wind direction despite valid apparent wind; its true-only presentation
explains the gap. The extra Weather navigation row pushed the profile shortcut
below the footer. Source corrections address both findings.

Source review also kept rail saves separate from the generic settings callback,
which would reconfigure sources and disable the pilot; preserved fifth/sixth old
rail selections; reset sparkline history across replay/live changes; and called
the Personalise drawer's own scaling method rather than its non-virtual base
method. Queued popup generations and current-state comparisons protect retained
UI callbacks. No physical commands were issued.

## Verification boundary

Codex agents reviewed independent source changes in isolated worktrees, then the
combined source integration. Authored cases cover Very Light paint boundaries,
settings/preset validation, apparent-wind coherence, display preference changes,
scaled isolated painters and geographic anchor/hull invariance. **None executed.**
No compile, full build, test, patch application/check, CI run, deployment, mode
restart or actuator command was performed. The current boat screenshot cannot
prove the unbuilt UI's geometry, font fit, rounded-window behavior or renderer
cache invalidation. Those checks and exact northern light-object identification
remain pending; SCRUM-378 must not be marked Done on this source work alone.
