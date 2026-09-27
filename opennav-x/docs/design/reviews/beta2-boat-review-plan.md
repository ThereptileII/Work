# Beta 2 boat display review plan

Planning record, not boat acceptance. Product candidate:
`9f59209914f57ff97af7e184b201752b8f422f0a`,
[CI 36299835767](https://github.com/ThereptileII/Work/actions/runs/36299835767).
Record the actual installed commit, executable hash, generation and tool hashes
in every private review; this candidate's qualification is still pending.

## Launch and evidence boundary

Use the actual shared profile and charts only after the exact installed product
and the [read-only commissioning transaction](../../installer/read-only-commissioning.md)
pass their independent checks. Preserve the verified backups and applied plugin
quarantine. Retained plugin shutdown must also have been reviewed before normal
close or a mode transition. No preparation/launch attestation is created by this review plan.
`run-xnav.ps1` supplies the successful audited launch `result.json`; retain that
path and SHA-256, its adjacent `request.json`, and the actual process identity.

The existing `tools/boat/review-window.ps1` accepts one fixed display action per
call, bound to that launch and the installed fixture-free build. Example, with
operator-supplied values from verified records rather than guessed paths/hashes:

```powershell
$review = @{
  Workspace = $BoatWorkspace
  LaunchResult = $AuditedLaunchResult
  ExpectedLaunchSha256 = $AuditedLaunchSha256
  ExpectedCommit = '9f59209914f57ff97af7e184b201752b8f422f0a'
}
& (Join-Path $ReviewedScriptDirectory 'review-window.ps1') @review -Action Capture
& (Join-Path $ReviewedScriptDirectory 'review-window.ps1') @review -Action Navigation
```

Each call returns private before/after image paths and SHA-256 values, measured
window bounds/DPI, visible page labels and, for list selection, the actual chosen
row. Review **each** after-image before the next action. A successful helper
result means the bounded action/capture completed, not that the screen passed.
If animation, a late sensor update or paint has not settled, use a separate
`Capture`; do not resend the interaction blindly. Refused/obscured captures,
ambiguous controls, another foreground window or an unexpected page are failures
to investigate. The helper deliberately has no arbitrary coordinates/keys.

Screenshots can disclose positions, saved names and licensed charts. Keep images,
launch records and raw diagnostics private. Public reviews contain redacted
observations and evidence identifiers, not chart content or user paths.

## Required screen matrix

In this table, `A → B` means two separate `-Action` calls, with image review
between them. Start each ordinary page sequence from `Navigation` if needed.
**Available** means the page can be opened safely, not that real data exists.
**Conditional** requires the stated real observation. **Gap** remains untested by
this fixed helper; a different screen is not a substitute.

| # | Required screen | Exact operations | Observable criteria and remaining boundary |
| --- | --- | --- | --- |
| 1 | Navigation — Day | `Navigation`; inspect current palette caption; `CyclePalette` one step at a time until Day; `Capture` | Available. Actual chart content, readable top status, four rail values/units in view, Center/zoom discoverable, no Demo. Identify real coastlines/chart features; an all-water or blank image fails. |
| 2 | Navigation — Dusk | From observed Day, `CyclePalette`; `Capture` | Available. Caption says Dusk, both chart and XNav change coherently, rail and status remain readable. |
| 3 | Navigation — Night | From observed Dusk, `CyclePalette`; `Capture` | Available. Caption says Night; no bright unused surfaces; chart features and important controls remain legible. |
| 4 | Active Route | `Route` opens the Passage page | Gap for active navigation. The cold-launch audit refuses a persisted active route, and this helper cannot activate one. Review honest No active route/unavailable values only; this does not qualify destination, ETA, next turn or arrival SOC with a real passage. |
| 5 | Waypoint selection | `Menu → Waypoints → SelectFirstVisibleWaypoint`; `Capture` | Conditional on a fully visible saved row. Opens **Waypoint detail**, verifies actual name/coordinates or unavailable state, truthful disabled GO TO and readable actions. Compact chart card, chart tap/long press, Edit/Remove/GO TO are not exercised. No new waypoint is created. |
| 6 | AIS target | `Menu → AIS → SelectFirstVisibleAis`; `Capture` | Conditional on an actual received/retained target row. Opens **AIS target** detail. Inspect identity/status, SOG/COG and CPA/TCPA/range only where valid; lost/stale targets stay explicit. An empty list proves no targets observed, not receiver health or AIS acceptance. Compact chart card and selected-target highlight remain gaps. |
| 7 | Instruments | `Menu → Instruments`; `Capture`; `PageDown` only if its enabled control is visible | Available. Navigation, wind and conditions groups show complete labels, values, units and quality within the actual viewport. Missing sensors remain unavailable; real changes need repeated captures and source-age evidence. |
| 8 | Propulsion / Energy | `Energy`; `Capture`; `PageDown` where available | Available layout/unavailable-state review. SOC, V/A, power, RPM and temperature must reflect actual source health. Estimates are advisory and withheld without valid inputs. No simulated pack/motor/route is supplied to fill this screen. |
| 9 | Autopilot | `PilotView`; `Capture` | Available **display only**. Control remains OFF; heading, rudder and connectivity may be unavailable. Inspect target sizes and disabled modes. Do not press STBY/STANDBY/AUTO/course/TRACK/WIND, enable control, discover identity or change configuration. Unexpected enabled control fails this review. |
| 10 | Anchor | `Menu → Anchor`; `Capture` | Available page; actual watch is conditional on existing state. Inspect distance/radius/history/depth/wind as present. Do not set/clear a watch. An inactive page cannot close B1-05/06/07 watch-label/icon/removal checks. |
| 11 | Alerts | `Menu → Alerts`; `Capture` | Available. Inspect actual conditions, severity and persistent top alert while opening other pages; all-clear is recorded as such. Do not manufacture sensor loss or acknowledge/clear an alarm. Critical-state layout and acknowledgement remain untested if not naturally present. Use Menu: the header is labelled `Alerts N`, which the fixed `Alerts` action intentionally does not target. |
| 12 | Settings | `Menu → Settings`; `Capture` | Available. VESSEL/NAVIGATION/SENSORS/AUTOPILOT/RADAR/DISPLAY/SYSTEM hierarchy readable and unclipped. SENSORS and DISPLAY have dedicated next-step actions; other categories/configuration edits are not covered. |
| 13 | Data Sources | `Menu → Settings → Sources`; `Capture`; `PageDown` where available | Available. Friendly source health for GPS, heading, depth, wind, motor/battery/tanks and AIS; absent data never looks connected or zero. Individual source-row selection/connection editing is not supported. Use Diagnostics to inspect available ages/provenance. |
| 14 | Diagnostics / System | `System → Diagnostics`; `Capture`; `PageDown` where available; `System → Capture` | Available. Correct Beta 2/version/commit, INSTALLED PRODUCT, no Demo/replay, actual OS/DPI, source validity/ages, route and energy reasons. System must not overlap or hide the alert/status region. Record missing fields rather than inferring them from package metadata. |
| 15 | Legacy transition | **No `review-window.ps1` action** | Gap for an in-app transition. `run-mode.ps1 -Mode Legacy` plus `capture-ui.ps1 -ProcessId <returned PID>` can review a separately audited cold Legacy launch, but does not prove XNav → Legacy → XNav. Guarded restart needs the qualified one-use broker, reviewed plugin shutdown and exact new-child evidence; see below. |
| 16 | Safe Mode | **No `review-window.ps1` action** | `run-mode.ps1 -Mode Safe` plus `capture-ui.ps1 -ProcessId <returned PID>` can review a separately audited cold Safe launch. Check real charts and no XNav advisory/control initialization from logs. Safe → XNav is a separate guarded-transition gap. Do not apply XNav-specific actions to a Legacy/Safe process. |

## Review order and common checks

1. Capture the untouched startup window and actual DPI/bounds. Try
   `Resize1280x800` only when the **monitor work area** accommodates it. A physical
   1280×800 panel with a visible taskbar can have less than 800 usable pixels;
   the helper correctly refuses. The separately qualified `Menu → Settings →
   Display → ToggleFullscreen` path changes only the application window.
   Capture each result and return through the same visible control. Preserve
   display/taskbar/remote-access configuration and record actual dimensions;
   fullscreen on a larger monitor is not physical 1280×800-panel acceptance.
2. Review the three navigation palettes, then pages 4–14 in Day and Night. Run
   `CyclePalette` on Navigation only: Display settings can contain additional
   Day/Dusk/Night controls and make the match ambiguous. Return using
   `Navigation`, or `Escape` on a normal page when focus is not editing text.
3. On Navigation, `ZoomIn`, `ZoomOut`, `Capture` can check rendering at different
   scales. Use `Center` only with verified current position and inspect whether
   the existing ownship/follow behavior is clear. `ToggleOrientation` uses the
   current North/Course control only; capture its actual label/chart result.
   Pan and chart-selection gestures still require their own bounded review.
   [Native display-tool qualification](../../installer/display-window-review.md)
   is separate from actual application and boat acceptance.
4. With a naturally occurring alert, repeat Navigation/System/Instruments and
   verify four rail cards and critical status stay visible. Do not unplug or
   reconfigure equipment to force a scenario. Use real source timestamps/ages
   and observed values; no inference from a connection definition alone.
5. `PageDown`/`PageUp` inspect only the current visible scroll view. Capture the
   initial viewport before scrolling so clipping cannot be hidden. Task-local
   HWND mouse messages are remote UI evidence, **not physical touch acceptance**.
   Record current DPI; this plan does not change Windows scaling to create
   100/125/150% results. Existing CI evidence remains separate.
6. Request normal close through `stop.ps1 -ProcessId <audited PID>`; inspect logs,
   process exit, profile delta and commissioning state before any new launch.
   Never blindly renew the INI hash, force-kill, restore plugin DLLs while the
   app runs, or relaunch unguarded after a refused transition.

## Transition and scenario limits

The [native broker qualification](../../installer/commissioning-restart-qualification.md)
proves marker-process/tooling boundaries, not this product's installed mode
buttons or real plugin shutdown. Ordinary `review-window.ps1` requires an
audited **XNav** Launch result and explicitly refuses mode restart. The newer
RequestMode/ReviewChild tooling has separate native marker-window qualification;
this does not establish installed-app or boat execution. Do not
substitute an old parent Launch result for a restarted child. Before transition
review, require exact qualified tools plus current Arm/readiness/normal parent
exit/full cold audit/consumed permit/child PID-and-creation-time evidence. Treat
refusal as a recorded gap, never permission to use arbitrary keys or clicks.

Feedback boundaries: B1-01/02/08 and the Beta caption/layout defects are directly
reviewable in the listed screens. B1-03/04 need a separate controlled connection
change/settings-apply scenario; unchanged live reception or cold mode startup
does not close them. B1-05/06/07 need a real watch lifecycle that this plan does
not create. B1-09 remains source/transport review only, with no physical commands.
B1-10 requires actual fresh AIS reports; empty/lost-target presentation alone
does not prove boat reception. Active-route, real energy prediction and compact
chart contexts similarly remain explicit gaps where no truthful scenario exists.

For each row record: available/conditional/gap, before/after evidence IDs and
hashes, actual page/palette/physical bounds/DPI, source observations and age, chart
content observation, clipping/readability/action result, and follow-up. No row
becomes accepted solely because its page opened or a script returned success.

## Plan verification

Reviewed `ReviewWindow.ps1`, `review-window.ps1`, `ReviewWindowNative.cs`, the cold
launch/profile checks, and current Shell/ProductPanel/PreviewPanel callbacks
against [user flows](../user-flows.md), [components](../component-inventory.md)
and [boat feedback](../../feedback/boat-beta1-feedback.md). The existing
`test-review-window.ps1` passed **145 policy/source/compilation checks** locally.
It executed no Win32 UI actions, accessed no boat, and sent no hardware commands.
No product code or action allowlist was changed for this plan.
