# Autopilot — prototype migration in progress

Authority: immutable HTML `case 'autopilot'`, final `.heading-dial` cascade and
the canonical Windows Day/Dusk/Night reference. No conformance or boat PASS.

Reference intent: HELM CONTROL / Autopilot in the 398px chart-side drawer; mode
and control pills; 165px dial with 36 ticks; prominent heading; four 48px course
actions; two rows of mode actions; compact consent toggle; adapter and rudder
rows. The original reference scrolls for its lower explanatory material.

`XNavPilotDrawer` replaces the previous full-page gauges and large action grid.
All existing pilot entry points, including alerts and the frame accelerator,
lead to this drawer. It preserves the chart viewport. The shared button now
supports the exact 42×25 toggle inside a 48×49 touch target; its state changes
only after the owner supplies a changed observation. No native checkbox is used.

Measured Windows button positions relative to content origin (705,190): course
row y223, mode rows y291/359; course gap 7, mode gap 9. Enable row starts y407;
the toggle face is y434 and is 42×25. Adapter/rudder rows follow y487/532.
Dial geometry uses the prototype's 200-unit SVG coordinate system scaled to 165px.
Numeric type is 45px, weight 350, tracking −2px. All colors come from the theme.

Safety-required differences: the dial and arrow require measured magnetic pilot
feedback; absent data is an em dash. Labels identify magnetic units. The
prototype's mock timeout toggle and mock hardware claims are not installed.
Advanced setup/diagnostics remains accessible below the primary controls.
Permission, command confirmation and pending/timeout states retain the existing
[manual-control contract](../../pilot-presentation-contract.md).

First Linux component review passes 61 checks/seven captured states. It exposed
missing numeric tracking, corrected before the second capture. Windows
typography, strict button geometry, physical touch and boat review remain open.
No command was sent to the boat.

Corrective review: the second component pass retains all 61 checks and seven
images. The first actual-product Day capture had incomplete child painting; it
is retained as a failure, and the new capture gate requires actual title and
all eight command labels. The subsequent run also exposed stale diagnostic
page publication after Instruments. Waiting for the requested semantic page
resolves that mismatch without retrying input. The replacement captures the
complete pilot in all three themes. Both production and fixture integrated
suites pass 132 tests. See [local evidence](../../evidence/prototype-pilot-local.json).

Native predecessor `403a4e8` passed 120 tests in both MSVC builds; its two UI
assertion failures and replacement checks are retained in
[negative evidence](../../evidence/prototype-native-403a4e8-failed.json).
No release or physical boat acceptance is inferred.

The final fixture run passes all eight retained scenarios, 44 captures and
shared-profile XNav/Legacy/Safe restarts, including the exact new drawer and
48px control geometry. No accepted scenario or adapter test was removed.

Downloaded Windows `0ce6835` passes 124 tests in both builds, 61 component
interactions, actual-product three-theme captures and development DPI checks.
Review nevertheless finds partially missing static content in pending/confirmed
component images after a modal. The corrective shared sheet exit queues
repainting of visible owned children after dialog destruction, guarded by weak
lifetime. Screenshot gates now require the header and eight control labels.
Linux retains all 61 checks/seven images with the stronger gate; native
replacement remains required. The retained preview's captioned-window height
assertion is corrected using actual client geometry, without weakening the
separate canonical 1280×800 test.
