# SCRUM-271 — notification affordance

Implementation/evidence only; native acceptance remains open. This isolated
work is based on `0394b4a3a356c1ea2323aa977419184184e22667`. It does not change
the frozen application candidate or claim qualification for its current run.

## Observed problem and computed mapping

The preceding native `9d98` software ENC image shows the upstream cyan bell in
a grey outlined square near x947–983/y101–139. The original image identity is
recorded in `docs/evidence/scrum271-notification/result.json`.

The immutable prototype was measured at 1280×800, DPR 1 using its real theme
button and settled browser clock. Full extracted values and original-file
identity are retained in `prototype-computed.json` in that evidence folder.
The alert button is a 44×44 target with radius 9, a centered 22×22 bell, 1.65
stroke, round caps/joins and no border. Its surrounding header supplies `--bg`.
The floating chart control instead has radius 12 and a pale Day surface.

| Token | Day | Dusk | Night |
| --- | --- | --- | --- |
| Alert backing `--bg` | #152326 | #1d282e | #0c1115 |
| Informational `--cyan` | #7bc8d7 | #82b0c1 | #78989c |
| Warning `--amber` | #ecc48c | #cfac84 | #aa9170 |
| Critical `--red` | #ec8f87 | #ec8f87 | #b77569 |

The implementation composes the existing prototype bell/icon target with this
alert backing and explicit severity inks. Keeping the dark backing makes the
amber and red readable over real chart backgrounds. This is a documented
notification-specific adaptation: the prototype header bell itself is neutral
and its sample badge does not represent OpenCPN's notifications. No sample
number is copied and no count is invented. Upstream's actual count still controls
visibility and its list retains repeat counts and acknowledgement behavior.

## Narrow boundary

- `ui/NotificationButtonBitmap.h` reuses `PrototypeIconPath(Bell)`, `Theme` and
  `NavigationContextInk`; known upstream icon names select the semantic ink.
- The target is an actual 44 DIP square converted to physical pixels for
  upstream's rectangle. It does not retain the old ~37 px target or inherit the
  unrelated compass scale. Upstream placement and `GetLogicalRect` remain intact.
- `CreateBmp` changes the bitmap and rectangle size only after a valid alpha
  bitmap is available. Unknown icon names and failures fall through to stock.
- Existing severity/theme rebuilds remain in place. A stale DPI-sized bitmap is
  invalidated by `UpdateStatus`; its existing texture refresh path remains used.
- Existing software and GL `Paint` methods remain in place. Only successful
  SKAGER bitmaps get transparent upload pixels and scoped alpha blending. The
  incoming GL enable, RGB/alpha factors and equations are restored exactly.
- Legacy/Safe and stock fallback do no new blend work and retain opaque stock
  texture upload. Upstream model, notification visibility, maximum severity,
  click-to-NotificationsList, GUID acknowledgement and repeated-message count
  methods are unchanged. No hardware command path is involved.

## Focused evidence

`docs/evidence/scrum271-notification/` retains the paint sheet, computed
reference, checks and production-object identities. The paint sheet has Day,
Dusk, Night rows and information, warning, critical columns. It is an isolated
Linux raster fixture, not an application or native Windows screenshot.

- Complete changed `notification_manager_gui.cpp` compiled against actual
  prepared pinned headers with `OPENNAV_X`/GL, `OPENNAV_X`/software-only, and
  stock GL without `OPENNAV_X`. No application link or full build was run.
- 219 bitmap checks exercised all three themes/severities at 44, 55, 66 and 88
  pixels, transparent corners, opaque surfaces and actual severity-color pixels.
- Extracted actual production cache/theme/logical-hit methods and the new
  `CreateBmp` entry hook exercised unchanged reuse, theme return, severity change,
  stale-size replacement, unchanged placement, full target and unknown/Legacy
  fallback. Stock rendering itself is a counted stand-in in this fixture.
- The scoped GL guard was tested with both initially enabled and disabled
  blending and distinct incoming RGB/alpha factors/equations; stock opt-out
  performs no GL calls. This is a state fixture, not a real GPU rendering claim.
- The new patch section applies cleanly to unchanged pinned notification
  sources. Existing stock bodies and notification routing are retained verbatim.

Reproduce the small fixture with `tools/test-notification-button.py` using
`--notification-source` for the prepared patched translation unit, `--upstream`
for pinned headers, `--wx-prefix` for the existing wx/Xvfb runtime and `--output`
for a disposable evidence directory. Compile command and object hashes are
retained separately; no shared build was modified.

## Remaining acceptance gates

Fresh exact-revision native Windows MSVC and real software/GL application
captures are still required, including Day/Dusk/Night/Day return, all severities,
notification count/acknowledgement, Legacy/Safe fallback, 1280×800 and 1920×1080
placement, and 100/125/150% DPI hit bounds. Actual GL texture upload/compositing,
boat-PC GPU/display/touch behavior and review against the chart composition are
unverified. The larger target's upstream placement must be checked natively.
No full workflow, boat access, application launch or push was performed.

## Bounded native compile follow-up (comment 10786)

The existing native changed-unit preflight now accepts
`--notification-unit-only`. This selects no local `.cpp` units and only the real
prepared `gui/src/notification_manager_gui.cpp`. The existing default chart
preflight still selects its original 23 units. Both reuse the unchanged
`tests/windows_changed_units/ChartUnits.cmake` object targets, pinned-source
preparation, SDK/header locks, include paths, generated configuration and
production `OPENNAV_X`/GL/Win32 policy. The production root CMake lists this
notification source in the application and supplies the same GUI/GL include
families; no replacement source, fixture header, forced include, `NOMINMAX` or
Windows macro workaround is added.

The two new notification headers are explicit required `localInputs`, in
addition to the existing tracked-source inventory. The generated MSVC project
must contain exactly the complete prepared notification source. The unchanged
object verifier requires precisely its one native x86 COFF object and rejects
extra units or wrong architecture. Existing before/after input hash checks and
reverification of the prepared patch tree remain intact. Evidence continues to
use `chart-inputs.json`/`chartPreflight`, with an explicit
`selection: notification-unit-only` and scope declaring compile-only status.

The workflow supports the `notification_unit` dispatch input, the
`[notification-unit]` commit selector and the dedicated
`skager-notification-preflight` branch. This mode skips component runtime,
geometry, capture dependency and Search-reference work; conflicting proof
modes fail. It runs only the small notification selection/CLI guards before
compiling the one object. It does not link or run the application.

Three notification guard tests passed, including rejection of substituted
project sources and unrelated objects. Two existing chart selection/CLI guards
also passed, and the workflow YAML parsed. This is local wiring verification,
not a native compilation result. No workflow was dispatched and no push, full
build, application runtime, frozen-candidate modification or boat access was
performed. Native/GL/visual/boat acceptance remains as listed above.
