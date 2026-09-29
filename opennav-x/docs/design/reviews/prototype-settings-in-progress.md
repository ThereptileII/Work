# Prototype Preferences review — in progress

Reference: immutable Settings/Sensors/Display/System states, 1280×800.
No Settings screen or boat visual acceptance is claimed.

## Changes and corrective review

Preferences now opens as the prototype's 432px native owned drawer at (648,80),
retaining the actual chart and horizon. The previous full-page menu no longer
replaces the chart. Eight native tabs use the computed sizes, wrapping and
selected colors. Existing advanced workflows remain reachable and return to
the last section. Shared suite-link buttons and bounded text follow the final
CSS, rather than carrying the older generic card appearance into this sheet.

The first corrective capture exposed taller-than-reference sensor links and
an unwrapped intro. A second pass uses the exact 72px links and bounded
13px/20.8px body text. The third corrects the sensor eyebrow to the computed
10px/650 secondary text with 1.3px tracking, and fixes Display selection after
the theme changes outside this drawer. Reference/current/diff images are
retained for all three passes. Linux's fallback font makes its HTML header
three pixels shorter than Windows; the native 89px header targets the actual
Windows reference. Linux captures cannot qualify Windows typography.

The component executable is separate and never installed. Its actions are
callback counters only. It checks exact bounds, all eight visible tabs, real
event dispatch, section return, disabled unavailable actions, Close, theme
updates and fullscreen delegation. No profile, network or equipment is used.
Integrated production captures additionally test real pointer input and that
the 1014×566 chart remains intact behind/after the sheet.

## Meaning and remaining differences

- Sensors reports observed source quality, not the HTML's fictional connected
  count. Add sensor delegates to the real OpenCPN connection editor.
- Vessel currently shows actual configured assumptions and links to validated
  editors. Its inline name/draft/safety-depth/capacity/reserve form is pending.
  Advisory hazard margin cannot be relabelled as OpenCPN chart safety depth.
- Navigation, Autopilot, Radar, Display, System and Help still need their
  individual prototype content migration. Existing actions work, or are disabled
  when their boundary is unavailable. No mock installer, radar or pilot action
  is presented as real.
- Native Windows, non-default DPI, physical boat comparison and full release
  lifecycle remain required. No conformance-table PASS is recorded.

Evidence: [local record](../../evidence/prototype-settings-local.json).

The fixture-free full product passes 123 tests and its loader self-test.
Twenty-one captures now pass: Preferences theme cycle, Sensors/Display/System,
Close preserving chart geometry and the existing AIS/Instruments flows.
The first product capture correctly rejected two unscoped Radar captions;
the corrected harness scopes left-navigation identity to the independently
measured sidebar, retaining exact rectangle checks. Day/Sensors/Night were
reviewed again. These captures confirm composition, not full content fidelity.
See [corrective evidence](../../evidence/prototype-settings-product-local.json).
