# Source Health — prototype disclosure migration

Reference: unchanged HTML health drawer and its existing GPS disclosure. The
canonical Day/Dusk/Night expansion is rendered through real prototype clicks;
no illustrative measurements enter the application. Reference/current/diff are
retained without masks or automatic similarity acceptance.

Reference intent: 398px floating sheet, small provenance tags, 69.1875px collapsed
source cards, 8px gaps/radii, 12px summary, 10px state and 7px status dot. Expanded
technical rows use the prototype's 48.59375px rhythm. The source-health header,
Settings entry and alert inspection lead to the same native disclosure sheet.

First review found independently rounded rows losing the cumulative fractional
spacing. Corrective layout retains 236/313/390/468/545/622/699px row positions at
the Windows-derived primary geometry. Tests verify those literal positions and
the 70px depth row, rather than copying the implementation's formula. The second
actual-product capture preserves that layout across Day/Dusk/Night and retains
expansion/close behavior. Windows additionally compares real control rectangles
with a fresh native Chromium render of the original HTML at a one-pixel bound.
That Windows comparison has not run for this increment yet.

Navigation-required differences: GPS requires a coherent latitude/longitude
pair; missing/aging/stale/estimated/uncertain/invalid are explicit; Online AIS
and onboard target reports stay separate; pilot state requires fresh measured
feedback. There are no prototype mock-network/dropout controls. A current motor
RPM does not imply that all motor data is healthy. Configuration and exports
remain deliberate actions, with live setup disabled for historical data.

Linux passes 133 product regressions, 133 fixture regressions and 49 native
component checks/six captures. The actual product was captured twice, including
all three themes and expanded GPS. Original-file checks pass for 113 files.
No native Windows or boat conformance category is marked PASS. Linux font
fallback, complete screen composition, real charts and physical review remain
separate from this local layout correction. See
[local evidence](../../evidence/prototype-source-health-local.json) and
[presentation contract](../../source-health-presentation.md).
