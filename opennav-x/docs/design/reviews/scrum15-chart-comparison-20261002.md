# SCRUM-15 — chart comparison, 2026-10-02

The user's observation is correct. Current chart presentation is not an
accepted match for the immutable HTML. This record is evidence for the Jira
workstream, not a separate backlog.

Compared the [canonical Windows reference](../prototype/reference/windows/navigation-day.png)
with the [combined local Day capture](../../evidence/scrum-15-25297-integrated-chart/XNav-Day.png),
[Dusk](../../evidence/scrum-15-25297-integrated-chart/XNav-Dusk.png) and
[Night](../../evidence/scrum-15-25297-integrated-chart/XNav-Night.png).
The native source is local `25297e3ef88682ad6d85438f762e314ef9fec8d6`, mapped to
published `6e0117de2fd636fa458dcef449f0b6121a5865c2`; the capture is Linux
software rendering, not Windows or physical boat acceptance. Public Seattle
ENC geography differs from the prototype's illustrative archipelago. That
explains geography, not the visibly different typography and symbols.

## Observed and corrected in this increment

- Large built-up areas now use the prototype shore neutral in all three modes.
  Only two BUAARE area instructions change; stock CHBRN remains for hazards and
  other objects. Separate matched Standard captures are pixel-identical in
  their chart regions. See SCRUM-231 evidence.
- The default accurate ownship uses the prototype chevron, with OpenCPN's
  position and course orientation. It is visible eastbound in all three
  captures. Existing course prediction still paints a red line across it;
  that line has not yet been restyled. Custom, scaled and inaccurate ownship
  paths retain their existing upstream meaning.
- Online AIS name placement has focused fixture evidence, but no online AIS
  target is present in these ENC captures. The later 500-pixel measurement
  correction is separate evidence; these images do not qualify it.

## Still visibly different

ENC geographic and object labels are larger and heavier than the prototype.
Soundings and some descriptive text remain dense, with visible overlaps.
Several chart symbols, hazard backgrounds, the chart selector and motion
predictor retain stock appearance. Full route/waypoint/onboard-AIS hierarchy
also remains unfinished. No chart view receives PASS from this increment.

Do not remove depths, light characteristics, warning backgrounds or class
distinctions merely to make a screenshot quieter. Soundings and the actual
selected chart depth units must remain navigationally meaningful. Decorative
prototype examples cannot replace missing chart or navigation information.

## Text source inspection

The prototype's final `.chart-label` resolves to Segoe UI, 12 CSS pixels,
normal/400 and 1-pixel tracking. Water labels use 16 pixels, italic, with
5-pixel tracking. Depth labels use 10 pixels with separate opacity. Generated
SVG font-size attributes are overridden by CSS, and viewport scaling still
applies.

Pinned OpenCPN `libs/s52plib/src/s52plib.cpp`, `RenderT_All`, maps lookup
weights below five to light, five to normal, and above five to bold. It adds
0–4 points to configured ChartTexts sizing with a ten-point minimum. BUAARE
names use `16120` (bold); SEAARE uses normal `15110`/`15120`. Soundings use the
separate `RenderSoundingSymbol` path; `TextRenderCheck` and importance gates
control visibility. Flattening all of these into one weight is not justified.

A narrowly gated geographic-name font adjustment needs an explicit policy for
user font preferences. `FontMgr::m_is_default` alone is insufficient provenance:
`SetFont` changes the font without clearing that flag. Existing user preferences
must not silently be overwritten. Standard/Legacy, soundings, light and hazard
descriptions, class distinctions and visibility filtering need comparison
evidence for any later adjustment.

## Separate AIS renderer defect

The current online AIS polygon sends bow/right/notch/left vertices to the
pinned `ocpnDC::DrawPolygon` four-vertex GL strip. Its internal order
`0,1,3,2` fills the stern notch. A cyclic right/notch/left/bow ordering preserves
the software polygon and avoids that wrong diagonal. SCRUM-234 tracks this
bounded correction; actual GL and native/boat rendering remain gates.

The captured application exited cleanly after one controlled navigation-input
run. This is useful development proof, not whole-product visual acceptance,
real vessel input, installed Windows qualification or authorization to deploy.
