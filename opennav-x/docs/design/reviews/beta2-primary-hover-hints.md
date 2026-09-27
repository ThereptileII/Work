# Primary hover hints and night lighting

The exact `60cd0547` Windows System screenshot contains a bright native Menu
tooltip. This was captured in **Day**, not Night. Source inspection found that
XNav buttons and data values attached native wx tooltips but only repainted
their own surfaces when the palette changed. Native tooltip appearance is
controlled by the operating system; the potential bright Night surface was a
source-backed risk, not a reproduced Windows Night failure.

A disconnected fixture-free Linux observation hovered Menu in Day and Night.
Neither GTK capture showed a tooltip. The process exited normally, and this
observation is not used as proof of Windows behavior.

The refinement is local to the primary XNav component library:

- Buttons retain their hint and accessible name. `GetHelpTextAtPoint` returns
  the stored description independently of the native hover window.
- Day attaches the native hint. Dusk and Night remove it from that control.
  Returning to Day restores the current hint.
- Data updates retain accessible value, source and age descriptions without
  recreating native tooltips in Dusk or Night.
- STBY uses the same hint API. Legacy controls keep their own tooltips.

There is no global tooltip enable/disable setting, global help-provider change,
OS theme change, or altered navigation/control behavior.

The existing isolated object scenario now constructs actual wx controls and
checks palette transitions, hint changes during Night, incoming data updates,
accessible names/help, Day restoration and an unaffected native Legacy control.
The Windows DPI gate also hovers the actual Menu and rail HWNDs. A positive Day
observation proves the native hint path works; Dusk/Night must have no visible
owned native tooltip, and actual screen captures pass the night-surface check.
The hover checks run at 100%, 125% and 150% DPI.

Exact native candidate `8e780edc` passed all twelve hover observations in
[run 36287991989](https://github.com/ThereptileII/Work/actions/runs/36287991989):
Day Menu produced an owned visible native tooltip at each scale; Dusk Menu,
Night Menu and Night SOG produced none. The absence checks lasted at least
1.54 seconds, longer than the observed 0.56–0.66 second Day appearance delay.
The primary-surface night checks also passed. Actual 150% Night Menu and SOG
screenshots were inspected and show dark primary surfaces with four rail values
visible. The Day screenshot itself does not establish tooltip pixels; its
positive native-window observation is recorded separately.

The complete Windows artifact and digests are recorded in
[the native evidence record](../../evidence/beta2-windows-8e780edc.json).
Physical boat review remains a separate gate. This candidate failed its later
endurance harness before sampling; the successful hover gate does not imply
release acceptance.

Local validation: the integrated Linux application compiled and the complete
object scenario passed 25 contract groups with 25 captures, including eleven
actual wx hint assertions. Windows hover-script syntax and repository diff
checks pass. Private local evidence is retained under
`evidence/local/boat-beta2/night-tooltip/`; these local results are separate from the executed native hover gate above.
