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
The hover checks run at 100%, 125% and 150% DPI. Native qualification and physical
boat review of this refinement remain pending.

Local validation: the integrated Linux application compiled and the complete
object scenario passed 25 contract groups with 25 captures, including eleven
actual wx hint assertions. Windows hover-script syntax and repository diff
checks pass. Private local evidence is retained under
`evidence/local/boat-beta2/night-tooltip/`; these local results do not qualify
the pending native hover gate.
