# First Beta 2 native frame review — partial candidate

Source `a0af22c248f8a81bb8068ccf3f64bd921478060f`, native MSVC Win32 on
Windows x64, [run 36266809347](https://github.com/ThereptileII/Work/actions/runs/36266809347).
Downloaded evidence artifact `10913823466` was verified against SHA-256
`817d0e22cfce211563ce0c734fb64c41d39ba89bca7b5d14465642c1269e142a`.

This is fixture-enabled CI evidence, **not a qualified product package or boat
acceptance**. The native build and 102 integrated tests passed. Mode switching
and shared-profile persistence passed before the navigation smoke stopped at
its obsolete requirement for a visible `Navigation stale` source caption.
The revised header uses that same space for safety alerts. The assertion is
being replaced with explicit critical-position-alert and stale-age checks.

## Screens reviewed at 1280×800

- `01-xnav-unavailable.png`: chart coastline clearly visible; four rail values
  fit, each explicitly unavailable. Center and current Day caption are legible.
- `03-xnav-night.png`: XNav surfaces and chart use the night palette; no white
  sheet. Native compass/GPS remains visually inconsistent and is removed from
  XNav in the next pass without altering the Legacy preference.
- `05-xnav-stale.png`: four retained values visibly muted and labelled
  `STALE 7s`; active alert uses the reserved top slot, without shifting heading
  or another rail value out of view. The fixture is explicitly labelled DEMO.
- `06-xnav-after-legacy.png`: same coastline content survives Legacy → XNav.
- `11-legacy-after-xnav.png`: native OpenCPN menus/toolbars and chart remain.
- `12-safe-shared-profile.png`: Safe retains the chart and standard frame.

## Next pass

Increase primary rail typography, replace the duplicate compass with the
labelled XNav orientation action, use compact chart object cards, and validate
actual pointer interaction into waypoint/route sheets. Repeat these captures
after that pass. Full product screens, 125/150% DPI, fixture-free packaging and
the real boat display remain pending; this partial review does not satisfy them.
