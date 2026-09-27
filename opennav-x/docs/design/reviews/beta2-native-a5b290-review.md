# Retained native DPI review — a5b290e

This is visual evidence, **not release acceptance**. Commit
`a5b290e52ebeb83bda75c1f7008879c398548dc3` failed the final direct cleanup
deadline in its installer harness. Its subsequent native DPI check passed at
100%, 125% and 150%. The complete replacement run must still pass. Reference:
`docs/design/OpenNavX_Design_Reference.png`.

Run [36307149573](https://github.com/ThereptileII/Work/actions/runs/36307149573),
artifact `10929391865`, 27,761,183 bytes, SHA-256
`fb36fd49b87efa502979ebdb4a38c4fef58d00c6936147a3b7f0ed8445b15aa2`:
API metadata, upload log, independently downloaded bytes and ZIP CRC agree.
Eighteen originals were individually viewed. File hashes are in
`docs/evidence/beta2-native-a5b290-visual-review.json`.

The `dpi-*` images use the isolated fixture-enabled CI executable, with explicit
Demo labelling. They establish layout behavior, not live data or installed
product content. The two `recovery-*` images use the fixture-free product.
Boat-PC screenshots remain a separate required authority.

| Images | Reference intent | Observed result and limit |
| --- | --- | --- |
| `dpi-{100,125,150}-01-navigation-day.png` | Chart dominance, four high-value rail readings | SOG/depth/wind/heading and their units/state fit with the critical alert present. The alert does not create a fifth rail row or push heading below the viewport. Basemap coastlines are visible. This is not licensed ENC or real-boat input evidence. |
| `dpi-{100,125,150}-system-page.png` | Direct recovery actions without an overlapping popup | All eight recovery/diagnostic actions fit. Alerts stays in the header. At 150% the longer alarm heading ellipsizes and explanatory copy is below the first viewport; critical level and Alerts remain visible. Up/Down provide access to expanded content. Day's native hover tooltip and thin desktop edge are visible in these originals and were not cropped away. |
| `dpi-150-night-menu-hover.png` | Low-light navigation and no bright hover surfaces | All four values remain visible. No bright native tooltip is present in this Night capture. Alert color remains brighter than ordinary information. |
| `dpi-150-night-pilot.png` | Clear feedback requirement, accessible manual control | The repaired subtitle says commands require feedback confirmation. Missing commanded heading is explicit. The course-button row extends below the first viewport at 150%; the persistent STBY action remains available in the bottom bar. The expanded page scrolls. Physical reach/legibility and actual command operation are not accepted from this image. |
| `dpi-150-night-energy.png` | Battery, propulsion, destination hierarchy | Large primary values, subordinate electrical/motor values and distinctly estimated arrival fit the first viewport. The synthetic data is explicitly labelled; it proves no live propulsion path. |
| `dpi-150-night-settings.png` | Approachable settings | Seven plain-language categories fit and retain consistent low-light surfaces; no technical protocol IDs lead the page. |
| `dpi-150-night-sheet.png` | Focused dark editing sheet | Battery assumptions, active field, scroll and Cancel/Save remain dark and readable. It intentionally scrolls at this scale; only the first field is visible in the capture. No physical touch usability is inferred. |
| `dpi-150-chart-context.png` | Four contextual chart actions | Compact sheet shows position, Go to/Waypoint/Measure/Info and a clear Close target. Go to is correctly disabled without vessel input. The rail remains visible. |
| `dpi-150-legacy.png`, `dpi-150-returned-xnav.png`, `dpi-150-safe.png`, `dpi-150-safe-to-xnav.png` | Reliable upstream fallback and preserved chart content | Visible coastlines persist through the recorded mode sequence. Legacy/Safe retain their native interface. Fixture-enabled XNav images must not be confused with the installed product. |
| `recovery-navigation-day.png`, `recovery-diagnostics.png` | Real product without synthetic controls; technical details separated | No Demo control/badge; missing vessel input stays unavailable. Diagnostics shows Beta 2, exact a5b290e commit, MSVC and INSTALLED PRODUCT. The disposable profile is not the boat profile. |

These observations close no boat maintenance or physical-touch gate. The
application/installer source matches the boat's 7827acb implementation, but a
shared implementation does not substitute for exact-executable acceptance.
Remaining visual tradeoffs are documented: expanded-page scrolling at high DPI,
ellipsized long alert text with its detail action still present, and native Day
tooltips. Do not call the 150% screens free of all clipping simply because the
automated layout checks passed.
