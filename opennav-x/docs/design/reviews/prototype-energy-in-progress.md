# Prototype Energy review — in progress

Reference: immutable HTML `energy` state, 1280×800, Windows Segoe UI.
This is a corrective development increment, not screen or boat acceptance.

## Reference and changes

The content workspace is 1014×698 at (80,68); Energy omits the horizon exactly
as the prototype does. Its two principal cards begin at (112,214) and (618,214),
using the computed 1.1:1 width ratio, 18px gap and 423px height. Header tracking,
60px/350 numeric displays, inline units, battery silhouette, 45px detail rows
and the four 75px power tiles now follow the final CSS cascade. The shared
dashboard-card primitive now paints the missing one-pixel theme border.

First Linux capture pass: all 44 widget checks/seven states passed, but direct
reference comparison exposed the missing border. The corrective second pass
again passes all 44 checks/seven states; Day, Night, stale SOC and shortfall
were inspected. Instruments still passes 41 checks/five states with the shared
card change. All 123 integrated Linux tests pass after the correction.

The comparison retains reference/current/diff images without a broad tolerance.
Linux font fallback is not Windows typography evidence. Same-source native CI
adds the separate Energy widget capture to the existing actual-product suite.

## Navigation-meaning requirements

The native view consumes owned, assessed observations and the existing tested
energy prediction. An arrival estimate must belong to the same current route
publication and observation batch. Stale/missing SOC, stale GPS, changed route,
expired observations and energy shortfall cannot become a zero-SOC arrival.
Valid independent propulsion readings remain visible during battery-source loss.

The prototype's decorative curve is replaced by the existing constant-condition
model's straight line between current and estimated destination SOC. It is
labelled as an estimate; a real configured reserve supplies the reserve line.
Stored kWh also carries an estimate indicator. Actual forecast quality and
failure reasons replace the prototype's illustrative good-quality/margin claims.
All component values are test-only in a separate, non-installed executable.

## Still open

- Below-fold Explore pace/calibration interaction still uses the older operating
  details. No fake slider, calibrated curve or hardware state is introduced.
- Windows font and exact line metrics, full-product Energy theme/Close flow,
  non-default DPI and physical boat comparisons remain required.
- Primary-product settings migration and complete release/installer gates remain
  separate work. No Energy PASS is recorded in the conformance table.

Hashes, both passes and assertions: [evidence](../../evidence/prototype-energy-local.json).

Native development run `36503170626` now passes all 44 Energy component checks
and seven captures, alongside both 115-test integrated suites. Day was compared
with the same-run HTML reference: primary numeric hierarchy, card widths and
power-row composition are improved; the advisory model line remains deliberately
straight for the documented navigation-meaning reason. This run fails a separate
active-route painting assertion and is not stage acceptance. Non-default DPI,
boat review and lower-page migration remain open. Artifact/image hashes are in
[the native record](../../evidence/prototype-native-6091c23-failed.json).
