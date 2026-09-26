# Beta 2 navigation frame — review in progress

## Reference intent

The real chart dominates, with a narrow status row and four readable primary
values. Center and zoom are immediately available. Contextual chart actions use
small sheets. Alerts remain accessible without moving depth, wind or heading.

## Observed Beta 1 problems

The permanent rail could scroll heading out of sight, especially after an alert
added another full-height row. System's tall popup overlapped the alert at 150%
DPI. A fixed Light caption gave no confirmation of the chosen palette. GPS was
not an understandable label for Center/Follow. Menu duplicated chart operations
and commissioning tools alongside everyday navigation.

## Implemented candidate changes

- Four proportional, non-scrolling rail readings: SOG, depth, wind, heading.
  Only the untouched older default migrates; custom persisted choices remain
  available for deliberate selection/reordering in Display.
- Alerts occupy the existing status slot. Chart and rail heights do not change.
- Current Day/Dusk/Night label, vector ownship/Center action and quiet menu icon.
- System is a product page within the normal navigation frame, preserving
  status/alert and STBY access. Technical recording/export tools live there.
- Chart context uses upstream hit-testing and copied coordinates; Waypoint,
  Go To, Measure and Info no longer require a long context menu.
- Route creation has explicit Undo/Cancel/Done and a focused name/save sheet.
  Cancelling the sheet retains the draft; cancellation of the route is confirmed.
- Theme, numeric hierarchy, card spacing and semantic action roles share the
  component implementation. No fixture controls compile into the product.

## Evidence and next review

Local fixture-enabled integrated build and 106 existing regressions passed before
additional AIS clock and interaction checks. These results do not qualify this
screen. Await native candidate images, actual four-region geometry checks at
100/125/150%, then real boat deployment after compatibility/profile preflight.

Review the chart area, font hierarchy, unavailable/stale state, every rail value,
alert hit target, chart context, named route save and mode return. Reject blank
or all-water chart checks. Compare before/after images with the approved design,
then revise this record with exact commit and captured file references.

## First local visual review

Reviewed `evidence/local/objects-timezone-stockholm/objects-settings-return.png`
and the AIS card from the same isolated fixture run. Coastline content is clear,
all four rail regions remain inside 800px, and the System/alert height change is
removed. AIS values are grouped with correct units, existing CPA/TCPA and muted
metadata. These are Linux development observations only.

Further refinement: primary rail numbers can use more of their available space;
the old compass/GPS canvas overlay needs review alongside a clear XNav orientation
control; selected AIS/waypoint views still occupy a full page and should gain
compact contextual presentation. Do not mark these resolved from this first pass.
