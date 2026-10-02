# Native 88c141b focused component review

Exact source `88c141ba9fee5c28e12b5f4ac0f7e7550d1a8ca6`,
[focused run 37032588707](https://github.com/ThereptileII/Work/actions/runs/37032588707).
Artifact `11238311244` was downloaded, hashed and checked against 247 source
inputs, 31 objects, 12 runtime files, three executables and 23 canonical captures.
See [verification](../../evidence/scrum-224-88c-focused-native-proof.json).

## Preferences activation

The negative control `60929a5` restores Advanced battery model focus and scrolls
the body by seven 24px units, moving Sensors out of its recorded pointer target.
The positive trace starts from the same physically selected deep action, but
explicit reopening focuses Vessel before resetting scroll. Subsequent owner
deactivation and drawer activation keep scroll zero, Sensors at
`(808,190,59,37)`, and both actual/cached pointer hits on Sensors. Physical input
then selects Sensors. The before/after positive images are identical; the
negative after-image visibly loses the tabs. The pointer, containment, foreground,
rectangle and scroll assertions were not weakened. Settings passed 230 checks.

## Search and shell

All five Search/Shell captures were inspected against native29 and the retained
Windows Search reference. Day result separators are now y323/y394 (71px rows).
Day/Night focused inputs show the external 2px outline and 3px gap. The title
ink now occupies y126–144, matching the reference. The right/bottom exposed
edges show the expected themed border color. Search passed 85 checks.

Shell 1280 and 853 captures differ from native29 only in the clock; compact
Search changes remain inside the drawer. No new clipping/overlap was found.
Existing compact Instruments ellipsis and timeline truncation remain. The
compact input was unfocused, so compact focus parity was not tested. A one-pixel
title ink-width difference and outline antialiasing differences remain; these
are raster differences, not a geometry tolerance waiver. Saved-object results
remain deliberately distinct from the prototype's illustrative place index.

## Settings and Chart presentation

Reviewed Settings Day, System Night, Display Dusk/Night and all four Chart
captures. Settings/Chart title ink now matches canonical y126–150 (previously
y130–154). Right/bottom pixels match Day `#35464a`, Dusk `#405059` and Night
`#29353b`. Chart's 11px supporting text fits the captured rows; notes use
secondary text, wrap completely and leave the palette action visible. Chart
passed the unchanged 46-check harness. System shows the direct Advanced /
Legacy Settings entry without duplicate mode buttons. Display's partly scrolled
fullscreen control is unchanged from native29.

This accepts only the reviewed corrections and reproduced focus mechanism.
Broader System composition, other DPI/Back-header states, integrated OpenCPN,
real charts, installer, hardware data and boat-display acceptance remain open.
The exact same commit entered [full qualification](https://github.com/ThereptileII/Work/actions/runs/37033702537)
after this review; that run is not yet accepted. The boat was not modified.
