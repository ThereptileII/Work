# Staging 0.4.0-beta2.4 — UI design and interaction review

Visual inspection of the XNav (`--xnav`) interface on 2026-10-08. Two sources:
the boat PC over RustDesk (real sensors, real charts, Windows) and a local Linux
run of `build/xnav-install/bin/opencpn` under Xvfb at 1600x1000 (reliable input,
full screenshots). Observations are design and interaction only; nothing here
records a navigation or release result.

Severity is about the helm, not the code: **A** misleads or blocks the user,
**B** makes the interaction model inconsistent, **C** is presentation.

## Status after verification

Every item below was re-checked against the source and against the running
application before being acted on. Four did not survive that check and are
marked WITHDRAWN; they are recorded rather than deleted so the reasoning is
visible. Screenshots alone proved unreliable for anything behavioural.

| Item | Outcome |
|---|---|
| A1 chart salmon at startup | open — real on the boat, not reproduced on Linux (SCRUM-335) |
| A2 Weather page self-reference | **fixed** and verified on screen (SCRUM-336) |
| A3 wrong Settings tooltip | **fixed** (SCRUM-337) |
| A4 tooltips cover the next item | open — native wxToolTip, needs a custom popup (SCRUM-337) |
| A5 tooltip/label vocabulary | open (SCRUM-337) |
| A6 Instruments clipped | WITHDRAWN — content scrolls (SCRUM-338) |
| A7 safety depth pre-fill | WITHDRAWN — seeded from the user's own value (SCRUM-339) |
| A8 horizon text clipping | WITHDRAWN — not reproducible (SCRUM-338) |
| B, C, D items | open (SCRUM-340 to SCRUM-346) |

Separately, the boat update was blocked by an installer defect found while
installing beta2.4; that is **fixed** here and tracked as SCRUM-334.


## A — Misleading or blocking

### A1. Chart unreadable at startup on the boat (not reproduced on Linux)
On the boat the chart canvas painted a flat salmon wash (land `216,151,146`,
water `203,144,140`) in which land and water were nearly indistinguishable. The
palette button showed the **sun (Day)** icon. One press cleared it and the chart
rendered correctly (land `235,237,227`). The surrounding SKAGER chrome was
unaffected (top bar `22,35,37` before and after), and stock OpenCPN rendered the
same charts correctly at the same moment, so this is not a RustDesk colour
artifact and not an upstream chart problem.

Salmon is not one of the three palettes: Day, Dusk (`78,97,93`) and Night
(`29,41,37`) all render correctly once cycled, and Dusk/Night sample identically
on Linux and on the boat.

**Not reproduced locally.** A fresh profile paints Day correctly, and relaunching
with `nColorScheme=3` (Night) saved paints Night correctly. So the saved-palette
restore path is not sufficient on its own. Needs a restart on the boat to confirm
and to decide whether it is Windows-only, GPU/driver-related, or tied to that
profile. Until then this is the most serious open item: an unreadable chart at
power-on is a navigation problem, not a cosmetic one.

### A2. Weather page sends the user to the page they are on
`Settings › Navigation › Weather` shows: *"Weather forecasts are off. Enable them
in Settings › Weather."* That is the current page. The instruction has nowhere to
lead. Separately, the page offers **Save token**, **Remove token** and **Test
connection** but shows no field to type a token into, so the primary setup action
has no visible input.

### A3. Settings tooltip describes a different feature
The Settings rail item's tooltip reads **"Open navigation menu"**. It opens
Preferences. 

### A4. Every rail tooltip covers the item below it
Tooltips render as opaque black boxes offset down-right of the trigger, landing
squarely on the next rail item: Passage's hides Traffic, Traffic's hides Energy,
Instruments' hides Anchor, Anchor's hides Radar, Radar's hides Settings, and
Settings' hides the bottom panel text. The same happens inside panels — the
"Settings section: Navigation" tooltip covers the first row of the list it
describes. Reaching for a neighbouring item means aiming at a hidden target.

### A5. Tooltips use different words than the labels
`Passage`→"Route", `Traffic`→"AIS targets", `Instruments`→"Vessel instruments",
`Anchor`→"Anchor watch", `Radar`→"Radar availability", `Settings`→"Open
navigation menu". Two vocabularies for one rail.

### A6. WITHDRAWN — Instruments content is reachable
Reported as a clipped, unreachable grid. It is not: the page scrolls by wheel
and touch, revealing Pressure, Rudder and three further actions. The explicit
Up/Down buttons are suppressed on this page deliberately
(`src/ui/Shell.cpp:1142`). What survives is only an affordance point — a card
cut by the Your Horizon band gives no hint that more lies below, which matters
for mouse use more than for the boat's touch display. Tracked and closed as
SCRUM-338; the affordance note folded into SCRUM-346.

### A7. WITHDRAWN — safety depth pre-fill is correct
Raised from the screen, withdrawn after reading the code. Boat setup step 1 says
*"Leave unknown values blank. Blank safety depth preserves the current OpenCPN
setting"* while the field shows `3`. That is not a default overwriting the
user's value: the field is seeded with the user's own current OpenCPN safety
depth (`src/ui/Shell.cpp:1488`) and blanking it restores that value
(`src/ui/BoatSetupDialog.cpp:105`); the model default is NaN
(`src/application/BoatSetup.h:13`). The `3` was simply OpenCPN's own default in
a fresh Linux test profile. No change required. Tracked and closed as SCRUM-339.

### A8. WITHDRAWN — horizon text does not clip
Reported from a startup screenshot showing "Navigation unavailal". Not
reproducible: at 1280x800 the panel renders "Navigation unavailable" in full and
ellipsises the secondary line correctly with a real ellipsis character. The
truncation seen once was the app mid-layout during startup, with the window at
its pre-resize size and the setup dialog overlapping. Closed as SCRUM-338.

## B — Inconsistent interaction model

### B1. The same rail produces three different containers
`Passage`, `Traffic`, `Anchor` and `Settings` open a right-hand drawer over the
chart. `Energy`, `Instruments` and `Radar` replace the whole content area and the
chart disappears. `Settings › Weather` then becomes a full page launched from
inside a drawer. Nothing in the rail distinguishes the three, so the user cannot
predict whether a click keeps the chart visible — which matters when under way.

### B2. Disabled controls keep their full shape
Disabled buttons retain the complete border or fill and differ only by dimmer
label text: "Back" on setup step 1, "End navigation" and "Reverse route",
"Remove token" and "Test connection", and "Set anchor & start watch" while GPS is
unavailable. In sunlight on a moving boat that difference is close to invisible,
and the anchor case is the worst: the main action of a safety feature looks
pressable when it cannot arm.

### B3. The working action is not the prominent one
In the Passage drawer the only enabled action, "Plot a new passage", is styled
exactly like the two disabled buttons above it, while "Passage library" is a bare
text link — three button styles in one stack and no visual primary. In setup step
3 ("Sources") the real action is a small "Check again" at the top while the bright
green button is "Continue", so a user with no sensors detected is steered past the
problem.

### B4. "Later" on step 6 of 6 does not say what it discards
After six screens of input the dismiss action gives no indication whether entries
are kept. Step 1 says changes are saved only at the final step, so "Later" on the
final step is exactly where the user most needs to be told.

### B5. Weather on/off is two buttons, not one control
"Off" and "Enabled" sit side by side as separate buttons; the current state is
only readable from the "Forecasts: Off" text line above. The five buttons on that
page (state, credential, diagnostic) share one undifferentiated 2-column grid.

### B6. "Show wind arrows on chart" shows no state
A full-width button with no indication of whether arrows are currently on.

### B7. "Follow boat" is prominent with no position
The button is one of the largest controls on the chart while the status bar reads
"NO POSITION". Pressing it can do nothing.

## C — Presentation

- **C1. Empty states repeat themselves.** The Radar screen says a variant of
  "unavailable" eight times (subtitle, chip, "NO RADAR IMAGE", "NO VALIDATED
  RECEIVE SOURCE", "No returns available", "Range unavailable", "Source
  unavailable", Guard zone). The Passage drawer states "no passage" four times.
- **C2. Three treatments of one word on one screen.** Energy shows `UNAVAILABLE`
  as a bordered chip, as plain caps text, and as "Unavailable" in sentence case.
- **C3. The "—" placeholder reads as a divider.** It is a thick horizontal rule,
  visually heavier than the unit beside it, so an empty value looks like a rule or
  an empty progress bar rather than "no reading".
- **C4. "At a glance" spends ~750 px on four values**, most of it gaps.
- **C5. Marketing headlines occupy the top of operational pages**: "More horizon.
  Less uncertainty.", "Feel the passage. Read the details.", "Read beyond the
  chart.", "A helm of your own". They are good brand lines and they cost the most
  valuable vertical space on a helm display.
- **C6. Two design languages side by side** at the bottom-left of the chart: the
  large rounded "Follow boat" pill next to a small square scale card.
- **C7. Duplicate position status** — "NO POSITION" and "GPS POSITION
  UNAVAILABLE" sit adjacent in the status bar.
- **C8. "Up"/"Down" scroll buttons appear in the top app bar** on the Weather
  page, next to the clock, displacing the vessel-input status.

## D — Identity

- **D1.** The window title is "OpenCPN 5.12.4" until the UI loads. The first-run
  dialog is titled "Välkommen till OpenCPN", and its text says *Klicka på "OK"*
  while the button is labelled **Acceptera**.
- **D2.** Searching Windows for "skager" returns **OpenCPN 5.12.4-0+37fd0cd** as
  best match and never offers the SKAGER app; the Start "Recommended" tile
  labelled "OpenCPN Legacy" carries the SKAGER icon. Following the obvious path —
  Start, type the product name — lands in the wrong application. This reviewer did
  exactly that and inspected the legacy UI before noticing.

## What was not covered

Touch input, the boat's own display geometry, AIS and radar with live targets,
route and waypoint cards, the autopilot pages, and every state that needs a GPS
fix. The local run has no position, no sensors and no configured vessel, so all
"unavailable" states above are expected there and only the layout of those states
is being judged.
