# Staging 0.4.0-beta2.4 — boat check

User checks for the boat-feedback batch (SCRUM-295, SCRUM-314–323) and the new
forecast weather (SCRUM-324–331). This guide does not record a pass; report what
you actually see, including anything that looks wrong.

Before changing anything, use **Settings → System → Export settings backup** to
save this boat's current `.skager-backup` file. Keep the existing profile.

Work through the read-only checks first. Nothing before step 7 sends a command
to any equipment.

## 1. Autopilot appears without a restart (SCRUM-295)

The old build needed the bridge's one-time address claim, so a PC that started
after the pilot showed **Unavailable**. Start SKAGER with the pilot already
powered and the usual Actisense connection in use.

Open **Autopilot**. Expect mode and heading within a few seconds, labelled with
its source. Then open **Settings → Vessel → Autopilot setup**: **Use detected
pilot** should offer the pilot (NAME if its claim was seen, otherwise its status
address). Binding is one confirmation and must leave **CONTROL OFF**.

Switch the pilot off, or disconnect it, and confirm the display degrades to
stale or unavailable rather than holding the last mode.

## 2. Night mode navigation aids (SCRUM-323)

Switch to Night. Buoys, beacons and lighthouses must stay recognizable against
the dark water, with red and green still distinguishable. Compare against Day:
only brightness should differ, never the colour meaning. Report anything that is
still too dark, and anything that now looks too bright.

## 3. Zooming out (SCRUM-317)

Zoom out over an area with many chart objects, the kind that previously became
unreadable or froze the PC. Expect progressive thinning: waypoint and AIS names
drop first, then waypoint markers become compact, and chart objects follow the
chart's own scale rules. Watch for stalls. Zoom back in and confirm detail
returns without missing or duplicated overlays.

## 4. Waypoints, route line and heading line (SCRUM-318, 319, 321)

On an active route, the next point should be a solid marker, passed points
dimmer, and all markers drawn above the route line, never crossed by it. Saved
standalone waypoints use the flag marker with the name below.

Watch the line extending from the boat: its length should grow and shrink with
speed. At rest, expect a short direction tick only, never a long projection.

## 5. Cards open on click and tap (SCRUM-316, 320)

Hovering over a route must no longer open its card. Click a route leg, a
waypoint and an AIS target in turn; each opens its own card. Repeat by touch on
the boat display. Dragging must still pan the chart, and a tap that opens a card
must not also move the chart. With two AIS targets close together, the nearest
one should be selected consistently.

## 6. Routes and anchor (SCRUM-315, 322, 314)

- **Delete route.** Open a route you do not need, choose **Delete route** and
  confirm. Waypoints shared with another route, or saved as marks, must survive.
  Try deleting the active route: expect a message asking you to stop navigation
  first, with nothing deleted. Restart and confirm the deletion persisted.
- **Activate from the card.** Click a route on the chart and use **Activate
  route** on its card. Cancel once first and confirm nothing changed.
- **Anchor mark.** Start the anchor watch, then stop it: the temporary anchor
  mark must disappear. Repeat, but this time activate a route while anchored:
  the watch stops and its temporary mark is removed, while your own waypoints
  near that position stay.

## 7. Forecast wind (SCRUM-324–331) — needs your GRIBstream token

Never tested against the live service; this is its first real request.

**Settings → Navigation → Weather**: enable it, paste your token, **Save**, then
**Test connection**. Expect a clear success or failure, with the token never
shown again. Then check:

- Forecast wind at the boat, labelled **FORECAST** with provider, model and run
  age, clearly separate from measured wind.
- **Chart layers → Wind**: arrows appear, thin out when zoomed out and stay
  readable in Day, Dusk and Night.
- The time steps move the forecast forward and back.
- Open a route: forecast wind per waypoint, with its timing assumption stated
  and **no forecast coverage** shown where there is none.
- Pull the internet connection: the forecast must become visibly **STALE**, with
  its age, and never pass as current. Navigation itself must be unaffected.
- **Remove token** must leave the feature disabled and the token gone.

## 8. Autopilot commands — last, and only with a person at the helm

Separate from everything above (SCRUM-313). Keep the drive clear, someone at the
physical helm, and physical STANDBY available. Permit manual commissioning,
enable the session, then try STANDBY, AUTO and the course steps one at a time,
confirming each against the physical pilot before the next. A command is
confirmed only by the pilot's own measured response; a message that was sent is
not a confirmation.

After a restart, expect **CONTROL OFF** again.

## Known gaps in this candidate

- No live GRIBstream request has ever run; the weather model is fixed to `gfs`
  in the config file, and the Wind layer toggle resets to off on restart.
- Delete route and pointer/touch selection have no automated test; steps 5 and 6
  are their first real verification.
- Windows and boat display are the acceptance authorities. Linux checks passed,
  but they do not qualify this candidate.
