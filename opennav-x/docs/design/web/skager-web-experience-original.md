# SKAGER Web Experience

## Overall feeling

The website should feel like a premium modern marine product rather than a traditional marine-electronics website.

Visual direction:

- dark, cinematic marine atmosphere
- large typography
- very little clutter
- strong use of the SKAGER/XNav cyan accent
- real chart/UI elements integrated into the visual design
- smooth movement and transitions
- large photography/video areas
- strong contrast between dark sea/background and illuminated navigation data
- product UI should frequently appear as part of the page rather than isolated screenshots

The first impression should sell the **experience and possibility** before explaining technical details.

The main visitor journey should always naturally move toward:

**Discover SKAGER → Understand why it is different → See it in action → Buy → Create account → Enter My Boat**

The primary CTA throughout the site should therefore be:

**GET SKAGER**

A secondary action in the header should be:

**LOGIN**

---

# 1. Homepage

## Hero

The hero is the most important part of the entire website.

Large statement, very little text.

Example direction:

**NAVIGATION. REIMAGINED.**

Modern navigation, vessel intelligence and passage planning — built around your boat.

Primary button:

**GET SKAGER**

Secondary:

**SEE IT IN ACTION**

The hero should immediately show the product visually.

### Animated hero

This does not necessarily require a traditional video.

A much more distinctive solution could be an entirely code-driven animated marine scene using:

- WebGL
- Canvas
- SVG
- CSS animation

For example:

A dark moving sea/chart background.

A boat moves slowly through the chart.

The route draws itself ahead of the vessel.

AIS targets appear.

Wind vectors move across the chart.

Depth contours fade in.

The data rail updates.

A route event appears ahead.

The display transitions between Day → Dusk → Night.

The camera can then smoothly move closer to the XNav interface.

This could effectively become a **living product demonstration** rather than a movie.

Benefits:

- loads faster than large cinematic video if implemented carefully
- can adapt to screen size
- can react to scrolling
- can reuse actual SKAGER visual components
- looks much more like the real product
- can later become interactive

On mobile or low-performance hardware it should automatically fall back to a simpler animation or static hero.

---

# 2. “Meet SKAGER”

Immediately below the hero.

Very short explanation of the product.

The message should not begin with OpenCPN or technical architecture.

Start with what SKAGER does for the boat owner.

For example:

**One interface for your entire boat.**

Navigation. AIS. Wind. Depth. Energy. Routes. Anchor. Autopilot. Weather.

All presented as one coherent system.

Then show a large XNav interface image or interactive mockup.

---

# 3. Navigation without the clutter

Show the main Navigation view.

Focus on:

- chart-first interface
- routes
- AIS
- contextual information
- touch operation
- Day / Dusk / Night
- decluttering
- readable information hierarchy

Keep text minimal.

Large visual on one side, two or three strong statements on the other.

---

# 4. Know your boat

Introduce the broader “vessel intelligence” idea.

Show:

- depth
- wind
- speed
- heading
- battery
- propulsion
- tanks
- sensor health

Core message:

SKAGER does not treat these as separate instruments.

They become one consistent picture of what the boat is doing.

---

# 5. Electric propulsion & Energy

This can be one of SKAGER's strongest differentiators.

Show:

- SOC
- motor power
- battery power
- range
- route distance
- predicted arrival SOC
- reserve
- speed/energy scenarios

Example message:

**Don't just know your battery percentage. Know whether you can reach your destination.**

Future versions can show an interactive speed slider:

6.0 kn → arrival SOC 38%  
5.5 kn → arrival SOC 47%  
5.0 kn → arrival SOC 56%

This is much more compelling than a normal instrument display.

---

# 6. Smart Passage

Introduce SmartNav without overpromising autonomous navigation.

Show:

- next route event
- ETA
- turn prediction
- wind
- energy
- AIS encounters
- later weather/current/tides

Core message:

**SKAGER helps you understand what is ahead — not just where you are now.**

---

# 7. Weather

Show forecast wind integrated directly into the chart.

Later this uses GRIBstream.

Display:

- forecast wind
- timeline
- route weather
- changing conditions
- eventually waves/current/tides

This section can visually transition the chart through the next 12–24 hours.

---

# 8. Designed for the helm

A section about real onboard use.

Show the physical 1280×800 installation.

Focus on:

- sunlight/touch use
- large targets
- night mode
- simple operation
- minimal menu diving
- works with existing OpenCPN ecosystem

OpenCPN can be mentioned here more clearly:

**Built on the proven OpenCPN navigation engine.**

This gives credibility without making OpenCPN the first thing a new visitor sees.

---

# 9. Future / Connected Boat

Introduce the SKAGER cloud carefully.

The concept should be:

**Your boat has a digital home.**

After buying SKAGER and creating an account, the customer enters:

# MY BOAT

This becomes their digital representation of the vessel.

---

# My Boat / Customer Portal

## Overview

The first view should look more like a modern vehicle app than a normal account dashboard.

Large boat name/photo.

Then major status cards.

Example:

**S/Y Aurora**

Last connected 8 min ago

Battery  
82%

Shore power  
Connected

Position  
Norrköping Archipelago

Wind  
14 kn

Depth  
4.8 m

SKAGER  
v1.4 — Up to date

---

# Software

The customer can:

- download SKAGER
- see installed version
- see latest version
- download recovery package
- view release notes
- manage update channel
- see their registered installations

This should feel much more polished than a normal download page.

---

# Boat Settings

Cloud-synced settings can eventually include:

- vessel name
- dimensions
- draft
- battery capacity
- reserve settings
- propulsion configuration
- preferred units
- display preferences
- sensor-source preferences
- selected dashboards
- energy calibration
- sailing polar selection
- weather settings

Sensitive credentials such as API tokens should not simply be synced as normal settings.

---

# Live Boat

When cloud connectivity is available, My Boat can show the same normalized data that SKAGER sees onboard.

Examples:

- position
- heading
- SOG
- wind
- depth
- battery SOC
- voltage/current
- propulsion state
- tank levels
- shore power
- bilge/sensors
- XNav system health

Every value should show its age.

For example:

**Battery 82% · 3 min ago**

rather than pretending cloud data is always live.

---

# Routes & Passage Planning

Future cloud functionality should include:

- view routes
- create routes at home
- edit routes
- send route to boat
- review planned passage
- weather along route
- predicted energy consumption
- predicted arrival SOC

Sending a route to the boat must never automatically activate it.

The user accepts/activates it onboard.

---

# Boat Health

Cloud should eventually make SKAGER useful even when the owner is away from the boat.

Possible data:

- battery status
- shore-power loss
- low-voltage warning
- bilge alarm
- temperature
- sensor failures
- SKAGER/OpenCPN status
- offline boat
- unusual power consumption

This naturally enables remote notifications.

---

# Alerts

Users could choose which alerts leave the boat.

Examples:

- shore power lost
- battery below 40%
- bilge alarm
- boat moved outside geofence
- boat unexpectedly offline
- anchor alarm
- sensor/system problem

Each alert must show source and time.

---

# Voyages / Logbook

Later the cloud portal can contain automatic trip history.

Each voyage could show:

- route
- track
- distance
- duration
- average/max speed
- battery consumption
- wind
- weather
- motor usage
- important events

Clicking a voyage could open a synchronized replay.

This can become one of SKAGER's strongest long-term features.

---

# Boat Analytics

Over time:

- energy efficiency
- battery health trends
- motor temperature
- propulsion efficiency
- sensor reliability
- sailing performance
- polar performance
- usage history

Eventually SKAGER can provide cautious maintenance insights.

For example:

**Motor temperature at 8 kW has increased approximately 9°C compared with similar passages.**

It should explain the evidence rather than claim certainty.

---

# Backup & Sync

Cloud functionality should include:

- SKAGER configuration backup
- version history
- restore to a new PC
- sync between navigation computers
- route backup
- selected calibration/settings backup

Charts and sensitive credentials should not automatically be copied into cloud backup.

---

# Support

My Boat should also become the support interface.

The user can:

- report bug
- request feature
- upload screenshot
- attach diagnostics
- see previous reports
- see status
- see known issues
- see current release notes

Because Jira is already the development source of truth, this can eventually connect directly to the Jira workflow through the controlled backend.

---

# Account

Much less prominent than My Boat.

Contains:

- profile
- purchases
- entitlement
- invoices
- installations
- privacy/export/delete
- logout

The customer buys **SKAGER for their boat**, not an “account dashboard”.

Therefore the boat should always be the centre of the portal.

---

# Final homepage CTA

After seeing navigation, energy, SmartNav, weather and My Boat, return to a very simple final section.

Large dark background.

Product display or boat underway.

**Your boat deserves a better interface.**

**GET SKAGER**

Small secondary link:

Already own SKAGER? **Log in**

---

# Navigation structure

Public website:

**SKAGER**  
Product  
Smart Navigation  
Energy  
Weather  
My Boat  
Support  

**GET SKAGER**  
**LOGIN**

After login:

**MY BOAT**  
Overview  
Navigation / Routes  
Weather  
Energy  
Voyages  
Boat Health  
Settings  
Software  
Support  

Account/profile becomes a small menu rather than the centre of the experience.

---

# Core design principle

The public website sells the **future of navigation**.

My Boat delivers the **ownership experience after purchase**.

The website should therefore gradually move the visitor from:

**emotion → product → proof → ownership → purchase**

rather than:

**features → specifications → price**.