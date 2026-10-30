"""Owned, plain-text support content: articles (how-tos and long-form guides) and the FAQ.

Article: slug, title, category, kind ("How-to" or "Guide"), summary, updated (ISO date),
optional applies_to/featured, and sections. A section has a title and a body (one string
or a tuple of paragraphs) and/or a tuple of bullets. FAQ items have a question and an
answer, with an optional link (URL name, label). Keep everything plain text so Django's
normal autoescaping applies to every value.
"""

APPLIES_TO = "SKAGER public beta · Windows · OpenCPN 5.12.4"

CATEGORIES = (
    ("Getting started", "Install SKAGER and OpenCPN and get your boat on screen."),
    ("Charts & navigation", "Charts, routes, display modes and reading the water ahead."),
    ("Instruments & data", "Connect your boat: NMEA, AIS, autopilots and what readings mean."),
    ("Passage planning", "Weather, energy and planning the passage before you leave."),
    ("Recovery & maintenance", "Updates, Legacy and Safe Mode, and useful problem reports."),
    ("Licensing & open source", "OpenCPN, the GPL and the source behind every release."),
)

ARTICLES = (
    # ------------------------------------------------------------------ Getting started
    {
        "slug": "opencpn-prerequisite",
        "title": "Install the supported OpenCPN version",
        "category": "Getting started",
        "kind": "How-to",
        "featured": True,
        "summary": "SKAGER runs on top of OpenCPN. Here is the exact version to install and how to check it.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "The supported combination", "body": (
                "SKAGER works with OpenCPN 5.12.4 on a 64-bit Windows PC, using the Win32 (x86) edition of OpenCPN. "
                "Win32 describes OpenCPN's application and plugin interface; it runs normally on 64-bit Windows.",
                "OpenCPN is installed separately and stays its own application. SKAGER adds a new interface on top of it "
                "and keeps your charts, routes and connections where OpenCPN already stores them.",
            )},
            {"title": "Install and check", "bullets": (
                "Download OpenCPN 5.12.4 for Windows from the official OpenCPN website or its official release page.",
                "Install it and open it once, so it creates its profile.",
                "Open About in OpenCPN and confirm the version reads 5.12.4.",
                "Close OpenCPN before you run SKAGER Setup.",
            )},
            {"title": "If Setup says the version is not supported", "body": (
                "SKAGER Setup checks the exact OpenCPN build, not just its version label. Portable or modified builds can "
                "carry the same number and still differ. If Setup refuses, install the official build and try again. "
                "Never rename files or copy them by hand to get past the check."
            )},
        ),
    },
    {
        "slug": "install-skager",
        "title": "Install SKAGER",
        "category": "Getting started",
        "kind": "How-to",
        "featured": True,
        "summary": "Back up, run Setup, and check your charts and instruments before you head out.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Before Setup", "bullets": (
                "Install and check the supported OpenCPN version first.",
                "Back up your OpenCPN profile and any charts stored outside it. Keep the backup outside the program folders.",
                "Close OpenCPN and any open SKAGER, Legacy or Safe Mode windows.",
                "Install while the boat is safely moored, with another means of navigation available.",
            )},
            {"title": "Run Setup", "body": (
                "Download SKAGER from your account and run Setup. Select your installed OpenCPN when asked. Setup checks "
                "compatibility, shows you what it will change and where it keeps its backup. Read the result before you "
                "continue; Setup stops rather than guessing if something is not right.",
            )},
            {"title": "After installation", "bullets": (
                "Start SKAGER from the Start menu or desktop shortcut.",
                "Check that your charts, routes, tracks and waypoints are all there.",
                "Open the instrument rail and confirm each value shows the source you expect and a fresh age.",
                "Find Legacy and Safe Mode in the Start menu, so you know your way back before you need it.",
            )},
        ),
    },
    {
        "slug": "boat-pc-setup",
        "title": "Running OpenCPN on a boat PC: a practical setup guide",
        "category": "Getting started",
        "kind": "Guide",
        "featured": True,
        "summary": "Power, screens, connections and Windows settings for a navigation PC that behaves at sea.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "Why a dedicated navigation PC", "body": (
                "A laptop on the chart table works, but a dedicated boat PC is calmer to live with. It is always in the "
                "same place, always connected to the same instruments, and it does not leave the boat with your email. "
                "OpenCPN asks little of the hardware, so a small, quiet machine is enough.",
            )},
            {"title": "Choosing the hardware", "bullets": (
                "Pick a fanless mini PC with an SSD. No fan means no salt air pulled across the board, and no spinning disk to wear out in a seaway.",
                "Prefer a 12 V input or a quality DC-DC supply. Engine starts can dip the battery voltage; a supply that rides through the dip saves you a reboot at the worst moment.",
                "Mount it somewhere dry and ventilated, away from the bilge and the companionway spray.",
                "8 GB of memory and a recent low-power processor are plenty for charts, AIS and instruments.",
            )},
            {"title": "A screen you can read at the helm", "body": (
                "Brightness matters more than resolution. A sunlight-readable display of around 1000 nits stays legible in "
                "full sun; an office monitor washes out. Projected-capacitive touch with a water or glove mode keeps working "
                "with wet fingers. SKAGER is designed around a 1280 × 800 helm display with large touch targets, and scales "
                "up to larger screens.",
                "At night you want the opposite: a display that can go very dim. Check the panel's minimum brightness before you buy.",
            )},
            {"title": "Connecting your instruments", "body": (
                "Most boats talk NMEA 0183, NMEA 2000 or both. Use a dedicated gateway for each: a USB NMEA 2000 gateway for "
                "the backbone and an isolated USB-to-NMEA 0183 adapter for older instruments. Galvanic isolation prevents "
                "ground loops between the PC and the boat's electrics, a common cause of strange readings and dropped connections.",
                "Add each connection in OpenCPN once. SKAGER uses the same connections, so there is nothing to configure twice.",
            )},
            {"title": "Windows settings that matter underway", "bullets": (
                "Set the power plan to never sleep and never turn off USB devices.",
                "Schedule Windows Update for when you are moored, and set active hours that cover your sailing day.",
                "Turn off notifications and startup apps you do not need at the helm.",
                "Keep the PC for navigation. Fewer programs means fewer surprises.",
            )},
            {"title": "Backups and a way back", "body": (
                "Back up your OpenCPN profile and charts to a USB drive at the start of the season and after big route-planning "
                "sessions. With SKAGER installed you also have Legacy, the standard OpenCPN interface, and Safe Mode for "
                "troubleshooting. And keep an independent means of navigation on board, whatever your PC setup.",
            )},
        ),
    },
    # ------------------------------------------------------------------ Charts & navigation
    {
        "slug": "charts-and-plugins",
        "title": "Keep your charts and plugins ready",
        "category": "Charts & navigation",
        "kind": "How-to",
        "summary": "SKAGER uses the charts and plugins you already have in OpenCPN. Here is how to keep them working.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Your charts stay yours", "body": (
                "SKAGER shows the charts configured in OpenCPN. It does not include charts or chart subscriptions; buy and "
                "update them from your chart provider as before.",
            )},
            {"title": "If charts are missing", "bullets": (
                "Open Legacy and check the chart folders in OpenCPN's chart settings.",
                "Make sure the chart files are reachable and any chart plugin and licence are installed.",
                "Take a backup before rebuilding the chart database. Never delete chart files to fix a display problem.",
            )},
            {"title": "Plugins", "body": (
                "SKAGER uses OpenCPN 5.12.4's Win32 plugin interface. Install plugins built for that version and check the "
                "ones you depend on after each update. A plugin built for another version or architecture may not load.",
            )},
            {"title": "Before you go", "body": (
                "Check chart coverage for your whole passage, at a scale suitable for the waters you will sail. An empty area "
                "on screen is missing data, never clear water.",
            )},
        ),
    },
    {
        "slug": "charts-for-opencpn",
        "title": "Which charts work with OpenCPN? ENC, o-charts and raster explained",
        "category": "Charts & navigation",
        "kind": "Guide",
        "featured": True,
        "summary": "Vector or raster, free or licensed: how electronic charts differ and what to choose for your waters.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "Vector and raster, in one minute", "body": (
                "A raster chart is a picture of a paper chart. It looks familiar, but zooming in only makes the pixels bigger, "
                "and you cannot switch information on or off.",
                "A vector chart (an ENC) is a database of objects: coastlines, depth areas, buoys, lights and soundings. The "
                "software draws it, so text stays readable at any zoom, depths can be shaded around your own safety depth, and "
                "you can declutter what you do not need. Most hydrographic offices now publish vector first, and several have "
                "retired their raster charts.",
            )},
            {"title": "S-57 ENC: the official vector format", "body": (
                "S-57 is the international standard for official electronic navigational charts. Some countries publish their "
                "ENCs free of charge; the United States is the best known example. OpenCPN opens unencrypted S-57 cells directly.",
                "Where ENCs are sold rather than given away, they are usually distributed encrypted, so the hydrographic office "
                "can control licences and updates.",
            )},
            {"title": "o-charts for OpenCPN", "body": (
                "o-charts sells official chart data from many hydrographic offices in formats made for OpenCPN: oeSENC and oeRNC. "
                "You buy a chart set, install the o-charts plugin and activate it for your computer. It is the most common way to "
                "get licensed official charts into OpenCPN for European waters.",
            )},
            {"title": "Choosing for your waters", "bullets": (
                "Start with the official charts for your cruising area, at the scales you will actually navigate by.",
                "Check the edition date and keep charts updated; buoyage and depths do change.",
                "Make sure licences cover the computer you navigate with, and keep the licence files in your backup.",
                "Use community or crowd-sourced maps as extra context, not as your primary chart.",
            )},
            {"title": "Charts in SKAGER", "body": (
                "SKAGER draws on whatever charts OpenCPN has configured, with SKAGER's day, dusk and night presentation on top. "
                "Your chart choices, licences and plugins carry straight over.",
            )},
        ),
    },
    {
        "slug": "night-vision-display",
        "title": "Night vision at the helm: day, dusk and night display modes",
        "category": "Charts & navigation",
        "kind": "Guide",
        "summary": "Why a bright screen ruins your night sight, and how display modes keep your eyes ready for the water.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "What a bright screen does to your eyes", "body": (
                "Your eyes take twenty to thirty minutes to fully adapt to darkness. A glance at a bright white screen undoes much "
                "of that in seconds. Lights on the horizon, the loom of a headland or an unlit buoy get harder to see just when "
                "you need them.",
            )},
            {"title": "Day, dusk and night palettes", "body": (
                "Electronic charts have long used separate colour schemes for each light level: a bright, high-contrast day "
                "palette, a darker dusk palette and a dim night palette with low-luminance colours on a near-black background. "
                "The same chart, re-coloured so the brightest thing on screen never dazzles.",
                "SKAGER applies this to the whole interface, not only the chart. Instruments, cards and controls all switch "
                "together, so no white panel suddenly lights up the cockpit.",
            )},
            {"title": "Good habits for night passages", "bullets": (
                "Switch to night mode before sunset fades, not after your eyes have adjusted.",
                "Turn the display's own brightness right down as well; the palette and the backlight work together.",
                "Use red or dim lighting in the cockpit and below, and keep white light off the helm.",
                "Look at the screen briefly and deliberately, then back to the water.",
            )},
        ),
    },
    {
        "slug": "anchor-watch",
        "title": "Anchor watch: choosing a swing radius that works",
        "category": "Charts & navigation",
        "kind": "Guide",
        "summary": "Scope, swing radius and alarm settings that wake you when the anchor drags, and not every time the wind shifts.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "Start with the right scope", "body": (
                "Scope is the length of rode you let out compared with the depth plus the height of your bow roller above the "
                "water. With all chain, 4:1 to 5:1 is a common starting point in good holding; with rope, more. Expect a blow, "
                "let out more.",
            )},
            {"title": "Work out your swing circle", "body": (
                "Your boat swings around the anchor, not around where you were when you dropped it. The radius is roughly the "
                "horizontal length of rode plus the length of your boat. Your GPS antenna may also sit some way from the bow, "
                "which adds a little more.",
                "Set the alarm a little larger than that radius. Too tight and every wind shift wakes you; too loose and you "
                "drag a long way before you know.",
            )},
            {"title": "Set the alarm in the right place", "bullets": (
                "Mark the anchor's position when it goes down, not where the boat settles afterwards.",
                "Check the circle against the chart: other boats, shallows and the shore should all be outside your swing.",
                "Allow for changing water level. Tides, or wind-driven levels in the Baltic, change both depth and swing.",
                "Watch the track for the first half hour. A neat arc means the anchor is holding; a straight line means it is not.",
            )},
            {"title": "Anchor watch in SKAGER", "body": (
                "SKAGER's anchor view shows your swing circle on the chart with your track inside it, and raises an alarm "
                "when you leave it. The alarm needs a fresh position: if GPS is lost, SKAGER says so rather than assuming "
                "you are still in place.",
            )},
        ),
    },
    # ------------------------------------------------------------------ Instruments & data
    {
        "slug": "phone-as-gps",
        "title": "Use an old phone as your boat's GPS",
        "category": "Instruments & data",
        "kind": "Guide",
        "featured": True,
        "summary": "A spare phone can feed its position to SKAGER over Bluetooth, a cable or Wi-Fi. Add a tablet with SKAGER Helm and you have a full navigation station.",
        "updated": "2026-10-30",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Why an old phone works so well", "body": (
                "Every smartphone has a satellite receiver, and it keeps working without a SIM card or a phone plan. "
                "That makes the phone in your drawer a capable GPS for a laptop that has none. A free app sends the "
                "phone's position to the laptop in NMEA 0183, the standard language OpenCPN and SKAGER already understand.",
                "Without mobile data the first fix can take a few minutes. After that, the phone tracks your position as well "
                "as it does on land.",
            )},
            {"title": "The setup we like", "bullets": (
                "A laptop at the chart table runs SKAGER, with your charts and routes.",
                "An old phone near a window sends its position to the laptop.",
                "A tablet in the cockpit runs SKAGER Helm and shows the same screen at the wheel.",
                "The laptop and the tablet share a Wi-Fi network: the boat's router, or the hotspot of a phone.",
            )},
            {"title": "The app: Share GPS", "body": (
                "We use Share GPS from Google Play or the App Store. It sends standard NMEA over Bluetooth, a USB cable or "
                "Wi-Fi, keeps running with the screen off and has no ads. The free version holds one saved connection, which "
                "is all you need here. Android phones can use all three connections; iPhones use Wi-Fi.",
            )},
            {"title": "Android over Bluetooth (recommended)", "bullets": (
                "Install Share GPS on the phone and allow location and nearby devices.",
                "In Share GPS, add an NMEA connection, choose Bluetooth and tap Connect. The phone now waits for the laptop.",
                "Pair the phone with the laptop in Windows Settings, under Bluetooth & devices.",
                "Open More Bluetooth settings, then COM Ports. Add an Outgoing port for the phone and note its number, for example COM5.",
                "In OpenCPN, open Options, then Connections, and add a serial connection on that COM port with the NMEA 0183 protocol.",
                "Within a minute SKAGER shows your position, and the instrument rail shows the new connection as its source.",
            )},
            {"title": "Android with a USB cable", "body": (
                "A cable also charges the phone, which is handy on long passages. Windows reaches the phone through the "
                "Android debug bridge, so this route suits the technically minded.",
            ), "bullets": (
                "On the phone, turn on Developer options and USB debugging, then connect the cable and allow the laptop.",
                "In Share GPS, add an NMEA connection, choose USB and tap Connect. It uses port 10110 unless you change it.",
                "On the laptop, install Android platform tools and run: adb forward tcp:10110 tcp:10110",
                "In OpenCPN, add a network connection: TCP, address 127.0.0.1, port 10110.",
                "Run the adb command again after the laptop restarts or the cable is reconnected.",
            )},
            {"title": "iPhone over Wi-Fi", "body": (
                "iPhones do not offer a Bluetooth serial port, so they share their position over Wi-Fi instead. The iPhone "
                "and the laptop must be on the same network, either the boat's Wi-Fi or the iPhone's own hotspot.",
            ), "bullets": (
                "In Share GPS, add an NMEA connection, choose Network, set the mode to Server and the port to 10110, then tap Start.",
                "Allow Local Network access when the iPhone asks.",
                "In OpenCPN, add a network connection: TCP, the iPhone's Wi-Fi address and port 10110.",
            )},
            {"title": "Add the tablet in the cockpit", "body": (
                "Install SKAGER Helm on the tablet and join the same Wi-Fi as the laptop. In SKAGER, open Settings, System, "
                "SKAGER Helm and choose Pair a phone or tablet, then scan the code with the tablet. From then on the tablet shows your SKAGER screen at the wheel, while the laptop "
                "stays dry below.",
            )},
            {"title": "Keep it reliable", "bullets": (
                "Keep the phone on a charger. A phone streaming its position all day uses far more battery than one in a pocket.",
                "On Android, set Share GPS to Unrestricted under the app's battery settings, so the system never pauses it.",
                "Place the phone where it sees the sky, by a window or under a hatch. Metal and carbon hulls block the signal.",
                "Check the position against a known landmark before you rely on it, and watch its age in the instrument rail.",
                "A phone is a fine everyday GPS. Keep an independent means of navigation on board, as always.",
            )},
        ),
    },
    {
        "slug": "nmea-0183-2000-signal-k",
        "title": "NMEA 0183, NMEA 2000 and Signal K explained",
        "category": "Instruments & data",
        "kind": "Guide",
        "featured": True,
        "summary": "The three languages your boat's instruments speak, and how to get them all onto one screen.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "NMEA 0183: the classic serial link", "body": (
                "NMEA 0183 sends lines of text over a simple serial connection, usually at 4800 baud, or 38400 for AIS. Each "
                "sentence starts with a talker and a type, such as a position, a depth or a wind reading. It is slow and simple, "
                "and an output is meant to feed listeners rather than share a bus, so several devices need a multiplexer to "
                "combine their data.",
            )},
            {"title": "NMEA 2000: the network backbone", "body": (
                "NMEA 2000 is a network. Devices plug into one backbone cable with a terminator at each end, and every device "
                "can see every message. It runs at 250 kbit/s and carries power for small devices. Data travels as numbered "
                "messages called PGNs: one for position, one for water depth, one for wind, one for AIS reports, and so on.",
                "Brand names hide it: SeaTalkNG and SimNet are NMEA 2000 with their own connectors. Adapters join them to "
                "a standard backbone.",
            )},
            {"title": "Signal K: an open data model", "body": (
                "Signal K is not a cable but a way of describing boat data in an open, modern format. A small Signal K server "
                "collects NMEA 0183 and NMEA 2000, converts it, and offers it over your boat's network. It is popular with "
                "people who like to build their own boat systems.",
            )},
            {"title": "Getting it all onto one screen", "bullets": (
                "For an NMEA 2000 backbone, use a USB gateway. OpenCPN 5.12 reads NMEA 2000 directly.",
                "For older NMEA 0183 instruments, use an isolated USB adapter or a multiplexer.",
                "If you run a Signal K server, add it as a connection in OpenCPN.",
                "Add every connection in OpenCPN once. SKAGER uses the same connections and shows each value with its source.",
            )},
            {"title": "One value, several sources", "body": (
                "Many boats have two sources for the same thing, such as a chartplotter GPS and an AIS transponder's GPS. "
                "SKAGER shows which source a value comes from and how fresh it is, so you always know what you are reading.",
            )},
        ),
    },
    {
        "slug": "ais-for-leisure-boats",
        "title": "AIS for leisure boats: receivers, transponders and online AIS",
        "category": "Instruments & data",
        "kind": "Guide",
        "featured": True,
        "summary": "What AIS shows, the difference between Class A and B, and where online AIS fits in.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "What AIS does", "body": (
                "AIS (Automatic Identification System) has ships broadcast who they are, where they are and where they are "
                "going, over two VHF channels. On your chart, every target becomes a symbol with a course line, and you can "
                "see how close it will pass and when: the closest point of approach (CPA) and the time until then (TCPA).",
            )},
            {"title": "Receiver or transponder", "body": (
                "A receiver only listens: you see others, they do not see you. A transponder listens and transmits, so you "
                "appear on their screens too. For coastal and offshore sailing, being seen is worth a lot, especially in "
                "busy shipping lanes and at night.",
            )},
            {"title": "Class A and Class B", "body": (
                "Commercial ships carry Class A: more power and frequent updates, every few seconds when underway. Leisure "
                "boats usually fit Class B, which transmits less often and with less power. Large ships may filter Class B "
                "targets in crowded waters, so never assume you have been seen.",
            )},
            {"title": "Range and antennas", "body": (
                "AIS is line-of-sight VHF. Range depends mostly on antenna height: ship to ship, twenty nautical miles is "
                "typical; a small boat's low antenna sees less. A splitter lets one masthead antenna serve both VHF radio and AIS.",
            )},
            {"title": "Online AIS", "body": (
                "Online AIS services collect reports from receivers on shore and share them over the internet. They are useful "
                "for seeing traffic beyond your own radio range, planning a crossing or checking a harbour before you arrive.",
                "Online data can be delayed and has gaps, so it complements onboard AIS rather than replacing it. SKAGER shows "
                "online targets separately, keeps onboard reports first when both describe the same ship, and never uses online "
                "targets for collision alarms.",
            )},
        ),
    },
    {
        "slug": "onboard-online-ais",
        "title": "Set up onboard AIS and Online AIS",
        "category": "Instruments & data",
        "kind": "How-to",
        "summary": "Your receiver comes first; Online AIS adds traffic beyond radio range with your own key.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Onboard AIS", "body": (
                "Connect your AIS receiver or transponder in OpenCPN as usual. SKAGER uses that connection, shows targets on "
                "the chart and in Traffic, and keeps OpenCPN's collision alarms and CPA/TCPA calculations.",
            )},
            {"title": "Adding Online AIS", "bullets": (
                "Create a free key with AISStream.",
                "In SKAGER, open Settings and enter the key under Online AIS. Never paste it into a screenshot or support report.",
                "Choose the area you want online traffic for. It needs an internet connection.",
            )},
            {"title": "How the two work together", "body": (
                "When both feeds report the same ship, the onboard report wins, and a newer online report never hides a stale "
                "onboard one. Online targets are marked as online, can be delayed, and never trigger collision alarms.",
            )},
            {"title": "If the online list is empty", "body": (
                "Check the service status in Settings, your internet connection and the selected area. Online services can be "
                "quiet in some areas; an empty list does not mean there is no traffic.",
            )},
        ),
    },
    {
        "slug": "sensors-and-stale-data",
        "title": "Read instruments and data quality",
        "category": "Instruments & data",
        "kind": "How-to",
        "summary": "Every value in SKAGER shows its source and age. Here is how to read them.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Unavailable is not zero", "body": (
                "A depth of zero is a measurement; no depth is a missing sensor. SKAGER keeps them apart: missing, stale and "
                "estimated values are marked as such and never shown as zero or as live.",
            )},
            {"title": "Source and age", "body": (
                "Each instrument shows where its value comes from (GPS, NMEA 2000, calculated) and how old it is. If a value "
                "looks wrong, check its source first, in the instrument rail or under Settings › Sensors.",
            )},
            {"title": "Choosing a source", "body": (
                "With automatic selection, SKAGER picks the freshest source by your priority list. If you pin a source and it "
                "stops, the value shows as stale rather than silently switching to another device.",
            )},
            {"title": "Know what you are measuring", "bullets": (
                "Depth is usually measured below the transducer, not below the keel. Set your offset.",
                "Apparent wind and true wind are different; true wind needs speed and heading as well.",
                "Battery power and motor power differ; check the device and current direction before trusting energy figures.",
            )},
        ),
    },
    {
        "slug": "autopilot-and-chartplotter",
        "title": "Connecting an autopilot to your chart plotter: what to know",
        "category": "Instruments & data",
        "kind": "Guide",
        "summary": "Heading hold, track mode and wind mode, how they connect, and how to stay in charge.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "The parts of an autopilot", "body": (
                "An autopilot has a course computer, a heading sensor (a fluxgate or electronic compass) and a drive: a tiller "
                "ram, a wheel drive or a hydraulic pump. The control head lets you engage it and adjust the course. A chart "
                "plotter adds information; it does not replace any of those parts.",
            )},
            {"title": "The modes", "bullets": (
                "Standby: you steer, the pilot does nothing.",
                "Auto: the pilot holds a compass heading. ±1° and ±10° steps adjust it.",
                "Track or Nav: the pilot follows a route, using cross-track error and the bearing to the next waypoint from your plotter.",
                "Wind: the pilot holds an angle to the apparent wind.",
            )},
            {"title": "How they connect", "body": (
                "Modern pilots sit on NMEA 2000 and receive navigation data as standard messages. Older pilots take NMEA 0183 "
                "sentences with cross-track error and waypoint bearing, or a manufacturer bus such as the original SeaTalk "
                "through a converter.",
            )},
            {"title": "Stay in charge", "bullets": (
                "Know where Standby is on your control head and practise hand-steering takeovers.",
                "Watch the first turn at each waypoint in track mode; some pilots ask you to confirm.",
                "Keep a lookout. An autopilot steers a course, it does not see the water.",
                "Never leave the helm unattended near hazards, traffic or in poor visibility.",
            )},
            {"title": "Autopilot control in SKAGER", "body": (
                "SKAGER can show your pilot's state and, when you enable it, send Standby, Auto and heading adjustments to "
                "supported pilots. Control is off until you switch it on and confirm, every command waits for the pilot's "
                "feedback, and SKAGER's advisory features never steer the boat.",
            )},
        ),
    },
    # ------------------------------------------------------------------ Passage planning
    {
        "slug": "grib-wind-forecasts",
        "title": "Reading wind forecasts on your chart: GRIB files explained",
        "category": "Passage planning",
        "kind": "Guide",
        "featured": True,
        "summary": "Where forecast wind comes from, what the arrows mean, and when to trust them.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "What a GRIB forecast is", "body": (
                "GRIB is the standard format for weather model output: a grid of values, such as wind speed and direction, for "
                "a series of forecast times. Chart software turns the grid into arrows or barbs on your chart, so you can see "
                "the wind along your route hour by hour.",
            )},
            {"title": "Where the data comes from", "body": (
                "Global models such as NOAA's GFS run four times a day on a grid of about a quarter of a degree. Regional models "
                "use finer grids over smaller areas. Every forecast has a source, a run time and a valid time; good software "
                "shows all three.",
            )},
            {"title": "What the model cannot see", "bullets": (
                "Local effects: sea breezes, wind bending around headlands and funnelling between islands.",
                "Gusts and squalls, which can be far stronger than the average wind on the grid.",
                "Detail smaller than the grid. In an archipelago, a whole island group can sit inside a single grid cell.",
                "Change after the run time. An old forecast is a stale forecast.",
            )},
            {"title": "Using forecasts well", "body": (
                "Compare forecasts from more than one model, check the run time, and look at the trend rather than a single "
                "arrow. Plan the passage around the weather window, then keep watching the real wind on your instruments.",
            )},
            {"title": "Forecast wind in SKAGER", "body": (
                "SKAGER draws forecast wind across your chart and labels it with its source, time and age, separately from your "
                "measured wind. Where the forecast has no coverage, SKAGER says so instead of filling the gap.",
            )},
        ),
    },
    {
        "slug": "electric-boat-range",
        "title": "Planning an electric boat passage: range, reserve and arrival charge",
        "category": "Passage planning",
        "kind": "Guide",
        "summary": "How speed, distance and conditions turn into kilowatt-hours, and how much reserve to keep.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "Energy is power times time", "body": (
                "A passage uses energy: the motor's power multiplied by the hours it runs. Six kilowatts for two hours is twelve "
                "kilowatt-hours. Add the boat's other loads, such as instruments, fridge and lights, and compare the total with "
                "the battery's usable capacity.",
            )},
            {"title": "Speed is the big lever", "body": (
                "For a displacement hull, the power needed rises steeply with speed, roughly with its cube near hull speed. A "
                "little slower can mean a lot further. Many electric boaters find their best range at well below hull speed.",
            )},
            {"title": "Usable capacity, not the label", "bullets": (
                "Lithium iron phosphate batteries can use most of their rated capacity; lead-acid far less.",
                "Cold reduces available capacity.",
                "Battery management systems may cut off before zero to protect the cells.",
                "Measure the whole boat's consumption at the battery, not only at the motor.",
            )},
            {"title": "Keep a reserve", "body": (
                "Head wind, current and waves can raise consumption sharply. Plan to arrive with a real reserve; many owners use "
                "a fifth to a third of capacity. Decide your reserve before you leave, not when the numbers start to fall.",
            )},
            {"title": "Energy planning in SKAGER", "body": (
                "SKAGER puts your battery in the context of your route: charge now, estimated charge at arrival, remaining range "
                "and how far above your reserve you are, with the assumptions behind each estimate in view. Estimates need fresh "
                "battery data and a configured capacity; without them SKAGER shows the gap rather than a guess.",
            )},
        ),
    },
    {
        "slug": "energy-and-smartnav",
        "title": "Configure energy estimates and SmartNav",
        "category": "Passage planning",
        "kind": "How-to",
        "summary": "Set capacity and reserve, check your readings, and read SmartNav's advice.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Set up your battery", "body": (
                "Enter the usable capacity and the reserve you want to keep, and select the battery monitor that measures the "
                "whole boat's consumption. Check the current direction: charging and discharging must not be swapped.",
            )},
            {"title": "Reading the estimate", "body": (
                "Range and arrival charge assume your current speed and consumption. Wind, current, load and temperature change "
                "them. Treat them as a guide, keep your reserve, and watch how the estimate develops.",
            )},
            {"title": "When an estimate disappears", "body": (
                "Missing or old battery data, no active route, or a changed route can make an estimate unavailable. Check the "
                "source; never type in a guessed capacity just to make a number appear.",
            )},
            {"title": "SmartNav", "body": (
                "SmartNav looks ahead along your route and brings the next turn, traffic and conditions into view. It advises; it "
                "never steers, and it cannot see hazards that are not in your charts and instruments.",
            )},
        ),
    },
    # ------------------------------------------------------------------ Recovery & maintenance
    {
        "slug": "updates-repair-uninstall",
        "title": "Update, repair or uninstall SKAGER",
        "category": "Recovery & maintenance",
        "kind": "How-to",
        "summary": "Keep SKAGER current, repair an installation, or remove it while keeping your OpenCPN data.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Updates", "body": (
                "SKAGER tells you when an update is available, with release notes. Install updates while moored, after a backup. "
                "Each update changes only SKAGER's own files; OpenCPN and your navigation data stay as they are.",
            )},
            {"title": "Before any change", "bullets": (
                "Note your SKAGER and OpenCPN versions.",
                "Back up your OpenCPN profile and separately stored charts.",
                "Close every SKAGER, Legacy and Safe Mode window.",
            )},
            {"title": "If an update or repair fails", "body": (
                "Stop, keep the message and the backup location Setup shows, and start Legacy or Safe Mode. Do not delete the "
                "OpenCPN profile to make Setup continue.",
            )},
            {"title": "Uninstall", "body": (
                "Uninstall SKAGER from Windows Settings. It removes SKAGER and leaves OpenCPN, your charts, routes and "
                "waypoints in place. Keep your backup until you have checked OpenCPN opens normally.",
            )},
        ),
    },
    {
        "slug": "legacy-safe-recovery",
        "title": "Use Legacy and Safe Mode",
        "category": "Recovery & maintenance",
        "kind": "How-to",
        "featured": True,
        "summary": "Switch back to the classic OpenCPN interface, or start with the minimum to sort out a problem.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Legacy", "body": (
                "Legacy opens the familiar OpenCPN interface with the same charts, routes and connections. Use it whenever you "
                "want the classic layout, or to reach an OpenCPN setting directly. Switching may restart the application.",
            )},
            {"title": "Safe Mode", "body": (
                "Safe Mode starts without the SKAGER interface and without optional plugins, to work out why something does not "
                "start. Your profile is kept; some features are deliberately off while you are in it.",
            )},
            {"title": "If SKAGER will not start", "bullets": (
                "Start Safe Mode or Legacy from the Start menu and note the time and any message.",
                "Check charts and navigation data before changing settings.",
                "Save a diagnostic bundle and review it before sharing it with support.",
            )},
        ),
    },
    {
        "slug": "diagnostics-and-privacy",
        "title": "Send a useful, private problem report",
        "category": "Recovery & maintenance",
        "kind": "How-to",
        "summary": "What to include, how to create a diagnostic bundle, and what to leave out.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "A good report", "bullets": (
                "Your SKAGER, OpenCPN and Windows versions, and roughly when the problem happened.",
                "What you expected, what happened, and the steps that make it happen again.",
                "Whether it also happens in Legacy or Safe Mode.",
            )},
            {"title": "Diagnostic bundle", "body": (
                "Choose Field diagnostic bundle from the menu or Settings. It creates a ZIP file on your PC; nothing is uploaded. "
                "The standard bundle leaves out positions, route names, AIS identities, network addresses and credentials, and "
                "keeps the versions, health and data quality that help us find the problem.",
            )},
            {"title": "Keep it private", "body": (
                "Review the bundle before you share it. Recordings and full logs can contain more than the standard bundle. "
                "Never send passwords, API keys, payment details or your whole OpenCPN profile.",
            )},
        ),
    },
    # ------------------------------------------------------------------ Licensing & open source
    {
        "slug": "licensing-and-source",
        "title": "Licences and source code",
        "category": "Licensing & open source",
        "kind": "How-to",
        "summary": "OpenCPN credit, your open-source rights, and where to find the source for every release.",
        "updated": "2026-10-15",
        "applies_to": APPLIES_TO,
        "sections": (
            {"title": "Built with OpenCPN", "body": (
                "SKAGER builds on OpenCPN, the open-source chart plotter, and credits its contributors. OpenCPN is installed "
                "separately. SKAGER is an independent product and is not endorsed by the OpenCPN project.",
            )},
            {"title": "Your rights stay", "body": (
                "Parts of SKAGER are covered by the GNU General Public License and other open-source licences. Buying Public "
                "Beta Access pays for our services: downloads, updates, support and your account. It never limits the rights "
                "those licences give you.",
            )},
            {"title": "Source for each release", "body": (
                "Every SKAGER release has a matching source package and licence notices, linked from the Open Source page and "
                "from About in the application. You can get them without an account.",
            )},
            {"title": "Charts are separate", "body": (
                "Chart licences come from your chart provider and are not changed by using them with SKAGER.",
            )},
        ),
    },
    {
        "slug": "opencpn-and-the-gpl",
        "title": "OpenCPN and the GPL: what open source means for boat owners",
        "category": "Licensing & open source",
        "kind": "Guide",
        "summary": "Why open-source navigation software matters, and what the GPL guarantees you.",
        "updated": "2026-10-15",
        "sections": (
            {"title": "OpenCPN in a nutshell", "body": (
                "OpenCPN is a free, open-source chart plotter and navigation program, developed by volunteers since 2010 and "
                "used by sailors all over the world. It runs on Windows, macOS, Linux and Android, reads official vector "
                "charts, and has a large ecosystem of plugins.",
            )},
            {"title": "What the GPL gives you", "bullets": (
                "Freedom to run the program for any purpose.",
                "Access to the source code of the program you received.",
                "Freedom to change it and to share your changes under the same licence.",
                "These rights travel with the software: whoever distributes it must pass them on.",
            )},
            {"title": "Paying for open-source software", "body": (
                "The GPL allows charging for distribution and services. What a seller cannot do is take away the freedoms above. "
                "That is why products built on GPL software sell services, such as tested builds, updates, support and "
                "convenience, and publish the source of what they distribute.",
            )},
            {"title": "How SKAGER works with it", "body": (
                "SKAGER requires a separately installed OpenCPN and adds a new interface and services on top. We publish the "
                "source for each SKAGER release, credit OpenCPN openly, and keep your rights under the GPL intact.",
            )},
        ),
    },
)

FAQS = (
    {"name": "Buying SKAGER", "items": (
        {"question": "What does SKAGER cost?",
         "answer": "Public Beta Access is a one-time payment of USD 20. SKAGER itself needs no subscription. "
                   "SKAGER Cloud, for online services, is optional at USD 2 a month.",
         "link": ("pricing", "See what is included")},
        {"question": "How do I get a SKAGER account?",
         "answer": "Your account is created when you buy SKAGER. After paying, you confirm your email with a code and "
                   "choose a password. Then you go straight to your downloads.",
         "link": ("checkout", "Get SKAGER")},
        {"question": "How do I log in?",
         "answer": "Enter your email and password, then the one-time code we email you. Forgot your password? Choose "
                   "“Forgot your password?” on the log-in page and set a new one with an emailed code."},
        {"question": "Can I add the SKAGER GPS module?",
         "answer": "Yes. Add it at checkout, or buy it later from the shop in your account. We ship within the EU/EEA, "
                   "the UK, Norway and Switzerland."},
        {"question": "What is SKAGER Cloud?",
         "answer": "An optional subscription for USD 2 a month: SKAGER installs new versions for you, and your boat "
                   "comes online with boat data, an online dashboard and more as they launch. Link SKAGER under "
                   "Settings, System, SKAGER Cloud. Start or cancel it any time from your account."},
        {"question": "What do I get for the payment?",
         "answer": "Access to the official beta downloads, including every update, your customer account, the support library, "
                   "support, bug reports and feature requests, and release information as the beta develops."},
        {"question": "Can I get a refund?",
         "answer": "Yes. If SKAGER is not for you, choose Refund on the Orders page of your account within 14 days of "
                   "purchase. No questions asked. A refund closes your account.",
         "link": ("refunds", "Refund policy")},
        {"question": "Are charts included?",
         "answer": "No. SKAGER uses the charts you already have in OpenCPN. Buy and update charts from your chart provider.",
         "link": ("article", "Which charts work with OpenCPN?", "charts-for-opencpn")},
    )},
    {"name": "Compatibility", "items": (
        {"question": "What do I need to run SKAGER?",
         "answer": "A 64-bit Windows PC and OpenCPN 5.12.4 for Windows, installed separately. A touch display of at least "
                   "1280 × 800 is recommended at the helm.",
         "link": ("installation", "Requirements and installation")},
        {"question": "Does SKAGER work on Mac, Linux or tablets?",
         "answer": "SKAGER is available for Windows today. Versions for Linux, Raspberry Pi and macOS are coming, and "
                   "appear in your account as they launch."},
        {"question": "Which instruments can I connect?",
         "answer": "Anything OpenCPN can read: NMEA 0183, NMEA 2000 through a gateway, and Signal K. SKAGER uses the "
                   "connections you set up in OpenCPN.",
         "link": ("article", "NMEA 0183, NMEA 2000 and Signal K explained", "nmea-0183-2000-signal-k")},
        {"question": "Will my routes, waypoints and charts carry over?",
         "answer": "Yes. They stay in your OpenCPN profile, and SKAGER works with them directly. Nothing is imported or copied."},
        {"question": "Does SKAGER need an internet connection?",
         "answer": "Not for navigating. Charts, instruments and AIS work offline. Downloads, updates, forecast wind and "
                   "Online AIS need a connection."},
    )},
    {"name": "Using SKAGER", "items": (
        {"question": "Can I go back to the normal OpenCPN interface?",
         "answer": "Any time. Legacy opens the classic OpenCPN interface with the same data, and Safe Mode helps if something "
                   "will not start.",
         "link": ("article", "Use Legacy and Safe Mode", "legacy-safe-recovery")},
        {"question": "Does SKAGER control my autopilot?",
         "answer": "Only if you want it to. Control is off until you enable and confirm it, and then works with supported "
                   "pilots: Standby, Auto and heading steps. SKAGER's advice features never steer.",
         "link": ("article", "Connecting an autopilot to your chart plotter", "autopilot-and-chartplotter")},
        {"question": "Where does the forecast wind come from?",
         "answer": "From an online forecast service, labelled with its source and time. It is shown separately from the wind "
                   "your own instruments measure.",
         "link": ("article", "Reading wind forecasts on your chart", "grib-wind-forecasts")},
        {"question": "Does it work at night?",
         "answer": "Yes. Day, dusk and night modes change the whole interface, not just the chart, to protect your night vision.",
         "link": ("article", "Night vision at the helm", "night-vision-display")},
    )},
    {"name": "Safety", "items": (
        {"question": "Can I rely on SKAGER as my only means of navigation?",
         "answer": "No navigation system should be your only one. Keep an independent means of navigation, current charts and "
                   "a proper lookout, as good seamanship requires.",
         "link": ("beta_terms", "Beta terms and safety")},
        {"question": "What happens if a sensor stops working?",
         "answer": "SKAGER shows the value as stale or unavailable, with its source and age. It never shows missing data as "
                   "zero or as live.",
         "link": ("article", "Read instruments and data quality", "sensors-and-stale-data")},
    )},
    {"name": "Privacy & open source", "items": (
        {"question": "Does SKAGER send my position anywhere?",
         "answer": "Your navigation data stays on your PC. Optional online services receive only what they need: the forecast "
                   "service receives the area you ask about, and Online AIS the area you select. We never use positions for "
                   "analytics or marketing.",
         "link": ("privacy", "Privacy policy")},
        {"question": "Is SKAGER open source?",
         "answer": "SKAGER builds on OpenCPN and includes open-source components under the GPL and other licences. We publish "
                   "the source for every release, and your payment never limits your open-source rights.",
         "link": ("open_source", "Open source and licences")},
    )},
)
