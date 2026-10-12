"""Owned, plain-text support articles for the closed SKAGER website preview.

Each article has slug/title/category/summary/applies_to and sections. A section
contains a title and either a body string or a tuple of bullet strings. Keep
content plain text so Django's normal autoescaping applies to every value.
Search and exact-slug lookup belong to the marketing views, not this data file.
"""

SUPPORT_VERSION = "Windows beta preview · OpenCPN 5.12.4 · public access closed"

ARTICLES = (
    {
        "slug": "opencpn-prerequisite",
        "title": "Start with the supported OpenCPN installation",
        "category": "Getting started",
        "summary": "Which OpenCPN version to use, and what to do if Setup refuses it.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "The supported combination", "body": "The Windows beta targets a 64-bit Windows PC running the Win32/x86 edition of OpenCPN 5.12.4. Win32/x86 describes the application and plugin interface, even on a 64-bit PC. OpenCPN is installed separately; it is not bundled with SKAGER."},
            {"title": "Check before installing", "bullets": ("Obtain OpenCPN from its official website or the official 5.12.4 release. The Installation page links to both.", "Open OpenCPN and check its About or version information. Confirm 5.12.4, then close it before running SKAGER Setup.", "Keep an independent backup of your OpenCPN profile and charts. A backup inside the application installation folder is not enough.")},
            {"title": "A version label is not the whole check", "body": "Setup also checks the exact supported OpenCPN build. A modified or portable build with the same version label may be unsupported. If Setup refuses the installation, stop. Do not rename executables, replace files by hand or bypass the check. Keep the displayed version and error information for support."},
            {"title": "Current availability", "body": "The public beta is still being qualified. Public downloads are closed. Use the version-specific installation guidance supplied with a qualified release when it becomes available; this preview does not announce a release."},
        ),
    },
    {
        "slug": "install-skager",
        "title": "Prepare for a SKAGER installation",
        "category": "Getting started",
        "summary": "Back up, close navigation windows and check the prerequisite before Setup.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Before Setup", "bullets": ("Confirm the supported OpenCPN 5.12.4 installation first.", "Back up the OpenCPN profile, charts stored elsewhere, and any separately managed plugin information.", "Close OpenCPN, SKAGER, Legacy and Safe Mode windows, including older portable copies.", "Make changes while the boat is safely stopped and you have another means of navigation.")},
            {"title": "When a qualified installer is available", "body": "Use the installer linked from the official version-specific release page. Select the original installed OpenCPN application when asked. Read the compatibility result and recovery details, and keep the backup location shown by Setup. Stop if the installer reports an unsupported build or cannot protect existing data."},
            {"title": "After installation", "body": "Before relying on the system, check your charts, routes, tracks, waypoints, connections and required plugins. Confirm the actual sources and ages of instrument readings. Check the Legacy and Safe Mode recovery instructions for that release. Installation success alone does not establish that your boat's equipment has been commissioned."},
            {"title": "Preview limit", "body": "No public installer is available here. The complete installation, update, repair, rollback and uninstall lifecycle still requires release qualification. These preparation steps are not evidence that the current candidate has passed those checks."},
        ),
    },
    {
        "slug": "updates-repair-uninstall",
        "title": "Updates, repair, rollback and uninstall",
        "category": "Installation & recovery",
        "summary": "Understand the intended recovery options and the checks still pending.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Release status", "body": "The installed updater and the complete protected maintenance lifecycle are still awaiting qualification for the public beta. This page does not offer an update or promise that an installed candidate can update itself."},
            {"title": "Before making changes", "bullets": ("Record the installed SKAGER and OpenCPN versions and read the matching release notes.", "Back up your profile and separately stored charts, then close all navigation windows.", "Use only the official release's documented maintenance options. Do not copy replacement application files over a running installation.")},
            {"title": "What the lifecycle is intended to preserve", "body": "SKAGER is designed to sit alongside the original OpenCPN application. Updates, repair, rollback and uninstall are intended to operate on SKAGER-owned files while retaining the original OpenCPN application and your navigation data. Qualification must verify this behavior for the exact release before public availability."},
            {"title": "If an update or repair fails", "body": "Stop further attempts, retain the displayed error and backup information, and use the release's Legacy or Safe Mode recovery route. Do not delete the OpenCPN profile to make Setup continue. A previous installer or source archive is not automatically a supported rollback package."},
            {"title": "Uninstall", "body": "Follow the instructions accompanying the qualified release. Do not manually remove the shared OpenCPN profile or chart directories. Keep your backup until the original OpenCPN installation and required data have been checked."},
        ),
    },
    {
        "slug": "legacy-safe-recovery",
        "title": "Recover with Legacy or Safe Mode",
        "category": "Installation & recovery",
        "summary": "Use the familiar OpenCPN interface or a reduced startup to investigate problems.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Legacy", "body": "Legacy opens the familiar OpenCPN interface against the same normal navigation profile. It is the route back to standard OpenCPN when the SKAGER interface is unsuitable or needs investigation. Check the release instructions for the supplied launcher; switching interfaces may require a controlled restart."},
            {"title": "Safe Mode", "body": "Safe Mode is a recovery startup intended to avoid the SKAGER interface and services while investigating startup or plugin trouble. It preserves the profile rather than resetting navigation data. Some capabilities are intentionally unavailable during recovery; do not assume every plugin or data connection is active."},
            {"title": "If the application repeatedly fails to start", "bullets": ("Use the release's Safe Mode or Legacy launcher and keep any recovery notice and failure time.", "Do not repeatedly bypass recovery or remove the profile to force a normal startup.", "When the application opens, check charts and navigation data before changing settings. Save a diagnostic bundle if available and review it before sharing.")},
            {"title": "What recovery does not establish", "body": "Startup protection cannot repair every crash, damaged file or hardware fault. Safe Mode is not a substitute for an independent navigation method. Recovery and maintenance must still pass the exact release's Windows and boat checks; this preview does not qualify the pending release."},
        ),
    },
    {
        "slug": "charts-and-plugins",
        "title": "Keep charts and plugins ready",
        "category": "Charts & equipment",
        "summary": "Use your own licensed charts and check the plugins your navigation depends on.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Charts are separate", "body": "SKAGER does not bundle licensed private charts or chart subscriptions. Obtain suitable charts from their providers and manage their licenses and updates separately. A chart shown in a preview is not a chart entitlement or evidence of coverage for your voyage."},
            {"title": "If charts are missing", "bullets": ("Open the familiar OpenCPN chart settings in Legacy and check the configured chart locations and provider requirements.", "Check that the chart files are accessible and any required chart plugin or license is available.", "Keep a backup before rebuilding chart information or changing directories. Do not delete chart files to troubleshoot a display issue.")},
            {"title": "Plugins and versions", "body": "The beta uses the OpenCPN 5.12.4 Win32/x86 plugin interface on a 64-bit Windows PC. A plugin intended for a different application architecture or version is not automatically compatible. Check required plugins individually; compatibility with one plugin does not qualify every combination."},
            {"title": "Before going afloat", "body": "Check coverage, chart age, depth reference and the displayed scale using the chart provider's guidance. If a required chart or plugin is unavailable, resolve it before relying on the system. An empty or incomplete chart must never be interpreted as clear water."},
        ),
    },
    {
        "slug": "onboard-online-ais",
        "title": "Understand onboard AIS and Online AIS",
        "category": "Charts & equipment",
        "summary": "Receiver data takes priority; AISStream adds optional Internet traffic information.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Start with the onboard receiver", "body": "Onboard AIS continues through OpenCPN's existing connections and calculations. Check receiver health separately from the combined traffic display. Seeing an Internet target does not prove that the onboard receiver is connected."},
            {"title": "Online AIS is supplemental", "body": "Online AIS uses AISStream with your own API key and an Internet connection. Enter the key only in the application's dedicated Online AIS settings; never put it in a screenshot or support report. Service access and coverage are separate from SKAGER beta access. No key is requested by this website preview."},
            {"title": "Read origin and age", "body": "Onboard data takes priority when both feeds report the same vessel. A newer Internet report must not hide a stale onboard report. Online reports can be delayed, incomplete or absent. A connected subscription does not mean that vessel positions have been received, and no reported targets does not mean no traffic."},
            {"title": "Safety boundary", "body": "Supplemental Internet targets do not supply OpenCPN collision alarms or CPA/TCPA calculations. CPA and TCPA mean closest approach and the time to it. Online AIS is not a collision-avoidance system or a replacement for onboard AIS, radar, lookout or safe seamanship."},
            {"title": "If the online list is empty", "body": "Check the application's service status, whether positions have actually arrived, the selected reception area and your Internet connection. Keep your key private. Do not change the onboard receiver setup just to make online targets appear."},
        ),
    },
    {
        "slug": "sensors-and-stale-data",
        "title": "Read instruments and data quality",
        "category": "Charts & equipment",
        "summary": "Check the source, age and meaning of a reading before treating it as current.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Unavailable is not zero", "body": "Missing, stale, invalid, estimated and uncertain readings have different meanings. Zero is a real measurement; unavailable data must not be displayed as zero. A value retained on screen is not necessarily a fresh observation."},
            {"title": "Check the actual source", "body": "NMEA 0183, NMEA 2000 and Signal K data depend on the equipment, gateway and OpenCPN connections you have configured. SKAGER does not make an absent sensor appear. Check source identity, units, observation age and connection health in Settings or Diagnostics before changing equipment configuration."},
            {"title": "When a source is pinned", "body": "If you explicitly select one instrument source, a missing or stale reading from it stays missing or stale. It must not silently become a reading from another device. Automatic source selection can choose a fresh alternative according to its configured priority; inspect the selected source when investigating a change."},
            {"title": "Use the correct physical meaning", "bullets": ("Depth below a transducer is not automatically clearance below the keel.", "Apparent wind and true wind are different measurements.", "Battery power and motor power are different; confirm the device and current direction before relying on energy estimates.", "An unavailable reading does not establish that the physical system is safe or stopped.")},
        ),
    },
    {
        "slug": "energy-and-smartnav",
        "title": "Treat energy and SmartNav as advice",
        "category": "Navigation advice",
        "summary": "Estimates need fresh inputs and boat-specific assumptions; SmartNav never steers.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Configure the boat, then check the readings", "body": "Energy estimates require explicitly configured usable battery capacity, reserve and battery identity, plus suitable fresh measurements. Confirm current direction and that measured battery discharge includes the boat's other electrical loads. Motor power alone is not whole-boat consumption."},
            {"title": "Understand the estimate", "body": "Range and arrival charge are advisory estimates based on their displayed assumptions. Constant speed and consumption cannot predict every change in wind, current, load, battery condition or temperature. Check source quality and leave an appropriate reserve; an estimate is not a promise of arrival."},
            {"title": "When an estimate disappears", "body": "Missing or stale inputs, an absent route or changed route information may make dependent estimates unavailable. This is a reason to inspect the data, not a reason to substitute guessed capacity, power or position. Unavailable arrival charge must not be read as a zero-percent arrival."},
            {"title": "SmartNav and physical control", "body": "SmartNav provides advice and never steers. Missing hazard information does not establish a safe route. Unqualified autopilot, propulsion, switching and other hardware outputs remain unavailable or off by default. Any separately qualified manual control requires its own commissioning and fresh physical feedback; a sent command alone is not proof of the equipment's state."},
        ),
    },
    {
        "slug": "diagnostics-and-privacy",
        "title": "Prepare a useful, private problem report",
        "category": "Troubleshooting",
        "summary": "Record versions and symptoms, review diagnostics and leave secrets out.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Start with a small report", "bullets": ("Record the SKAGER version, OpenCPN version, Windows version and approximate failure time with time zone.", "Describe what you expected, what happened and the few steps that reproduce it. Mention whether it also happens in Legacy or Safe Mode.", "Include the displayed error and relevant data-quality status. Remove private chart names, positions, account details and keys from screenshots.")},
            {"title": "Field diagnostic bundle", "body": "Where available in your build, choose Field diagnostic bundle from the menu or Settings. This creates a local ZIP; it does not upload it. Review the contents before choosing to share them. The standard bundle is designed to withhold positions, route names, AIS identities, network addresses and credentials, while retaining useful version, health and data-quality information."},
            {"title": "Recordings and logs need separate care", "body": "An optional recording is included only after deliberate selection and consent. It can contain device names and navigation information if navigation capture was enabled. Full logs and recordings may reveal more than the standard bundle. Do not send a complete profile, raw marine-network capture, API key, password or payment information as a routine report."},
            {"title": "Support preview", "body": "This library is available for reading. Ticket submission, uploads and support contact details are not enabled in this preview. No report is sent from these pages. A verified support route and the final privacy notice must be published before those services open."},
        ),
    },
    {
        "slug": "licensing-and-source",
        "title": "Find licenses and corresponding source",
        "category": "Open source",
        "summary": "OpenCPN credit, your open-source rights and source tied to each release.",
        "applies_to": SUPPORT_VERSION,
        "sections": (
            {"title": "Built with OpenCPN", "body": "SKAGER builds on OpenCPN and credits its contributors. OpenCPN is a separately installed prerequisite. SKAGER is an independent product; this attribution does not imply endorsement by the OpenCPN project."},
            {"title": "Your license rights remain", "body": "Applicable GPL and other open-source license rights remain in place. Paid beta access, account terms and support services must not remove the rights granted by those licenses. Chart-provider licenses are separate and do not become open-source licenses through use with SKAGER."},
            {"title": "Source for the exact version", "body": "Qualified public releases will have version-specific links to corresponding source, applicable license texts and third-party notices. Source must match the released software; an upstream project link alone does not provide SKAGER's corresponding source. Those source and notice links will remain independent of paid account access."},
            {"title": "Current availability", "body": "The Open Source page links to the official OpenCPN project and its pinned baseline. No SKAGER public release or downloadable SKAGER source package is announced by this preview. Exact-release source mapping and compliance review remain release gates."},
        ),
    },
)
