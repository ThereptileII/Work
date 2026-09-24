# Developer Preview 0.1 data, UI and portable boundary

The preceding remaining-route contract passed Linux and native Windows at
`954b4505`. Preview acceptance is recorded in [status](status.md); implementation
alone is not acceptance. This milestone ends at a downloadable portable preview.

## Owned Vessel Data

`VesselState` carries optional numeric `Sample` and textual `TextSample` values,
source, monotonic observation time and validity. `Assess`/`AssessText` compute
live/aging/stale/unavailable quality without renewing observations. Existing
2-second aging and 5-second stale thresholds remain unchanged. Navigation now
also carries an immutable owned `RouteProgressSnapshot`, STW and heading fields.
Wind, depth/temperature, rudder, motor electrical/shaft power, RPM/temperature,
gear/regeneration, battery voltage/current/SOC/SOH/usable energy/whole-pack net
power, tank levels and connectivity fields remain separate source contracts.
Missing values stay absent; a field's presence does not claim live acquisition.

Only selected OpenCPN latitude/longitude/SOG/COG and the accepted route observer
currently provide live data. Unproven heading and unconnected hardware stay
unavailable. No desktop code decodes Nissan Leaf CAN identifiers. The boat-side
ESP32 CAN to NMEA 2000 bridge is the intended future hardware boundary.

## Explicit deterministic Demo source

`DemoSource` is independent of OpenCPN and hardware. For a scenario and elapsed
whole second, it returns the same values and source timestamps relative to the
scenario start. It moves a synthetic coastal trip at 60x time, with wind, depth,
rudder, propulsion, battery and three synthetic waypoints. Waypoint transition
observations withhold route distance. Route completion publishes NoActiveRoute,
not a valid zero arrival. Pausing and the stale scenario freeze observations;
reading or repainting never makes them fresh. Eight selectable scenarios cover
cruising, stale, unavailable, inactive route, ending, low SOC, high power and
insufficient energy. DEMO source strings, route identities and visible banners
separate this source from live data. Demo never fills gaps in live inputs.

The synthetic route uses the owned route contract but does **not** claim to be
OpenCPN progress. Its distance is a labelled fixture timeline. The real route
observer and pinned geometry remain unchanged. Demo does not activate a route,
inject ownship navigation, send NMEA, enable a connection or drive devices.
The real OpenCPN chart canvas remains pannable/zoomable; synthetic GPS is shown
as data and does not move OpenCPN's ownship symbol. Initial Demo view is the
Swedish coast. Included coastline overview is not a nautical chart collection.

## Advisory energy presentation

`VesselEnergy` maps owned snapshots into the existing tested energy model.
Each UI refresh obtains the current live route publication and reassesses its
position/progress age. Arrival uses valid route NM, SOC, SOG and **whole-pack
net discharge**, never motor power as a silent substitute. Missing/stale/invalid
inputs withhold the dependent estimate. Shortfall is explicit, not negative SOC
or a fabricated successful zero arrival. Range remains possible without a route.

Demo explicitly configures 48 kWh usable energy, 15% reserve and 0.4 kW hotel
load. The model assumes constant speed/net discharge and linear SOC. These are
fixture assumptions, not vessel defaults. Live capacity/reserve remain
unconfigured and predictions unavailable until a later calibrated data slice.
No prediction is a control command. The route page's passage time is an advisory
current-SOG estimate, not a weather/current-aware ETA.

## Interface and diagnostics

Custom XNav Navigation, Route, Energy and scrollable System pages share design
tokens/day-dusk-night controls and the real OpenCPN chart. No final route editor,
AIS redesign or device control is implied. System exposes native OS/DPI, build
commit/compiler/date/CI, pinned OpenCPN, profile, route provenance and individual
source/validity/age. Atomic `logs/opennav-diagnostics.json` retains full values,
monotonic timestamps and validity; it is diagnostic data, not a public remote
API. Explicit non-packaged test profiles write it inside the profile.

Content pages participate in the existing AUI layout as an alternate center pane.
The integration supplies the pinned chart pane identities; the shell temporarily
hides navigation panes and restores their previous visibility on return or before
upstream saves its perspective. Chart canvases keep their original ownership and
parent. Native tests verify page geometry and sibling visibility after resize,
as well as return to the chart. An earlier unmanaged overlay failed Windows
visual review despite passing data assertions; that candidate is not accepted.

Legacy and Safe retain upstream UI. System permits controlled XNav restart,
Legacy switch and Safe startup. Demo is intentionally not restored across a
mode switch; `Run-XNav-Demo.cmd` explicitly requests it. All packaged modes use
the same private profile and existing restart/persistence mechanism.

## Portable isolation and packaging

`app/OPENNAV_PORTABLE_PREVIEW` identifies an intact package. Before upstream
profile initialization, the integration derives sibling `profile` and `logs`
from the executable, rejects external `--configdir`, canonical path escape and
remote-command startup, then forces upstream portable mode. The process working directory is the private
profile, matching OpenCPN portable resource normalization; this also applies to
direct launch and restart. First launch fits the window to the available desktop.
Direct executable
launch is also isolated. Do not remove this marker or rearrange app resources.
Three small guarded upstream changes preserve that mode after command parsing,
force independent startup instead of forwarding to another installation, and
suppress LAN REST/mDNS discovery in the preview. No production installation,
registry, normal chart configuration or system-wide plugin is changed.

Launchers use quoted relative paths and require a writable extraction folder,
not administrator rights. The ZIP bundles installed resources/plugins and
app-local x86 MSVC runtime DLLs for the approved Win32 OpenCPN ABI on x64 Windows.
PE import auditing and an extracted-package smoke test with a restricted PATH
check runtime closure. File hashes and ZIP SHA-256 accompany build provenance;
corresponding source is published separately. See [human test guide](preview/TEST_ME_FIRST.md)
and [limitations](preview/KNOWN_LIMITATIONS.md).

OpenCPN's Windows portable plugin loader uses `profile/plugins`, separately from
the platform's `app/plugins` path. The package supplies its bundled DLLs and data
in both locations; the portable loader uses the private-profile copies. The test
requires actual Dashboard initialization/clean unloading across normal launches
and inactivity in Safe Mode, not just retention of an enabled configuration flag.

## Gates

Ten portable CTest contracts include deterministic Demo/energy, portable profile
and basemap-resource tests, alongside existing energy, route, freshness and mode tests.
The unchanged integrated suite contains 67 Linux / 57 Windows tests. Existing
mode, selected NMEA, normal-timer route and repeated restart gates continue.
`smoke-preview.py` exercises every scenario and page, moving values, withheld
predictions and shared-profile mode switching. Native Windows additionally tests
all launchers, direct EXE isolation, refused external profile, unchanged normal
profile canary, unchanged existing common/user OpenCPN files, package hashes
and DLL closure. The test harness marks only its disposable extracted copy for
fixture seeding; the shipped profile contains no test marker. It captures ten native 1280x800
images for review. 125/150% DPI, touch, target PC and at-sea validation remain
explicit manual limitations where the hosted desktop cannot provide them.

The official artifact is published only after every required workflow job
passes. Human/native screenshot review and evidence records are still required
before declaring the exact package accepted. No production installer follows
automatically.

The resource contract covers portable basemap defaults and migration.
An empty upstream shapefile default must not round-trip into an
explicit empty profile-directory lookup. The integration resolves the existing
bundled data before canvas construction in all preview modes, preserves custom
paths, and recognizes the old preview's dot-directory artifact. Normal OpenCPN
chart configuration and rendering remain authoritative. The preview smoke test
now checks actual coastline pixels after Legacy return and Safe restart, with
an additional Windows capture after migrating the old setting. Data/process
assertions alone do not establish that the chart has rendered.
