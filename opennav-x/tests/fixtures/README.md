# Test Fixtures

Store deterministic, non-secret test data here.

Recommended fixtures:
- NMEA 0183 navigation stream
- normalized NMEA 2000-derived vessel state samples
- wind/depth/rudder changes
- propulsion/battery traces
- AIS crossing encounter
- route with several course changes
- stale sensor sequence
- anchor swing sequence

Fixtures used in automated tests must have documented expected outputs.

`mode-persistence.gpx` contains one synthetic standalone mark, a two-point route
and a two-point recorded track. Every object is visibly named SIMULATED. Fixed
GUIDs, positions and UTC timestamps are compared after OpenCPN imports the file
into `navobj.db` and through the UI mode cycle. The harness reads the SQLite
store in read-only mode and checks integrity and foreign-key consistency.

The same disposable profile contains one **disabled**, input-only loopback
connection and AIS CPA-warning settings (enabled, 0.75 NM). This tests settings
persistence; it does not claim to simulate an actual AIS encounter. No fixture
is loaded into a real navigation profile or transmitted to a device.

## Plugin preference coverage

The shared-profile fixture enables the bundled Dashboard plugin and checks its
saved enabled preference after every transition, including Safe Mode and the
following normal startup. It has no instrument panes configured, so the plugin
can initialize without covering the canonical screenshot. Safe Mode must suppress initialization for its session
without persisting a disabled normal-mode preference. Other third-party plugins
still need representative Windows compatibility tests.
