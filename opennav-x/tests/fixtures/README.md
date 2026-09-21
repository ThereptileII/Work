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
