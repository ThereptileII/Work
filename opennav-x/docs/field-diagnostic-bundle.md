# Field diagnostic bundle

Beta adds **Menu → Field diagnostic bundle** (also in Settings; development
shortcut Ctrl+Shift+X). Export is a deliberate local action. Nothing is uploaded.
Review the ZIP before sending it with the failure time, reproduction steps and
screenshots. The optional recording action requires a separate file selection
and consent showing whether navigation capture was enabled.

The bundle is assembled from copied OpenNav state, never a directory walk or a
copy of OpenCPN's full profile. Its fixed entries contain:

* Build, OS/DPI, interface/data mode, plugin versions/initialization and startup
  recovery status. No private profile path.
* Normalized sensor values, validity, observation ages and thresholds; candidate
  priority, selection, cadence and invalid counts. Protocol/PGN/instance remain;
  transport/device identity is replaced with a snapshot-local source alias.
* Numeric energy assumptions, current convention, model/battery configuration
  state and advisory result/reason. No curve filename or device binding.
* Autopilot feedback/command lifecycle and capability state, radar availability.
* Current typed SmartNav events and a bounded journal of the latest 128 health,
  recording, route-state, command and advisory transitions in this process.

Positions are withheld, as are route/waypoint names, AIS identities, raw bus
streams, network addresses, credentials, arbitrary logs and unrelated files.
The journal is normalized diagnostic logging, not high-rate raw NMEA logging.
Crash information is startup-recovery state, not a memory dump. Full local logs
remain available through the diagnostics folder for deliberate manual review.

An explicitly selected `.onxr` recording is decoded with the accepted strict
limits and re-encoded before being included under a fixed archive name. It may
contain device/interface names and optional navigation information. Its original
filename/path is not embedded. No recording is included automatically, even if
capture is active. Bad/truncated/oversized recordings fail before publication.

Reports have fixed names and a 256 KiB bound per report; source/event/metadata
counts are bounded. One optional recording is limited to 8 MiB. wxWidgets writes
the ZIP through a temporary output file; failure discards it and successful
completion atomically commits it. The user selects the destination and confirms
replacement in the file picker. Export never sends control commands, changes
sensor ages, mutates OpenCPN objects or changes live configuration.

Portable tests cover privacy, source/age preservation, bounds, path names,
explicit recording selection and malformed recording rejection. The actual UI
recording smoke now exports/opens the ZIP, checks its CRC/content/privacy and,
on native Windows, separately exercises optional selection/consent and exact
recording equality. Platform acceptance is tracked in status/evidence.
