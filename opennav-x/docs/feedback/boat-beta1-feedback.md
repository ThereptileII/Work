# Boat Beta 1 feedback — 2026-09-26

The complete user report was read through the existing `ssh boat` alias before
changing product code. Desktop enumeration identified `Bugs XNAV-beta.txt`;
the filename was not assumed. Only actionable project feedback is recorded
here. Personal account paths, unrelated Desktop files and chart contents are
excluded. The user has accepted the concept/core architecture and now requests
Beta 2 design and real-installation refinement.

| ID | Report, faithfully normalized | Classification | Required disposition / verification |
| --- | --- | --- | --- |
| B1-01 | Changing the light setting never changes the button label; it always says Light. | Interaction/flow; Data presentation | Show current Day/Dusk/Night and an understandable change action. Verify every palette on the actual display. |
| B1-02 | The boat does not use Signal K; remove it. | Interaction/flow; Hardware/data source | Remove Signal K from normal XNav choices/presentation. Preserve upstream Legacy compatibility; inspect configuration before changing any real connection. |
| B1-03 | Adding a GPS sensor yields no data until XNav is restarted; baud rate may be involved. Sensor UI is hard to understand. | Functional bug; Hardware/data source; Interaction/flow | Reproduce live connection changes, trace normal OpenCPN selected-navigation/subscription lifecycle, explain port/baud/connection health without assuming baud is the cause. |
| B1-04 | Adding sensors makes the chart disappear; restart restores it. | Functional bug; OpenCPN integration | Inspect settings/canvas rebuild and layout lifecycle; verify real chart content immediately after settings apply and across modes. |
| B1-05 | Anchor-watch chart label has too many decimal places. | Data presentation; Visual design | Format the watch label in useful whole meters without changing upstream radius semantics. |
| B1-06 | Removing anchor watch leaves the anchor waypoint behind. | Functional bug; Interaction/flow | Distinguish an XNav-created watch mark from a user's existing waypoint; remove only a safely owned isolated watch mark. Preserve unrelated/shared navigation objects. |
| B1-07 | Anchor waypoint should use an anchor symbol. | Visual design; OpenCPN integration | Reuse an existing OpenCPN anchor icon and verify chart rendering. |
| B1-08 | Center-on-boat control was hard to find, although it exists. | Interaction/flow; Visual design | Clear ownship/center icon and accessible name; verify follow and recenter behavior. |
| B1-09 | SeaTalk translation belongs to the hardware aboard; XNav pilot commands should use NMEA 2000. Inspect AutoTrack as a working reference. | Hardware/data source; OpenCPN integration | Inspect actual AutoTrack and pinned translator before protocol decisions. Keep vendor details behind adapter; distinguish standard PGN envelopes from manufacturer-specific parameters. No physical commands authorized. |
| B1-10 | No AIS targets found, either Demo or live. | Functional bug; Hardware/data source; Data presentation | Inspect OpenCPN target acquisition/filter/selection; verify real received targets when available and deterministic isolated tests. Absence of reception must be explicit. Demo is removed from the product independently. |

## Additional explicit Beta 2 requirements

- Fix the Diagnostics Alpha caption, 150% System/Alerts overlap and rail values
  pushed out by alerts; do not defer these Beta 1 defects again.
- Installed product contains no Demo controls, fixtures or synthetic fallback.
  Deterministic generators remain only in tests or explicit test builds.
- Design reference, not current Beta rendering, governs every primary screen.
- Use actual supported stock OpenCPN, shared real profile/charts and read-only
  marine input on the boat PC. Check exact architecture/hash before deployment.
- Remove older XNav versions after inventory and preservation of user data and
  a known-good recovery path. Do not delete unrelated files or stock OpenCPN.
- No physical autopilot, propulsion, throttle, radar-transmit or switching
  commands. Preserve SSH, Tailscale and RustDesk throughout development.

## Acceptance tracking

No item is marked accepted on the boat yet. Current implementation/evidence:

- B1-01/02/08: current palette caption, simplified Sensors and explicit Center
  are implemented; native/boat interaction review is pending.
- B1-03/04: isolated input-only TCP added after startup acquires GPS and AIS
  without restarting. Normal settings-reconfiguration return retains measured
  coastline pixels. Sensor connection flow and real serial baud still need
  boat validation; the stock/portable GPS baud settings differ.
- B1-05/06/07: integration tests exercise whole-metre anchor label, existing
  anchor icon and removal only of a safely owned temporary watch mark.
- B1-09: desktop remains high-level NMEA 2000; the standard PGN126208 envelope
  carries reviewed manufacturer-specific pilot parameters. SeaTalk1 decoding
  stays in the physical translator. No command output was performed.
- B1-10: actual AIVDM input exposed a timezone bug: the bridge interpreted the
  pinned decoder's shifted UTC-style ticks as Unix timestamps, aging fresh
  UTC+2 targets by two hours. The fix preserves elapsed upstream age and passes
  native-wx/unit and real loopback tests; actual boat reception remains open.

Every closure must identify the exact native candidate and actual boat evidence.
No hardware result is inferred from simulator tests or earlier acceptance.
