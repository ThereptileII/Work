# Read-only alert presentation and human acknowledgement

`AlertCenter` remains the authority for XNav presentation episodes. It consumes
owned Vessel Data, the original onboard AIS snapshot, OpenCPN anchor-watch
state, validated energy predictions and manual-pilot feedback. Supplemental
Online AIS is not injected into safety alarms.

The native notification drawer copies the current vector. The view never owns
OpenCPN route, waypoint, AIS decoder or driver pointers. It does not refresh
sensor timestamps. It rebuilds only when its displayed episode/acknowledgement,
replay state or theme changes, retaining scroll position across updates.

A manual acknowledgement is `(id, episode)` and is delivered unchanged even if
the display updates before queued delivery. `AlertCenter::Acknowledge` rejects
an absent/replaced episode. Repeated queued taps are suppressed. Acknowledging
only records that the human has seen the condition: it does not remove it,
resolve the fault, acknowledge upstream equipment or disable an alarm.
Dismissal closes the sheet only. Replay interactions cannot affect live state.

Integration source inspection: `src/integration/NavigationObjects.cpp` copies
`AisTargetData::n_alert_state` and guarded upstream CPA/TCPA. Pinned OpenCPN
`model/src/ais_decoder.cpp` owns those calculations and alarm semantics. Existing
anchor-watch observations and adapter state are unchanged. No upstream hook or
physical control boundary changes are required by this UI migration.

SCRUM-374 adds a read-only alarm-settings overview, distinct from active
notifications. `NavigationActions::alarm_preferences` copies current OpenCPN
AIS flags and finite, nonnegative optional threshold values on the application
thread. Off-thread observation is unavailable. The view owns no upstream
pointers and reading it neither updates navigation nor changes preferences.
The effective moored suppression includes OpenCPN's hide-moored setting because
the pinned decoder uses both flags for alarm suppression.

Explicit AIS and sound-settings actions open the pinned OpenCPN options pages
(Ships/AIS Targets 3/1 and User Interface/Sounds 4/1). The previous initial-page
selection is restored after opening, and the normal upstream dialog owns edits
and persistence. Replay guards cover both actions; the native overview also
rechecks mode before deferred mutation entry. The bell drawer's overview link
remains usable during replay for explanation only. No upstream patch or new
alarm configuration store is needed. See [available settings](operational-alerts.md#finding-and-adjusting-alarms).

The 2026-10-17 source-only increment has no new execution/visual evidence.
Existing acceptance evidence must not be attributed to these changes.

The prototype fixture is a separate, non-installed `alert_drawer_test` binary.
The normal product has no synthetic alert injection interface. Deterministic
fault episodes are tested offline and never aboard.

Interaction diagnostics expose the control's readable accessibility name
separately from its short painted caption. This lets tests identify the exact
condition without making assumptions about severity ordering or putting
internal identifiers into normal UI. No text-field values are collected.
