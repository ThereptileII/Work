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

The prototype fixture is a separate, non-installed `alert_drawer_test` binary.
The normal product has no synthetic alert injection interface. Deterministic
fault episodes are tested offline and never aboard.

Interaction diagnostics expose the control's readable accessibility name
separately from its short painted caption. This lets tests identify the exact
condition without making assumptions about severity ordering or putting
internal identifiers into normal UI. No text-field values are collected.
