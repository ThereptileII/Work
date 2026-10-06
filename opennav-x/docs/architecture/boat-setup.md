# Boat Setup & Sensor Check (SCRUM-28)

The native six-step flow follows the supplied prototype's vessel, display,
sources, energy, helm control and summary sequence. It uses the current XNav
buttons, selectors and palette. Display edits cover existing interface scale
and chart layout; OpenCPN unit and light-mode preferences are preserved.

## Startup and recovery contract

Before the startup recovery journal is created, integration snapshots whether
this is a new SKAGER profile: no `/OpenNav` group and no `opennav-startup.state`
file. Existing profiles, including older generations with no vessel settings,
are adopted without an automatic wizard. Portable preview is not an install.
The versioned `/OpenNav/BoatSetupV1` entry records pending or complete (and accepts
the earlier adopted-existing marker). Existing profile inspection performs no
write or flush; the existing profile/journal is sufficient to suppress setup.
Unknown records do not trigger a reset. A pending flow resumes
after Later or restart; product version changes are irrelevant to this policy.

Automatic display occurs only after OpenCPN deferred initialization in the live
XNav shell. The window is modeless: the application event loop, navigation
caution handling and 30-second startup-health checkpoint remain independent.
Later and window close discard all draft edits. Preferences → Vessel → Run boat
setup explicitly resets only setup progress and starts from existing settings.
Demo and Replay cannot save or automatically start live setup.

## Data and write boundary

`application/BoatSetup` owns validation and read-only summary policy. Sensor
rows use actual selected navigation and registry observations, their source
identities and the existing freshness assessment; no readings are manufactured.
The Sources page's Check again button refreshes the snapshot; the final summary
checks again. Demo/Replay observations are excluded from this live inventory.
Missing, stale, invalid, estimated and uncertain data remain distinguishable.

The final explicit Save writes vessel name, draft, chart safety depth, capacity
and reserve assumptions, display preferences and completion in one profile
flush. On failure all touched entries are restored and completion remains
pending. Unknown assumptions may remain blank; blank chart safety depth retains
the existing OpenCPN value. The actual chart parameters refresh after successful
persistence. Concurrent profile changes reject a stale modeless draft rather
than overwrite another settings view.

Setup owns no protocol, source selection, command, enablement or transport API.
Fresh autopilot permission stays OFF. Existing source mappings, calibration,
battery identity, current sign and pilot permission/identity are preserved.
An existing pilot permission is described explicitly rather than falsely
reported OFF; setup never grants runtime enablement. The installed product's
hardware output policy and all existing identity/feedback gates remain intact.
Configured capacity/reserve are assumptions, not battery observations.
Completing setup is not passage-readiness certification.

## Verification boundary

Focused tests exercise new/old/interrupted/unknown progress policy, missing and
stale observations, invalid safety depth, persisted completion, explicit reset,
failed-flush rollback and preservation of unrelated settings. The native dialog
fixture exercises six-step navigation, summary before save, failed-save retry,
Later and modeless operation with offline inputs. Component compilation and
these focused tests do not replace integrated OpenCPN startup, native Windows
MSVC, usable controls at supported DPI/resolutions, or target boat gates.
