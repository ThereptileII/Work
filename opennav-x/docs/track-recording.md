# Sailing tracks in SKAGER

Work item: [SCRUM-377](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-377).
Implementation is source-only. The user explicitly requested no builds; the
batch also retains its no-test restriction. No compilation, patch application,
runtime, Windows or boat acceptance is claimed.

## User flow

The compact **Record** button beside the route summary opens **Track recording**.
**Start recording** starts the native OpenCPN recorder; the header then shows a
steady recording indicator. Tap the indicator and **Stop recording** to stop.
Closing the drawer does not stop recording. The drawer reports the current
recorder, position availability, start time, elapsed time and recorded distance
where available. Commands read back the actual recorder state.

**Saved tracks** opens the navigation library, alongside routes and waypoints.
The current recording is first, followed by saved tracks in descending date
order within the copied catalog. Existing OpenCPN tracks are included. The list is populated when opened
or explicitly refreshed, not rebuilt on every sensor tick. Detail offers
fit-to-chart, per-track visibility and one inline name editor; Save/Cancel appear
only while editing. An imported-layer track or a track being edited in the
native manager is protected. Advanced export, deletion and conversion remain in
the existing OpenCPN manager.

**Chart presentation → Tracks** is a separate display filter. It hides all
own-ship track objects, including tracks within imported layers, without changing
their individual visibility flags or their layer's routes/waypoints. Turning it
back on restores those individual choices. Recording continues while hidden and
its drawer explains this. Software and OpenGL drawing, selection and rollover
share the filter. Legacy/Safe ignore the SKAGER master filter.

Sailing tracks are distinct from autopilot **TRACK**, AIS trails and diagnostic
instrument recordings. No new vessel-control output or cloud upload is added.

## Ownership and validity

OpenCPN retains the recorder, points, persistence, distance calculations and daily
tracking settings. `application/TrackRecording.h` contains owned values only.
Application-thread callbacks resolve a GUID again before using native objects;
no route/track pointer leaves the integration boundary. Commands bind metadata
revisions, so stale selections cannot overwrite intervening changes. Point
appends do not by themselves invalidate a name-edit draft.

Starting requires a fresh, valid selected live position consistent with OpenCPN.
The narrow sampling hook withholds stale/unavailable position samples in SKAGER,
including the forced final point. Recording can remain enabled while GPS is
unavailable; this is visibly different from recording fresh positions. Recovery
uses the existing native sampler. The line remains OpenCPN's interpolation
between recorded fixes, not proof of a continuously observed sailed path or a
new gap-segmentation algorithm. Diagnostic replay cannot mutate real tracks.

Points are persisted incrementally by OpenCPN. There is no separate unsaved-trip
database. A very short recording may be discarded by OpenCPN, and daily tracking
may merge into an existing track. Stop therefore does not promise a new row or
an unchanged GUID. Restart carryover remains an upstream preference, not a
promise to resume the identical recording object. Storage failures must remain
explicit; an in-memory state change is not proof of durable saving.
The existing native track-replacement transaction now checks reported write and
commit failures and rolls back instead of accepting a partial point replacement.
Daily merging saves the destination before deleting the source, and a failed
daily recorder restart remains pending rather than dereferencing an absent
recorder. These source corrections still require failure-injection verification.

The master visibility preference is stored under
`/OpenNav/TrackLayerVisibleV1`; invalid settings default to showing tracks. Failed
writes restore the previous setting where possible. Track geometry and
individual visibility remain in the normal OpenCPN navigation store.

Catalog copying is limited to 256 unambiguous tracks from a registry of at most
10,000 entries; the Advanced manager remains available for the full library.
Distance uses OpenCPN's own `Track::Length`, with validated coordinates, a
20,000-point per-read budget and a short cache. Larger tracks report distance
unavailable rather than blocking the navigation UI. Fit-to-chart is limited to
200,000 points and declines polar or excessively wide extents that this native
Mercator fitting path cannot handle safely.

## Deferred verification

Source review covers declarations/call sites, source lists, hook ownership,
thread/lifetime boundaries and patch/source inventories. Tests were authored or
updated where relevant but not executed. At the next authorized build, perform
focused Linux/native Windows checks for recording and replay gates, GPS
loss/recovery/final sample, short/stationary recordings, daily merge, carryover,
storage failure/rollback, rename persistence and visibility independence. Check software/OpenGL filtering,
header/drawer interaction and Legacy sharing in the resulting candidate.

Boat delivery remains separate and must resolve the startup issue SCRUM-376.
This source change neither replaces nor qualifies the installed Beta 2.5 build.
