# Bounded normalized recording and replay — Beta design

Status: implemented in the current Beta working tree; native acceptance pending. This extends
Vessel Data; it does not replace OpenCPN's transport or navigation processing.

## Capture and privacy boundary

Recording is an explicit action, OFF on every application start. Capture copied
normalized observations/state at a bounded cadence, preserving each original
observation age, validity, source and device identity. A unchanged observation
must not become fresh just because another frame is captured. Include the
configuration assumptions required to interpret an energy estimate.

Default recordings omit positions, route names and identifying AIS details.
An explicit navigation-data option may include these for route/SmartNav fault
reproduction, with clear notice before capture. No unrelated files, passwords,
connection credentials, full user configuration or raw EV-CAN frames. High-rate
raw logging is a separate explicitly enabled diagnostic facility, never a side
effect of normalized recording.

Use a versioned, strict, bounded format: finite/domain-checked numbers, known
record types, limited strings/items/frames, ordered session-relative timestamps,
no filenames supplied by recording contents. Missing values are explicit;
invalid and stale are not zero. Readers reject malformed/truncated/oversized
records and unsupported versions. Writers cap segment/session size and use
rotation limited to their own recording directory; errors stop capture visibly.
No synchronous high-rate disk IO inside marine receive callbacks.

## Replay boundary

Replay is visibly marked REPLAY throughout the shell. It drives owned OpenNav
snapshots only, using a deterministic virtual timeline that preserves inter-item
age and dropouts. It never injects positions, routes, AIS or controls into the
real OpenCPN model. Live acquisition and recordings cannot be silently blended.
Every OpenNav hardware command is disabled during replay. The integration
preflight refuses active output-capable OpenCPN connections (and marine drivers
whose output direction is unknown). Review recordings offline in the portable
profile; OpenNav does not intercept arbitrary third-party plugin transports. A selected recording remains
safe after the original route/device is destroyed. End/pause/seek behavior must
not make retained data look freshly measured.

## Calibration

Export only coherently timed, valid speed and whole-pack/motor power pairs with
clear reference (STW versus SOG), device, units, sign and quality. Do not quietly
mix total pack consumption with motor-only power. Empirical curve import remains
explicit, bounded, strictly increasing speed and no extrapolation. Auxiliary
loads and efficiency must have documented configuration; no sample boat curve
is installed as if measured. Raw observations remain available for human review
before a derived curve becomes a prediction input.

## Gates

Portable and upstream-linked tests must cover full round trip, source/age
preservation, navigation privacy options, deterministic replay/dropout,
record/rotation limits, failed IO, malformed/nonfinite/out-of-order input,
calibration filtering and control exclusion. Native runtime tests must exercise
record/start/stop/replay/export and source-loss UI without modifying the real
profile. Windows and Linux validate the same commit before acceptance.

## Implemented format and controls

`diagnostics/Recording` stores a strict version-1 UTF-8/hex/tab `.onxr` file.
Times are milliseconds relative to session start, including pre-start retained
observations. Every numeric item has canonical units, validity, original time,
aging/stale thresholds, source and device. Discrete gear/regeneration/connectivity
retain the same contract. `F` records identify original live versus Demo state;
`R/L` route records require explicit navigation consent. They contain copied
route revision/active-point identity and the accepted route distances/courses,
not recalculated geometry. No AIS identity/history or anchor locations are
included in this format. A complete `END` count is required; malformed or
partially written files fail closed.

Limits: 8 MiB/file, 3,600 frames/file, 128 KiB/frame, 4 KiB/item text, 128 route
steps, 60 KiB model assumptions, one-day capture session. UTF-8, numeric domains,
units, duplicate records, revision/index, route/time coherence and finite values
are checked. The only saved settings are energy/current/hazard assumptions;
profile paths, connection configuration, source pins and custom mappings are
not copied. Provenance/device strings can contain transport names/addresses and
should be reviewed before sharing even an instruments-only recording.

The commissioning service is created only in XNav. **Menu → Commissioning &
recordings** (development shortcut Ctrl+Shift+C) provides the bench overview,
source selection entry, instruments-only capture, explicit navigation capture,
stop/save, file replay, pause/resume, rewind and stop. Capture defaults OFF at
every launch. Source callbacks perform no recording IO: the shell submits owned
snapshots at most once per second to an eight-frame queue, with serialization
and IO on a worker. A slow/full queue or disk error stops recording and exposes
the error in the page and main status bar. Only this session's own segment files
are rotated, retaining three segments; previous sessions are never deleted.
The chosen application log root is resolved once (including normal Windows
short-name/case aliases); only the uniquely created canonical session directory
is used for IO. Replacing session/file paths with links fails closed.
Atomic checkpoints are published every ten frames and at stop/close. A crash
can lose the last nine frames; the previous complete checkpoint remains usable.
The UI shows captured versus saved counts. A normal stop joins the worker.

Replay keeps a fixed epoch and explicit virtual clock. Pause freezes that clock
and is labelled PAUSED; it never renews sample timestamps. Resume continues that
timeline. After the last frame the clock keeps advancing so retained data turns
stale. Rewind selects the original frame times. UI reads cannot renew them.
Historical live records retain live battery identity requirements; setting the
separate `replayed` flag does not turn them into Demo or bypass those checks.
No historical position/route/AIS is injected into OpenCPN. Live source candidates,
AIS and anchor state are withheld from the replay view. OpenNav navigation edits,
advanced settings entry and pilot commands are guarded at the application/
integration boundary as well as in the UI. Ending replay does not enable a pilot.

The chart continues to be OpenCPN's own chart/current live state and is not an
animation of recorded ownship positions. This is a diagnostic replay, not a
navigation transport or GPX importer. The main status and page banner identify
REPLAY, including when the recording originally came from Demo.

## Calibration observations

The export action selects a recording, STW/SOG reference, whole-pack / motor
electrical / shaft power basis and an exact power-device identity. It exports
canonical knots/kW with source, age and measured/derived-or-estimated/Demo quality.
Pairs require positive speed/discharge, ages at most two seconds and observation
skew at most one second. Duplicate observations, wrong devices, uncertainty,
stale/missing data, regeneration/charging and stopped-vessel frames are omitted.
Source changes remain labelled in individual rows, never silently averaged.
Text cells receive a non-formula prefix for safe spreadsheet review.

This export does **not** fit or install a curve automatically. Review the
observations, derive a defensible empirical curve, then import the existing
`OpenNavXPowerCurve,1` format in Energy settings. STW versus SOG and total pack
versus motor/shaft power remain explicit. Existing no-extrapolation, hotel-load,
efficiency, capacity, reserve and freshness rules still apply.

Portable tests now cover codec/route lifetime and original ages, privacy,
malformed/oversized/Unicode/nonfinite inputs, pause/end/rewind, calibration
filtering, asynchronous rotation/failed IO, control and navigation-mutation
isolation, and transport preflight. `tools/smoke-recording.py` drives the real UI
in an offline disposable profile on both platforms and retains screenshots and
recorded/exported evidence. Acceptance is recorded separately for the exact
published revision after both platform gates and native visual review.
