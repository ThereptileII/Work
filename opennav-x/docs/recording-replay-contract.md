# Bounded normalized recording and replay — Beta design

Status: planned contract, implementation and acceptance pending. This extends
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
Every hardware output is disabled during replay. A selected recording remains
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
