# Alpha instrument source contract

`SensorRegistry` is a portable, owned-value reducer. OpenCPN keeps connection
ownership and its existing selected position/SOG/COG semantics. This registry
adds selection only for the other instrument quantities; `Merge` cannot replace
those three selected-navigation inputs. Integration observes on the application
thread and consumers receive copies, without upstream pointers.

Every sample retains normalized units, source, device identity, observation time,
validity and per-quantity aging/stale thresholds. Reading selection or diagnostics
never changes observation time. Zero is a value; absent/nonfinite/out-of-domain
values are invalid. Delayed and future observations are rejected. Candidate sets
are bounded to 32 sources per quantity. Reconfiguration retains observations;
clearing observations retains configured policies.

Automatic selection prefers fresh measured over estimated/uncertain observations,
then the configured numeric priority (lower wins). Stable source identity breaks
ties. A stale preferred source may fall back to a fresh candidate. Explicit
source pinning never silently falls back: a missing or stale pinned source stays
missing or stale. Diagnostics expose every candidate and which is selected.
Position freshness remains the previously accepted five-second contract.

## Battery coherence and physical meanings

Raw battery current has a separate field with its source convention. It is not
automatically labelled positive discharge. Canonical current and V×I whole-pack
power require an explicitly configured pack device and current polarity. Voltage
and current must be fresh measured observations from the same device and exact
observation epoch. The derived power is estimated and keeps that epoch. The
installation must confirm the sensor is a whole-pack shunt including hotel loads.
Energy consumption also requires SOC from that configured battery device.

Engine coolant and motor temperature are distinct fields. Depth is below the
transducer, not under-keel clearance. Heading is true heading only; apparent wind
is never relabelled true wind. Capacity in a marine message is not guessed to be
usable kWh. No Leaf CAN IDs occur in the desktop model.

## Validation and integration status

`sensor_source_selection`, `sensor_source_validity`, and
`sensor_battery_coherence` test priority, pinning, stale fallback, read age,
owned lifetime, zero/missing/invalid inputs, out-of-order/future input, source
bounds, coherent pack observations and configured polarity. The first Alpha
core increment adds these contracts and diagnostics fields; marine-bus decoding
and settings persistence are subsequent integration work, not claimed here.
