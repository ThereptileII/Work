# Marine input bridge

`MarineBridge` owns application-thread subscriptions to OpenCPN's `NavMsgBus`.
It creates no serial, CAN, TCP or Signal K connection and sends no messages.
Legacy and Safe do not instantiate it. Its copied instrument samples merge
with the unchanged selected position/SOG/COG subscriber in XNav. Demo bypasses
the live result rather than filling missing live fields.

## NMEA 2000

The pinned `N2kMessages` parsers decode bounded, validated OpenCPN receive
envelopes. Type, PGN, source address and exact declared payload size are checked
before constructing the parser message. All fields are copied before returning.
Unknown PGNs and malformed frames supply no observations. Missing-value
sentinels invalidate fields rather than becoming zero.

| PGN | Alpha normalized observations |
| --- | --- |
| 127245 | Measured rudder angle; command/order fields are not observations |
| 127250 | True heading, or estimated magnetic heading plus same-message variation |
| 127257 | Roll/heel |
| 127488 | Engine/motor RPM |
| 127489 | Engine coolant field; motor meaning requires explicit boat mapping |
| 127493 | Transmission forward/neutral/reverse; unavailable for unknown |
| 127751 | Wider-range DC voltage/current pair, by connection instance |
| 127505 | Fresh water, fuel or waste tank level, by instance |
| 127506 | Battery SOC/SOH; coulomb capacity is not guessed to be usable kWh |
| 127508 | Battery voltage and source-convention current, by instance |
| 128259 | Speed through water |
| 128267 | Depth below transducer; offset is not applied silently |
| 130306 | Apparent wind, or estimated true wind with explicit ground/water reference |
| 130310 / 130316 | Sea temperature; 130310 atmospheric pressure |

The pinned 127489 parser body is compiled out. Beta copies its inspected coolant
field without modifying the upstream decoder. It does not silently substitute
coolant for motor temperature. The signed 127508 codec still cannot represent
HV pack voltage; 127751 now supplies a coherent wider-range pair. Its current
sign is not inferred. See [actual boat-source inspection](beta-boat-source-inspection.md)
for producer mappings, source-freshness limits and commissioning gates.

Source identity retains interface, available NAME, source address, instance,
PGN and reference meaning. Address reassignment changes the selection identity;
an explicit pin must be reviewed after that change. Receipt age is translated
from upstream `system_clock` to the Vessel Data steady clock. Transport/sensor
latency before OpenCPN is not claimed known.

## NMEA 0183

HDT, VHW, DPT, DBT, MWV, MTW and RSA use the pinned `SENTENCE` checksum/field
parser and inspected core/Dashboard field meanings. Missing checksums, invalid
checksums, malformed fields, absent numbers and invalid status never yield
valid zero. Relative and true MWV have distinct provenance. RSA's port and
starboard/single rudders are separate candidate sources. DPT offsets do not
change below-transducer depth. OpenCPN's selected navigation remains separate.

## Signal K

Read only own-vessel deltas with an explicit matching context, source identity
and valid UTC timestamp. Preserve observation age; reject future, malformed,
more-than-one-day-old timestamps and oversized messages. Null, boolean and
string values never become numeric zero. Units and paths follow the
[Signal K vessel schema](https://signalk.org/specification/1.7.0/doc/vesselsBranch.html).
Heading provenance is estimated because this path may already include variation.

Standard mappings include heading, STW, wind, depth, water temperature, pressure,
rudder, attitude/heel, propulsion revolutions and coolant temperature, battery
voltage/current/SOC/SOH and tank levels. Battery current's documented sign is
positive out of the device; installation configuration must still confirm the
pack/shunt identity and mapping before whole-pack prediction. The decoder does
not equate nominal or measured total capacity with configured usable energy.

Explicit `SignalKBinding` entries allow exact additional numeric paths with
configured canonical-unit scale/offset. No default proprietary motor power or
winding-temperature path is assumed. Motor-only power never becomes whole-pack
power. Text state/regeneration acquisition remains an additional mapping gate;
the existing fields stay unavailable until supported input exists.

## Selection and tests

Initial automatic priorities are NMEA 2000 (10), NMEA 0183 (20), Signal K (30),
then the per-quantity freshness/validity rules in the
[source contract](vessel-source-contract.md). Explicit pins override fallback.
No source is blended with another device implicitly.

Thirteen upstream-linked `OpenNavMarine` tests cover real pinned codecs,
truncation/PGN/NA checks, heading/wind reference semantics, voltage saturation,
units, checksums/status, missing versus zero, Signal K context/time/source and
explicit extensions. Live UI subscription tests and native MSVC remain required
integration gates. Boat data and sensor calibration are not physically accepted.

Advanced explicit propulsion mapping import and persistence are described in
[propulsion-source-mapping.md](propulsion-source-mapping.md). Standard marine
paths retain precedence; no proprietary boat fields are assumed by default.

## Beta source health

Bounded per-source counters and an interval EWMA expose update rate without raw
bus logging. Duplicate/out-of-order/future observations cannot inflate the rate.
Rate decays during dropout and becomes unavailable at the source stale threshold;
reads never update it. Invalid input has a separate count and INVALID display
state, with its original observation age. Device/PGN/instance and selection/
priority remain visible. Other tank types retain fluid type in source identity;
gear preserves validity and age when converted to its display text.
