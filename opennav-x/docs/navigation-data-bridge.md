# Read-only selected navigation slice

Alpha extends other instruments through the separate
[source contract](vessel-source-contract.md). The selected-navigation behavior
documented below remains unchanged.
The Alpha [marine bridge](marine-input-contract.md) now observes additional
instrument messages through OpenCPN's input bus; the original slice's remaining
field limitations below describe its historical scope.

OpenCPN revision: `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
Scope: position, SOG and COG from OpenCPN's decoded, priority-selected message
bus. No connection ownership, NMEA parsing, chart mutation or output commands
are added to the UI. Wind, depth, heading and device telemetry stay unavailable
until separately integrated. Explicit UI simulation remains a separate source.

## Inspected source and decisions

- `model/src/comm_bridge.cpp`, `SendBasicNavdata` and NMEA/Signal K handlers:
  OpenCPN selects input priorities and publishes `BasicNavDataMsg` deltas.
  Reuse that decision rather than open another connection or choose a sensor.
- `model/include/model/comm_appmsg.h`: position validity and individual
  `POS_UPDATE`, `SOG_UPDATE`, `COG_UPDATE` bits distinguish changes from cached
  values in the same message. Refresh only the indicated fields.
- `model/src/cutil.cpp` and `model/include/model/cutil.h`: Windows uses QPC
  for `clock_gettime_monotonic`. Calculate queue age in the upstream clock,
  then subtract that duration from the Vessel Data steady clock. Do not assume
  their epochs match. This is OpenCPN receipt age, not a claim about the physical
  sensor's measurement timestamp or transport delay before OpenCPN.
- `BasicNavDataMsg::source` is not populated with physical sensor identity.
  Expose "OpenCPN selected navigation (sensor identity unavailable)" honestly.
- True heading can be derived from magnetic heading and variation without
  provenance in this message. Do not label it measured; leave it unavailable.
- `libs/observable/include/observable.h`: use the owned `ObsListener` callback
  overload. Subscription exists only in XNav and is destroyed before the shell.
  Safe/Legacy do not construct this OpenNav subscriber.

## Portable contract

`NavigationInput` accepts deltas with canonical units and source/time metadata.
Invalid position suppresses the coordinate pair; negative/nonfinite speed and
invalid course suppress their values. A measured zero remains valid. True north
360° is normalized to 0°. Out-of-order updates cannot overwrite newer values.
Unknown receipt time cannot make a value current. No heading, battery or other
fields are inferred from position or speed. Missing source/time fails closed.

## Validation

`navigation_input_contract` checks partial-update aging, invalid/out-of-range
fixes, zero speed, north wrap, old packets, missing source/time and no fabricated
extra fields. The existing quality contract covers aging/stale boundaries.

`tools/smoke-navigation.py` creates an isolated profile and an input-only TCP
connection to a loopback test server on an ephemeral port. It sends checked
synthetic GGA/RMC with SOG 6.3 kn and COG 147°, then only position updates, then
no updates. Required screenshots show unavailable → live → stale SOG/COG while
position continues → all stale. Wind/depth remain unavailable throughout.
The fixture never uses a production profile or sends to an external device.
Native Windows also checks the visible source-status transitions automatically.
Numeric values and stale card labels require screenshot review on both platforms.

Run these GUI tests sequentially after CTest: upstream REST tests and OpenCPN
both use port 8443. Windows native build, screenshots, shared-profile cycle and
the five portable contracts must pass at the same commit before acceptance.

Both platform gates pass at `bc0af30d647120c90dc603a97fcf7a6ea0e6415c` in
[run 35638553140](https://github.com/ThereptileII/Work/actions/runs/35638553140).
All seven CI jobs succeeded. Reviewed native Windows 1280×800/96 DPI screenshots
show 6.3 kn / 147°, then STALE 7s while position continues, then Navigation stale
and STALE 14s after all input stops. Wind and depth stay unavailable. Numeric
and layout review hashes are in `docs/evidence/windows-bc0af30-review.json`.
Linux's matching sequence and mode cycle pass; the integrated regression suite
passes 60 compiled cases. Native Windows passes 50 platform-available cases and
the portable contract suite. This establishes the selected-navigation slice;
physical sensor provenance and the remaining vessel fields are still pending.

## Remaining active-route distance

The read-only route observer uses selected position provenance while observing
normal OpenCPN route progress. Its contract, lifetime/coherence rules and tests
are documented separately in [route-progress-contract.md](route-progress-contract.md).
The original navigation subscriber and displayed SOG/COG behavior remain unchanged.

## Developer Preview consumers

The shell now copies the current immutable route publication alongside selected
navigation values on the application thread. The separate Demo source never
modifies this live state or fills its missing inputs. Extended fields and
diagnostics are described in the [preview contract](developer-preview-contract.md).
