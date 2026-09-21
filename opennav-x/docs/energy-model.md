# Advisory energy model, first increment

Implemented after the dual-mode Windows gate passed at `c5a0fd0`. This is a
tested calculation component, not the completed propulsion/energy product screen.
Linux and native MSVC portable tests pass at `5e1320d`. The Windows contract
lane is being aligned explicitly to Win32, matching the approved application
ABI; earlier portable jobs used the generator default. Full integration gates
remain tracked in the project status.

## Source inspection and boundary

The existing `src/vessel/VesselState.h` distinguishes battery SOC, voltage,
current and usable capacity from propulsion electrical power. The model
therefore requires **total net battery discharge including hotel loads**; it
does not substitute the motor-only power field or invent unmeasured hotel loads.
No Leaf-specific CAN decoding belongs here or in the UI.

OpenCPN `model/include/model/route.h::m_route_length` is total planned route
length. `routeman.h::GetCurrentRngToActivePoint` is range to the active waypoint.
Neither is blindly treated as remaining distance to the destination. This
increment accepts an explicit current remaining-distance sample; a later
read-only route bridge must calculate it using the actual remaining legs and
OpenCPN's route geometry. No upstream source is changed for the energy model.

## Inputs and assumptions

- Configured capacity C in kWh: deliverable energy across the BMS's reported
  0–100% SOC interval, with a recorded model source. No default boat capacity.
- Configured reserve R in percent, required explicitly and bounded 0–100.
- Fresh SOC S in percent, SOG V in knots and positive total net discharge P in kW.
- Fresh remaining route distance D in nautical miles for arrival prediction.
  Without a route, range can still be estimated; arrival stays unavailable.
- Linear SOC-to-energy relationship, constant current speed and total discharge.
  No claim to model future wind/current, battery temperature, aging, route-leg
  speeds, regeneration or propulsion efficiency changes. All results must be
  presented as estimates with these assumptions inspectable.
- A configurable low-speed floor defaults to 0.5 kn. Below it, no underway
  range or passage prediction is returned. Zero/negative net discharge gives no
  infinite-range claim. These states require a different model, not division by
  a tiny substitute value.

Inputs use Vessel Data source, validity and receipt-time metadata. Stale,
uncertain, future-dated, missing, unsourced, nonfinite and out-of-domain inputs
cannot produce an apparently current result. Fresh explicitly estimated inputs
are allowed only within this already advisory estimate model. Capacity settings
are configuration, not a five-second sensor stream. The result retains model
source and calculation time; a future consumer must recalculate as inputs age.

## Equations

Energy now = C × S / 100 kWh.
Energy above reserve = C × max(0, S − R) / 100 kWh.
Endurance above reserve = energy above reserve / P hours.
Range above reserve = V × endurance nautical miles.
Passage duration = D / V hours; passage energy = P × duration kWh.
Arrival SOC = 100 × (energy now − passage energy) / C.

If passage energy exceeds available energy, arrival SOC is unavailable and the
required extra energy is reported explicitly. It is never a negative physical
SOC or a fabricated successful 0% arrival. A below-reserve arrival is flagged.
When D is explicitly zero, fresh current SOC supports a zero-duration arrival
without needing speed or power. A missing D never takes this shortcut.

## Validation and remaining work

The deterministic fixture uses a **synthetic** 20 kWh pack, 80% SOC, 20% reserve,
5 kn, 2 kW total discharge and 10 NM passage: 12 kWh above reserve, 6 hours/
30 NM range, 2-hour passage using 4 kWh, and estimated 60% arrival SOC.
These values are test data, not the user's vessel configuration.

`energy_prediction_contract` covers reserve boundaries, exhausted energy,
stationary/charging states, missing/stale/uncertain inputs, invalid domains,
floating-point limits and conservation/monotonicity across a grid of capacities
and SOCs. Windows MSVC runs the same portable contract.

Remaining before a user-facing energy feature: acquire actual battery SOC and
net discharge with provenance; configure/calibrate usable capacity and reserve;
integrate remaining route distance; define presentation/uncertainty; build the
propulsion screen; validate with recorded and live boat data; review Windows UI.
The model has no device control path and is not connected to the shell yet.
