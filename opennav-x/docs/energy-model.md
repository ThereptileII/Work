# Advisory energy model and preview presentation

## Alpha calibrated input contract

`EnergyConfiguration` adds a live-input wrapper around the tested numerical
core. The Alpha settings integration also honors shorter per-source freshness
limits without relaxing the established energy age ceiling. A live calculation requires explicitly configured usable capacity, reserve
and battery device identity. SOC and measured whole-pack discharge must belong
to that device. Missing configuration suppresses estimates. Demo remains an
explicit separate source and is never substituted for live telemetry.

An optional empirical curve uses this UTF-8/ASCII CSV format (the numbers below
are test examples, not installed boat defaults):

```csv
OpenNavXPowerCurve,1
reference,STW
basis,whole-pack
speed_kn,power_kw
2,1
4,3
6,9
```

`reference` is explicitly `STW` or `SOG`. `basis` is `whole-pack`,
`motor-electrical` or `shaft`. Speeds must increase strictly; two to 512 finite
positive power points are accepted. The import is bounded to 32 KiB and retains
its source. Linear interpolation is allowed only inside the measured domain;
there is no extrapolation. Motor curves require configured hotel power; shaft
curves additionally require configured efficiency in (0,1]. Neither is guessed.
The required speed sample must be fresh. The estimate keeps its original time,
uses SOG for passage duration, and reports its curve/reference/power assumptions.
This remains a constant-condition advisory estimate, not a leg/weather forecast.

`energy_curve_import` and `energy_live_configuration` cover format/units,
round-trip import, domain limits, interpolation, STW/SOG distinction, battery
identity, efficiency/hotel configuration, stale inputs, invalid route states and
energy shortfall. The [Alpha settings contract](alpha-settings-contract.md)
connects explicit profile configuration and normalized live inputs to the route,
energy, diagnostic and advisory consumers. Physical calibration acceptance
remains a separate boat gate. Existing energy and Demo regressions are retained.

The tested calculation core was implemented after the dual-mode Windows gate
passed at `c5a0fd0`. Developer Preview consumes it through owned Vessel Data
snapshots; see the presentation section below and current acceptance in status.
Linux and native MSVC portable tests pass. Explicit Win32 contracts passed at
`ae5ed81` and `c2535f5`, matching the approved application ABI; earlier portable
jobs used the generator default. The current nine-contract suite includes the
energy core and its Demo-driven snapshot consumer.
Full integration gates and remaining product work are tracked in
[project status](status.md).

## Source inspection and boundary

The existing `src/vessel/VesselState.h` distinguishes battery SOC, voltage,
current and usable capacity from propulsion electrical power. The model
therefore requires **total net battery discharge including hotel loads**; it
does not substitute the motor-only power field or invent unmeasured hotel loads.
No Leaf-specific CAN decoding belongs here or in the UI.

OpenCPN `model/include/model/route.h::m_route_length` is total planned route
length. `routeman.h::GetCurrentRngToActivePoint` is range to the active waypoint.
Neither is blindly treated as remaining distance to the destination. The
accepted read-only route bridge observes the current active-point range plus
subsequent stored legs after normal OpenCPN progress, preserving pinned geometry
and coherence rules. `VesselEnergy` accepts that owned snapshot without retaining
upstream objects. No upstream source is changed for the energy calculation.

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

Remaining before live vessel energy acceptance: acquire actual battery SOC and
net discharge with provenance; configure/calibrate usable capacity and reserve;
validate against recorded and live boat data. The preview supplies the first
advisory presentation with explicit synthetic inputs, as described below.
The model has no device control path. Its original core acceptance predates the
preview presentation described below.

## Developer Preview presentation

The accepted route snapshot now feeds `VesselEnergy` and the preview Route/Energy
views. See [the preview contract](developer-preview-contract.md) for freshness,
source separation and explicit Demo assumptions. Demo uses 48 kWh / 15% reserve;
live configuration remains unavailable. The existing numerical model is unchanged.
Nine portable tests now include Demo-driven valid, stale, missing, high-power,
low-SOC, inactive-route and energy-shortfall consumption. Production live battery
acquisition and calibration remain future work.

## Beta recording/calibration boundary (in development)

The commissioning recorder preserves normalized measurements and energy
assumptions without renewing sensor times. Its explicit calibration export
selects STW/SOG, total-pack/motor/shaft basis and device identity, then withholds
stale, uncertain, incoherent, duplicate, charging and stopped-vessel pairs.
Measured and derived/estimated pairs retain their source quality; Demo pairs
remain labelled. Export is for human review, not automatic model installation.
The existing bounded empirical curve import and advisory model remain the
prediction boundary. No unmeasured boat curve or usable capacity is supplied.
See [recording/replay contract](recording-replay-contract.md) for file limits,
privacy, capture/replay clocks and test gates. Exact native acceptance remains
in [status](status.md); physical calibration remains outstanding.
