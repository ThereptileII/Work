# Explicit propulsion source mappings

Alpha reuses the existing OpenCPN Signal K connection/input bus. Standard RPM
and coolant paths remain decoded by the fixed marine contract. Motor winding
or inverter temperature and electrical/shaft power must not be guessed from an
unrelated coolant/current field. Data Sources therefore supports an advanced,
explicit CSV import for documented boat-bridge extensions:

```csv
OpenNavXSignalK,1
path,quantity,scale,offset
propulsion.main.motorTemperature,motor_temperature,1,-273.15
propulsion.main.electricalPower,motor_power,0.001,0
propulsion.main.shaftPower,shaft_power,0.001,0
```

These are **format examples**, not a configuration for this boat. They mean
Kelvin→Celsius and watts→kW only if the actual bridge contract documents those
units. No mapping is installed by default. Inspect the confirmation's path,
quantity, scale, offset and canonical unit before applying a real mapping.
Hardware verification is still required. No desktop Nissan Leaf CAN IDs exist.

Only explicit `propulsion.instance.path` entries for `motor_temperature`,
`motor_power` and `shaft_power` are supported. GPS/heading, battery SOC/current,
standard RPM/coolant and control commands cannot be replaced by this importer.
The bounded 16 KiB / sixteen-row format rejects unknown quantities, malformed
paths, duplicate paths, zero/nonfinite scales and nonfinite offsets. The live
registry still rejects values outside each quantity's valid domain.

Mappings are configuration owned by Application, interpreted by Integration,
then copied into Vessel Data with the original Signal K timestamp, transport,
source and configured-path provenance. Consumers never see protocol pointers.
Changing/removing mappings clears retained marine instrument observations;
subsequent normal input reacquires them without renewing old timestamps. Saving
unrelated settings with the same mappings does not clear observations.

The shared-profile settings record stores this optional field. Existing records
without it retain zero mappings. Older Alpha candidates which do not understand
the field fail closed and preserve the record, as with other unknown settings;
they do not overwrite it. Advanced diagnostics expose the applied mapping.

Power at the motor or shaft is never substituted for whole-pack consumption.
Live energy still needs the existing configured battery/sign/coherent readings
or a calibrated propulsion curve with its explicit loss/hotel assumptions.
