# SmartNav boundary

The first dual-mode gate passed on both platforms at `c5a0fd0`. `Energy.h/.cpp`
now contain the initial constant-condition advisory model; see
`docs/energy-model.md`. It has portable numeric/failure-state tests and no widget,
OpenCPN-global or device-command dependency. Native validation remains required.

Developer Preview displays advisory energy from its explicit Demo source and
the accepted route snapshot contract. Alpha adds calibrated consumption inputs,
route timeline/turns, energy/AIS events and a chart-corridor abstraction. See
`docs/smartnav-alpha-contract.md`. Live source/UI gates are tracked separately.
SmartNav never emits steering commands or links the adapter library.
