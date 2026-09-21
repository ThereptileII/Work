# SmartNav boundary

The first dual-mode gate passed on both platforms at `c5a0fd0`. `Energy.h/.cpp`
now contain the initial constant-condition advisory model; see
`docs/energy-model.md`. It has portable numeric/failure-state tests and no widget,
OpenCPN-global or device-command dependency. Native validation remains required.

No prediction is yet displayed: battery/power acquisition, capacity configuration,
route-remaining-distance integration and the propulsion UI are separate work.
Route events, turns, hazard/AIS context and sailing/anchor intelligence are still
unimplemented. SmartNav never emits steering commands.
