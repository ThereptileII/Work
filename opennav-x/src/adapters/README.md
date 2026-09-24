# Hardware adapter boundary

The Alpha interfaces, manual autopilot reducer and deterministic simulators are
implemented with portable tests. See `docs/hardware-adapter-contract.md`.
Only an explicitly enabled simulator can accept commands in Alpha; live output
is unavailable. Requested state is never observed state. No SmartNav dependency
or autonomous steering path exists. Native UI and physical tests remain gates.
