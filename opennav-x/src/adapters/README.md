# Hardware adapter boundary

Portable interfaces, the manual autopilot controller and deterministic simulators
remain independent of UI, SmartNav and OpenCPN pointers. Beta adds the explicitly
bound ST4000 adapter over the qualified OpenCPN TCP path. Saved permission and a
freshly enabled session are both required; fresh measured feedback confirms a
command. Requested/sent state is never observed success. TRACK/WIND remain
unavailable and no SmartNav or autonomous steering path exists.

See [hardware contract](../../docs/hardware-adapter-contract.md) and
[ST4000 transport and physical gates](../../docs/st4000-beta-contract.md).
Radar remains unavailable without a verified scanner/plugin adapter.
