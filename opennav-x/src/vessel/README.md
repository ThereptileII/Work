# Vessel Data boundary

Initial portable snapshot, quality assessment and deterministic simulator
contracts are implemented. The read-only selected-navigation input is described
in `docs/navigation-data-bridge.md`; native acceptance is tracked there and in
the baseline evidence. Snapshots use
explicit validity, age, units, estimate and uncertainty metadata. Missing is not zero. Prefer
normalized marine input through OpenCPN/plugin messages; the UI does not decode
Leaf CAN or device-specific formats. Simulator data is a separate, explicit
source. No simulation source may silently substitute for lost live data.
