# Boat recovery inspection — September 28

At 18:36 UTC SSH became reachable. Read-only inspection found no OpenCPN
process, the original stock executable SHA-256
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`, and unchanged
navigation/chart database bytes relative to the prior closed checkpoint.
The input-only commissioning transaction is still active. The current INI
differs and must be inspected before restoration, preserving user changes.

The local log records clean frame/application exit at 18:34 UTC. The retained
o-charts decoder was started during plugin teardown. Its reviewed executable
hash is `ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb`.
Its parent has exited and has no tool-owned launch receipt. The old stock-launch
shutdown tool is therefore inapplicable; no receipt was invented or refreshed.
Cold `InspectRestore` correctly refused the still-running helper.

The new orphan-recovery tool verifies the active hash-bound cold transaction,
actual user/session, exact installed generation, complete plugin inventory,
known decoder bytes, exact PID/creation time, absent parent PID and exact
source-derived pipe arguments. Every other app/helper process still blocks.
It reuses the already tested one-shot chart-decoder CMD_EXIT transport, with
actual pipe-server PID checks, retained process handle, bounded cancellation,
measured exit, no retry and no forced termination. The shared attempt locator
prevents using the older tool to retry an uncertain operation.

This is a local chart-decoder lifecycle operation, not marine equipment output.
It does not mutate a profile or restore plugins. A separate cold inspection,
review/adoption of INI changes and full restore verification are still required.
The new policy/native gates and actual boat recovery are pending. Private logs,
user paths and navigation data remain outside committed evidence.
