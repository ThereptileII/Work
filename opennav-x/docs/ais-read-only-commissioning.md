# Explicit read-only AIS service commissioning

`aisstream_live_probe` is a separate developer tool, never installed or started
by XNav. It links the same fixed-endpoint TLS/provider and protected credential
implementation as the product. It cannot load an OpenCPN profile, chart, plugin
or marine driver. It has no test-transport endpoint override or synthetic data.

`--describe` is entirely offline. A deliberate
`--read-only-live-ais stockholm` (or `oresund`) connects for at most 45 seconds
to the public region defined in the source. Shutdown disables/clears the owned
cache and joins the network worker. Output contains connection-state enums,
elapsed time and aggregate counts only. No key, server error text, vessel
identity or coordinates are printed. Invalid arguments are not echoed.

Native CI packages only the verified x86 dependency closure, app-local licensed
runtime, notices, hashes and complete corresponding source after its tests and
captures succeed. A clean-PATH offline launch verifies dependency resolution.
The tool reads the current user's existing Windows Credential Manager entry
`OpenNavX/AISStream/v1`; it never exports, modifies or removes it. Linux uses
only `AISSTREAM_API_KEY`. No real key or service connection is required by CI.

Before boat use, verify the exact successful run, artifact/inner ZIP/binary
hashes and capability record. Run through the `boat` SSH alias with the fixed
command, a bounded outer process deadline and private output capture. Do not
change network settings or interfere with remote recovery. This probe does
not require launching the installed application or restoring any previous
commissioning receipts. A later product launch still requires its own fresh
read-only profile/plugin audit and preparation.

A positive result qualifies only service/TLS/subscription/report acquisition.
Actual product chart marks, viewport resubscription, selected cards, offline
aging, reconnect and physical display remain separate acceptance gates.

## First isolated boat probe: negative credential gate

On 2026-09-28, the independently verified `c86c6a7` package from successful
native run `36485693077` was staged in a new isolated run directory and executed
through the SSH alias. It returned exit 4, `credential_missing`, no subscription,
zero targets and `disabledAndCleared=true`. No probe or OpenCPN process remained.
The installed application, profile, charts, plugins and output configuration
were not opened or modified. This only establishes clean dependency loading,
credential-unavailable handling and shutdown in that logon. It does not establish
whether another interactive logon has a credential, nor any live service gate.
See [aggregate evidence](evidence/boat-ais-probe-c86c6a7.json).
