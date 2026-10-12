# Read-only AIS connection attribution

SCRUM-227 needs attribution before any separately authorized outage test. This
contract supplies bounded observation evidence only. It neither permits filtering
nor replaces the existing inert-only Windows Filtering Platform guard.

## Product observation

By default the product does not export a network tuple. Launching it with the
exact process environment value `SKAGER_AIS_CONNECTION_DIAGNOSTICS=1` opts in to
`runtime.online_ais.connection_observation` in its existing diagnostic JSON.
No user-facing Demo or synthetic source is introduced.

The integration bridge copies the provider's owned connection observation after
reading its normal health snapshot. It does not query a socket, reconnect,
resubscribe, change the chart, or refresh the provider's capture time. An available
schema-1 observation contains source `AISSTREAM_ONLINE`, process ID, read UTC,
explicit IPv4/IPv6 family, connection generation, capture/read monotonic times,
capture age and local/remote addresses, ports and scopes. Addresses use 32
lowercase network-byte hex digits; IPv4 uses four bytes followed by twelve zero
bytes. Monotonic nanoseconds, generation and age are decimal strings to avoid
JSON precision loss. UTC has whole-second `YYYY-MM-DDTHH:MM:SSZ` form.

Disconnected or incoherent state is explicitly unavailable and has no endpoint
fields. Future/invalid timestamps, unsupported families, ambiguous mapped IPv6,
invalid ports and arithmetic overflow are refused. Capture age is **not** traffic
liveness: a long-lived healthy connection legitimately has an old capture time.
Reading diagnostics does not make a retained connection fresh.

## Independent native observation

`tools/collect-ais-connection.py` is an operator-only Windows Python 3.8+ helper,
not a product runtime dependency. Its inputs are an explicit PID, diagnostic
file, independently reviewed expected image/module path-and-SHA-256 inventory,
and a new output path. It must not derive its authority from writable adjacent
installation ownership metadata.
Native DLL loading is restricted to System32.

The helper checks the actual installed-product/fixture-free diagnostic markers,
retains a native process handle, observes creation identity, inventories loaded
modules, and hashes held on-disk image/module files. It compares the complete
module inventory to reviewed pins before opening those paths. Two process,
module, established-TCP and fixed-host DNS samples surround separately advancing
product diagnostic observations. Multiple matching connections, process identity
changes, unknown modules, inaccessible competing same-name processes, changing
DNS, stale diagnostics and ambiguous paths fail closed.

Only independently pinned, retained binary files may have a stable positive
hard-link count greater than one. Their handles deny write/delete sharing and
their full file identity, including the link count, is checked again. Input
manifests, diagnostic files and newly created output retain the single-link
restriction. Output failure cleanup uses the retained created file handle; it
must never delete a later file merely because it has the same name.

The only network operation is resolution of `stream.aisstream.io`. No credentials,
API keys, profile configuration, vessel positions, routes or unrelated diagnostic
fields are exported. Output is a bounded, exclusively created evidence JSON file;
errors use fixed redacted text. A collector-child watchdog may terminate only its
own helper child, never OpenCPN/SKAGER. Native handle cleanup is not skipped because
the observation deadline expired.

## Portable validation and authority limits

`tools/ais_connection_binding.py` validates strict bounded JSON, independent pins,
identity continuity, exact tuple membership, ambiguity, DNS membership, timestamp
ordering and freshness. Observations must occur within a collection of at most
15 seconds and be at most 10 seconds old when validated. The same capture time
and generation must remain present while the diagnostic read time advances.

A valid result is only:

```json
{"schema":1,"state":"OBSERVATION_MATCHED","authorizes_filter":false}
```

Disk hashes do not authenticate mapped memory. Two sampled inventories cannot
prove that no transient module loaded between them, that DNS will stay fixed,
or that a socket will remain owned after collection. PID/path/hash evidence is
not Windows Filtering Platform authorization. A same-user diagnostic file is
not a cryptographic attestation. Never turn this result into a durable permission,
broaden the inert guard, or interrupt boat traffic automatically.

Focused Linux checks cover the pure exporter and offline validator/collector
behavior. Windows ABI, actual API execution, real installed loader closure,
current product diagnostics and live boat attribution remain separate native
qualification gates. A cross-compile or mock result is not their acceptance.

Native API references: [TCP owner rows](https://learn.microsoft.com/en-us/windows/win32/api/tcpmib/ns-tcpmib-mib_tcprow_owner_pid),
[Windows component-store hard links](https://learn.microsoft.com/en-us/windows-hardware/manufacture/desktop/manage-the-component-store?view=windows-11).
The complete module inventory includes operating-system DLLs; a single-link
assumption for every binary is not a valid general Windows installation model.
