# Private HTTPS from the boat — 2026-10-14

After the owner's Tailscale Serve activation, the existing Serve configuration
maps the pinned private hostname's HTTPS port 443 to the existing loopback origin
at `127.0.0.1:8765`. No public Funnel or remote-access configuration was changed.

A read-only PowerShell request through the existing `ssh boat` alias fetched
`https://omarchy.tail5878fc.ts.net/1.root.json` with native certificate validation,
automatic redirects disabled, a 15-second timeout and an 8 KiB response limit.
The command exited zero and returned:

- HTTP 200; no redirect.
- 1,577 bytes.
- SHA-256 `a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57`.
- Exact match to the separately retained public root from October 12.

This observation was recorded at 2026-10-14 08:03:32 UTC. It supersedes only the
earlier **boat HTTPS reachability** blocker in
[the October 12 receipt](../scrum311-private-origin-20261012/README.md).
It does not establish signed-offer publication, expiry/rollback behavior,
installed update/repair/uninstall, public release or CI-runner connectivity.
The hosted runner needs its own authorized private-network connection.
No private key, credential, boat profile or hardware command was read or changed
by this check; only the public root's length and digest were returned.
