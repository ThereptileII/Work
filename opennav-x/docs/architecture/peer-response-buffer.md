# Peer REST response buffer boundary

The OpenCPN peer client uses this response buffer for `/api/ping`,
`/api/get-version`, `/api/writable`, and `/api/rx_object`. Their replies contain
only status, version, or object-presence JSON. Navigation objects are sent in
the POST request body and are not stored in this response buffer. A 64 KiB
response ceiling therefore provides substantial protocol headroom while
bounding memory controlled by a LAN peer.

The patch initializes the empty allocation as a NUL-terminated string and
rejects failed initial allocation, null state or nonempty input, multiplication
or addition overflow, response growth beyond the cap, and reallocation failure.
Both GET and POST reject a null response, failed initial buffer allocation, or
failed curl initialization before setting options or performing network I/O.
Rejection leaves the previously accepted bytes and terminator intact. The test
harness applies the patch to the pinned source, extracts the marked production
struct and callback verbatim, and compiles that exact code for deterministic
allocation and boundary tests. The standalone target is portable to Linux and
native Windows C++ toolchains.

This change does not alter pairing, authentication, endpoint behavior, or peer
TLS configuration. In particular, the existing disabled certificate and host
verification in `ApiGet` and `ApiPost` remains an unresolved SCRUM-211 security
and native acceptance gate.
