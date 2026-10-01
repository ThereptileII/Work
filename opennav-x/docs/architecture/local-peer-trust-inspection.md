# Local peer trust inspection

SCRUM-212 records a public-beta launch blocker in OpenCPN's local peer object
transfer. This document is a read-only source inspection and design input. It
does not claim an exploit was executed, and it does not implement or accept a
new trust protocol.

## Observed behavior at the pinned source

The send dialog discovers peers over mDNS, displays the advertised hostname and
IP address, then derives port 8443 or 8444 from the displayed name. The selected
hostname becomes `server_name` and the advertised IP becomes
`dest_ip_address`.[^peer-selection] The client performs version, pairing,
writability, and object-transfer requests against that address.

Both the GET and POST paths disable certificate-chain and hostname verification
with `CURLOPT_SSL_VERIFYPEER=0` and `CURLOPT_SSL_VERIFYHOST=0`.[^client-tls]
HTTPS therefore encrypts bytes but does not authenticate which peer received or
returned them.

Client API keys are stored in `/Settings/RESTClient/ServerKeys` as a
semicolon-separated `server_name:key` string. A peer without a stored entry
starts with the value `1`.[^client-keys] The server stores a similar
`source:key` map under `/Settings/RestServer`; `source` is supplied by the
client request.[^server-keys]

On an unknown or incorrect key, the server generates a PIN-derived expected
key, stores it immediately, and displays a dialog saying that the claimed
source wants to send data and that the shown PIN should be entered on that
device.[^server-pairing] The sending client displays a PIN-entry dialog, hashes
the entered PIN, retries `/api/ping`, and stores the resulting key under the
advertised server name if accepted.[^client-pairing] The PIN generator seeds
`rand()` from the current time, emits one of 9,999 four-digit values, and the
new-format key is the first 12 hexadecimal characters of SHA-256 over that PIN
string.[^pincode]

The server certificate is not a persistent peer identity. On every
network-enabled application startup, OpenCPN generates a new key and
self-signed certificate in the private data directory, ignores the generation
return value, and starts the REST server.[^startup] The generated certificate
uses RSA-2048, serial number 1, a one-year lifetime, `CN=localhost`, the first
local IPv4 address as its SAN, and SHA-1 for its signature. The key and
certificate are written directly to `key.pem` and `cert.pem`.[^certificate]
Mongoose loads those files for its HTTPS listener without a client CA.[^server-tls]

The existing user-visible flow reports generic HTTP/curl errors, server result
errors, transfer success, JSON parse failure, bad PIN, and unsupported route
activation. It does not display or ask the user to confirm a certificate or
peer fingerprint.[^dialogs]

## Credential exposure boundary

The client places `source` and `apikey` in query strings for ping, writable
checks, and object transfer.[^query-credentials] The POST path enables libcurl
verbose diagnostics when wx logging is at debug level while passing the full
URL to curl.[^curl-verbose] This can expose the API key in request-line
diagnostics. Query credentials are also inherently available to future URL,
error, proxy, or access logging.

The server globally selects Mongoose's debug log level.[^mongoose-log] The
pinned Mongoose debug paths inspected here do not establish that parsed query
strings are currently written by the default server logger. Raw request
hexdumps or later access logging would expose the same query credential, so the
security boundary must not depend on today's incidental logger behavior.

The narrow redaction boundary is to remove credentials from the URI and carry
one token in a dedicated authorization header. All request construction should
pass through one helper which returns a sanitized URL for diagnostics. Raw curl
verbose output should remain disabled unless a single debug callback redacts
the complete authorization header and any legacy `apikey` parameter before it
reaches a log sink. Server request logging and hexdumps must apply the same
redaction. PINs, bearer tokens, and full certificate pins must never be logged.

## Trust strategies requiring a decision

All viable options must retain support for local self-signed peers. Switching
only to public-CA validation would silently break the existing feature.

### Certificate-bound confirmation

Create a stable random server identity and modern self-signed certificate once,
validate that its certificate and private key match at each startup, store them
with restrictive permissions and atomic replacement, and rotate them only
through an explicit user action.

For first pairing, show a short authentication string on both devices. Derive
it from the presented certificate's SPKI SHA-256 digest and a fresh,
high-entropy server nonce. The user must explicitly confirm that the two values
match. Only after confirmation should the client persist a stable peer ID,
certificate pin, and random 256-bit bearer token. A changed or missing pin must
block before the token or navigation objects are sent and require explicit
re-pairing.

Merely receiving the first certificate must never accept it. This sketch is
not an approved authentication protocol: a short display derived from an
attacker-influenced nonce or key can permit offline search for matching values.
Commitments, transcript binding, display length, retry limits, expiry, token
issuance and recovery require specialist review before claiming resistance to
active attacks. No implementation should proceed from this sketch alone.

### Password-authenticated key exchange

A reviewed PAKE such as SPAKE2 can use a human-entered pairing code without
exposing a verifier suitable for offline guessing, then bind the resulting
session to the server certificate and issue a random long-lived token. This is
cryptographically stronger for short codes, but introduces protocol,
dependency, interoperability, and migration work. No PAKE implementation or
library has been selected.

### Manual full-fingerprint confirmation

Displaying and comparing the complete certificate fingerprint avoids a custom
pairing protocol and can authenticate self-signed peers. It is the smallest
cryptographic model but has poor usability and a high comparison-error risk.
It remains a possible recovery or advanced verification path rather than an
accepted primary flow.

The current four-digit PIN hash must not simply be combined with an
automatically accepted first certificate. That would retain offline guessing
and let an attacker establish the initial pin. Keying trust only by mDNS name
is also insufficient because the advertised name and address are not stable,
authenticated identities.

## Migration and failure behavior

Existing stored key-only peers need an explicit **verify and upgrade** state.
The client may recognize a legacy entry, but must not silently pin the next
certificate it sees. Cancellation or any mismatch stores nothing. Successful
upgrade writes the new identity, pin, and token atomically; only then may the
legacy entry be retired.

Certificate rotation, lost configuration, and peer rename need distinct user
messages. A certificate change is a security stop, not a retryable generic curl
error. Re-pairing must show the old and new identity context and require a new
authenticated ceremony. No automatic pin update is acceptable.

## Required acceptance evidence

The selected design should have deterministic model tests and native user-flow
evidence covering:

- persistent certificate identity across clean restart and rejection of a
  mismatched key/certificate pair, malformed file, or unsafe storage state;
- no token or navigation-object transmission before explicit authenticated
  confirmation;
- successful first pairing with the intended self-signed peer;
- rejection when a relay presents another certificate, including when it
  forwards the real peer's nonce or responses;
- cancellation, mismatched confirmation, timeout, replay, excessive attempts,
  and concurrent pairing attempts leaving no accepted credentials;
- reconnect only when stable peer identity and certificate pin both match;
- changed certificate blocking before any saved token is sent;
- explicit re-pairing replacing the association atomically;
- a spoofed duplicate mDNS name or changed IP not inheriting another peer's
  trust;
- deliberate policy for expired and not-yet-valid pinned self-signed
  certificates;
- legacy key-only migration requiring confirmation and preserving the ability
  to cancel safely;
- query-only credentials rejected after the compatibility window closes;
- logs at every supported level containing neither API keys, authorization
  tokens, PINs, nor unredacted legacy credential parameters;
- Linux protocol/model tests plus native Windows two-instance dialog and
  persistence review.

No boat or physical navigation action is required to resolve this trust design.
Native Windows remains authoritative for the final pairing dialogs, credential
persistence, certificate storage permissions, and upgrade/recovery behavior.

[^peer-selection]: `upstream/OpenCPN/gui/src/SendToPeerDlg.cpp:143-153,303-318,363-383`
[^client-tls]: `upstream/OpenCPN/model/src/peer_client.cpp:109-131,146-159`
[^client-keys]: `upstream/OpenCPN/model/src/peer_client.cpp:166-198`
[^server-keys]: `upstream/OpenCPN/model/src/rest_server.cpp:419-435,486-500`
[^server-pairing]: `upstream/OpenCPN/model/src/rest_server.cpp:503-524`
[^client-pairing]: `upstream/OpenCPN/model/src/peer_client.cpp:257-295`; `upstream/OpenCPN/gui/src/SendToPeerDlg.cpp:125-140`
[^pincode]: `upstream/OpenCPN/model/src/pincode.cpp:31-58`
[^startup]: `upstream/OpenCPN/gui/src/ocpn_app.cpp:1977-1991`
[^certificate]: `upstream/OpenCPN/model/src/certificates.cpp:50-68,102-173,176-214,217-246`
[^server-tls]: `upstream/OpenCPN/model/src/rest_server.cpp:324-336,462-472`
[^dialogs]: `upstream/OpenCPN/gui/src/SendToPeerDlg.cpp:66-140`; `upstream/OpenCPN/gui/src/rest_server_gui.cpp:47-76`
[^query-credentials]: `upstream/OpenCPN/model/src/peer_client.cpp:226-229,267-268,350-359,402-409`
[^curl-verbose]: `upstream/OpenCPN/model/src/peer_client.cpp:109-131`
[^mongoose-log]: `upstream/OpenCPN/model/src/rest_server.cpp:390-403`
