# SCRUM-211 pinned Downloader baseline

On 2026-09-30 at 21:23:49 CEST, a temporary probe compiled the actual pinned
OpenCPN `model/src/downloader.cpp` at revision
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`, using the local wx/curl
dependencies. The compiler command and sanitized output are retained under
`evidence/local/scrum211-baseline/`.

An owned HTTPS server on `127.0.0.1` used a newly generated self-signed
`CN=localhost` certificate. The probe called both production `Downloader`
operations: `download()` accepted the 36-byte payload with error code 0, and
the protected `get_filesize()` path performed a HEAD request and returned
content length 36 with error code 0. The server log records `GET` and `HEAD`.
The downloaded bytes match the recorded expected payload SHA-256.

Independent controls rejected the same certificate: Python `urllib.request`
failed with `SSLCertVerificationError`, and normal curl exited 60 with
`self-signed certificate (18)`. This is a baseline observation of the
existing trust boundary; it does not claim an exploit. The result is
consistent with the pinned source's explicit `CURLOPT_SSL_VERIFYPEER=0L`.

The first server attempt was denied by the sandbox's loopback socket policy.
The exact same temporary fixture and probes succeeded after scoped escalation.
No trust store, repository source, credentials, plugins, boat hardware or
external network was used. The certificate, private key, executable and probe
sources were not copied into Git. This evidence does not qualify remediation,
native Windows behavior, or release acceptance.
