# Native Windows downloader trust gate

SCRUM-211's native gate builds the actual prepared `model/src/downloader.cpp`
with Visual Studio 2022 for Win32 using `/MD`, real wxWidgets 3.2.8 `base` and
`core` libraries, and the maintained `cache/buildwin` libcurl headers and import
library. It deliberately does not define `OPENNAV_DOWNLOADER_TLS_TEST`, so the
probe exercises the production Windows `CURLSSLOPT_NATIVE_CA` path.

The gate may run only on a disposable GitHub-hosted Windows runner in `ThereptileII/Work`. It creates
a unique CA and server fixtures locally, adds only that exact thumbprint to
`Cert:\LocalMachine\Root`, serves only owned loopback HTTPS endpoints, and tests
GET plus HEAD behavior. Valid content and an HTTPS redirect must pass.
Untrusted, expired, wrong-host, HTTPS-to-HTTP, file redirect, initial HTTP,
partial-body, stream-write-exception, and removed-trust cases must fail without
replacing the destination. A successful case also runs from an unrelated
working directory.

The script always stops its server processes, removes the exact certificate it
added, verifies that certificate is absent, and fails if cleanup is uncertain.
It never changes certificate-store policy, contacts a live endpoint, or
executes a dependency DLL from `cache/buildwin`. The one fresh test root exists
only in the disposable runner and is removed by exact thumbprint; this harness
is never executed on the boat or a customer machine. Runtime DLLs come from the
supplied `build/production-install` or `build/xnav-install`; the installed
libcurl DLL is required to match its maintained producer manifest. Evidence
records every case and SHA-256 hashes for the source, probe, manifest, and all
runtime DLL prerequisites.

SCRUM-288 replaces CurrentUser root import after a native reproduction showed
an unattended Security Warning dialog. The machine-root import uses bounded
`certutil`, verifies both thumbprint and raw certificate bytes, and never
disables certificate or hostname verification. Import and probe subprocesses
have a 30-second limit with progress/output receipts and exact-owned cleanup.

Native MSVC execution is an acceptance gate. Linux parsing or review of this
harness cannot satisfy it.
