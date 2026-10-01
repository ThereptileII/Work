# Native Windows wxCurl trust gate

The SCRUM-211 native trust harness also compiles the actual integrated wxCurl
`base.cpp` and `http.cpp` with Visual Studio 2022 Win32, real wxWidgets, and the
maintained curl headers and import library. The maintained include directory is
ordered before wxCurl's legacy bundled curl snapshot. The target deliberately
does not define `OPENNAV_WXCURL_TLS_TEST`, so it exercises the production
Windows native-CA path.

The wxCurl cases reuse the downloader gate's disposable GitHub Windows runner,
owned loopback TLS servers, exact CurrentUser Root CA lifecycle, manifest-bound
installed runtime DLLs, and verified cleanup. GET and HEAD must accept the
36-byte payload and HTTPS redirect, while wrong-host, expired, untrusted,
downgrade, file redirect, and removed-trust cases must reject. Every case also
proves that a failed curl option prevents transfer execution.

Linux source and PowerShell checks do not satisfy this gate. Native MSVC build,
execution, certificate-store cleanup, and retained per-case evidence are
required before acceptance.
