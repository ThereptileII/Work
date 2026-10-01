# WXCURL HTTPS trust boundary

SCRUM-211's bounded WXCURL slice changes the pinned `wxCurlBase` transfer
configuration used by the actual `wxCurlHTTP` class. It removes the global TLS
verification bypass and the Windows working-directory `curl-ca-bundle.crt`
lookup. HTTPS now requires certificate-chain and hostname verification. On
Windows every transfer requests libcurl's native CA store, covering HTTPS,
FTPS and a non-TLS origin which redirects to HTTPS. This requires the
maintained curl 8.22.0 integration from SCRUM-209.

HTTPS redirects are limited to HTTPS and capped at five. This restriction is
selected only when the initial URL is HTTPS, so existing FTP, Telnet and other
supported initial protocols are not globally disabled. An HTTP or local-file
redirect from an HTTPS request fails in libcurl before the destination is
accessed.

Every failed `curl_easy_setopt` marks the handle unusable, and `Perform` refuses
to execute that partially configured request. The checked state covers the
existing HTTP, FTP, Telnet and DAV wrappers without replacing their public API.
The Windows build has a compile-time floor on libcurl headers with native-CA
support; the ancient 7.37 headers stored under `libs/wxcurl/src/include` cannot
silently describe this Windows contract. The maintained CMake curl include
path supplies the supported headers.

`tools/test-wxcurl-trust.py` copies the actual pinned WXCURL sources, applies
the reviewed patch, compiles `base.cpp` and `http.cpp` against the project's
real wxWidgets libraries and libcurl, and executes the real `wxCurlHTTP::Get`
and `Head` methods against owned loopback TLS endpoints. Its test-only compile
definition selects an isolated CA file. Production code has no environment CA
override and Linux uses libcurl's configured system trust. The harness obtains
all wxWidgets compile/link flags from `wx-config` and libcurl flags from
`pkg-config`; a caller may select a relocated wx installation with
`WX_CONFIG`, `WX_CONFIG_PREFIX` and the ordinary runtime library environment.

The Linux harness accepts a valid chain and HTTPS redirect from an unrelated
working directory. It rejects untrusted, expired and wrong-host certificates,
HTTP downgrade, local-file redirect and missing trust for both GET and HEAD.
It also performs ordinary plain-HTTP and HTTP-to-HTTPS redirect transfers,
verifies FTP and Telnet configuration remain accepted, and proves a
deliberately rejected option prevents the actual class from entering
`Perform`.
This is transport evidence, not native Windows, full application, installer or
release acceptance. Peer pairing, local peer identity and boat behavior remain
unchanged.
