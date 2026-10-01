# Download trust boundary inspection — SCRUM-211

Status: first bounded Downloader implementation, pending native qualification.
Inspected pinned
OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7` and local integration candidate
`f752daeb0fc07e2abb878c0c43a4b64760f22101` on 2026-09-30. Jira owns the work and
acceptance; this document records the source boundary and proposed decisions.

## Confirmed callers

| Boundary | Observed behavior | Consequence |
| --- | --- | --- |
| `model/src/downloader.cpp`, `download` and `get_filesize` | Explicit `CURLOPT_SSL_VERIFYPEER=0L`; follows redirects | Replacing TLS libraries does not restore certificate-chain validation. |
| `model/src/catalog_handler.cpp` | Catalog and API requests use `Downloader` | The bypass affects more than chart data. |
| `model/src/plugin_handler.cpp`, `InstallPlugin(PluginMetadata)` | Calls `Downloader::download`, ignores its result, then calls archive installation | A transport failure must stop installation; a partial file is not an accepted download. |
| `libs/wxcurl/src/base.cpp`, `SetCurlHandleToDefaults` | Windows sets relative `curl-ca-bundle.crt`; all platforms then disable peer verification | Trust currently depends on an obsolete workaround, not strict TLS policy. |
| `model/src/peer_client.cpp`, `ApiGet` / `ApiPost` | Both peer and hostname verification disabled | Local peer trust needs separate treatment. |
| `model/src/certificates.cpp` and peer pairing | Self-signed server certificate and PIN-derived application keys | Enabling public PKI checks blindly would break existing pairing; a PIN alone must not be described as verified server identity. |

These findings concern source behavior. No intercepted traffic or exploit is
claimed. The stock installation, boat and pristine upstream have not changed.

## Proposed public-download policy

1. Apply a small documented patch only to the disposable integration source.
   Preserve the pristine baseline and original installation hashes.
2. Explicitly require certificate-chain and hostname verification. Check option
   setup failures and propagate them; never retry with verification disabled.
3. On Windows, qualify the maintained curl/OpenSSL combination from SCRUM-209
   with Windows native trust (`CURLSSLOPT_NATIVE_CA`). curl 8.22.0's
   `lib/vtls/openssl.c` loads Windows ROOT and CA stores for this option. Do not
   embed a build-machine CA path or discover trust files from the working
   directory. Existing system trust remains the candidate policy on Linux.
4. Review initial URL and redirect schemes per caller. Public plugin/catalog
   downloads require authenticated transport; HTTPS must not downgrade to HTTP
   or redirect into local files. Keep this decision separate from WXCURL's
   existing FTP/Telnet support rather than disabling library protocols globally.
5. A failed transfer must not enter the plugin archive-installation path. Close
   and discard the owned temporary partial download while retaining a useful
   error for the user. Preserve pre-existing installed plugins.
6. Keep local peer pairing outside this first patch. Its server-authentication
   gap remains open in SCRUM-211 until a compatible reviewed trust mechanism is
   implemented and tested. Do not count unchanged bypasses as qualified TLS.

## Required implementation evidence

Exercise the actual patched `Downloader` and WXCURL call paths with owned
loopback TLS endpoints: trusted valid chain, untrusted issuer, expired
certificate, wrong hostname, valid HTTPS redirect, downgrade/local-file
redirect refusal, missing/unusable trust material, and interrupted response.
Check that rejected connections deliver no accepted payload or plugin install.
Use a test-only CA provisioned in an isolated environment; never change the
developer's or boat's global trust store. Test the native Windows root-store
path separately in disposable CI. Changing the working directory must not
change accepted trust roots.

The fixture harness must not replace the production verification logic with
an independently implemented approximation. Source-string assertions cannot
qualify this behavior. Preserve failed evidence, existing chart/plugin
functionality, Linux/native Windows gates, installer recovery and exact-source
mapping. Native trust behavior and local-peer remediation remain unaccepted.

## First vertical slice

`patches/opencpn-5.12.4-download-trust.patch` changes the disposable
integration source only. `Downloader::download` and `get_filesize` now share a
checked public-HTTPS setup: chain verification, hostname verification, HTTPS
initial and redirect protocols, bounded redirects, HTTP error handling, and
checked initialization and option results. Windows requests
`CURLSSLOPT_NATIVE_CA`; this requires the maintained curl/OpenSSL build from
SCRUM-209 and still needs native Windows qualification. Linux uses libcurl's
configured system trust. Neither path searches for a CA bundle in the working
directory.

File downloads use an owned sibling staging file. Only a complete successful
transfer replaces the destination; failed or interrupted staging files are
removed. `PluginHandler::InstallPlugin(PluginMetadata)` now checks that result,
records the Downloader error, removes its owned temporary path and returns
before archive extraction. The separate local-file overload is unchanged.

`tools/test-downloader-trust.py` compiles the actual patched Downloader and
uses its GET and HEAD paths against owned loopback TLS servers. Its test-only
compile definition selects an explicit temporary CA file; production builds do
not contain that override. The retained pinned-source baseline accepted an
owned self-signed localhost certificate for both GET and HEAD while the system
curl rejected the same certificate with error 60. The isolated patched Linux
proof accepts a valid chain and
valid HTTPS redirect and rejects untrusted, expired, wrong-host, downgrade,
initial-HTTP, missing-trust and partial-response cases. It also proves failed
file transfers preserve the pre-existing destination. This is Linux transport
evidence, not native Windows, full PluginHandler extraction, installer or
release acceptance.

WXCURL and local peer TLS remain unchanged and open under SCRUM-211. This slice
must not be described as complete application TLS remediation.

Review follow-up also checks rejection on both GET and HEAD, catches stream
exceptions at the libcurl callback boundary, reports final flush failures, and
cleans failed staging-path allocations. The real harness includes a refusing
output stream with exceptions enabled; no exception escapes the callback.
