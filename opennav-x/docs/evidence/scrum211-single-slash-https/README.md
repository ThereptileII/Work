# SCRUM-211: single-slash HTTPS downgrade

At audited base `777481230556dd50c3ffbc48568b1909de727632`, the core and private
wxCurl patches selected HTTPS redirect restrictions only for `https://`.
The actual core wxCurlHTTP accepted libcurl's `https:/` URL spelling and followed
an owned trusted HTTPS endpoint to a reachable owned plain-HTTP endpoint:
GET and HEAD succeeded and GET returned all 36 payload bytes. Conventional
`https://` correctly refused that same destination. This is an executable policy
bypass, not a claim about interception or any remote service.

Both trust patches now match the scheme prefix `https:` (case-insensitively).
[before-after.json](before-after.json) retains actual probe output: the same
single-slash downgrade now refuses GET and HEAD with `Unsupported protocol`
and the detail `Protocol "http" is disabled (in redirect)`.

Focused verification on Linux:

- [wxcurl-linux.json](wxcurl-linux.json): 17 actual core wxCurl cases pass,
  including reachable conventional/single-slash/mixed-case HTTP downgrade
  refusal, valid single-slash HTTPS, all previous certificate cases,
  HTTP-to-HTTPS compatibility, and failed-option refusal.
- [downloader-linux.json](downloader-linux.json): all 12 existing actual
  Downloader cases pass; Downloader already requires conventional HTTPS and
  limits all redirects to HTTPS, and is unchanged.
- Adapter preparation: 17 tests pass. Trust paths: 18 drive/UNC/POSIX and
  6 absent-input checks pass. Updated native PowerShell harness parses.

Both real-source harnesses read the exact pinned pristine OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7` from the primary checkout and copied
only their required source into private temporary directories. wxCurl used the
existing relocated wxWidgets installation through `WX_CONFIG`; no shared build
or source tree was changed. Linux uses its isolated test CA.

The native core/private wxCurl matrix now includes valid single-slash HTTPS
and lower-/mixed-case single-slash downgrade cases. Downgrade checks require
`Unsupported protocol` independently for GET and HEAD, preventing a failed
connection to the disallowed target from falsely passing. These native changes
have only been parsed here; native MSVC runtime and Windows trust cleanup have
not been rerun. No full application build or boat operation was performed.

Historical evidence `docs/evidence/windows-55ef51e/` records passing
12 Downloader + 9 core wxCurl and 12 Downloader + 9 private wxCurl executions
with exact trust cleanup at published revision
`55ef51e4944e8570f6a391dad20b0db8447a7443` (run `37199379614`). Those historical
passes do not cover this new spelling regression or accept this revision.
SCRUM-211 remains open pending exact native core/private trust execution,
actual PluginHandler failure-propagation acceptance, and package/installer
and source-compliance gates. Local-peer trust remains separately SCRUM-212.
