# SCRUM-42 private R2 download checkpoint — 2026-10-14

Exact web source `1acc781ed1ba50d565201911c15cdab00c24a364` passes
[web CI 38512040319](https://github.com/ThereptileII/Work/actions/runs/38512040319):
88 Django/PostgreSQL and 94 Edge tests (**182 total**, no failures or skips),
plus disposable PostgreSQL assertions, migration checks and reversals/reapplication.
The downloaded original artifact matches the provider length and SHA-256; all
10 ZIP members pass CRC validation. `ci.json` records exact artifact identity.
These checks use mocked provider calls and do not prove live account/payment or
R2 download behavior.

The retained frontend and backend were packaged from that exact clean web source.
`build-info.json` records both output hashes and the byte-identical pinned MIT
`aws4fetch` dependency. The independent known-signature test binds the GET path,
60-second expiry and attachment filename. Portal authorization, current active
entitlement, CSRF, rate limit and post-sign entitlement recheck remain required.
Invalid account/bucket configuration fails closed without a provider fallback.

The owner activated R2. Dashboard inspection confirmed the new
`skager-private-releases` bucket is private and empty; `provider-state.json`
records that separate provider observation. The owner has since confirmed saving
the R2 credentials directly in Supabase Edge Secrets, and
their binding names are present in the provider list. Configuration presence does
not establish valid R2 authentication. No credential value is stored in this
evidence, Git or client code.

## Provider deployment and owner credential handoff

Both exact bundles are now deployed. Cloudflare reports version `33f77360`
**Active / Latest**, replacing `e18a9fb4`, for the existing `beta.skager.app`
Worker. The complete 503100-byte editor contents matched the retained frontend
SHA-256 before Deploy. Supabase reports deployment count 13, and a fresh Code
page readback after activation matched all 77857 bytes of the retained server
bundle. `deployment.json` records the exact output hashes and observation limits.

The owner confirmed saving both R2 credentials in Supabase Edge Secrets; their
names are present in the provider list. The three nonsecret backend/account/
bucket bindings were saved and their names read back. No credential values are
stored in repository evidence. This confirms configuration presence, not valid
R2 authentication or an authorized download.

Live direct-backend checks returned health 200 with `ready:false` and downloads
404 with `Downloads unavailable.` An earlier Python urllib health request on the
public beta origin returned text HTTP 403 with error 1010 rather than the
application's health JSON. That failure remains recorded separately in
`deployment.json`. Chrome UI preview reported `ERR_BLOCKED_BY_CLIENT`, a distinct
client-side observation without an observed HTTP response.

One later ordinary curl 8.22.0 GET, using its unmodified default User-Agent and
following no redirects, returned HTTP/2 **200** at
**2026-10-14 07:27:03 UTC**, with the expected private-beta health JSON and
`ready:false`. Cloudflare Ray ID was `a4a4dbfff978bd27-ARN`.
`curl-health.json` retains the sanitized method, URL, client, timestamp, selected
response headers and exact body. This confirms public health reachability for
that request. It does not establish the cause of the earlier client/time-specific
failure or successful email, payment, R2 authentication, signing or portal
behavior. No provider security settings were changed.

The qualified desktop release catalog remains empty and downloads stay OFF.
Pending gates include authorized object upload and integrity/custody checks,
actual browser/customer-journey access, live entitlement/expiry/cross-account
tests, and independent desktop package qualification. Public account/payment/download
access remains closed; this checkpoint neither qualifies the Windows application
nor authorizes public release.
