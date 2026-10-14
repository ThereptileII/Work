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
records that separate provider observation. A bucket-scoped read-only credential
form is prepared for the owner to create and enter directly into Supabase Edge
Secrets. No credential value is stored in this evidence, Git or client code.

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
404 with `Downloads unavailable.` The same health request on the public beta
origin returned a Cloudflare text 403/1010, not an application response. The
dashboard preview is also browser-blocked; neither result is counted as proxy or
customer-journey acceptance. Provider activation and direct backend checks do
not establish successful email, payment, R2 signing or portal behavior.

The qualified desktop release catalog remains empty and downloads stay OFF.
Pending gates include authorized object upload and integrity/custody checks,
actual beta-origin access, live entitlement/expiry/cross-account tests, and
independent desktop package qualification. Public account/payment/download
access remains closed; this checkpoint neither qualifies the Windows application
nor authorizes public release.
