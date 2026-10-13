# Private staging downloads — SCRUM-42

This is the Supabase Edge backend contract, not the retained Django application.
The default release catalog is empty and `SKAGER_DOWNLOADS_ENABLED` is disabled.
No package, qualification decision, bucket, provider configuration, credential or
public download is created by this increment. Configuration is not qualification.

## Requests and authorization

`handleDownloads(request, ctx)` uses the existing `Context` account lookup,
transport and `commerce_portal_access({p_account_id: verifiedSubject})`. Both
`SKAGER_DOWNLOADS_ENABLED=true` and `PORTAL_ENABLED=true`, with
`STRIPE_TEST_ENVIRONMENT=staging`, are required; otherwise the route returns 404.
Every listing and mint request checks the current authenticated session and exact
`active` entitlement for product `skager-beta`, environment `staging`. An order,
redirect, amount or caller-supplied account never grants access. A verified 100%
coupon's active entitlement has the same access as paid TEST fulfillment.

`GET /api/portal/downloads` returns
`{available, reason, releases:[{id,version,sourceCommit,channel,artifacts:[{id,name,bytes,sha256}]}]}`.
An active account with the empty catalog receives 200, `available:false` and an
honest qualification-pending reason. There are no signed links in this response.
Anonymous/invalid sessions receive 401, inactive or other-scope entitlements 403,
and account/database uncertainty 503. No authenticated state is cached.

`POST /api/portal/downloads` accepts only JSON `{releaseId,artifactId}`. It rejects
unknown fields, duplicate JSON keys, arbitrary URLs, paths or accounts, queries,
invalid UTF-8 and bodies larger than 1 KiB. Reading the selection has a one-second
deadline. Exact beta Origin and double-submit CSRF are checked in the module as
well as the router. An unknown server selection is 404. A successful response is
`{url,expiresAt,releaseId,artifactId,name,bytes,sha256}`, with an ISO expiry time.
The same current subject and entitlement are checked again after Storage returns;
logout, refund, suspension, account changes or uncertainty withhold the bearer.
Only a valid selected signing request consumes the existing atomic
`skager_auth_throttle` quota: `download:account:{verifiedSubject}`, ten requests
per sixty seconds. Only boolean `true` authorizes Storage; `false` returns 429,
and malformed results or database errors return 503 without a Storage call.
Listings, unknown selections and rejected access do not consume this signing
quota. No SQL is changed. The separate key cannot consume auth send/verify
counters, but the existing function shares its table, expiry cleanup and advisory
lock with authentication. Signing traffic can contend with auth; actual database
concurrency/load qualification remains outstanding.

## Reviewed source catalog and object custody

`createDownloadHandler(reviewedCatalog)` snapshots and freezes a trusted server
source catalog; requests and environment JSON cannot add entries. A maximum of 16
current/historical staging entries is supported. Each entry has a unique bounded
ID, version, exact 40-hex source commit and channel, plus a repository qualification
record path and its SHA-256. All five artifact kinds are required: `installer`,
`recovery`, `checksums`, `source` and `release-notes`. Each has a safe bounded
filename, positive bounded byte length and SHA-256. Production entries are not
admitted by this private staging handler.

The fixed proposed private bucket is `skager-private-releases`, on the already
pinned `kelwgubkuvditalsbqpr.supabase.co` project. No bucket is created here. Object
keys are derived only from reviewed metadata:

`staging/{sourceCommit}/{releaseId}/{artifactId}/{sha256}/{filename}`.

Before adding a real catalog entry, independently establish the exact candidate's
qualification, corresponding-source/license completeness, supported platform and
artifact identities, then authenticate uploads against those hashes/lengths and
verify private bucket/RLS and read/sign permissions. Preserve immutable objects:
hash-named keys alone do not prevent a privileged overwrite. The handler does not
download multi-gigabyte objects to rehash them on each request, and neither the
qualification record fields nor successful signing prove stored bytes or release
acceptance. Historical entries retain their own original commit/version/hashes.

## Storage signing and limits

Supabase documents [time-limited private downloads](https://supabase.com/docs/guides/storage/serving/downloads)
and [`createSignedUrl(path, expiresIn)` in seconds](https://supabase.com/docs/reference/javascript/storage-from-createsignedurl).
The reviewed server implementation at
[`supabase/storage@27c5b7eb`](https://github.com/supabase/storage/blob/27c5b7ebebe08959be67be291b0fd7384529355c/src/http/routes/object/getSignedURL.ts)
defines `POST /storage/v1/object/sign/{bucket}/{object}` with
`{expiresIn:60}`, returning `{signedURL:"/object/sign/...?...token=..."}`.
Its [object signer](https://github.com/supabase/storage/blob/27c5b7ebebe08959be67be291b0fd7384529355c/src/storage/object.ts)
binds the token to bucket/object; its
[JWT signer](https://github.com/supabase/storage/blob/27c5b7ebebe08959be67be291b0fd7384529355c/src/internal/auth/jwt.ts)
adds issued-at and expiry. These source references establish the API contract,
not the live project's deployed Storage revision or actual delivery acceptance.

Only the pinned HTTPS project is called, with the server-only service binding.
The call uses manual redirects and requires HTTP 200 without redirection or URL
substitution. JSON has a 16 KiB bound, strict decoding and duplicate-key refusal;
encoded responses and inconsistent content lengths are refused. The request and
response-body read share an eight-second deadline. Exceptions, provider bodies,
credentials, subjects and tokens are never logged or included in refusal replies.

The returned URL must be the exact relative signing path for the selected object,
with a single token and no fragment, alternate origin, path alias or extra query.
Its bounded JWT must name the exact bucket/object, carry issued-at and expiry,
and have download scope when a scope is present. Legacy scope-less download
tokens remain compatible with the documented API. Expiry is at most 60 seconds
after issuance, with at most five seconds of provider clock skew. Claims are
constraints on an authenticated HTTPS Storage response; parsing them is not
independent JWT signature verification. Storage authenticates its signature when
the bearer is redeemed. The server appends the reviewed filename using Supabase's
documented `download` parameter; caller filenames are not accepted.

Responses use private/no-store/no-cache and no-referrer safeguards. The portal
must retain its no-referrer document policy and use a fresh POST for every click;
do not persist, log, prefetch or share signed URLs. A signed URL is a transferable
bearer until expiry: a completed entitlement revocation stops new URLs, while an
already issued URL can remain usable for the short remaining lifetime (including
the bounded clock allowance). A transfer already in progress is not interrupted
by a later refund, and already downloaded files are not revoked. Supabase notes
that Auth-key rotation does not revoke these Storage URLs.

## Integration and remaining acceptance

Root owns router/frontend glue: import `handleDownloads`, reject queries before
URL normalization, route both methods after standard CSRF processing, and load
the dedicated authorized catalog on the Downloads page. Remove the old dashboard
hardcoded availability claim. Render version/channel/commit and hash/length
metadata safely; require a fresh authorized POST for each download, validate the
pinned returned URL and expiry, and handle OFF/empty/unauthorized states honestly.

This increment's tests use synthetic catalog entries, tokens and Storage replies.
They do not qualify a package or real signed URL. Actual private Storage setup,
object custody, native release qualification, download/expiry/refund behavior,
privacy/egress handling and independent combined integration review remain gates.
Before enabling a nonempty catalog, verify actual Storage URL expiry, attachment
headers and private/CDN cache behavior against immutable uploaded bytes. The
API's no-store response headers do not establish the artifact server's cache
policy, and synthetic mocks establish neither live TTL enforcement nor object
immutability.
Auth/checkout/portal/download access must remain off until separately authorized.
Public download/catalog publication still requires the owner's explicit GO.

Portal entitlement controls access to this service; it does not revoke GPL rights
to software already distributed. Preserve recipients' corresponding-source access,
notices and redistribution rights independently of refunds or portal membership.
An active-only source link is not a substitute for the release's source-delivery
obligations; establish the approved durable source-delivery mechanism before
distribution. This handler creates no new source offer or legal acceptance.
