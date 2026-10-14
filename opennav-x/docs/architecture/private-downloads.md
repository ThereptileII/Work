# Private staging downloads — SCRUM-42

This is the Supabase Edge backend contract, not the retained Django application.
The default release catalog is empty and `SKAGER_DOWNLOADS_ENABLED` is disabled.
The private R2 bucket is created as recorded below; no package upload, credential,
qualified catalog entry or public download follows from that setup. Configuration
is not qualification.

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
The same current subject and entitlement are checked again after local signing;
logout, refund, suspension, account changes or uncertainty withhold the bearer.
Only a valid selected signing request consumes the existing atomic
`skager_auth_throttle` quota: `download:account:{verifiedSubject}`, ten requests
per sixty seconds. Only boolean `true` authorizes Storage; `false` returns 429,
and malformed results or database errors return 503 without signing.
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

The private bucket is `skager-private-releases` on the source-pinned Cloudflare R2
account. Supabase remains the identity and entitlement authority. Object keys are
derived only from reviewed metadata:

`staging/{sourceCommit}/{releaseId}/{artifactId}/{sha256}/{filename}`.

Before adding a real catalog entry, independently establish the exact candidate's
qualification, corresponding-source/license completeness, supported platform and
artifact identities, then authenticate uploads against those hashes/lengths and
verify private bucket and read/sign permissions. Preserve immutable objects:
hash-named keys alone do not prevent a privileged overwrite. The handler does not
download multi-gigabyte objects to rehash them on each request, and neither the
qualification record fields nor successful signing prove stored bytes or release
acceptance. Historical entries retain their own original commit/version/hashes.

## Observed deployment capacity — 2026-10-13

The actual project's Storage Settings show Supabase Free with a fixed 50 MB
per-object upload limit, consistent with the [provider file-limit documentation](https://supabase.com/docs/guides/storage/uploads/file-limits).
Required Windows recovery and corresponding-source archives exceed that limit.
The implementation's bounded metadata size is not a promise that the provider
plan can store that object. No real catalog entry or bucket has been enabled.

At the 2026-10-13 checkpoint the existing Cloudflare account's R2 page was still at subscription activation.
R2 Standard offers 10 GB-month storage, one million Class A and ten million
Class B operations free per month; overage is usage-billed under its
[pricing policy](https://developers.cloudflare.com/r2/pricing/).
Activation requires owner acceptance of the recurring terms/billing step.
No R2 subscription, bucket, binding or upload had been created by the operator at that checkpoint.
Provider selection/adaptation remains in SCRUM-42; do not split archives into
opaque chunks or expose public objects to work around this capacity blocker.
Supabase identity, database and payment boundaries remain unchanged.

### Owner activation and private bucket — 2026-10-14

The owner activated R2. Dashboard readback confirms the existing account now
has R2, and the operator created `skager-private-releases` with Standard storage,
the default Eastern Europe placement and **Public Access: Disabled**. The bucket
contains no objects. No public development URL, custom domain or public access
was enabled. This removes the subscription/capacity setup blocker, not the
separate credential, provider qualification or software release gates.

A Cloudflare account-token form is prepared with **Object Read only**, restricted
to this bucket. The owner must complete creation and enter its Access Key ID and
Secret Access Key directly into Supabase Edge Secrets as `R2_ACCESS_KEY_ID` and
`R2_SECRET_ACCESS_KEY`. The portal runtime must never receive write/delete/admin
rights. Upload custody needs a separate authorized deployment route. No token
has been created or read by the operator, and no credentials are in source.

The R2 signing adapter is being implemented under SCRUM-42; the currently deployed
frontend/backend have not been replaced by that work. Downloads remain disabled
and the reviewed catalog remains empty. A private empty bucket is not proof of
authenticated delivery, signed-link expiry or exact artifact integrity.

## R2 signing and limits

The R2 adapter supersedes the earlier Supabase Storage signer because the actual
Free project's per-object capacity cannot accommodate the release set. There is
no automatic provider fallback. [Cloudflare documents S3 presigned URLs](https://developers.cloudflare.com/r2/api/s3/presigned-urls/)
for temporary access to private objects; these use the S3 endpoint, not a public
bucket or custom domain.

The server requires `SKAGER_DOWNLOAD_BACKEND=r2`, the exact `R2_ACCOUNT_ID` and
`R2_DOWNLOAD_BUCKET` pins, plus `R2_ACCESS_KEY_ID` and `R2_SECRET_ACCESS_KEY` in
Supabase Edge Secrets. Only the single-bucket **Object Read only** credential
belongs in this runtime. Upload access must be separate. Missing or mismatched
configuration refuses signing; no caller may override the account, bucket,
region, object key, filename or method.

The adapter uses byte-identical `aws4fetch@1.0.20` ESM distribution, vendored with
its MIT license, upstream revision, official npm integrity and file hashes in
`web/edge/vendor/aws4fetch-1.0.20/manifest.json`. Packaging verifies those files,
bundles the dependency only into the server artifact and retains the full MIT
notice and dependency provenance. No runtime dependency download is required.

`AwsClient.sign()` signs a GET locally with S3 region `auto`, a fixed attachment
filename and a 60-second expiry. It makes no network signing request and never
signs an upload, delete or alternate method. The output is checked against the
exact HTTPS origin, object path, allowed SigV4 parameters, credential scope,
issued time and expiry; unexpected headers, body, URL aliases or parameters
refuse. The overall signing call has an eight-second deadline, and the same
session and entitlement must still be valid before the link is returned.

Exceptions, secret values and signed URLs are never logged. The portal validates
the matching artifact/path/expiry before navigation, retains no-referrer policy,
and obtains a fresh authorized POST for each click. Replies are private/no-store.
A signed URL is a transferable bearer until its short expiry: revoking access
stops new links but cannot revoke an already downloaded file or necessarily stop
a transfer in progress. A client clock substantially different from the server
can cause the portal to refuse an otherwise valid link. R2's actual signature,
expiry, private access and stored-byte behavior still need live qualification.

## Integration and remaining acceptance

The integrated router imports `handleDownloads`, rejects queries before URL
normalization and routes both methods after standard CSRF processing. The portal
loads the dedicated authorized catalog on Downloads instead of relying on a
hardcoded dashboard availability claim. It safely renders version/channel/commit
and hash/length metadata, requires a fresh authorized POST for each click, checks
the pinned returned URL and expiry, and handles OFF/empty/unauthorized states.
The R2 source changes are not yet deployed or provider-qualified.

This increment's tests use synthetic catalog entries and credentials, independent SigV4 checks and controlled SDK faults.
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
