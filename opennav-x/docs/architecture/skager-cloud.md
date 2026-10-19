# SKAGER Cloud: account link, automatic updates and boat data

SCRUM-44 (paid access to the update channel) and SCRUM-369 (My Boat telemetry),
under epic SCRUM-13. Owner decisions of 2026-10-19:

- The app logs in with a **pairing code**: SKAGER shows an 8-character code, the
  customer approves it at `beta.skager.app/link/` while signed in, and the app
  receives a revocable device token. No password is typed at the helm.
- **Automatic updates are a SKAGER Cloud service.** Beta Access buyers keep every
  update as a manual download from the portal.
- **Boat data**: every `VesselState` data item with its age, quality and source.
  The latest snapshot is sent every 15 s; one sample a minute is kept for 30 days.
  Sharing is off until the customer switches it on in SKAGER.

Cloud is optional and never DRM. Without an account, without Cloud or without a
network, SKAGER starts and navigates exactly as before. The launcher then makes
no update offer. The installed version keeps working, and the open-source
licences are unaffected.

## Components

| Part | Where | Role |
|---|---|---|
| Pairing, tokens, telemetry storage | `web/edge/cloud.sql` | Private tables. Only `service_role` may execute the RPCs. Codes and tokens are stored as SHA-256 only. |
| Device and customer API | `web/edge/cloud.ts` | `/api/device/*` for the app. These routes accept no cookies and no CSRF, refuse any browser `Origin`, and take a bearer token. `/api/cloud/*` serves the signed-in customer (cookie and CSRF). |
| Update feed gate | `web/edge/cloudflare-worker.mjs` | `/updates/<channel>/…` streams from the `SKAGER_UPDATES` R2 binding, only for a device token whose account has active Cloud. Only grants are cached (60 s). |
| Launcher | `tools/update-verifier` | Reads the token from Credential Manager (`SKAGER/Cloud/v1`). Without a token it makes no update check. With one, it sends the token as a bearer to the exact metadata and artifact origins. |
| App | `src/cloud`, `src/integration/OnlineCloud.*`, `src/ui/CloudPage.cpp` | Worker thread for pairing, status and upload. The settings page is **Settings → System → SKAGER Cloud**. |
| Portal | `web/edge/static/portal.js` | `/link/` approval page. The Cloud tab lists linked computers and shows the latest readings and 24 h history. |

The token never changes what is trusted. TUF metadata signatures and the signed
artifact length and SHA-256 still decide that. The token only decides who may
download.

## Pairing

1. App: `POST /api/device/link {name, platform, version}` returns the device code
   (secret, kept in the app), the user code, the verification URL, `expiresIn` (600 s)
   and `interval` (5 s).
2. Customer: `/link/`, then `POST /api/cloud/link/find {code}` shows the computer,
   then `POST /api/cloud/link/decide {code, approve}`.
3. App: `POST /api/device/link/poll {deviceCode}` returns `pending`, `approved`
   (the token, exactly once), `denied` or `expired`.

Tokens are `skg1_` followed by 64 lowercase hex characters. The Worker sends the
app's `Authorization: Bearer` to the function as `X-Skager-Device`, so the
Supabase gateway never sees a non-JWT Authorization header. A token stops working
at once in any of these cases: the customer unlinks it (portal or app), the
account is closed or refunded (trigger on `store_customers`), or the auth user is
deleted or banned.

## Cloud status

Cloud is active while the Stripe subscription is `active` or `trialing`, or is
`past_due` within 7 days of the period end, so updating the card is enough.
`/api/device/status` reports `services.updates` and `services.telemetry`.

## Boat data

The keys and value ranges are identical in `src/cloud/CloudProtocol.cpp`
(`TelemetryKeys`) and `web/edge/cloud.ts` (`TELEMETRY_KEYS`). The server refuses
an upload as a whole if it contains an unknown key or an out-of-range value. The
app therefore leaves out values it cannot send rather than sending zeros.

The app never sends simulated or replayed state, values without a source, or
anything older than a day. Missing data is never sent as zero. The boat clock is
never trusted: observation time is the receive time minus the item's age. The
portal shows every value with its age and labels a snapshot older than 2 minutes
as offline. Uploads are limited to one per 5 s per device. Switching sharing off
deletes the latest snapshot; unlinking deletes all of that computer's data.

## Deployment (not yet done)

Order: SQL first, then the function, then the Worker.

1. Apply `web/edge/cloud.sql` (after `store-guide.sql`), as owner, in one transaction.
2. Redeploy the `skager-web` function (bundle from `build_cloudflare_beta.py`).
3. Worker: add an R2 bucket binding named **`SKAGER_UPDATES`** (bucket name is the
   owner's choice, for example `skager-updates`), then upload the new
   `worker.js`. Without the binding, `/updates/*` answers 503 and nothing else changes.
4. Publish the TUF repository's public tree into that bucket under the channel
   prefix (`beta/…`). The repository and the release's `app/update-trust.json`
   must name `https://beta.skager.app/updates/beta` (metadata) and
   `https://beta.skager.app` (artifact origin). Today's staging trust points at a
   private test host. Moving installed clients to the new origin is a
   trust-configuration change that needs a release, not a server instruction.

## Verification

- `web/edge/cloud.postgres.sql`: SQL assertions (CI applies `cloud.sql` too).
- `web/edge/cloud.test.ts`, `cloud-portal.test.ts`, `cloudflare-worker.test.ts`: routes, validation, gate and rendering.
- `tools/update-verifier/access_test.go`: the token reaches only the configured
  origin, and a signed check plus download against a gated feed works end to end.
  The launcher fixture covers the cases "not linked", "malformed token" and "no provider".
- `tests/cloud_tests.cpp`: wire format and worker lifecycle. It is clean under
  ThreadSanitizer. `cloud_tests emit` prints the app's real payload for contract checks.
- Local end-to-end check (2026-10-19): Worker, router, `rpcClient`, `cloud.ts`,
  real `cloud.sql` in disposable Postgres and the C++ app payload. The flow
  covered pairing, refusal without Cloud, subscribing, the feed, telemetry, the
  portal and revocation.

Still required: a native Windows build and run (Credential Manager read in the
launcher, the settings page at 1280×800), then a boat run with real instruments.
Privacy and security review is also required before production telemetry
(SCRUM-13 principles).
