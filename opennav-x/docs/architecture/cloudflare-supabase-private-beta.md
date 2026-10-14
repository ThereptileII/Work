# ADR: Cloudflare frontend and Supabase private-beta backend

Date: 2026-10-13. Decision: owner-selected implementation direction for the
invited Staging website. Source reviewed: `b730d2364430de9e706798041a2da48c544a44db`.
This records architecture and verified source checks, not a deployed-service or
public-launch acceptance decision. Jira remains the sole backlog.

## Decision and supersession

Use a Cloudflare Free Worker for the exported frontend at `beta.skager.app`,
with one Supabase Edge Function, `skager-web`, providing the account, commerce
and portal API. Supabase Auth supplies email proof and canonical identities;
Supabase PostgreSQL stores private application state. Stripe Checkout remains
restricted to its test environment.

On 2026-10-14 the owner activated Cloudflare R2 for private release files. The
Supabase Free project's 50 MB object limit cannot hold the Windows release set.
R2 therefore replaces only download-object storage: Supabase remains the auth,
database, commerce and entitlement backend. The newly created bucket is private
and empty. A server-only R2 signer and portal URL checks are implemented, with
credentials, deployed behavior and real artifact qualification still pending.
See [private downloads](private-downloads.md); this change enables no public access.

This replaces the Django/Render runtime selected in
[public-beta web and commerce](public-beta-web-commerce.md) and the Django
runtime retained in the earlier [Supabase/Stripe decision](supabase-stripe-decision.md).
The owner declined a paid Cloudflare Containers backend. Django templates and
the fixed informational-page exporter remain build inputs, and existing Django
models, migrations, tests and historical CI evidence remain intact. They are
not the deployed authentication, authorization or settlement boundary for this
Edge implementation. The earlier owner-private Sites preview remains historical
preview evidence, not proof of this deployment.

The selected Free frontend plan is a deployment choice, not a claim that every
provider, email service, backup, usage level or future operation costs zero.
No paid upgrade or live-payment enablement follows from this decision.

## Runtime and trust boundaries

| Boundary | Responsibility and limit |
| --- | --- |
| Browser and exported assets | Informational marketing, OTP entry and portal shells. No provider access/refresh tokens, service-role keys, Stripe secrets or database credentials. A visible shell is not authorization. |
| Cloudflare Worker | Serves the fixed public asset bundle only on the exact beta origin and proxies `/api/*` to the pinned Supabase function. It bounds uploads and forwards an explicit header allowlist; it does not forward caller-supplied provider authorization or IP/proxy metadata. |
| Supabase Edge Function | Owns authentication, account-scoped portal access, checkout creation and verified webhook processing. It also bounds direct requests and applies application authorization itself; the Worker is not assumed to be the only caller. |
| Supabase Auth | An allowlisted server-side invitation sends a code without auto-confirming identity. `/verify` and canonical `/user` proof precede every application login. Public signups stay disabled and email confirmation stays required. |
| Supabase PostgreSQL | `skager_private` contains sessions, challenges, limits, orders, settlements, effects, entitlements and boat names. The schema stays outside exposed Data API schemas, with no `anon`/`authenticated` access. RLS and fixed-search-path SECURITY DEFINER RPCs restricted to `service_role` enforce the server boundary. |
| Stripe test Checkout/webhooks | Hosts the test payment form. Server-selected price and immutable order/session binding constrain fulfillment. Bounded raw-body signature verification, provider re-fetch and transactional SQL determine effects. Browser redirects never grant access. |
| Cloudflare R2 | Private, hash-named qualified release objects. The Supabase handler checks the current session and entitlement before signing a 60-second GET and again before returning it. Its credential is read-only for the single release bucket; upload custody is separate. |

The implementation lives in `web/edge/`: `cloudflare-worker.mjs` is the frontend
router, `index.ts` and `platform.ts` compose the backend, and `auth`, `commerce`
and `portal` modules own their respective handlers and SQL. Provider targets
are pinned to the selected project and approved HTTPS endpoints. Secrets belong
only in backend platform bindings; no environment values are read into the
public build.

The server's application-owned auth bindings are `SKAGER_AUTH_ENABLED` and
`SKAGER_INVITE_EMAILS`. The flag must be exactly `true`, and the allowlist must
contain the intended exact email addresses; missing configuration denies access.
Supabase reserves the `SUPABASE_` prefix for injected variables, so the old custom
auth names are ignored. Keep the vendor-injected `SUPABASE_URL`,
`SUPABASE_ANON_KEY` and `SUPABASE_SERVICE_ROLE_KEY` unchanged and server-only.
See [the current configuration contract](supabase-login-flow.md#server-configuration-and-routing).

`web/tools/build_cloudflare_beta.py` requires an exact clean web source revision
and a fresh output directory. It produces separate frontend and backend bundles,
plus SHA-256 provenance in `build-info.json`. A successful build does not upload
or activate either bundle. Deployment must bind those exact outputs to the
recorded source and keep server-only code out of public assets.

## Identity, session and request policy

[The login contract](supabase-login-flow.md) defines fixed login/signup/recovery
routes, generic send responses, bounded strict provider JSON and fresh canonical
identity verification. Application identity is the Supabase subject UUID; email
cannot join another account or its purchases. Exact friend-email allowlisting
applies at invitation, verification and session lookup. Disabled auth or an empty
allowlist denies access.

The browser receives a random host-only HttpOnly Secure SameSite=Lax session
cookie. Only its hash, subject and absolute twenty-minute expiry are stored;
Supabase tokens are neither persisted nor returned to the browser. Logout
revokes the record. User deletion/bans and allowlist removal are checked during
session lookup. Provider global signout alone does not revoke an independent
application session; its remaining validity is bounded by that absolute expiry.

Mutations require exact `https://beta.skager.app` Origin plus the matching
readable host-only CSRF cookie and `X-CSRF-Token`. The Stripe webhook instead
requires its provider signature. Challenges are browser-bound and consumed
atomically once. Database limits apply by email and globally; an IP limit is
used only if authenticated trusted ingress metadata is actually supplied. The
current router supplies no IP, so no forwarded header is treated as evidence.
API responses are not cached, and upstream exception bodies are not exposed.

## Test offer, ownership and release gates

The Staging offer is USD 20 one-time **test-mode** beta-services access. A
verified full-price TEST checkout and an approved 100% coupon checkout create
the same `active` entitlement for product `skager-beta`, environment `staging`.
The zero-total path requires `no_payment_required`, the approved bound coupon,
and no payment intent. Both paths record zero real funds received. No return
URL, query parameter, email, pending order or displayed amount grants access.
[Settlement and refund rules](checkout-entitlements.md) retain durable event
deduplication, immutable bindings, atomic effects and terminal refund behavior.

The portal derives account IDs only from the authenticated session and scopes
orders and boat names to that owner. Current boat storage is a name record;
cloud telemetry, health, alerts, voyages and remote control are not delivered.
The private download path now has a separate, default-off handler and an empty
reviewed release catalog. Every catalog read and link request requires a current
session and active Staging entitlement; a link request also requires Origin/CSRF.
The portal fetches availability from this handler instead of assuming access from
the dashboard. Links are minted only after a user selects a qualified artifact,
expire after sixty seconds, and are not embedded in catalog HTML. Query parameters
are rejected at both Worker and Edge boundaries before routing can discard them.
The owner-activated R2 bucket is private and empty; no release object or catalog
entry is provisioned by the runtime implementation.
Downloads therefore remain unavailable until exact package qualification and
private Storage custody are independently completed. See the
[private download contract](private-downloads.md). Public distribution still
requires the separate human GO decision.
An active Staging entitlement is not a public distribution or payment approval,
and does not alter installed GPL software or corresponding-source rights.

## Evidence and remaining qualification

[Web CI 38383036021](https://github.com/ThereptileII/Work/actions/runs/38383036021)
passes on exact source `b730d2364430de9e706798041a2da48c544a44db`:
88 preserved Django/PostgreSQL tests and 39 Edge contract tests, **127 total**,
plus real PostgreSQL checks from `commerce.postgres.sql` and
`security.postgres.sql` after applying the auth/commerce/portal SQL in a disposable
database. Django checks and fresh/reverse/reapply migrations also pass. The CI
uses mocked provider HTTP for Edge tests; it does not prove live provider flows.

At this documentation checkpoint, reviewed SQL deployment, function/Worker
publication and domain routing, backend secret configuration, both email-code
templates and real delivery, invited login/recovery/logout, actual Stripe test
checkout/coupon/webhook/refund and cross-account portal behavior on the deployed
origin remain pending. Backup/restore, incident operations, legal/privacy and
public-release readiness remain separate gates. No provider deployment or live
end-to-end result is claimed here. Public signup, live payments and downloads
remain closed; the owner has not given public-launch GO.
