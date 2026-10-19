# Private staging customer portal — SCRUM-43

The edge portal serves a public static layout with no embedded account data.
`web/edge/portal.ts` provides the authenticated API; `portal.sql` owns boat
metadata and service-only operations. The root edge router owns secure session
resolution, exact-origin and double-submit CSRF enforcement, security headers,
and deployment. This increment does not open public beta or production access.

## Integration contract

`handlePortal(request, context)` handles `/api/portal/dashboard` (GET) and
`/api/portal/boats` (GET/POST), returning `null` for unrelated routes. Context
provides `env`, `getAccount(request)` and `rpc(name, arguments)`. The portal is
off unless `PORTAL_ENABLED=true` and `STRIPE_TEST_ENVIRONMENT=staging` are both
explicitly set. The fixed product is `skager-beta`. Unknown/expired sessions
return 401 and make no database calls. Private JSON responses use
`Cache-Control: private, no-store, max-age=0`.

The root must reject every state-changing request with an invalid origin or
CSRF token **before** calling a handler. The portal does not duplicate that
router enforcement. `/api/auth/session` bootstraps an opaque cookie session and
returns `{account, csrfToken}`. Browser mutations send `X-CSRF-Token` with
same-origin credentials. No token or session is saved in local/session storage.

Static routing maps `/portal/`, `/portal/account/`, `/portal/orders/`,
`/portal/boats/` and `/portal/downloads/` to `static/portal.html`; login, signup,
recovery and verification routes map to `static/auth.html`. `/portal.css`,
`/portal.js` and `/auth.js` serve their matching static files. Unauthenticated
portal browsers navigate to fixed `/auth/login/`; successful verification goes
to fixed `/portal/`. No request-controlled return URL is used. Auth forms use
POST `/api/auth/login|signup|recover` with `{email}`, verify with `{code}`, and
logout with `{}`. Identity/session implementation belongs to the auth module.

## Ownership and access

Every RPC receives `p_account_id` only from the verified session's Supabase user
UUID. Owner/account fields in boat submissions are rejected. Boat creation
accepts one bounded JSON name, trims it, and checks 1–80 Unicode characters and
no control characters. The SQL constraint independently enforces the boundary.
The static interface escapes all customer values before inserting HTML.

`skager_private.portal_boats` references `auth.users`, enables RLS, and grants no table
access to browser roles or `service_role`. Only the service role can execute
`public.portal_list_boats(uuid)` and `public.portal_create_boat(uuid,text)`.
Both SECURITY DEFINER functions fix `search_path=pg_catalog` and use explicit
private-schema references. There is no ownership-transfer, edit or delete API.
The service key belongs only in the edge runtime. Apply the versioned SQL after
the auth schema setup; deployment, real-role testing and recovery acceptance
remain coordinator-owned gates.

Commerce owns the service-only `commerce_portal_access(p_account_id uuid)` RPC.
It returns only that owner's staging beta entitlement and sanitized orders.
The portal allowlists fields again. Active access requires entitlement status
`active`, environment `staging` and product `skager-beta`; checkout redirects and
order amounts never grant access. Paid and fulfilled zero-total discounted
orders receive identical entitlement access. Billing labels zero-total
fulfillment “Full discount / no payment collected”; other totals are test
checkout totals, never claims of real money received. Provider session IDs,
payment identifiers, secrets and internal database errors are not returned.
Suspended orders remain visible in history but do not grant access.

## Honest availability

Downloads remain closed even for active staging entitlements. No binary key,
signed URL, invented release version or release note is exposed. A later
qualified delivery service must separately authorize and audit actual artifact
delivery. Support and FAQ links work; bug/feature submission, customer report
status, telemetry, connected instruments, boat position and profile editing are
explicitly unavailable. API failures show an error, never demo data.

Test checkout appears only with `STRIPE_TEST_CHECKOUT_ENABLED=true`. Its button
posts to `/api/checkout/start`; hosted Stripe Checkout owns promotion entry.
The browser accepts only an HTTPS URL on `checkout.stripe.com` from that API.

## Focused evidence

Run `node --test web/edge/portal.test.ts` from `opennav-x` with Node 26. Tests
cover session refusal/no RPC, fail-closed environment gating, trusted-owner
arguments and forged ownership rejection, paid/zero access equivalence,
refunded/suspended/foreign-product denial, validation, escaped rendering, empty
states, download closure and generic failure responses. These mock-backed
checks do not claim real Supabase role/RLS, email delivery, checkout or deployment
acceptance. The integrated router must separately verify mutation CSRF and
session expiration with these handlers connected.
