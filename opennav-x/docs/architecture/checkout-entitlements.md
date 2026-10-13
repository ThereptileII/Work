# Beta checkout and entitlements

The beta commerce boundary is deliberately closed to Stripe **TEST** mode. A
server-side flag, a `sk_test_` secret, a `whsec_` webhook secret, one `price_`
ID, and an exact HTTPS return origin are required before either route exists.
No payment secret or service-role credential belongs in the browser. The Edge
router invokes `handleCommerce(request, context)` for
`POST /api/checkout/start` and `POST /api/stripe/webhook`; it enforces its
normal exact-origin and double-submit CSRF checks for the checkout start route.
The webhook instead authenticates Stripe's timestamped HMAC over bounded raw
bytes. The commerce module also independently checks the start route's exact
Origin. Only `POST /api/checkout/start` requires a signed-in Supabase subject.

The context has `env`, `getAccount(request)`, `rpc(name, args)`, and optional
`fetch`. `getAccount` must return the authenticated Supabase `auth.users.id`,
not a caller-provided account ID. `rpc` must use the server's service-role
client. The browser receives `{ "url": "https://checkout.stripe.com/..." }`
and follows the hosted checkout URL. Return pages do not grant access or
infer completion from query parameters.

Configuration keys are `STRIPE_TEST_CHECKOUT_ENABLED=true`,
`STRIPE_TEST_SECRET_KEY`, `STRIPE_TEST_WEBHOOK_SECRET`, `STRIPE_TEST_PRICE_ID`,
`STRIPE_TEST_RETURN_ORIGIN`, and a comma-separated
`STRIPE_TEST_ALLOWED_COUPON_IDS`. The configured price must be the one-time
USD 20.00 test price. The origin must be the exact beta web origin, such as
`https://beta.skager.app`, without a trailing slash. Stripe's webhook endpoint
must use the pinned API version in `commerce.ts` and deliver the five event
types admitted there. A coupon ID on the allowlist is still subject to the
provider's eligibility rules; the verified session must show its actual
discount before fulfillment. Because hosted promotion-code entry cannot limit
what a customer types to this local allowlist, an unapproved Stripe coupon can
create a charge that this boundary refuses to fulfill. Configure the Stripe
test account so customer-facing promotion codes map only to approved coupons,
and reconcile rejected sessions before offering this flow publicly.

`commerce.sql` creates private orders, immutable session bindings, webhook
receipts, settlements, refund markers, entitlements, and append-only effects
under `skager_private`. Its only PostgREST entry points are fixed-search-path
`public.commerce_*` security-definer RPCs granted to `service_role`. The
private tables have RLS enabled and no client policy. Apply the migration in
one transaction as the project owner; do not add `skager_private` to exposed
PostgREST schemas. The authenticated portal calls
`commerce_portal_access(p_account_id)` through the server after subject
validation. It returns `{ entitlement, orders }`, where `entitlement.status`
is `active`, `refunded`, or `suspended`, and each order reports its provider
TEST amount in minor USD units.

An approved 100% coupon produces a verified `no_payment_required` completed
session with total zero and no payment intent. It creates the **same `active`
entitlement** as a fully paid TEST checkout. Its settlement and effect record
the discount, the zero TEST total, and `real_funds_received_minor: 0`. A
refund of any amount conservatively revokes a paid order's entitlement. A
refund received before completion leaves a marker and makes the eventual
entitlement terminal `refunded` from its first version. Duplicate webhook
IDs with identical evidence are idempotent; a changed digest conflicts.
Expiration or failure events cannot reopen a terminal entitlement.

The Node tests run with `node --test web/edge/commerce.test.ts` from the
`opennav-x` directory. `commerce.postgres.sql` contains rollback-only
assertions for a disposable PostgreSQL database after the migration. It needs
`auth.users(id uuid primary key)` and the `anon`, `authenticated`, and
`service_role` roles. The focused tests use mocked provider responses and do
not qualify a real Stripe/Supabase connection, webhook delivery, deployment,
or the beta domain. Keep the feature flag off until those live checks and the
coupon configuration have been verified.
