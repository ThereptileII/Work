# Beta checkout and entitlements

The beta commerce boundary is deliberately closed to Stripe **TEST** mode. A
server-side flag, a `rk_test_` restricted key (or existing `sk_test_` secret), a `whsec_` webhook secret, one `price_`
ID, and an exact HTTPS return origin are required before either route exists.
No payment secret or service-role credential belongs in the browser. The Edge
router invokes `handleCommerce(request, context)` for
`POST /api/checkout/start` and `POST /api/stripe/webhook`; it enforces its
normal exact-origin and double-submit CSRF checks for the checkout start route.
The webhook instead authenticates Stripe's timestamped HMAC over bounded raw
bytes. The commerce module also independently checks the start route's exact
Origin. Only `POST /api/checkout/start` requires a signed-in Supabase subject.
The start request may contain JSON `{ "discountCode": "FREE100" }`; an absent
body or empty code uses the full price. Codes contain 1–64 ASCII letters or
digits. An invalid code returns a generic 400 before an order or Checkout
session is created.

The context has `env`, `getAccount(request)`, `rpc(name, args)`, and optional
`fetch`. `getAccount` must return the authenticated Supabase `auth.users.id`,
not a caller-provided account ID. `rpc` must use the server's service-role
client. The browser receives `{ "url": "https://checkout.stripe.com/..." }`
and follows the hosted checkout URL. Return pages do not grant access or
infer completion from query parameters.

`STRIPE_TEST_SECRET_KEY` accepts the exact `sk_test_` or `rk_test_` prefix
with a nonempty suffix using the [RFC 6750 section 2.1 bearer-token grammar](https://www.rfc-editor.org/rfc/rfc6750#section-2.1):
ASCII letters, digits, `.`, `_`, `~`, `+`, `/`, and `-`, followed by optional
`=` padding only at the end. The 4096-byte total limit is an application
transport bound, not a Stripe format claim. Whitespace, controls, non-ASCII,
quotes and commas are refused; the value is never trimmed or transformed.
[Stripe documents the TEST key prefixes](https://docs.stripe.com/keys), without
specifying the previous local alphanumeric suffix or 16–256-character bound;
[Stripe's SDK](https://github.com/stripe/stripe-node/blob/master/src/utils.ts)
forwards the opaque key in `Authorization: Bearer`. Local framing only admits
a safely transportable TEST credential: Stripe still authenticates it and
checks permissions. This correction does not prove that an operator's actual
credential works; provider validation remains required. TEST-resource, price,
coupon, settlement, authentication and CSRF checks remain in force.

Configuration keys are `STRIPE_TEST_CHECKOUT_ENABLED=true`,
`STRIPE_TEST_SECRET_KEY`, `STRIPE_TEST_WEBHOOK_SECRET`, `STRIPE_TEST_PRICE_ID`,
`STRIPE_TEST_RETURN_ORIGIN`, comma-separated
`STRIPE_TEST_ALLOWED_COUPON_IDS`, and comma-separated
`STRIPE_TEST_ALLOWED_PROMOTION_IDS`. The configured price must be the one-time
USD 20.00 test price. The origin must be the exact beta web origin, such as
`https://beta.skager.app`, without a trailing slash. Stripe's webhook endpoint
must use the pinned API version in `commerce.ts` and deliver the five event
types admitted there. The server searches Stripe's active promotion codes for
the entered code and accepts exactly one active, unexpired promotion whose
promotion ID and coupon ID are on both local allowlists. Customer-specific,
minimum-amount, and first-transaction promotions are rejected. It authenticates
the configured Price to derive its product ID and separately retrieves the
Coupon with `expand[]=applies_to`. The coupon must explicitly apply to exactly
that product; missing, unrestricted, empty, wrong-product and multi-product
applicability are refused. The Price must be active and both resources TEST-only;
the existing one-time USD 20 terms remain required. It places that promotion ID on the Checkout session. Hosted
promotion-code entry is disabled, and each order binds its selected promotion
and coupon before payment. A completed session must show that same promotion,
coupon, and exact amount before fulfillment. Stripe still decides final
redemption eligibility and may reject a code at Checkout creation or payment.
Discounted requests omit `allow_promotion_codes`; its default disables hosted
entry and Stripe [does not permit it alongside `discounts`](https://github.com/stripe/stripe-node/issues/2248).
Full-price requests retain explicit `allow_promotion_codes=false` and no discount.

The pinned `2026-09-30.endive` [PromotionCode schema](https://docs.stripe.com/api/promotion_codes/object?api-version=2026-09-30.endive&query=promotion)
uses `promotion.type=coupon` and an expandable `promotion.coupon` ID.
The [Checkout Session discount wrapper](https://docs.stripe.com/api/checkout/sessions/object?api-version=2026-09-30.endive&query=discounts)
still uses direct `coupon` and `promotion_code` fields; it is distinct from the
generic Discount object's `source.coupon`. [Coupon product applicability](https://docs.stripe.com/api/coupons/object?api-version=2026-09-30.endive&query=applies_to)
is not returned by default, so absence cannot establish its scope. Settlement
re-fetches the bound promotion-to-coupon relationship, Price and scoped Coupon,
then verifies the completed session's IDs, product and amounts. A promotion-only
wrapper with `coupon:null` requires that authenticated relationship. Embedded
coupon fields alone never establish applicability. Redemption availability is
checked at creation; a completed bound session can settle after the last
permitted redemption, later expiry or inactivity, or Price archival.

A dedicated SKAGER sandbox and restricted TEST key remain deployment gates.
The source requires Checkout Sessions write/read plus Promotion Codes, Prices,
Coupons and Charges read; no Product read call or refund/capture/write-management
API is used. Stripe [restricted keys](https://docs.stripe.com/keys) use `rk_test_`
in sandbox mode. A separate product or endpoint in a shared sandbox does not
isolate unrelated provider event delivery. Keep checkout disabled until dedicated
scope, actual key permissions and live TEST flows are verified.

Before `commerce_receive_event`, every checkout event must match the existing
immutable local order/session binding. Signature, event version, TEST mode and
structural fields are checked first. A canonical order reference triggers the
binding read before rejecting metadata, including malformed metadata types, or
inspecting SKAGER money terms. SQL `rejected`
means **unowned-or-mismatched**: the event is ignored with 200 and no receipt or
effect. Pending, malformed or failing binding reads retry with 503 and no write.
A structurally valid session without a canonical order reference is excluded
because SKAGER's creator always supplies one; metadata alone never grants
ownership. Once bound, the strict SKAGER identity/money checks still apply, and
altered own metadata retries rather than being silently discarded. Completion
uses the original bound price and completes its provider checks before ingestion.

Refund ownership starts with an authenticated Charge retrieval, followed by the
documented [Checkout Session PaymentIntent filter](https://docs.stripe.com/api/checkout/sessions/list?api-version=2026-09-30.endive&query=payment_intent)
using that exact intent and `limit=2`. The result must be one unambiguous TEST
payment session with matching intent, paid amount and currency, then match its
immutable local binding and SKAGER metadata. Empty, multiple or paginated
results, provider errors and pending bindings retry without inserting a receipt
or refund marker, preserving refund-before-completion retries. After preflight,
the existing atomic SQL refund marker, receipt deduplication and effect functions
remain unchanged; write-RPC retries retain their original idempotency boundary.

This filters persistent SKAGER application receipts and effects. It does not
isolate Stripe's account-wide webhook transport: unrelated TEST payloads can
still reach the Supabase handler before being discarded, and no raw payload is
logged. It neither authorizes configuring a shared webhook nor closes the
separate provider configuration/consent, dedicated-scope or live acceptance gates.
Keep the candidate's checkout flag off pending that review.

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
