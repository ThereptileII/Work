# Stripe test-only adapter boundary (SCRUM-36 / SCRUM-41)

This foundation permits a closed Stripe sandbox experiment. It does not select
Stripe for production or supersede the existing Paddle/MoR architecture. It
adds no HTTP route, settings enablement, database write, entitlement effect,
public checkout, download access, deployment or vendor-account change. No
actual Stripe API calls were made while implementing or running these tests.

## Contract

`web/skager_web/payments/stripe.py` exposes two operations:

- `create_hosted_test_checkout(TestStripeConfig, UUID)` creates an explicitly
  hosted, one-time test Checkout Session. The frozen server configuration holds
  one approved sandbox `price_…` ID, an explicitly reviewed HTTPS backend return
  origin and an optional promotion-code switch; the
  caller cannot supply a price, quantity, amount, currency, URL or arbitrary
  metadata. Before provision, an operator must verify that the configured Price
  is active, one-time, USD 20 (2000 minor units) in the intended sandbox. A Price
  ID alone does not encode mode, currency or amount. The response must confirm
  test mode, the same immutable order references, USD 2000 subtotal and a
  bounded total. No price-discovery call or external account check is included.
- `verify_test_webhook(raw_bytes, signature_header, signing_secret, now=…)`
  authenticates the exact bounded raw body before JSON parsing and returns a
  frozen `VerifiedTestEvent`. It has no network or database operations. The
  envelope contains only event/session/order references, creation/signature
  timestamps, SHA-256 body digest, currency, total and payment status. It never
  contains customer email, account metadata, card information or raw payload.

The local order UUID is an immutable correlation value. It does not prove
ownership of an authenticated account. The future application must bind the
order to the authenticated immutable Auth0 identity, persist the returned
session mapping and verify it independently before any payment effect.
Metadata, email, browser redirects and this envelope cannot grant access.

Both request and webhook schemas pin **`2026-09-30.endive`**, which Stripe's
[current versioning documentation](https://docs.stripe.com/api/versioning)
identified as current on 2026-10-13. Requests send `Stripe-Version` explicitly;
the future sandbox event destination must be configured to the same version.
An API request header does not change an event destination's schema version.
Changing this pin requires review and contract tests.

## Transport and input bounds

The sole API URL is `https://api.stripe.com/v1/checkout/sessions`. The adapter
accepts only `sk_test_` secret keys. It opens a fresh Requests session with
ambient proxy and `.netrc` configuration disabled, TLS verification enabled by
default, redirects disabled and no automatic retries. Connect/read inactivity
timeouts are 3.05/10 seconds; these are Requests socket limits, not a guaranteed
total wall-clock deadline. An uncertain timeout must be reconciled using the
original order/session mapping before another purchase attempt.

The deterministic `skager-test-checkout-v1-<order UUID>` idempotency key is
reused on retries of the same immutable order. Callers must not change price or
promotion configuration for an existing order. Stripe's API v1 key retention
is finite (at least 24 hours); this is not durable application deduplication.
See [Stripe idempotency](https://docs.stripe.com/api/idempotent_requests).

The return origin is a required frozen server configuration value, never browser
input. Missing configuration fails closed before network access. It must be a
bare lowercase HTTPS DNS origin, with no credentials, IP address, port, path,
query or fragment. The adapter derives exactly `/checkout/test/success/` and
`/checkout/test/cancel/` under that origin. They are presentation destinations,
and no routes are added here. No real backend callbacks are configured or
qualified yet; the existing static Site cannot implement these backend routes.
A later closed backend deployment must qualify the origin and both routes
before use. No production domain or sandbox Price ID is an environment default. The returned URL must use the exact HTTPS authority
`checkout.stripe.com`, no user information, port or query, and the exact
`/c/pay/<returned test session ID>` path. Custom checkout domains are excluded.
These choices follow the
[hosted Checkout Session contract](https://docs.stripe.com/api/checkout/sessions/create).

API responses require status 200, the exact API response URL, no redirect
history, JSON content type and no content encoding. A bounded raw read accepts
at most 64 KiB; length claims, truncation, malformed/duplicate-key JSON and
unexpected test-session identities are rejected. Responses always close.
Boundary failures expose fixed messages without vendor bodies, keys or chained
provider exception text. Generic errors are raised after leaving the exception
handler, so neither `__context__` nor `__cause__` retains the provider exception. Credential fields are omitted from configuration repr.

Webhooks accept at most 256 KiB of bytes, a 2048-character ASCII signature
header, 32 header fields, exactly one canonical timestamp and 1–8 valid v1
signatures. Other signature schemes do not authenticate. Every v1 comparison
uses `hmac.compare_digest`; HMAC-SHA256 covers `timestamp + '.' + raw body`.
The fixed tolerance rejects timestamps more than 300 seconds old **or in the
future**. The host needs a synchronized clock. JSON is parsed only after HMAC
and freshness validation; duplicate keys, malformed UTF-8, non-finite constants,
non-object JSON, unsupported API versions/types and live-mode events fail
closed. See [Stripe manual verification](https://docs.stripe.com/webhooks#verify-manually).

Only four checkout snapshot event types are accepted: completed, expired,
async-payment-succeeded and async-payment-failed. Both event and session require
`livemode: false`; connected-account/organization envelopes are rejected. Both
order reference fields must contain the same canonical nonzero UUID. This does
not establish price ownership, vendor-account identity, discount authorization
or final financial status. Refund/dispute/reversal schemas are deliberately
unsupported pending their own verified processing design.

## Replay and unfinished integration gates

Timestamp freshness mitigates captured stale requests. A valid redelivery in
the tolerance window produces the same event ID and payload digest, which must
be deduplicated transactionally by the future persistent inbox. A pure verifier
cannot durably reject replays across processes or restarts. Stripe retries have
fresh delivery timestamps; events can arrive out of order. Never use event
creation timestamps as the ordering or deduplication authority.

Required later work includes sandbox vendor-account confirmation, approved
Price provisioning and exact mapping, CSRF-protected authenticated order
creation, bounded webhook HTTP handling, durable inbox identity/digest
conflict checks, immutable provider binding, discount validation, monotonic
payment/refund/dispute effects, reconciliation, privacy/security review and
staging rehearsal. Public payments/downloads remain closed pending all release
gates and explicit user GO. This adapter alone satisfies neither checkout nor
entitlement acceptance criteria.

## Focused verification

Run from `opennav-x/web`:

```text
python -m unittest skager_web.payments.tests.test_stripe -v
```

The 21 mock-only test methods cover fixed request parameters/version, immutable
UUID and idempotency reuse, key rejection, required/hostile return-origin configuration and secret
sanitization (including absent exception context/cause), timeouts and
no retries, hostile response hosts/redirects, bounded/truncated/compressed and
malformed responses, wrong order/environment/currency/amount, exact-byte HMAC
before parse, multiple v1 signatures and constant-time comparisons, stale and
future delivery rejection, valid retry identity, signed JSON/schema attacks,
and exclusion of unowned account/customer fields. These tests do not qualify
Stripe sandbox behavior, deployment, persistent replay processing or payment
and entitlement integration.
