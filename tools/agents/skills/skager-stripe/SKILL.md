---
name: skager-stripe
description: Configure and troubleshoot SKAGER Stripe TEST Checkout, promotion codes, signed webhooks, entitlements and refunds. Use for invited-beta payment flows and zero-cost tester access; it does not authorize live commerce.
---

# SKAGER Stripe

Locate the current web checkout. Read
`opennav-x/docs/architecture/checkout-entitlements.md`, actual
`opennav-x/web/edge/commerce.ts`, related SQL and tests. Use the source-pinned API
version and endpoint contract. Confirm account, sandbox and resource ownership;
the current offer is a one-time USD 20 **TEST** purchase for invited beta access.
Keep live payments and public access closed pending explicit launch approval.

## Keys and account scope

- API secrets start `sk_test_` or restricted `rk_test_`; webhook signing secrets
  start `whsec_`. A publishable/API key cannot verify a webhook. A CLI listener's
  secret does not replace a dashboard endpoint's signing secret.
- Server bindings are `STRIPE_TEST_SECRET_KEY` and
  `STRIPE_TEST_WEBHOOK_SECRET`. Do not expose values in Git, chat, screenshots,
  logs or frontend assets. Use the approved secret channel; if tool policy
  requires user creation/input, prepare other fields and hand off only that step.
- Recheck permissions against actual calls. The current restricted-key path
  needs Checkout Sessions read/write and Prices, Promotion Codes, Coupons and
  Charges read. Do not grant broad access to fix a wrong key or endpoint.
- Treat credential bytes as opaque. Do not trim, repair, invent a suffix regex
  or weaken framing checks to accept an unknown value. Passing local framing is
  not evidence that Stripe authenticated the credential.
- An earlier shared TEST sandbox also contained Ember. A distinct product,
  coupon or endpoint secret does **not** isolate account-level webhook events.
  Preserve the existing explicitly approved boundary or use a separately
  authorized sandbox. Reject foreign events through owned immutable bindings;
  do not log raw events or infer access to unrelated customer/payment data.

## Checkout, discounts and settlement

- Read the exact event types and API version from source when configuring the
  endpoint. Preserve raw-body signatures, timestamp checks, provider re-fetch,
  atomic SQL effects and durable deduplication. A success return URL never grants
  entitlement. Avoid creating duplicate endpoints on an uncertain retry.
- Use the existing pre-checkout discount field. Resolve an active, allowed
  promotion/coupon against the bound product and server-owned price. Verify
  expanded applicability; the pinned Promotion Code and Checkout discount
  response shapes differ. Do not combine explicit `discounts` with
  `allow_promotion_codes` in the same Checkout request.
- An approved 100% discount must yield the same **ACTIVE Staging entitlement**
  as a verified full-price TEST purchase. Verify the owned completed session,
  `no_payment_required`, zero total, bound approved coupon and no PaymentIntent.
  Do not fake a paid row or manually grant access; no real funds are involved.
- Bind order, session and authenticated subject. Replayed events have one
  effect; foreign events have none. Failed/expired checkouts grant nothing;
  refund-before-completion remains terminal. Test refunds only on owned TEST
  transactions in the authorized scope.

## Efficient diagnosis

Find the first failing layer: session/CSRF → server binding → provider key/scope
→ resource/API schema → Checkout → signed webhook → SQL → portal entitlement.
Do not rotate keys or redeploy unchanged code by guesswork. For uncertain create
results, reconcile the existing operation/idempotency key before retrying.

Run focused mocked-provider and disposable SQL tests, then a bounded authorized
live TEST journey. Record checkout created, completed, signed settlement, active
entitlement and portal access separately. Key presence, a settings save or a
mocked test does not prove live settlement. This requires no desktop rebuild.
Use `skager-web-publish` for coordinated deployment and `skager-supabase` only
when backend permissions/configuration need attention.

Primary references: [keys](https://docs.stripe.com/keys),
[webhooks](https://docs.stripe.com/webhooks), and
[Checkout discounts](https://docs.stripe.com/payments/checkout/discounts).
