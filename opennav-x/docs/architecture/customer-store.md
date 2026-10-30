# SKAGER customer store: purchase, account, portal

This replaces the invited-beta flow described in
[checkout-entitlements.md](checkout-entitlements.md) and
[supabase-login-flow.md](supabase-login-flow.md). Stripe stays in **TEST** mode.

## Journey

1. `/pricing/` and every “Get SKAGER” call to action lead to `/checkout/`, one
   step per requirement: **Account** (email; Public Beta Access, USD 20, the
   configured Stripe Price), then each active checkout step from
   `store_steps` (seeded: **Your helm PC** and **GPS**) where the customer picks
   one of the step's products or its “I already have one” answer, then **Review
   and pay** (lines, shipping, discount code, total). Products belong to a step
   through `store_products.step_id`; `checkout_addon` is kept equal to “in an
   active checkout step” so order validation is unchanged. `?add=<slug>`
   preselects a product in its step. No public page offers sign-up; **an
   account exists only because of a purchase**.
2. `POST /api/store/checkout` (CSRF + exact Origin, throttled per email and
   globally) creates an order from database prices, refuses emails that already
   own SKAGER (`exists`), have an unfinished account (`setup`) or were refunded
   (`revoked`), and opens a hosted Checkout Session (customer email, shipping
   address and a fixed shipping rate only when an item ships, a server-approved
   promotion or none). The session is bound to the order before the URL is
   returned.
3. Stripe returns to `/welcome/?session_id=…`. The page drops the reference
   from history and calls `POST /api/auth/claim`. While the webhook has not
   settled the order it answers `202 pending` (the page polls). Once paid it
   emails a one-time code (Supabase invite/OTP) bound to the challenge cookie.
4. `POST /api/auth/verify` consumes the challenge, verifies the code with
   Supabase, links the Supabase user to the customer (`store_link_account`) and
   creates a 20-minute session. A first login lands on
   `/portal/account/?setup=1`; every portal API answers `403 {setup:true}` until
   `POST /api/auth/password` sets a password (10–128 characters).
5. Later logins: `POST /api/auth/login` checks email + password with Supabase,
   then emails a code; `verify` completes it. `POST /api/auth/forgot` sends a
   reset code, after which the account is in setup mode again. Errors never
   reveal whether an email has bought.

## Settlement and refunds

`POST /api/stripe/webhook` verifies the signature over the raw body, binds the
event to an order through the stored session ID, records the event once, then
**re-fetches** the session and checks every line, total, discount, shipping and
payment/subscription before `store_apply_paid`. Handled events:
`checkout.session.completed`, `checkout.session.async_payment_succeeded`,
`checkout.session.async_payment_failed`, `checkout.session.expired`,
`charge.refunded`, `customer.subscription.updated` and
`customer.subscription.deleted`.

A refund of Public Beta Access (self-service within 14 days from Orders, or
by an admin) **revokes the customer**: sessions are deleted, `store_session_get`
stops resolving, login, claim and verify are refused, and any SKAGER Cloud
subscription is cancelled at Stripe. Zero-total orders refund locally.

## Portal (`/portal/…`, `edge/static/portal.*`)

Overview · Downloads (first each active download step from `store_steps`,
seeded **Install OpenCPN** with a link to the official OpenCPN 5.12.4 Windows
installer or “I already have OpenCPN installed”, then Windows x64 from the
reviewed catalog in `downloads.ts`,
one fresh 60-second R2 link per press; Windows on ARM, Linux x64, Linux ARM64
and macOS shown disabled as “Coming”; then each active guide step, seeded
**Forecast wind with GRIBstream** and **Online AIS with AISstream**, where the
first line of the text is the introduction and each further line a numbered
step; then an “All set” card) · Shop (hardware, `POST /api/store/shop`)
· Cloud (USD 2/month subscription Checkout `POST /api/store/cloud`, Stripe
billing portal `POST /api/store/billing`, services listed as “Coming”) · Orders
(refund) · Account (password, boats).

## Admin (`/admin/…`, `edge/static/admin.*`, `admin.ts`)

Only verified sessions whose email is in `SKAGER_ADMIN_EMAILS` and that have a
password. Steps (checkout, download and get-going guide stages: title, text
(line breaks kept, up to 1,200 characters), “already have it” or skip answer,
optional https link, multiple choice for checkout steps, sort, shown/hidden;
the account, review and SKAGER download steps are fixed). Products (name, summary,
description, price, checkout step, visibility,
image up to 2 MB stored in R2 bucket `skager-media`, served at
`/api/media/products/<id>/<sha256>.<ext>`), orders (mark shipped with tracking,
refund), customers, shipping rate and countries. The access price is fixed by
the database (it must match `STRIPE_TEST_PRICE_ID`). Every change is audited in
`store_admin_audit`.

## Email and jobs

Transactional mail (welcome/receipt, shop receipt, shipped, refunded, cloud
started/ended, admin new-order notice) is written to `email_outbox` in the same
transaction as the state change and sent through Resend with an idempotency key,
inline where possible and by `POST /api/jobs/tick` every minute
([store-schedule.sql](../../web/deploy/supabase/store-schedule.sql)). Failed sends
back off exponentially and stop after six attempts.

## Discount codes

A code works only if its promotion code and coupon IDs are in the two allowlists
below, and it is checked with Stripe before the order exists (hosted code entry
stays off). Where it applies depends on how the coupon is scoped in Stripe:

- **SKAGER (Beta Access) code:** a coupon restricted to the Beta Access product.
  It is accepted only on the guest checkout and discounts only the access line.
- **SKAGER Cloud code:** a coupon with no product restriction, with any duration
  (once, repeating for N months, forever). It is accepted only on the portal's
  SKAGER Cloud subscription, where the order has the Cloud line and nothing else
  (`store-discounts.sql`). A fully discounted first month settles as
  `no_payment_required` and still starts the subscription.

The two kinds never accept each other's coupons. To add a code: create the coupon
and a promotion code in Stripe, then add both IDs to the allowlists.

## Stripe Checkout styling

Every session sets `branding_settings`: night background `#050f13`, cyan buttons
`#6fe0ea`, Inter, rounded corners and the SKAGER wordmark from
`/static/marketing/skager-wordmark.png`. If Stripe refuses the branded session,
the server opens the standard page once instead (separate idempotency key), so
styling never blocks a payment.

## Configuration (Supabase function secrets)

| Key | Purpose |
| --- | --- |
| `SKAGER_AUTH_ENABLED`, `PORTAL_ENABLED`, `SKAGER_DOWNLOADS_ENABLED` | `true` to open each area |
| `STRIPE_TEST_CHECKOUT_ENABLED`, `STRIPE_TEST_ENVIRONMENT=staging` | Store switch and environment |
| `STRIPE_TEST_SECRET_KEY` | `rk_test_` key: Checkout Sessions write, Customers write, Prices/Products read, Promotion codes/Coupons read, Refunds write, Charges read, Subscriptions write, Invoices read, Billing portal write |
| `STRIPE_TEST_WEBHOOK_SECRET`, `STRIPE_TEST_PRICE_ID`, `STRIPE_TEST_RETURN_ORIGIN` | Webhook signing, access Price, `https://beta.skager.app` |
| `STRIPE_TEST_ALLOWED_PROMOTION_IDS`, `STRIPE_TEST_ALLOWED_COUPON_IDS` | Discount allowlists |
| `RESEND_API_KEY`, `SKAGER_EMAIL_FROM` (optional) | Email sending from `auth.skager.app` |
| `SKAGER_ADMIN_EMAILS` | Comma-separated admin emails |
| `SKAGER_JOB_SECRET` | ≥ 32 characters; same value as Vault `skager_job_secret` |
| `R2_ACCOUNT_ID`, `R2_DOWNLOAD_BUCKET`, `R2_ACCESS_KEY_ID`, `R2_SECRET_ACCESS_KEY` | Read-only release downloads |
| `R2_MEDIA_BUCKET`, `R2_MEDIA_ACCESS_KEY_ID`, `R2_MEDIA_SECRET_ACCESS_KEY` | Read/write `skager-media` for product images |

Supabase Auth: email provider on, password sign-in on, minimum length 10,
public sign-up **off** (users are created only by the service through invite),
email OTP length 6, templates send `{{ .Token }}`.

## Data

`web/edge/store.sql`, then `web/edge/store-steps.sql`, then
`web/edge/store-guide.sql` (private schema
`skager_private`, RLS on, SECURITY DEFINER RPCs callable only by
`service_role`). Assertions in `store.postgres.sql` and
`store-steps.postgres.sql` and `store-guide.postgres.sql` run in
CI against disposable PostgreSQL; never apply `bootstrap-test.sql` or
`*.postgres.sql` to a real project.
