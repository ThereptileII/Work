# Owner-selected Supabase and Stripe — 2026-10-13

This decision supersedes the Auth0/Paddle/vendor-database selection in the
historical SCRUM-31 architecture. Django remains the server-rendered application
and server-side authorization/commerce/support boundary. Supabase supplies
managed PostgreSQL and Auth; Stripe supplies hosted Checkout. No paid plan,
production payment, telemetry collection or public-beta GO is approved here.

## Observed environments

The owner made the existing Chrome sessions available. Supabase's Skager project
is in West EU (Ireland), with no migrations/backups listed at initial inspection.
Email confirmation is on, anonymous login and manual identity linking are off;
email is the only enabled identity provider. The default Site URL is still
`http://localhost:3000`, with no redirect allowlist. Do not invent a callback or
point Auth at the static Sites preview: no server callback exists there.

The open Stripe account is a sandbox, not live mode. Only test catalog setup is
in scope. No card details, customer data, keys or tokens belong in Git, screenshots,
Jira, static exports or the normal browser client. No live key is needed now.

## Data and identity

Use a non-exposed `skager_private` PostgreSQL schema for Django-owned accounts,
orders, entitlements, session state and future boat metadata. Do not add it to
Supabase's exposed Data API schemas, or grant it to `anon`/`authenticated`.
Apply versioned Django migrations only after a dedicated limited application
role, separate migrator, verified TLS connection and backup/restore process
have been provisioned. Bootstrap SQL creates only the empty restricted namespace;
it does not deploy the application or qualify database recovery.

Supabase's Auth user UUID and the fixed project's issuer identify an account.
Email is contact data, never permission to join identities or purchases. The
new identity adapter validates a token against that project's Auth server. It
does not enable signup/login/session/recovery UI or replace the qualified local
OIDC tests. Fresh provider verification, session lifetime/revocation, CSRF,
rate limits, recovery email delivery and abuse controls still require integration
and real-project acceptance before login opens.

Normal browser code receives no service-role, database, Stripe or Jira secret.
Boat reads and writes must be scoped to the authenticated owner on the server.
There is no vessel telemetry ingestion or remote command path in this increment.
Future direct client data access would need separately reviewed RLS policies.

## Payments

The initial offer remains USD 20 one-time beta-services access, with promotion
codes. Use server-selected product/price, immutable local order/account mapping,
hosted Checkout and verified bounded raw-body webhooks. Redirects never grant
access. Durable deduplication, transactionally applied entitlement effects,
refund/reversal ordering, reconciliation and audit remain mandatory before
acceptance. Pure signature verification cannot provide durable deduplication.

Ordinary Stripe Checkout does not itself establish Merchant-of-Record coverage.
Do not inherit the older Paddle tax/VAT claims. Determine the actual Stripe
product, seller obligations, tax/refund/receipt configuration and qualified
legal review before public payments. Test-mode adapters fail closed for live keys
and live events; upgrading them to production is a separate reviewed change.

## Website and My Boat

The unchanged owner brief is
`docs/design/web/skager-web-experience-original.md`, SHA-256
`87fad9df0f225afdfd95493214e24638820a455a2a255ec1b2c6cc3b33d1a5cb`.
It sets the marketing experience and boat-centred portal direction. Public
`GET SKAGER` leads to the planned offer; `LOGIN` explains closed account access
until authentication is deployed. Code-driven chart imagery is clearly
illustrative and supports pause, reduced motion and small-device fallbacks.

SCRUM-43 governs the ownership portal. Optional cloud telemetry, boat health,
voyages, remote alerts and analytics are captured in SCRUM-369. Existing
SCRUM-81/82/83/88/116 retain sync, backup, installations, fleet and route handoff.
These are future capabilities, not promised beta entitlements or invented live
measurements. Core navigation stays offline-capable and SmartNav never steers.

Official references:
[Supabase custom schemas](https://supabase.com/docs/guides/api/using-custom-schemas),
[Supabase Auth](https://supabase.com/docs/guides/auth),
[Stripe Checkout](https://docs.stripe.com/payments/checkout),
[Stripe fulfillment](https://docs.stripe.com/checkout/fulfillment).
