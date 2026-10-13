# Supabase preparation

For the current Cloudflare/Supabase Edge deployment, follow the
[architecture decision](../../../docs/architecture/cloudflare-supabase-private-beta.md)
and [auth configuration](../../../docs/architecture/supabase-login-flow.md#server-configuration-and-routing).
Configure application secrets as `SKAGER_AUTH_ENABLED` and `SKAGER_INVITE_EMAILS`;
custom `SUPABASE_` names are reserved and rejected by Supabase. Keep the injected
`SUPABASE_URL`, `SUPABASE_ANON_KEY` and `SUPABASE_SERVICE_ROLE_KEY` unchanged.
Auth remains disabled unless the flag is exactly `true`, with an exact nonempty
invitation allowlist. The preparation observations below remain historical;
they do not establish current deployment or live-flow acceptance.

Owner-authorized Skager project inspected through its existing Chrome session
on 2026-10-13. Region: West EU (Ireland). No project keys are stored here.

`001_private_schema.sql` was applied once to the inspected project after a
read-only check showed that `skager_private` did not exist and the Supabase
`anon`/`authenticated` roles did. Readback: owner `postgres`, anonymous USAGE
false, authenticated USAGE false. It creates no user, table, bucket, password or
vessel data. Do not rerun it to repair/adopt an unknown namespace. Do not expose
this schema through the Data API.

This is preparation, not a hosted application deployment. The Django-owned
Boat/accounts/payment migrations have **not** been applied to Supabase. Before
doing that, provision separate limited application/migration credentials in the
deployment secret manager, verified TLS/CA, private schema search path,
backup/restore and staging isolation. Existing provider-wide roles/defaults and
unrelated project data must not be changed to make migrations pass.

Email confirmation remains enabled; anonymous sign-in/manual identity linking
remain disabled. Public signup stays closed pending qualification. The default
Site URL/empty redirect list is deliberately unchanged: no real backend callback
has been deployed. The owner-private Sites website is a static visual preview,
not an Auth callback or Stripe webhook host.

Configure exact HTTPS callback addresses only after the actual staging server
exists. Verify session/recovery/email/rate-limit behavior against that project,
then complete the privacy/security review before enabling customer registration.
Never paste database, service-role, SMTP, Stripe or signing secrets into Git,
Jira, screenshots, browser-facing JavaScript or a static preview.

Stripe test catalog created in the owner's existing sandbox:
`SKAGER Public Beta Access`, one-time USD 20. Product reference
`prod_VQqM7r6I6UWt6g`; price reference `price_1UPyfbIqaIVnHG136VnHPQTF`.
These references are sandbox metadata, not credentials or production defaults.
No Checkout session, payment, refund, webhook endpoint or entitlement was
created. Review the actual price and pin the test key/return origin server-side
before exercising the provider adapter. Public payments remain closed.
