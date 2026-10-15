---
name: skager-supabase
description: Configure and diagnose SKAGER Supabase Auth, Edge Functions, private PostgreSQL/RLS, sessions and server secrets. Use for login, gateway, migration and backend-state failures in the invited beta.
---

# SKAGER Supabase

Locate the current web checkout and read
`opennav-x/docs/architecture/cloudflare-supabase-private-beta.md`, then the relevant
`supabase-login-flow.md`, `checkout-entitlements.md` or `private-downloads.md` in
that directory. Inspect the actual `opennav-x/web/edge` handlers and deployment
SQL. The current backend is the `skager-web` Edge Function, not the historical
Django deployment. Confirm the selected project before any mutation.

## Access, gateway and secret configuration

- Prefer existing authenticated provider tools; otherwise use the authorized
  dashboard. Do not extract browser access tokens or use browser fetch to bypass
  unavailable UI automation.
- Separate a gateway rejection from an application rejection. The existing
  Worker does not forward a Supabase service-role Authorization header. SKAGER's
  handler validates its own sessions, exact Origin/CSRF and Stripe signatures;
  it does not trust requests merely because they passed the Worker. Follow the
  approved gateway policy; do not remove application checks or expose a service
  key in the browser as a workaround.
- Custom `SUPABASE_` names are reserved. Keep vendor-provided variables intact;
  current application auth bindings use `SKAGER_AUTH_ENABLED` and
  `SKAGER_INVITE_EMAILS`. Resolve source/config mismatches rather than inventing
  renamed bindings in the dashboard.
- Keep server secrets only in their approved protected store, with no values in
  Git, chat, command history, screenshots or diagnostics. Name/presence does not
  prove a valid value. Prepare nonsecret fields before a credential handoff when
  tool policy requires the user to create or enter a new credential.
- **Auth SMTP Password** is separate from Edge Secrets. Use `skager-resend` for
  SMTP/domain delivery work. Keep public signup OFF, confirmation ON, exact
  invited-email allowlisting and both Invitation/Magic Link `{{ .Token }}`
  templates. An invitation alone is not a confirmed account or valid session.

## Database changes

1. Inspect existing schemas, migration state, roles, grants, exposed schemas and
   RLS before applying the selected deployment migration. Preserve `auth.users`
   and provider-managed roles. Do not adopt or overwrite unfamiliar objects.
2. Never run `bootstrap-test.sql`, `*.postgres.sql` test fixtures, disposable role
   setup or auth test data on the real project. Use reviewed deployment SQL in
   its required order; transaction/idempotency behavior comes from that SQL.
3. Preserve the private schema outside exposed Data API schemas, no direct
   `anon`/`authenticated` table access, RLS and fixed-search-path SECURITY DEFINER
   RPCs restricted to `service_role`. Do not broaden grants to cure a failing RPC.
4. A timeout or an operation still running is not proof of success or failure.
   Read state before retrying a non-idempotent migration or provider mutation.

## Diagnose and verify

Trace proxy → gateway → configuration → Auth/SQL/SMTP and fix the first proved
failure. Use sanitized failure categories and timestamps, not raw provider
response bodies. Do not loosen key validation, SQL permissions or session
ownership simply to get a green check.

For email-code failures, inspect the current challenge semantics: verification
consumes the pending challenge even on failure. Arrange one fresh code in the
same browser when necessary; the user enters it on the site. Never request an OTP
in chat, auto-confirm an account or substitute a fabricated session.

Verify actual login/session, logout, persistence and account isolation as relevant.
A table, deployed function, rendered shell or `ready:true` is not end-to-end
acceptance. Run focused handler tests and SQL checks in a disposable database;
no desktop build is required. Use `skager-web-publish` only when publication is
part of the authorized task. Record exact web revision and provider deployment
separately, keeping public access closed.

Primary references: [Edge secrets](https://supabase.com/docs/guides/functions/secrets),
[custom SMTP](https://supabase.com/docs/guides/auth/auth-smtp), and
[email OTP](https://supabase.com/docs/guides/auth/auth-email-passwordless).
