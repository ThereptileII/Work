# Supabase identity verification boundary

This is a bounded SCRUM-40/SCRUM-94 foundation. Supabase is the selected replacement
identity provider; the existing Auth0/OIDC implementation remains disabled and
has not been converted into a Supabase login flow by this slice. The adapter has
no route, settings, session, model, database or migration integration. It does
not implement signup, login, token refresh, local sessions, logout, password
recovery, account linking, entitlements, purchases or support authorization.
Those capabilities remain unqualified under the public-beta contract.

## Fixed trust boundary

`web/skager_web/accounts/supabase.py` accepts explicit, reviewed server
configuration. `SupabaseIdentityVerifier(project_url, publishable_key)` is
disabled by default; verification requires the explicit Boolean `enabled=True`.
There are no environment-variable readers or application enablement settings in
this slice. Never construct configuration from a request or token contents.

The coordinator's authenticated Chrome inspection on 2026-10-13 identified the
project as `https://kelwgubkuvditalsbqpr.supabase.co`, region West EU (Ireland).
The inspection found no migrations or backups; this records a project
observation, not backup qualification. No credentials are recorded here, and
this implementation and its tests did not access the project or change provider
configuration.

Only canonical HTTPS hosted-project origins and modern `sb_publishable_...`
keys are accepted. Reject secret keys, legacy `anon`/`service_role` JWT keys,
custom domains, ports, paths, credentials, queries, fragments and local services.
Project configuration is frozen after construction. A publishable key identifies
the application component; it does not establish the user's identity. See
[Supabase API keys](https://supabase.com/docs/guides/getting-started/api-keys).

Each enabled call sends a bounded bearer JWT to exactly
`<project_url>/auth/v1/user`, with the publishable key in `apikey`. It uses a fresh
Requests session, TLS certificate verification, no redirects, no inherited
proxy/netrc configuration, and connect/read timeouts of 3.05/5 seconds. The read
timeout is an inactivity timeout, not a total request deadline. Compressed
responses are refused. Responses are closed on every path and limited to 64 KiB;
tokens are limited to 16 KiB. There is no local JWT decoding, signature fallback,
cached success or offline identity fallback. If the provider cannot be reached,
identity is unavailable. Deployment-level concurrency and overall request
deadlines still require review.

Only a 200 response at the exact expected URL with a JSON object can succeed.
Malformed UTF-8/JSON, duplicate keys, non-finite numbers, incorrect or excessive
declared lengths, and excessive bodies fail closed. The top-level user must have
a canonical nonzero UUID, `aud=authenticated`, `role=authenticated`, and an
explicit `is_anonymous=false`; deleted and currently banned users are refused.
Auth failures, expiry/revocation refusals, rate limits, redirects, timeouts and
other provider failures return the same generic authentication exception without
retaining the provider exception as a cause or context. Do not log bearer tokens,
raw HTTP objects or provider bodies.

The result is a frozen `SupabaseIdentity` containing only the owned issuer
`<project_url>/auth/v1`, the provider user UUID as `subject`, and optional
`verified_email`. Only a valid top-level `email` with a valid, non-future
`email_confirmed_at` grants confirmed contact data. General `confirmed_at`, token
claims, `user_metadata` and `app_metadata` cannot supply identity or email
confirmation. Preserve email spelling as contact data; any future local lookup
must use `(issuer, subject)`, never email. Equal emails do not merge accounts.

Supabase documents [get_user](https://supabase.com/docs/reference/python/auth-getuser)
as a server-validated user lookup. Current upstream
[Auth authentication middleware](https://github.com/supabase/auth/blob/master/internal/api/auth.go)
checks token signature/expiry, user existence, bans and the session row when a
nonzero `session_id` is present. That upstream source is not evidence of the
hosted project's deployed version. The adapter refuses every provider rejection;
it does not independently inspect session IDs or prove immediate logout
revocation for every token form. Supabase's
[session documentation](https://supabase.com/docs/guides/auth/sessions)
distinguishes JWT lifetime, signout and session-policy enforcement. Qualify these
behaviors with the actual hosted project before connecting application sessions.

## Configuration and qualification checklist

These are acceptance gates for the existing Jira work, not a separate backlog.

- Verify the project origin and obtain its modern publishable key through the
  owner's provider session. Supply reviewed values to the adapter explicitly;
  keep enablement false until an application integration has passed review.
- Review Auth email/password settings, required email confirmation, disabled
  anonymous signup, permitted external providers, abuse protection and rate
  limits. Record actual settings; adapter checks are not provider configuration.
- Configure the exact staging Site URL and redirect allowlist only once the
  reviewed callback/recovery routes exist. Do not enable wildcard production
  redirects or repoint the existing Auth0/OIDC callback without implementing the
  Supabase flow. Review SMTP delivery, sender/domain ownership, email templates
  and recovery behavior in staging.
- Implement and verify the separate login/session/recovery boundary: PKCE and
  one-time state, CSRF, session rotation and expiry, secure opaque cookies,
  refresh-token handling, logout/revocation policy and account authorization.
  Define handling for unconfirmed users; the adapter can return identity without
  confirmed contact and grants no portal permissions by itself.
- Qualify real ordinary/anonymous users, email confirmation, wrong-project and
  expired tokens, signout/session revocation, deletion/bans, signing-key rotation,
  provider outages, privacy and incident handling. Mock tests do not prove those
  live behaviors. Keep public access closed until the wider launch gates pass.
- Keep application data in the selected Django/PostgreSQL boundary. This adapter
  neither creates Supabase tables nor establishes RLS or backup policies. Review
  database ownership, region, migrations and restore evidence separately before
  storing application data or claiming operational readiness.

## Focused evidence

From `opennav-x/web`, run:

```sh
DJANGO_SETTINGS_MODULE=skager_web.settings.preview \
  /home/standard/Projects/.tmp/skager-web-beta-venv/bin/python manage.py test \
  skager_web.accounts.tests.test_supabase --noinput
```

The suite uses injected transports plus a real-HTTP guard and no database. Its
20 tests cover default-off configuration, fixed TLS transport, identity/contact
ownership, malformed and excessive responses, invalid UUID/security fields,
remote refusals, resource cleanup, failure redaction and repeated verification
after a prior success. No provider requests, account writes, migrations,
dependency changes, application package builds or live acceptance are implied.
