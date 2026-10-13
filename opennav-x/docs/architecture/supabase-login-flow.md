# SKAGER private-beta email sign-in (SCRUM-40)

Cloudflare serves the public shell and forwards the API to the single Supabase
Edge Function. No passwords, Supabase access tokens, refresh tokens or service
keys reach browser JavaScript. The backend calls fixed HTTPS `/auth/v1/otp`,
`/verify`, then `/user` to obtain the canonical verified subject. Public Supabase signups stay disabled. Only the allowlisted signup route calls
the privileged `/auth/v1/invite`, which sends an email code and leaves the
identity unconfirmed. Existing confirmed-user invitation conflicts fall back to
ordinary OTP. Every `/otp` call uses `create_user: false`; login and recovery
cannot create accounts. No successful invitation grants a session without OTP
verification and canonical `/user` proof.
Recovery means regaining passwordless access using a fresh emailed code.

Supabase's [email OTP documentation](https://supabase.com/docs/guides/auth/auth-email-passwordless)
requires both the Magic Link and Invitation templates to show `{{ .Token }}` for code entry. Configure
that template and an SMTP sender before inviting testers; keep public signup disabled and email confirmation enabled; this change makes no
vendor configuration or delivery claim. Keep public registrations disabled at
the application boundary; only configured friends are accepted.

## Server configuration and routing

`SUPABASE_AUTH_ENABLED` defaults disabled and must equal `true` to allow auth.
`SUPABASE_INVITE_EMAILS` is a comma-separated, case-normalized exact email
allowlist. Empty denies everyone, including previously logged-in accounts.
Supabase supplies `SUPABASE_URL`, `SUPABASE_ANON_KEY` and
`SUPABASE_SERVICE_ROLE_KEY` only to the backend. The service key is used by the
coordinator's `ctx.rpc` implementation and the fixed `/auth/v1/invite` request,
never by clients. The provider URL must
be an HTTPS `*.supabase.co` origin. Custom provider domains are not supported.

`handleAuth(request, ctx)` returns a response for `/api/auth/session` (GET), and
`/api/auth/login`, `/signup`, `/recover`, `/verify`, `/logout` (POST), otherwise
null. Send endpoints receive `{email}`, verification receives `{code}`. A
successful send always returns the same `{ok:true,message:...}`, including
unknown/disallowed emails, provider failures and throttle denial. Verification
returns `{ok:true}` or a generic 400 error. A failed attempt consumes the
challenge; request a new code. No `next`/redirect URL is accepted.

`getAccount(request,ctx)` returns `{id,email}` or null. `id` is the canonical
Supabase Auth subject UUID. The session GET returns `{account}`. The router adds
`csrfToken` and sets a 32-byte lowercase hexadecimal `__Host-skager_csrf` cookie,
Secure, SameSite=Lax, Path=/, readable by JavaScript. All API mutations require
exact Origin `https://beta.skager.app` and the matching `X-CSRF-Token`. Auth also
checks these requirements itself, including direct Edge Function requests.

`ctx` supplies `env`, `rpc(name,args)`, optional `fetch` for tests and optional
`clientIp`. The IP MUST come from authenticated, trusted Cloudflare ingress
metadata. Never copy caller-controlled `X-Forwarded-For`/`CF-Connecting-IP` at
the publicly reachable Edge Function. An absent trusted IP uses a shared global
bucket. The Worker/Edge router must authenticate its proxy metadata independently
of CSRF. The auth module never reads forwarding headers.

## Storage and bounds

Apply `web/edge/auth.sql` as database owner. All data lives in `skager_private`,
with RLS enabled and no client-role access. Public SECURITY DEFINER RPCs have an
empty fixed search path and execute permission for service_role only. Challenge
cookies hold 32 random bytes; only SHA-256 digests and the intended email are
stored, for ten minutes. Atomic delete-and-return consumes a challenge before
provider verification, preventing replay and concurrent verification. An OTP
from another browser cannot create a session without the matching pending
challenge cookie. Sending a new code replaces and consumes the old pending
challenge if eligible. Origin/CSRF prevents attackers from setting a victim's
pending identity.

Sessions use a fresh random `__Host-skager_session` HttpOnly Secure SameSite=Lax
host-only cookie and server-side hash, subject and absolute expiry. They expire
after twenty minutes, do not slide, and replace the prior browser session after
verified login. At most five sessions are retained per subject. Logout deletes
the server record and pending challenge and expires both cookies. OTP/provider
tokens exist only in request memory, with no persistence or serialization.

DB throttles enforce five sends per fifteen minutes for each email, trusted IP,
and the entire private beta; this intentionally conservative global cap protects
the small friend test. Verify is limited to ten per IP/challenge per fifteen
minutes. Atomic limits do not rely on process memory. Expired rows are pruned
on subsequent creates/throttle calls. A DB/provider failure fails closed for
verification and account lookup. Responses and exceptions never expose upstream
bodies or secrets, and this module logs nothing.

Session lookup joins `auth.users`: user deletion, bans, unconfirmed email and
changed allowlist deny access immediately. Supabase Auth global signout or
provider token revocation alone does not revoke these independent sessions.
Their residual validity is bounded to twenty minutes. Incident response can
remove a subject's private session rows or disable auth. Rechecking the provider
on every request would require persisting a provider token; this flow deliberately
does not. Live SMTP delivery, deployment routing, SQL permissions and persistent
DB concurrency require integration qualification in addition to mocked tests.

The privileged invite path is required because the upstream [magic-link handler](https://github.com/supabase/auth/blob/master/internal/api/magic_link.go) routes unconfirmed users through signup, which is disabled here. Admin create with `email_confirm:false` followed by `/otp` therefore cannot onboard a new beta user. The [invite handler](https://github.com/supabase/auth/blob/master/internal/api/invite.go) sends a confirmation token without this public-signup path; the [verify handler](https://github.com/supabase/auth/blob/master/internal/api/verify.go) accepts that code with `type:email`. Backend-issued invites remain restricted by the exact server allowlist.
