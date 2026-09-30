# Auth0 session foundation

Status: first implementation slice of SCRUM-40. It implements server-side OIDC
login, callback validation, local account lookup, database-backed sessions and
local logout. It does not configure a real tenant, deploy authentication,
authorize portal objects, call Auth0 management APIs, or implement payment,
download or support behavior.

## Boundary

Auth0 is the only password and recovery system. The application identifies an
account only by immutable `(issuer, subject)`. Email and display name are
optional contact data and never identity or purchase join keys. The browser
receives only Django's opaque session identifier; ID, access and refresh tokens
are discarded after callback validation and are not persisted in the browser,
session, account record or logs.

Authentication is disabled by default. Production enables it only with
`OIDC_AUTH_ENABLED=1` and complete configuration. The issuer must exactly match
the fixed allow-list; issuer, authorization, token and JWKS URLs must be HTTPS
on the same configured Auth0 origin. The callback and post-login/logout paths
are fixed server configuration rather than request parameters.

The login request uses Authorization Code Flow with PKCE S256, unique state and
nonce values, and no `offline_access` scope. Pending protocol values live in
an opaque, five-minute database attempt bound to a digest of the Django session
key. Callback consumes it atomically under a row lock before token exchange,
then erases the stored nonce and PKCE verifier, making concurrent or repeated
callbacks one-shot. Schedule `manage.py purge_oidc_attempts` to remove expired
attempt rows; scheduler configuration remains an operations gate. The
confidential regular web client uses
Auth0's `client_secret_post` token-endpoint method. Authlib's OIDC claims
validator and the maintained `joserfc` library verify an RS256 signature against
the configured JWKS plus issuer, subject, audience, expiry, issued-at and nonce.
The successful callback flushes the pre-authentication session,
creates a fresh short-lived session and stores only the local account UUID.
Logout accepts CSRF-protected POST only and flushes that session.

Token and JWKS requests never follow redirects, so credentials and the PKCE
verifier cannot be forwarded to another origin through a provider redirect.
Responses are streamed through byte limits and closed on success or failure;
ID-token size, JWKS key count and incoming callback
parameter lengths are also capped. Provider failures return a generic error
without logging token material or exception details.

Authlib 1.8.0 and Requests 2.34.2 were selected from their official current
releases on 2026-09-30. Relevant primary documentation:

- <https://docs.authlib.org/en/latest/jose/jwt.html>
- <https://auth0.com/docs/get-started/authentication-and-authorization-flow/authorization-code-flow-with-pkce/add-login-using-the-authorization-code-flow-with-pkce>
- <https://auth0.com/docs/secure/tokens/id-tokens/validate-id-tokens>
- <https://auth0.com/docs/secure/attack-protection/state-parameters>

## Remaining gates

SCRUM-40 remains incomplete until a separate staging slice configures an Auth0
tenant/application, validates Universal Login and recovery/MFA behavior,
reviews cookie lifetime and incident/logout policy, adds abuse monitoring and
rate limits, verifies key rotation and network failure behavior, and completes
privacy/security review. No authentication route should be publicly enabled
before those gates and the overall launch GO.

The repository web workflow runs both core schema and account tests against
PostgreSQL, including concurrent callback consumption and reverse/reapply of
the authorization-attempt migration. Local success is not a substitute for
the required CI run on the merged commit.
