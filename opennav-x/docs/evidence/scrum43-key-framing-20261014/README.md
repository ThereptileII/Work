# Stripe TEST key framing: source and package evidence

Jira SCRUM-43. Exact source `2c1511c4e8dc269fce0846e616af9b99675faca7`.
The exact retained server candidate was **deployed and read back on 2026-10-14**.

The server now accepts opaque TEST secret/restricted keys using bounded ASCII
Bearer framing instead of an invented provider suffix length/alphabet. It
preserves accepted bytes exactly and refuses empty suffixes, LIVE/publishable
keys, whitespace, control characters and header injection before effects. Stripe
remains the authority for actual credential validity. The earlier static
configuration diagnosis remains valid historical evidence; it did not identify
whether the stored key or its former local predicate caused the rejection.

[CI 38613459842](https://github.com/ThereptileII/Work/actions/runs/38613459842),
attempt 1, succeeds at that exact source: **208 tests** (88 Django/PostgreSQL and
120 Edge), no failures or skips, plus migrations and disposable SQL checks.
Original artifact `11919036360` is 7,830 bytes, SHA-256
`27ad6c2199e96558620cec7320e6384802229a83388bf26ad27805b2a973c50c`.
Its original retained ZIP matches GitHub metadata and every member CRC passes.
The original logs and build manifest remain in the private local folder
`/home/standard/Work/skager-web-key-framing-2c1511c4`.
[Sanitized receipt](receipt.json) records their identities and verification scope.

Packaged server `supabase-index.ts`: **79,004 bytes**, SHA-256
`d2bcfbc710c9488d62c51ed62188f4af1d07934fcfc1a9b95c809ee913cbd358`.
The retained build manifest and actual server bytes agree. Frontend upload stays
504,268 bytes, SHA-256
`a42d54ba857209cc76cee87a66905ad34502e5d80fc79d1091e3c27931427f0e`,
unchanged from active Cloudflare `ac34d9d4`; no Cloudflare redeploy is needed.

Chrome reconnected. The existing provider editor matched the prior 78,865-byte
server exactly before replacement. Whole-document readback before deployment and
after reloading the provider page matches all 79,004 candidate bytes and the
server SHA-256 above. Supabase shows a fresh deployment. Its subsequent function log identifies
deployment `kelwgubkuvditalsbqpr_8fda2b14-f5cf-4bde-bb44-cd27323b6291_24`,
version **24**. The existing legacy-JWT
gateway flag remains OFF; application sessions, CSRF and webhook signatures
remain enforced. No secrets or frontend bytes changed.

The version-24 log at **2026-10-14T19:28:49Z**, ID
`6bf3cc1e-a328-4bdb-9bde-494fbc7d1d9d`, still reports
`skager-commerce:configuration-v1:STRIPE_TEST_SECRET_KEY`. The corrected broader
framing therefore has not resolved the stored binding. No secret was read or
changed and no Stripe authentication was attempted. The owner correction form
contains only the exact binding name and an empty value; webhook and other
settings remain untouched. Correcting that binding precedes another checkout.

The owner previously confirmed sign-in. The new portal visit redirected to
login because that session expired; a new authorized sign-in code was requested
and the same browser is waiting for owner entry. No live key authentication is
claimed. Actual TEST checkout, the approved 100% coupon, verified webhook/active
entitlement/refund, cross-account checks and customer download remain unproven.
Public signup, public payments and downloads remain closed. No credential, OTP,
account identifier or private browser evidence is included here.
