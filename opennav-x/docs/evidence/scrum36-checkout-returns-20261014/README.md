# SCRUM-36/43 — TEST checkout return destinations

Source `af986f7416bebb638c0e59dfbf98e58732a21157` replaces unpublished
`/checkout/test/success/` and `/checkout/test/cancel/` destinations with the
existing `/portal/orders/` and `/portal/` pages. Only signed webhook/provider
settlement can grant access; visiting a return URL has no payment authority.

The focused commerce/Worker/portal suite passes 54 cases. Exact
[CI 38547971840, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38547971840),
job `115798639801`, passes 88 Django and 105 Edge tests, with no failures, plus
disposable SQL boundary assertions. Original artifact `11885466806` is 7,343
bytes, SHA-256 `9adaef781e3a8258c0b6ee66eb4e6a298f4551ee7770e4daeb39183438da0520`.
Provider source/run/job identity, outer bytes and all ten ZIP member CRCs were
authenticated. The new assertions check actual generated Stripe form URLs and
actual member HTML/JS/CSS through the Worker router, with no settlement or
private-data backend call from return navigation.

## Deployed bytes

The immutable local package is `/tmp/skager-web-checkout-candidate-af986f74`.
Its server-only `supabase-index.ts` is 77,896 bytes, SHA-256
`09d1de0db9a9ea8ac3cae2b2fec42195ca9f85fd32c533b1fa14b6cfbf113a61`.
The full selected editor document matched before Deploy and after a fresh page
reload. Supabase `skager-web` in project `kelwgubkuvditalsbqpr` showed a fresh
deployment time. Its previously approved legacy-JWT gateway remains off; no
application session, CSRF, provider-signature or entitlement guard was removed.

Frontend bytes remain identical to active Cloudflare `f5508044`: 503,124 bytes,
SHA-256 `68743a3eb86ef6545c6119ddd3a8cf7e9aef45eec28f43c6176d75fe7bd2bbbd`.
No frontend redeployment or feature-flag change was needed. Ordinary unauthenticated
navigation to the published orders destination reached the SKAGER login page,
not a missing-page response. Direct Chrome health navigation was client-blocked;
it is not a new health or customer-flow pass.

## Remaining live gates

SMTP authentication still requires owner correction; no new invite was sent.
Checkout, portal access, downloads and public access remain closed. This source,
CI and deployment evidence does not establish delivered email, completed login,
Stripe checkout/webhook/refund, paid or zero-total entitlement, cross-account
isolation, private R2 downloads, or any Windows product acceptance. Original
email failures remain in the SCRUM-40 evidence record.
