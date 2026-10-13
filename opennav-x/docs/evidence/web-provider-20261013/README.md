# Private beta web provider evidence — 2026-10-13

Scope: SCRUM-40, private invited beta. This record contains only sanitized
deployment and catalog observations. It does not establish a working customer
login, checkout, payment, download, or public launch.

The initial source revision is `c51836f29901cba1c6c221db734a316174a53bac`.
[Web CI run 38451243615](https://github.com/ThereptileII/Work/actions/runs/38451243615)
passed 88 Django tests and 45 Edge tests (133 total), with zero reported test
failures or skips. Its downloaded evidence ZIP SHA-256 is
`b4c435a7a4becfa1d155bbcf5e271d7c15736e78745baddf8e18e1d5ea25c01b`,
matching GitHub's artifact digest. The PostgreSQL SQL and migration steps passed.

The operator verified full predeployment byte equality for the 47,842-byte
Supabase Edge source (SHA-256
`0492c12db66726e0699750a00c164de99d2b0e57d0948a9329fd20dbd88a7664`)
and a successful deployment UI result. The previous source (SHA-256
`c27a5c890e40525bc0e4418c5e5bbf0abbe28b7f87b0dc5792b6824b5e8cf2fc`)
remains available for rollback. The initial 492,495-byte Cloudflare frontend
(SHA-256 `86ff2ae9ce56fa94f80999edc172327545daff1b08c2667662d85af75a63cc66`)
had deployment `07a6061a` active at 100%, with runtime dated 2026-10-13.
Direct Edge `/health` returned HTTP 200 with
`ready=false`; direct `/session` returned HTTP 200 with no account and CSRF
present. The beta origin and `/login` returned HTTP 200, while the beta
origin `/session` returned HTTP 503. That routing/session failure is unresolved
in this evidence.

A later diagnostic source revision, `3d87570441cb8d9375f73f2763033291c26b022e`,
passed [web CI run 38453173061](https://github.com/ThereptileII/Work/actions/runs/38453173061):
88 Django and 51 Edge tests, zero failures or skips, with evidence ZIP SHA-256
`ca4f51f8d8aa68c50f79f714338869c4c2ffd964f0968c6bbd2ae7a2421b83db`
matching GitHub's digest. Its 492,638-byte Cloudflare frontend (SHA-256
`0b5887f161fdb500bedb760bd78381032efb45dd24151480d8a65b01f62cc898`)
is deployment `99f572ea`, marked Active Latest. Supabase Edge remained on the
same source identified above. This deployment status alone does not establish a
working beta-origin session.

The saved provider catalog comparison matches the source on 21 function
definitions, 70 columns, and 25 structural constraints. Its 25 reported
`constraint_flags_or_type` differences are a comparison-query false positive:
all are primary-key, foreign-key, or unique constraints with PostgreSQL's
normal `connoinherit=true` flag
([PostgreSQL catalog semantics](https://www.postgresql.org/docs/18/catalog-pg-constraint.html)).
The recorded CHECK and default expressions are semantically consistent with
the source SQL. The catalog also observed 11
RLS-enabled private tables, 17 public service RPCs, four private functions,
zero policies, and seven triggers. Provider UI showed `public` and `graphql_public`
as exposed API schemas and `skager_private` unchecked. The catalog query alone
did not establish the live API session's exposed-schema setting.

Authentication flags and credentials remain unset. Custom SMTP is required,
pending the owner's choice. The USD 20 Stripe **test-mode** product and price
were verified. A test-mode 100% coupon (`SKAGERFRIENDS`, coupon `xecg4vnM`,
promotion `promo_1UQB86IqaIVnHG13BVLPyLWW`) was created and verified for
SKAGER only, duration once, maximum 25 uses. It is configuration only; no
entitlement was granted. No live Checkout or payment was exercised. Public signup,
payment, and downloads remain closed.

[`evidence.json`](evidence.json) gives the structured observations and source
links. It contains no provider credentials, customer identifiers, session
material, or screenshots.
