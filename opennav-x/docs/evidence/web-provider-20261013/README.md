# Private beta web provider evidence — 2026-10-13

Scope: SCRUM-40, private invited beta. This record contains only sanitized
deployment and catalog observations. It does not establish a working customer
login, checkout, payment, download, or public launch.

The source revision is `c51836f29901cba1c6c221db734a316174a53bac`.
[Web CI run 38451243615](https://github.com/ThereptileII/Work/actions/runs/38451243615)
passed 88 Django tests and 45 Edge tests (133 total), with zero reported test
failures or skips. Its downloaded evidence ZIP SHA-256 is
`b4c435a7a4becfa1d155bbcf5e271d7c15736e78745baddf8e18e1d5ea25c01b`,
matching GitHub's artifact digest. The PostgreSQL SQL and migration steps passed.

The operator verified full predeployment byte equality for the 47,842-byte
Supabase Edge source (`0492` identifier prefix) and a successful deployment UI
result. The previous `c27a` deployment remains available for rollback. The
Cloudflare frontend `86ff492495` has active deployment `07a6061a` at 100%,
with runtime dated 2026-10-13. Direct Edge `/health` returned HTTP 200 with
`ready=false`; direct `/session` returned HTTP 200 with no account and CSRF
present. The beta origin and `/login` returned HTTP 200, while the beta
origin `/session` returned HTTP 503. That routing/session failure is unresolved.

The saved provider catalog comparison matches the source on 21 function
definitions, 70 columns, and 25 structural constraints. Its 25 reported
`constraint_flags_or_type` differences are a comparison-query false positive:
all are primary-key, foreign-key, or unique constraints with PostgreSQL's
normal `connoinherit=true` flag
([PostgreSQL catalog semantics](https://www.postgresql.org/docs/18/catalog-pg-constraint.html)).
The recorded CHECK and default expressions are semantically consistent with
the source SQL. The catalog also observed 11
RLS-enabled private tables, 17 public service RPCs, four private functions,
zero policies, and seven triggers. Provider UI showed `public` and `graphql`
as exposed API schemas and `skager_private` unchecked. The catalog query alone
did not establish the live API session's exposed-schema setting.

Authentication flags and credentials remain unset. Custom SMTP is required,
pending the owner's choice. The USD 20 Stripe **test-mode** product and price
were verified; no live Checkout or payment was exercised. Public signup,
payment, and downloads remain closed.

[`evidence.json`](evidence.json) gives the structured observations and source
links. It contains no provider credentials, customer identifiers, session
material, or screenshots.
