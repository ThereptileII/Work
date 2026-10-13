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
origin `/session` returned HTTP 503. That original failed observation is retained;
the later correction and observed result follow below.

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

At that checkpoint authentication flags and credentials were unset. Custom SMTP
remains required, pending the owner's choice. The USD 20 Stripe **test-mode** product and price
were verified. A test-mode 100% coupon (`SKAGERFRIENDS`, coupon `xecg4vnM`,
promotion `promo_1UQB86IqaIVnHG13BVLPyLWW`) was created and verified for
SKAGER only, duration once, maximum 25 uses. It is configuration only; no
entitlement was granted. No live Checkout or payment was exercised. Public signup,
payment, and downloads remain closed.

## Verified proxy correction and remaining provider gates

Exact source `839ea17f76dc631b46a4a70d9b164e1f70d26b69` passes
[web CI 38454462943](https://github.com/ThereptileII/Work/actions/runs/38454462943):
88 Django/PostgreSQL and 56 Edge tests, no failures, and real disposable SQL and
migration checks. Artifact `11840734310` (5,478 bytes) SHA-256
`3652722b2a706b86cb4682f710e8f9acaa8294a17d4c586de99571a6fec6722d`
matches the downloaded ZIP. Independently, six offline groups in actual
`workerd@1.20261013.1` pass with the unchanged Worker module and a local upstream
stub. The original deployed module reproduced the immediate HTTP 503: this
runtime rejects `redirect: 'error'`. The correction uses supported `manual`
mode and refuses every 3xx without forwarding its body, location or cookies.
Tests cover successful secure session cookies and 301/302/303/307/308 refusal,
with exactly one upstream request. Only static failure phases enter logs.

The 492,723-byte frontend SHA-256
`0a71e211d4cf3aca04162838abc1dccdffde3cb8c2a29d795f5dd9469a72aac2`
was read back in full and matched before deployment. Cloudflare `e18a9fb4` is
Active Latest. The Supabase server bundle remains byte-identical to the earlier
`0492c12d…` bundle; no SQL was reapplied. A real Chrome request to
`https://beta.skager.app/api/auth/session` now returns HTTP 200 and the sign-in
form renders. Opening the unauthenticated portal returns to login. A separate
unauthenticated command-line probe received non-JSON HTTP 403 before any
application result could be established; it does not prove backend refusals or
webhook delivery. No bot/security protection was disabled to change that result.

Eight non-secret backend settings were saved and their provider digests read
back: auth, checkout and portal flags are explicitly **false**; environment is
`staging`, price is the verified USD 20 price, return origin is the exact beta
origin, and coupon/promotion allowlists contain the two IDs above. No secret key,
webhook secret, SMTP credential or friend email was entered. The empty invitation
allowlist continues to deny general access.

Review before activating commerce found two source/provider mismatches: restricted
TEST keys (`rk_test_`) were rejected by that source, and the pinned Stripe
PromotionCode schema differs from its test fixtures. The SKAGER-only coupon
was also not supported by that applicability check. These corrections were
selected under SCRUM-36/37 and are addressed below. The existing Stripe sandbox also serves unrelated
products, so no broad webhook was added: unrelated refund data could reach the
unmatched-refund store despite entitlement ownership checks. Isolated test scope,
credential setup, email delivery, real login, checkout, coupon reconciliation,
refund, and cross-account portal acceptance remain pending. Configuration and
source tests are not completed end-to-end acceptance.

## Reviewed Stripe source correction deployed with access still closed

Exact source `251c88bed6063fe5212339df73caa81c48cb931f` passes
[web CI 38457534904](https://github.com/ThereptileII/Work/actions/runs/38457534904):
88 Django/PostgreSQL and 59 Edge tests, no failures or skips, plus disposable
SQL and migration checks. Downloaded artifact `11845055319` is 5,638 bytes,
SHA-256 `69c4e40ccead4b3abb4da674cc50fea90848ee244d2b6faa2e8cbf228db783da`;
length, digest and ZIP CRC match. An independent source review found the
discount/`allow_promotion_codes=false` parameter conflict; the follow-up removes
that parameter on discounted requests and tests both outgoing request forms.

The correction accepts only restricted or ordinary TEST keys, authenticates the
pinned PromotionCode relationship and the exact configured Price/product-scoped
Coupon, and preserves settlement for a completed bound purchase after expiry,
exhaustion or Price archival. Provider availability checks apply at creation;
historical completed transactions are still validated against their immutable
order/session/coupon binding. Live keys remain refused. SQL, webhook signatures,
session/CSRF protections and entitlement idempotency are unchanged.

The new 50,491-byte Supabase server bundle has SHA-256
`09728b9034eade3c3cae57b1021a28c47776c8d9f6303fd791b30c84c049898b`.
Its complete editor content matched before deployment; after the dashboard
reported the recent successful update, reopening the Code tab and copying its
complete content matched again. No opaque deployment ID was exposed by that
view, so none is inferred. The approved legacy gateway JWT switch remains OFF.
The frontend is byte-identical to the deployed `e18a9fb4` bundle; no redundant
Worker deployment or SQL migration was performed. A fresh Chrome login reload
renders the form and `/api/auth/session` returns HTTP 200.

Auth, checkout and portal remain explicitly OFF. No credentials, authorized
tester emails, invitations, Checkout sessions, payments or entitlements were
created. The current Stripe sandbox's Create/Manage sandbox controls were
disabled. An account-switch inspection reached live-account activation; no
bank/legal/account fields were entered, and the tab was returned to TEST mode.
The dedicated test scope gate remains: a possible shared-account filter would
need authenticated PaymentIntent-to-Checkout and local binding proof before
event ingestion, including completion events and refund-first race tests. That
design was reviewed only; it is neither implemented nor accepted. SMTP choice,
credential handoff and real end-to-end acceptance are still pending.

## Reviewed webhook ownership candidate packaged; not deployed

Exact source `662c62478ded1ea75f5646b984d5670cda36525a` passes
[web CI 38460214097](https://github.com/ThereptileII/Work/actions/runs/38460214097),
attempt 1, with 88 Django/PostgreSQL and 69 Edge tests (**157 total**), no failures
or reported skips, plus disposable SQL and migration checks. The successful
`postgres-integrity` job is `115513126793`. Original artifact `11844724305`,
`skager-web-postgresql-662c62478ded1ea75f5646b984d5670cda36525a`, is 6,086 bytes,
SHA-256 `a9d063db3d8d787191d1904fd18297fa630e7581c2151d611b655f604548e85e`.
The authenticated run/artifact API identities match that exact source; downloaded
length and digest match GitHub, and all ten ZIP members pass CRC. The artifact
contains CI logs; the runtime bundles below are separately retained.

The independent review passed isolated commits `5eb5f0e0` and `0497c63e`,
integrated as `cff85841` and `662c6247`. Ownership now precedes event ingestion:
checkout events require the original immutable local order/session binding;
refunds authenticate Charge → PaymentIntent-filtered unique TEST paid Checkout
Session → local binding. Unowned-or-mismatched sessions are ignored without a
receipt or effect. Pending, ambiguous, missing or failing preflight reads retry
with 503 before a write is attempted. Known owned malformed metadata also retries.
Completion provider checks precede ingestion, historical bound prices are
preserved, and the SQL refund-first/idempotency functions are unchanged. The
reviewed-only design described at the preceding `251c88be` checkpoint is retained
as the then-current state; this new implementation supersedes that source status.

The retained package identifies this exact `662c6247` source. Its 492,723-byte
frontend SHA-256 is
`0a71e211d4cf3aca04162838abc1dccdffde3cb8c2a29d795f5dd9469a72aac2`,
byte-identical to Cloudflare `e18a9fb4`; no Worker redeployment occurred. Its
52,899-byte backend SHA-256 is
`345b14d49db6fb9a214a66e8294f4cbb0e8e5a0bb15c1f4cdd636bf8b61149dc`.
Both retained lengths/digests and the package's source/hash manifest match.
**This backend is packaged, not deployed.** Live Supabase remains the earlier
`251c88be` / `09728b9034eade3c3cae57b1021a28c47776c8d9f6303fd791b30c84c049898b`
deployment described above. No new provider readback or end-to-end result is
claimed for the new candidate.

The filter protects persistent SKAGER receipts and effects; it does not isolate
Stripe's account-wide delivery. Unrelated TEST payloads can still transit the
Supabase handler before discard. Specific shared TEST webhook delivery requires
separate provider consent/configuration and live qualification; dedicated scope
remains a gate. No raw payload logging was added. Auth, checkout and portal
remain OFF, credential handoff and SMTP remain pending, and no account, payment
or live end-to-end acceptance has occurred. Public access remains closed.

[`evidence.json`](evidence.json) gives the structured observations and source
links. It contains no provider credentials, customer identifiers, session
material, or screenshots.
