# Owner-private web experience and provider foundations

Date: 2026-10-13. Jira: SCRUM-31/33/36/40/41/43/94; future cloud vision SCRUM-369.
Isolated branch: `codex/web-experience-20261013`.

## Exact source and validation

Web implementation and CI source: `52d07f404a7c6c6257382e46ea09bb38ae5cf9cc`.
[Run 38355728110](https://github.com/ThereptileII/Work/actions/runs/38355728110)
succeeded: **88 tests** on PostgreSQL 18.6/Python 3.12, Django checks, migration
drift, fresh migrations and reverse/reapply. These include 20 mocked Supabase
identity cases, 21 mocked Stripe cases and 9 Boat service/PostgreSQL cases.
Local 48 focused marketing/provider checks overlap CI and are not added to88.

[Failed run 38355275628](https://github.com/ThereptileII/Work/actions/runs/38355275628)
on `fdf3adf02f0fefbf0f819f87e2f7ba9fe94862ce` is preserved. PostgreSQL correctly
rejected an overlong Boat name with DataError; the test expected IntegrityError.
The replacement narrows the expected exception without weakening rejection.
GitHub also rejected a synthetic live-shaped secret string in an unpublished
fixture. It was removed from unpublished history; push protection was neither
bypassed nor disabled. No real credential was involved.

A subsequent documentation/Git-ignore commit does not relabel these source gates.
Local `.env` paths and web credential/private-key files are ignored; no tracked
web environment/key/certificate file was found in the focused filename check.
This is not a claim of a complete repository-history secret audit.

## Private publication

URL: https://skager-beta-launch-preview.swedishcountrysideli.chatgpt.site

Sites project: `appgprj_6acd44a03b60819192aa8cde1e7b719b`.
Generated static source: `1264b297caa79b13a0e03bffbf98c2035c091443`.
Deployment: `appgdep_6acdceb248788191a55a5fc8f8803d06`, succeeded.
Archive SHA-256: `020517c42a91bb46cc6e1a28ae191961911c2e7c9d92b7593879d59618cce1b0`.
Twenty allowlisted informational pages and owned assets are exported from the
exact web source above; provenance maps the content hashes. No backend source,
provider configuration, keys, boat telemetry or licensed charts are exported.
Existing owner-only audience was verified before publishing and preserved.
Post-login DOM readback verified the new homepage and closed-access disclosures.
[Deployed capture](home-deployed.jpg) uses the normal narrow browser viewport;
[desktop 1280×800](home-desktop.jpg) and [mobile 390×844](home-mobile.jpg) captures
are local Django smoke evidence. Day/Night, pause, mobile static fallback,
menu opening and Escape dismissal were exercised; mobile has no horizontal
overflow. Temporary viewport overrides were reset. Content-hashed asset URLs
avoid reusing an earlier stylesheet during browser review.

The unchanged owner brief is `docs/design/web/skager-web-experience-original.md`,
SHA-256 `87fad9df0f225afdfd95493214e24638820a455a2a255ec1b2c6cc3b33d1a5cb`.
Original wordmark retained. The chart scene is an explicit illustration, not a
product screenshot or live navigation. Future cloud functionality stays labelled
unavailable. No native software rebuild, chart/design acceptance, endurance run
or boat modification was requested by this website increment.

## Actual provider changes and remaining boundaries

Supabase project Skager: email confirmation enabled; anonymous/manual identity
linking disabled. New signups changed from enabled to disabled and verified
again after reload. Existing localhost Site URL and empty redirect list retained
until an actual backend callback exists. No user or real credential was fetched.

The exact reviewed `web/deploy/supabase/001_private_schema.sql` was applied through
the SQL editor after read-only preflight confirmed the schema absent. It creates
only an empty `skager_private` namespace and revokes PUBLIC/anon/authenticated
namespace/default-object access for the current creator. Readback: owner postgres,
anon USAGE false, authenticated USAGE false. No application tables, users,
passwords or buckets were created. Limited application/migrator roles, server
secret storage, TLS, migrations and backend operations still require qualification.

Stripe sandbox: SKAGER Public Beta Access, USD 20.00 one-time, product
`prod_VQqM7r6I6UWt6g`, price `price_1UPyfbIqaIVnHG136VnHPQTF`.
Existing unrelated product retained. No live-mode configuration, Checkout session,
payment, customer, refund, webhook destination or entitlement was created.
The pure adapters use mocks; actual auth sessions/recovery, durable payment
inbox/deduplication/effects/refunds and end-to-end entitlement remain unimplemented.
Stripe is not assumed a Merchant of Record. Provider settings screenshots remain
local, outside Git and the preview. Real keys were not read or copied.

The static private preview is not the Django/Supabase/Stripe backend deployment.
Public purchases/accounts/downloads remain closed; legal/tax/security and full
customer-journey acceptance are pending. No public-beta GO is implied.
