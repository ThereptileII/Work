---
name: skager-web-publish
description: Prepare, publish, and verify SKAGER's invited beta website on Cloudflare and Supabase. Use for beta.skager.app deployment, email-code access, test checkout, or portal publication; desktop release promotion is separate.
---

# SKAGER web publication

Publish the owner-selected Cloudflare Free frontend/static Worker with Supabase
Auth, Edge Functions and PostgreSQL. Stripe stays TEST-only. Follow repository
`AGENTS.md`, `opennav-x/AGENTS.md` and the selected Jira issue; Jira is the sole
backlog. This skill does not authorize deployment, provider changes, tester
invitations, public launch or paid upgrades beyond the user's existing scope.

For provider-specific setup or diagnosis, load only the relevant installed
`skager-resend`, `skager-cloudflare`, `skager-supabase` or `skager-stripe` guide
(versioned fallback: `tools/agents/skills/<name>/SKILL.md`). A full publication
does not require loading all four when their layers are already verified.

## Establish the candidate

The website may live in a separate active Git worktree from the desktop candidate.
Locate the selected web checkout and verify its HEAD; if it lacks `web/edge` or
the current architecture decision below, do not assume the older Django runtime
is deployed or borrow another branch's acceptance. Resolve the intended web
revision before proceeding.

Read these repository-root paths before preparing publication:

- `opennav-x/docs/architecture/cloudflare-supabase-private-beta.md`: current
  architecture; it supersedes older Django/Render runtime guidance.
- `opennav-x/docs/architecture/supabase-login-flow.md` and
  `opennav-x/docs/architecture/checkout-entitlements.md`: identity and settlement.
- `opennav-x/web/edge/{index.ts,platform.ts,cloudflare-worker.mjs}` and relevant
  auth/commerce/portal handlers and SQL: actual deployed behavior.
- `opennav-x/web/tools/build_cloudflare_beta.py` and
  `.github/workflows/skager-web.yml`: actual build arguments and CI coverage.

Identify the exact full committed source SHA, clean web inputs and CI run SHA.
Inspect current CI artifacts/results rather than carrying forward historical
counts or treating an ancestor's pass as current qualification. The existing web
workflow runs Django/PostgreSQL, Edge mocked-provider and disposable SQL checks;
it does not package or deploy either service or prove live E2E.

For packaging, provider setup, editor upload or live verification, read
[the publication procedure](references/publication.md). If requested only to
prepare/review, leave provider mutations and live checks unperformed and say so.

## Preserve the boundaries

- Target only `https://beta.skager.app`; no apex-domain exchange, DNS replacement
  for `skager.app`, paid Containers backend or architecture substitution.
- Keep global public signup OFF, email confirmation ON and exact friend-email
  allowlisting enforced. Empty allowlist denies all. Both Magic Link and
  Invitation email templates must render `{{ .Token }}` for code entry.
- Only Supabase server secret bindings receive service-role/database/SMTP/Stripe
  secrets. Never put credentials, OTPs, provider tokens or raw private evidence in
  Git, Jira, chat, screenshots, public assets or browser code. Never extract
  browser tokens or use browser `fetch` to bypass an authenticated provider UI.
- Never apply `bootstrap-test.sql`, `commerce.postgres.sql`,
  `security.postgres.sql` or other SQL tests to a real provider database. They
  belong only in a disposable test database. Inspect existing objects before
  applying reviewed deployment SQL; do not adopt or overwrite unknown state.
- An approved 100% coupon must produce the same ACTIVE (`active`) Staging
  entitlement as a verified full-price TEST checkout, with zero real funds.
  Never fake a paid order, manually grant entitlement or trust a return URL.
- Keep raw-body webhook signature verification, provider re-fetch, immutable
  order/session binding, durable deduplication and atomic effects intact.
- Public payments/downloads stay closed until the complete exact-candidate
  readiness review and explicit owner GO. Invited website work is independent
  of desktop release qualification; neither substitutes for the other.

## Record the result

Return the exact source SHA, CI link/result, retained build manifest and hashes,
provider deployment identifiers/domain readback when performed, observed live
checks, and remaining gates in the selected Jira issue or linked evidence.
Distinguish source checks, packaging, uploaded/deployed bytes and real live E2E.
Health `ready:true`, a rendered shell or passing mocked tests proves none of the
email, checkout, webhook, entitlement or cross-account flows by itself.
