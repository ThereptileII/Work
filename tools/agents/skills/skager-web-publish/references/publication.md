# Publication procedure

Use repository-root paths below. Reinspect the actual builder, workflow and
provider tools for the candidate; this procedure is not a deployment receipt.

## Retain exact outputs

The builder requires Python with `opennav-x/web/requirements.txt` installed and
`npm` available; it invokes pinned `esbuild@0.28.2`. From the repository root,
with `candidate_sha` set to the verified full current HEAD and
`candidate_output` set to a fresh absolute directory outside the repository:

```sh
python3 opennav-x/web/tools/build_cloudflare_beta.py \
  --source-commit "$candidate_sha" --output "$candidate_output"
```

This is packaging only. The script checks exact HEAD and a clean web subtree;
also review relevant committed workflow/build inputs. Keep the output directory
immutable, including `build-info.json`, `public/preview-provenance.json`,
`cloudflare-upload.js` and `supabase-index.ts`. Verify the manifest's source,
hostname, environment, `livePayments:false` and both SHA-256 values against the
retained files before upload. Do not edit generated bundles or relabel their
provenance after a helper repair; use the same retained bytes or build a fresh
candidate explicitly. `public/` contains public assets; `supabase-index.ts` is
server-only and must never be uploaded as a frontend asset.

The current workflow's SQL loop includes test bootstrap/assertions because its
PostgreSQL is disposable. It is never a provider deployment recipe.

## Select an authenticated deployment route

Discover current tools first and prefer purpose-built authenticated provider
tools when available. At this skill's creation, no Cloudflare authenticated MCP
tool was available; do not assume that remains true. An installed CLI is not
proof of authentication. Inspect its supported commands/help and account/project
identity before use. There is no checked-in Wrangler deployment configuration
in this candidate; do not invent a repository deploy command or imply the build
script deploys. Consult applicable Cloudflare skills/current official provider
documentation when introducing a CLI/configuration.

If no suitable authenticated API/CLI exists and browser work is authorized,
use the owner's existing provider browser session through documented UI tools.
Confirm the account/project, service and selected environment before mutation.
Never recover credentials from storage, cookies, page internals or network
requests, and never make browser `fetch` calls to provider administration APIs.
Stop a failing route when authentication, target identity or upload integrity
cannot be established; report the specific missing evidence instead of bypassing
the boundary or repeatedly deploying guesses.

For a dashboard Monaco editor fallback:

1. Load the complete retained `cloudflare-upload.js` into the Cloudflare Worker
   editor, or `supabase-index.ts` into the `skager-web` function editor. Replace
   the whole document once; a viewport or hidden input is not the full model.
2. Before clicking Deploy, read back the **entire selected editor model** through
   supported UI/editor access and compare its UTF-8 bytes to the retained file.
   Require exact byte equality, byte length and SHA-256 equality, including the
   final newline. A visible beginning/end, character count alone, successful
   paste notification or screenshot cannot establish equality.
3. If readback is truncated, unsupported or unequal, do not deploy that paste.
   Use a supported whole-file upload/authenticated tool or resolve the mismatch.
   Record non-secret expected/observed hashes and the comparison outcome.
4. After Deploy, read back the deployment/version identifier and active routing;
   retain provider source/hash evidence where available. Editor equality proves
   the proposed upload; it does not prove activation or live service behavior.

## Configure the private backend

Confirm the selected Supabase project is the source-pinned
`kelwgubkuvditalsbqpr` project. Inspect existing schemas, migration history and
permissions read-only first. Apply only reviewed `web/edge/auth.sql`,
`commerce.sql` and `portal.sql` from `opennav-x/`, in that order as database owner,
within their supplied transactions. `commerce.sql` is not a blindly rerunnable
migration. Do not rerun `web/deploy/supabase/001_private_schema.sql` to adopt an
unknown namespace. Preserve `auth.users` and provider roles/defaults; never use
test bootstrap or assertions here. Read back RLS, revoked client access,
fixed-search-path service-role-only RPC permissions and absence of
`skager_private` from exposed Data API schemas.

An editor showing Running is pending, not success or failure. Confirm completion
and read-only object/permission state before replaying a migration; if the outcome
is unknown, inspect it rather than repeating non-idempotent SQL.

Publish the retained server bundle as `skager-web`. The Worker forwards no
provider Authorization header; verify the function gateway permits these
application-authenticated requests and Stripe webhooks. Check current supported
gateway configuration rather than forwarding a service key or weakening handler
authentication/CSRF/signatures to cure a gateway failure.

Supply only server bindings, without printing values:

- Auth: `SUPABASE_URL` (exact source-pinned HTTPS origin), `SUPABASE_ANON_KEY`,
  `SUPABASE_SERVICE_ROLE_KEY`, `SUPABASE_AUTH_ENABLED`, `SUPABASE_INVITE_EMAILS`.
- Commerce: `STRIPE_TEST_CHECKOUT_ENABLED`, `STRIPE_TEST_SECRET_KEY`,
  `STRIPE_TEST_WEBHOOK_SECRET`, `STRIPE_TEST_PRICE_ID`,
  `STRIPE_TEST_RETURN_ORIGIN`, `STRIPE_TEST_ALLOWED_COUPON_IDS`,
  `STRIPE_TEST_ALLOWED_PROMOTION_IDS`.
- Portal: `PORTAL_ENABLED` and `STRIPE_TEST_ENVIRONMENT=staging`. Portal access
  remains unavailable unless both its flag and the exact staging environment
  match; an enabled checkout alone does not enable the portal.

Start feature flags disabled; enable only within the authorized invited live
qualification window when required configuration is verified. Keep general
access closed while checks are pending or failing. Verify global signup OFF,
confirmation ON, exact allowlist, SMTP sender/delivery, and both Magic Link and
Invitation templates containing `{{ .Token }}`. Backend invitations must not
auto-confirm users; login/recovery OTP uses `create_user:false`, followed by
code verification and canonical `/user` proof. Use only authorized tester
addresses; creating this skill does not authorize sending invitations.

Verify the one-time USD 20 TEST price and bound promotion/coupon allowlists.
Set return origin to `https://beta.skager.app` without a trailing slash. Configure
the TEST webhook at `https://beta.skager.app/api/stripe/webhook`, matching the API
version and five admitted event types in current `commerce.ts`; put its signing
secret only in the backend binding. Never enable live keys, payment access or
public registration to make a test pass.

## Activate and qualify

Publish the retained frontend bundle to the selected Free Worker and bind only
`beta.skager.app`. Confirm the pinned Supabase proxy target, HTTPS routing and
security headers. Preserve the apex and other DNS/services. Verify responses on
the exact beta origin; alternate Worker origins are intentionally refused.

Record actual observations separately from local/CI results:

- New invited signup with delivered invitation code; existing-user login,
  recovery and logout; wrong/replayed code, disallowed email and cross-browser
  challenge refusal; cookie/CSRF behavior and expiry/revocation.
- Full-price TEST checkout and approved 100% coupon checkout; delivered signed
  webhooks and durable settlement; duplicate delivery creates no extra effect;
  failed/expired checkout grants nothing; refund yields terminal revocation.
  Both completed paths must show the same `active` Staging access; zero-total
  requires `no_payment_required`, bound approved coupon and no payment intent.
  Both paths record `real_funds_received_minor:0`.
- Two authorized accounts see only their own orders and boat names; caller IDs
  cannot change ownership. Downloads remain unavailable. The portal currently
  stores boat names, not telemetry, remote control or navigation data.

Retain sanitized deployment IDs, source/hash mapping, provider event/result
references and failed/pending checks. Do not store OTPs, session cookies, personal
inbox content or secrets in public evidence. A public-launch GO remains a
separate owner decision after all product and operational gates pass.
