---
name: skager-cloudflare
description: Operate SKAGER's existing Cloudflare Worker, beta.skager.app DNS and private R2 release storage. Use for deployment routing, upload verification, private downloads and Cloudflare faults within the established Supabase architecture.
---

# SKAGER Cloudflare

Locate the active web checkout; it can differ from the desktop candidate. Read
`opennav-x/docs/architecture/cloudflare-supabase-private-beta.md` and, for R2,
`opennav-x/docs/architecture/private-downloads.md`. Verify current provider state.
The selected architecture is Cloudflare Free Worker frontend/proxy + Supabase
backend + private R2 objects. Do not introduce Containers, a replacement backend
or a paid plan merely to perform an existing workflow.

## Choose the existing access and deployment path

- Discover available authenticated tools. Use the general `cloudflare` skill for
  product questions and `cf-cli` for supported CLI operations; use `wrangler`
  where the repository already has its configuration. Do not invent deployment
  commands or recover browser tokens to work around a disconnected browser.
- Preserve the apex domain and unrelated DNS. SKAGER invited-beta publication
  targets `beta.skager.app`; this skill grants no wider publication authority.
- For an actual deployment, follow `skager-web-publish` (or its versioned source
  under `tools/agents/skills`). Check the source SHA, clean web inputs, builder
  arguments and CI evidence before producing the bundle.
- With dashboard editor upload, verify the **whole file**, source manifest and
  SHA-256 before Deploy. A visible first page of code is not proof of a complete
  upload. Do not manually edit a generated bundle or deploy a truncated paste.
- Read back the active deployment/version and route, then exercise the deployed
  path. A file in the editor or successful upload is not a published version.

## Diagnose boundaries without changing architecture

Trace DNS/TLS → Worker → Supabase gateway → application handler. Separate a
disconnected browser tool from a server failure. Preserve the pinned upstream,
exact origin checks, body/header limits, session/CSRF controls and Stripe raw-body
signature verification. No server bundle or secret belongs in frontend assets.

The current Worker uses manual redirects and rejects upstream 3xx responses;
an earlier runtime rejected `redirect: "error"`. Inspect the current source and
runtime before reusing either behavior. Do not follow redirects around the
pinned origin. Health `ready:true` proves neither login nor payment settlement.

## Private R2 downloads

- Keep `skager-private-releases` private. Do not enable a public bucket URL to
  bypass entitlement checks or solve a failed download.
- The Supabase signing service needs an **Object Read only**, single-bucket S3
  credential. Upload authority is separate; the portal never needs write/delete
  or account-administration rights.
- An account management token is not an S3 Access Key ID/Secret Access Key pair.
  Check the account and jurisdiction endpoint. R2 object-only permission applies
  to the S3 API, not the management REST API; choose the right API instead of
  expanding permissions to cure an authentication failure.
- Keep credentials in the approved server secret store. Never print values or
  include signed download URLs in public evidence. Do not overwrite unrelated
  secrets or expose key-bearing dashboards in screenshots.
- Upload only selected, qualified release bytes with exact hashes. Verify
  session/entitlement enforcement, short-lived GET expiry, unauthorized refusal
  and downloaded-byte hash. Storage creation or an empty catalog is not a
  qualified download path.

Record source/hash, deployment identity, observed checks and remaining failures.
Reuse retained web bundles and focused checks where inputs permit; no desktop
rebuild is needed. Public payment/download access remains closed pending the
owner's explicit launch decision.

Current provider references: [Workers](https://developers.cloudflare.com/workers/)
and [R2 token permissions](https://developers.cloudflare.com/r2/api/tokens/).
