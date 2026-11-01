# SKAGER's public addresses

Decided 2026-11-01 for the public launch. One Cloudflare Worker (`skager-beta-web`)
answers on every address; `web/edge/cloudflare-worker.mjs` decides what each serves.

| Address | Serves |
|---|---|
| `skager.app` | The website: home, features, pricing, apps, guides, FAQ, legal, open source. Account, checkout and API paths redirect (308) to `app.skager.app`, so the site's relative links need no change. |
| `www.skager.app` | Redirects to `skager.app`. |
| `app.skager.app` | Sign-in, the account portal, checkout and welcome, boat linking (`/link/`), Companion sign-in (`/app/authorize/`) and the API. `/` opens the account; website pages redirect to `skager.app`. |
| `beta.skager.app` | The API (`/api/`) and the update feed (`/updates/`, `/artifacts/`), unchanged. Its pages redirect to the new addresses, so old emails, QR codes and Companion's sign-in still land on the right page. |

## Why beta.skager.app stays

- SKAGER on board up to 0.4.0-beta2.10 calls `https://beta.skager.app` for Cloud, and
  accepts a link page only on that address (`CloudProtocol.cpp`). The server answers
  each build on its own address: the HTTP client sends `Origin: https://<host>:443`,
  and `linkUrl()` in `web/edge/cloud.ts` picks `app.skager.app` or `beta.skager.app`.
- Every installed SKAGER trusts updates only from `https://beta.skager.app`
  (`installer/windows/production-update-trust.json`). Moving the feed needs a release
  that trusts the new address first and a long overlap; customers never see it.
- Companion builds from before the move call `https://beta.skager.app/api/companion/*`.

New builds use `app.skager.app`: SKAGER (`kOrigin`), Companion (`SKAGER_API` default).

## Sessions and checkout

Session and CSRF cookies are `__Host-` cookies, bound to one address: a customer signs
in again on `app.skager.app`. Changes are accepted from `app.skager.app` and
`beta.skager.app` (`WEB_ORIGINS` in `web/edge/platform.ts`), never from the website.
Stripe returns the buyer to the address the purchase started on; the Stripe webhook may
stay at `https://beta.skager.app/api/stripe/webhook`. Emails link to `app.skager.app`.

## Switching over (owner)

1. Deploy the `skager-web` Supabase function from this commit (it accepts both addresses
   and answers each SKAGER on its own; safe while only `beta.skager.app` exists).
2. Deploy the Worker: `wrangler deploy --config <build>/deploy/wrangler.jsonc`, built by
   `web/tools/build_cloudflare_beta.py`. Its custom domains create the DNS records for
   `skager.app`, `www.skager.app` and `app.skager.app` (Cloudflare refuses if another
   record already uses one of the names). From then on `beta.skager.app` pages redirect.
3. Check: `https://skager.app/`, `https://app.skager.app/` (sign-in), Link a boat from
   SKAGER 0.4.0-beta2.10 (beta address) and from a new build (app address), Companion
   sign-in on an old and a new build, a test checkout.
4. Release SKAGER and Companion builds made from this commit only after step 2: they
   call `app.skager.app`.

Still to do before going public, separately: search engines (`X-Robots-Tag: noindex`
on every response, `robots.txt`), live Stripe payments, and the skager.app IPv6 setting.
