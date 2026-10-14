# SCRUM-40 — private email activation and live checks

On 2026-10-14 the owner authorized one exact private test address. Only that
address was saved in the server allowlist. `SKAGER_AUTH_ENABLED=true` is confined
by that allowlist; checkout, portal and downloads remain off. Supabase public
signup and anonymous sign-in remain disabled; email confirmation is enabled.
No private recipient address, OTP, key, cookie or raw provider log is retained here.

## Provider configuration

- Owner-approved DKIM TXT and the two documented DNS-only CNAMEs for
  `auth.skager.app` were saved. Resend reports the domain verified, with Enforced
  TLS, receiving off and no configured tracking. Existing apex/mail routing was
  not replaced.
- Owner performed the credential creation/input handoff. A fresh Supabase SMTP
  page confirmed the persisted enabled state, `smtp.resend.com` and sender name
  SKAGER. This was configuration evidence, not accepted credentials/delivery.
- Invitation and Magic Link subjects and exact HTML bodies were saved. Complete
  editor readback after reload matched source. HTML hashes are
  `44f49b0918eacb33219a47016ee95dd458406cf0b4be299dc0e9021ceea82d80`
  and `882b1b795d32183cdf77a86757ddcd6ce00c3f2c5fc21866b31811e3e0dcd0ab`.
  The templates retain literal `{{ .Token }}`, not a live credential.

## Exact deployed candidates

Frontend source `a6a1768d01963388416d48ab1d51dd191d9288db`, Cloudflare
`f5508044` Active, 503,124 bytes,
SHA-256 `68743a3eb86ef6545c6119ddd3a8cf7e9aef45eec28f43c6176d75fe7bd2bbbd`.
Full pre-deploy editor readback matched; live `auth.js` matched its 3,145-byte
retained file, SHA-256
`16229e53e7a98dc41c4cb04943f9859052e723ff83472a0839268c3535a4d01e`.
`preview-provenance.json` remains local evidence, deliberately excluded from
public assets by the builder; its public URL returned 404 as designed.

The admin-invitation correction was packaged from exact
`004a8472cc49e11c64bcd5ebffabe52afe791966`. The frontend is byte-identical and
was not deployed again. Supabase's new server bundle is 77,827 bytes,
SHA-256 `e4341712a7a8e5d6457dad1d6c831fbd1acf5aa3f5684196e201dd9ce3dbecf5`.
Full editor bytes matched before deployment and after fresh navigation. The
function showed its fresh deployment time; the previously approved legacy-JWT
gateway setting remains off, with application session/CSRF/signature checks intact.

[CI 38538024121, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38538024121)
passes 88 Django + 104 Edge tests and disposable SQL boundary checks. Original
artifact 11881172696 is 7,277 bytes, SHA-256
`c134edca965faa5b904085418afaa779acd1bdf9ceea35c8fc024487355b52c2`.
Provider metadata, original ZIP and every member CRC were verified. This evidence
is separate from live-provider acceptance and the Windows product candidate.

## Live outcomes and remaining gate

1. First authorized normal signup, 08:45:15 UTC: two throttle RPCs returned 200;
   `/auth/v1/invite` returned 403. Auth reported a malformed bearer token.
   Resend listed no sent email. Generic client success deliberately conceals
   account/provider state and did not prove delivery.
2. After the SDK-aligned admin-header correction, the retry at 11:58:27 Stockholm
   time reached SMTP but returned `535 Authentication credentials invalid`.
   The fallback `/otp` returned 422, `Signups not allowed for otp`, correctly
   preserving disabled public signup. Resend still listed no sent email.

The owner must correct the SMTP credential in the provider UI. Do not read it,
log it, disable email confirmation, open public signup or fake a confirmed user
to work around this failure. Actual delivery, owner-entered OTP verification,
login/recovery/logout and session-isolation acceptance remain pending. No
payment, entitlement, download, public launch or desktop acceptance follows.
