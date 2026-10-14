# Private-beta email-code templates

These are reviewed deployment inputs for Supabase Auth, not application routes
or proof of email delivery. They contain the literal Go-template value
`{{ .Token }}` and no confirmation link, remote image, external font, stylesheet,
script, tracking pixel or recipient data. Inline styles and system fonts keep
the code readable without loading external content.

| Supabase template | Subject | HTML body | Plain-text counterpart |
| --- | --- | --- | --- |
| Invitation | Your SKAGER account code | `invitation.html` | `invitation.txt` |
| Magic Link | Your SKAGER sign-in code | `magic-link.html` | `magic-link.txt` |

The application uses Invitation for a new allowlisted account. Login and
passwordless recovery use the ordinary email OTP/Magic Link template. Both
paths require code entry in the original browser, its pending challenge cookie,
CSRF protection and canonical provider user verification. A provider link must
not substitute for that application flow.

## Provider prerequisites and deployment

On 2026-10-14 the live Supabase UI reported **custom SMTP is required to edit
templates**, and emails still use the default templates. Do not infer that those
defaults expose the code. The owner's email-provider signup/login and SMTP
handoff remain separate prerequisites; preparing these files accepts no terms,
creates no account, configures no sender and changes no provider setting.

1. Complete the owner-authorized email-provider setup and verify its sender and
   required DNS authentication. Enter SMTP credentials directly through the
   provider's secret/configuration interface; never put them in this directory,
   chat, screenshots or a client bundle. Recheck Supabase's actual template
   controls after custom SMTP is configured. Do not upgrade a plan or enable an
   email hook merely to bypass the current prerequisite.
2. Keep public signup disabled, email confirmation enabled, and anonymous sign-in
   disabled. Preserve the existing injected Supabase keys and private SQL/RPC
   permissions. Auth, checkout, portal and downloads remain disabled until their
   respective invited qualification windows are authorized.
3. In the intended project's Invitation and Magic Link editors, save the subjects
   above and the exact corresponding HTML bodies. Preserve `{{ .Token }}` as a
   template expression; do not replace it with a real code. The `.txt` files are
   plain-text counterparts for review or a provider exposing a separate text
   body; they are not an assumed second Supabase editor field.
4. Read back the saved subjects and complete HTML without invoking a preview
   containing a real token. Record the template file hashes and project/template
   names, never an OTP, provider key or private recipient address.
5. For an authorized intended tester, set an exact `SKAGER_INVITE_EMAILS`
   allowlist and enable `SKAGER_AUTH_ENABLED` only for the private rehearsal.
   Leave checkout and downloads off and the release catalog empty. Request one
   invitation from `/auth/signup/`, receive the actual email, and enter its code
   at `/auth/verify/` in the same browser. Confirm the account through the session
   API, then log out. A successful send response alone does not prove delivery.
6. Separately qualify existing-user login/recovery, wrong/replayed codes,
   cross-browser refusal, session expiry and allowlist revocation before opening
   access to additional invited friends. Respect both provider and application
   send limits; do not lower protections to complete a rehearsal.

The ten-minute request wording follows the application's pending challenge,
not a claim that the provider's independent OTP expiry has been changed. Inspect
the actual received HTML and plain-text rendering without retaining its code.
Template upload, SMTP delivery and authentication acceptance remain unperformed
until separately observed. See [the auth contract](../../../../docs/architecture/supabase-login-flow.md)
and [Supabase's SMTP guidance](https://supabase.com/docs/guides/auth/auth-smtp).
