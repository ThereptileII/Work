---
name: skager-resend
description: Configure and troubleshoot SKAGER transactional email through Resend and Supabase SMTP. Use for sending-domain DNS, sender setup, missing login emails or code-delivery failures; application authorization belongs to skager-supabase.
---

# SKAGER Resend

Use this for SKAGER's existing email path: **Supabase Auth → Resend SMTP → invited
recipient**. Locate the active web checkout and read
`opennav-x/docs/architecture/supabase-login-flow.md`. Verify current configuration;
historical setup notes do not prove delivery. These are project conventions, not
requirements for unrelated Resend projects.

## Configure the correct layer

- Use an authenticated provider API/CLI when available, otherwise the authorized
  dashboard. Never extract browser tokens or bypass an unavailable browser tool.
- The selected sending domain is `auth.skager.app`. Read Resend's exact required
  DNS records and compare existing Cloudflare records before changing them.
  Preserve unrelated records and incoming mail; do not replace apex MX records.
- Check verified domain, matching sender, key scope and current account limits.
  Do not purchase a plan or rotate a working key merely to troubleshoot delivery.
- SMTP uses `smtp.resend.com`, username `resend`, API key as password. Match the
  configured transport: port 465 uses implicit TLS; 587 uses STARTTLS. Do not
  disable TLS to work around a mismatch.
- Save the SMTP password in **Supabase Authentication → SMTP settings**. An Edge
  Function secret alone does not configure Auth's mail transport. Keep values out
  of Git, chat, logs, shell history and screenshots. Use the approved credential
  channel; if tool policy requires user creation/input, prepare nonsecret fields
  and hand off only that final step. Do not request the key in chat.
- Both **Invitation** and **Magic Link** templates must render `{{ .Token }}` for
  SKAGER's code-entry flow. Preserve signup disabled, email confirmation enabled
  and the application invite allowlist.

## Diagnose once, at the failing layer

1. Use one already-authorized recipient. Trace the application request, Supabase
   Auth response and Resend delivery status at the matching timestamp. Record
   sanitized identifiers/categories, never message bodies, keys or codes.
2. Distinguish accepted, sent, delivered, bounced and application login verified.
   A green domain or SMTP settings save is not an end-to-end login test.
3. If no message was accepted, inspect Auth SMTP/template/sender configuration
   and limits. If accepted but not delivered, inspect provider delivery events
   before sending another message. Resend's SMTP emails appear in its Emails
   view; do not assume a separate SMTP server log is available.
4. If delivery succeeded but a code fails, check the newest challenge, same
   browser/session, expiry and prior attempts. The current SKAGER handler
   consumes a challenge on a verification attempt, including failure. Coordinate
   one fresh request when needed; do not repeatedly resend or reuse an old code.
   The user enters the code in the site, never in chat.
5. Stop retrying when evidence identifies a provider/configuration blocker.
   Correct that layer before another bounded delivery check; do not weaken Auth
   or manually confirm a user to make the test pass.

Use focused email/Auth checks and one authorized live delivery where needed.
This work requires no desktop build. Report separately what was configured,
delivered and actually verified. Use `skager-web-publish` for a coordinated web
publication, not for a mail-only diagnosis.

Consult current primary documentation when changing provider behavior:
[Resend SMTP](https://resend.com/docs/send-with-smtp) and
[Supabase custom SMTP](https://supabase.com/docs/guides/auth/auth-smtp).
