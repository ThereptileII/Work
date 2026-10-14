# Private checkout configuration diagnosis

Jira SCRUM-43. Exact source `5be97309474a54e463119d65a92fed9a0838f9ac`.

The owner completed email-code verification and the actual authenticated portal
works. Approved TEST checkout returned exact generic `Unavailable`; the existing
handler emits that response before Stripe I/O when configuration validation
fails. The nonsecret origin, price and coupon/promotion binding digests match the
known exact valid values. No secret value was read to diagnose this failure.

The minimal change reuses the same configuration predicates and emits one static
startup marker per isolate. It reports only a finite binding name, `valid`, or
`unknown`; disabled checkout is silent. It never logs values, lengths, hashes,
exceptions, requests or account data. Public responses, request authentication,
CSRF, signature verification and settlement behavior are unchanged. No diagnostic
endpoint or per-request logging was introduced.

## Exact tests and bytes

[CI 38600049762](https://github.com/ThereptileII/Work/actions/runs/38600049762)
passed **208 tests**: 88 Django/PostgreSQL and 120 Edge, no failures or skips,
plus migration and disposable SQL checks. Four new tests cover static-name-only
redaction, unchanged fail-closed behavior/no effects, one startup observation
across multiple requests, and logging/diagnostic failure containment. The agent's
30 focused passes overlap this total.

Original artifact ID `11913247729`, 7,857 bytes, SHA-256
`ee8cbad997decf6631f557eee6474669d85ca253c45e8a7d860bcb43c82f2eac` was downloaded
unchanged and checked against GitHub metadata, byte size and every ZIP CRC.
It and the immutable build manifest remain in the private local evidence folder
`/home/standard/Work/skager-web-commerce-diagnostic-5be97309`.

Server `supabase-index.ts`: 78,865 bytes, SHA-256
`4cd186666730396e16fd8da811f97460f53fa53c2954fc7f604829ab44f49f99`.
Complete editor-copy comparison was byte-identical before deployment and after
reloading the published source. Supabase showed a new deployment timestamp.
The existing frontend SHA-256 remains
`a42d54ba857209cc76cee87a66905ad34502e5d80fc79d1091e3c27931427f0e`, identical to
active Cloudflare `ac34d9d4`; it was not redundantly redeployed.

## Actual runtime result

A controlled checkout attempt after deployment produced the static log entry
at **2026-10-14T16:42:23.289Z**:

```
skager-commerce:configuration-v1:STRIPE_TEST_SECRET_KEY
```

The provider log belongs to function `8fda2b14-f5cf-4bde-bb44-cd27323b6291`.
This identifies the first failed local format predicate only. It does not prove
whether the stored value was copied incorrectly or whether the predicate rejects
a legitimate opaque provider format. No provider authentication was attempted.
The owner has been asked to correct that one binding in Supabase; webhook and
other bindings remain unchanged. The empty correction form is retained privately
for handoff. No key was copied, logged, committed or sent in chat.

## Remaining acceptance

Actual TEST checkout, 100% coupon, verified webhook/active entitlement/refund,
cross-account boundaries and R2 customer download remain unaccepted. No manual
paid grant or test-order fabrication was used. General signup, public payments
and downloads remain closed. The authenticated portal and this server diagnostic
do not qualify the desktop package or public launch.
