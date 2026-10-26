# SKAGER Companion end-to-end and preview

Both use the real request path: Cloudflare Worker → `skager-web` edge router →
RPCs executed by `psql` in a **disposable** PostgreSQL with `web/edge/*.sql`
applied (PostgREST's named-argument call is emulated in `stack.ts`). Push
providers are replaced by a recorder.

```sh
# Apply the schema (as in .github/workflows/skager-companion.yml), then:
SKAGER_E2E_PSQL="-h <socket-dir> -p <port> -U <user> -d scrum94_schema" node --test tests/e2e/*.e2e.test.ts
# Optional: SKAGER_E2E_CLOUD_TESTS=<build>/cloud_tests uses the C++ app's own encoder for boat payloads.

# Browser preview with a synthetic PREVIEW FIXTURE boat (never product data):
SKAGER_PREVIEW=1 npm run build
SKAGER_E2E_PSQL="…" node tests/e2e/preview.ts 8787   # prints the URL with a preview phone token
# Preview-only URL parameters: ?tab=overview|map|anchor|history|alarms&theme=day|dusk|night
```
