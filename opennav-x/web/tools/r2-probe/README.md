# Administrative R2 connectivity probe (SCRUM-42)

This is a separate server-only Supabase function named `skager-r2-probe`. It is
not imported by the website runtime, exposed through the Worker, or enabled by
any product flag. Preparing this source does not deploy it or authorize provider
changes. Root owns review and any separately authorized temporary invocation.

A GET to the exact project function path, with no query/body, must carry
`Authorization: Bearer <existing platform service-role key>`. Keep Supabase's
platform JWT verification **ON**. This function additionally requires exact
possession of the platform-injected `SUPABASE_SERVICE_ROLE_KEY`, with digest
comparison. The injected binding must be a valid-time legacy HS256 service-role
key for the selected project and differ from its anon key. Caller JWT claims,
user sessions, anon/publishable keys and an `apikey` header alone never authorize
it. New opaque secret-key formats are deliberately unsupported. No new key,
permission or disabling of verification follows from this tool. Never place an
admin credential in browser JavaScript, chat, logs, screenshots or saved files.

Routing accepts only `/skager-r2-probe` or `/functions/v1/skager-r2-probe`.
The incoming HTTP(S) origin is transport metadata: Supabase's gateway can
rewrite the public URL before forwarding the request. It never selects a
provider endpoint or credential. Project identity remains pinned in the
trusted `SUPABASE_URL` and service binding. The existing website handler uses
the same function-prefix approach. The exact hosted internal URL has not been
observed; source regressions do not establish a successful live invocation.

After authorization, the existing approved R2 bindings must match the fixed
account `7869adcf376c630b175c94729321e016` and bucket
`skager-private-releases`. One header-signed HTTPS **HeadBucket** request uses
`auto` region and a shared five-second signing/fetch deadline. There are no
redirects, retries, caller-selected paths or writes. HEAD consumes zero response
body bytes; unexpected bodies are cancelled and refused. Nothing enumerates
objects or reads contents. No upstream header, body, error or signed request is
returned or logged. A 200 result means only that this exact authenticated
HeadBucket operation succeeded. It proves no download, object custody, release
qualification, entitlement, readiness or public-access acceptance.

Official references checked 2026-10-14:

- [Supabase platform-injected legacy service key](https://supabase.com/docs/guides/functions/secrets).
- [Supabase authorization headers and the separate handler boundary](https://supabase.com/docs/guides/functions/auth-headers).
- [Supabase function-name-prefixed routing](https://supabase.com/docs/guides/functions/routing).
- [Upstream gateway strips the public prefix and proxies to the internal runtime](https://github.com/supabase/supabase/blob/master/docker/volumes/api/kong.yml).
- [Cloudflare R2 S3 compatibility, including HeadBucket and auto region](https://developers.cloudflare.com/r2/api/s3/api/).
- [HeadBucket success and no response body](https://docs.aws.amazon.com/AmazonS3/latest/API/API_HeadBucket.html).

Run focused source checks from repository root:

```sh
node --test opennav-x/web/tools/r2-probe/index.test.ts
```

After committing, bundle into a fresh directory outside the web source:

```sh
python opennav-x/web/tools/r2-probe/build.py --source-commit "$(git rev-parse HEAD)" --output /tmp/skager-r2-probe-reviewed
```

The builder uses the existing pinned `esbuild@0.28.2`, verifies the existing
`aws4fetch@1.0.20` manifest hashes and embeds its complete MIT notice in
`index.ts`. `build-info.json` binds the output hash to the exact clean source.
It reads no credentials and performs no deployment. Only that server artifact
belongs in a separately reviewed Supabase function; never publish it as a static
website asset. Root must record any actual deployment/invocation separately and
retire the temporary diagnostic afterward; mocked tests are not live R2 proof.
