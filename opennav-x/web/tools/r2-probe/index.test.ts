import assert from 'node:assert/strict';
import test, { mock } from 'node:test';
import { createHash, createHmac } from 'node:crypto';
import { readFile } from 'node:fs/promises';
import { AwsClient } from '../../edge/vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';
import { handleProbe, DEADLINE_MS } from './index.ts';

const origin = 'https://kelwgubkuvditalsbqpr.supabase.co';
const target = 'https://7869adcf376c630b175c94729321e016.r2.cloudflarestorage.com/skager-private-releases';
function jwt(patch: Record<string, unknown> = {}, header = { alg: 'HS256', typ: 'JWT' }) {
  const encode = (value: unknown) => Buffer.from(JSON.stringify(value)).toString('base64url');
  return [encode(header), encode({ iss: 'supabase', ref: 'kelwgubkuvditalsbqpr', role: 'service_role',
    iat: 1, exp: 4102444800, ...patch }), 'a'.repeat(43)].join('.');
}
const service = jwt();
const env = { SUPABASE_URL: origin, SUPABASE_SERVICE_ROLE_KEY: service, SUPABASE_ANON_KEY: jwt({ role: 'anon' }),
  SKAGER_DOWNLOAD_BACKEND: 'r2', R2_ACCOUNT_ID: '7869adcf376c630b175c94729321e016',
  R2_DOWNLOAD_BUCKET: 'skager-private-releases', R2_ACCESS_KEY_ID: 'f'.repeat(32), R2_SECRET_ACCESS_KEY: 'e'.repeat(64) };
function request(token: string | null = service, path = '/functions/v1/skager-r2-probe', init: RequestInit = {}) {
  return new Request(origin + path, { ...init, headers: { ...(token === null ? {} : { Authorization: 'Bearer ' + token }), ...init.headers } });
}
const noIO: typeof fetch = async () => { assert.fail('Denied request reached R2'); };
async function result(req = request(), configured: Record<string, string | undefined> = env, transport: typeof fetch = noIO) {
  const response = await handleProbe(req, configured, transport);
  return { response, body: await response.json() };
}

// Independently verify the actual SDK's HEADER signature using Node/OpenSSL.
function signatureFor(req: Request, method = req.method) {
  const url = new URL(req.url), date = req.headers.get('x-amz-date')!;
  const scope = `${date.slice(0, 8)}/auto/s3/aws4_request`;
  const names = req.headers.get('authorization')!.match(/SignedHeaders=([^,]+)/)![1];
  const headers = names.split(';').map(name => `${name}:${name === 'host' ? url.host : req.headers.get(name)}\n`).join('');
  const canonical = [method, url.pathname, '', headers, names, 'UNSIGNED-PAYLOAD'].join('\n');
  const string = ['AWS4-HMAC-SHA256', date, scope, createHash('sha256').update(canonical).digest('hex')].join('\n');
  let key = Buffer.from('AWS4' + env.R2_SECRET_ACCESS_KEY);
  for (const part of [date.slice(0, 8), 'auto', 's3', 'aws4_request']) key = createHmac('sha256', key).update(part).digest();
  return createHmac('sha256', key).update(string).digest('hex');
}

test('genuine injected service-key possession makes exactly one fixed authenticated HEAD with no caller metadata', async () => {
  let calls = 0;
  const { response, body } = await result(request(service, '/functions/v1/skager-r2-probe', { headers: {
    apikey: 'caller-untrusted', 'X-Endpoint': 'https://untrusted.example.test', 'X-Amz-Security-Token': 'caller-secret',
  } }), env, async input => {
    calls++;
    const signed = input as Request;
    assert.equal(signed.url, target); assert.equal(signed.method, 'HEAD'); assert.equal(signed.body, null);
    assert.equal(signed.redirect, 'error'); assert.equal(signed.cache, 'no-store');
    assert.equal(signed.headers.get('apikey'), null); assert.equal(signed.headers.get('x-endpoint'), null);
    assert.equal(signed.headers.get('x-amz-security-token'), null);
    const authorization = signed.headers.get('authorization')!;
    assert.match(authorization, /^AWS4-HMAC-SHA256 Credential=ffffffffffffffffffffffffffffffff\/[0-9]{8}\/auto\/s3\/aws4_request,/);
    assert.equal(authorization.split('Signature=')[1], signatureFor(signed));
    for (const method of ['GET', 'PUT', 'POST', 'DELETE']) assert.notEqual(authorization.split('Signature=')[1], signatureFor(signed, method));
    assert.equal([...signed.headers].some(([, value]) => value.includes(service)), false);
    return new Response(null, { status: 200 });
  });
  assert.equal(calls, 1); assert.equal(response.status, 200);
  assert.deepEqual(body, { probe: 'r2-head-bucket', authenticated: true });
  assert.equal(response.headers.get('Cache-Control'), 'private, no-store');
  assert.equal(JSON.stringify(body).includes(env.R2_ACCESS_KEY_ID), false);
});

test('missing, anon, publishable, user, forged role and other project credentials cannot reach R2', async () => {
  for (const token of [null, '', env.SUPABASE_ANON_KEY, 'sb_publishable_fixture', 'sb_secret_fixture',
    jwt({ role: 'authenticated' }), jwt({ ref: 'other-project' }), jwt({ exp: 1 }), service.slice(0, -1) + 'b', service + ', ' + service]) {
    const denied = await result(request(token));
    assert.equal(denied.response.status, 401);
    assert.deepEqual(denied.body, { error: 'Administrative credential required.' });
  }
  assert.equal((await result(request(null, '/functions/v1/skager-r2-probe', { headers: { apikey: service } }))).response.status, 401);
});

test('invalid trusted service binding refuses instead of accepting caller JWT role claims', async () => {
  for (const patch of [
    { SUPABASE_URL: 'https://other.supabase.co' }, { SUPABASE_SERVICE_ROLE_KEY: undefined },
    { SUPABASE_SERVICE_ROLE_KEY: 'sb_secret_fixture' }, { SUPABASE_SERVICE_ROLE_KEY: jwt({ role: 'anon' }) },
    { SUPABASE_SERVICE_ROLE_KEY: jwt({ role: 'authenticated' }) }, { SUPABASE_SERVICE_ROLE_KEY: jwt({ ref: 'other-project' }) },
    { SUPABASE_SERVICE_ROLE_KEY: jwt({ exp: 1 }) }, { SUPABASE_SERVICE_ROLE_KEY: jwt({ iat: 4102444800 }) },
    { SUPABASE_SERVICE_ROLE_KEY: jwt({}, { alg: 'none', typ: 'JWT' }) }, { SUPABASE_ANON_KEY: service },
  ]) assert.equal((await result(request(patch.SUPABASE_SERVICE_ROLE_KEY || service), { ...env, ...patch })).response.status, 503);
});

test('caller paths, query, methods and bodies cannot select an operation', async () => {
  for (const path of ['/functions/v1/skager-r2-probe?bucket=other', '/functions/v1/skager-r2-probe/other',
    '/functions/v1/skager-r2-probe/', '/api/health', '/api/portal/downloads', '/skager-r2-probe?method=PUT']) {
    assert.equal((await result(request(service, path))).response.status, 404);
  }
  assert.equal((await result(new Request('https://other.supabase.co/skager-r2-probe'))).response.status, 401);
  for (const method of ['POST', 'PUT', 'DELETE', 'HEAD', 'OPTIONS']) {
    assert.equal((await result(request(service, '/skager-r2-probe', { method }))).response.status, 405);
  }
  const req = request();
  let cancelled = false;
  Object.defineProperty(req, 'body', { value: new ReadableStream({ cancel() { cancelled = true; } }) });
  assert.equal((await result(req)).response.status, 400); assert.equal(cancelled, true);
});

test('gateway-rewritten HTTP origins preserve fixed routing and exact service-key authorization', async () => {
  for (const runtimeOrigin of ['http://functions:9000', 'http://localhost:9999', origin]) {
    for (const path of ['/skager-r2-probe', '/functions/v1/skager-r2-probe']) {
      let calls = 0;
      const transport: typeof fetch = async input => {
        calls++;
        assert.equal((input as Request).url, target);
        assert.equal((input as Request).method, 'HEAD');
        return new Response(null, { status: 200 });
      };
      const make = (key: string) => new Request(runtimeOrigin + path, { headers: { Authorization: 'Bearer ' + key } });
      assert.equal((await result(make(service), env, transport)).response.status, 200);
      assert.equal(calls, 1);
      for (const key of [env.SUPABASE_ANON_KEY, jwt({ role: 'authenticated' }), service.slice(0, -1) + 'b']) {
        assert.equal((await result(make(key), env, noIO)).response.status, 401);
      }
      assert.equal((await result(make(service), { ...env, SUPABASE_URL: 'https://other.supabase.co' })).response.status, 503);
    }
  }
});

test('missing or changed R2 bindings cannot trigger provider I/O', async () => {
  for (const patch of [{ SKAGER_DOWNLOAD_BACKEND: undefined }, { SKAGER_DOWNLOAD_BACKEND: 'supabase' },
    { R2_ACCOUNT_ID: 'other' }, { R2_DOWNLOAD_BUCKET: 'other' }, { R2_ACCESS_KEY_ID: 'wrong' },
    { R2_SECRET_ACCESS_KEY: undefined }, { R2_SECRET_ACCESS_KEY: 'x'.repeat(64) }]) {
    assert.equal((await result(request(), { ...env, ...patch })).response.status, 503);
  }
});

test('only exact administrators see finite binding/config checks and never their values', async () => {
  const binding = await result(request(), { ...env, SUPABASE_URL: 'https://private-value.example.test' });
  assert.deepEqual(binding.body, { error: 'Administrative R2 probe unavailable.', phase: 'trusted-binding', checks: { project: false } });
  for (const [patch, failed] of [
    [{ SKAGER_DOWNLOAD_BACKEND: undefined }, 'backend'], [{ R2_ACCOUNT_ID: 'private-account' }, 'account'],
    [{ R2_DOWNLOAD_BUCKET: 'private-bucket' }, 'bucket'], [{ R2_ACCESS_KEY_ID: undefined }, 'accessKeyFormat'],
    [{ R2_SECRET_ACCESS_KEY: 'private-secret' }, 'secretKeyFormat'],
  ] as const) {
    const configured = { ...env, ...patch };
    const observed = await result(request(), configured);
    assert.deepEqual(observed.body, { error: 'Administrative R2 probe unavailable.', phase: 'config',
      checks: { backend: true, account: true, bucket: true, accessKeyFormat: true, secretKeyFormat: true, [failed]: false } });
    assert.doesNotMatch(JSON.stringify(observed.body), /private-account|private-bucket|private-secret/);
    for (const token of [null, env.SUPABASE_ANON_KEY, jwt({ role: 'authenticated' })]) {
      const denied = await result(request(token), configured);
      assert.equal(denied.response.status, 401);
      assert.deepEqual(denied.body, { error: 'Administrative credential required.' });
    }
  }
  for (const key of [undefined, env.SUPABASE_ANON_KEY, jwt({ ref: 'other-project' })]) {
    const refused = await result(request(), { ...env, SUPABASE_SERVICE_ROLE_KEY: key });
    assert.deepEqual(refused.body, { error: 'Administrative R2 probe unavailable.' });
  }
});

test('SDK failure and missing Request cache support expose only finite sign diagnostics to admin', async () => {
  const original = AwsClient.prototype.sign;
  mock.method(AwsClient.prototype, 'sign', async () => { throw Error(`${service} ${env.R2_SECRET_ACCESS_KEY}`); });
  try { assert.deepEqual((await result()).body, { error: 'Administrative R2 probe unavailable.', phase: 'sign' }); }
  finally { mock.restoreAll(); }
  mock.method(AwsClient.prototype, 'sign', async function (...args) {
    const signed = await original.apply(this, args);
    Object.defineProperty(signed, 'cache', { value: undefined });
    return signed;
  });
  try {
    assert.deepEqual((await result()).body, { error: 'Administrative R2 probe unavailable.', phase: 'sign', checks: {
      destination: true, headMethod: true, bodyAbsent: true, redirectsBlocked: true, cacheDisabled: false,
      authorizationPresent: true, callerApiKeyAbsent: true, sessionTokenAbsent: true,
    } });
  } finally { mock.restoreAll(); }
});

test('R2 error statuses and redirects are generic, never retried or forwarded', async () => {
  for (const status of [204, 301, 302, 307, 400, 401, 403, 404, 429, 500, 503]) {
    let calls = 0;
    const { response, body } = await result(request(), env, async () => {
      calls++; return new Response(null, { status, headers: { Location: 'https://untrusted.example.test/?secret=credential', 'X-Secret': env.R2_SECRET_ACCESS_KEY } });
    });
    assert.equal(calls, 1); assert.equal(response.status, 503);
    assert.deepEqual(body, { error: 'Administrative R2 probe unavailable.', phase: 'provider', providerStatus: status });
    assert.equal(response.headers.has('Location'), false); assert.equal(response.headers.has('X-Secret'), false);
  }
});

test('unexpected provider body is cancelled without reading object names or contents', async () => {
  let cancelled = false;
  const body = new ReadableStream({ pull() { assert.fail('Unexpected provider body was read'); },
    cancel() { cancelled = true; } }, { highWaterMark: 0 });
  const { response } = await result(request(), env, async () => new Response(body, { status: 200 }));
  assert.equal(response.status, 503); assert.equal(cancelled, true);
});

test('transport exceptions are sanitized and no credential or signed request leaves the server', async () => {
  let calls = 0;
  const { response, body } = await result(request(), env, async () => {
    calls++; throw Error(`${service} ${env.R2_SECRET_ACCESS_KEY} ${target}?signature=private`);
  });
  assert.equal(calls, 1); assert.equal(response.status, 503);
  assert.deepEqual(body, { error: 'Administrative R2 probe unavailable.', phase: 'fetch' });
});

test('shared signing/network deadline aborts a stalled request and returns once', async () => {
  let entered!: () => void;
  const started = new Promise<void>(resolve => { entered = resolve; });
  let signal!: AbortSignal, calls = 0;
  mock.timers.enable({ apis: ['setTimeout'] });
  try {
    const pending = result(request(), env, async input => {
      calls++; signal = (input as Request).signal; entered(); return new Promise<Response>(() => {});
    });
    await started;
    mock.timers.tick(DEADLINE_MS);
    const { response, body } = await pending;
    assert.equal(response.status, 503); assert.equal(signal.aborted, true); assert.equal(calls, 1);
    assert.deepEqual(body, { error: 'Administrative R2 probe unavailable.', phase: 'timeout' });
  } finally { mock.timers.reset(); }
});

test('a changed SDK request cannot redirect credentials or select another method', async () => {
  mock.method(AwsClient.prototype, 'sign', async () => new Request('https://untrusted.example.test', { method: 'GET' }));
  try { assert.equal((await result()).response.status, 503); }
  finally { mock.restoreAll(); }
});

test('late transport response is cancelled without replacing the recorded timeout phase', async () => {
  let entered!: () => void, finish!: (response: Response) => void;
  const started = new Promise<void>(resolve => { entered = resolve; });
  let cancelled = false;
  mock.timers.enable({ apis: ['setTimeout'] });
  try {
    const pending = result(request(), env, async () => { entered(); return new Promise<Response>(resolve => { finish = resolve; }); });
    await started;
    mock.timers.tick(DEADLINE_MS);
    const observed = await pending;
    assert.deepEqual(observed.body, { error: 'Administrative R2 probe unavailable.', phase: 'timeout' });
    finish(new Response(new ReadableStream({ pull() { assert.fail('Late response was read'); },
      cancel() { cancelled = true; } }, { highWaterMark: 0 }), { status: 200 }));
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(cancelled, true);
    assert.deepEqual(observed.body, { error: 'Administrative R2 probe unavailable.', phase: 'timeout' });
  } finally { mock.timers.reset(); }
});

test('deadline during signing prevents later network I/O when the signer completes', async () => {
  let entered!: () => void, finish!: (value: Request) => void;
  const started = new Promise<void>(resolve => { entered = resolve; });
  const signing = new Promise<Request>(resolve => { finish = resolve; });
  mock.method(AwsClient.prototype, 'sign', async () => { entered(); return signing; });
  mock.timers.enable({ apis: ['setTimeout'] });
  try {
    const pending = result();
    await started;
    mock.timers.tick(DEADLINE_MS);
    const observed = await pending;
    assert.equal(observed.response.status, 503);
    assert.deepEqual(observed.body, { error: 'Administrative R2 probe unavailable.', phase: 'timeout' });
    finish(new Request(target, { method: 'HEAD', redirect: 'error', cache: 'no-store' }));
    await signing;
  } finally { mock.restoreAll(); mock.timers.reset(); }
});

test('a successful response from another location or after a redirect is refused', async () => {
  for (const fields of [{ url: 'https://untrusted.example.test/' }, { redirected: true }]) {
    const response = new Response(null, { status: 200 });
    for (const [name, value] of Object.entries(fields)) Object.defineProperty(response, name, { value });
    assert.equal((await result(request(), env, async () => response)).response.status, 503);
  }
});

test('standalone diagnostic is absent from runtime imports and retains complete pinned SDK notice', async () => {
  for (const file of ['../../edge/index.ts', '../../edge/cloudflare-worker.mjs']) {
    assert.doesNotMatch(await readFile(new URL(file, import.meta.url), 'utf8'), /r2-probe|handleProbe/);
  }
  const base = new URL('../../edge/vendor/aws4fetch-1.0.20/', import.meta.url);
  const manifest = JSON.parse(await readFile(new URL('manifest.json', base), 'utf8'));
  for (const [name, expected] of Object.entries(manifest.files)) {
    assert.equal(createHash('sha256').update(await readFile(new URL(name, base))).digest('hex'), expected);
  }
  assert.match(await readFile(new URL('LICENSE', base), 'utf8'), /Permission is hereby granted/);
});
