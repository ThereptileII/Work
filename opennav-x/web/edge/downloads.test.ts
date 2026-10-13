import test, { mock } from 'node:test';
import assert from 'node:assert/strict';
import { createDownloadHandler, handleDownloads, DOWNLOAD_BUCKET, DOWNLOAD_TTL_SECONDS } from './downloads.ts';
import type { DownloadRelease } from './downloads.ts';
import { PUBLIC_ORIGIN, SUPABASE_ORIGIN } from './platform.ts';

// Inert source-contract fixture only: no real qualified package or Storage object.
const release: DownloadRelease = { id: 'fixture-beta-1', sourceCommit: 'a'.repeat(40), version: '1.0.0-beta.1',
  channel: 'staging', qualification: { record: 'docs/evidence/inert-fixture.json', sha256: 'b'.repeat(64) },
  artifacts: [ ['installer', 'Setup.exe'], ['recovery', 'Recovery.zip'], ['checksums', 'SHA256SUMS.txt'],
    ['source', 'Source.zip'], ['release-notes', 'Notes.md'] ].map(([id, name], index) => ({
      id: id as any, name, bytes: 100 + index, sha256: String(index + 1).repeat(64),
    })),
};
const accountId = '11111111-1111-4111-8111-111111111111';
const secret = 'synthetic-server-only-key';
const csrf = 'c'.repeat(64);
const entitlement = { status: 'active', environment: 'staging', product_code: 'skager-beta' };
const encode = (value: unknown) => Buffer.from(JSON.stringify(value)).toString('base64url');
function request(method = 'GET', body?: string, headers: Record<string, string> = {}, suffix = '') {
  return new Request(PUBLIC_ORIGIN + '/api/portal/downloads' + suffix, { method,
    headers: { Origin: PUBLIC_ORIGIN, Cookie: `__Host-skager_csrf=${csrf}`, 'X-CSRF-Token': csrf,
      'Content-Type': 'application/json', ...headers }, ...(body !== undefined ? { body } : {}) });
}
function signedURL(target: string, mutate: (payload: any) => void = () => {}) {
  const path = new URL(target).pathname.replace('/storage/v1/object/sign/', '');
  const now = Math.floor(Date.now() / 1000);
  const payload = { url: path, scope: 'download', iat: now, exp: now + DOWNLOAD_TTL_SECONDS };
  mutate(payload);
  return '/object/sign/' + path + '?token=' + encode({ alg: 'HS256' }) + '.' + encode(payload) + '.' + 'd'.repeat(43);
}
function fixture(options: any = {}) {
  const calls = { accounts: 0, rpc: [] as any[], provider: [] as any[] };
  const ctx = { env: { SKAGER_DOWNLOADS_ENABLED: 'true', PORTAL_ENABLED: 'true', STRIPE_TEST_ENVIRONMENT: 'staging',
    SUPABASE_URL: SUPABASE_ORIGIN, SUPABASE_SERVICE_ROLE_KEY: secret, ...options.env },
    getAccount: async () => { calls.accounts++; return options.account === undefined ? { id: accountId } : options.account; },
    rpc: async (name: string, args: any) => {
      calls.rpc.push([name, args]);
      if (options.rpc) return options.rpc(calls.rpc.length);
      return { entitlement: options.entitlement === undefined ? entitlement : options.entitlement, orders: [] };
    },
    fetch: async (target: string, init: RequestInit) => {
      calls.provider.push([target, init]);
      if (options.provider) return options.provider(target, init);
      return new Response(JSON.stringify({ signedURL: signedURL(target) }), { headers: { 'Content-Type': 'application/json' } });
    },
  };
  return { calls, ctx: ctx as any };
}
const selection = JSON.stringify({ releaseId: release.id, artifactId: 'installer' });

test('downloads default off and empty; configuration never invents a qualified release', async () => {
  for (const env of [{ SKAGER_DOWNLOADS_ENABLED: '' }, { SKAGER_DOWNLOADS_ENABLED: 'TRUE' },
    { PORTAL_ENABLED: 'false' }, { STRIPE_TEST_ENVIRONMENT: 'production' }]) {
    const { ctx, calls } = fixture({ env });
    assert.equal((await handleDownloads(request(), ctx))?.status, 404);
    assert.equal(calls.accounts, 0); assert.deepEqual(calls.rpc, []); assert.deepEqual(calls.provider, []);
  }
  const { ctx, calls } = fixture();
  const empty = await handleDownloads(request(), ctx);
  assert.deepEqual(await empty?.json(), { available: false,
    reason: 'Release qualification and secure download delivery are pending.', releases: [] });
  assert.equal((await handleDownloads(request('POST', selection), ctx))?.status, 404);
  assert.deepEqual(calls.provider, []);
});

test('session and exact current entitlement gate both listing and signing, including zero-payment active access', async () => {
  const handle = createDownloadHandler([release]);
  for (const options of [{ account: null }, { account: { id: 'forged' } }, { entitlement: null },
    { entitlement: { ...entitlement, status: 'refunded' } }, { entitlement: { ...entitlement, status: 'suspended' } },
    { entitlement: { ...entitlement, environment: 'production' } },
    { entitlement: { ...entitlement, product_code: 'unrelated' } }]) {
    for (const method of ['GET', 'POST']) {
      const { ctx, calls } = fixture(options);
      const response = await handle(request(method, method === 'POST' ? selection : undefined), ctx);
      assert.equal(response?.status, options.account !== undefined ? 401 : 403);
      assert.deepEqual(calls.provider, []);
    }
  }
  const { ctx, calls } = fixture();
  const response = await handle(request(), ctx);
  const data = await response?.json();
  assert.equal(data.available, true); assert.equal(data.releases[0].sourceCommit, release.sourceCommit);
  assert.deepEqual(data.releases[0].artifacts, release.artifacts);
  assert.equal(JSON.stringify(data).includes('qualification'), false);
  assert.deepEqual(calls.rpc, [['commerce_portal_access', { p_account_id: accountId }]]);
  // The authority is active entitlement, never a payment amount or order list.
  assert.deepEqual(calls.provider, []);
});

test('server catalog is bounded, complete, staging-only and snapshotted against caller mutation', async () => {
  for (const catalog of [Array(17).fill(release), [release, release],
    [{ ...release, id: '../path' }], [{ ...release, channel: 'production' }],
    [{ ...release, sourceCommit: 'not-qualified' }], [{ ...release, artifacts: release.artifacts.slice(1) }],
    [{ ...release, qualification: { record: '../../outside', sha256: 'b'.repeat(64) } }],
    [{ ...release, artifacts: release.artifacts.map((item, index) => index ? item : { ...item, name: '../Setup.exe' }) }],
    [{ ...release, artifacts: release.artifacts.map((item, index) => index ? item : { ...item, bytes: 0 }) }],
    [{ ...release, artifacts: release.artifacts.map((item, index) => index ? item : { ...item, sha256: 'bad' }) }]]) {
    assert.throws(() => createDownloadHandler(catalog as any));
  }
  const input = structuredClone(release);
  const handle = createDownloadHandler([input]);
  input.artifacts[0].name = 'Changed.exe'; input.sourceCommit = 'f'.repeat(40);
  const { ctx } = fixture();
  const response = await handle(request(), ctx);
  assert.deepEqual((await response?.json()).releases[0].artifacts, release.artifacts);
});

test('selectors reject arbitrary URL, path, account, duplicate fields, oversized data and CSRF bypass before Storage', async () => {
  const handle = createDownloadHandler([release]);
  for (const raw of ['{}', '[]', '{"releaseId":"fixture-beta-1","releaseId":"other","artifactId":"installer"}',
    JSON.stringify({ releaseId: release.id, artifactId: 'installer', accountId }),
    JSON.stringify({ releaseId: release.id, artifactId: 'installer', url: 'https://evil.invalid' }),
    JSON.stringify({ releaseId: release.id, artifactId: '../source' }), 'x'.repeat(1025)]) {
    const { ctx, calls } = fixture();
    assert.equal((await handle(request('POST', raw), ctx))?.status, 400);
    assert.deepEqual(calls.provider, []);
  }
  for (const headers of [{ Origin: 'https://evil.invalid' }, { 'X-CSRF-Token': '' },
    { Cookie: `__Host-skager_csrf=${csrf}; __Host-skager_csrf=${csrf}` }]) {
    const { ctx, calls } = fixture();
    assert.equal((await handle(request('POST', selection, headers), ctx))?.status, 403);
    assert.equal(calls.accounts, 0); assert.deepEqual(calls.provider, []);
  }
  const { ctx, calls } = fixture();
  assert.equal((await handle(request('GET', undefined, {}, '?account=other'), ctx))?.status, 400);
  assert.equal((await handle(request('POST', JSON.stringify({ releaseId: 'unknown', artifactId: 'source' })), ctx))?.status, 404);
  assert.equal((await handle(request('DELETE'), ctx))?.status, 405);
  assert.deepEqual(calls.provider, []);
});

test('each supported artifact signs only its immutable server object with private no-cache metadata', async () => {
  const handle = createDownloadHandler([release]);
  for (const artifact of release.artifacts) {
    const { ctx, calls } = fixture();
    const response = await handle(request('POST', JSON.stringify({ releaseId: release.id, artifactId: artifact.id })), ctx);
    assert.equal(response?.status, 200);
    const data = await response?.json();
    const target = `${SUPABASE_ORIGIN}/storage/v1/object/sign/${DOWNLOAD_BUCKET}/staging/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`;
    assert.equal(calls.provider.length, 1); assert.equal(calls.provider[0][0], target);
    const init = calls.provider[0][1];
    assert.equal(init.method, 'POST'); assert.equal(init.redirect, 'manual');
    assert.deepEqual(JSON.parse(init.body), { expiresIn: 60 });
    assert.equal(init.headers.Authorization, `Bearer ${secret}`); assert.equal(init.headers.apikey, secret);
    assert.ok(init.signal instanceof AbortSignal);
    const link = new URL(data.url);
    assert.equal(link.origin, SUPABASE_ORIGIN); assert.equal(link.pathname, new URL(target).pathname);
    assert.equal(link.searchParams.get('download'), artifact.name);
    assert.equal(data.sha256, artifact.sha256); assert.equal(data.bytes, artifact.bytes);
    assert.equal(calls.accounts, 2); assert.equal(calls.rpc.length, 2);
    assert.match(response!.headers.get('Cache-Control')!, /private, no-store/);
    assert.equal(response!.headers.get('Referrer-Policy'), 'no-referrer');
    assert.equal(JSON.stringify(data).includes(secret), false);
  }
});

test('logout, refund or suspension while signing prevents the signed bearer from being returned', async () => {
  const handle = createDownloadHandler([release]);
  for (const status of ['refunded', 'suspended']) {
    const { ctx, calls } = fixture({ rpc: async (count: number) => ({ entitlement: count === 1 ? entitlement : { ...entitlement, status } }) });
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 403); assert.equal(calls.provider.length, 1);
    assert.equal((await response?.text())?.includes('token='), false);
  }
  const { ctx, calls } = fixture();
  ctx.getAccount = async () => ++calls.accounts === 1 ? { id: accountId } : null;
  assert.equal((await handle(request('POST', selection), ctx))?.status, 401);
  const switched = fixture();
  switched.ctx.getAccount = async () => ({ id: ++switched.calls.accounts === 1 ? accountId : '22222222-2222-4222-8222-222222222222' });
  assert.equal((await handle(request('POST', selection), switched.ctx))?.status, 401);
});

test('provider URL, token binding, expiry and redirect failures are generic and leak no vendor details', async () => {
  const handle = createDownloadHandler([release]);
  const badPayloads = [ (p: any) => { p.url = 'other-bucket/private'; }, (p: any) => { p.exp = p.iat + 61; },
    (p: any) => { p.exp = p.iat - 1; }, (p: any) => { p.iat -= 10; }, (p: any) => { p.scope = 'upload'; } ];
  const providers = [
    async () => { throw new Error(secret); },
    async () => new Response(secret, { status: 302, headers: { Location: 'https://evil.invalid' } }),
    async () => new Response(secret, { status: 503 }),
    async () => new Response('{"signedURL":"https://evil.invalid/token"}', { headers: { 'Content-Type': 'application/json' } }),
    async () => new Response('{"signedURL":"/object/sign/other/file?token=private"}', { headers: { 'Content-Type': 'application/json' } }),
    async (target: string) => new Response(JSON.stringify({ signedURL: signedURL(target) + '&token=duplicate' }), { headers: { 'Content-Type': 'application/json' } }),
    ...badPayloads.map(mutate => async (target: string) => new Response(JSON.stringify({ signedURL: signedURL(target, mutate) }), { headers: { 'Content-Type': 'application/json' } })),
    async () => new Response('x'.repeat(16385), { headers: { 'Content-Type': 'application/json' } }),
    async () => new Response('{"signedURL":"one","signedURL":"two"}', { headers: { 'Content-Type': 'application/json' } }),
    async () => new Response(new Uint8Array([0xff]), { headers: { 'Content-Type': 'application/json' } }),
    async (target: string) => new Response(JSON.stringify({ signedURL: signedURL(target) }), { headers: { 'Content-Type': 'text/html' } }),
    async (target: string) => new Response(JSON.stringify({ signedURL: signedURL(target) }), { headers: { 'Content-Type': 'application/json', 'Content-Encoding': 'gzip' } }),
  ];
  for (const provider of providers) {
    const { ctx, calls } = fixture({ provider });
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 503);
    assert.equal(await response?.text(), '{"error":"Downloads are temporarily unavailable. Please try again."}');
    assert.equal(calls.rpc.length, 1);
    assert.equal(JSON.stringify([...response!.headers]).includes(secret), false);
  }
});

test('wrong project, missing key and database uncertainty fail closed before signing', async () => {
  const handle = createDownloadHandler([release]);
  for (const options of [{ env: { SUPABASE_URL: 'https://evil.invalid' } },
    { env: { SUPABASE_SERVICE_ROLE_KEY: '' } }, { rpc: async () => { throw Error(secret); } },
    { rpc: async () => ({ orders: [] }) }, { rpc: async () => 'active' }]) {
    const { ctx, calls } = fixture(options);
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 503); assert.deepEqual(calls.provider, []);
    assert.equal((await response?.text())?.includes(secret), false);
  }
});

test('reviewed historical entries keep their own commit, version, artifact hashes and Storage identity', async () => {
  const historical = { ...structuredClone(release), id: 'fixture-beta-older', sourceCommit: 'e'.repeat(40), version: '0.9.0-beta.1' };
  const handle = createDownloadHandler([release, historical]);
  const { ctx, calls } = fixture();
  const listed = await handle(request(), ctx);
  assert.deepEqual((await listed?.json()).releases.map((item: any) => item.version), [release.version, historical.version]);
  const response = await handle(request('POST', JSON.stringify({ releaseId: historical.id, artifactId: 'source' })), ctx);
  assert.equal(response?.status, 200);
  assert.match(calls.provider[0][0], new RegExp('/staging/' + historical.sourceCommit + '/' + historical.id + '/source/'));
});

test('Storage request and stalled response body both stop at the bounded deadline', async () => {
  const handle = createDownloadHandler([release]);
  for (const stallBody of [false, true]) {
    let entered: () => void = () => {};
    const ready = new Promise<void>(resolve => { entered = resolve; });
    const { ctx } = fixture({ provider: async () => {
      if (!stallBody) { entered(); return new Promise(() => {}); }
      return new Response(new ReadableStream({ pull() { entered(); } }), { headers: { 'Content-Type': 'application/json' } });
    } });
    mock.timers.enable({ apis: ['setTimeout'] });
    try {
      const pending = handle(request('POST', selection), ctx);
      await ready;
      mock.timers.tick(8001);
      const response = await pending;
      assert.equal(response?.status, 503);
      assert.equal((await response?.text())?.includes('token='), false);
    } finally { mock.timers.reset(); }
  }
});
