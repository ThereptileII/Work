import test, { mock } from 'node:test';
import assert from 'node:assert/strict';
import { createDownloadHandler, handleDownloads, DOWNLOAD_BUCKET } from './downloads.ts';
import type { DownloadRelease } from './downloads.ts';
import { PUBLIC_ORIGIN } from './platform.ts';
import { AwsClient } from './vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';
import { R2_ACCOUNT_ID, R2_DOWNLOAD_ORIGIN } from './r2-downloads.ts';
const originalSign = AwsClient.prototype.sign;

// Inert source-contract fixture only: no real qualified package or R2 object.
const release: DownloadRelease = { id: 'fixture-beta-1', sourceCommit: 'a'.repeat(40), version: '1.0.0-beta.1',
  channel: 'staging', qualification: { record: 'docs/evidence/inert-fixture.json', sha256: 'b'.repeat(64) },
  artifacts: [ ['installer', 'Setup.exe'], ['recovery', 'Recovery.zip'], ['checksums', 'SHA256SUMS.txt'],
    ['source', 'Source.zip'], ['release-notes', 'Notes.md'] ].map(([id, name], index) => ({
      id: id as any, name, bytes: 100 + index, sha256: String(index + 1).repeat(64),
    })),
};
const accountId = '11111111-1111-4111-8111-111111111111';
const secret = 'e'.repeat(64);
const accessKeyId = 'f'.repeat(32);
const csrf = 'c'.repeat(64);
const entitlement = { status: 'active', environment: 'staging', product_code: 'skager-beta' };
function request(method = 'GET', body?: string, headers: Record<string, string> = {}, suffix = '') {
  return new Request(PUBLIC_ORIGIN + '/api/portal/downloads' + suffix, { method,
    headers: { Origin: PUBLIC_ORIGIN, Cookie: `__Host-skager_csrf=${csrf}`, 'X-CSRF-Token': csrf,
      'Content-Type': 'application/json', ...headers }, ...(body !== undefined ? { body } : {}) });
}
function fixture(options: any = {}) {
  const calls = { accounts: 0, rpc: [] as any[], signer: [] as any[], provider: [] as any[] };
  const ctx = { env: { SKAGER_DOWNLOADS_ENABLED: 'true', PORTAL_ENABLED: 'true', STRIPE_TEST_ENVIRONMENT: 'staging',
    SKAGER_DOWNLOAD_BACKEND: 'r2', R2_ACCOUNT_ID, R2_DOWNLOAD_BUCKET: DOWNLOAD_BUCKET,
    R2_ACCESS_KEY_ID: accessKeyId, R2_SECRET_ACCESS_KEY: secret, ...options.env },
    getAccount: async () => { calls.accounts++; return options.account === undefined ? { id: accountId } : options.account; },
    rpc: async (name: string, args: any) => {
      calls.rpc.push([name, args]);
      if (name === 'skager_auth_throttle') {
        if (options.throttleError) throw Error(secret);
        return Object.hasOwn(options, 'throttle') ? options.throttle : true;
      }
      if (options.rpc) return options.rpc(calls.rpc.length);
      return { entitlement: options.entitlement === undefined ? entitlement : options.entitlement, orders: options.orders ?? [] };
    },
    fetch: async (...args: any[]) => { calls.provider.push(args); throw Error('Unexpected network request'); },
  };
  mock.restoreAll();
  mock.method(AwsClient.prototype, 'sign', async function(input: Request, init: any) {
    calls.signer.push([input, init]);
    if (options.signer) return options.signer(input, init);
    return originalSign.call(this, input, init);
  });
  return { calls, ctx: ctx as any };
}
const selection = JSON.stringify({ releaseId: release.id, artifactId: 'installer' });

test('downloads default off and empty; configuration never invents a qualified release', async () => {
  for (const env of [{ SKAGER_DOWNLOADS_ENABLED: '' }, { SKAGER_DOWNLOADS_ENABLED: 'TRUE' },
    { PORTAL_ENABLED: 'false' }, { STRIPE_TEST_ENVIRONMENT: 'production' }]) {
    const { ctx, calls } = fixture({ env });
    assert.equal((await handleDownloads(request(), ctx))?.status, 404);
    assert.equal(calls.accounts, 0); assert.deepEqual(calls.rpc, []); assert.deepEqual(calls.signer, []);
  }
  const { ctx, calls } = fixture();
  const empty = await handleDownloads(request(), ctx);
  assert.deepEqual(await empty?.json(), { available: false,
    reason: 'Release qualification and secure download delivery are pending.', releases: [] });
  assert.equal((await handleDownloads(request('POST', selection), ctx))?.status, 404);
  assert.deepEqual(calls.signer, []);
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
      assert.deepEqual(calls.signer, []);
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
  assert.deepEqual(calls.signer, []);
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

test('selectors reject arbitrary URL, path, account, duplicate fields, oversized data and CSRF bypass before signing', async () => {
  const handle = createDownloadHandler([release]);
  for (const raw of ['{}', '[]', '{"releaseId":"fixture-beta-1","releaseId":"other","artifactId":"installer"}',
    JSON.stringify({ releaseId: release.id, artifactId: 'installer', accountId }),
    JSON.stringify({ releaseId: release.id, artifactId: 'installer', url: 'https://evil.invalid' }),
    JSON.stringify({ releaseId: release.id, artifactId: '../source' }), 'x'.repeat(1025)]) {
    const { ctx, calls } = fixture();
    assert.equal((await handle(request('POST', raw), ctx))?.status, 400);
    assert.deepEqual(calls.signer, []);
  }
  for (const headers of [{ Origin: 'https://evil.invalid' }, { 'X-CSRF-Token': '' },
    { Cookie: `__Host-skager_csrf=${csrf}; __Host-skager_csrf=${csrf}` }]) {
    const { ctx, calls } = fixture();
    assert.equal((await handle(request('POST', selection, headers), ctx))?.status, 403);
    assert.equal(calls.accounts, 0); assert.deepEqual(calls.signer, []);
  }
  const { ctx, calls } = fixture();
  assert.equal((await handle(request('GET', undefined, {}, '?account=other'), ctx))?.status, 400);
  assert.equal((await handle(request('POST', JSON.stringify({ releaseId: 'unknown', artifactId: 'source' })), ctx))?.status, 404);
  assert.equal((await handle(request('DELETE'), ctx))?.status, 405);
  assert.deepEqual(calls.signer, []);
});

test('each supported artifact signs only its immutable server object with private no-cache metadata', async () => {
  const handle = createDownloadHandler([release]);
  for (const artifact of release.artifacts) {
    const { ctx, calls } = fixture();
    const response = await handle(request('POST', JSON.stringify({ releaseId: release.id, artifactId: artifact.id })), ctx);
    assert.equal(response?.status, 200);
    const data = await response?.json();
    const path = `/${DOWNLOAD_BUCKET}/staging/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`;
    assert.equal(calls.signer.length, 1);
    const [input, init] = calls.signer[0];
    assert.equal(input.method, 'GET'); assert.equal(input.body, null);
    assert.equal(new URL(input.url).origin, R2_DOWNLOAD_ORIGIN);
    assert.equal(new URL(input.url).pathname, path);
    assert.equal(init.aws.signQuery, true);
    const link = new URL(data.url);
    assert.equal(link.origin, R2_DOWNLOAD_ORIGIN); assert.equal(link.pathname, path);
    assert.equal(link.searchParams.get('X-Amz-Expires'), '60');
    assert.equal(link.searchParams.get('response-content-disposition'), `attachment; filename="${artifact.name}"`);
    assert.deepEqual(calls.provider, []);
    assert.equal(data.sha256, artifact.sha256); assert.equal(data.bytes, artifact.bytes);
    assert.equal(calls.accounts, 2); assert.equal(calls.rpc.length, 3);
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
    assert.equal(response?.status, 403); assert.equal(calls.signer.length, 1);
    assert.equal((await response?.text())?.includes('X-Amz-Signature'), false);
  }
  const { ctx, calls } = fixture();
  ctx.getAccount = async () => ++calls.accounts === 1 ? { id: accountId } : null;
  assert.equal((await handle(request('POST', selection), ctx))?.status, 401);
  const switched = fixture();
  switched.ctx.getAccount = async () => ({ id: ++switched.calls.accounts === 1 ? accountId : '22222222-2222-4222-8222-222222222222' });
  assert.equal((await handle(request('POST', selection), switched.ctx))?.status, 401);
});

test('post-mint database uncertainty and expiry during the final access check withhold the bearer', async () => {
  const handle = createDownloadHandler([release]);
  for (const result of [undefined, { entitlement: {} }, 'active']) {
    const { ctx, calls } = fixture({ rpc: async (count: number) => count === 1 ? { entitlement } : result });
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 503); assert.equal(calls.signer.length, 1);
    assert.equal((await response?.text())?.includes('X-Amz-Signature'), false);
  }
  const { ctx, calls } = fixture({ rpc: async (count: number) => {
    if (count > 1) mock.timers.tick(60000);
    return { entitlement };
  } });
  mock.timers.enable({ apis: ['Date'], now: Date.parse('2026-10-14T12:00:00Z') });
  try {
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 503); assert.equal(calls.signer.length, 1);
    assert.equal((await response?.text())?.includes('X-Amz-Signature'), false);
  } finally { mock.timers.reset(); }
});

test('SDK failures are generic and leak no credential, bearer or exception details', async () => {
  const handle = createDownloadHandler([release]);
  const { ctx, calls } = fixture({ signer: async () => { throw Error(secret + '?X-Amz-Signature=private'); } });
  const response = await handle(request('POST', selection), ctx);
  assert.equal(response?.status, 503);
  assert.equal(await response?.text(), '{"error":"Downloads are temporarily unavailable. Please try again."}');
  assert.equal(calls.rpc.length, 2);
  assert.equal(JSON.stringify([...response!.headers]).includes(secret), false);
  assert.deepEqual(calls.provider, []);
});

test('wrong R2 configuration, missing key and database uncertainty fail closed before signing', async () => {
  const handle = createDownloadHandler([release]);
  for (const options of [{ env: { R2_ACCOUNT_ID: 'wrong-account' } },
    { env: { R2_SECRET_ACCESS_KEY: '' } }, { rpc: async () => { throw Error(secret); } },
    { rpc: async () => ({ orders: [] }) }, { rpc: async () => 'active' }]) {
    const { ctx, calls } = fixture(options);
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, 503); assert.deepEqual(calls.signer, []);
    assert.equal((await response?.text())?.includes(secret), false);
  }
});

test('reviewed historical entries keep their own commit, version, artifact hashes and R2 identity', async () => {
  const historical = { ...structuredClone(release), id: 'fixture-beta-older', sourceCommit: 'e'.repeat(40), version: '0.9.0-beta.1' };
  const handle = createDownloadHandler([release, historical]);
  const { ctx, calls } = fixture();
  const listed = await handle(request(), ctx);
  assert.deepEqual((await listed?.json()).releases.map((item: any) => item.version), [release.version, historical.version]);
  const response = await handle(request('POST', JSON.stringify({ releaseId: historical.id, artifactId: 'source' })), ctx);
  assert.equal(response?.status, 200);
  assert.match(calls.signer[0][0].url, new RegExp('/staging/' + historical.sourceCommit + '/' + historical.id + '/source/'));
});

test('stalled local signing stops at the bounded deadline', async () => {
  const handle = createDownloadHandler([release]);
  let entered: () => void = () => {};
  const ready = new Promise<void>(resolve => { entered = resolve; });
  const { ctx } = fixture({ signer: async () => { entered(); return new Promise(() => {}); } });
  mock.timers.enable({ apis: ['setTimeout'] });
  try {
    const pending = handle(request('POST', selection), ctx);
    await ready;
    mock.timers.tick(8001);
    const response = await pending;
    assert.equal(response?.status, 503);
    assert.equal((await response?.text())?.includes('X-Amz-Signature'), false);
  } finally { mock.timers.reset(); }
});

test('only selected signing consumes its verified account quota; denial and uncertain limits never reach signing', async () => {
  const handle = createDownloadHandler([release]);
  for (const options of [{ throttle: false }, { throttle: 'true' }, { throttle: null },
    { throttle: {} }, { throttle: undefined }, { throttleError: true }]) {
    const { ctx, calls } = fixture(options);
    const response = await handle(request('POST', selection), ctx);
    assert.equal(response?.status, options.throttle === false ? 429 : 503);
    assert.deepEqual(calls.signer, []);
    assert.deepEqual(calls.rpc, [
      ['commerce_portal_access', { p_account_id: accountId }],
      ['skager_auth_throttle', { p_keys: ['download:account:' + accountId], p_limit: 10, p_seconds: 60 }],
    ]);
    assert.equal((await response?.text())?.includes(secret), false);
  }
  for (const amount_minor of [0, 2000]) {
    const { ctx, calls } = fixture({ throttle: true, orders: [{ status: 'fulfilled', amount_minor }] });
    assert.equal((await handle(request('POST', selection), ctx))?.status, 200);
    assert.deepEqual(calls.rpc.map(([name]) => name), ['commerce_portal_access', 'skager_auth_throttle', 'commerce_portal_access']);
  }
  for (const input of [request(), request('POST', JSON.stringify({ releaseId: 'unknown', artifactId: 'source' }))]) {
    const { ctx, calls } = fixture();
    await handle(input, ctx);
    assert.equal(calls.rpc.some(([name]) => name === 'skager_auth_throttle'), false);
    assert.deepEqual(calls.signer, []);
  }
});
