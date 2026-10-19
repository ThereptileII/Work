import test from 'node:test';
import assert from 'node:assert/strict';
import { handleRequest } from './index.ts';

test('hosted function-name and full gateway paths route to the same health handler', async () => {
  for (const path of ['/skager-web/api/health', '/functions/v1/skager-web/api/health', '/api/health']) {
    const response = await handleRequest(new Request('https://kelwgubkuvditalsbqpr.supabase.co' + path), {});
    assert.equal(response.status, 200, path);
    assert.deepEqual(await response.json(), { service: 'skager-web', mode: 'private-beta', ready: false });
  }
});

test('health uses only the deployable SKAGER auth flag', async () => {
  for (const [env, ready] of [
    [{ SUPABASE_AUTH_ENABLED: 'true' }, false],
    [{ SUPABASE_AUTH_ENABLED: 'true', SKAGER_AUTH_ENABLED: 'false' }, false],
    [{ SKAGER_AUTH_ENABLED: 'true' }, true],
  ] as const) {
    const response = await handleRequest(new Request('https://kelwgubkuvditalsbqpr.supabase.co/skager-web/api/health'), env);
    assert.equal(response.status, 200);
    assert.equal((await response.json()).ready, ready);
  }
});

test('hosted auth session route reaches the handler and bootstraps CSRF', async () => {
  const response = await handleRequest(new Request('https://kelwgubkuvditalsbqpr.supabase.co/skager-web/api/auth/session'), {});
  assert.equal(response.status, 200);
  const body = await response.json();
  assert.equal(body.account, null);
  assert.match(body.csrfToken, /^[a-f0-9]{64}$/);
  assert.equal(response.headers.getSetCookie().length, 1);
});

test('similar function names cannot reach the application router', async () => {
  for (const path of ['/other/api/health', '/skager-web-copy/api/health', '/functions/v1/other/api/health']) {
    const response = await handleRequest(new Request('https://kelwgubkuvditalsbqpr.supabase.co' + path), {});
    assert.equal(response.status, 404, path);
  }
});

test('download query parameters are refused before hosted path normalization or provider calls', async () => {
  for (const prefix of ['', '/skager-web', '/functions/v1/skager-web']) {
    for (const method of ['GET', 'POST']) {
      const response = await handleRequest(new Request(`https://kelwgubkuvditalsbqpr.supabase.co${prefix}/api/portal/downloads?account_id=other`, { method }), {}, async () => assert.fail('no provider call'));
      assert.equal(response.status, 400);
      assert.equal(response.headers.get('Cache-Control'), 'no-store');
    }
  }
});

test('download route stays closed without its explicit enablement; POST still requires CSRF', async () => {
  for (const prefix of ['', '/skager-web', '/functions/v1/skager-web']) {
    const url = `https://kelwgubkuvditalsbqpr.supabase.co${prefix}/api/portal/downloads`;
    const transport = async () => assert.fail('disabled downloads cannot call provider');
    const get = await handleRequest(new Request(url), {}, transport);
    assert.equal(get.status, 404);
    const enabled = await handleRequest(new Request(url), {
      SKAGER_DOWNLOADS_ENABLED: 'true', PORTAL_ENABLED: 'true', STRIPE_TEST_ENVIRONMENT: 'staging',
    }, transport);
    assert.equal(enabled.status, 401, 'the enabled handler must still require an authenticated account');
    const post = await handleRequest(new Request(url, { method: 'POST', body: '{}' }), {}, transport);
    assert.equal(post.status, 403);
  }
});
