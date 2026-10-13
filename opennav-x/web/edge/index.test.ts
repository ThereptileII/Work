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
