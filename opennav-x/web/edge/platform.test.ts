import test from 'node:test';
import assert from 'node:assert/strict';
import { bootstrapCsrf, boundedBytes, csrfValid, json, rpcClient, SUPABASE_ORIGIN } from './platform.ts';

test('CSRF requires exact origin, matching single cookie and header', async () => {
  const token = 'a'.repeat(64);
  const headers = { Origin: 'https://beta.skager.app', Cookie: `__Host-skager_csrf=${token}`, 'X-CSRF-Token': token };
  assert.equal(await csrfValid(new Request('https://beta.skager.app/api/test', { method: 'POST', headers })), true);
  for (const other of [{ Origin: 'https://evil.example' }, { 'X-CSRF-Token': 'b'.repeat(64) },
    { Cookie: `${headers.Cookie}; ${headers.Cookie}` }, { 'X-CSRF-Token': '' }]) {
    assert.equal(await csrfValid(new Request('https://beta.skager.app/api/test', { method: 'POST', headers: { ...headers, ...other } })), false);
  }
});
test('session bootstrap has a secure host cookie, preserves session cookie and never caches', async () => {
  const original = json({ account: null });
  original.headers.append('Set-Cookie', '__Host-skager_session=opaque; Secure; HttpOnly; Path=/');
  const result = await bootstrapCsrf(new Request('https://beta.skager.app/api/auth/session'), original);
  assert.match((await result.json()).csrfToken, /^[a-f0-9]{64}$/);
  assert.equal(result.headers.getSetCookie().length, 2);
  assert.match(result.headers.getSetCookie()[1], /Secure; SameSite=Lax; Path=\//);
  assert.equal(result.headers.get('Cache-Control'), 'no-store');
});
test('bounded streaming refuses an oversized body without content length', async () => {
  await assert.rejects(boundedBytes(new Response(new Uint8Array(11)), 10));
  assert.equal((await boundedBytes(new Response('ok'), 10)).length, 2);
});
test('RPC only reaches fixed project with service role header and approved function names', async () => {
  const secret = 'synthetic-server-only';
  const calls: { url: string; init: RequestInit }[] = [];
  const transport = async (url: string, init: RequestInit) => { calls.push({ url, init }); return json({ accepted: true }); };
  const rpc = rpcClient({ SUPABASE_URL: SUPABASE_ORIGIN, SUPABASE_SERVICE_ROLE_KEY: secret }, transport as typeof fetch);
  assert.deepEqual(await rpc('portal_list_boats', { p_account_id: 'owned' }), { accepted: true });
  assert.equal(calls[0].url, SUPABASE_ORIGIN + '/rest/v1/rpc/portal_list_boats');
  assert.equal(calls[0].init.redirect, 'error');
  assert.equal((calls[0].init.headers as Record<string, string>).Authorization, `Bearer ${secret}`);
  await assert.rejects(rpc('../admin', {}));
  await assert.rejects(rpcClient({ SUPABASE_URL: 'https://evil.example', SUPABASE_SERVICE_ROLE_KEY: secret }, transport as typeof fetch)('portal_list_boats', {}));
  assert.equal(calls.length, 1);
});
test('provider failures expose no body, credential or exception cause', async () => {
  const rpc = rpcClient({ SUPABASE_URL: SUPABASE_ORIGIN, SUPABASE_SERVICE_ROLE_KEY: 'synthetic' },
    (async () => { throw new Error('synthetic credential in transport'); }) as typeof fetch);
  await assert.rejects(rpc('skager_auth_session_get', {}), (error: Error) => {
    assert.equal(error.message, 'Backend unavailable'); assert.equal(error.cause, undefined); return true;
  });
});
