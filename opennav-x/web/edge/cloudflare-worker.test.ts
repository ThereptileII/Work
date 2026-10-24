import test from 'node:test';
import assert from 'node:assert/strict';
import { registerHooks } from 'node:module';
import { readFileSync } from 'node:fs';

// These are the original member assets copied by the packager. Load the actual
// Worker routing against their bytes without a build or any provider access.
const assets = Object.fromEntries(['portal.html', 'portal.js', 'portal.css', 'auth.html', 'admin.html'].map(name => [
  '/' + name, { type: name.endsWith('.html') ? 'text/html' : name.endsWith('.css') ? 'text/css' : 'text/javascript',
    body: readFileSync(new URL('./static/' + name, import.meta.url)).toString('base64') },
]));
const workerURL = new URL('./cloudflare-worker.mjs', import.meta.url).href;
const hook = registerHooks({
  resolve(specifier, context, nextResolve) {
    if (specifier === './generated-assets.mjs' && context.parentURL === workerURL) {
      return { url: 'data:text/javascript,' + encodeURIComponent('export default Object.freeze(' + JSON.stringify(assets) + ');'), shortCircuit: true };
    }
    return nextResolve(specifier, context);
  },
});
const { handleWorker } = await import(workerURL);
hook.deregister();

const secret = 'synthetic-private-cookie-and-provider-error';
test('checkout return destinations serve the published member shell and its assets without granting access', async () => {
  const noBackend = async () => assert.fail('a return navigation cannot settle an order or query private data');
  for (const path of ['/portal/orders/', '/portal/']) {
    const response = await handleWorker(new Request('https://beta.skager.app' + path), noBackend);
    assert.equal(response.status, 200);
    assert.equal(response.headers.get('Content-Type'), 'text/html');
    assert.equal(response.headers.get('Cache-Control'), 'no-store');
    assert.equal(await response.text(), readFileSync(new URL('./static/portal.html', import.meta.url), 'utf8'));
    assert.equal(response.headers.has('Set-Cookie'), false);
  }
  for (const path of ['/portal.js', '/portal.css']) {
    const response = await handleWorker(new Request('https://beta.skager.app' + path), noBackend);
    assert.equal(response.status, 200);
    assert.equal(await response.text(), readFileSync(new URL('./static' + path, import.meta.url), 'utf8'));
  }
});
test('member, sign-in and admin shells have fixed routes; public signup routes no longer exist', async () => {
  const noBackend = async () => assert.fail('static shells never call the backend');
  const shells: Record<string, string> = {
    '/portal/downloads/': 'portal.html', '/portal/shop/': 'portal.html', '/portal/cloud/': 'portal.html', '/portal/account': 'portal.html',
    '/auth/login/': 'auth.html', '/auth/forgot/': 'auth.html', '/auth/verify': 'auth.html',
    '/admin/': 'admin.html', '/admin/steps/': 'admin.html', '/admin/orders/': 'admin.html', '/admin/settings/': 'admin.html',
  };
  for (const [path, file] of Object.entries(shells)) {
    const response = await handleWorker(new Request('https://beta.skager.app' + path), noBackend);
    assert.equal(response.status, 200, path);
    assert.equal(await response.text(), readFileSync(new URL('./static/' + file, import.meta.url), 'utf8'));
  }
  for (const path of ['/auth/signup/', '/auth/recover/', '/portal/other/', '/admin/products/']) {
    assert.equal((await handleWorker(new Request('https://beta.skager.app' + path), noBackend)).status, 404, path);
  }
});
test('only the admin image route accepts a larger body; product images may be cached', async () => {
  const seen: number[] = [];
  const upstream = async (_url: string, init: RequestInit) => { seen.push((init.body as Uint8Array).byteLength); return new Response('{}', { headers: { 'content-type': 'application/json' } }); };
  const big = new Uint8Array(1024 * 1024);
  const post = (path: string) => new Request('https://beta.skager.app' + path, { method: 'POST', body: big });
  assert.equal((await handleWorker(post('/api/admin/product'), upstream)).status, 413);
  assert.equal((await handleWorker(post('/api/admin/product-image'), upstream)).status, 200);
  assert.deepEqual(seen, [big.byteLength]);
  const tooBig = new Request('https://beta.skager.app/api/admin/product-image', { method: 'POST', body: new Uint8Array(3 * 1024 * 1024) });
  assert.equal((await handleWorker(tooBig, upstream)).status, 413);
  const image = await handleWorker(new Request('https://beta.skager.app/api/media/products/x.png'), async () => new Response('png', { headers: { 'content-type': 'image/png' } }));
  assert.equal(image.headers.get('Cache-Control'), 'public, max-age=86400');
  assert.equal(image.headers.get('Content-Type'), 'image/png');
  const missing = await handleWorker(new Request('https://beta.skager.app/api/media/products/x.png'), async () => new Response('{}', { status: 404 }));
  assert.equal(missing.headers.get('Cache-Control'), 'no-store');
});
test('download query parameters never get silently discarded or reach the backend', async () => {
  for (const method of ['GET', 'POST']) {
    const response = await handleWorker(new Request('https://beta.skager.app/api/portal/downloads?path=other-account', { method }), async () => assert.fail('no upstream call'));
    assert.equal(response.status, 400);
    assert.equal(response.headers.get('Cache-Control'), 'no-store');
    assert.equal(response.headers.get('Referrer-Policy'), 'no-referrer');
  }
});
function request() {
  return new Request('https://beta.skager.app/api/auth/session', {
    headers: { Cookie: `private=${secret}`, Authorization: `Bearer ${secret}`, 'X-Private': secret },
  });
}

for (const phase of ['timeout-init', 'upstream-request', 'cookie-copy', 'response-build']) {
  test(`Worker logs only the static ${phase} code and retains its generic refusal`, async () => {
    const warnings: unknown[][] = [];
    const originalWarn = console.warn;
    const originalTimeout = AbortSignal.timeout;
    console.warn = (...args) => { warnings.push(args); };
    let calls = 0;
    const privateError = Object.assign(new Error(secret), { name: secret, cause: secret });
    try {
      if (phase === 'timeout-init') AbortSignal.timeout = () => { throw privateError; };
      const response = await handleWorker(request(), async () => {
        calls++;
        if (phase === 'upstream-request') throw privateError;
        const upstream = new Response(JSON.stringify({ private: secret }), {
          headers: { 'Content-Type': 'application/json', 'Set-Cookie': `private=${secret}` },
        });
        if (phase === 'cookie-copy') upstream.headers.getSetCookie = () => { throw privateError; };
        if (phase === 'response-build') Object.defineProperty(upstream, 'body', { get() { throw privateError; } });
        return upstream;
      });
      assert.equal(calls, phase === 'timeout-init' ? 0 : 1);
      assert.deepEqual(warnings, [[`skager-worker:${phase}`]]);
      assert.equal(response.status, 503);
      assert.equal(await response.text(), '{"error":"The service is temporarily unavailable. Please try again."}');
      assert.equal(response.headers.get('Content-Type'), 'application/json');
      assert.equal(response.headers.get('Cache-Control'), 'no-store');
      assert.equal(response.headers.get('X-Content-Type-Options'), 'nosniff');
      assert.equal(response.headers.get('Strict-Transport-Security'), 'max-age=31536000');
      assert.match(response.headers.get('Content-Security-Policy')!, /connect-src 'self'/);
      assert.equal(response.headers.has('Set-Cookie'), false);
      assert.equal(JSON.stringify(warnings).includes(secret), false);
      assert.equal(JSON.stringify([...response.headers]).includes(secret), false);
    } finally {
      console.warn = originalWarn;
      AbortSignal.timeout = originalTimeout;
    }
  });
}

test('Worker successful session preserves cookies and fixed proxy boundaries without diagnostics', async () => {
  const warnings: unknown[][] = [];
  const originalWarn = console.warn;
  console.warn = (...args) => { warnings.push(args); };
  try {
    const response = await handleWorker(request(), async (url, init) => {
      assert.equal(url, 'https://kelwgubkuvditalsbqpr.supabase.co/functions/v1/skager-web/api/auth/session');
      assert.equal(init.method, 'GET');
      assert.equal(init.redirect, 'manual');
      assert.ok(init.signal instanceof AbortSignal);
      assert.equal(init.headers.get('Cookie'), `private=${secret}`);
      assert.equal(init.headers.has('Authorization'), false);
      assert.equal(init.headers.has('X-Private'), false);
      const headers = new Headers({ 'Content-Type': 'application/json' });
      headers.append('Set-Cookie', '__Host-skager_csrf=fixture; Secure; Path=/');
      headers.append('Set-Cookie', '__Host-skager_session=fixture; Secure; HttpOnly; Path=/');
      return new Response('{"account":null,"csrfToken":"fixture"}', { headers });
    });
    assert.equal(response.status, 200);
    assert.deepEqual(await response.json(), { account: null, csrfToken: 'fixture' });
    assert.equal(response.headers.getSetCookie().length, 2);
    assert.equal(response.headers.get('Cache-Control'), 'no-store');
    assert.deepEqual(warnings, []);
  } finally { console.warn = originalWarn; }
});

test('Worker forwards an upstream refusal without inventing a local exception diagnostic', async () => {
  const warnings: unknown[][] = [];
  const originalWarn = console.warn;
  console.warn = (...args) => { warnings.push(args); };
  try {
    const response = await handleWorker(request(), async () => new Response('{"error":"upstream refusal"}', {
      status: 503, headers: { 'Content-Type': 'application/json' },
    }));
    assert.equal(response.status, 503);
    assert.equal(await response.text(), '{"error":"upstream refusal"}');
    assert.deepEqual(warnings, []);
  } finally { console.warn = originalWarn; }
});

for (const status of [301, 302, 303, 307, 308]) {
  test(`Worker refuses upstream ${status} without following or forwarding its private response`, async () => {
    const warnings: unknown[][] = [];
    const originalWarn = console.warn;
    console.warn = (...args) => { warnings.push(args); };
    let calls = 0;
    try {
      const response = await handleWorker(request(), async (_url, init) => {
        calls++;
        assert.equal(init.redirect, 'manual');
        const upstream = new Response(secret, { status, headers: {
          Location: `https://must-not-follow.invalid/${secret}`, 'Set-Cookie': `private=${secret}`,
        } });
        upstream.headers.getSetCookie = () => { assert.fail('redirect cookies must not be inspected'); };
        return upstream;
      });
      assert.equal(calls, 1);
      assert.equal(response.status, 503);
      assert.equal(await response.text(), '{"error":"The service is temporarily unavailable. Please try again."}');
      assert.equal(response.headers.has('Location'), false);
      assert.equal(response.headers.has('Set-Cookie'), false);
      assert.deepEqual(warnings, [['skager-worker:upstream-request']]);
      assert.equal(JSON.stringify([...response.headers]).includes(secret), false);
    } finally { console.warn = originalWarn; }
  });
}

const DEVICE_TOKEN = 'skg1_' + 'b'.repeat(64);
function updatesBucket(objects: Record<string, string>) {
  const reads: string[] = [];
  return { reads, SKAGER_UPDATES: {
    async get(key: string) { reads.push(key); return key in objects ? { size: objects[key].length, body: new Blob([objects[key]]).stream() } : null; },
    async head(key: string) { reads.push(key); return key in objects ? { size: objects[key].length } : null; },
  } };
}
test('the update feed is served only to a linked SKAGER whose account has SKAGER Cloud', async () => {
  const env = updatesBucket({ 'beta/metadata/timestamp.json': '{"signed":{}}' });
  const checks: string[] = [];
  const edge = (status: number) => async (url: string, init: RequestInit) => {
    checks.push(url + ' ' + new Headers(init.headers).get('x-skager-device'));
    return new Response(null, { status });
  };
  const get = (path: string, token?: string) => new Request('https://beta.skager.app' + path,
    { headers: token ? { Authorization: 'Bearer ' + token } : {} });

  const anonymous = await handleWorker(get('/updates/beta/metadata/timestamp.json'), edge(204), env);
  assert.equal(anonymous.status, 401);
  assert.equal(checks.length, 0);
  const unpaid = await handleWorker(get('/updates/beta/metadata/timestamp.json', 'skg1_' + 'c'.repeat(64)), edge(402), env);
  assert.equal(unpaid.status, 402);
  assert.match(await unpaid.text(), /SKAGER Cloud/);
  assert.equal(env.reads.length, 0);
  // A refusal is never cached: subscribing takes effect on the very next request.
  assert.equal((await handleWorker(get('/updates/beta/metadata/timestamp.json', 'skg1_' + 'c'.repeat(64)), edge(204), env)).status, 200);

  const ok = await handleWorker(get('/updates/beta/metadata/timestamp.json', DEVICE_TOKEN), edge(204), env);
  assert.equal(ok.status, 200);
  assert.equal(await ok.text(), '{"signed":{}}');
  assert.equal(ok.headers.get('Content-Type'), 'application/json');
  assert.equal(ok.headers.get('Cache-Control'), 'private, no-store');
  assert.equal(checks.at(-1), 'https://kelwgubkuvditalsbqpr.supabase.co/functions/v1/skager-web/api/device/update-access ' + DEVICE_TOKEN);
  // Every object request checks current access, including immediately after a grant.
  const before = checks.length;
  assert.equal((await handleWorker(get('/updates/beta/metadata/missing.json', DEVICE_TOKEN), edge(204), env)).status, 404);
  assert.equal(checks.length, before + 1);
  assert.equal((await handleWorker(get('/updates/beta/metadata/timestamp.json', DEVICE_TOKEN), edge(401), env)).status, 401);
  assert.equal((await handleWorker(get('/updates/beta/metadata/timestamp.json', DEVICE_TOKEN), edge(402), env)).status, 402);
  assert.equal((await handleWorker(get('/updates/beta/metadata/timestamp.json', DEVICE_TOKEN), edge(503), env)).status, 503);

  for (const path of ['/updates/beta/../secret', '/updates/stable/x.json', '/updates/other/x.json', '/updates/beta//x', '/updates/beta/x.json?y=1', '/updates/beta/.hidden']) {
    assert.equal((await handleWorker(get(path, DEVICE_TOKEN), edge(204), env)).status, 404, path);
  }
  const post = new Request('https://beta.skager.app/updates/beta/metadata/timestamp.json', { method: 'POST', headers: { Authorization: 'Bearer ' + DEVICE_TOKEN } });
  assert.equal((await handleWorker(post, edge(204), env)).status, 405);
  assert.equal((await handleWorker(get('/updates/beta/metadata/timestamp.json', 'skg1_' + 'd'.repeat(64)), edge(204), {})).status, 503);
});
test('signed artifact URLs use the bare origin and map only to immutable beta objects with current access', async () => {
  const commit = 'a'.repeat(40);
  const path = `/artifacts/${commit}/SKAGER-Beta1-Setup.exe`;
  const env = updatesBucket({ ['beta' + path]: 'exe' });
  const request = (method = 'GET', token = DEVICE_TOKEN) => new Request('https://beta.skager.app' + path,
    { method, headers: { Authorization: 'Bearer ' + token } });
  assert.equal((await handleWorker(request(), async () => new Response(null, { status: 204 }), env)).status, 200);
  assert.deepEqual(env.reads, ['beta' + path]);
  assert.equal((await handleWorker(request('HEAD'), async () => new Response(null, { status: 204 }), env)).status, 200);
  assert.equal((await handleWorker(request(), async () => new Response(null, { status: 401 }), env)).status, 401);
  for (const bad of ['/artifacts/x/file.exe', `/artifacts/${commit}/x/secret`, `/artifacts/${commit}/.hidden`, path + '?other=1']) {
    assert.equal((await handleWorker(new Request('https://beta.skager.app' + bad), async () => assert.fail('no auth for invalid path'), env)).status, 404);
  }
});
test('device API calls carry the bearer token under its own header, never to other routes', async () => {
  const seen: Headers[] = [];
  const upstream = async (_url: string, init: RequestInit) => { seen.push(new Headers(init.headers)); return new Response('{}', { headers: { 'content-type': 'application/json' } }); };
  const headers = { Authorization: 'Bearer ' + DEVICE_TOKEN, 'X-Skager-Version': '0.4.0-beta2.7' };
  await handleWorker(new Request('https://beta.skager.app/api/device/status', { headers }), upstream);
  await handleWorker(new Request('https://beta.skager.app/api/portal/dashboard', { headers }), upstream);
  assert.equal(seen[0].get('x-skager-device'), DEVICE_TOKEN);
  assert.equal(seen[0].get('x-skager-version'), '0.4.0-beta2.7');
  assert.equal(seen[0].has('authorization'), false);
  assert.equal(seen[1].has('x-skager-device'), false);
  assert.equal(seen[1].has('authorization'), false);
});
test('the SKAGER pairing page uses the member shell', async () => {
  const noBackend = async () => assert.fail('static shells never call the backend');
  for (const path of ['/link/', '/link']) {
    const response = await handleWorker(new Request('https://beta.skager.app' + path), noBackend);
    assert.equal(response.status, 200, path);
  }
});
