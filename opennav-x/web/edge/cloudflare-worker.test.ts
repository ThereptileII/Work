import test from 'node:test';
import assert from 'node:assert/strict';
import { registerHooks } from 'node:module';

// Public assets are generated at packaging time. Load the actual Worker module
// with an empty asset inventory; API tests never enter its static-page branch.
const workerURL = new URL('./cloudflare-worker.mjs', import.meta.url).href;
const hook = registerHooks({
  resolve(specifier, context, nextResolve) {
    if (specifier === './generated-assets.mjs' && context.parentURL === workerURL) {
      return { url: 'data:text/javascript,export default Object.freeze({});', shortCircuit: true };
    }
    return nextResolve(specifier, context);
  },
});
const { handleWorker } = await import(workerURL);
hook.deregister();

const secret = 'synthetic-private-cookie-and-provider-error';
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
