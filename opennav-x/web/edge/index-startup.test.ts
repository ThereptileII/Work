import test from 'node:test';
import assert from 'node:assert/strict';

// Exercise the actual hosted entrypoint without a socket, provider or secrets.
// Fresh module identities simulate fresh isolates; every request is handled locally.
test('Deno entrypoint emits one enabled startup marker across repeated public requests', async () => {
  const original = console.warn;
  const warnings: unknown[][] = [];
  const host = globalThis as typeof globalThis & { Deno?: unknown };
  const priorDeno = Object.getOwnPropertyDescriptor(host, 'Deno');
  let handler: (request: Request) => Promise<Response>;
  let registrations = 0;
  console.warn = (...args) => { warnings.push(args); };
  try {
    host.Deno = { env: { toObject: () => ({ SKAGER_AUTH_ENABLED: 'true', STRIPE_TEST_CHECKOUT_ENABLED: 'true',
      STRIPE_TEST_SECRET_KEY: 'PRIVATE-SENTINEL-NEVER-LOG' }) },
      serve: (serve: typeof handler) => { handler = serve; registrations++; } };
    await import('./index.ts?startup=enabled');
    assert.equal(registrations, 1);
    assert.deepEqual(warnings, [['skager-commerce:configuration-v1:STRIPE_TEST_SECRET_KEY']]);
    for (const path of ['/api/health', '/skager-web/api/health', '/functions/v1/skager-web/api/health']) {
      const response = await handler!(new Request('https://kelwgubkuvditalsbqpr.supabase.co' + path));
      assert.equal(response.status, 200);
      assert.deepEqual(await response.json(), { service: 'skager-web', mode: 'private-beta', ready: true });
    }
    const token = 'c'.repeat(64);
    for (let attempt = 0; attempt < 3; attempt++) {
      const response = await handler!(new Request('https://kelwgubkuvditalsbqpr.supabase.co/skager-web/api/checkout/start', {
        method: 'POST', headers: { Origin: 'https://beta.skager.app',
          Cookie: `__Host-skager_csrf=${token}`, 'X-CSRF-Token': token, 'Content-Type': 'application/json' }, body: '{}',
      }));
      assert.equal(response.status, 404);
      assert.deepEqual(await response.json(), { error: 'Unavailable' });
    }
    assert.deepEqual(warnings, [['skager-commerce:configuration-v1:STRIPE_TEST_SECRET_KEY']]);
  } finally {
    console.warn = original;
    if (priorDeno) Object.defineProperty(host, 'Deno', priorDeno); else delete host.Deno;
  }
});

test('disabled or unavailable startup diagnostics preserve Deno server registration and stay silent', async () => {
  const original = console.warn;
  const warnings: unknown[][] = [];
  const host = globalThis as typeof globalThis & { Deno?: unknown };
  const priorDeno = Object.getOwnPropertyDescriptor(host, 'Deno');
  console.warn = (...args) => { warnings.push(args); };
  try {
    for (const kind of ['disabled', 'environment-failure']) {
      let registrations = 0;
      host.Deno = { env: { toObject: () => {
        if (kind === 'environment-failure') throw Error('PRIVATE-SENTINEL-NEVER-LOG');
        return { STRIPE_TEST_CHECKOUT_ENABLED: 'false' };
      } }, serve: () => { registrations++; } };
      await import(`./index.ts?startup=${kind}`);
      assert.equal(registrations, 1);
      assert.deepEqual(warnings, []);
    }
  } finally {
    console.warn = original;
    if (priorDeno) Object.defineProperty(host, 'Deno', priorDeno); else delete host.Deno;
  }
});
