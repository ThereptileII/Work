import { getAccount, handleAuth } from './auth.ts';
import { reportCommerceConfiguration } from './commerce.ts';
import { handleStore } from './store.ts';
import { handleAdmin, IMAGE_BODY_LIMIT, IMAGE_ROUTE } from './admin.ts';
import { handlePortal } from './portal.ts';
import { handleDownloads } from './downloads.ts';
import { handleCloud, isDeviceRoute } from './cloud.ts';
import { handleCompanion, isCompanionRoute } from './companion.ts';
import { bootstrapCsrf, boundedBytes, csrfValid, json, MAX_BODY, PUBLIC_ORIGIN, rpcClient } from './platform.ts';
import type { Context, RuntimeEnv } from './platform.ts';

export async function handleRequest(request: Request, env: RuntimeEnv, transport: typeof fetch = fetch): Promise<Response> {
  try {
    // Hosted Edge requests reach Deno with the function-name prefix. The local
    // route tests also use the full public /functions/v1 path; accept only these
    // two exact prefixes before mapping onto the application's /api routes.
    const url = new URL(request.url);
    const path = url.pathname.replace(/^\/(?:functions\/v1\/)?skager-web(?=\/api\/)/, '');
    if (path === '/api/portal/downloads' && url.search) return json({ error: 'Download query parameters are not supported.' }, 400);
    if (path === '/api/health' && request.method === 'GET') return json({ service: 'skager-web', mode: 'private-beta', ready: env.SKAGER_AUTH_ENABLED === 'true' });
    if (!path.startsWith('/api/') || !['GET', 'POST'].includes(request.method)) return json({ error: 'Not found' }, 404);
    // Only the admin image upload may exceed the normal body bound.
    const limit = path === IMAGE_ROUTE ? IMAGE_BODY_LIMIT : MAX_BODY;
    const length = request.headers.get('content-length');
    if (length && (!/^[0-9]{1,9}$/.test(length) || Number(length) > limit)) return json({ error: 'Request too large' }, 413);
    // The Worker also bounds upload bytes; cap direct Edge requests before handlers.
    if (request.method === 'POST') {
      let bytes: Uint8Array;
      try { bytes = await boundedBytes(new Response(request.body), limit); }
      catch { return json({ error: 'Request too large' }, 413); }
      request = new Request(PUBLIC_ORIGIN + path, { method: 'POST', headers: request.headers, body: bytes });
    } else {
      // Only the phone API reads query parameters (device, history window, paging).
      request = new Request(PUBLIC_ORIGIN + path + (isCompanionRoute(path) ? url.search : ''), { headers: request.headers });
    }
    // Stripe webhooks are signed, the scheduled job carries its own secret, and SKAGER app
    // and Companion routes use a device/phone token and refuse cookies; browsers need CSRF.
    if (request.method === 'POST' && path !== '/api/stripe/webhook' && path !== '/api/jobs/tick' && !isDeviceRoute(path)
        && !isCompanionRoute(path)
        && !await csrfValid(request)) {
      return json({ error: 'Please refresh the page and try again.' }, 403);
    }
    const ctx: Context = { env, fetch: transport, rpc: rpcClient(env, transport), getAccount: async () => null };
    ctx.getAccount = req => getAccount(req, ctx);
    let response = await handleAuth(request, ctx) ?? await handleStore(request, ctx) ?? await handleAdmin(request, ctx)
      ?? await handlePortal(request, ctx) ?? await handleDownloads(request, ctx) ?? await handleCloud(request, ctx)
      ?? await handleCompanion(request, ctx);
    if (!response) response = json({ error: 'Not found' }, 404);
    if (path === '/api/auth/session' && request.method === 'GET') response = await bootstrapCsrf(request, response);
    // The phone API's ETag/304 answers keep their own private revalidation policy.
    if (!path.startsWith('/api/media/') && !response.headers.has('ETag')) response.headers.set('Cache-Control', 'no-store');
    response.headers.set('X-Content-Type-Options', 'nosniff');
    return response;
  } catch { return json({ error: 'The service is temporarily unavailable. Please try again.' }, 503); }
}

declare const Deno: { env: { toObject(): RuntimeEnv }; serve: (handler: (r: Request) => Promise<Response>) => void };
if (typeof Deno !== 'undefined') {
  // One static startup observation per isolate; never log on a public request.
  try { reportCommerceConfiguration(Deno.env.toObject()); }
  catch { /* A diagnostic cannot prevent the existing server from starting. */ }
  Deno.serve(request => handleRequest(request, Deno.env.toObject()));
}
