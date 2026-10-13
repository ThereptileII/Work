import { getAccount, handleAuth } from './auth.ts';
import { handleCommerce } from './commerce.ts';
import { handlePortal } from './portal.ts';
import { bootstrapCsrf, boundedBytes, csrfValid, json, MAX_BODY, PUBLIC_ORIGIN, rpcClient } from './platform.ts';
import type { Context, RuntimeEnv } from './platform.ts';

export async function handleRequest(request: Request, env: RuntimeEnv, transport: typeof fetch = fetch): Promise<Response> {
  try {
    // Hosted Edge requests reach Deno with the function-name prefix. The local
    // route tests also use the full public /functions/v1 path; accept only these
    // two exact prefixes before mapping onto the application's /api routes.
    const path = new URL(request.url).pathname.replace(/^\/(?:functions\/v1\/)?skager-web(?=\/api\/)/, '');
    if (path === '/api/health' && request.method === 'GET') return json({ service: 'skager-web', mode: 'private-beta', ready: env.SUPABASE_AUTH_ENABLED === 'true' });
    if (!path.startsWith('/api/') || !['GET', 'POST'].includes(request.method)) return json({ error: 'Not found' }, 404);
    const length = request.headers.get('content-length');
    if (length && (!/^[0-9]{1,9}$/.test(length) || Number(length) > MAX_BODY)) return json({ error: 'Request too large' }, 413);
    // The Worker also bounds upload bytes; cap direct Edge requests before handlers.
    if (request.method === 'POST') {
      let bytes: Uint8Array;
      try { bytes = await boundedBytes(new Response(request.body), MAX_BODY); }
      catch { return json({ error: 'Request too large' }, 413); }
      request = new Request(PUBLIC_ORIGIN + path, { method: 'POST', headers: request.headers, body: bytes });
    } else request = new Request(PUBLIC_ORIGIN + path, { headers: request.headers });
    if (request.method === 'POST' && path !== '/api/stripe/webhook' && !await csrfValid(request)) {
      return json({ error: 'Please refresh the page and try again.' }, 403);
    }
    const ctx: Context = { env, fetch: transport, rpc: rpcClient(env, transport), getAccount: async () => null };
    ctx.getAccount = req => getAccount(req, ctx);
    let response = await handleAuth(request, ctx) ?? await handleCommerce(request, ctx) ?? await handlePortal(request, ctx);
    if (!response) response = json({ error: 'Not found' }, 404);
    if (path === '/api/auth/session' && request.method === 'GET') response = await bootstrapCsrf(request, response);
    response.headers.set('Cache-Control', 'no-store');
    response.headers.set('X-Content-Type-Options', 'nosniff');
    return response;
  } catch { return json({ error: 'The service is temporarily unavailable. Please try again.' }, 503); }
}

declare const Deno: { env: { toObject(): RuntimeEnv }; serve: (handler: (r: Request) => Promise<Response>) => void };
if (typeof Deno !== 'undefined') Deno.serve(request => handleRequest(request, Deno.env.toObject()));
