// Generated ASSETS contains only reviewed public pages, never server bindings.
import ASSETS from './generated-assets.mjs';

const ORIGIN = 'https://beta.skager.app';
const EDGE = 'https://kelwgubkuvditalsbqpr.supabase.co/functions/v1/skager-web';
const MAX_BODY = 262144;
// Admin product images are sent as base64 JSON (2 MB image); every other route keeps the small limit.
const IMAGE_ROUTE = '/api/admin/product-image';
const IMAGE_BODY = 2 * 1024 * 1024 * 4 / 3 + 8192;
const CSP = "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; font-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'self'; form-action 'self' https://checkout.stripe.com; frame-ancestors 'none'";

function finish(response) {
  const headers = new Headers(response.headers);
  headers.set('X-Content-Type-Options', 'nosniff');
  headers.set('Referrer-Policy', 'no-referrer');
  headers.set('X-Robots-Tag', 'noindex, nofollow');
  headers.set('Content-Security-Policy', CSP);
  headers.set('Permissions-Policy', 'geolocation=(), camera=(), microphone=()');
  headers.set('Strict-Transport-Security', 'max-age=31536000');
  return new Response(response.body, { status: response.status, headers });
}

async function readBounded(stream, limit = MAX_BODY) {
  if (!stream) return new Uint8Array();
  const reader = stream.getReader();
  const chunks = [];
  let size = 0;
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      size += value.byteLength;
      if (size > limit) throw new RangeError();
      chunks.push(value);
    }
  } finally { await reader.cancel().catch(() => {}); reader.releaseLock(); }
  const result = new Uint8Array(size);
  let cursor = 0;
  for (const chunk of chunks) { result.set(chunk, cursor); cursor += chunk.length; }
  return result;
}

export async function handleWorker(request, transport = fetch) {
  const url = new URL(request.url);
  if (url.origin !== ORIGIN) return finish(new Response('Use beta.skager.app', { status: 421 }));
  if (url.pathname.startsWith('/api/')) {
    if (url.pathname === '/api/portal/downloads' && url.search) return finish(new Response('{"error":"Download query parameters are not supported."}', {
      status: 400, headers: { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' },
    }));
    if (!['GET', 'POST'].includes(request.method)) return finish(new Response('Not found', { status: 404 }));
    const headers = new Headers();
    // Explicit allowlist: never relay ambient provider credentials or spoofed proxy metadata.
    for (const name of ['content-type', 'origin', 'cookie', 'x-csrf-token', 'stripe-signature']) {
      const value = request.headers.get(name);
      if (value && value.length <= 8192) headers.set(name, value);
    }
    let body;
    try { if (request.method === 'POST') body = await readBounded(request.body, url.pathname === IMAGE_ROUTE ? IMAGE_BODY : MAX_BODY); }
    catch { return finish(new Response('Request too large', { status: 413 })); }
    let failurePhase = 'timeout-init';
    try {
      const signal = AbortSignal.timeout(20000);
      failurePhase = 'upstream-request';
      const response = await transport(EDGE + url.pathname, {
        // workerd supports manual/follow; explicitly refuse redirects below.
        method: request.method, headers, body, redirect: 'manual',
        signal,
      });
      if (response.status >= 300 && response.status < 400) {
        await response.body?.cancel().catch(() => {});
        throw new Error();
      }
      failurePhase = 'response-build';
      // Public product images may be cached; every other API response is private.
      const media = url.pathname.startsWith('/api/media/') && response.ok;
      const outputHeaders = new Headers({ 'Cache-Control': media ? 'public, max-age=86400' : 'no-store' });
      outputHeaders.set('Content-Type', response.headers.get('content-type') || 'application/json');
      failurePhase = 'cookie-copy';
      for (const value of response.headers.getSetCookie()) outputHeaders.append('Set-Cookie', value);
      failurePhase = 'response-build';
      return finish(new Response(response.body, { status: response.status, headers: outputHeaders }));
    } catch {
      // Only these static phases reach logs; never log the exception or request.
      console.warn(`skager-worker:${failurePhase}`);
      return finish(new Response('{"error":"The service is temporarily unavailable. Please try again."}', {
        status: 503, headers: { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' },
      }));
    }
  }
  if (!['GET', 'HEAD'].includes(request.method)) return finish(new Response('Not found', { status: 404 }));
  if (url.pathname === '/account/' || url.pathname === '/account') {
    return finish(new Response(null, { status: 303, headers: { Location: '/auth/login/', 'Cache-Control': 'no-store' } }));
  }
  let path = url.pathname;
  if (/^\/portal(?:\/(?:downloads|shop|cloud|orders|account|boats))?\/?$/.test(path)) path = '/portal.html';
  else if (/^\/auth\/(?:login|forgot|verify)\/?$/.test(path)) path = '/auth.html';
  else if (/^\/admin(?:\/(?:steps|orders|customers|settings))?\/?$/.test(path)) path = '/admin.html';
  else if (path.endsWith('/')) path += 'index.html';
  const asset = ASSETS[path];
  if (!asset) return finish(new Response('Not found', { status: 404 }));
  const bytes = Uint8Array.from(atob(asset.body), c => c.charCodeAt(0));
  return finish(new Response(request.method === 'HEAD' ? null : bytes, {
    headers: { 'Content-Type': asset.type, 'Cache-Control': path.startsWith('/static/') ? 'public, max-age=3600' : 'no-store' },
  }));
}

export default { fetch: request => handleWorker(request) };
