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

// The SKAGER app and its updater authenticate with a device token, never a cookie.
const DEVICE_TOKEN = /^Bearer (skg1_[a-f0-9]{64})$/;
const VERSION = /^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$/;
// TUF metadata and signed artifacts for one channel; no traversal or empty segments.
const UPDATE_KEY = /^(?:beta|stable)\/[A-Za-z0-9][A-Za-z0-9._-]{0,199}(?:\/[A-Za-z0-9][A-Za-z0-9._-]{0,199}){0,4}$/;
const ACCESS_TTL_MS = 60000;
const accessCache = new Map();

function plain(status, body) {
  return finish(new Response(body, { status, headers: { 'Content-Type': 'text/plain; charset=utf-8', 'Cache-Control': 'no-store' } }));
}

async function updateAccess(token, transport) {
  const cached = accessCache.get(token);
  if (cached && cached.until > Date.now()) return cached.status;
  const response = await transport(EDGE + '/api/device/update-access', {
    method: 'GET', headers: { 'x-skager-device': token }, redirect: 'manual', signal: AbortSignal.timeout(10000),
  });
  await response.body?.cancel().catch(() => {});
  const status = response.status === 204 ? 204 : response.status === 401 || response.status === 402 ? response.status : 503;
  // Only a grant is reused, so a new subscription works at once; a revoked device
  // can read the (signed, integrity-checked) feed for at most ACCESS_TTL_MS more.
  if (status === 204) {
    if (accessCache.size >= 1000) accessCache.clear();
    accessCache.set(token, { status, until: Date.now() + ACCESS_TTL_MS });
  }
  return status;
}

/** SKAGER Cloud update feed: only a linked SKAGER with an active subscription may read
 * it. TUF signatures, not this check, establish what is trusted; this only decides who
 * may download. Objects come from the SKAGER_UPDATES R2 binding on this same origin. */
async function serveUpdate(request, url, env, transport) {
  if (!['GET', 'HEAD'].includes(request.method)) return plain(405, 'GET required');
  const key = url.pathname.slice('/updates/'.length);
  if (url.search || !UPDATE_KEY.test(key) || key.split('/').some(part => part === '.' || part === '..')) return plain(404, 'Not found');
  const match = DEVICE_TOKEN.exec(request.headers.get('authorization') || '');
  if (!match) return plain(401, 'Link SKAGER to your account to receive automatic updates.');
  let access;
  try { access = await updateAccess(match[1], transport); } catch { access = 503; }
  if (access === 401) return plain(401, 'This SKAGER is no longer linked to an account.');
  if (access === 402) return plain(402, 'Automatic updates are part of SKAGER Cloud.');
  if (access !== 204 || !env?.SKAGER_UPDATES) return plain(503, 'Update service temporarily unavailable.');
  const object = request.method === 'HEAD' ? await env.SKAGER_UPDATES.head(key) : await env.SKAGER_UPDATES.get(key);
  if (!object) return plain(404, 'Not found');
  const headers = new Headers({ 'Content-Type': key.endsWith('.json') ? 'application/json' : 'application/octet-stream',
    'Content-Length': String(object.size), 'Cache-Control': 'private, no-store', 'Content-Encoding': 'identity' });
  return finish(new Response(request.method === 'HEAD' ? null : object.body, { headers }));
}

export async function handleWorker(request, transport = fetch, env = undefined) {
  const url = new URL(request.url);
  if (url.origin !== ORIGIN) return finish(new Response('Use beta.skager.app', { status: 421 }));
  if (url.pathname.startsWith('/updates/')) return serveUpdate(request, url, env, transport);
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
    if (url.pathname.startsWith('/api/device/')) {
      // The app's bearer token travels to the function under its own name only.
      const token = DEVICE_TOKEN.exec(request.headers.get('authorization') || '');
      if (token) headers.set('x-skager-device', token[1]);
      const version = request.headers.get('x-skager-version');
      if (version && VERSION.test(version)) headers.set('x-skager-version', version);
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
  if (/^\/portal(?:\/(?:downloads|shop|cloud|orders|account|boats))?\/?$/.test(path) || /^\/link\/?$/.test(path)) path = '/portal.html';
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

export default { fetch: (request, env) => handleWorker(request, fetch, env) };
