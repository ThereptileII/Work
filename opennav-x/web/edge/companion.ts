/** SKAGER Companion phone API (SCRUM-386, SCRUM-388). Read-only view of the account's
 * boats; the app can never send anything to a boat.
 *  - POST /api/companion/token   one-time code + PKCE verifier -> phone token (no session yet)
 *  - everything else             "Authorization: Bearer skc1_..." (the Worker moves it to
 *                                X-Skager-Companion so the Supabase gateway never sees it)
 * Cookies are refused, so a browser session can never be used here and these routes need no
 * CSRF check. Tokens are stored only as SHA-256. */
import type { CloudContext } from './cloud.ts';
import { APP_STATE, TELEMETRY_KEYS, jsonBody, object, only, publicCloud, publicDevice, randomHex, reply, sha256,
  validAnchor, validTelemetry } from './cloud.ts';

type Json = Record<string, any>;
const TOKEN = /^skc1_[a-f0-9]{64}$/;
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/;
const CODE = /^[a-f0-9]{64}$/;
const VERIFIER = /^[A-Za-z0-9._~-]{43,128}$/;
const VERSION = /^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$/;
const PUSH_TOKEN = { fcm: /^[A-Za-z0-9_:.-]{20,4096}$/, apns: /^[0-9a-f]{64,200}$/ } as const;
const SERIES_HOURS = [1, 6, 24, 72, 168, 720];
const TRACK_HOURS = [1, 6, 24, 72, 168];
/** WebView origins of the packaged app. A request may carry one of these or none. */
export const APP_ORIGINS = ['capacitor://localhost', 'https://localhost'];
/** Name shown in the app and the portal; also used for the deep link. */
export const APP_SCHEME = 'app.skager.companion';
export { APP_STATE };

/** RFC 7636 S256: base64url(SHA-256(verifier)). */
export async function pkceChallenge(verifier: string): Promise<string> {
  const digest = new Uint8Array(await crypto.subtle.digest('SHA-256', new TextEncoder().encode(verifier)));
  return btoa(String.fromCharCode(...digest)).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

type Phone = { sessionId: string; accountId: string; email: string; push: boolean; cloud: Json };
async function phoneAuth(req: Request, ctx: CloudContext): Promise<Phone | null> {
  const token = req.headers.get('X-Skager-Companion') ?? '';
  if (!TOKEN.test(token)) return null;
  const version = req.headers.get('X-Skager-Version');
  const found = object(await ctx.rpc('companion_session_auth', { p_token_hash: await sha256(token),
    p_version: version && VERSION.test(version) ? version : null }));
  if (found.state !== 'active' || typeof found.session_id !== 'string' || !UUID.test(found.session_id)) return null;
  return { sessionId: found.session_id, accountId: found.account_id, email: found.email, push: found.push === true,
    cloud: publicCloud(found.cloud) };
}

function query(req: Request): URLSearchParams { return new URL(req.url).searchParams; }
function deviceParam(req: Request): string | null {
  const id = query(req).get('device');
  return id && UUID.test(id) ? id : null;
}
function onlyParams(req: Request, allowed: string[]): boolean {
  return [...query(req).keys()].every(key => allowed.includes(key)) && new Set(query(req).keys()).size === [...query(req).keys()].length;
}

async function exchange(req: Request, ctx: CloudContext): Promise<Response> {
  let input: Json;
  try {
    input = await jsonBody(req);
    only(input, ['code', 'verifier', 'name', 'platform', 'version']);
    if (!(typeof input.code === 'string' && CODE.test(input.code) && typeof input.verifier === 'string'
      && VERIFIER.test(input.verifier) && typeof input.name === 'string' && input.name.trim().length >= 1
      && input.name.length <= 60 && !/[\u0000-\u001f\u007f]/.test(input.name) && ['android', 'ios'].includes(input.platform)
      && typeof input.version === 'string' && VERSION.test(input.version))) throw new Error();
  } catch { return reply(400, { error: 'Sign-in could not be completed. Start again.' }); }
  if (await ctx.rpc('skager_auth_throttle', { p_keys: ['companion:token'], p_limit: 60, p_seconds: 60 }) !== true) {
    return reply(429, { error: 'Too many sign-in attempts. Try again in a minute.' });
  }
  const token = 'skc1_' + randomHex(32);
  const result = object(await ctx.rpc('companion_grant_redeem', { p_code_hash: await sha256(input.code),
    p_challenge: await pkceChallenge(input.verifier), p_token_hash: await sha256(token), p_name: input.name.trim(),
    p_platform: input.platform, p_version: input.version }));
  if (result.state !== 'ok') return reply(400, { error: 'This sign-in has expired or was already used. Start again.' });
  return reply(200, { token });
}

/** Re-validates stored values one by one on the way out: an entry this version does not
 * know is dropped without hiding the rest. */
function knownItems(items: unknown): Json {
  const out: Json = {};
  if (!items || typeof items !== 'object' || Array.isArray(items)) return out;
  for (const [key, value] of Object.entries(items)) {
    const valid = validTelemetry({ [key]: value });
    if (valid) Object.assign(out, valid);
  }
  return out;
}
/** Re-validates stored values on the way out: the app only ever receives known keys. */
function publicBoat(latest: Json): Json {
  return { device: publicDevice(latest.device), receivedAt: latest.received_at ?? null,
    items: knownItems(latest.items),
    anchor: latest.anchor ? validAnchor(latest.anchor) : null };
}
function publicEvent(e: Json): Json {
  return { id: Number(e.id), deviceId: e.device_id, boat: e.boat, kind: e.kind, severity: e.severity, title: e.title,
    detail: e.detail, data: e.data && typeof e.data === 'object' ? e.data : {}, occurredAt: e.occurred_at, receivedAt: e.received_at };
}

async function route(req: Request, ctx: CloudContext, path: string, phone: Phone): Promise<Response> {
  if (path === '/api/companion/signout') {
    if (req.method !== 'POST') return reply(405, { error: 'POST required' });
    await ctx.rpc('companion_signout', { p_session_id: phone.sessionId });
    return reply(200, { state: 'signed_out' });
  }
  if (path === '/api/companion/push') {
    if (req.method !== 'POST') return reply(405, { error: 'POST required' });
    let input: Json;
    try {
      input = await jsonBody(req, 8192);
      only(input, ['service', 'token']);
      if (!(input.token === null || (['fcm', 'apns'].includes(input.service)
        && typeof input.token === 'string' && PUSH_TOKEN[input.service as 'fcm' | 'apns'].test(input.token)))) throw new Error();
    } catch { return reply(400, { error: 'Invalid notification token.' }); }
    await ctx.rpc('companion_push_set', { p_session_id: phone.sessionId,
      p_service: input.token === null ? null : input.service, p_token: input.token });
    return reply(200, { push: input.token !== null });
  }
  if (req.method !== 'GET') return reply(405, { error: 'GET required' });
  if (path === '/api/companion/session') {
    if (!onlyParams(req, [])) return reply(400, { error: 'Invalid request.' });
    return reply(200, { account: { email: phone.email }, cloud: phone.cloud, push: phone.push });
  }
  if (path === '/api/companion/boats') {
    if (!onlyParams(req, [])) return reply(400, { error: 'Invalid request.' });
    const boats = object(await ctx.rpc('companion_boats', { p_account_id: phone.accountId }));
    const devices = (Array.isArray(boats.devices) ? boats.devices : []).map((d: Json) => ({ ...publicDevice(d),
      anchor: d.anchor ? validAnchor(d.anchor) : null, lastEventId: d.last_event_id ?? null }));
    return reply(200, { cloud: boats.cloud ? publicCloud(boats.cloud) : null, devices, telemetryKeys: TELEMETRY_KEYS });
  }
  if (path === '/api/companion/events') {
    const params = query(req);
    const device = params.get('device'), before = params.get('before'), limit = params.get('limit');
    if (!onlyParams(req, ['device', 'before', 'limit']) || (device !== null && !UUID.test(device))
      || (before !== null && !/^[1-9][0-9]{0,15}$/.test(before)) || (limit !== null && !/^(?:[1-9][0-9]?|100)$/.test(limit))) {
      return reply(400, { error: 'Invalid request.' });
    }
    const result = object(await ctx.rpc('companion_events', { p_account_id: phone.accountId, p_device_id: device,
      p_before: before === null ? null : Number(before), p_limit: limit === null ? 50 : Number(limit) }));
    return reply(200, { events: (Array.isArray(result.events) ? result.events : []).map(publicEvent) });
  }
  const device = deviceParam(req);
  if (path === '/api/companion/boat') {
    if (!device || !onlyParams(req, ['device'])) return reply(400, { error: 'Invalid request.' });
    const latest = object(await ctx.rpc('companion_boat', { p_account_id: phone.accountId, p_device_id: device }));
    if (latest.state !== 'ok') return reply(404, { error: 'Boat not found.' });
    const boat = publicBoat(latest);
    // The snapshot changes at most every 5 s; an unchanged one is answered with 304 and no body.
    const tag = `"${(await sha256(JSON.stringify(boat))).slice(0, 32)}"`;
    if (req.headers.get('If-None-Match') === tag) {
      return new Response(null, { status: 304, headers: { ETag: tag, 'Cache-Control': 'private, no-cache' } });
    }
    // snapshotAge: seconds since the server received it, so the phone's clock is never trusted.
    const age = boat.receivedAt ? Math.max(0, (Date.now() - Date.parse(boat.receivedAt)) / 1000) : null;
    const response = reply(200, { ...boat, snapshotAge: age === null ? null : Math.round(age * 10) / 10 });
    response.headers.set('ETag', tag);
    response.headers.set('Cache-Control', 'private, no-cache');
    return response;
  }
  if (path === '/api/companion/series') {
    const params = query(req);
    const keys = (params.get('keys') ?? '').split(',');
    const hours = Number(params.get('hours'));
    if (!device || !onlyParams(req, ['device', 'keys', 'hours']) || keys.length < 1 || keys.length > 4
      || !keys.every(key => Object.hasOwn(TELEMETRY_KEYS, key)) || !SERIES_HOURS.includes(hours)) {
      return reply(400, { error: 'Invalid history request.' });
    }
    const series = object(await ctx.rpc('cloud_boat_series', { p_account_id: phone.accountId, p_device_id: device,
      p_keys: keys, p_hours: hours }));
    if (series.state !== 'ok') return reply(404, { error: 'Boat not found.' });
    return reply(200, { bucketSeconds: series.bucket_seconds, series: series.series });
  }
  if (path === '/api/companion/track') {
    const hours = Number(query(req).get('hours'));
    if (!device || !onlyParams(req, ['device', 'hours']) || !TRACK_HOURS.includes(hours)) {
      return reply(400, { error: 'Invalid track request.' });
    }
    const track = object(await ctx.rpc('companion_track', { p_account_id: phone.accountId, p_device_id: device, p_hours: hours }));
    if (track.state !== 'ok') return reply(404, { error: 'Boat not found.' });
    return reply(200, { points: Array.isArray(track.points) ? track.points : [] });
  }
  return reply(404, { error: 'Not found' });
}

const ROUTES = ['/api/companion/token', '/api/companion/session', '/api/companion/signout', '/api/companion/push',
  '/api/companion/boats', '/api/companion/boat', '/api/companion/series', '/api/companion/track', '/api/companion/events'];
/** Phone routes use a bearer token and refuse cookies; they are exempt from the CSRF check. */
export function isCompanionRoute(path: string): boolean { return ROUTES.includes(path); }

export async function handleCompanion(req: Request, ctx: CloudContext): Promise<Response | null> {
  const path = new URL(req.url).pathname;
  if (!ROUTES.includes(path)) return null;
  const origin = req.headers.get('Origin');
  if (req.headers.has('Cookie') || (origin !== null && !APP_ORIGINS.includes(origin))) return reply(403, { error: 'Request rejected' });
  try {
    if (path === '/api/companion/token') {
      return req.method === 'POST' ? await exchange(req, ctx) : reply(405, { error: 'POST required' });
    }
    const phone = await phoneAuth(req, ctx);
    if (!phone) return reply(401, { error: 'Please sign in again.', signedIn: false });
    return await route(req, ctx, path, phone);
  } catch { return reply(503, { error: 'SKAGER Cloud is temporarily unavailable.' }); }
}
