/** SKAGER Cloud. Two audiences:
 *  - /api/device/*: the SKAGER app. Never cookies or CSRF: pairing uses a secret device
 *    code, later requests a bearer device token. A browser Origin header is refused, so a
 *    web page cannot drive these routes with a visitor's network position.
 *  - /api/cloud/*: the logged-in customer (cookie session; the router checks CSRF).
 * Tokens and device codes are stored only as SHA-256. Missing Cloud never blocks the
 * app: it only withholds the update feed and telemetry storage. */
import { dispatchPush, inBackground } from './push.ts';

export interface CloudContext {
  env: Record<string, string | undefined>;
  getAccount(request: Request): Promise<{ id: string; email?: string; passwordSet?: boolean } | null>;
  rpc(name: string, args: Record<string, unknown>): Promise<unknown>;
  /** Outbound HTTPS for push notifications (SKAGER Companion). */
  fetch?: typeof fetch;
}
type Json = Record<string, any>;

export const LINK_URL = 'https://beta.skager.app/link/';
const LINK_SECONDS = 600;
const POLL_SECONDS = 5;
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/;
const HEX64 = /^[a-f0-9]{64}$/;
const TOKEN = /^skg1_[a-f0-9]{64}$/;
const USER_CODE = /^[BCDFGHJKLMNPQRSTVWXZ2-9]{8}$/;
const CODE_ALPHABET = 'BCDFGHJKLMNPQRSTVWXZ23456789';
const VERSION = /^[0-9A-Za-z][0-9A-Za-z.+-]{0,39}$/;
const QUALITIES = ['live', 'aging', 'stale', 'estimated', 'uncertain'];
const MAX_TELEMETRY_BYTES = 16384;
const EVENT_ID = /^[A-Za-z0-9-]{8,64}$/;
const PKCE_CHALLENGE = /^[A-Za-z0-9_-]{43}$/;
const APP_STATE = /^[A-Za-z0-9_-]{16,64}$/;
/** What the boat may report; notification wording is fixed on the server. */
export const DEVICE_EVENT_KINDS = ['anchor_alarm', 'anchor_alarm_cleared', 'anchor_armed', 'anchor_disarmed',
  'alert_critical', 'alert_cleared'];
const SERIES_HOURS = [1, 6, 24, 72, 168, 720];

/** Every value SKAGER may upload. Units are fixed here so the web never guesses. */
export const TELEMETRY_KEYS: Record<string, { label: string; unit: string; min: number; max: number }> = {
  'navigation.latitude': { label: 'Latitude', unit: 'deg', min: -90, max: 90 },
  'navigation.longitude': { label: 'Longitude', unit: 'deg', min: -180, max: 180 },
  'navigation.sog': { label: 'Speed over ground', unit: 'kn', min: 0, max: 100 },
  'navigation.cog': { label: 'Course over ground', unit: 'deg', min: 0, max: 360 },
  'navigation.heading': { label: 'Heading', unit: 'deg', min: 0, max: 360 },
  'navigation.stw': { label: 'Speed through water', unit: 'kn', min: 0, max: 100 },
  'wind.apparent_speed': { label: 'Apparent wind speed', unit: 'kn', min: 0, max: 200 },
  'wind.apparent_angle': { label: 'Apparent wind angle', unit: 'deg', min: -180, max: 360 },
  'wind.true_speed': { label: 'True wind speed', unit: 'kn', min: 0, max: 200 },
  'wind.true_angle': { label: 'True wind angle', unit: 'deg', min: -180, max: 360 },
  'environment.depth': { label: 'Depth below transducer', unit: 'm', min: 0, max: 12000 },
  'environment.water_temperature': { label: 'Water temperature', unit: 'C', min: -5, max: 50 },
  'environment.pressure': { label: 'Air pressure', unit: 'hPa', min: 800, max: 1100 },
  'steering.rudder': { label: 'Rudder', unit: 'deg', min: -90, max: 90 },
  'steering.heel': { label: 'Heel', unit: 'deg', min: -90, max: 90 },
  'propulsion.electrical_power': { label: 'Motor power', unit: 'kW', min: -1000, max: 1000 },
  'propulsion.shaft_power': { label: 'Shaft power', unit: 'kW', min: -1000, max: 1000 },
  'propulsion.rpm': { label: 'Motor speed', unit: 'rpm', min: -20000, max: 20000 },
  'propulsion.gear': { label: 'Gear', unit: 'code', min: 0, max: 2 },
  'propulsion.regeneration': { label: 'Regeneration', unit: 'code', min: 0, max: 2 },
  'propulsion.motor_temperature': { label: 'Motor temperature', unit: 'C', min: -50, max: 250 },
  'propulsion.coolant_temperature': { label: 'Coolant temperature', unit: 'C', min: -50, max: 250 },
  'battery.voltage': { label: 'Battery voltage', unit: 'V', min: 0, max: 1500 },
  'battery.current': { label: 'Battery current', unit: 'A', min: -5000, max: 5000 },
  'battery.soc': { label: 'State of charge', unit: '%', min: 0, max: 100 },
  'battery.soh': { label: 'State of health', unit: '%', min: 0, max: 100 },
  'battery.capacity': { label: 'Usable capacity', unit: 'kWh', min: 0, max: 10000 },
  'battery.net_discharge': { label: 'Net discharge', unit: 'kW', min: -1000, max: 1000 },
  'tanks.fresh_water': { label: 'Fresh water', unit: '%', min: 0, max: 100 },
  'tanks.fuel': { label: 'Fuel', unit: '%', min: 0, max: 100 },
  'tanks.waste': { label: 'Waste', unit: '%', min: 0, max: 100 },
  'tanks.other': { label: 'Other tank', unit: '%', min: 0, max: 100 },
  'connectivity.input_rate': { label: 'Instrument messages', unit: 'msg/s', min: 0, max: 100000 },
};

const encoder = new TextEncoder();
const invalid = () => new Error('Invalid request');
function requireValue(value: unknown): asserts value { if (!value) throw invalid(); }
function reply(status: number, value: Json | null): Response {
  return new Response(value === null ? null : JSON.stringify(value), { status, headers: {
    'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'private, no-store, max-age=0',
    'X-Content-Type-Options': 'nosniff', 'Referrer-Policy': 'no-referrer' } });
}
async function sha256(value: string): Promise<string> {
  return [...new Uint8Array(await crypto.subtle.digest('SHA-256', encoder.encode(value)))]
    .map(byte => byte.toString(16).padStart(2, '0')).join('');
}
function randomHex(bytes: number): string {
  return Array.from(crypto.getRandomValues(new Uint8Array(bytes)), b => b.toString(16).padStart(2, '0')).join('');
}
export function userCode(): string {
  // Rejection sampling keeps every symbol equally likely (256 is not a multiple of 28).
  let code = '';
  while (code.length < 8) {
    for (const byte of crypto.getRandomValues(new Uint8Array(16))) {
      if (byte < 252 && code.length < 8) code += CODE_ALPHABET[byte % CODE_ALPHABET.length];
    }
  }
  return code;
}
/** Accepts what a person types: any case, spaces or a dash ("bcdf-2345"). */
export function normalizeUserCode(value: unknown): string | null {
  if (typeof value !== 'string' || value.length > 20) return null;
  const code = value.toUpperCase().replace(/[\s-]/g, '');
  return USER_CODE.test(code) ? code : null;
}
async function body(req: Request, maximum = 2048): Promise<Json> {
  requireValue(req.headers.get('Content-Type')?.split(';')[0].trim().toLowerCase() === 'application/json');
  const declared = req.headers.get('Content-Length');
  requireValue(!declared || Number(declared) <= maximum);
  const bytes = new Uint8Array(await req.arrayBuffer());
  requireValue(bytes.length <= maximum);
  const value = JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(bytes));
  requireValue(value && typeof value === 'object' && !Array.isArray(value));
  return value;
}
function only(value: Json, keys: string[]): void {
  requireValue(Object.keys(value).every(key => keys.includes(key)));
}
function object(value: unknown): Json {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new Error('Invalid RPC response');
  return value as Json;
}
const plainText = (value: unknown, max: number) => typeof value === 'string' && value.trim().length >= 1
  && value.length <= max && !/[\u0000-\u001f\u007f]/.test(value);

function publicCloud(value: unknown): Json {
  const cloud = object(value);
  return { active: cloud.active === true, status: cloud.status ?? null,
    renewsAt: cloud.current_period_end ?? null, cancelAtPeriodEnd: cloud.cancel_at_period_end === true };
}
function publicPhone(value: unknown): Json {
  const p = object(value);
  requireValue(typeof p.id === 'string' && UUID.test(p.id));
  return { id: p.id, name: p.name, platform: p.platform, appVersion: p.app_version, createdAt: p.created_at,
    lastSeenAt: p.last_seen_at, push: p.push === true };
}
function publicDevice(value: unknown): Json {
  const d = object(value);
  requireValue(typeof d.id === 'string' && UUID.test(d.id));
  return { id: d.id, name: d.name, platform: d.platform, appVersion: d.app_version,
    telemetryEnabled: d.telemetry_enabled === true, createdAt: d.created_at, lastSeenAt: d.last_seen_at,
    lastDataAt: d.last_data_at ?? null };
}

/** Validates an upload against TELEMETRY_KEYS. Unknown keys or values are refused whole. */
export function validTelemetry(items: unknown): Json | null {
  if (!items || typeof items !== 'object' || Array.isArray(items)) return null;
  const entries = Object.entries(items as Json);
  if (entries.length < 1 || entries.length > Object.keys(TELEMETRY_KEYS).length) return null;
  const output: Json = {};
  for (const [key, item] of entries) {
    const spec = Object.hasOwn(TELEMETRY_KEYS, key) ? TELEMETRY_KEYS[key] : null;
    if (!spec || !item || typeof item !== 'object' || Array.isArray(item)) return null;
    if (!Object.keys(item).every(k => ['v', 'age', 'q', 'src'].includes(k))) return null;
    const { v, age, q, src } = item as Json;
    if (typeof v !== 'number' || !Number.isFinite(v) || v < spec.min || v > spec.max) return null;
    if (typeof age !== 'number' || !Number.isFinite(age) || age < 0 || age > 86400) return null;
    if (!QUALITIES.includes(q)) return null;
    if (src !== undefined && !(typeof src === 'string' && src.length <= 64 && !/[\u0000-\u001f\u007f]/.test(src))) return null;
    output[key] = { v, age: Math.round(age * 10) / 10, q, ...(src ? { src } : {}) };
  }
  return output;
}

const finite = (value: unknown, min: number, max: number) =>
  typeof value === 'number' && Number.isFinite(value) && value >= min && value <= max;

/** The anchor watch as OpenCPN reports it on the boat. The alarm is OpenCPN's own decision,
 * copied, never recalculated. A negative radius is OpenCPN's "alarm when inside" watch.
 * Returns null for anything malformed; {armed:false} when no watch is set. */
export function validAnchor(value: unknown): Json | null {
  if (!value || typeof value !== 'object' || Array.isArray(value)) return null;
  const a = value as Json;
  if (!Object.keys(a).every(k => ['armed', 'alarm', 'lat', 'lon', 'radius_m', 'distance_m', 'age', 'src'].includes(k))) return null;
  if (typeof a.armed !== 'boolean') return null;
  if (!a.armed) return Object.keys(a).length === 1 ? { armed: false } : null;
  if (typeof a.alarm !== 'boolean') return null;
  if ((a.lat === undefined) !== (a.lon === undefined)) return null;
  if (a.lat !== undefined && !(finite(a.lat, -90, 90) && finite(a.lon, -180, 180))) return null;
  if (a.radius_m !== undefined && !(finite(Math.abs(a.radius_m), 1, 10000))) return null;
  if (a.distance_m !== undefined && !finite(a.distance_m, 0, 100000)) return null;
  if (a.age !== undefined && !finite(a.age, 0, 86400)) return null;
  if (a.src !== undefined && !plainText(a.src, 64)) return null;
  if ((a.distance_m === undefined) !== (a.age === undefined)) return null;
  const out: Json = { armed: true, alarm: a.alarm };
  if (a.lat !== undefined) { out.lat = a.lat; out.lon = a.lon; }
  if (a.radius_m !== undefined) out.radius_m = a.radius_m;
  if (a.distance_m !== undefined) { out.distance_m = Math.round(a.distance_m * 10) / 10; out.age = Math.round(a.age * 10) / 10; }
  if (a.src !== undefined) out.src = a.src;
  return out;
}

/** Up to 20 events from the boat. Each carries the boat's own id so a retry is stored once. */
export function validEvents(value: unknown): Json[] | null {
  if (!Array.isArray(value) || value.length < 1 || value.length > 20) return null;
  const out: Json[] = [];
  for (const item of value) {
    if (!item || typeof item !== 'object' || Array.isArray(item)) return null;
    const e = item as Json;
    if (!Object.keys(e).every(k => ['id', 'kind', 'age', 'alert', 'data'].includes(k))) return null;
    if (typeof e.id !== 'string' || !EVENT_ID.test(e.id) || !DEVICE_EVENT_KINDS.includes(e.kind)) return null;
    if (!finite(e.age, 0, 7 * 86400)) return null;
    const alertKind = e.kind === 'alert_critical' || e.kind === 'alert_cleared';
    if (e.alert !== undefined && !(alertKind && plainText(e.alert, 80))) return null;
    const event: Json = { id: e.id, kind: e.kind, age: Math.round(e.age * 10) / 10 };
    if (e.alert !== undefined) event.alert = e.alert.trim();
    if (e.data !== undefined) {
      if (!e.data || typeof e.data !== 'object' || Array.isArray(e.data)) return null;
      if (!Object.keys(e.data).every(k => ['distance_m', 'radius_m'].includes(k))) return null;
      if (e.data.distance_m !== undefined && !finite(e.data.distance_m, 0, 100000)) return null;
      if (e.data.radius_m !== undefined && !finite(Math.abs(e.data.radius_m), 1, 10000)) return null;
      event.data = { ...e.data };
    }
    out.push(event);
  }
  return new Set(out.map(e => e.id)).size === out.length ? out : null;
}

// ---------------------------------------------------------------- device routes
async function throttled(ctx: CloudContext, keys: string[], limit: number, seconds: number): Promise<boolean> {
  return await ctx.rpc('skager_auth_throttle', { p_keys: keys, p_limit: limit, p_seconds: seconds }) !== true;
}

async function linkStart(req: Request, ctx: CloudContext): Promise<Response> {
  let input: Json;
  try {
    input = await body(req);
    only(input, ['name', 'platform', 'version']);
    requireValue(plainText(input.name, 60) && ['windows', 'linux'].includes(input.platform)
      && typeof input.version === 'string' && VERSION.test(input.version));
  } catch { return reply(400, { error: 'Invalid pairing request.' }); }
  if (await throttled(ctx, ['cloud:link:start'], 100, 60)) return reply(429, { error: 'Too many requests. Try again in a minute.' });
  for (let attempt = 0; attempt < 3; attempt++) {
    const deviceCode = randomHex(32);
    const code = userCode();
    const started = object(await ctx.rpc('cloud_link_start', { p_device_hash: await sha256(deviceCode),
      p_user_code: code, p_name: input.name.trim(), p_platform: input.platform, p_version: input.version }));
    if (started.state === 'pending') {
      return reply(200, { deviceCode, userCode: code, verificationUrl: LINK_URL,
        expiresIn: LINK_SECONDS, interval: POLL_SECONDS });
    }
  }
  return reply(503, { error: 'Pairing is temporarily unavailable.' });
}

async function linkPoll(req: Request, ctx: CloudContext): Promise<Response> {
  let deviceCode: string;
  try {
    const input = await body(req);
    only(input, ['deviceCode']);
    requireValue(typeof input.deviceCode === 'string' && HEX64.test(input.deviceCode));
    deviceCode = input.deviceCode;
  } catch { return reply(400, { error: 'Invalid pairing poll.' }); }
  const deviceHash = await sha256(deviceCode);
  if (await throttled(ctx, ['cloud:poll:' + deviceHash.slice(0, 32)], 20, 60)) return reply(429, { state: 'slow_down' });
  const token = 'skg1_' + randomHex(32);
  const claimed = object(await ctx.rpc('cloud_link_claim', { p_device_hash: deviceHash, p_token_hash: await sha256(token) }));
  if (claimed.state === 'approved' && typeof claimed.device_id === 'string' && UUID.test(claimed.device_id)) {
    return reply(200, { state: 'approved', token, deviceId: claimed.device_id });
  }
  if (['pending', 'denied', 'expired'].includes(claimed.state)) return reply(200, { state: claimed.state });
  throw new Error('Invalid claim');
}

type DeviceAuth = { deviceId: string; accountId: string; email: string; name: string; cloud: Json };
/** Resolves the bearer device token; null for missing, malformed, revoked or unknown tokens. */
export async function deviceAuth(req: Request, ctx: CloudContext): Promise<DeviceAuth | null> {
  // The Worker moves the app's "Authorization: Bearer" here; the Supabase gateway
  // never sees a non-JWT Authorization header.
  const token = req.headers.get('X-Skager-Device') ?? '';
  if (!TOKEN.test(token)) return null;
  const version = req.headers.get('X-Skager-Version');
  const found = object(await ctx.rpc('cloud_device_auth', { p_token_hash: await sha256(token),
    p_version: version && VERSION.test(version) ? version : null }));
  if (found.state !== 'active' || typeof found.device_id !== 'string' || !UUID.test(found.device_id)) return null;
  return { deviceId: found.device_id, accountId: found.account_id, email: found.email, name: found.name,
    cloud: publicCloud(found.cloud) };
}

async function deviceRoute(req: Request, ctx: CloudContext, path: string): Promise<Response> {
  const device = await deviceAuth(req, ctx);
  if (!device) return reply(401, { error: 'This SKAGER is not linked to an account.', linked: false });
  if (path === '/api/device/status') {
    if (req.method !== 'GET') return reply(405, { error: 'GET required' });
    return reply(200, { linked: true, account: { email: device.email }, device: { id: device.deviceId, name: device.name },
      cloud: device.cloud, services: { updates: device.cloud.active, telemetry: device.cloud.active } });
  }
  if (path === '/api/device/update-access') {
    if (req.method !== 'GET') return reply(405, { error: 'GET required' });
    return device.cloud.active ? reply(204, null)
      : reply(402, { error: 'Automatic updates are part of SKAGER Cloud.', cloud: device.cloud });
  }
  if (req.method !== 'POST') return reply(405, { error: 'POST required' });
  if (path === '/api/device/unlink') {
    await ctx.rpc('cloud_device_forget', { p_device_id: device.deviceId });
    return reply(200, { state: 'unlinked' });
  }
  if (path === '/api/device/telemetry/stop') {
    await ctx.rpc('cloud_telemetry_stop', { p_device_id: device.deviceId });
    return reply(200, { state: 'stopped' });
  }
  if (!device.cloud.active) return reply(402, { error: 'Boat data needs SKAGER Cloud.', cloud: device.cloud });
  if (path === '/api/device/events') {
    let events: Json[] | null;
    try {
      const input = await body(req, 8192);
      only(input, ['events']);
      events = validEvents(input.events);
      requireValue(events);
    } catch { return reply(400, { error: 'Invalid events.' }); }
    const stored = object(await ctx.rpc('cloud_event_put', { p_device_id: device.deviceId, p_events: events }));
    if (stored.state === 'revoked') return reply(401, { error: 'This SKAGER is no longer linked.', linked: false });
    if (stored.state === 'inactive') return reply(402, { error: 'Boat events need SKAGER Cloud.' });
    if (stored.state !== 'stored') throw new Error('Invalid event result');
    // Notifications go out after the boat has its answer; the scheduled job retries.
    if (Number(stored.new) > 0) await inBackground(dispatchPush(ctx));
    return reply(202, { state: 'stored', new: Number(stored.new) || 0 });
  }
  // /api/device/telemetry
  let items: Json | null;
  let anchor: Json | null = null;
  try {
    const input = await body(req, MAX_TELEMETRY_BYTES);
    only(input, ['items', 'anchor']);
    items = validTelemetry(input.items);
    requireValue(items);
    if (input.anchor !== undefined) {
      anchor = validAnchor(input.anchor);
      requireValue(anchor);
    }
  } catch { return reply(400, { error: 'Invalid boat data.' }); }
  const stored = object(await ctx.rpc('cloud_telemetry_put', { p_device_id: device.deviceId, p_items: items }));
  // Authorization is checked again while holding the database rows used by revocation.
  if (stored.state === 'revoked') return reply(401, { error: 'This SKAGER is no longer linked.', linked: false });
  if (stored.state === 'inactive') return reply(402, { error: 'Boat data needs SKAGER Cloud.' });
  if (stored.state === 'throttled') return reply(429, { state: 'throttled' });
  if (stored.state !== 'stored') throw new Error('Invalid telemetry result');
  // Older SKAGER versions send no anchor field: stored as unknown, never as "off".
  const extra = object(await ctx.rpc('cloud_anchor_put', { p_device_id: device.deviceId, p_anchor: anchor }));
  if (extra.restored === true) await inBackground(dispatchPush(ctx));
  return reply(202, { state: 'stored' });
}

// ---------------------------------------------------------------- customer routes
async function customer(req: Request, ctx: CloudContext): Promise<{ id: string } | Response> {
  const account = await ctx.getAccount(req);
  if (!account || !UUID.test(account.id)) return reply(401, { error: 'Sign in required.' });
  if (account.passwordSet === false) return reply(403, { error: 'Choose a password first.', setup: true });
  return account;
}

async function customerRoute(req: Request, ctx: CloudContext, path: string): Promise<Response> {
  const account = await customer(req, ctx);
  if (account instanceof Response) return account;
  if (path === '/api/cloud/overview') {
    if (req.method !== 'GET') return reply(405, { error: 'GET required' });
    const overview = object(await ctx.rpc('cloud_overview', { p_account_id: account.id }));
    const phones = await ctx.rpc('companion_phones', { p_account_id: account.id });
    return reply(200, { cloud: overview.cloud ? publicCloud(overview.cloud) : null,
      devices: (Array.isArray(overview.devices) ? overview.devices : []).map(publicDevice),
      phones: (Array.isArray(phones) ? phones : []).map(publicPhone),
      telemetryKeys: TELEMETRY_KEYS });
  }
  if (req.method !== 'POST') return reply(405, { error: 'POST required' });
  let input: Json;
  try { input = await body(req); } catch { return reply(400, { error: 'Invalid request.' }); }
  if (path === '/api/cloud/companion/authorize') {
    // The signed-in page hands SKAGER Companion a one-time code bound to the app's
    // PKCE challenge; the app redeems it with its verifier (companion.ts).
    try {
      only(input, ['challenge']);
      requireValue(typeof input.challenge === 'string' && PKCE_CHALLENGE.test(input.challenge));
    } catch { return reply(400, { error: 'This sign-in link is not valid. Start again in the app.' }); }
    const code = randomHex(32);
    const created = object(await ctx.rpc('companion_grant_create', { p_account_id: account.id,
      p_code_hash: await sha256(code), p_challenge: input.challenge }));
    if (created.state === 'throttled') return reply(429, { error: 'Too many sign-in attempts. Wait two minutes and try again.' });
    if (created.state !== 'ok') return reply(403, { error: 'This account cannot use SKAGER Companion.' });
    return reply(200, { code });
  }
  if (path === '/api/cloud/phones/revoke') {
    try {
      only(input, ['phoneId']);
      requireValue(typeof input.phoneId === 'string' && UUID.test(input.phoneId));
    } catch { return reply(400, { error: 'Invalid phone.' }); }
    const result = object(await ctx.rpc('companion_phone_revoke', { p_account_id: account.id, p_session_id: input.phoneId }));
    return result.state === 'revoked' ? reply(200, { state: 'revoked' }) : reply(404, { error: 'Phone not found.' });
  }
  if (path === '/api/cloud/link/find' || path === '/api/cloud/link/decide') {
    const code = normalizeUserCode(input.code);
    const decide = path.endsWith('/decide');
    try {
      only(input, decide ? ['code', 'approve'] : ['code']);
      requireValue(code && (!decide || typeof input.approve === 'boolean'));
    } catch { return reply(400, { error: 'Enter the 8-character code shown in SKAGER.' }); }
    if (await throttled(ctx, ['cloud:code:' + account.id], 20, 600)) {
      return reply(429, { error: 'Too many attempts. Wait a few minutes and try again.' });
    }
    const result = object(decide
      ? await ctx.rpc('cloud_link_decide', { p_account_id: account.id, p_user_code: code, p_approve: input.approve })
      : await ctx.rpc('cloud_link_find', { p_account_id: account.id, p_user_code: code }));
    if (result.state === 'unknown') return reply(404, { error: 'That code is not valid or has expired. Check SKAGER for a new code.' });
    return reply(200, { state: result.state, name: result.name, platform: result.platform ?? null,
      appVersion: result.app_version ?? null, expiresAt: result.expires_at ?? null });
  }
  const deviceId = input.deviceId;
  if (typeof deviceId !== 'string' || !UUID.test(deviceId)) return reply(400, { error: 'Invalid device.' });
  if (path === '/api/cloud/devices/revoke') {
    if (Object.keys(input).length !== 1) return reply(400, { error: 'Invalid request.' });
    const result = object(await ctx.rpc('cloud_device_revoke', { p_account_id: account.id, p_device_id: deviceId }));
    return result.state === 'revoked' ? reply(200, { state: 'revoked' }) : reply(404, { error: 'Device not found.' });
  }
  if (path === '/api/cloud/boat') {
    if (Object.keys(input).length !== 1) return reply(400, { error: 'Invalid request.' });
    const latest = object(await ctx.rpc('cloud_boat_latest', { p_account_id: account.id, p_device_id: deviceId }));
    if (latest.state !== 'ok') return reply(404, { error: 'Boat not found.' });
    // Values are re-validated on the way out as well: the page only ever renders known keys.
    return reply(200, { device: publicDevice(latest.device), receivedAt: latest.received_at ?? null,
      items: latest.items ? validTelemetry(latest.items) ?? {} : {} });
  }
  if (path === '/api/cloud/boat/series') {
    try {
      only(input, ['deviceId', 'keys', 'hours']);
      requireValue(Array.isArray(input.keys) && input.keys.length >= 1 && input.keys.length <= 4
        && input.keys.every((key: unknown) => typeof key === 'string' && Object.hasOwn(TELEMETRY_KEYS, key))
        && SERIES_HOURS.includes(input.hours));
    } catch { return reply(400, { error: 'Invalid history request.' }); }
    const series = object(await ctx.rpc('cloud_boat_series', { p_account_id: account.id, p_device_id: deviceId,
      p_keys: input.keys, p_hours: input.hours }));
    if (series.state !== 'ok') return reply(404, { error: 'Boat not found.' });
    return reply(200, { bucketSeconds: series.bucket_seconds, series: series.series });
  }
  return reply(404, { error: 'Not found' });
}

const DEVICE_ROUTES = ['/api/device/link', '/api/device/link/poll', '/api/device/status', '/api/device/update-access',
  '/api/device/telemetry', '/api/device/telemetry/stop', '/api/device/unlink', '/api/device/events'];
const CUSTOMER_ROUTES = ['/api/cloud/overview', '/api/cloud/link/find', '/api/cloud/link/decide',
  '/api/cloud/devices/revoke', '/api/cloud/boat', '/api/cloud/boat/series', '/api/cloud/companion/authorize',
  '/api/cloud/phones/revoke'];

/** Device routes are exempt from the browser CSRF check; see the module comment. */
export function isDeviceRoute(path: string): boolean { return DEVICE_ROUTES.includes(path); }

export async function handleCloud(req: Request, ctx: CloudContext): Promise<Response | null> {
  const path = new URL(req.url).pathname;
  if (DEVICE_ROUTES.includes(path)) {
    if (req.headers.has('Origin') || req.headers.has('Cookie')) return reply(403, { error: 'Request rejected' });
    try {
      if (path === '/api/device/link' || path === '/api/device/link/poll') {
        if (req.method !== 'POST') return reply(405, { error: 'POST required' });
        return path === '/api/device/link' ? await linkStart(req, ctx) : await linkPoll(req, ctx);
      }
      return await deviceRoute(req, ctx, path);
    } catch { return reply(503, { error: 'SKAGER Cloud is temporarily unavailable.' }); }
  }
  if (CUSTOMER_ROUTES.includes(path)) {
    try { return await customerRoute(req, ctx, path); }
    catch { return reply(503, { error: 'SKAGER Cloud is temporarily unavailable.' }); }
  }
  return null;
}

// Shared with the SKAGER Companion phone API (companion.ts).
export { APP_STATE, publicCloud, publicDevice, reply, sha256, randomHex, object, body as jsonBody, only };
