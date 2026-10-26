/** SKAGER Companion push notifications (SCRUM-389). Sent directly from this function:
 *  - Android: Firebase Cloud Messaging HTTP v1 (service account, RS256 OAuth assertion).
 *  - iOS: Apple Push Notification service (token-based .p8 key, ES256 provider token).
 * Without credentials nothing is sent; events still reach the alarm log. Messages carry
 * the boat name and what happened, never a position. Credentials, tokens and responses
 * are never logged. */
type Json = Record<string, any>;
export interface PushContext {
  env: Record<string, string | undefined>;
  rpc(name: string, args: Record<string, unknown>): Promise<unknown>;
  fetch?: typeof fetch;
}
export type Target = { session_id: string; service: 'fcm' | 'apns'; token: string };
export type Message = { title: string; body: string; critical: boolean; data: Record<string, string> };
type Outcome = 'ok' | 'invalid' | 'error' | 'unconfigured';

const encoder = new TextEncoder();
const FCM_TOKEN_URL = 'https://oauth2.googleapis.com/token';
const FCM_SCOPE = 'https://www.googleapis.com/auth/firebase.messaging';
const APNS_HOSTS = ['api.push.apple.com', 'api.sandbox.push.apple.com'];
const APNS_TOKEN = /^[0-9a-f]{64,200}$/;
const FCM_PROJECT = /^[a-z][a-z0-9-]{4,40}$/;
/** Android channels created by the app: alarms sound and break through, events do not. */
export const ANDROID_CHANNELS = { critical: 'skager-alarms', info: 'skager-events' };

const base64url = (bytes: Uint8Array) => btoa(String.fromCharCode(...bytes)).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
const b64json = (value: unknown) => base64url(encoder.encode(JSON.stringify(value)));
function pemBytes(pem: string): Uint8Array {
  const body = pem.replace(/-----(BEGIN|END) PRIVATE KEY-----/g, '').replace(/\\n/g, '').replace(/\s+/g, '');
  if (!/^[A-Za-z0-9+/]+=*$/.test(body)) throw new Error('Invalid key');
  return Uint8Array.from(atob(body), c => c.charCodeAt(0));
}
export async function signJwt(header: Json, claims: Json, pem: string, kind: 'RS256' | 'ES256'): Promise<string> {
  const algorithm = kind === 'RS256' ? { name: 'RSASSA-PKCS1-v1_5', hash: 'SHA-256' } : { name: 'ECDSA', namedCurve: 'P-256' };
  const key = await crypto.subtle.importKey('pkcs8', pemBytes(pem), algorithm, false, ['sign']);
  const input = `${b64json(header)}.${b64json(claims)}`;
  // WebCrypto ECDSA yields the raw r||s form that JWS ES256 requires.
  const signature = await crypto.subtle.sign(kind === 'RS256' ? algorithm : { name: 'ECDSA', hash: 'SHA-256' }, key, encoder.encode(input));
  return `${input}.${base64url(new Uint8Array(signature))}`;
}

// Provider credentials are cached per isolate, never across services or keys.
let fcmCache: { key: string; token: string; until: number } | null = null;
let apnsCache: { key: string; token: string; until: number } | null = null;
export function resetPushCaches(): void { fcmCache = null; apnsCache = null; }

function fcmAccount(env: PushContext['env']): { project: string; email: string; key: string } | null {
  try {
    const account = JSON.parse(env.SKAGER_FCM_SERVICE_ACCOUNT ?? '');
    if (FCM_PROJECT.test(account.project_id) && typeof account.client_email === 'string'
      && /^[^\s@]+@[^\s@]+\.iam\.gserviceaccount\.com$/.test(account.client_email) && typeof account.private_key === 'string') {
      return { project: account.project_id, email: account.client_email, key: account.private_key };
    }
  } catch { /* unconfigured */ }
  return null;
}
async function fcmAccess(account: { email: string; key: string }, transport: typeof fetch): Promise<string | null> {
  const now = Math.floor(Date.now() / 1000);
  if (fcmCache && fcmCache.key === account.email && fcmCache.until > now + 60) return fcmCache.token;
  const assertion = await signJwt({ alg: 'RS256', typ: 'JWT' },
    { iss: account.email, scope: FCM_SCOPE, aud: FCM_TOKEN_URL, iat: now, exp: now + 3600 }, account.key, 'RS256');
  const response = await transport(FCM_TOKEN_URL, { method: 'POST', redirect: 'error', signal: AbortSignal.timeout(8000),
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `grant_type=${encodeURIComponent('urn:ietf:params:oauth:grant-type:jwt-bearer')}&assertion=${assertion}` });
  if (!response.ok) { await response.body?.cancel().catch(() => {}); return null; }
  const granted = await response.json().catch(() => null);
  if (!granted || typeof granted.access_token !== 'string') return null;
  fcmCache = { key: account.email, token: granted.access_token,
    until: now + Math.min(3600, Number(granted.expires_in) || 3600) };
  return granted.access_token;
}
export async function sendFcm(env: PushContext['env'], transport: typeof fetch, token: string, message: Message): Promise<Outcome> {
  const account = fcmAccount(env);
  if (!account) return 'unconfigured';
  if (!/^[A-Za-z0-9_:.-]{20,4096}$/.test(token)) return 'invalid';
  const access = await fcmAccess(account, transport);
  if (!access) return 'error';
  const response = await transport(`https://fcm.googleapis.com/v1/projects/${account.project}/messages:send`, {
    method: 'POST', redirect: 'error', signal: AbortSignal.timeout(8000),
    headers: { Authorization: `Bearer ${access}`, 'Content-Type': 'application/json' },
    body: JSON.stringify({ message: { token, notification: { title: message.title, body: message.body }, data: message.data,
      android: { priority: message.critical ? 'HIGH' : 'NORMAL', ttl: '3600s',
        notification: { channel_id: message.critical ? ANDROID_CHANNELS.critical : ANDROID_CHANNELS.info,
          tag: message.data.eventId } } } }),
  });
  if (response.ok) { await response.body?.cancel().catch(() => {}); return 'ok'; }
  const failure = await response.json().catch(() => null);
  const code = failure?.error?.details?.find?.((d: Json) => typeof d?.errorCode === 'string')?.errorCode ?? failure?.error?.status;
  if (response.status === 401) fcmCache = null;
  // UNREGISTERED: the app was removed or the token rotated. INVALID_ARGUMENT here is the token.
  return response.status === 404 || code === 'UNREGISTERED' || (response.status === 400 && code === 'INVALID_ARGUMENT')
    ? 'invalid' : 'error';
}

function apnsConfig(env: PushContext['env']): { key: string; keyId: string; team: string; topic: string; host: string } | null {
  const host = env.SKAGER_APNS_HOST || APNS_HOSTS[0];
  if (!env.SKAGER_APNS_KEY || !/^[A-Z0-9]{10}$/.test(env.SKAGER_APNS_KEY_ID ?? '') || !/^[A-Z0-9]{10}$/.test(env.SKAGER_APNS_TEAM_ID ?? '')
    || !/^[A-Za-z0-9.-]{3,100}$/.test(env.SKAGER_APNS_TOPIC ?? '') || !APNS_HOSTS.includes(host)) return null;
  return { key: env.SKAGER_APNS_KEY, keyId: env.SKAGER_APNS_KEY_ID!, team: env.SKAGER_APNS_TEAM_ID!, topic: env.SKAGER_APNS_TOPIC!, host };
}
export async function sendApns(env: PushContext['env'], transport: typeof fetch, token: string, message: Message): Promise<Outcome> {
  const config = apnsConfig(env);
  if (!config) return 'unconfigured';
  if (!APNS_TOKEN.test(token)) return 'invalid';
  const now = Math.floor(Date.now() / 1000);
  // Apple accepts a provider token for up to an hour and rejects refreshing it more than every 20 minutes.
  if (!apnsCache || apnsCache.key !== config.keyId + config.team || apnsCache.until <= now) {
    apnsCache = { key: config.keyId + config.team, until: now + 40 * 60,
      token: await signJwt({ alg: 'ES256', kid: config.keyId }, { iss: config.team, iat: now }, config.key, 'ES256') };
  }
  const response = await transport(`https://${config.host}/3/device/${token}`, {
    method: 'POST', redirect: 'error', signal: AbortSignal.timeout(8000),
    headers: { authorization: `bearer ${apnsCache.token}`, 'apns-topic': config.topic, 'apns-push-type': 'alert',
      'apns-priority': message.critical ? '10' : '5', 'apns-expiration': String(now + 3600), 'content-type': 'application/json' },
    body: JSON.stringify({ aps: { alert: { title: message.title, body: message.body }, sound: 'default',
      'interruption-level': message.critical ? 'time-sensitive' : 'active', 'thread-id': message.data.deviceId }, ...message.data }),
  });
  if (response.ok) { await response.body?.cancel().catch(() => {}); return 'ok'; }
  const reason = (await response.json().catch(() => null))?.reason;
  if (reason === 'ExpiredProviderToken' || reason === 'InvalidProviderToken') apnsCache = null;
  return response.status === 410 || ['BadDeviceToken', 'Unregistered', 'DeviceTokenNotForTopic'].includes(reason)
    ? 'invalid' : 'error';
}

/** Notification text for one stored event. A late report says so instead of looking current. */
export function pushMessage(event: Json, now = Date.now()): Message {
  const boat = typeof event.boat === 'string' ? event.boat : 'Your boat';
  const late = Date.parse(event.received_at) - Date.parse(event.occurred_at);
  const minutes = Math.round((now - Date.parse(event.occurred_at)) / 60000);
  const detail = typeof event.detail === 'string' && event.detail ? event.detail : '';
  const lateNote = Number.isFinite(late) && late > 120000 ? ` Reported ${minutes} min after it happened.` : '';
  return { title: `${event.title} · ${boat}`, body: `${detail}${lateNote}`.trim() || boat,
    critical: event.severity === 'critical',
    data: { kind: String(event.kind), eventId: String(event.id), deviceId: String(event.device_id) } };
}

/** Sends every due notification once. Safe to run concurrently: events are claimed in SQL. */
export async function dispatchPush(ctx: PushContext, limit = 20): Promise<{ events: number; sent: number }> {
  const transport = ctx.fetch ?? fetch;
  const claimed = await ctx.rpc('cloud_push_claim', { p_limit: limit });
  const events = Array.isArray(claimed) ? claimed : [];
  let sent = 0;
  for (const event of events) {
    const message = pushMessage(event);
    const targets: Target[] = Array.isArray(event.targets) ? event.targets : [];
    const outcomes = await Promise.all(targets.map(target => (target.service === 'fcm'
      ? sendFcm(ctx.env, transport, target.token, message) : sendApns(ctx.env, transport, target.token, message))
      .catch((): Outcome => 'error')));
    const invalid = targets.filter((_, i) => outcomes[i] === 'invalid').map(t => t.token);
    // One delivered phone ends the attempt so no phone is notified twice by a retry.
    const state = outcomes.includes('ok') || targets.length === 0 ? 'sent'
      : outcomes.every(o => o === 'unconfigured') ? 'unconfigured'
      : outcomes.includes('error') ? 'failed' : 'sent';
    if (outcomes.includes('ok')) sent++;
    await ctx.rpc('cloud_push_done', { p_event_id: event.id, p_state: state, p_invalid: invalid.length ? invalid : null });
  }
  return { events: events.length, sent };
}

/** Lets a request answer first when the runtime supports it (Supabase EdgeRuntime). */
export async function inBackground(work: Promise<unknown>): Promise<void> {
  const runtime = (globalThis as Json).EdgeRuntime;
  const guarded = work.catch(() => { /* retried by the scheduled job */ });
  if (runtime && typeof runtime.waitUntil === 'function') runtime.waitUntil(guarded);
  else await guarded;
}
