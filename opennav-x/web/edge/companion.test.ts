import assert from 'node:assert/strict';
import test from 'node:test';
import { handleCloud, validAnchor, validEvents } from './cloud.ts';
import type { CloudContext } from './cloud.ts';
import { handleCompanion, isCompanionRoute, pkceChallenge } from './companion.ts';
import { dispatchPush, pushMessage, resetPushCaches, sendApns, sendFcm, signJwt } from './push.ts';
import { appRequest, appReturnUrl, renderAppAuthorize, renderPhones } from './static/portal.js';

const OWNER = '88888888-8888-4888-8888-888888888888';
const DEVICE = '22222222-2222-4222-8222-222222222222';
const SESSION = '33333333-3333-4333-8333-333333333333';
const PHONE = 'skc1_' + 'c'.repeat(64);
const DEVICE_TOKEN = 'skg1_' + 'd'.repeat(64);
const VERIFIER = 'dBjftJeZ4CVP-mB92K27uhbUJU1p1r_wW1gFWFOEjXk'; // RFC 7636 appendix B

function fake(overrides: Record<string, (args: any) => unknown> = {}) {
  const calls: { name: string; args: any }[] = [];
  const rpc = async (name: string, args: any) => {
    calls.push({ name, args });
    if (overrides[name]) return overrides[name](args);
    switch (name) {
      case 'skager_auth_throttle': return true;
      case 'companion_session_auth': return args.p_token_hash ? { state: 'active', session_id: SESSION, account_id: OWNER,
        email: 'skipper@example.test', push: false, cloud: { active: true, status: 'active' } } : { state: 'unknown' };
      case 'cloud_device_auth': return { state: 'active', device_id: DEVICE, account_id: OWNER, email: 'skipper@example.test',
        name: 'Aurora helm', cloud: { active: true, status: 'active' } };
      case 'cloud_push_claim': return [];
      default: throw new Error('Unexpected RPC ' + name);
    }
  };
  const ctx: CloudContext = { env: {}, rpc, getAccount: async () => ({ id: OWNER, passwordSet: true }) };
  return { ctx, calls };
}
const phone = (path: string, init: RequestInit & { headers?: Record<string, string> } = {}) => new Request('https://beta.skager.app' + path,
  { ...init, headers: { 'X-Skager-Companion': PHONE, ...(init.body ? { 'Content-Type': 'application/json' } : {}), ...init.headers } });

test('PKCE uses S256 exactly as RFC 7636', async () => {
  assert.equal(await pkceChallenge(VERIFIER), 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM');
});

test('the token exchange redeems a code with its verifier and never stores the token', async () => {
  const { ctx, calls } = fake({ companion_grant_redeem: args => args.p_challenge === 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM'
    ? { state: 'ok', session_id: SESSION } : { state: 'invalid' } });
  const request = (body: unknown) => new Request('https://beta.skager.app/api/companion/token', { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const good = { code: 'a'.repeat(64), verifier: VERIFIER, name: 'Pixel 9', platform: 'android', version: '1.0.0' };
  const response = await handleCompanion(request(good), ctx);
  assert.equal(response!.status, 200);
  const { token } = await response!.json();
  assert.match(token, /^skc1_[a-f0-9]{64}$/);
  const redeem = calls.find(c => c.name === 'companion_grant_redeem')!.args;
  assert.ok(!JSON.stringify(calls).includes(token) && !JSON.stringify(calls).includes('a'.repeat(64)));
  assert.equal(redeem.p_name, 'Pixel 9');
  assert.equal((await handleCompanion(request({ ...good, verifier: 'x'.repeat(43) }), ctx))!.status, 400);
  for (const bad of [{ ...good, platform: 'windows' }, { ...good, name: 'a\nb' }, { ...good, extra: 1 }, { ...good, code: 'A'.repeat(64) }]) {
    assert.equal((await handleCompanion(request(bad), ctx))!.status, 400, JSON.stringify(bad));
  }
});

test('phone routes refuse cookies and foreign origins, accept the app WebView', async () => {
  const { ctx } = fake({ companion_boats: () => ({ cloud: { active: true }, devices: [] }) });
  assert.equal((await handleCompanion(phone('/api/companion/boats', { headers: { Cookie: '__Host-skager_session=x' } }), ctx))!.status, 403);
  assert.equal((await handleCompanion(phone('/api/companion/boats', { headers: { Origin: 'https://beta.skager.app' } }), ctx))!.status, 403);
  assert.equal((await handleCompanion(phone('/api/companion/boats', { headers: { Origin: 'capacitor://localhost' } }), ctx))!.status, 200);
  assert.equal((await handleCompanion(phone('/api/companion/boats', { headers: { 'X-Skager-Companion': 'skc1_short' } }), ctx))!.status, 401);
  assert.equal(await handleCompanion(phone('/api/cloud/overview'), ctx), null);
  assert.ok(isCompanionRoute('/api/companion/push') && !isCompanionRoute('/api/companion/../cloud/overview'));
});

test('latest boat data: known keys only, anchor re-validated, ETag answers 304 without a body', async () => {
  const latest = { state: 'ok', device: { id: DEVICE, name: 'Aurora helm', telemetry_enabled: true },
    received_at: new Date(Date.now() - 12000).toISOString(),
    items: { 'navigation.sog': { v: 5.2, age: 1, q: 'live', src: 'GPS' }, 'evil.key': { v: 1, age: 0, q: 'live' } },
    anchor: { armed: true, alarm: false, lat: 59.3, lon: 18.1, radius_m: 50, distance_m: 12.34, age: 2 } };
  const { ctx, calls } = fake({ companion_boat: args => args.p_device_id === DEVICE ? latest : { state: 'unknown' } });
  const first = await handleCompanion(phone(`/api/companion/boat?device=${DEVICE}`), ctx);
  assert.equal(first!.status, 200);
  const data = await first!.json();
  assert.deepEqual(Object.keys(data.items), ['navigation.sog']);
  assert.equal(data.anchor.distance_m, 12.3);
  assert.ok(data.snapshotAge >= 11 && data.snapshotAge < 20);
  assert.equal(calls.find(c => c.name === 'companion_boat')!.args.p_account_id, OWNER);
  const tag = first!.headers.get('ETag')!;
  const again = await handleCompanion(phone(`/api/companion/boat?device=${DEVICE}`, { headers: { 'If-None-Match': tag } }), ctx);
  assert.equal(again!.status, 304);
  assert.equal(await again!.text(), '');
  assert.equal((await handleCompanion(phone('/api/companion/boat?device=nope'), ctx))!.status, 400);
  assert.equal((await handleCompanion(phone(`/api/companion/boat?device=${DEVICE}&device=${DEVICE}`), ctx))!.status, 400);
  assert.equal((await handleCompanion(phone('/api/companion/boat?device=' + OWNER), ctx))!.status, 404);
});

test('history, track and alarm log validate their windows and page by id', async () => {
  const { ctx, calls } = fake({
    cloud_boat_series: () => ({ state: 'ok', bucket_seconds: 360, series: { 'battery.voltage': [[1, 13.1]] } }),
    companion_track: () => ({ state: 'ok', points: [[1, 59.3, 18.1, 0.2]] }),
    companion_events: () => ({ events: [{ id: 7, device_id: DEVICE, boat: 'Aurora helm', kind: 'anchor_alarm', severity: 'critical',
      title: 'Anchor alarm', detail: 'x', data: { distance_m: 61 }, occurred_at: 'a', received_at: 'b', push_state: 'sent' }] }),
  });
  assert.equal((await handleCompanion(phone(`/api/companion/series?device=${DEVICE}&keys=battery.voltage,battery.soc&hours=24`), ctx))!.status, 200);
  assert.deepEqual(calls.at(-1)!.args.p_keys, ['battery.voltage', 'battery.soc']);
  for (const bad of ['keys=evil.key&hours=24', 'keys=battery.voltage&hours=5', 'keys=a,b,c,d,e&hours=24']) {
    assert.equal((await handleCompanion(phone(`/api/companion/series?device=${DEVICE}&${bad}`), ctx))!.status, 400, bad);
  }
  const track = await handleCompanion(phone(`/api/companion/track?device=${DEVICE}&hours=168`), ctx);
  assert.deepEqual((await track!.json()).points, [[1, 59.3, 18.1, 0.2]]);
  assert.equal((await handleCompanion(phone(`/api/companion/track?device=${DEVICE}&hours=720`), ctx))!.status, 400);
  const events = await (await handleCompanion(phone('/api/companion/events?before=99&limit=20'), ctx))!.json();
  assert.equal(events.events[0].title, 'Anchor alarm');
  assert.equal('push_state' in events.events[0], false);
  assert.deepEqual([calls.at(-1)!.args.p_before, calls.at(-1)!.args.p_limit], [99, 20]);
  assert.equal((await handleCompanion(phone('/api/companion/events?limit=500'), ctx))!.status, 400);
});

test('push token registration accepts only well-formed FCM/APNs tokens; sign-out ends the session', async () => {
  const { ctx, calls } = fake({ companion_push_set: () => ({ state: 'ok' }), companion_signout: () => null });
  const post = (path: string, body: unknown) => handleCompanion(phone(path, { method: 'POST', body: JSON.stringify(body) }), ctx);
  assert.equal((await post('/api/companion/push', { service: 'apns', token: 'ab'.repeat(32) }))!.status, 200);
  assert.equal(calls.at(-1)!.args.p_session_id, SESSION);
  assert.equal((await post('/api/companion/push', { service: 'fcm', token: 'fcm-token:APA91b' + 'x'.repeat(40) }))!.status, 200);
  assert.equal((await post('/api/companion/push', { token: null }))!.status, 200);
  assert.equal(calls.at(-1)!.args.p_service, null);
  for (const bad of [{ service: 'apns', token: 'not hex' }, { service: 'web', token: 'x'.repeat(64) }, { service: 'fcm' }]) {
    assert.equal((await post('/api/companion/push', bad))!.status, 400, JSON.stringify(bad));
  }
  assert.equal((await post('/api/companion/signout', {}))!.status, 200);
  assert.equal(calls.at(-1)!.name, 'companion_signout');
});

test('anchor reports are copied from OpenCPN and malformed ones are refused', () => {
  assert.deepEqual(validAnchor({ armed: false }), { armed: false });
  assert.equal(validAnchor({ armed: false, alarm: true }), null);
  assert.deepEqual(validAnchor({ armed: true, alarm: true, lat: 59.3, lon: 18.1, radius_m: -40, distance_m: 30, age: 1.25, src: 'GPS' }),
    { armed: true, alarm: true, lat: 59.3, lon: 18.1, radius_m: -40, distance_m: 30, age: 1.3, src: 'GPS' });
  for (const bad of [{ armed: true }, { armed: true, alarm: false, lat: 59 }, { armed: true, alarm: false, radius_m: 0 },
    { armed: true, alarm: false, distance_m: 5 }, { armed: true, alarm: false, lat: 91, lon: 0 }, { armed: 'yes' }, [], null,
    { armed: true, alarm: false, extra: 1 }]) {
    assert.equal(validAnchor(bad), null, JSON.stringify(bad));
  }
});

test('boat events: known kinds, unique ids, bounded data; alert text only for alert kinds', () => {
  assert.deepEqual(validEvents([{ id: 'evt-00000001', kind: 'anchor_alarm', age: 1.24, data: { distance_m: 61, radius_m: 50 } }]),
    [{ id: 'evt-00000001', kind: 'anchor_alarm', age: 1.2, data: { distance_m: 61, radius_m: 50 } }]);
  assert.equal(validEvents([{ id: 'evt-00000001', kind: 'alert_critical', age: 0, alert: ' GPS position lost ' }])![0].alert, 'GPS position lost');
  for (const bad of [[], [{ id: 'short', kind: 'anchor_alarm', age: 0 }], [{ id: 'evt-00000001', kind: 'connection_lost', age: 0 }],
    [{ id: 'evt-00000001', kind: 'anchor_alarm', age: 0, alert: 'text' }], [{ id: 'evt-00000001', kind: 'anchor_alarm', age: -1 }],
    [{ id: 'evt-00000001', kind: 'anchor_alarm', age: 0, data: { lat: 1 } }],
    [{ id: 'evt-00000001', kind: 'anchor_alarm', age: 0 }, { id: 'evt-00000001', kind: 'anchor_armed', age: 0 }],
    Array.from({ length: 21 }, (_, i) => ({ id: `evt-${String(i).padStart(8, '0')}`, kind: 'anchor_armed', age: 0 }))]) {
    assert.equal(validEvents(bad), null, JSON.stringify(bad).slice(0, 80));
  }
});

test('the boat sends events and anchor state through the device API; new events trigger delivery', async () => {
  let anchor: unknown = 'unset';
  const { ctx, calls } = fake({
    cloud_event_put: () => ({ state: 'stored', new: 1 }),
    cloud_telemetry_put: () => ({ state: 'stored' }),
    cloud_anchor_put: args => { anchor = args.p_anchor; return { state: 'stored', restored: false }; },
  });
  const device = (path: string, body: unknown) => handleCloud(new Request('https://beta.skager.app' + path, { method: 'POST',
    headers: { 'X-Skager-Device': DEVICE_TOKEN, 'Content-Type': 'application/json' }, body: JSON.stringify(body) }), ctx);
  const sent = await device('/api/device/events', { events: [{ id: 'evt-00000001', kind: 'anchor_alarm', age: 1 }] });
  assert.equal(sent!.status, 202);
  assert.ok(calls.some(c => c.name === 'cloud_push_claim'), 'delivery attempted at once');
  assert.equal((await device('/api/device/events', { events: [{ id: 'x', kind: 'anchor_alarm', age: 1 }] }))!.status, 400);
  const items = { 'navigation.sog': { v: 0.1, age: 1, q: 'live', src: 'GPS' } };
  assert.equal((await device('/api/device/telemetry', { items }))!.status, 202);
  assert.equal(anchor, null, 'an old SKAGER without anchor support is unknown, not "off"');
  assert.equal((await device('/api/device/telemetry', { items, anchor: { armed: true, alarm: false, radius_m: 50 } }))!.status, 202);
  assert.deepEqual(anchor, { armed: true, alarm: false, radius_m: 50 });
  assert.equal((await device('/api/device/telemetry', { items, anchor: { armed: true } }))!.status, 400);
});

test('the signed-in page issues a code bound to the PKCE challenge; phones can be signed out', async () => {
  const { ctx, calls } = fake({ companion_grant_create: () => ({ state: 'ok' }), companion_phone_revoke: () => ({ state: 'revoked' }) });
  const post = (path: string, body: unknown) => handleCloud(new Request('https://beta.skager.app' + path, { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }), ctx);
  const result = await post('/api/cloud/companion/authorize', { challenge: 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM' });
  const { code } = await result!.json();
  assert.match(code, /^[a-f0-9]{64}$/);
  assert.ok(!JSON.stringify(calls).includes(code));
  assert.equal((await post('/api/cloud/companion/authorize', { challenge: 'short' }))!.status, 400);
  assert.equal((await post('/api/cloud/phones/revoke', { phoneId: SESSION }))!.status, 200);
  assert.equal(calls.at(-1)!.args.p_account_id, OWNER);
});

// ---------------------------------------------------------------- push
async function pem(algorithm: RsaHashedKeyGenParams | EcKeyGenParams) {
  const pair = await crypto.subtle.generateKey(algorithm, true, ['sign', 'verify']) as CryptoKeyPair;
  const der = new Uint8Array(await crypto.subtle.exportKey('pkcs8', pair.privateKey));
  return { pair, pem: `-----BEGIN PRIVATE KEY-----\n${Buffer.from(der).toString('base64')}\n-----END PRIVATE KEY-----\n` };
}
const decode = (part: string) => JSON.parse(Buffer.from(part, 'base64url').toString());
async function verify(jwt: string, key: CryptoKey, algorithm: AlgorithmIdentifier | EcdsaParams) {
  const [h, c, s] = jwt.split('.');
  return crypto.subtle.verify(algorithm, key, Buffer.from(s, 'base64url'), new TextEncoder().encode(`${h}.${c}`));
}
const MESSAGE = { title: 'Anchor alarm · Aurora helm', body: 'The boat is outside the anchor watch circle.', critical: true,
  data: { kind: 'anchor_alarm', eventId: '7', deviceId: DEVICE } };

test('FCM: signed service-account assertion, high-priority alarm channel, removed tokens reported', async () => {
  resetPushCaches();
  const rsa = await pem({ name: 'RSASSA-PKCS1-v1_5', modulusLength: 2048, publicExponent: new Uint8Array([1, 0, 1]), hash: 'SHA-256' });
  const env = { SKAGER_FCM_SERVICE_ACCOUNT: JSON.stringify({ project_id: 'skager-companion', client_email: 'push@skager-companion.iam.gserviceaccount.com',
    private_key: rsa.pem }) };
  const requests: { url: string; init: RequestInit }[] = [];
  let unregistered = false;
  const transport = (async (url: string, init: RequestInit) => {
    requests.push({ url, init });
    if (url === 'https://oauth2.googleapis.com/token') return Response.json({ access_token: 'ya29.test', expires_in: 3600 });
    return unregistered ? Response.json({ error: { status: 'NOT_FOUND', details: [{ errorCode: 'UNREGISTERED' }] } }, { status: 404 })
      : Response.json({ name: 'projects/x/messages/1' });
  }) as typeof fetch;
  assert.equal(await sendFcm(env, transport, 'fcm-token:' + 'x'.repeat(40), MESSAGE), 'ok');
  const assertion = new URLSearchParams(String(requests[0].init.body)).get('assertion')!;
  assert.ok(await verify(assertion, rsa.pair.publicKey, { name: 'RSASSA-PKCS1-v1_5' }));
  assert.equal(decode(assertion.split('.')[1]).scope, 'https://www.googleapis.com/auth/firebase.messaging');
  assert.equal(requests[1].url, 'https://fcm.googleapis.com/v1/projects/skager-companion/messages:send');
  const sent = JSON.parse(String(requests[1].init.body)).message;
  assert.equal(sent.android.priority, 'HIGH');
  assert.equal(sent.android.notification.channel_id, 'skager-alarms');
  assert.ok(!JSON.stringify(sent).match(/59\.|18\./), 'no position in a notification');
  unregistered = true;
  assert.equal(await sendFcm(env, transport, 'fcm-token:' + 'x'.repeat(40), MESSAGE), 'invalid');
  assert.equal(requests.filter(r => r.url.includes('oauth2')).length, 1, 'access token reused');
  assert.equal(await sendFcm({}, transport, 'fcm-token:' + 'x'.repeat(40), MESSAGE), 'unconfigured');
});

test('APNs: ES256 provider token, time-sensitive alarm, gone tokens reported', async () => {
  resetPushCaches();
  const ec = await pem({ name: 'ECDSA', namedCurve: 'P-256' });
  const env = { SKAGER_APNS_KEY: ec.pem, SKAGER_APNS_KEY_ID: 'ABC123DEFG', SKAGER_APNS_TEAM_ID: 'TEAM123456', SKAGER_APNS_TOPIC: 'app.skager.companion' };
  const requests: { url: string; init: RequestInit }[] = [];
  let status = 200;
  const transport = (async (url: string, init: RequestInit) => {
    requests.push({ url, init });
    return status === 200 ? new Response(null) : Response.json({ reason: 'Unregistered' }, { status });
  }) as typeof fetch;
  const token = 'ab'.repeat(32);
  assert.equal(await sendApns(env, transport, token, MESSAGE), 'ok');
  assert.equal(requests[0].url, `https://api.push.apple.com/3/device/${token}`);
  const headers = requests[0].init.headers as Record<string, string>;
  assert.equal(headers['apns-topic'], 'app.skager.companion');
  assert.equal(headers['apns-priority'], '10');
  const jwt = headers.authorization.replace('bearer ', '');
  assert.ok(await verify(jwt, ec.pair.publicKey, { name: 'ECDSA', hash: 'SHA-256' }));
  assert.deepEqual(decode(jwt.split('.')[0]), { alg: 'ES256', kid: 'ABC123DEFG' });
  assert.equal(JSON.parse(String(requests[0].init.body)).aps['interruption-level'], 'time-sensitive');
  status = 410;
  assert.equal(await sendApns(env, transport, token, MESSAGE), 'invalid');
  assert.equal(await sendApns(env, transport, 'not-a-token', MESSAGE), 'invalid');
  assert.equal(await sendApns({ ...env, SKAGER_APNS_HOST: 'evil.example' }, transport, token, MESSAGE), 'unconfigured');
});

test('dispatch: one result per event, invalid tokens removed, late reports say so', async () => {
  resetPushCaches();
  const done: any[] = [];
  const ec = await pem({ name: 'ECDSA', namedCurve: 'P-256' });
  const now = Date.now();
  const event = { id: 7, device_id: DEVICE, boat: 'Aurora helm', kind: 'anchor_alarm', severity: 'critical', title: 'Anchor alarm',
    detail: 'The boat is outside the anchor watch circle.', occurred_at: new Date(now - 600000).toISOString(),
    received_at: new Date(now).toISOString(),
    targets: [{ session_id: SESSION, service: 'apns', token: 'ab'.repeat(32) }, { session_id: OWNER, service: 'apns', token: 'cd'.repeat(32) }] };
  const ctx = { env: { SKAGER_APNS_KEY: ec.pem, SKAGER_APNS_KEY_ID: 'ABC123DEFG', SKAGER_APNS_TEAM_ID: 'TEAM123456', SKAGER_APNS_TOPIC: 'app.skager.companion' },
    rpc: async (name: string, args: any) => { if (name === 'cloud_push_claim') return [event]; done.push(args); return null; },
    fetch: (async (url: string) => url.includes('ab'.repeat(32)) ? new Response(null) : Response.json({ reason: 'BadDeviceToken' }, { status: 400 })) as typeof fetch };
  assert.deepEqual(await dispatchPush(ctx), { events: 1, sent: 1 });
  assert.deepEqual(done, [{ p_event_id: 7, p_state: 'sent', p_invalid: ['cd'.repeat(32)] }]);
  assert.match(pushMessage(event, now).body, /Reported 10 min after it happened/);
  assert.equal(pushMessage({ ...event, occurred_at: event.received_at }, now).body, event.detail);
  // Without credentials nothing is sent and the event is not retried forever.
  done.length = 0;
  await dispatchPush({ ...ctx, env: {} });
  assert.equal(done[0].p_state, 'unconfigured');
  assert.match(await signJwt({ alg: 'ES256' }, { iss: 'x' }, ec.pem, 'ES256'), /^[\w-]+\.[\w-]+\.[\w-]{86}$/);
});

// ---------------------------------------------------------------- portal page
test('the Companion sign-in page accepts only a well-formed request and returns to the app', () => {
  const challenge = 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM', state = 's'.repeat(24);
  const request = appRequest(`?challenge=${challenge}&state=${state}`);
  assert.deepEqual(request, { challenge, state });
  for (const bad of ['', '?challenge=x&state=' + state, `?challenge=${challenge}`, `?challenge=${challenge}&state=${state}&next=//evil`,
    `?challenge=${challenge}&state=<script>${'a'.repeat(16)}`]) assert.equal(appRequest(bad), null, bad);
  assert.equal(appReturnUrl(request, 'f'.repeat(64)), `app.skager.companion://auth?code=${'f'.repeat(64)}&state=${state}`);
  assert.equal(appReturnUrl(request, null), `app.skager.companion://auth?error=access_denied&state=${state}`);
  assert.match(renderAppAuthorize({ request }), /id="app-allow"/);
  assert.match(renderAppAuthorize({ request }), /cannot control the boat/);
  assert.match(renderAppAuthorize({}), /not valid/);
  assert.match(renderAppAuthorize({ request, done: appReturnUrl(request, 'f'.repeat(64)) }), /href="app\.skager\.companion:\/\/auth\?code=/);
  const phones = renderPhones({ phones: [{ id: SESSION, name: 'Pixel <9>', platform: 'android', appVersion: '1.0.0', push: true,
    lastSeenAt: new Date(Date.now() - 60000).toISOString() }] });
  assert.ok(phones.includes('Pixel &lt;9&gt;') && phones.includes(`data-phone-revoke="${SESSION}"`));
  assert.equal(renderPhones({ phones: [] }), '');
});
