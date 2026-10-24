import assert from 'node:assert/strict';
import test from 'node:test';
import { handleCloud, isDeviceRoute, normalizeUserCode, userCode, validTelemetry, type CloudContext } from './cloud.ts';

const OWNER = '11111111-1111-4111-8111-111111111111';
const DEVICE = '22222222-2222-4222-8222-222222222222';
const encoder = new TextEncoder();
const sha256 = async (value: string) => [...new Uint8Array(await crypto.subtle.digest('SHA-256', encoder.encode(value)))]
  .map(b => b.toString(16).padStart(2, '0')).join('');

/** In-memory stand-in for cloud.sql: enough state to follow one pairing end to end. */
function backend(options: { cloudActive?: boolean } = {}) {
  const links = new Map<string, { code: string; name: string; status: string; account?: string }>();
  const devices = new Map<string, { id: string; account: string }>();
  const calls: { name: string; args: Record<string, unknown> }[] = [];
  let throttleCount = 0;
  const state = { links, devices, calls, cloudActive: options.cloudActive ?? true, telemetry: null as unknown,
    throttleLimit: Infinity, telemetryResult: 'stored' };
  const rpc = async (name: string, args: Record<string, any>) => {
    calls.push({ name, args });
    switch (name) {
      case 'skager_auth_throttle': return ++throttleCount <= state.throttleLimit;
      case 'cloud_link_start':
        links.set(args.p_device_hash, { code: args.p_user_code, name: args.p_name, status: 'pending' });
        return { state: 'pending' };
      case 'cloud_link_find': {
        const link = [...links.values()].find(l => l.code === args.p_user_code && l.status === 'pending');
        return link ? { state: 'pending', name: link.name, platform: 'windows', app_version: '0.4.0' } : { state: 'unknown' };
      }
      case 'cloud_link_decide': {
        const link = [...links.values()].find(l => l.code === args.p_user_code && l.status === 'pending');
        if (!link) return { state: 'unknown' };
        link.status = args.p_approve ? 'approved' : 'denied'; link.account = args.p_account_id;
        return { state: link.status, name: link.name };
      }
      case 'cloud_link_claim': {
        const link = links.get(args.p_device_hash);
        if (!link) return { state: 'expired' };
        if (link.status === 'pending') return { state: 'pending' };
        links.delete(args.p_device_hash);
        if (link.status === 'denied') return { state: 'denied' };
        devices.set(args.p_token_hash, { id: DEVICE, account: link.account! });
        return { state: 'approved', device_id: DEVICE };
      }
      case 'cloud_device_auth': {
        const device = devices.get(args.p_token_hash);
        return device ? { state: 'active', device_id: device.id, account_id: device.account, email: 'skipper@example.test',
          name: 'Aurora helm', cloud: { active: state.cloudActive, status: state.cloudActive ? 'active' : null } } : { state: 'unknown' };
      }
      case 'cloud_telemetry_put': if (state.telemetryResult === 'stored') state.telemetry = args.p_items;
        return { state: state.telemetryResult };
      case 'cloud_device_forget': for (const [k, d] of devices) if (d.id === args.p_device_id) devices.delete(k); return null;
      case 'cloud_overview': return { cloud: { active: state.cloudActive, status: 'active' },
        devices: [{ id: DEVICE, name: 'Aurora helm', platform: 'windows', app_version: '0.4.0', token_hash: 'never returned' }] };
      case 'cloud_boat_latest': return args.p_account_id === OWNER ? { state: 'ok', device: { id: DEVICE, name: 'Aurora helm' },
        received_at: '2026-10-19T10:00:00Z', items: { 'navigation.sog': { v: 5.2, age: 1, q: 'live' }, 'evil.key': { v: 1, age: 0, q: 'live' } } }
        : { state: 'unknown' };
      case 'cloud_boat_series': return { state: 'ok', bucket_seconds: 360, series: { 'navigation.sog': [[1, 5.2]] } };
      default: throw new Error('Unexpected RPC ' + name);
    }
  };
  const ctx: CloudContext = { env: {}, rpc, getAccount: async () => ({ id: OWNER, email: 'skipper@example.test', passwordSet: true }) };
  return { ctx, state };
}

function app(path: string, init: { method?: string; body?: unknown; token?: string; headers?: Record<string, string> } = {}) {
  const headers: Record<string, string> = { ...(init.headers ?? {}) };
  if (init.body !== undefined) headers['Content-Type'] = 'application/json';
  if (init.token) headers['X-Skager-Device'] = init.token;
  return new Request('https://beta.skager.app' + path, { method: init.method ?? (init.body === undefined ? 'GET' : 'POST'),
    headers, body: init.body === undefined ? undefined : JSON.stringify(init.body) });
}
async function call(ctx: CloudContext, request: Request) {
  const response = await handleCloud(request, ctx);
  assert.ok(response);
  const text = await response.text();
  return { status: response.status, data: text ? JSON.parse(text) : null, headers: response.headers };
}

test('user codes use the unambiguous alphabet and accept what people type', () => {
  for (let i = 0; i < 200; i++) assert.match(userCode(), /^[BCDFGHJKLMNPQRSTVWXZ2-9]{8}$/);
  assert.equal(normalizeUserCode(' bcdf-2345 '), 'BCDF2345');
  for (const bad of ['BCDF234', 'BCDF23450', 'ABCD2345', 'BCDF2341', 7, null, 'x'.repeat(30)]) assert.equal(normalizeUserCode(bad), null);
});

test('pairing: the app starts, polls, the logged-in customer approves and the app receives its token once', async () => {
  const { ctx, state } = backend();
  const started = await call(ctx, app('/api/device/link', { body: { name: 'Aurora helm', platform: 'windows', version: '0.4.0-beta2.6' } }));
  assert.equal(started.status, 200);
  assert.match(started.data.deviceCode, /^[a-f0-9]{64}$/);
  assert.equal(started.data.verificationUrl, 'https://beta.skager.app/link/');
  assert.equal(started.data.interval, 5);
  // Only the hash of the secret device code reaches the database.
  const stored = state.calls.find(c => c.name === 'cloud_link_start')!.args;
  assert.equal(stored.p_device_hash, await sha256(started.data.deviceCode));
  assert.ok(!JSON.stringify(state.calls).includes(started.data.deviceCode));

  assert.deepEqual((await call(ctx, app('/api/device/link/poll', { body: { deviceCode: started.data.deviceCode } }))).data, { state: 'pending' });
  const found = await call(ctx, app('/api/cloud/link/find', { body: { code: started.data.userCode.toLowerCase() } }));
  assert.equal(found.data.name, 'Aurora helm');
  const decided = await call(ctx, app('/api/cloud/link/decide', { body: { code: started.data.userCode, approve: true } }));
  assert.equal(decided.data.state, 'approved');

  const approved = await call(ctx, app('/api/device/link/poll', { body: { deviceCode: started.data.deviceCode } }));
  assert.equal(approved.data.state, 'approved');
  assert.match(approved.data.token, /^skg1_[a-f0-9]{64}$/);
  assert.ok(state.devices.has(await sha256(approved.data.token)));
  assert.ok(!JSON.stringify(state.calls).includes(approved.data.token));
  assert.deepEqual((await call(ctx, app('/api/device/link/poll', { body: { deviceCode: started.data.deviceCode } }))).data, { state: 'expired' });

  const status = await call(ctx, app('/api/device/status', { token: approved.data.token, headers: { 'X-Skager-Version': '0.4.0-beta2.7' } }));
  assert.equal(status.status, 200);
  assert.equal(status.data.account.email, 'skipper@example.test');
  assert.deepEqual(status.data.services, { updates: true, telemetry: true });
  assert.equal(state.calls.at(-1)!.args.p_version, '0.4.0-beta2.7');
});

test('device routes refuse browsers, cookies, bad bodies and unknown tokens', async () => {
  const { ctx, state } = backend();
  for (const headers of [{ Origin: 'https://beta.skager.app' }, { Origin: 'https://evil.example' }, { Cookie: '__Host-skager_session=x' }]) {
    const response = await call(ctx, app('/api/device/link', { body: { name: 'x', platform: 'windows', version: '1' }, headers }));
    assert.equal(response.status, 403);
  }
  for (const body of [{ name: '', platform: 'windows', version: '1' }, { name: 'a\nb', platform: 'windows', version: '1' },
    { name: 'x', platform: 'mac', version: '1' }, { name: 'x', platform: 'windows', version: '1', extra: 1 }]) {
    assert.equal((await call(ctx, app('/api/device/link', { body }))).status, 400);
  }
  assert.equal((await call(ctx, app('/api/device/link/poll', { body: { deviceCode: 'nothex' } }))).status, 400);
  for (const token of [undefined, 'skg1_' + 'z'.repeat(64), 'skg1_' + 'a'.repeat(64)]) {
    const response = await call(ctx, app('/api/device/status', { token }));
    assert.equal(response.status, 401);
    assert.equal(response.data.linked, false);
  }
  assert.ok(!state.calls.some(c => c.name === 'cloud_link_start'));
  assert.ok(isDeviceRoute('/api/device/telemetry') && !isDeviceRoute('/api/cloud/overview'));
});

test('a link start flood is throttled before any pairing is created', async () => {
  const { ctx, state } = backend();
  state.throttleLimit = 0;
  const response = await call(ctx, app('/api/device/link', { body: { name: 'x', platform: 'linux', version: '1' } }));
  assert.equal(response.status, 429);
  assert.ok(!state.calls.some(c => c.name === 'cloud_link_start'));
});

async function linked(cloudActive = true) {
  const b = backend({ cloudActive });
  const token = 'skg1_' + 'a'.repeat(64);
  b.state.devices.set(await sha256(token), { id: DEVICE, account: OWNER });
  return { ...b, token };
}

test('update access and telemetry need an active SKAGER Cloud subscription', async () => {
  const off = await linked(false);
  assert.equal((await call(off.ctx, app('/api/device/update-access', { token: off.token }))).status, 402);
  const refused = await call(off.ctx, app('/api/device/telemetry', { token: off.token,
    body: { items: { 'navigation.sog': { v: 5, age: 0, q: 'live' } } } }));
  assert.equal(refused.status, 402);
  assert.equal(off.state.telemetry, null);
  const status = await call(off.ctx, app('/api/device/status', { token: off.token }));
  assert.deepEqual(status.data.services, { updates: false, telemetry: false });

  const on = await linked(true);
  assert.equal((await call(on.ctx, app('/api/device/update-access', { token: on.token }))).status, 204);
  const stored = await call(on.ctx, app('/api/device/telemetry', { token: on.token,
    body: { items: { 'navigation.sog': { v: 5.24, age: 0.42, q: 'live', src: 'GPS 1' }, 'battery.soc': { v: 81, age: 3, q: 'aging' } } } }));
  assert.equal(stored.status, 202);
  assert.deepEqual(on.state.telemetry, { 'navigation.sog': { v: 5.24, age: 0.4, q: 'live', src: 'GPS 1' }, 'battery.soc': { v: 81, age: 3, q: 'aging' } });
});

test('telemetry validation refuses unknown keys, impossible values and extra fields as a whole', () => {
  const ok = { v: 1, age: 0, q: 'live' };
  assert.ok(validTelemetry({ 'navigation.sog': ok }));
  for (const items of [{}, [], null, { 'navigation.unknown': ok }, { constructor: ok }, { toString: ok },
    JSON.parse('{"__proto__":{"v":1,"age":0,"q":"live"}}'), { 'navigation.sog': { ...ok, v: Number.NaN } },
    { 'navigation.latitude': { ...ok, v: 91 } }, { 'navigation.sog': { ...ok, age: -1 } }, { 'navigation.sog': { ...ok, q: 'fresh' } },
    { 'navigation.sog': { ...ok, extra: true } }, { 'navigation.sog': { ...ok, src: 'a\u0000b' } }, { 'navigation.sog': { ...ok, v: '5' } }]) {
    assert.equal(validTelemetry(items), null, JSON.stringify(items));
  }
});

test('write-time revocation and cancellation are propagated even after successful request authentication', async () => {
  for (const [state, expected] of [['revoked', 401], ['inactive', 402], ['unexpected', 503]] as const) {
    const on = await linked(true);
    on.state.telemetryResult = state;
    const response = await call(on.ctx, app('/api/device/telemetry', { token: on.token,
      body: { items: { 'navigation.sog': { v: 5, age: 0, q: 'live', src: 'GPS' } } } }));
    assert.equal(response.status, expected);
    assert.equal(on.state.telemetry, null);
  }
});

test('unlink revokes the device; afterwards the token no longer resolves', async () => {
  const { ctx, token } = await linked();
  assert.equal((await call(ctx, app('/api/device/unlink', { token, body: {} }))).data.state, 'unlinked');
  assert.equal((await call(ctx, app('/api/device/status', { token }))).status, 401);
});

test('customer routes come from the session and never return secrets or unknown keys', async () => {
  const { ctx } = backend();
  const overview = await call(ctx, app('/api/cloud/overview'));
  assert.equal(overview.data.devices[0].id, DEVICE);
  assert.ok(!JSON.stringify(overview.data).includes('never returned'));
  assert.ok('navigation.sog' in overview.data.telemetryKeys);
  const boat = await call(ctx, app('/api/cloud/boat', { body: { deviceId: DEVICE } }));
  assert.deepEqual(Object.keys(boat.data.items), []);  // one unknown key spoils the stored snapshot as a whole
  const series = await call(ctx, app('/api/cloud/boat/series', { body: { deviceId: DEVICE, keys: ['navigation.sog'], hours: 24 } }));
  assert.deepEqual(series.data.series['navigation.sog'], [[1, 5.2]]);
  for (const body of [{ deviceId: DEVICE, keys: ['nope'], hours: 24 }, { deviceId: DEVICE, keys: ['constructor'], hours: 24 },
    { deviceId: DEVICE, keys: ['__proto__'], hours: 24 }, { deviceId: DEVICE, keys: ['navigation.sog'], hours: 5 },
    { deviceId: 'x', keys: ['navigation.sog'], hours: 24 }]) {
    assert.equal((await call(ctx, app('/api/cloud/boat/series', { body }))).status, 400);
  }
  const anonymous = { ...ctx, getAccount: async () => null, rpc: async () => assert.fail('no RPC without a session') };
  assert.equal((await call(anonymous, app('/api/cloud/overview'))).status, 401);
  assert.equal(await handleCloud(app('/api/unrelated'), ctx), null);
});
