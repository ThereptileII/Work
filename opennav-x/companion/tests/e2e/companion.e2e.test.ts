// End to end through the real stack (see stack.ts) with the app's own API client:
// sign-in with PKCE through the signed-in web page, boat data and anchor watch from
// SKAGER on board, ETag, events and push, "no contact", the alarm log, and signing
// the phone out from the portal. Runs only when SKAGER_E2E_PSQL names a disposable
// database with web/edge/*.sql applied.
import assert from 'node:assert/strict';
import test from 'node:test';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { Api, SignedOutError } from '../../src/lib/api.ts';
import { authorizeUrl, newPending } from '../../src/lib/pkce.ts';
import { psqlArgs, pushes, sql, stack } from './stack.ts';

const OWNER = '77777777-7777-4777-8777-777777777777';
const SESSION = 'b2'.repeat(32), CSRF = 'd4'.repeat(32);
const DEVICE_TOKEN = 'skg1_' + 'e'.repeat(64);
const sha = (value: string) => createHash('sha256').update(value).digest('hex');

/** What SKAGER on board sends: the C++ encoder when built (SKAGER_E2E_CLOUD_TESTS), else the same shape. */
function boatPayload(mode: 'emit' | 'emit-events'): any {
  const binary = process.env.SKAGER_E2E_CLOUD_TESTS;
  if (binary) return JSON.parse(execFileSync(binary, [mode], { encoding: 'utf8' }));
  return mode === 'emit'
    ? { items: { 'navigation.latitude': { v: 59.3, age: 1, q: 'live', src: 'GPS' }, 'navigation.longitude': { v: 18.1, age: 1, q: 'live', src: 'GPS' },
        'navigation.sog': { v: 0.3, age: 1, q: 'live', src: 'GPS' }, 'battery.voltage': { v: 51.2, age: 2, q: 'live', src: 'BMS' } },
      anchor: { armed: true, alarm: true, lat: 59.3, lon: 18.1, radius_m: 50, distance_m: 61.5, age: 1, src: 'GPS' } }
    : { events: [{ id: crypto.randomUUID(), kind: 'anchor_alarm', age: 2, data: { distance_m: 61.5, radius_m: 50 } },
        { id: crypto.randomUUID(), kind: 'alert_critical', age: 1, alert: 'GPS position lost' }] };
}

test('SKAGER Companion through the real Worker, router and SQL', { skip: !psqlArgs() && 'SKAGER_E2E_PSQL not set' }, async () => {
  const rsa = await crypto.subtle.generateKey({ name: 'RSASSA-PKCS1-v1_5', modulusLength: 2048, publicExponent: new Uint8Array([1, 0, 1]),
    hash: 'SHA-256' }, true, ['sign']) as CryptoKeyPair;
  const pem = `-----BEGIN PRIVATE KEY-----\n${Buffer.from(await crypto.subtle.exportKey('pkcs8', rsa.privateKey)).toString('base64')}\n-----END PRIVATE KEY-----\n`;
  const worker = await stack({ SKAGER_JOB_SECRET: 'j'.repeat(40),
    SKAGER_FCM_SERVICE_ACCOUNT: JSON.stringify({ project_id: 'skager-local', client_email: 'push@skager-local.iam.gserviceaccount.com', private_key: pem }) });
  sql(`delete from skager_private.cloud_devices where account_id = '${OWNER}';
       delete from skager_private.store_subscriptions where email = 'companion@example.com';
       delete from skager_private.store_customers where email = 'companion@example.com';
       delete from auth.users where id = '${OWNER}';
       insert into auth.users(id,email,email_confirmed_at) values ('${OWNER}','companion@example.com',now());
       insert into skager_private.store_customers(email,account_id,status,password_set) values ('companion@example.com','${OWNER}','active',true);
       insert into skager_private.store_subscriptions(subscription_id,email,status,current_period_end) values ('sub_companione2e01','companion@example.com','active',now()+interval '30 days');
       insert into skager_private.auth_sessions(token_hash,subject,expires_at) values ('${sha(SESSION)}','${OWNER}',now()+interval '1 hour');
       insert into skager_private.cloud_devices(account_id,token_hash,name,platform,app_version) values ('${OWNER}','${sha(DEVICE_TOKEN)}','Aurora helm','windows','0.4.0-beta2.7');`);
  const cookie = `__Host-skager_session=${SESSION}; __Host-skager_csrf=${CSRF}`;
  const browser = (path: string, body?: unknown) => worker(path, body === undefined ? { headers: { cookie } }
    : { method: 'POST', body: JSON.stringify(body), headers: { 'content-type': 'application/json', origin: 'https://beta.skager.app', 'x-csrf-token': CSRF, cookie } });
  const boat = (path: string, body: unknown) => worker(path, { method: 'POST', body: JSON.stringify(body),
    headers: { authorization: `Bearer ${DEVICE_TOKEN}`, 'content-type': 'application/json', 'x-skager-version': '0.4.0-beta2.8' } });
  let token: string | null = null;
  const phone = new Api({ token: async () => token }, ((url: string, init: RequestInit) =>
    worker(url.replace('https://beta.skager.app', ''), init)) as typeof fetch);

  // 1. Sign-in: the app opens the page; the signed-in owner approves; the app redeems with its verifier.
  const pending = newPending();
  const page = new URL(await authorizeUrl(pending));
  assert.equal((await worker(page.pathname + page.search)).status, 200);
  const challenge = page.searchParams.get('challenge')!;
  const granted = await browser('/api/cloud/companion/authorize', { challenge });
  assert.equal(granted.status, 200);
  const { code } = await granted.json();
  await assert.rejects(phone.exchange(code, newPending().verifier, 'Pixel 9', 'android'), /expired or was already used/);
  const second = await (await browser('/api/cloud/companion/authorize', { challenge })).json();
  token = await phone.exchange(second.code, pending.verifier, 'Pixel 9', 'android');
  assert.match(token, /^skc1_[a-f0-9]{64}$/);
  assert.equal(sql(`select count(*) from skager_private.companion_sessions where token_hash = '${sha(token)}'`), '1');
  assert.equal((await phone.session()).account.email, 'companion@example.com');

  // 2. The boat shares data with its anchor watch; the phone reads it, then gets 304 while unchanged.
  const shared = await boat('/api/device/telemetry', boatPayload('emit'));
  assert.equal(shared.status, 202, await shared.text());
  const boats = await phone.boats();
  assert.equal(boats.devices.length, 1);
  assert.equal(boats.devices[0].anchor?.armed, true);
  const device = boats.devices[0].id;
  const first = await phone.boat(device);
  assert.ok(first.boat && Object.keys(first.boat.items).length >= 4);
  assert.equal(first.boat!.anchor?.radius_m, 50);
  assert.ok(first.boat!.snapshotAge! >= 0 && first.boat!.snapshotAge! < 30);
  assert.deepEqual(await phone.boat(device, first.etag!), { boat: null, etag: first.etag });
  assert.equal((await phone.track(device, 24)).length, 1);
  assert.ok((await phone.series(device, ['navigation.sog'], 24)).series['navigation.sog'].length >= 1);

  // 3. Alarms: with a registered phone, the boat's events become notifications without positions.
  await phone.registerPush('fcm', 'fcm-local-token:' + 'x'.repeat(40));
  pushes.length = 0;
  const events = boatPayload('emit-events');
  assert.equal((await boat('/api/device/events', events)).status, 202);
  assert.equal((await boat('/api/device/events', events)).status, 202, 'a retried batch is accepted');
  assert.equal(pushes.length, 2, 'each event notified once');
  const alarm = pushes.find(p => p.body.message.data.kind === 'anchor_alarm')!.body.message;
  assert.equal(alarm.notification.title, 'Anchor alarm · Aurora helm');
  assert.equal(alarm.android.notification.channel_id, 'skager-alarms');
  assert.doesNotMatch(JSON.stringify(pushes), /59\.3|18\.1/);
  const log = await phone.events();
  assert.deepEqual(log.slice(0, 2).map(e => e.kind).sort(), ['alert_critical', 'anchor_alarm']);
  assert.equal(log.find(e => e.kind === 'alert_critical')!.title, 'GPS position lost');

  // 4. The boat goes quiet with the watch on: the scheduled job raises "no contact" once and notifies.
  sql(`update skager_private.cloud_telemetry_latest set received_at = now() - interval '4 minutes' where device_id = '${device}'`);
  pushes.length = 0;
  const tick = () => worker.direct('/api/jobs/tick', { method: 'POST', headers: { 'x-skager-job': 'j'.repeat(40) } });
  const job = await (await tick()).json();
  assert.equal(job.companion.contact_lost, 1);
  assert.equal(job.push.sent, 1);
  await tick();
  assert.equal(pushes.filter(p => p.body.message.data.kind === 'connection_lost').length, 1);
  assert.equal((await phone.events({ limit: 1 }))[0].kind, 'connection_lost');
  assert.equal((await boat('/api/device/telemetry', boatPayload('emit'))).status, 202);
  assert.equal((await phone.events({ limit: 1 }))[0].kind, 'connection_restored');

  // 5. A browser session can never use the phone API; the portal lists and signs out the phone.
  assert.equal((await worker('/api/companion/boats', { headers: { cookie, authorization: `Bearer ${token}` } })).status, 403);
  const overview = await (await browser('/api/cloud/overview')).json();
  assert.equal(overview.phones.length, 1);
  assert.equal(overview.phones[0].push, true);
  assert.equal((await browser('/api/cloud/phones/revoke', { phoneId: overview.phones[0].id })).status, 200);
  await assert.rejects(phone.boats(), SignedOutError);

  // 6. Unlinking the boat computer removes its track and events.
  assert.equal((await browser('/api/cloud/devices/revoke', { deviceId: device })).status, 200);
  assert.equal(sql(`select count(*) from skager_private.cloud_events where device_id = '${device}'`), '0');
  assert.equal(sql(`select count(*) from skager_private.cloud_track where device_id = '${device}'`), '0');
});
