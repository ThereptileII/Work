// Browser preview of the built app (SKAGER_PREVIEW=1 npm run build) against the real
// Worker/router/SQL stack with a clearly synthetic PREVIEW FIXTURE boat. For looking
// at the screens on a computer; never a source of product data.
//   SKAGER_E2E_PSQL="-h … -d scrum94_schema" node tests/e2e/preview.ts [port]
import { createHash } from 'node:crypto';
import { readFileSync, existsSync } from 'node:fs';
import { createServer } from 'node:http';
import { extname, join, normalize } from 'node:path';
import { fileURLToPath } from 'node:url';
import { sql, stack } from './stack.ts';

const DIST = fileURLToPath(new URL('../../dist/', import.meta.url));
const OWNER = '66666666-6666-4666-8666-666666666666';
const PHONE = 'skc1_' + 'f'.repeat(64);
const BOAT = 'skg1_' + '9'.repeat(64);
const sha = (v: string) => createHash('sha256').update(v).digest('hex');
const port = Number(process.argv[2] || 8787);

// The fixture: a boat at anchor off Sandhamn, with a day of synthetic history.
sql(`delete from skager_private.cloud_devices where account_id = '${OWNER}';
  delete from skager_private.companion_sessions where account_id = '${OWNER}';
  delete from skager_private.store_subscriptions where email = 'preview@example.com';
  delete from skager_private.store_customers where email = 'preview@example.com';
  delete from auth.users where id = '${OWNER}';
  insert into auth.users(id,email,email_confirmed_at) values ('${OWNER}','preview@example.com',now());
  insert into skager_private.store_customers(email,account_id,status,password_set) values ('preview@example.com','${OWNER}','active',true);
  insert into skager_private.store_subscriptions(subscription_id,email,status,current_period_end) values ('sub_preview000001','preview@example.com','active',now()+interval '30 days');
  insert into skager_private.companion_sessions(account_id,token_hash,name,platform,app_version) values ('${OWNER}','${sha(PHONE)}','Preview phone','android','0.1.0');
  insert into skager_private.cloud_devices(account_id,token_hash,name,platform,app_version,telemetry_enabled) values ('${OWNER}','${sha(BOAT)}','Aurora','windows','0.4.0-beta2.8',true);`);
const device = sql(`select id from skager_private.cloud_devices where token_hash = '${sha(BOAT)}'`);
const [alat, alon] = [59.2899, 18.9147];
sql(`insert into skager_private.cloud_telemetry_history(device_id, minute, items)
  select '${device}', date_trunc('minute', now()) - make_interval(mins => m), jsonb_build_object(
    'battery.voltage', round((50.85 + m / 1440.0 * 1.1 + 0.05 * sin(m / 9.0))::numeric, 2),
    'battery.soc', round((72.5 + m / 1440.0 * 14)::numeric, 1),
    'battery.net_discharge', case when m between 300 and 420 then round((3.4 + 0.4 * sin(m / 5.0))::numeric, 2) else round((0.18 + 0.03 * sin(m / 7.0))::numeric, 2) end,
    'propulsion.electrical_power', case when m between 300 and 420 then round((3.2 + 0.4 * sin(m / 5.0))::numeric, 2) else 0 end,
    'navigation.sog', case when m between 300 and 420 then round((5.6 + 0.3 * sin(m / 6.0))::numeric, 1) else round((0.3 + 0.2 * abs(sin(m / 4.0)))::numeric, 1) end,
    'wind.true_speed', round((9 + 3 * sin(m / 120.0) + abs(sin(m / 7.0)))::numeric, 1),
    'environment.depth', round((7.6 + 0.4 * sin(m / 30.0))::numeric, 1),
    'environment.water_temperature', round((12.4 + 0.6 * sin(m / 240.0))::numeric, 1))
  from generate_series(1, 1440) m where m not between 900 and 960;
  insert into skager_private.cloud_track(device_id, at, lat, lon, sog)
  select '${device}', now() - make_interval(mins => m), ${alat} + 0.00028 * cos(m / 17.0) + 0.00004 * sin(m / 3.0),
    ${alon} - 0.0004 + 0.00042 * sin(m / 17.0), 0.3 from generate_series(1, 360) m;
  insert into skager_private.cloud_events(device_id, account_id, kind, severity, title, detail, data, occurred_at, push_state) values
   ('${device}','${OWNER}','anchor_armed','info','Anchor watch on','', '{}', now() - interval '6 hours', 'none'),
   ('${device}','${OWNER}','anchor_alarm','critical','Anchor alarm','The boat is outside the anchor watch circle.', '{"distance_m":53.2,"radius_m":50}', now() - interval '2 hours 10 minutes', 'sent'),
   ('${device}','${OWNER}','anchor_alarm_cleared','info','Anchor alarm ended','The boat is back within the anchor watch circle.', '{}', now() - interval '2 hours 7 minutes', 'sent'),
   ('${device}','${OWNER}','connection_lost','critical','No contact with the boat','The anchor watch was on. SKAGER on board keeps watching and sounds its alarm there, but this phone cannot see it until the boat is back online.', '{}', now() - interval '27 hours', 'sent'),
   ('${device}','${OWNER}','connection_restored','info','Contact with the boat restored','SKAGER is sending boat data again.', '{}', now() - interval '26 hours', 'sent');
  update skager_private.cloud_events set received_at = occurred_at where device_id = '${device}';`);

const worker = await stack();
const snapshot = (alarm: boolean) => ({
  items: {
    'navigation.latitude': { v: alat + 0.0002, age: 1.2, q: 'live', src: 'GPS 1' }, 'navigation.longitude': { v: alon, age: 1.2, q: 'live', src: 'GPS 1' },
    'navigation.sog': { v: 0.3, age: 1.2, q: 'live', src: 'GPS 1' }, 'navigation.cog': { v: 212, age: 1.2, q: 'live', src: 'GPS 1' },
    'navigation.heading': { v: 208, age: 0.8, q: 'live', src: 'Compass' },
    'battery.soc': { v: 72.4, age: 3, q: 'live', src: 'BMS' }, 'battery.voltage': { v: 50.84, age: 3, q: 'live', src: 'BMS' },
    'battery.current': { v: 3.6, age: 3, q: 'live', src: 'BMS' }, 'battery.net_discharge': { v: 0.18, age: 3, q: 'live', src: 'BMS' },
    'propulsion.electrical_power': { v: 0, age: 2, q: 'live', src: 'Motor' }, 'propulsion.motor_temperature': { v: 18.5, age: 2, q: 'live', src: 'Motor' },
    'wind.true_speed': { v: 11.2, age: 1, q: 'live', src: 'Wind' }, 'wind.true_angle': { v: 34, age: 1, q: 'live', src: 'Wind' },
    'environment.depth': { v: 7.4, age: 1, q: 'live', src: 'Sounder' }, 'environment.water_temperature': { v: 12.1, age: 95, q: 'aging', src: 'Sounder' },
  },
  anchor: { armed: true, alarm, lat: alat, lon: alon, radius_m: 50, distance_m: alarm ? 53.6 : 31.4, age: 1.2, src: 'GPS 1' },
});
const post = (body: unknown) => worker('/api/device/telemetry', { method: 'POST', body: JSON.stringify(body),
  headers: { authorization: `Bearer ${BOAT}`, 'content-type': 'application/json' } });
console.log('telemetry', (await post(snapshot(process.env.PREVIEW_ALARM === '1'))).status);
setInterval(() => void post(snapshot(process.env.PREVIEW_ALARM === '1')), 15000);

const TYPES: Record<string, string> = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.css': 'text/css',
  '.png': 'image/png', '.woff2': 'font/woff2', '.txt': 'text/plain' };
createServer(async (req, res) => {
  const url = new URL(req.url!, `http://127.0.0.1:${port}`);
  if (url.pathname.startsWith('/api/')) {
    const chunks: Buffer[] = [];
    for await (const chunk of req) chunks.push(chunk as Buffer);
    const headers = new Headers();
    for (const [k, v] of Object.entries(req.headers)) if (typeof v === 'string' && !['origin', 'host', 'cookie', 'referer'].includes(k)) headers.set(k, v);
    const reply = await worker(url.pathname + url.search, { method: req.method, headers, body: chunks.length ? Buffer.concat(chunks) : undefined });
    res.writeHead(reply.status, Object.fromEntries(reply.headers));
    res.end(Buffer.from(await reply.arrayBuffer()));
    return;
  }
  const file = normalize(join(DIST, url.pathname === '/' ? 'index.html' : url.pathname));
  if (!file.startsWith(DIST) || !existsSync(file)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'content-type': TYPES[extname(file)] ?? 'application/octet-stream' });
  res.end(readFileSync(file));
}).listen(port, '127.0.0.1', () => console.log(`preview http://127.0.0.1:${port}/#token=${PHONE}`));
