import assert from 'node:assert/strict';
import test from 'node:test';
import { formatReading, pageForPath, renderBoat, renderDashboard, renderDevices, renderLink, sparkline } from './static/portal.js';
import { TELEMETRY_KEYS } from './cloud.ts';

const NOW = Date.parse('2026-10-19T12:00:00Z');
const DEVICE = { id: '22222222-2222-4222-8222-222222222222', name: 'Aurora <helm>', platform: 'windows', appVersion: '0.4.0-beta2.7',
  telemetryEnabled: true, createdAt: '2026-10-19T10:00:00Z', lastSeenAt: '2026-10-19T11:59:00Z', lastDataAt: '2026-10-19T11:59:50Z' };

test('the pairing page has its own route and asks only for the code', () => {
  assert.equal(pageForPath('/link/'), 'link');
  assert.equal(pageForPath('/link'), 'link');
  const form = renderLink();
  assert.match(form, /id="link-form"/);
  assert.match(form, /Settings → System → SKAGER Cloud/);
  const confirm = renderLink({ code: 'BCDF2345', found: { name: '<script>x</script>', platform: 'windows', appVersion: '0.4.0' } });
  assert.match(confirm, /id="link-approve"/);
  assert.match(confirm, /id="link-decline"/);
  assert.ok(!confirm.includes('<script>x'));
  assert.match(renderLink({ done: { name: 'SKAGER on HELM' } }), /SKAGER on HELM is linked/);
  assert.match(renderDashboard({}, 'link', null, undefined, { link: {} }), /link-form/);
});

test('readings keep their unit and never invent a value', () => {
  assert.equal(formatReading('navigation.sog', 5.25, 'kn'), '5.3 kn');
  assert.equal(formatReading('battery.soc', 81.4, '%'), '81 %');
  assert.equal(formatReading('navigation.heading', 271.6, 'deg'), '272°');
  assert.equal(formatReading('navigation.latitude', 59.5, 'deg'), '59° 30.000′ N');
  assert.equal(formatReading('navigation.longitude', -0.25, 'deg'), '0° 15.000′ W');
  assert.equal(formatReading('propulsion.gear', 2, 'code'), 'Reverse');
  assert.equal(formatReading('propulsion.gear', 7, 'code'), '—');
  assert.equal(formatReading('navigation.sog', undefined, 'kn'), '—');
  assert.equal(formatReading('navigation.sog', Number.NaN, 'kn'), '—');
});

test('boat data shows each value with its age; old snapshots are labelled as offline', () => {
  const boat = { device: DEVICE, receivedAt: '2026-10-19T11:59:50Z', items: {
    'navigation.sog': { v: 5.2, age: 2, q: 'live', src: 'GPS <1>' },
    'environment.depth': { v: 12.5, age: 40, q: 'stale' },
    'navigation.latitude': { v: 59.3, age: 1, q: 'live' }, 'navigation.longitude': { v: 18.1, age: 1, q: 'live' } },
    series: { 'navigation.sog': [[1, 4], [2, 5], [3, 6]] } };
  const html = renderBoat(boat, TELEMETRY_KEYS, NOW);
  assert.match(html, /Last update 10 s ago/);
  assert.ok(!html.includes('offline'));
  assert.match(html, /5\.2 kn/);
  assert.match(html, /12 s ago/);  // 10 s since upload + 2 s observation age
  assert.match(html, /Stale/);
  assert.ok(html.includes('GPS &lt;1&gt;') && !html.includes('GPS <1>'));
  assert.match(html, /openstreetmap\.org\/\?mlat=59\.30000&amp;mlon=18\.10000/);
  assert.match(html, /<svg class="spark"/);
  assert.ok(!/style=/.test(html), 'CSP forbids inline style attributes');

  const old = renderBoat({ ...boat, receivedAt: '2026-10-19T09:00:00Z' }, TELEMETRY_KEYS, NOW);
  assert.match(old, /Last update 3 h ago · SKAGER is offline or not sharing right now/);
  const none = renderBoat({ device: { ...DEVICE, telemetryEnabled: false }, receivedAt: null, items: {} }, TELEMETRY_KEYS, NOW);
  assert.match(none, /Boat data sharing is off in SKAGER/);
  // Keys the page does not know are never rendered.
  assert.ok(!renderBoat({ ...boat, items: { 'evil.key': { v: 1, age: 0, q: 'live' } } }, TELEMETRY_KEYS, NOW).includes('evil'));
});

test('linked devices list escapes names and offers unlink and the live view', () => {
  const overview = { cloud: { active: true }, devices: [DEVICE], telemetryKeys: TELEMETRY_KEYS };
  const html = renderDevices(overview, null, NOW);
  assert.ok(html.includes('Aurora &lt;helm&gt;') && !html.includes('Aurora <helm>'));
  assert.match(html, /data-boat="22222222-2222-4222-8222-222222222222"/);
  assert.match(html, /data-unlink="22222222-2222-4222-8222-222222222222"/);
  assert.match(html, /seen 60 s ago/);
  assert.match(renderDevices({ devices: [] }, null, NOW), /href="\/link\/"/);
  assert.equal(renderDevices(undefined), '');
  const cloudPage = renderDashboard({ subscription: { status: 'active' } }, 'cloud', { products: [] }, undefined, { overview });
  assert.match(cloudPage, /Your boats/);
  assert.match(cloudPage, /Automatic updates/);
});

test('sparklines need two finite points', () => {
  assert.match(sparkline([[1, 2]]), /Not enough history/);
  assert.match(sparkline([[1, 2], [2, Number.NaN], [3, 4]]), /<path d="M2\.0,54\.0L278\.0,2\.0"/);
});
