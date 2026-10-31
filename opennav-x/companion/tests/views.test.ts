import assert from 'node:assert/strict';
import test from 'node:test';
import { TELEMETRY_KEYS } from '../../web/edge/cloud.ts';
import type { Boat, BoatEvent, Device } from '../src/lib/api.ts';
import { DEFAULT_LAYOUT, layoutKeys, renderAlarmBanner, renderAlarmRules, renderAlarms, renderAnchor, renderHistory,
  renderLayoutEditor, renderNoBoats, renderOverview, renderSettings, renderSignIn, renderStatus, renderTabs } from '../src/views.ts';

const DEVICE: Device = { id: '22222222-2222-4222-8222-222222222222', name: 'Aurora <helm>', platform: 'windows', appVersion: '0.4.0',
  telemetryEnabled: true, createdAt: '2026-10-20T10:00:00Z', lastSeenAt: '2026-10-26T10:00:00Z', lastDataAt: '2026-10-26T10:00:00Z' };
const boat = (items: Boat['items'], anchor: Boat['anchor'] = null): Boat =>
  ({ device: DEVICE, receivedAt: '2026-10-26T10:00:00Z', items, anchor, snapshotAge: 5 });

test('overview: values with unit; only old or estimated values are flagged; unknown keys never shown', () => {
  const items: Boat['items'] = {
    'navigation.sog': { v: 5.24, age: 1, q: 'live', src: 'GPS <1>' },
    'battery.soc': { v: 81.4, age: 2, q: 'live' },
    'battery.voltage': { v: 51.234, age: 400, q: 'live' },
    'wind.true_speed': { v: 11.8, age: 1, q: 'estimated' },
    'navigation.latitude': { v: 59.3, age: 1, q: 'live' }, 'navigation.longitude': { v: 18.1, age: 1, q: 'live' },
    'evil.key': { v: 1, age: 0, q: 'live' } as any,
  };
  const html = renderOverview(boat(items), TELEMETRY_KEYS, 5);
  assert.match(html, /class="tile live hero"[\s\S]*?>5\.2<span class="unit">kn<\/span>/, 'the first tile leads');
  assert.ok(!html.includes('GPS'), 'no source line on a current value');
  assert.ok(!/Current ·/.test(html));
  assert.match(html, /class="tile stale[^"]*"[\s\S]*?51\.23[\s\S]*?Old · 7 min ago/);
  assert.match(html, /True wind speed[\s\S]*?Estimated/);
  assert.match(html, /class="fill w16"/, 'SOC bar at 80 %');
  assert.match(html, /59° 18\.000′ N<br>18° 6\.000′ E/);
  assert.ok(!html.includes('evil'));
  assert.ok(!html.includes('Not reported by this boat'), 'no list of what is missing');
  assert.match(html, /id="layout-edit"/);
  // Default order: only what the boat reports.
  assert.deepEqual(layoutKeys(null, items, TELEMETRY_KEYS), ['navigation.sog', 'wind.true_speed', 'battery.soc', 'battery.voltage', 'navigation.latitude']);
  assert.match(renderOverview(null, TELEMETRY_KEYS, null), /Waiting for boat data/);
});

test('dashboard: a saved layout decides tiles and order; the editor moves, hides and adds', () => {
  const items: Boat['items'] = { 'navigation.sog': { v: 5, age: 1, q: 'live' }, 'battery.soc': { v: 80, age: 1, q: 'live' } };
  const layout = ['battery.soc', 'environment.depth', 'nonsense'];
  assert.deepEqual(layoutKeys(layout, items, TELEMETRY_KEYS), ['battery.soc', 'environment.depth']);
  const html = renderOverview(boat(items), TELEMETRY_KEYS, 5, layout);
  assert.ok(html.indexOf('State of charge') < html.indexOf('Depth below transducer'));
  assert.ok(!html.includes('Speed over ground'), 'hidden tiles stay hidden');
  assert.match(html, /Depth below transducer[\s\S]*?—/, 'a chosen value the boat does not send is shown as missing');
  const editor = renderLayoutEditor(['battery.soc', 'environment.depth'], items, TELEMETRY_KEYS);
  assert.match(editor, /data-layout-move="battery.soc:-1"[^>]*disabled/);
  assert.match(editor, /Depth below transducer<span class="note"> · not reported/);
  assert.match(editor, /data-layout-toggle="navigation.sog">Show/);
  assert.match(editor, /id="layout-reset"/);
  assert.ok(DEFAULT_LAYOUT.every(k => k in TELEMETRY_KEYS), 'every default tile is a known key');
});

test('connection status and the anchor alarm banner', () => {
  const ok = renderStatus(boat({}), DEVICE, 12, null);
  assert.match(ok, /class="updated"[\s\S]*Updated 12 s ago/);
  assert.ok(!ok.includes('status-card'), 'all well: one quiet line, no card');
  assert.match(renderStatus(boat({}), DEVICE, 900, { since: Date.parse('2026-10-26T10:00:00Z') }),
    /status-card bad[\s\S]*Offline[\s\S]*No data for 15 min[\s\S]*This phone is offline · saved/);
  assert.match(renderStatus(null, { ...DEVICE, telemetryEnabled: false, lastDataAt: null }, null, null), /Sharing off[\s\S]*Share boat data/);
  assert.ok(!renderStatus(boat({}), DEVICE, 12, null).includes('<helm>'));
  const alarm = renderAlarmBanner(DEVICE, { armed: true, alarm: true, distance_m: 61.5, radius_m: 50 }, 5);
  assert.match(alarm, /role="alert"[\s\S]*Anchor alarm[\s\S]*62 m from the anchor/);
  assert.match(renderAlarmBanner(DEVICE, { armed: true, alarm: true }, 600), /Last report: anchor alarm/);
  assert.equal(renderAlarmBanner(DEVICE, { armed: true, alarm: false }, 5), '');
  assert.match(renderTabs('anchor', true, 3), /aria-label="Anchor alarm"[\s\S]*badge-count">3/);
});

test('anchor screen: watching, alarm, no contact, off and unknown are distinct', () => {
  const watching = renderAnchor(DEVICE, { armed: true, alarm: false, distance_m: 38, radius_m: 50, age: 2, lat: 59.3, lon: 18.1 }, 5, true);
  assert.match(watching, /class="anchor-hero watching"[\s\S]*Watching[\s\S]*38 m[\s\S]*of 50 m radius/);
  assert.match(watching, /data-map-slot="anchor"/);
  assert.match(watching, /decided by SKAGER on board/);
  assert.match(renderAnchor(DEVICE, { armed: true, alarm: true, distance_m: 61, radius_m: 50, age: 1 }, 5, true), /role="alert"[\s\S]*Anchor alarm[\s\S]*fill w20 over/);
  assert.match(renderAnchor(DEVICE, { armed: true, alarm: false, radius_m: 50 }, 900, false), /No contact[\s\S]*keeps watching and sounds its alarm there/);
  assert.match(renderAnchor(DEVICE, { armed: true, alarm: false, radius_m: -40 }, 5, false), /keep-out radius[\s\S]*Distance unavailable/);
  assert.match(renderAnchor(DEVICE, { armed: false }, 5, false), /anchor watch is off[\s\S]*cannot start or stop it/);
  assert.match(renderAnchor(DEVICE, null, 5, false), /No anchor watch report yet/);
});

test('history: only reported metrics, honest energy estimate, empty period stated', () => {
  const now = Math.floor(Date.now() / 1000);
  const series = { bucketSeconds: 3600, series: { 'battery.net_discharge': [[now - 7200, 4], [now - 3600, 2]] as [number, number][] } };
  const html = renderHistory('battery.net_discharge', 24, series, false, new Set(['battery.net_discharge', 'battery.voltage']), TELEMETRY_KEYS);
  assert.match(html, /<h2 class="history-title">[\s\S]*<select id="metric-select" class="title-select">/, 'the heading is the chooser');
  assert.match(html, /<option value="battery.net_discharge" selected>/);
  assert.match(html, /<option value="battery.voltage">/);
  assert.ok(!html.includes('value="wind.true_speed"'));
  assert.match(html, /Energy used <strong>6\.0 kWh<\/strong>[\s\S]*1-hour averages over 2\.0 h with data/);
  assert.match(html, /<path d="M/);
  assert.match(renderHistory('battery.voltage', 6, { bucketSeconds: 60, series: { 'battery.voltage': [] } }, false, new Set(['battery.voltage']), TELEMETRY_KEYS), /No data in this period/);
  assert.match(renderHistory('x', 6, null, false, new Set(), TELEMETRY_KEYS), /No history yet/);
});

test('alarm log: newest first by day, late reports marked, text escaped', () => {
  const events: BoatEvent[] = [
    { id: 9, deviceId: DEVICE.id, boat: 'Aurora <helm>', kind: 'anchor_alarm', severity: 'critical', title: 'Anchor alarm',
      detail: 'The boat is outside the anchor watch circle.', data: { distance_m: 61.5, radius_m: 50 },
      occurredAt: new Date(Date.now() - 600_000).toISOString(), receivedAt: new Date().toISOString() },
    { id: 8, deviceId: DEVICE.id, boat: 'Aurora <helm>', kind: 'alert_critical', severity: 'critical', title: '<b>GPS</b> position lost',
      detail: '', data: {}, occurredAt: new Date(Date.now() - 86_400_000 * 3).toISOString(), receivedAt: new Date(Date.now() - 86_400_000 * 3).toISOString() },
  ];
  const html = renderAlarms(events, true, false, 9, true);
  assert.match(html, /Today[\s\S]*Anchor alarm[\s\S]*62 m from the anchor · radius 50 m[\s\S]*reported/);
  assert.match(html, /class="event critical highlight"/);
  assert.ok(html.includes('&lt;b&gt;GPS&lt;/b&gt;') && !html.includes('<b>GPS'));
  assert.match(html, /Aurora &lt;helm&gt;/);
  assert.match(html, /id="events-more"/);
  assert.match(renderAlarms([], false, false, null, false), /Nothing has happened yet/);
});

test('sign-in and settings say what Companion can and cannot do', () => {
  assert.match(renderSignIn({}), /id="sign-in"[\s\S]*not for navigation and cannot control the boat/);
  assert.match(renderSignIn({ message: 'Sign-in was cancelled.' }), /role="status">Sign-in was cancelled/);
  const settings = renderSettings({ email: 'a@b.c', cloud: { active: true, status: 'active', renewsAt: null, cancelAtPeriodEnd: false },
    push: 'unconfigured', theme: 'night', version: '0.1.0', boats: [DEVICE] });
  assert.match(settings, /Not available in this test build/);
  assert.match(settings, /class="active" data-theme-choice="night"/);
  assert.match(settings, /id="sign-out"/);
  assert.ok(settings.includes('Aurora &lt;helm&gt;'));
});

test('no boat yet: the real steps, including the QR code', () => {
  const html = renderNoBoats();
  assert.match(html, /Settings → System → SKAGER Cloud[\s\S]*Link to my account[\s\S]*QR code[\s\S]*Share boat data/);
});

test('your alarms: list with state, and a form that keeps what was typed', () => {
  const keys = { 'battery.soc': { label: 'State of charge', unit: '%', min: 0, max: 100 },
    'wind.true_speed': { label: 'True wind speed', unit: 'kn', min: 0, max: 200 } };
  const alarms = [
    { id: 1, deviceId: DEVICE.id, key: 'battery.soc', label: 'State of charge', unit: '%', direction: 'below' as const, threshold: 30,
      enabled: true, state: 'alarm' as const, value: 24, changedAt: null },
    { id: 2, deviceId: DEVICE.id, key: 'wind.true_speed', label: 'True wind <b>', unit: 'kn', direction: 'above' as const, threshold: 25,
      enabled: false, state: 'unknown' as const, value: null, changedAt: null },
  ];
  const list = renderAlarmRules({ alarms, keys, form: null, saving: false });
  assert.match(list, /class="rule alarm" data-alarm-edit="1"[\s\S]*Below 30 %[\s\S]*>Alarm</);
  assert.match(list, /class="rule off"[\s\S]*Above 25 kn[\s\S]*>Off</);
  assert.ok(list.includes('True wind &lt;b&gt;'));
  assert.match(list, /id="alarm-add"/);
  const form = renderAlarmRules({ alarms, keys, form: { id: 1, key: 'battery.soc', direction: 'below', threshold: '3"0', enabled: true,
    error: 'Enter a value from 0 to 100 %.' }, saving: false });
  assert.match(form, /id="alarm-form"[\s\S]*<option value="battery.soc" selected>/);
  assert.match(form, /value="3&quot;0"/);
  assert.match(form, /class="active" data-alarm-direction="below"/);
  assert.match(form, /role="alert">Enter a value/);
  assert.match(form, /id="alarm-delete"/);
  assert.ok(!renderAlarmRules({ alarms, keys, form: { key: 'battery.soc', direction: 'above', threshold: '', enabled: true }, saving: false })
    .includes('alarm-delete'), 'a new alarm has nothing to delete');
  assert.match(renderAlarmRules({ alarms: [], keys, form: null, saving: false }), /above or below a limit you choose/);
});
