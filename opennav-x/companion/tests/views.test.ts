import assert from 'node:assert/strict';
import test from 'node:test';
import { TELEMETRY_KEYS } from '../../web/edge/cloud.ts';
import type { Boat, BoatEvent, Device } from '../src/lib/api.ts';
import { renderAlarmBanner, renderAlarms, renderAnchor, renderHistory, renderOverview, renderSettings, renderSignIn,
  renderStatus, renderTabs } from '../src/views.ts';

const DEVICE: Device = { id: '22222222-2222-4222-8222-222222222222', name: 'Aurora <helm>', platform: 'windows', appVersion: '0.4.0',
  telemetryEnabled: true, createdAt: '2026-10-20T10:00:00Z', lastSeenAt: '2026-10-26T10:00:00Z', lastDataAt: '2026-10-26T10:00:00Z' };
const boat = (items: Boat['items'], anchor: Boat['anchor'] = null): Boat =>
  ({ device: DEVICE, receivedAt: '2026-10-26T10:00:00Z', items, anchor, snapshotAge: 5 });

test('overview: values with unit and age; stale is marked; unknown keys are never shown; missing is explicit', () => {
  const html = renderOverview(boat({
    'navigation.sog': { v: 5.24, age: 1, q: 'live', src: 'GPS <1>' },
    'battery.soc': { v: 81.4, age: 2, q: 'live' },
    'battery.voltage': { v: 51.234, age: 400, q: 'live' },
    'navigation.latitude': { v: 59.3, age: 1, q: 'live' }, 'navigation.longitude': { v: 18.1, age: 1, q: 'live' },
    'evil.key': { v: 1, age: 0, q: 'live' } as any,
  }), TELEMETRY_KEYS, 5);
  assert.match(html, />5\.2<span class="unit">kn<\/span>/);
  assert.match(html, /Current · 6 s ago · GPS &lt;1&gt;/);
  assert.match(html, /class="tile stale[^"]*"[\s\S]*?51\.23[\s\S]*?Old · 7 min ago/);
  assert.match(html, /class="fill w16"/, 'SOC bar at 80 %');
  assert.match(html, /59° 18\.000′ N<br>18° 6\.000′ E/);
  assert.ok(!html.includes('evil'));
  assert.ok(!html.includes('GPS <1>'));
  // Required values the boat did not send are listed, not invented.
  assert.match(html, /Not reported by this boat/);
  assert.match(html, /Water temperature/);
  assert.match(html, /Not for navigation/);
  assert.match(renderOverview(null, TELEMETRY_KEYS, null), /Not reported by this boat/);
});

test('connection status and the anchor alarm banner', () => {
  assert.match(renderStatus(boat({}), DEVICE, 12, null), /Online[\s\S]*Last update 12 s ago/);
  assert.match(renderStatus(boat({}), DEVICE, 900, { since: Date.parse('2026-10-26T10:00:00Z') }), /Offline[\s\S]*offline or not sharing[\s\S]*No connection · saved/);
  assert.match(renderStatus(null, { ...DEVICE, telemetryEnabled: false, lastDataAt: null }, null, null), /Sharing off/);
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
  assert.match(html, /data-metric="battery.voltage"/);
  assert.ok(!html.includes('data-metric="wind.true_speed"'));
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
