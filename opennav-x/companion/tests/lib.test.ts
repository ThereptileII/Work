import assert from 'node:assert/strict';
import test from 'node:test';
import { Api, OfflineError, ServiceError, SignedOutError } from '../src/lib/api.ts';
import { anchorView, classify, connection, snapshotAgeNow, valueAge } from '../src/lib/freshness.ts';
import { bounds, circle, distanceM, trackSegments } from '../src/lib/geo.ts';
import { chartPath, clean, energy, niceDomain, stats } from '../src/lib/history.ts';
import { authorizeUrl, challengeFor, newPending, readCallback } from '../src/lib/pkce.ts';
import { resolveTheme } from '../src/theme.ts';

const DEVICE = '22222222-2222-4222-8222-222222222222';
const TOKEN = 'skc1_' + 'c'.repeat(64);

test('PKCE: RFC 7636 S256 and a 43-character verifier the server accepts', async () => {
  assert.equal(await challengeFor('dBjftJeZ4CVP-mB92K27uhbUJU1p1r_wW1gFWFOEjXk'), 'E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM');
  const pending = newPending(1000);
  assert.match(pending.verifier, /^[A-Za-z0-9_-]{43}$/);
  assert.match(pending.state, /^[A-Za-z0-9_-]{32}$/);
  assert.notEqual(newPending().verifier, pending.verifier);
  assert.match(await authorizeUrl(pending), /^https:\/\/beta\.skager\.app\/app\/authorize\/\?challenge=[A-Za-z0-9_-]{43}&state=[A-Za-z0-9_-]{32}$/);
});

test('the browser callback is accepted only with this sign-in\'s state', () => {
  const pending = { verifier: 'v'.repeat(43), state: 's'.repeat(32), createdAt: 0 };
  const code = 'a'.repeat(64);
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=${code}&state=${pending.state}`, pending, 1000), { code });
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=${code}&state=${'x'.repeat(32)}`, pending, 1000), { error: 'invalid' });
  assert.deepEqual(readCallback(`app.skager.companion://auth?error=access_denied&state=${pending.state}`, pending, 1000), { error: 'denied' });
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=${code}&state=${pending.state}`, null), { error: 'invalid' });
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=${code}&state=${pending.state}`, pending, 11 * 60_000), { error: 'expired' });
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=${code}&state=${pending.state}&state=${pending.state}`, pending, 1), { error: 'invalid' });
  assert.deepEqual(readCallback(`app.skager.companion://auth?code=XYZ&state=${pending.state}`, pending, 1), { error: 'invalid' });
  assert.equal(readCallback('https://evil.example/auth?code=1', pending), null);
});

test('API client: bearer token, version, ETag; offline, signed-out and service errors are distinct', async () => {
  const seen: { url: string; init: RequestInit }[] = [];
  let reply: () => Response | Promise<Response> = () => Response.json({ devices: [] });
  const api = new Api({ token: async () => TOKEN }, (async (url: string, init: RequestInit) => { seen.push({ url, init }); return reply(); }) as typeof fetch);
  await api.boats();
  const headers = seen[0].init.headers as Record<string, string>;
  assert.equal(seen[0].url, 'https://beta.skager.app/api/companion/boats');
  assert.equal(headers.Authorization, `Bearer ${TOKEN}`);
  assert.equal(seen[0].init.credentials, 'omit');
  reply = () => new Response(null, { status: 304 });
  assert.deepEqual(await api.boat(DEVICE, '"abc"'), { boat: null, etag: '"abc"' });
  assert.equal((seen[1].init.headers as Record<string, string>)['If-None-Match'], '"abc"');
  reply = () => Response.json({ device: { id: DEVICE } }, { headers: { ETag: '"def"' } });
  assert.equal((await api.boat(DEVICE)).etag, '"def"');
  reply = () => { throw new TypeError('network'); };
  await assert.rejects(api.boats(), OfflineError);
  reply = () => Response.json({ error: 'Please sign in again.' }, { status: 401 });
  await assert.rejects(api.boats(), SignedOutError);
  reply = () => Response.json({ error: 'Invalid request.' }, { status: 400 });
  await assert.rejects(api.boats(), ServiceError);
  reply = () => Response.json({ error: 'busy' }, { status: 503 });
  await assert.rejects(api.boats(), OfflineError);
  const noToken = new Api({ token: async () => null }, (async () => assert.fail('no request without a token')) as typeof fetch);
  await assert.rejects(noToken.boats(), SignedOutError);
  reply = () => Response.json({ token: TOKEN });
  assert.equal(await api.exchange('a'.repeat(64), 'v'.repeat(43), 'Pixel', 'android'), TOKEN);
  assert.equal((seen.at(-1)!.init.headers as Record<string, string>).Authorization, undefined, 'the exchange carries no token');
  assert.equal(JSON.parse(String(seen.at(-1)!.init.body)).platform, 'android');
});

test('freshness: server age plus phone-measured time; the boat\'s own quality wins when worse', () => {
  const boat = { snapshotAge: 10 };
  assert.equal(snapshotAgeNow(boat, 1_000, 6_000), 15);
  assert.equal(snapshotAgeNow(boat, 6_000, 1_000), 10, 'a clock step backwards never makes data younger than the server said');
  assert.equal(snapshotAgeNow({ snapshotAge: null }, 0, 0), null);
  const item = { v: 5, age: 2, q: 'live' as const };
  assert.equal(valueAge(item, 15), 17);
  assert.equal(classify(item, 17), 'live');
  assert.equal(classify(item, 45), 'aging');
  assert.equal(classify(item, 200), 'stale');
  assert.equal(classify({ ...item, q: 'stale' }, 3), 'stale');
  assert.equal(classify({ ...item, q: 'estimated' }, 3), 'estimated');
  assert.equal(classify(undefined, 3), 'unavailable');
  const device = { telemetryEnabled: true } as any;
  assert.equal(connection({ device, receivedAt: 'x' }, 20), 'online');
  assert.equal(connection({ device, receivedAt: 'x' }, 60), 'delayed');
  assert.equal(connection({ device, receivedAt: 'x' }, 400), 'offline');
  assert.equal(connection({ device: { telemetryEnabled: false } as any, receivedAt: null }, null), 'sharing-off');
  assert.equal(connection(null, null), 'no-data');
});

test('anchor view: an old report is never presented as a current watch', () => {
  assert.equal(anchorView(null, 5).state, 'unknown');
  assert.deepEqual(anchorView({ armed: false }, 5), { state: 'off', stale: false, distanceAge: null });
  assert.deepEqual(anchorView({ armed: true, alarm: true, distance_m: 60, age: 2 }, 10), { state: 'alarm', stale: false, distanceAge: 12 });
  assert.equal(anchorView({ armed: true, alarm: false }, 600).stale, true);
});

test('geometry: anchor circle radius, bounds and track gaps', () => {
  const ring = circle(59.3, 18.1, 50);
  assert.equal(ring.length, 73);
  assert.deepEqual(ring[0], ring[72]);
  for (const [lon, lat] of ring) assert.ok(Math.abs(distanceM(59.3, 18.1, lat, lon) - 50) < 0.01);
  assert.ok(Math.abs(distanceM(59.3, 18.1, ...circle(59.3, 18.1, -40)[10].slice().reverse() as [number, number]) - 40) < 0.01);
  const box = bounds([[18.1, 59.3]], 150)!;
  assert.ok(box[0][0] < 18.1 && box[1][1] > 59.3);
  assert.equal(bounds([]), null);
  assert.deepEqual(trackSegments([[0, 59, 18, 1], [15, 59.001, 18, 1], [5000, 59.1, 18.2, null], [5015, 59.11, 18.2, null]]).map(s => s.length), [2, 2]);
});

test('history: gaps stay gaps, energy is integrated from average power, stats are honest', () => {
  const points = clean([[600, 2], [0, 1], [60, 1.5], ['x', 3], [120, NaN]]);
  assert.deepEqual(points, [[0, 1], [60, 1.5], [600, 2]]);
  const path = chartPath(points, 60, 100, 50, { t0: 0, t1: 600, y0: 0, y1: 2 });
  assert.equal((path.match(/M/g) ?? []).length, 2, 'a 9-minute hole breaks the line');
  assert.deepEqual(stats(points), { min: 1, max: 2, mean: 1.5, last: 2, count: 3 });
  assert.equal(stats([]), null);
  const e = energy([[0, 6], [3600, 6], [7200, -3]], 3600)!;
  assert.deepEqual([e.used, e.charged, e.coveredHours], [12, 3, 3]);
  assert.equal(energy([], 60), null);
  const flat = niceDomain([[0, 5], [1, 5]]);
  assert.ok(flat.y0 < 5 && flat.y1 > 5);
});

test('theme: Auto follows the phone appearance', () => {
  assert.equal(resolveTheme('auto', true), 'night');
  assert.equal(resolveTheme('auto', false), 'day');
  assert.equal(resolveTheme('dusk', true), 'dusk');
});
