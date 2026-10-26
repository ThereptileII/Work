import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ago, pilotLabel, quality, renderBoats, renderOverlay, renderToolbar, renderWelcome } from '../src/views.ts';

test('names from the boat and the phone are escaped', () => {
  const html = renderBoats([{ serverId: 's', name: '<img src=x onerror=alert(1)>', deviceId: 'd', addresses: [], pairedAt: '2026-10-26T12:00:00Z' }], null);
  assert.ok(!html.includes('<img src=x'));
  assert.ok(html.includes('&lt;img'));
  const busy = renderOverlay({ kind: 'busy', name: 'Aurora', by: '"><script>' });
  assert.ok(!busy.includes('<script>'));
});

test('pilot status is shown read-only', () => {
  assert.equal(pilotLabel({ state: 'auto', heading: 364.4 }), 'Auto 4°');
  assert.equal(pilotLabel({ mode: 'track' }), 'Track');
  assert.equal(pilotLabel({}), null);
  const bar = renderToolbar({ name: 'Aurora', quality: 'good', rtt: 12, pilot: { state: 'standby' }, keyboard: false, magnifier: false, menu: false, live: true });
  assert.match(bar, /status only/);
  assert.ok(!/engage|command/i.test(bar));
});

test('connection quality and times', () => {
  assert.equal(quality(null), 'none');
  assert.equal(quality(20), 'good');
  assert.equal(quality(140), 'fair');
  assert.equal(quality(300), 'poor');
  const now = Date.parse('2026-10-26T12:00:00Z');
  assert.equal(ago(undefined, now), 'Not connected yet');
  assert.equal(ago('2026-10-26T11:59:30Z', now), 'Connected just now');
  assert.equal(ago('2026-10-26T11:40:00Z', now), 'Connected 20 min ago');
});

test('every helm state offers a way forward', () => {
  for (const s of [{ kind: 'unreachable' }, { kind: 'busy', by: 'iPad' }, { kind: 'taken', by: 'iPad' }, { kind: 'revoked' },
    { kind: 'ended', reason: 'server_stopped' }, { kind: 'error', message: 'x' }, { kind: 'connecting' }] as const) {
    const html = renderOverlay({ name: 'Aurora', ...s } as any);
    assert.match(html, /data-action="/, s.kind);
  }
  assert.match(renderWelcome(), /Scan pairing code/);
});
