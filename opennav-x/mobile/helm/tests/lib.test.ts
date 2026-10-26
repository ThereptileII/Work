import { test } from 'node:test';
import assert from 'node:assert/strict';
import { answerMessage, b64url, deviceKey, formatKeyInput, mac, normalizeAddress, offerMessage, pairedMessage, pairMessage,
  parsePairingKey, parsePairingLink, parseStreamMessage, sameMac } from '../src/lib/protocol.ts';
import { Gestures } from '../src/lib/gestures.ts';
import type { Point } from '../src/lib/gestures.ts';
import { clampPan, fitRect, toFrame } from '../src/lib/screen.ts';
import type { InputEvent } from '../src/lib/client.ts';

const hex = (b: Uint8Array) => Buffer.from(b).toString('hex');

test('crypto matches the Helm Server (tools/helm-server/vectors_test.go)', () => {
  const secret = parsePairingKey('MFRG-GZDF-MZTW-Q2LK')!;
  assert.equal(hex(secret), '6162636465666768696a');
  const cn = Uint8Array.from({ length: 16 }, (_, i) => i), sn = Uint8Array.from({ length: 16 }, (_, i) => 0xf0 + i);
  const key = deviceKey(secret, cn, sn, 'srv-id-123', 'dev-id-456');
  assert.equal(hex(key), 'f09c0dbb89b09fb48068dfb70e0b9ee7c50f61f7f8abd6f6419633d54e4273df');
  assert.equal(mac(secret, pairMessage('srv-id-123', b64url(cn), "Kristoffer's iPhone")), 'I3rNCSABtAlm1yCr1h-ZYq-i0Kh44ruXuNyzn8XTgaY');
  assert.equal(mac(key, pairedMessage('dev-id-456', b64url(cn), b64url(sn))), 'AisXu2TQiz_G5mFCPGey6yBWZuocJqaK94uZrsEsAB8');
  assert.equal(mac(key, offerMessage('dev-id-456', 'chal', b64url(cn), 'v=0\r\no=- 1 2 IN IP4 0.0.0.0\r\n', true)), 'u-0UVbiCISaOHcH3s7Fm_0l2N9an9gNqBFFhB-9Q3ZE');
  assert.equal(mac(key, answerMessage('dev-id-456', 'chal', b64url(cn), 'v=0\r\n')), '0JUoQoVOUgZbNCxh13Wr2DbLjqgTe89magcsLpveGTw');
  assert.ok(sameMac('abc', 'abc') && !sameMac('abc', 'abd') && !sameMac('abc', 'ab'));
});

test('pairing keys and links', () => {
  assert.equal(hex(parsePairingKey('mfrg gzdf mztw q2lk')!), '6162636465666768696a');
  assert.equal(parsePairingKey('MFRG-GZDF'), null);
  assert.equal(formatKeyInput('mfrggzdfmztwq2lkXYZ'), 'MFRG-GZDF-MZTW-Q2LK');
  const link = parsePairingLink('app.skager.helm://pair?v=1&id=N0E4FbxAWy2bDg&n=Aurora+%26+co&a=192.168.1.20%3A43872%2C10.0.0.5%3A43872&k=MFRGGZDFMZTWQ2LK');
  assert.ok(link);
  assert.deepEqual(link.addresses, ['192.168.1.20:43872', '10.0.0.5:43872']);
  assert.equal(link.name, 'Aurora & co');
  assert.equal(parsePairingLink('https://evil.example/pair?k=MFRGGZDFMZTWQ2LK'), null);
  assert.equal(parsePairingLink('app.skager.helm://pair?v=2&id=N0E4FbxAWy2bDg&k=MFRGGZDFMZTWQ2LK'), null);
  assert.equal(normalizeAddress('192.168.1.20'), '192.168.1.20:43872');
  assert.equal(normalizeAddress('http://Boat-PC.local:5000/x'), 'boat-pc.local:5000');
  assert.equal(normalizeAddress('a b'), null);
  assert.equal(normalizeAddress('10.0.0.1:99999'), null);
});

test('stream messages', () => {
  const begin = new ArrayBuffer(12);
  const v = new DataView(begin);
  v.setUint8(0, 1); v.setUint32(1, 7, true); v.setUint16(5, 1920, true); v.setUint16(7, 1080, true); v.setUint8(9, 1); v.setUint16(10, 3, true);
  assert.deepEqual(parseStreamMessage(begin), { kind: 'begin', frame: 7, width: 1920, height: 1080, full: true, tiles: 3 });
  const tile = new Uint8Array([2, 7, 0, 0, 0, 64, 0, 128, 0, 64, 0, 64, 0, 0xff, 0xd8]);
  const t = parseStreamMessage(tile.buffer);
  assert.equal(t?.kind, 'tile');
  if (t?.kind === 'tile') { assert.equal(t.x, 64); assert.equal(t.y, 128); assert.deepEqual([...t.data], [0xff, 0xd8]); }
  assert.equal(parseStreamMessage(new ArrayBuffer(3)), null);
});

test('screen fit, magnifier and touch mapping', () => {
  const r = fitRect(800, 400, 1920, 1080);
  assert.equal(Math.round(r.w), 711);
  assert.equal(r.h, 400);
  assert.deepEqual(toFrame(r, { x: r.x + r.w / 2, y: 200 }), { x: 0.5, y: 0.5 });
  assert.equal(toFrame(r, { x: 10, y: 10 }), null);
  assert.deepEqual(toFrame(r, { x: 10, y: -5 }, true), { x: 0, y: 0 });
  // Magnified 2×: panning stops at the frame edge.
  assert.deepEqual(clampPan(800, 400, 1920, 1080, 2, 10_000, -10_000), [2800 / 9, -200]);
  assert.deepEqual(clampPan(800, 400, 1920, 1080, 1, 50, 50), [0, 0]);
});

function gestures() {
  const out: InputEvent[] = [];
  const views: number[] = [];
  const map = (p: Point, clamp?: boolean) => {
    const q = { x: p.x / 100, y: p.y / 100 };
    if (clamp) return { x: Math.min(1, Math.max(0, q.x)), y: Math.min(1, Math.max(0, q.y)) };
    return q.x >= 0 && q.x <= 1 && q.y >= 0 && q.y <= 1 ? q : null;
  };
  const g = new Gestures(map, ev => out.push(ev), v => views.push(v.scale), () => {}, { slop: 10, longPressMs: 40, pinchStep: 1.2, moveIntervalMs: 0 });
  return { g, out, views };
}

test('tap is a click where the finger landed', () => {
  const { g, out } = gestures();
  g.down(1, { x: 50, y: 50 }, 0);
  g.move(1, { x: 53, y: 52 }, 10); // within slop
  g.up(1, { x: 53, y: 52 }, 80);
  assert.deepEqual(out.map(e => e.type), ['pointer_down', 'pointer_up']);
  assert.deepEqual(out[0], { type: 'pointer_down', x: 0.5, y: 0.5, button: 0 });
});

test('drag presses, moves and releases; past the edge it stays at the edge', () => {
  const { g, out } = gestures();
  g.down(1, { x: 20, y: 50 }, 0);
  g.move(1, { x: 40, y: 50 }, 10);
  g.move(1, { x: 140, y: 50 }, 20);
  g.up(1, { x: 140, y: 50 }, 30);
  assert.deepEqual(out.map(e => e.type), ['pointer_down', 'pointer_move', 'pointer_move', 'pointer_move', 'pointer_up']);
  assert.deepEqual(out.at(-1), { type: 'pointer_up', x: 1, y: 0.5, button: 0 });
});

test('touch and hold is a secondary click', async () => {
  const { g, out } = gestures();
  g.down(1, { x: 30, y: 30 }, 0);
  await new Promise(r => setTimeout(r, 80));
  g.up(1, { x: 30, y: 30 }, 100);
  assert.deepEqual(out.map(e => [e.type, 'button' in e ? e.button : undefined]), [['pointer_down', 2], ['pointer_up', 2]]);
});

test('pinch zooms the chart; with the magnifier it zooms the view instead', () => {
  const { g, out, views } = gestures();
  g.down(1, { x: 40, y: 50 }, 0);
  g.down(2, { x: 60, y: 50 }, 5);
  g.move(2, { x: 80, y: 50 }, 10); // distance 20 → 40: two steps of 1.2 and more
  assert.ok(out.length >= 3 && out.every(e => e.type === 'wheel' && e.delta === 1));
  g.up(1, { x: 40, y: 50 }, 20);
  g.up(2, { x: 80, y: 50 }, 20);
  const before = out.length;
  g.magnifier = true;
  g.down(1, { x: 40, y: 50 }, 30);
  g.down(2, { x: 60, y: 50 }, 31);
  g.move(2, { x: 70, y: 50 }, 40);
  assert.equal(out.length, before);
  assert.ok(Math.abs(views[0] - 1.5) < 1e-9);
});

test('a drag cut off by the system never leaves the button held', () => {
  const { g, out } = gestures();
  g.down(1, { x: 20, y: 20 }, 0);
  g.move(1, { x: 50, y: 20 }, 10);
  g.cancel();
  assert.equal(out.at(-1)?.type, 'pointer_up');
});
