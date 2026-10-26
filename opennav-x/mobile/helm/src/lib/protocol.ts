// SKAGER Helm wire protocol, the phone side of tools/helm-server (pairing.go,
// link.go, stream.go). Every authenticated message is a fixed label and
// newline-separated fields; see docs/helm/protocol.md.
import { hmac } from '@noble/hashes/hmac.js';
import { hkdf } from '@noble/hashes/hkdf.js';
import { sha256 } from '@noble/hashes/sha2.js';

export const APP_SCHEME = 'app.skager.helm';
export const DEFAULT_PORT = 43872;

const enc = new TextEncoder();

export function b64url(bytes: Uint8Array): string {
  let s = '';
  for (const b of bytes) s += String.fromCharCode(b);
  return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}
export function fromB64url(text: string): Uint8Array {
  const s = atob(text.replace(/-/g, '+').replace(/_/g, '/') + '==='.slice((text.length + 3) % 4));
  return Uint8Array.from(s, c => c.charCodeAt(0));
}
export function randomBytes(n: number): Uint8Array {
  const b = new Uint8Array(n);
  crypto.getRandomValues(b);
  return b;
}
export const nonce = () => b64url(randomBytes(16));

export function mac(key: Uint8Array, message: string): string {
  return b64url(hmac(sha256, key, enc.encode(message)));
}

/** Constant-time comparison of two base64url MACs. */
export function sameMac(a: string, b: string): boolean {
  if (a.length !== b.length) return false;
  let diff = 0;
  for (let i = 0; i < a.length; i++) diff |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return diff === 0;
}

export const pairMessage = (serverId: string, clientNonce: string, name: string) =>
  `skager-helm pair v1\n${serverId}\n${clientNonce}\n${name}`;
export const pairedMessage = (deviceId: string, clientNonce: string, serverNonce: string) =>
  `skager-helm paired v1\n${deviceId}\n${clientNonce}\n${serverNonce}`;
export const offerMessage = (deviceId: string, challenge: string, clientNonce: string, sdp: string, takeover: boolean) =>
  `skager-helm offer v1\n${deviceId}\n${challenge}\n${clientNonce}\n${takeover ? 1 : 0}\n${sdp}`;
export const answerMessage = (deviceId: string, challenge: string, clientNonce: string, sdp: string) =>
  `skager-helm answer v1\n${deviceId}\n${challenge}\n${clientNonce}\n${sdp}`;

export function deviceKey(secret: Uint8Array, clientNonce: Uint8Array, serverNonce: Uint8Array, serverId: string, deviceId: string): Uint8Array {
  const salt = new Uint8Array(clientNonce.length + serverNonce.length);
  salt.set(clientNonce);
  salt.set(serverNonce, clientNonce.length);
  return hkdf(sha256, secret, salt, enc.encode(`skager-helm device v1\n${serverId}\n${deviceId}`), 32);
}

// ---------------------------------------------------------------- pairing key and link
const B32 = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ234567';

/** The pairing key as typed: any case, dashes or spaces; 0/1 read as O/I. */
export function parsePairingKey(text: string): Uint8Array | null {
  const clean = text.toUpperCase().replace(/[\s-]/g, '').replace(/0/g, 'O').replace(/1/g, 'I');
  if (!/^[A-Z2-7]{16}$/.test(clean)) return null;
  const out = new Uint8Array(10);
  let bits = 0, value = 0, i = 0;
  for (const c of clean) {
    value = (value << 5) | B32.indexOf(c);
    bits += 5;
    if (bits >= 8) { out[i++] = (value >>> (bits - 8)) & 0xff; bits -= 8; }
  }
  return out;
}

/** Formats typed key text as ABCD-EFGH-JKLM-NPQR while the user types. */
export function formatKeyInput(text: string): string {
  const clean = text.toUpperCase().replace(/[^A-Z0-9]/g, '').slice(0, 16);
  return clean.replace(/(.{4})(?=.)/g, '$1-');
}

export type PairingLink = { serverId: string; name: string; addresses: string[]; secret: Uint8Array };

export function parsePairingLink(link: string): PairingLink | null {
  const m = /^app\.skager\.helm:\/\/pair\/?\?(.*)$/i.exec(link.trim());
  if (!m) return null;
  const q = new URLSearchParams(m[1]);
  if (q.get('v') !== '1') return null;
  const secret = parsePairingKey(q.get('k') ?? '');
  const serverId = q.get('id') ?? '';
  if (!secret || !/^[A-Za-z0-9_-]{8,64}$/.test(serverId)) return null;
  const addresses = (q.get('a') ?? '').split(',').map(normalizeAddress).filter((a): a is string => !!a).slice(0, 6);
  return { serverId, name: (q.get('n') ?? '').slice(0, 60) || 'SKAGER', addresses, secret };
}

/** "192.168.1.20", "192.168.1.20:43872" or a host name → host:port, or null. */
export function normalizeAddress(text: string): string | null {
  let t = text.trim().replace(/^https?:\/\//i, '').replace(/\/.*$/, '');
  if (!t) return null;
  if (!/:\d+$/.test(t)) t += `:${DEFAULT_PORT}`;
  const m = /^([A-Za-z0-9.-]{1,253}):(\d{1,5})$/.exec(t);
  if (!m || Number(m[2]) < 1 || Number(m[2]) > 65535) return null;
  return `${m[1].toLowerCase()}:${m[2]}`;
}

// ---------------------------------------------------------------- screen stream
export const MSG_BEGIN = 1, MSG_TILE = 2, MSG_END = 3;
export type StreamMessage =
  | { kind: 'begin'; frame: number; width: number; height: number; full: boolean; tiles: number }
  | { kind: 'tile'; frame: number; x: number; y: number; w: number; h: number; data: Uint8Array }
  | { kind: 'end'; frame: number; captureMs: number; encodeMs: number };

export function parseStreamMessage(buffer: ArrayBuffer): StreamMessage | null {
  const v = new DataView(buffer);
  if (v.byteLength < 9) return null;
  const frame = v.getUint32(1, true);
  switch (v.getUint8(0)) {
    case MSG_BEGIN:
      if (v.byteLength < 12) return null;
      return { kind: 'begin', frame, width: v.getUint16(5, true), height: v.getUint16(7, true), full: (v.getUint8(9) & 1) === 1, tiles: v.getUint16(10, true) };
    case MSG_TILE:
      if (v.byteLength < 14) return null;
      return { kind: 'tile', frame, x: v.getUint16(5, true), y: v.getUint16(7, true), w: v.getUint16(9, true), h: v.getUint16(11, true), data: new Uint8Array(buffer, 13) };
    case MSG_END:
      return { kind: 'end', frame, captureMs: v.getUint16(5, true), encodeMs: v.getUint16(7, true) };
  }
  return null;
}
