// Talks to the SKAGER Helm Server on the boat network: finds it, pairs, and opens
// the authenticated WebRTC connection that carries the screen and the touches.
import { answerMessage, b64url, deviceKey, fromB64url, mac, nonce, normalizeAddress, offerMessage,
  pairedMessage, pairMessage, parsePairingKey, sameMac } from './protocol.ts';
import type { PairingLink } from './protocol.ts';

export type Boat = { serverId: string; name: string; deviceId: string; addresses: string[]; lastAddress?: string;
  pairedAt: string; lastConnected?: string };

export type ErrorCode = 'unreachable' | 'no_pairing' | 'wrong_key' | 'unpaired' | 'busy' | 'expired' | 'too_many'
  | 'invalid' | 'webrtc' | 'auth' | 'other_boat';

export class HelmError extends Error {
  code: ErrorCode;
  by?: string;
  constructor(code: ErrorCode, message: string, by?: string) {
    super(message);
    this.code = code;
    this.by = by;
  }
}

export type Info = { v: number; server_id: string; name: string; pairing: boolean; software: string };

type Fetch = typeof fetch;
let fetcher: Fetch = (...args) => fetch(...args);
/** Tests and the native layer may replace the HTTP function. */
export function setFetch(f: Fetch): void { fetcher = f; }

async function call<T>(address: string, path: string, body: unknown | undefined, timeoutMs: number): Promise<{ status: number; data: T }> {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const res = await fetcher(`http://${address}${path}`, {
      method: body === undefined ? 'GET' : 'POST', signal: controller.signal, cache: 'no-store',
      headers: body === undefined ? undefined : { 'content-type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    const data = await res.json().catch(() => ({})) as T;
    return { status: res.status, data };
  } catch {
    throw new HelmError('unreachable', 'Could not reach SKAGER on this network.');
  } finally {
    clearTimeout(timer);
  }
}

type ErrorBody = { error?: string; message?: string; by?: string };
function fail(status: number, body: ErrorBody): never {
  const code = (body.error ?? '') as ErrorCode;
  const known: ErrorCode[] = ['no_pairing', 'wrong_key', 'unpaired', 'busy', 'expired', 'too_many', 'invalid', 'webrtc'];
  if (known.includes(code)) throw new HelmError(code, body.message ?? 'SKAGER refused the request.', body.by);
  throw new HelmError(status === 429 ? 'expired' : 'invalid', body.message ?? `SKAGER answered ${status}.`);
}

/**
 * Finds the SKAGER PC: asks every known address at once and takes the first that
 * answers as the expected server. The last working address is tried first.
 */
export async function locate(addresses: string[], serverId?: string, timeoutMs = 2500): Promise<{ address: string; info: Info }> {
  const unique = [...new Set(addresses.map(a => normalizeAddress(a)).filter((a): a is string => !!a))];
  if (!unique.length) throw new HelmError('unreachable', 'No address for this SKAGER.');
  let wrongBoat = false;
  const attempts = unique.map(async address => {
    const { status, data } = await call<Info>(address, '/helm/v1/info', undefined, timeoutMs);
    if (status !== 200 || data.v !== 1) throw new HelmError('unreachable', 'Not SKAGER Helm.');
    if (serverId && data.server_id !== serverId) { wrongBoat = true; throw new HelmError('other_boat', 'A different SKAGER answered.'); }
    return { address, info: data };
  });
  try {
    return await Promise.any(attempts);
  } catch {
    if (wrongBoat) throw new HelmError('other_boat', 'A different SKAGER answered at this address.');
    throw new HelmError('unreachable', 'Could not reach SKAGER on this network.');
  }
}

export type Paired = { boat: Boat; key: Uint8Array };

async function pairAt(address: string, info: Info, secret: Uint8Array, deviceName: string, addresses: string[]): Promise<Paired> {
  const name = deviceName.replace(/[\u0000-\u001f\u007f-\u009f]/g, '').trim().slice(0, 60) || 'Phone';
  const clientNonce = nonce();
  const { status, data } = await call<ErrorBody & { device_id?: string; server_id?: string; server_nonce?: string; proof?: string }>(
    address, '/helm/v1/pair', { v: 1, device_name: name, client_nonce: clientNonce, proof: mac(secret, pairMessage(info.server_id, clientNonce, name)) }, 6000);
  if (status !== 200) fail(status, data);
  if (!data.device_id || !data.server_nonce || !data.proof || data.server_id !== info.server_id) throw new HelmError('auth', 'SKAGER sent an unexpected answer.');
  const key = deviceKey(secret, fromB64url(clientNonce), fromB64url(data.server_nonce), info.server_id, data.device_id);
  if (!sameMac(mac(key, pairedMessage(data.device_id, clientNonce, data.server_nonce)), data.proof)) {
    throw new HelmError('auth', 'This SKAGER could not prove it holds the pairing key.');
  }
  const all = [address, ...addresses.filter(a => a !== address)];
  return { key, boat: { serverId: info.server_id, name: info.name, deviceId: data.device_id, addresses: all, lastAddress: address,
    pairedAt: new Date().toISOString() } };
}

export async function pairWithLink(link: PairingLink, deviceName: string): Promise<Paired> {
  const { address, info } = await locate(link.addresses, link.serverId);
  return pairAt(address, info, link.secret, deviceName, link.addresses);
}

export async function pairManually(addressText: string, keyText: string, deviceName: string): Promise<Paired> {
  const address = normalizeAddress(addressText);
  if (!address) throw new HelmError('invalid', 'Enter the address shown on the SKAGER screen, for example 192.168.1.20.');
  const secret = parsePairingKey(keyText);
  if (!secret) throw new HelmError('wrong_key', 'The pairing key has 16 letters and digits, for example ABCD-EFGH-JKLM-NPQR.');
  const { info } = await locate([address]);
  return pairAt(address, info, secret, deviceName, [address]);
}

// ---------------------------------------------------------------- connection

export type InputEvent =
  | { type: 'pointer_down' | 'pointer_up' | 'pointer_move'; x: number; y: number; button?: 0 | 2 }
  | { type: 'wheel'; x: number; y: number; delta: number }
  | { type: 'key'; key: string }
  | { type: 'text'; text: string };

export type ControlMessage =
  | { type: 'hello'; name: string; server_id: string; software: string; capabilities: string[]; pilot?: Pilot }
  | { type: 'stats'; stats: { fps: number; kbps: number; capture_ms: number; encode_ms: number; width: number; height: number } }
  | { type: 'screen'; available: boolean }
  | { type: 'pilot'; status: Pilot }
  | { type: 'bye'; reason: string; by?: string }
  | { type: 'pong'; t: number };

export type Pilot = { state?: string; mode?: string; heading?: number; commanded?: number; source?: string; message?: string };

export type ConnectionHandlers = {
  screen(data: ArrayBuffer): void;
  control(msg: ControlMessage): void;
  inputAck(seq: number, accepted: boolean, reason?: string): void;
  closed(reason: string): void;
};

/** One authenticated WebRTC session with the SKAGER PC. */
export class Connection {
  boat: Boat;
  address: string;
  pc: RTCPeerConnection;
  screen: RTCDataChannel;
  input: RTCDataChannel;
  control: RTCDataChannel;
  handlers: ConnectionHandlers;
  seq = 0;
  rtt: number | null = null;
  lastPong = performance.now();
  lostTimer: ReturnType<typeof setTimeout> | undefined;
  closed = false;
  /** Who took over, when SKAGER ended the session for another phone. */
  by = '';
  pinger: ReturnType<typeof setInterval> | undefined;

  constructor(boat: Boat, address: string, pc: RTCPeerConnection, handlers: ConnectionHandlers) {
    this.boat = boat;
    this.address = address;
    this.pc = pc;
    this.handlers = handlers;
    this.screen = pc.createDataChannel('screen', { ordered: true });
    this.screen.binaryType = 'arraybuffer';
    this.input = pc.createDataChannel('input', { ordered: true });
    this.control = pc.createDataChannel('control', { ordered: true });
    this.screen.onmessage = e => { if (e.data instanceof ArrayBuffer) handlers.screen(e.data); };
    this.control.onmessage = e => {
      try {
        const msg = JSON.parse(String(e.data)) as ControlMessage;
        if (msg.type === 'pong') { this.lastPong = performance.now(); this.rtt = Math.max(0, this.lastPong - msg.t); }
        if (msg.type === 'bye') { this.by = msg.by ?? ''; this.finish(msg.reason); }
        handlers.control(msg);
      } catch { /* ignore malformed */ }
    };
    this.input.onmessage = e => {
      try {
        const a = JSON.parse(String(e.data));
        if (a.type === 'input_ack') handlers.inputAck(a.sequence, a.accepted, a.reason);
      } catch { /* ignore */ }
    };
    pc.onconnectionstatechange = () => {
      const state = pc.connectionState;
      if (state === 'failed' || state === 'closed') this.finish('connection_lost');
      // Wi-Fi gone or the PC unreachable: give up after a short grace instead
      // of WebRTC's long failure timeout, and reconnect.
      clearTimeout(this.lostTimer);
      if (state === 'disconnected') this.lostTimer = setTimeout(() => this.finish('connection_lost'), 3000);
    };
  }

  get live(): boolean { return !this.closed && this.input.readyState === 'open' && this.pc.connectionState === 'connected'; }

  /** Sends one input event; false when it could not be sent (never queued for later). */
  send(ev: InputEvent): boolean {
    if (!this.live || this.input.bufferedAmount > 16_384) return false;
    this.input.send(JSON.stringify({ type: 'input', event: { ...ev, sequence: ++this.seq, t: performance.now() } }));
    return true;
  }

  sendControl(msg: Record<string, unknown>): void {
    if (this.control.readyState === 'open') this.control.send(JSON.stringify(msg));
  }

  startPings(): void {
    this.lastPong = performance.now();
    this.pinger = setInterval(() => {
      // No answer for 6 s: the PC is gone even if WebRTC has not noticed yet.
      if (performance.now() - this.lastPong > 6000) { this.finish('connection_lost'); return; }
      this.sendControl({ type: 'ping', t: performance.now() });
    }, 2000);
    this.sendControl({ type: 'ping', t: performance.now() });
  }

  finish(reason: string): void {
    if (this.closed) return;
    this.closed = true;
    clearInterval(this.pinger);
    clearTimeout(this.lostTimer);
    this.handlers.closed(reason);
    setTimeout(() => this.pc.close(), 100);
  }

  /** Ends the session on purpose: SKAGER is told, then the connection closes. */
  close(): void {
    if (this.closed) return;
    this.sendControl({ type: 'bye' });
    this.finish('closed_by_user');
  }
}

function iceGathered(pc: RTCPeerConnection, timeoutMs: number): Promise<void> {
  if (pc.iceGatheringState === 'complete') return Promise.resolve();
  return new Promise(resolve => {
    const done = () => { clearTimeout(timer); pc.removeEventListener('icegatheringstatechange', check); resolve(); };
    const check = () => { if (pc.iceGatheringState === 'complete') done(); };
    const timer = setTimeout(done, timeoutMs);
    pc.addEventListener('icegatheringstatechange', check);
  });
}

function opened(channels: RTCDataChannel[], pc: RTCPeerConnection, timeoutMs: number): Promise<void> {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new HelmError('webrtc', 'The connection to SKAGER did not open.')), timeoutMs);
    const check = () => {
      if (channels.every(c => c.readyState === 'open')) { clearTimeout(timer); resolve(); }
      if (pc.connectionState === 'failed') { clearTimeout(timer); reject(new HelmError('webrtc', 'The connection to SKAGER failed.')); }
    };
    for (const c of channels) c.addEventListener('open', check);
    pc.addEventListener('connectionstatechange', check);
    check();
  });
}

/**
 * Connects to a paired SKAGER. The offer and the answer are both authenticated
 * with the device key; they carry the DTLS fingerprints, so the screen and the
 * touches travel end to end encrypted to the PC this phone paired with.
 */
export async function connect(boat: Boat, key: Uint8Array, handlers: ConnectionHandlers, takeover = false): Promise<Connection> {
  const order = boat.lastAddress ? [boat.lastAddress, ...boat.addresses] : boat.addresses;
  const { address } = await locate(order, boat.serverId);
  const pc = new RTCPeerConnection({ iceServers: [] });
  const conn = new Connection(boat, address, pc, handlers);
  try {
    await pc.setLocalDescription(await pc.createOffer());
    await iceGathered(pc, 2500);
    const sdp = pc.localDescription?.sdp ?? '';
    const ch = await call<ErrorBody & { challenge?: string; server_id?: string }>(address, '/helm/v1/challenge', undefined, 4000);
    if (ch.status !== 200 || !ch.data.challenge) fail(ch.status, ch.data);
    if (ch.data.server_id !== boat.serverId) throw new HelmError('other_boat', 'A different SKAGER answered at this address.');
    const challenge = ch.data.challenge;
    const clientNonce = nonce();
    const res = await call<ErrorBody & { sdp?: string; mac?: string }>(address, '/helm/v1/connect', {
      device_id: boat.deviceId, challenge, client_nonce: clientNonce, sdp, takeover,
      mac: mac(key, offerMessage(boat.deviceId, challenge, clientNonce, sdp, takeover)) }, 8000);
    if (res.status !== 200) fail(res.status, res.data);
    const answer = res.data.sdp ?? '';
    if (!res.data.mac || !sameMac(mac(key, answerMessage(boat.deviceId, challenge, clientNonce, answer)), res.data.mac)) {
      throw new HelmError('auth', 'The answer was not signed by the paired SKAGER.');
    }
    await pc.setRemoteDescription({ type: 'answer', sdp: answer });
    await opened([conn.screen, conn.input, conn.control], pc, 10000);
    conn.startPings();
    return conn;
  } catch (err) {
    conn.closed = true;
    pc.close();
    throw err;
  }
}

export const keyText = (key: Uint8Array) => b64url(key);
export const keyFromText = (text: string) => fromB64url(text);
