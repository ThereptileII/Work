// SKAGER Cloud phone API client (web/edge/companion.ts). Read-only: nothing here
// can send a command to a boat.
import { API_ORIGIN, APP_VERSION } from './config.ts';

export type Json = Record<string, any>;
export type Cloud = { active: boolean; status: string | null; renewsAt: string | null; cancelAtPeriodEnd: boolean };
export type TelemetryKey = { label: string; unit: string; min: number; max: number };
export type Item = { v: number; age: number; q: 'live' | 'aging' | 'stale' | 'estimated' | 'uncertain'; src?: string };
export type Anchor = { armed: boolean; alarm?: boolean; lat?: number; lon?: number; radius_m?: number; distance_m?: number;
  age?: number; src?: string };
export type Device = { id: string; name: string; platform: string; appVersion: string; telemetryEnabled: boolean;
  createdAt: string; lastSeenAt: string; lastDataAt: string | null; anchor?: Anchor | null; lastEventId?: number | null };
export type Boats = { cloud: Cloud | null; devices: Device[]; telemetryKeys: Record<string, TelemetryKey> };
export type Boat = { device: Device; receivedAt: string | null; items: Record<string, Item>; anchor: Anchor | null;
  snapshotAge: number | null };
export type BoatEvent = { id: number; deviceId: string; boat: string; kind: string; severity: 'critical' | 'warning' | 'info';
  title: string; detail: string; data: Json; occurredAt: string; receivedAt: string };
export type Series = { bucketSeconds: number; series: Record<string, [number, number][]> };
export type TrackPoint = [number, number, number, number | null];

/** The phone could not reach SKAGER Cloud (no network, timeout, server unreachable). */
export class OfflineError extends Error {}
/** The phone token is no longer valid: signed out here, in the portal, or the account closed. */
export class SignedOutError extends Error {}
export class ServiceError extends Error {
  readonly status: number;
  constructor(message: string, status: number) { super(message); this.status = status; }
}

export interface TokenSource { token(): Promise<string | null> }

export class Api {
  private readonly tokens: TokenSource;
  private readonly transport: typeof fetch;
  private readonly origin: string;
  private readonly timeoutMs: number;
  constructor(tokens: TokenSource, transport: typeof fetch = (...a) => fetch(...a), origin = API_ORIGIN, timeoutMs = 15_000) {
    this.tokens = tokens; this.transport = transport; this.origin = origin; this.timeoutMs = timeoutMs;
  }

  private async call(path: string, init: { method?: 'GET' | 'POST'; body?: unknown; etag?: string; auth?: boolean } = {}) {
    const headers: Record<string, string> = { 'X-Skager-Version': APP_VERSION };
    if (init.auth !== false) {
      const token = await this.tokens.token();
      if (!token) throw new SignedOutError('Not signed in');
      headers.Authorization = `Bearer ${token}`;
    }
    if (init.body !== undefined) headers['Content-Type'] = 'application/json';
    if (init.etag) headers['If-None-Match'] = init.etag;
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeoutMs);
    let response: Response;
    try {
      response = await this.transport(this.origin + path, { method: init.method ?? 'GET', headers, cache: 'no-store',
        credentials: 'omit', redirect: 'error', signal: controller.signal,
        body: init.body === undefined ? undefined : JSON.stringify(init.body) });
    } catch { throw new OfflineError('SKAGER Cloud cannot be reached'); }
    finally { clearTimeout(timer); }
    if (response.status === 304) return { status: 304, etag: init.etag ?? null, data: null as Json | null };
    const data = await response.json().catch(() => null) as Json | null;
    if (response.status === 401 && init.auth !== false) throw new SignedOutError(data?.error ?? 'Please sign in again.');
    if (response.status === 503 || response.status === 502 || response.status === 504) throw new OfflineError(data?.error ?? 'SKAGER Cloud is temporarily unavailable');
    if (!response.ok || !data) throw new ServiceError(data?.error ?? 'Something went wrong. Please try again.', response.status);
    return { status: response.status, etag: response.headers.get('ETag'), data };
  }

  async exchange(code: string, verifier: string, name: string, platform: 'android' | 'ios'): Promise<string> {
    const { data } = await this.call('/api/companion/token', { method: 'POST', auth: false,
      body: { code, verifier, name: name.slice(0, 60) || 'Phone', platform, version: APP_VERSION } });
    if (!/^skc1_[a-f0-9]{64}$/.test(data?.token ?? '')) throw new ServiceError('Sign-in could not be completed.', 500);
    return data!.token;
  }
  async session(): Promise<{ account: { email: string }; cloud: Cloud; push: boolean }> {
    return (await this.call('/api/companion/session')).data as any;
  }
  async boats(): Promise<Boats> { return (await this.call('/api/companion/boats')).data as Boats; }
  /** Unchanged since `etag`: returns null (HTTP 304, no body transferred). */
  async boat(device: string, etag?: string): Promise<{ boat: Boat | null; etag: string | null }> {
    const result = await this.call(`/api/companion/boat?device=${encodeURIComponent(device)}`, { etag });
    return { boat: result.status === 304 ? null : result.data as Boat, etag: result.etag };
  }
  async series(device: string, keys: string[], hours: number): Promise<Series> {
    return (await this.call(`/api/companion/series?device=${encodeURIComponent(device)}&keys=${keys.join(',')}&hours=${hours}`)).data as Series;
  }
  async track(device: string, hours: number): Promise<TrackPoint[]> {
    return ((await this.call(`/api/companion/track?device=${encodeURIComponent(device)}&hours=${hours}`)).data as Json).points;
  }
  async events(options: { device?: string; before?: number; limit?: number } = {}): Promise<BoatEvent[]> {
    const query = [options.device && `device=${options.device}`, options.before && `before=${options.before}`,
      options.limit && `limit=${options.limit}`].filter(Boolean).join('&');
    return ((await this.call('/api/companion/events' + (query ? `?${query}` : ''))).data as Json).events;
  }
  async registerPush(service: 'fcm' | 'apns' | null, token: string | null): Promise<void> {
    await this.call('/api/companion/push', { method: 'POST', body: token ? { service, token } : { token: null } });
  }
  async signOut(): Promise<void> { await this.call('/api/companion/signout', { method: 'POST', body: {} }); }
}
