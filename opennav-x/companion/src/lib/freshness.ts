// How old a value is and how to label it. The phone clock is never trusted
// against the server: ages come from the server (snapshotAge) plus the time the
// phone has measured since it received that answer.
import type { Anchor, Boat, Item } from './api.ts';

/** SKAGER sends every 15 s: three missed uploads are "delayed", two minutes "offline". */
export const CONNECTION = { delayedAfter: 45, offlineAfter: 120 };
/** Per-value thresholds on top of the boat's own quality flag. */
export const VALUE = { agingAfter: 30, staleAfter: 120 };

export type Freshness = 'live' | 'aging' | 'stale' | 'estimated' | 'unavailable';
export type Connection = 'online' | 'delayed' | 'offline' | 'no-data' | 'sharing-off';

/** Seconds since the server received the snapshot, as of `now` (phone monotonic-ish time). */
export function snapshotAgeNow(boat: Pick<Boat, 'snapshotAge'>, receivedLocal: number, now: number): number | null {
  if (boat.snapshotAge === null || !Number.isFinite(boat.snapshotAge)) return null;
  return boat.snapshotAge + Math.max(0, (now - receivedLocal) / 1000);
}

export function valueAge(item: Item | undefined, snapshotAge: number | null): number | null {
  if (!item || snapshotAge === null || !Number.isFinite(item.age)) return null;
  return snapshotAge + item.age;
}

/** The boat's own quality wins when it is worse; age can only make a value older. */
export function classify(item: Item | undefined, age: number | null): Freshness {
  if (!item || age === null || !Number.isFinite(item.v)) return 'unavailable';
  if (item.q === 'stale' || age > VALUE.staleAfter) return 'stale';
  if (item.q === 'estimated' || item.q === 'uncertain') return 'estimated';
  if (item.q === 'aging' || age > VALUE.agingAfter) return 'aging';
  return 'live';
}

export function connection(boat: Pick<Boat, 'device' | 'receivedAt'> | null, snapshotAge: number | null): Connection {
  if (!boat) return 'no-data';
  if (!boat.device.telemetryEnabled && !boat.receivedAt) return 'sharing-off';
  if (snapshotAge === null) return 'no-data';
  if (snapshotAge > CONNECTION.offlineAfter) return 'offline';
  return snapshotAge > CONNECTION.delayedAfter ? 'delayed' : 'online';
}

export const FRESHNESS_LABEL: Record<Freshness, string> = {
  live: 'Current', aging: 'Aging', stale: 'Old', estimated: 'Estimated', unavailable: 'Not reported',
};
export const CONNECTION_LABEL: Record<Connection, string> = {
  online: 'Online', delayed: 'Delayed', offline: 'Offline', 'no-data': 'No data yet', 'sharing-off': 'Sharing off',
};

export type AnchorView = { state: 'off' | 'watching' | 'alarm' | 'unknown'; stale: boolean; distanceAge: number | null };
/** The watch as OpenCPN last reported it. An old report is never shown as a current watch. */
export function anchorView(anchor: Anchor | null | undefined, snapshotAge: number | null): AnchorView {
  if (!anchor) return { state: 'unknown', stale: true, distanceAge: null };
  const stale = snapshotAge === null || snapshotAge > CONNECTION.offlineAfter;
  if (!anchor.armed) return { state: 'off', stale, distanceAge: null };
  const distanceAge = anchor.age !== undefined && snapshotAge !== null ? snapshotAge + anchor.age : null;
  return { state: anchor.alarm ? 'alarm' : 'watching', stale, distanceAge };
}

export function ago(seconds: number | null): string {
  if (seconds === null || !Number.isFinite(seconds)) return '—';
  if (seconds < 90) return `${Math.max(0, Math.round(seconds))} s ago`;
  if (seconds < 5400) return `${Math.round(seconds / 60)} min ago`;
  if (seconds < 172800) return `${Math.round(seconds / 3600)} h ago`;
  return `${Math.round(seconds / 86400)} days ago`;
}
