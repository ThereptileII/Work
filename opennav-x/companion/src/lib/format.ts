// Values are formatted exactly like the SKAGER portal does (shared code), so a
// reading looks the same on the web and on the phone.
// @ts-ignore: plain JavaScript module shared with web/edge/static.
import { formatReading as portalFormat } from '../../../web/edge/static/portal.js';

export const formatReading: (key: string, value: unknown, unit: string) => string = portalFormat;

const HTML: Record<string, string> = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' };
export const esc = (value: unknown) => String(value ?? '').replace(/[&<>"']/g, c => HTML[c]);

export function formatDistance(metres: number | undefined | null): string {
  if (typeof metres !== 'number' || !Number.isFinite(metres)) return '—';
  return metres < 1000 ? `${Math.round(metres)} m` : `${(metres / 1852).toFixed(2)} nm`;
}

/** Local clock time for an ISO timestamp, with the date when it is not today. */
export function formatTime(iso: string, now = new Date()): string {
  const at = new Date(iso);
  if (!Number.isFinite(at.getTime())) return '—';
  const time = at.toLocaleTimeString(undefined, { hour: '2-digit', minute: '2-digit' });
  return at.toDateString() === now.toDateString() ? time
    : `${at.toLocaleDateString(undefined, { day: 'numeric', month: 'short' })} ${time}`;
}
export function dayLabel(iso: string, now = new Date()): string {
  const at = new Date(iso);
  const yesterday = new Date(now.getTime() - 86_400_000);
  if (at.toDateString() === now.toDateString()) return 'Today';
  if (at.toDateString() === yesterday.toDateString()) return 'Yesterday';
  return at.toLocaleDateString(undefined, { weekday: 'long', day: 'numeric', month: 'long' });
}
