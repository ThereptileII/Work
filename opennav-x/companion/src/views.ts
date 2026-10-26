// SKAGER Companion screens as pure HTML-string renderers (tested in Node).
// Every value from SKAGER Cloud is escaped. Missing data is shown as missing,
// and every value carries its age.
import type { Anchor, Boat, BoatEvent, Boats, Cloud, Device, Item, TelemetryKey } from './lib/api.ts';
import { anchorView, ago, classify, connection, CONNECTION_LABEL, FRESHNESS_LABEL, valueAge } from './lib/freshness.ts';
import type { Freshness } from './lib/freshness.ts';
import { dayLabel, esc, formatDistance, formatReading, formatTime } from './lib/format.ts';
import { chartPath, clean, energy, METRICS, niceDomain, RANGES, stats } from './lib/history.ts';
import type { Series } from './lib/api.ts';

export type Tab = 'overview' | 'map' | 'anchor' | 'history' | 'alarms';
export const TABS: [Tab, string][] = [['overview', 'Boat'], ['map', 'Map'], ['anchor', 'Anchor'], ['history', 'History'], ['alarms', 'Alarms']];

const ICON: Record<string, string> = {
  overview: '<path d="M4 15a8 8 0 1 1 16 0"/><path d="M12 15l4-5"/><circle cx="12" cy="15" r="1.4"/>',
  map: '<path d="M9 4 3 6.5v13L9 17l6 2.5 6-2.5v-13L15 6.5 9 4Z"/><path d="M9 4v13M15 6.5v13"/>',
  anchor: '<circle cx="12" cy="5" r="2"/><path d="M12 7v13M5 13a7 7 0 0 0 14 0M8 10h8"/>',
  history: '<path d="M4 19h16"/><path d="m5 15 4-5 4 3 6-7"/>',
  alarms: '<path d="M6 16V11a6 6 0 0 1 12 0v5l1.5 2h-15L6 16Z"/><path d="M10 20a2 2 0 0 0 4 0"/>',
  settings: '<circle cx="12" cy="12" r="3"/><path d="M12 2.8v2.4M12 18.8v2.4M2.8 12h2.4M18.8 12h2.4M5.5 5.5l1.7 1.7M16.8 16.8l1.7 1.7M5.5 18.5l1.7-1.7M16.8 7.2l1.7-1.7"/>',
  chevron: '<path d="m9 6 6 6-6 6"/>',
  close: '<path d="M6 6l12 12M18 6 6 18"/>',
};
export const icon = (name: string, label = '') =>
  `<svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" ${label ? `role="img" aria-label="${esc(label)}"` : 'aria-hidden="true"'}>${ICON[name] ?? ''}</svg>`;

export function renderTabs(active: Tab, alarm: boolean, unread: number): string {
  return TABS.map(([tab, label]) => `<button class="tab${tab === active ? ' active' : ''}" type="button" data-tab="${tab}" aria-current="${tab === active ? 'page' : 'false'}">
    <span class="tab-icon">${icon(tab)}${tab === 'anchor' && alarm ? '<span class="dot critical" aria-label="Anchor alarm"></span>' : ''}${tab === 'alarms' && unread ? `<span class="badge-count">${Math.min(unread, 99)}</span>` : ''}</span>
    <span>${label}</span></button>`).join('');
}

// ---------------------------------------------------------------- sign-in and empty states
export function renderSignIn(state: { busy?: boolean; message?: string; error?: boolean }): string {
  return `<section class="welcome">
    <img class="wordmark" src="assets/skager-wordmark.png" alt="SKAGER" width="170" height="54">
    <p class="eyebrow">Companion</p>
    <h1>Your boat, wherever you are.</h1>
    <p class="lead">See your boat's position, battery, wind and anchor watch from your phone, and get an alarm when something needs attention.</p>
    <button class="button primary" type="button" id="sign-in" ${state.busy ? 'disabled' : ''}>${state.busy ? 'Waiting for the browser…' : 'Sign in with your SKAGER account'}</button>
    ${state.message ? `<p class="message${state.error ? ' error' : ''}" role="status">${esc(state.message)}</p>` : ''}
    <p class="note">Sign-in opens beta.skager.app. Companion needs SKAGER Cloud and boat data sharing switched on in SKAGER on board (Settings → System → SKAGER Cloud).</p>
    <p class="note fine">Companion shows what SKAGER reports. It is not for navigation and cannot control the boat.</p>
  </section>`;
}

export function renderNoCloud(cloud: Cloud | null): string {
  return `<section class="card empty-state"><p class="eyebrow">SKAGER Cloud</p><h2>SKAGER Cloud is not active on this account.</h2>
    <p class="note">${cloud?.status === 'past_due' ? 'The last payment did not go through.' : 'Boat data, alarms and the anchor watch on your phone are part of SKAGER Cloud.'} You can manage your account at beta.skager.app.</p></section>`;
}

export function renderNoBoats(): string {
  return `<section class="card empty-state"><p class="eyebrow">No boat yet</p><h2>Link SKAGER on board to your account.</h2>
    <p class="note">In SKAGER on the boat computer, open Settings → System → SKAGER Cloud, choose Link, and switch on boat data sharing. Your boat appears here within a minute.</p></section>`;
}

export function renderBoatSwitch(boats: Boats | null, selected: string | null): string {
  const devices = boats?.devices ?? [];
  if (devices.length < 2) return '';
  return `<div class="boat-switch" role="tablist" aria-label="Boats">${devices.map(d => `<button type="button" role="tab" class="chip${d.id === selected ? ' active' : ''}" data-boat="${esc(d.id)}" aria-selected="${d.id === selected}">${esc(d.name)}</button>`).join('')}</div>`;
}

// ---------------------------------------------------------------- status
const STATUS_CLASS: Record<string, string> = { online: 'ok', delayed: 'wait', offline: 'bad', 'no-data': 'off', 'sharing-off': 'off' };

export function renderStatus(boat: Boat | null, device: Device | undefined, age: number | null, offline: { since: number | null } | null): string {
  const state = connection(boat ?? (device ? { device, receivedAt: device.lastDataAt } : null), age);
  const detail = state === 'sharing-off' ? 'Switch on boat data sharing in SKAGER on board.'
    : state === 'no-data' ? 'No boat data received yet.'
    : `Last update ${ago(age)}${state === 'offline' ? ' · SKAGER on board is offline or not sharing' : ''}`;
  return `<section class="status-card ${STATUS_CLASS[state]}">
    <div><p class="eyebrow">Connection</p><p class="status-line"><span class="dot ${STATUS_CLASS[state]}"></span>${CONNECTION_LABEL[state]}</p>
    <p class="note">${esc(detail)}</p></div>
    ${offline ? `<p class="offline-chip" role="status">No connection${offline.since ? ` · saved ${esc(formatTime(new Date(offline.since).toISOString()))}` : ''}</p>` : ''}
  </section>`;
}

export function renderAlarmBanner(device: Device | undefined, anchor: Anchor | null | undefined, age: number | null): string {
  const view = anchorView(anchor, age);
  if (view.state !== 'alarm') return '';
  const distance = anchor?.distance_m !== undefined ? ` · ${formatDistance(anchor.distance_m)} from the anchor` : '';
  return `<button type="button" class="alarm-banner${view.stale ? ' stale' : ''}" data-tab="anchor" role="alert">
    <strong>${view.stale ? 'Last report: anchor alarm' : 'Anchor alarm'}</strong>
    <span>${esc(device?.name ?? 'Your boat')}${esc(distance)}${view.stale ? ` · ${esc(ago(age))}` : ''}</span></button>`;
}

// ---------------------------------------------------------------- overview
type Tile = { key: string; label?: string; wide?: boolean; hero?: boolean };
const SECTIONS: { title: string; tiles: Tile[] }[] = [
  { title: 'Position and motion', tiles: [{ key: 'navigation.sog', hero: true }, { key: 'navigation.cog' }, { key: 'navigation.heading' },
    { key: 'navigation.stw' }, { key: 'navigation.latitude', label: 'Position', wide: true }] },
  { title: 'Battery', tiles: [{ key: 'battery.soc', hero: true }, { key: 'battery.voltage' }, { key: 'battery.current' },
    { key: 'battery.net_discharge', label: 'Battery power' }, { key: 'battery.soh' }, { key: 'battery.capacity' }] },
  { title: 'Motor', tiles: [{ key: 'propulsion.electrical_power', hero: true }, { key: 'propulsion.rpm' },
    { key: 'propulsion.motor_temperature' }, { key: 'propulsion.coolant_temperature' }, { key: 'propulsion.gear' },
    { key: 'propulsion.shaft_power' }, { key: 'propulsion.regeneration' }] },
  { title: 'Wind', tiles: [{ key: 'wind.true_speed', hero: true }, { key: 'wind.true_angle' }, { key: 'wind.apparent_speed' }, { key: 'wind.apparent_angle' }] },
  { title: 'Water and weather', tiles: [{ key: 'environment.depth', hero: true }, { key: 'environment.water_temperature' }, { key: 'environment.pressure' }] },
  { title: 'Tanks', tiles: [{ key: 'tanks.fresh_water' }, { key: 'tanks.fuel' }, { key: 'tanks.waste' }, { key: 'tanks.other' }] },
  { title: 'Steering', tiles: [{ key: 'steering.rudder' }, { key: 'steering.heel' }] },
];
/** The values the owner asked for on the overview: always listed, reported or not. */
export const REQUIRED = ['navigation.latitude', 'navigation.cog', 'navigation.sog', 'battery.soc', 'battery.voltage',
  'battery.current', 'battery.net_discharge', 'propulsion.electrical_power', 'propulsion.motor_temperature', 'wind.true_speed',
  'wind.true_angle', 'environment.depth', 'environment.water_temperature'];

function split(text: string): [string, string] {
  const match = /^(-?[\d.,]+)\s?(.*)$/.exec(text);
  return match ? [match[1], match[2]] : [text, ''];
}
function tile(t: Tile, items: Record<string, Item>, keys: Record<string, TelemetryKey>, age: number | null): string {
  const spec = keys[t.key];
  if (!spec) return '';
  const item = items[t.key];
  const itemAge = valueAge(item, age);
  const fresh: Freshness = classify(item, itemAge);
  let value: string, unit = '';
  if (t.key === 'navigation.latitude') {
    const lon = items['navigation.longitude'];
    value = item && lon ? `${esc(formatReading('navigation.latitude', item.v, 'deg'))}<br>${esc(formatReading('navigation.longitude', lon.v, 'deg'))}` : '—';
  } else {
    [value, unit] = item ? split(formatReading(t.key, item.v, spec.unit)) : ['—', ''];
    value = esc(value);
  }
  const bar = t.key === 'battery.soc' && item && fresh !== 'unavailable'
    ? `<span class="bar" aria-hidden="true"><span class="fill w${Math.max(0, Math.min(20, Math.round(item.v / 5)))}"></span></span>` : '';
  return `<div class="tile ${fresh}${t.wide ? ' wide' : ''}${t.hero ? ' hero' : ''}">
    <p class="tile-label">${esc(t.label ?? spec.label)}</p>
    <p class="tile-value${t.key === 'navigation.latitude' ? ' position' : ''}">${value}${unit ? `<span class="unit${unit === '°' ? ' deg' : ''}">${esc(unit)}</span>` : ''}</p>${bar}
    <p class="tile-age">${fresh === 'unavailable' ? 'Not reported' : `${FRESHNESS_LABEL[fresh]} · ${esc(ago(itemAge))}`}${item?.src ? ` · ${esc(item.src)}` : ''}</p>
  </div>`;
}

export function renderOverview(boat: Boat | null, keys: Record<string, TelemetryKey>, age: number | null): string {
  const items = boat?.items ?? {};
  const sections = SECTIONS.map(section => {
    const tiles = section.tiles.filter(t => items[t.key] || REQUIRED.includes(t.key));
    if (!tiles.some(t => items[t.key])) return { section, html: '' };
    return { section, html: `<h2 class="section-title">${esc(section.title)}</h2><div class="tiles">${tiles.map(t => tile(t, items, keys, age)).join('')}</div>` };
  });
  const missing = REQUIRED.filter(key => !items[key] && keys[key] && key !== 'navigation.longitude')
    .filter(key => !sections.some(s => s.html && s.section.tiles.some(t => t.key === key)));
  return `${sections.map(s => s.html).join('')}
    ${missing.length ? `<section class="card muted-card"><p class="eyebrow">Not reported by this boat</p><p class="note">${missing.map(k => esc(k === 'navigation.latitude' ? 'Position' : keys[k].label)).join(' · ')}</p></section>` : ''}
    <p class="note fine center">Values as SKAGER on board measured them. Not for navigation.</p>`;
}

// ---------------------------------------------------------------- anchor
export function renderAnchor(device: Device | undefined, anchor: Anchor | null | undefined, age: number | null, hasTrack: boolean): string {
  const view = anchorView(anchor, age);
  const name = esc(device?.name ?? 'your boat');
  if (view.state === 'unknown') {
    return `<section class="card empty-state"><p class="eyebrow">Anchor watch</p><h2>No anchor watch report yet.</h2>
      <p class="note">SKAGER on ${name} reports its anchor watch with boat data. Update SKAGER on board if this stays empty.</p></section>`;
  }
  if (view.state === 'off') {
    return `<section class="card empty-state"><p class="eyebrow">Anchor watch</p><h2>The anchor watch is off.</h2>
      <p class="note">Set the anchor watch in SKAGER on ${name}. Companion shows it here and alarms on your phone, but cannot start or stop it.</p>
      ${view.stale ? `<p class="note warn">Last report ${esc(ago(age))}.</p>` : ''}</section>`;
  }
  const radius = anchor!.radius_m;
  const inside = radius !== undefined && radius < 0;
  const ratio = anchor!.distance_m !== undefined && radius ? anchor!.distance_m / Math.abs(radius) : null;
  const headline = view.stale ? 'No contact' : view.state === 'alarm' ? 'Anchor alarm' : 'Watching';
  return `<section class="anchor-hero ${view.stale ? 'stale' : view.state}" role="${view.state === 'alarm' && !view.stale ? 'alert' : 'status'}">
      <p class="eyebrow">Anchor watch · ${name}</p>
      <h2>${headline}</h2>
      <p class="anchor-distance"><span class="big">${esc(formatDistance(anchor!.distance_m))}</span><span class="of">${radius !== undefined ? `of ${esc(formatDistance(Math.abs(radius)))} ${inside ? 'keep-out radius' : 'radius'}` : 'radius not reported'}</span></p>
      ${ratio !== null ? `<span class="bar anchor-bar" aria-hidden="true"><span class="fill w${Math.max(0, Math.min(20, Math.round(ratio * 20)))}${ratio >= 1 ? ' over' : ''}"></span></span>` : ''}
      <p class="note">${view.stale ? `Last report ${esc(ago(age))}. SKAGER on board keeps watching and sounds its alarm there; this phone cannot see it until the boat is back online.`
        : anchor!.distance_m === undefined ? 'Distance unavailable: SKAGER on board has no current position.'
        : `Distance measured ${esc(ago(view.distanceAge))}${anchor!.src ? ` · ${esc(anchor!.src)}` : ''}.`}</p>
    </section>
    <div class="map-slot anchor-map" data-map-slot="anchor"></div>
    <p class="note fine">${hasTrack ? 'Swing track: the boat’s reported positions over the last 6 hours.' : 'No positions reported in the last 6 hours.'} The alarm is decided by SKAGER on board (OpenCPN anchor watch) and works without internet; the circle shows the radius it reported.</p>`;
}

// ---------------------------------------------------------------- map
export function renderMapPanel(boat: Boat | null, age: number | null, hours: number, points: number): string {
  const lat = boat?.items['navigation.latitude'], lon = boat?.items['navigation.longitude'];
  const fresh = classify(lat, valueAge(lat, age));
  return `<div class="map-slot full" data-map-slot="map"></div>
    <div class="map-overlay">
      <div class="segmented" role="group" aria-label="Track length">${[1, 6, 24, 72, 168].map(h => `<button type="button" class="${h === hours ? 'active' : ''}" data-track="${h}">${h < 24 ? `${h} h` : `${h / 24} d`}</button>`).join('')}</div>
      <p class="map-caption">${lat && lon ? `${fresh === 'live' || fresh === 'aging' ? 'Position' : 'Last known position'} · ${esc(ago(valueAge(lat, age)))}` : 'No position reported'} · ${points} track points</p>
      <p class="map-caption fine">Not for navigation.</p>
    </div>`;
}

// ---------------------------------------------------------------- history
export function renderHistory(metric: string, hours: number, series: Series | null, loading: boolean, available: Set<string>,
  keys: Record<string, TelemetryKey>): string {
  const metrics = METRICS.filter(m => keys[m.key] && available.has(m.key));
  const selected = metrics.find(m => m.key === metric) ?? metrics[0];
  if (!selected) return `<section class="card empty-state"><p class="eyebrow">History</p><h2>No history yet.</h2><p class="note">History builds up from the boat data SKAGER shares, one sample a minute for 30 days.</p></section>`;
  const points = clean(series?.series[selected.key]);
  const s = stats(points);
  const unit = keys[selected.key].unit;
  const fmt = (v: number) => formatReading(selected.key, v, unit);
  const W = 340, H = 170;
  const now = Date.now() / 1000;
  const domain = points.length ? { t0: now - hours * 3600, t1: now, ...niceDomain(points) } : null;
  const path = domain && series ? chartPath(points, series.bucketSeconds, W, H, domain) : '';
  const used = selected.key === 'battery.net_discharge' && series ? energy(points, series.bucketSeconds) : null;
  return `<div class="chips scroll" role="tablist" aria-label="Measurement">${metrics.map(m => `<button type="button" class="chip${m.key === selected.key ? ' active' : ''}" data-metric="${m.key}">${esc(m.label)}</button>`).join('')}</div>
    <div class="segmented" role="group" aria-label="Period">${RANGES.map(r => `<button type="button" class="${r.hours === hours ? 'active' : ''}" data-hours="${r.hours}">${r.label}</button>`).join('')}</div>
    <section class="card chart-card${loading ? ' loading' : ''}">
      <p class="eyebrow">${esc(selected.label)}</p>
      ${s ? `<p class="chart-now">${esc(fmt(s.last))}<span class="note"> latest average</span></p>` : ''}
      ${path ? `<svg class="chart" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" role="img" aria-label="${esc(selected.label)} over ${RANGES.find(r => r.hours === hours)?.label}">
        <line x1="0" y1="${H}" x2="${W}" y2="${H}" class="axis"/><path d="${path}" class="line"/></svg>
        <div class="chart-axis"><span>${esc(fmt(domain!.y1))}</span><span>${esc(fmt(domain!.y0))}</span></div>
        <div class="chart-times"><span>${RANGES.find(r => r.hours === hours)?.label} ago</span><span>now</span></div>`
        : `<p class="note">${loading ? 'Loading…' : 'No data in this period.'}</p>`}
      ${s ? `<dl class="stats"><div><dt>Min</dt><dd>${esc(fmt(s.min))}</dd></div><div><dt>Average</dt><dd>${esc(fmt(s.mean))}</dd></div><div><dt>Max</dt><dd>${esc(fmt(s.max))}</dd></div></dl>` : ''}
      ${used ? `<p class="energy">Energy used <strong>${used.used.toFixed(1)} kWh</strong>${used.charged > 0.05 ? ` · charged ${used.charged.toFixed(1)} kWh` : ''}<span class="note"> · estimated from ${series!.bucketSeconds >= 3600 ? `${Math.round(series!.bucketSeconds / 3600)}-hour` : `${Math.round(series!.bucketSeconds / 60)}-minute`} averages over ${used.coveredHours.toFixed(1)} h with data</span></p>` : ''}
      <p class="note fine">Gaps mean SKAGER was not sharing. Values are averages, not every reading.</p>
    </section>`;
}

// ---------------------------------------------------------------- alarm log
const KIND_ICON: Record<string, string> = { anchor_alarm: 'anchor', anchor_alarm_cleared: 'anchor', anchor_armed: 'anchor',
  anchor_disarmed: 'anchor', connection_lost: 'alarms', connection_restored: 'alarms', alert_critical: 'alarms', alert_cleared: 'alarms' };
export function renderAlarms(events: BoatEvent[], more: boolean, loading: boolean, highlight: number | null, multipleBoats: boolean): string {
  if (!events.length) {
    return `<section class="card empty-state"><p class="eyebrow">Alarm log</p><h2>${loading ? 'Loading…' : 'Nothing has happened yet.'}</h2>
      <p class="note">Anchor alarms, lost contact and critical alerts from SKAGER on board appear here, newest first, for 90 days.</p></section>`;
  }
  let day = '';
  // Newest first by when it happened; an event reported late still lands on its own day.
  const ordered = [...events].sort((a, b) => Date.parse(b.occurredAt) - Date.parse(a.occurredAt) || b.id - a.id);
  const rows = ordered.map(e => {
    const heading = dayLabel(e.occurredAt);
    const late = Date.parse(e.receivedAt) - Date.parse(e.occurredAt) > 120_000;
    const head = heading !== day ? `<h2 class="section-title">${esc(heading)}</h2>` : '';
    day = heading;
    return `${head}<article class="event ${esc(e.severity)}${e.id === highlight ? ' highlight' : ''}" data-event="${e.id}">
      <span class="event-icon">${icon(KIND_ICON[e.kind] ?? 'alarms')}</span>
      <div><p class="event-title">${esc(e.title)}${multipleBoats ? ` <span class="note">· ${esc(e.boat)}</span>` : ''}</p>
      ${e.detail ? `<p class="note">${esc(e.detail)}</p>` : ''}
      ${typeof e.data?.distance_m === 'number' ? `<p class="note">${esc(formatDistance(e.data.distance_m))} from the anchor${typeof e.data.radius_m === 'number' ? ` · radius ${esc(formatDistance(Math.abs(e.data.radius_m)))}` : ''}</p>` : ''}</div>
      <time datetime="${esc(e.occurredAt)}">${esc(formatTime(e.occurredAt))}${late ? `<span class="note"> reported ${esc(formatTime(e.receivedAt))}</span>` : ''}</time>
    </article>`;
  }).join('');
  return `${rows}${more ? `<button type="button" class="button secondary" id="events-more" ${loading ? 'disabled' : ''}>${loading ? 'Loading…' : 'Show older'}</button>` : ''}`;
}

// ---------------------------------------------------------------- settings
export type SettingsView = { email: string; cloud: Cloud | null; push: string; theme: string; version: string; boats: Device[] };
const PUSH_TEXT: Record<string, string> = {
  granted: 'On. Alarms reach this phone even when Companion is closed.',
  prompt: 'Not yet allowed.',
  denied: 'Blocked in the phone settings. Allow notifications for SKAGER Companion there to receive alarms.',
  unconfigured: 'Not available in this test build (no Firebase/APNs set-up). The alarm log still works.',
  unsupported: 'Not available in the browser preview.',
  failed: 'Registration failed. Try again later.',
};
export function renderSettings(v: SettingsView): string {
  return `<div class="sheet" role="dialog" aria-modal="true" aria-labelledby="settings-title">
    <div class="sheet-head"><h2 id="settings-title">Settings</h2><button type="button" class="icon-button" id="settings-close" aria-label="Close">${icon('close')}</button></div>
    <section class="card"><p class="eyebrow">Account</p><p>${esc(v.email)}</p>
      <p class="note">SKAGER Cloud: ${v.cloud?.active ? 'active' : 'not active'}${v.cloud?.renewsAt && v.cloud.active ? ` · ${v.cloud.cancelAtPeriodEnd ? 'ends' : 'renews'} ${esc(new Date(v.cloud.renewsAt).toLocaleDateString())}` : ''}</p></section>
    <section class="card"><p class="eyebrow">Notifications</p><p class="note">${esc(PUSH_TEXT[v.push] ?? v.push)}</p>
      ${v.push === 'prompt' ? '<button type="button" class="button secondary" id="push-enable">Allow alarm notifications</button>' : ''}</section>
    <section class="card"><p class="eyebrow">Display</p><div class="segmented" role="group" aria-label="Display mode">${[['auto', 'Auto'], ['day', 'Day'], ['dusk', 'Dusk'], ['night', 'Night']].map(([k, l]) => `<button type="button" class="${v.theme === k ? 'active' : ''}" data-theme-choice="${k}">${l}</button>`).join('')}</div></section>
    <section class="card"><p class="eyebrow">Boats</p>${v.boats.length ? v.boats.map(b => `<p>${esc(b.name)} <span class="note">· SKAGER ${esc(b.appVersion)}</span></p>`).join('') : '<p class="note">No boat linked yet.</p>'}
      <p class="note">Boats and phones are managed at beta.skager.app under SKAGER Cloud.</p></section>
    <button type="button" class="button danger" id="sign-out">Sign out of this phone</button>
    <p class="note fine center">SKAGER Companion ${esc(v.version)} · Not for navigation · Map data © OpenStreetMap contributors, OpenMapTiles, OpenFreeMap</p>
  </div>`;
}
