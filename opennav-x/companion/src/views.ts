// SKAGER Companion screens as pure HTML-string renderers (tested in Node).
// Every value from SKAGER Cloud is escaped. Missing data is shown as missing,
// and every value carries its age.
import type { Alarm, AlarmKey, Anchor, Boat, BoatEvent, Boats, Cloud, Device, Item, TelemetryKey } from './lib/api.ts';
import { anchorView, ago, classify, connection, CONNECTION_LABEL, valueAge } from './lib/freshness.ts';
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
  check: '<path d="m5 12.5 4.5 4.5L19 7.5"/>',
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
    <p class="note">Sign-in opens beta.skager.app. Companion needs SKAGER Cloud, and SKAGER on board linked to your account.</p>
    <p class="note fine">Companion shows what SKAGER reports. It is not for navigation and cannot control the boat.</p>
  </section>`;
}

export function renderNoCloud(cloud: Cloud | null): string {
  return `<section class="card empty-state"><p class="eyebrow">SKAGER Cloud</p><h2>SKAGER Cloud is not active on this account.</h2>
    <p class="note">${cloud?.status === 'past_due' ? 'The last payment did not go through.' : 'Boat data, alarms and the anchor watch on your phone are part of SKAGER Cloud.'} You can manage your account at beta.skager.app.</p></section>`;
}

export function renderNoBoats(): string {
  return `<section class="card empty-state"><p class="eyebrow">No boat yet</p><h2>Link SKAGER on board to your account.</h2>
    <ol class="steps">
      <li>On the boat computer, open SKAGER and go to <strong>Settings → System → SKAGER Cloud</strong>.</li>
      <li>Tap <strong>Link to my account</strong>. A QR code and a pairing code appear.</li>
      <li>Scan the QR code with this phone's camera, sign in and tap <strong>Link to my account</strong>. Or open beta.skager.app/link and type the code.</li>
      <li>Back in SKAGER, switch <strong>Share boat data</strong> on.</li>
    </ol>
    <p class="note">Your boat appears here within a minute.</p></section>`;
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
  const phone = offline ? `<p class="offline-chip" role="status">This phone is offline${offline.since ? ` · saved ${esc(formatTime(new Date(offline.since).toISOString()))}` : ''}</p>` : '';
  if (state === 'online') return `<div class="updated"><p class="note"><span class="dot ok"></span>Updated ${esc(ago(age))}</p>${phone}</div>`;
  const detail = state === 'sharing-off' ? 'Switch on Share boat data in SKAGER on board.'
    : state === 'no-data' ? 'No boat data yet.'
    : state === 'offline' ? `No data for ${ago(age).replace(/ ago$/, '')}. Is SKAGER running and online on board?`
    : `Last update ${ago(age)}.`;
  return `<section class="status-card ${STATUS_CLASS[state]}" role="status">
    <div><p class="status-line"><span class="dot ${STATUS_CLASS[state]}"></span>${CONNECTION_LABEL[state]}</p>
    <p class="note">${esc(detail)}</p></div>${phone}
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
/** The Boat view's tiles, most useful first. Without a saved layout, only what the boat reports is shown. */
export const DEFAULT_LAYOUT = ['navigation.sog', 'wind.true_speed', 'wind.true_angle', 'environment.depth', 'battery.soc',
  'battery.voltage', 'battery.current', 'battery.net_discharge', 'navigation.latitude', 'navigation.cog', 'navigation.heading',
  'navigation.stw', 'wind.apparent_speed', 'wind.apparent_angle', 'propulsion.electrical_power', 'propulsion.rpm',
  'propulsion.motor_temperature', 'propulsion.coolant_temperature', 'propulsion.gear', 'propulsion.shaft_power',
  'propulsion.regeneration', 'environment.water_temperature', 'environment.pressure', 'battery.soh', 'battery.capacity',
  'tanks.fresh_water', 'tanks.fuel', 'tanks.waste', 'tanks.other', 'steering.rudder', 'steering.heel'];
const TILE_LABEL: Record<string, string> = { 'navigation.latitude': 'Position', 'battery.net_discharge': 'Battery power' };
const tileLabel = (key: string, keys: Record<string, TelemetryKey>) => TILE_LABEL[key] ?? keys[key]?.label ?? key;

/** A section divider in a saved layout: 'divider:<id>:<title>'; an empty title draws a plain line. */
export const isDivider = (entry: string) => /^divider:[a-z0-9]{1,16}:/.test(entry);
export const dividerRef = (entry: string) => entry.slice(0, entry.indexOf(':', 8));
export const dividerTitle = (entry: string) => entry.slice(entry.indexOf(':', 8) + 1);
export const DIVIDER_TITLE_MAX = 40;
export const cleanDividerTitle = (title: string) => title.replace(/[\u0000-\u001f\u007f]/g, '').slice(0, DIVIDER_TITLE_MAX);

/** The tiles and dividers to show: the saved layout, or every reported value in the default order. */
export function layoutKeys(layout: string[] | null, items: Record<string, Item>, keys: Record<string, TelemetryKey>): string[] {
  const known = (key: string) => DEFAULT_LAYOUT.includes(key) && !!keys[key];
  return layout ? layout.filter(e => isDivider(e) || known(e)) : DEFAULT_LAYOUT.filter(key => known(key) && !!items[key]);
}

function split(text: string): [string, string] {
  const match = /^(-?[\d.,]+)\s?(.*)$/.exec(text);
  return match ? [match[1], match[2]] : [text, ''];
}
function tile(key: string, items: Record<string, Item>, keys: Record<string, TelemetryKey>, age: number | null): string {
  const spec = keys[key];
  if (!spec) return '';
  const item = items[key];
  const itemAge = valueAge(item, age);
  const fresh: Freshness = classify(item, itemAge);
  const position = key === 'navigation.latitude';
  let value: string, unit = '';
  if (position) {
    const lon = items['navigation.longitude'];
    value = item && lon ? `${esc(formatReading('navigation.latitude', item.v, 'deg'))}<br>${esc(formatReading('navigation.longitude', lon.v, 'deg'))}` : '—';
  } else {
    [value, unit] = item ? split(formatReading(key, item.v, spec.unit)) : ['—', ''];
    value = esc(value);
  }
  const bar = key === 'battery.soc' && item && fresh !== 'unavailable'
    ? `<span class="bar" aria-hidden="true"><span class="fill w${Math.max(0, Math.min(20, Math.round(item.v / 5)))}"></span></span>` : '';
  // Only what needs attention: an old or estimated value. Current values carry no label.
  const flag = fresh === 'stale' ? `Old · ${esc(ago(itemAge))}` : fresh === 'aging' ? esc(ago(itemAge))
    : fresh === 'estimated' ? 'Estimated' : '';
  // Half width unless the value needs the room (position: two lines of coordinates).
  return `<div class="tile ${fresh}${position ? ' wide' : ''}">
    <p class="tile-label">${esc(tileLabel(key, keys))}</p>
    <p class="tile-value${position ? ' position' : ''}">${value}${unit ? `<span class="unit${unit === '°' ? ' deg' : ''}">${esc(unit)}</span>` : ''}</p>${bar}
    ${flag ? `<p class="tile-age">${flag}</p>` : ''}
  </div>`;
}

export function renderOverview(boat: Boat | null, keys: Record<string, TelemetryKey>, age: number | null, layout: string[] | null = null): string {
  const items = boat?.items ?? {};
  // Dividers split the tiles into sections.
  const sections: { title: string | null; keys: string[] }[] = [{ title: null, keys: [] }];
  for (const entry of layoutKeys(layout, items, keys)) {
    if (isDivider(entry)) sections.push({ title: dividerTitle(entry), keys: [] });
    else sections.at(-1)!.keys.push(entry);
  }
  const filled = sections.filter(s => s.keys.length);
  const tiles = filled.length ? filled.map(s => `${s.title === null ? '' : s.title ? `<h2 class="section-title">${esc(s.title)}</h2>` : '<hr class="section-rule">'}
    <div class="tiles">${s.keys.map(key => tile(key, items, keys, age)).join('')}</div>`).join('')
    : `<section class="card empty-state"><h2>No values to show.</h2><p class="note">${boat ? 'Choose what to show with Edit dashboard.' : 'Waiting for boat data.'}</p></section>`;
  return `${tiles}<button type="button" class="button secondary wide-button" id="layout-edit">Edit dashboard</button>`;
}

/** Choose and order the Boat view's tiles. Saved on this phone only. */
export function renderLayoutEditor(shown: string[], items: Record<string, Item>, keys: Record<string, TelemetryKey>): string {
  const rest = DEFAULT_LAYOUT.filter(key => keys[key] && !shown.includes(key));
  const missing = (key: string) => (key === 'navigation.latitude' ? !items[key] || !items['navigation.longitude'] : !items[key])
    ? '<span class="note"> · not reported</span>' : '';
  const moves = (ref: string, name: string, i: number) => `
      <button type="button" class="icon-button small" data-layout-move="${esc(ref)}:-1" aria-label="Move ${esc(name)} up" ${i === 0 ? 'disabled' : ''}>↑</button>
      <button type="button" class="icon-button small" data-layout-move="${esc(ref)}:1" aria-label="Move ${esc(name)} down" ${i === shown.length - 1 ? 'disabled' : ''}>↓</button>`;
  const row = (key: string, i: number) => isDivider(key) ? `<li class="layout-row divider-row">
      <input class="divider-input" data-divider="${esc(dividerRef(key))}" value="${esc(dividerTitle(key))}" maxlength="${DIVIDER_TITLE_MAX}"
        placeholder="Section title (optional)" aria-label="Section title" autocomplete="off" enterkeyhint="done">${moves(dividerRef(key), 'divider', i)}
      <button type="button" class="chip" data-layout-remove="${esc(dividerRef(key))}">Remove</button></li>`
    : `<li class="layout-row">
      <span class="layout-name">${esc(tileLabel(key, keys))}${missing(key)}</span>${moves(key, tileLabel(key, keys), i)}
      <button type="button" class="chip" data-layout-toggle="${esc(key)}">Hide</button></li>`;
  return `<div class="editor-head"><h2>Edit dashboard</h2><button type="button" class="button primary" id="layout-done">Done</button></div>
    <p class="note">Saved on this phone.</p>
    <h3 class="section-title">Shown</h3>
    ${shown.length ? `<ol class="layout-list">${shown.map(row).join('')}</ol>` : '<p class="note">Nothing yet. Add values below.</p>'}
    <button type="button" class="button secondary wide-button" id="layout-divider">Add section divider</button>
    <p class="note">A divider starts a new section, with a title or as a plain line. Move it with the arrows.</p>
    ${rest.length ? `<h3 class="section-title">More</h3><ul class="layout-list">${rest.map(key => `<li class="layout-row">
      <span class="layout-name">${esc(tileLabel(key, keys))}${missing(key)}</span>
      <button type="button" class="chip" data-layout-toggle="${esc(key)}">Show</button></li>`).join('')}</ul>` : ''}
    <button type="button" class="button secondary wide-button" id="layout-reset">Reset to default</button>`;
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
/** The values History can show: what the boat reports and SKAGER Cloud knows. */
export const historyMetrics = (available: Set<string>, keys: Record<string, TelemetryKey>) =>
  METRICS.filter(m => keys[m.key] && available.has(m.key));

export function renderHistory(metric: string, hours: number, series: Series | null, loading: boolean, available: Set<string>,
  keys: Record<string, TelemetryKey>): string {
  const metrics = historyMetrics(available, keys);
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
  // The heading is the chooser: what the graph shows, and the way to change it.
  // A sheet, not a native <select>: Android closes its picker when the screen refreshes.
  return `<h2 class="history-title"><button type="button" class="title-select" data-picker="metric" aria-haspopup="dialog"${metrics.length < 2 ? ' disabled' : ''}>${esc(selected.label)}</button></h2>
    <div class="segmented" role="group" aria-label="Period">${RANGES.map(r => `<button type="button" class="${r.hours === hours ? 'active' : ''}" data-hours="${r.hours}">${r.label}</button>`).join('')}</div>
    <section class="card chart-card${loading ? ' loading' : ''}">
      ${s ? `<p class="chart-now">${esc(fmt(s.last))}<span class="note"> latest average</span></p>` : ''}
      ${path ? `<svg class="chart" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" role="img" aria-label="${esc(selected.label)} over ${RANGES.find(r => r.hours === hours)?.label}">
        <line x1="0" y1="${H}" x2="${W}" y2="${H}" class="axis"/><path d="${path}" class="line"/></svg>
        <div class="chart-axis"><span>${esc(fmt(domain!.y1))}</span><span>${esc(fmt(domain!.y0))}</span></div>
        <div class="chart-times"><span>${RANGES.find(r => r.hours === hours)?.label} ago</span><span>now</span></div>`
        : `<p class="note">${loading ? 'Loading…' : 'No data in this period.'}</p>`}
      ${s ? `<dl class="stats"><div><dt>Min</dt><dd>${esc(fmt(s.min))}</dd></div><div><dt>Average</dt><dd>${esc(fmt(s.mean))}</dd></div><div><dt>Max</dt><dd>${esc(fmt(s.max))}</dd></div></dl>` : ''}
      ${used ? `<p class="energy">Energy used <strong>${used.used.toFixed(1)} kWh</strong>${used.charged > 0.05 ? ` · charged ${used.charged.toFixed(1)} kWh` : ''}<span class="note"> · estimated from ${series!.bucketSeconds >= 3600 ? `${Math.round(series!.bucketSeconds / 3600)}-hour` : `${Math.round(series!.bucketSeconds / 60)}-minute`} averages over ${used.coveredHours.toFixed(1)} h with data</span></p>` : ''}
      <p class="note fine">Gaps: no boat data then.</p>
    </section>`;
}

// ---------------------------------------------------------------- alarm log
const KIND_ICON: Record<string, string> = { anchor_alarm: 'anchor', anchor_alarm_cleared: 'anchor', anchor_armed: 'anchor',
  anchor_disarmed: 'anchor', connection_lost: 'alarms', connection_restored: 'alarms', alert_critical: 'alarms', alert_cleared: 'alarms',
  threshold_alarm: 'alarms', threshold_cleared: 'alarms' };
export function renderAlarms(events: BoatEvent[], more: boolean, loading: boolean, highlight: number | null, multipleBoats: boolean): string {
  if (!events.length) {
    return `<h2 class="section-title">Log</h2><p class="note">${loading ? 'Loading…' : 'Nothing has happened yet.'}</p>`;
  }
  let day = '';
  // Newest first by when it happened; an event reported late still lands on its own day.
  const ordered = [...events].sort((a, b) => Date.parse(b.occurredAt) - Date.parse(a.occurredAt) || b.id - a.id);
  const rows = ordered.map(e => {
    const heading = dayLabel(e.occurredAt);
    const late = Date.parse(e.receivedAt) - Date.parse(e.occurredAt) > 120_000;
    const head = heading !== day ? `<h3 class="section-title">${esc(heading)}</h3>` : '';
    day = heading;
    return `${head}<article class="event ${esc(e.severity)}${e.id === highlight ? ' highlight' : ''}" data-event="${e.id}">
      <span class="event-icon">${icon(KIND_ICON[e.kind] ?? 'alarms')}</span>
      <div><p class="event-title">${esc(e.title)}${multipleBoats ? ` <span class="note">· ${esc(e.boat)}</span>` : ''}</p>
      ${e.detail ? `<p class="note">${esc(e.detail)}</p>` : ''}
      ${typeof e.data?.distance_m === 'number' ? `<p class="note">${esc(formatDistance(e.data.distance_m))} from the anchor${typeof e.data.radius_m === 'number' ? ` · radius ${esc(formatDistance(Math.abs(e.data.radius_m)))}` : ''}</p>` : ''}</div>
      <time datetime="${esc(e.occurredAt)}">${esc(formatTime(e.occurredAt))}${late ? `<span class="note"> reported ${esc(formatTime(e.receivedAt))}</span>` : ''}</time>
    </article>`;
  }).join('');
  return `<h2 class="section-title">Log</h2>${rows}${more ? `<button type="button" class="button secondary" id="events-more" ${loading ? 'disabled' : ''}>${loading ? 'Loading…' : 'Show older'}</button>` : ''}`;
}

// ---------------------------------------------------------------- your alarms
export type AlarmForm = { id?: number; key: string; direction: 'above' | 'below'; threshold: string; enabled: boolean; error?: string };
export type AlarmsView = { alarms: Alarm[] | null; keys: Record<string, AlarmKey>; form: AlarmForm | null; saving: boolean };
const RULE_STATE: Record<Alarm['state'], string> = { alarm: 'Alarm', ok: 'OK', unknown: 'Waiting for data' };
const number = (v: number) => String(Math.round(v * 100) / 100);

function alarmForm(f: AlarmForm, keys: Record<string, AlarmKey>, saving: boolean): string {
  const spec = keys[f.key];
  return `<form class="card alarm-form" id="alarm-form" novalidate>
    <h3>${f.id ? 'Change alarm' : 'New alarm'}</h3>
    <div class="field"><span class="note" id="alarm-key-label">Data</span><button type="button" class="select" data-picker="alarm-key"
      aria-haspopup="dialog" aria-describedby="alarm-key-label">${esc(spec?.label ?? 'Choose')}</button></div>
    <div class="segmented" role="group" aria-label="When">${(['below', 'above'] as const).map(d => `<button type="button" class="${f.direction === d ? 'active' : ''}" data-alarm-direction="${d}">${d === 'below' ? 'Below' : 'Above'}</button>`).join('')}</div>
    <label class="field"><span class="note">Value</span><span class="value-input"><input id="alarm-threshold" inputmode="decimal" autocomplete="off" value="${esc(f.threshold)}"${spec ? ` placeholder="${esc(number(spec.min))} to ${esc(number(spec.max))}"` : ''}><span class="unit">${esc(spec?.unit ?? '')}</span></span></label>
    ${f.id ? `<div class="segmented" role="group" aria-label="Alarm">${[true, false].map(on => `<button type="button" class="${f.enabled === on ? 'active' : ''}" data-alarm-enabled="${on}">${on ? 'On' : 'Off'}</button>`).join('')}</div>` : ''}
    ${f.error ? `<p class="message error" role="alert">${esc(f.error)}</p>` : ''}
    <div class="form-actions"><button type="submit" class="button primary" ${saving ? 'disabled' : ''}>${saving ? 'Saving…' : 'Save'}</button>
      <button type="button" class="button secondary" id="alarm-cancel">Cancel</button></div>
    ${f.id ? `<button type="button" class="button danger" id="alarm-delete" ${saving ? 'disabled' : ''}>Delete alarm</button>` : ''}
  </form>`;
}

/** What an alarm can watch, by name. */
export const alarmKeyOptions = (keys: Record<string, AlarmKey>): PickerOption[] =>
  Object.entries(keys).map(([value, k]) => ({ value, label: k.label })).sort((a, b) => a.label.localeCompare(b.label));

export function renderAlarmRules(v: AlarmsView): string {
  if (!v.alarms) return '<h2 class="section-title">Your alarms</h2><p class="note">Loading…</p>';
  const rows = v.alarms.map(a => `<button type="button" class="rule ${a.enabled ? a.state : 'off'}" data-alarm-edit="${a.id}">
      <span><span class="rule-title">${esc(a.label)}</span>
      <span class="note">${a.direction === 'below' ? 'Below' : 'Above'} ${esc(number(a.threshold))}${a.unit ? ` ${esc(a.unit)}` : ''}</span></span>
      <span class="rule-state">${a.enabled ? RULE_STATE[a.state] : 'Off'}</span></button>`).join('');
  return `<h2 class="section-title">Your alarms</h2>
    ${v.form && !v.form.id ? '' : rows || '<p class="note">Get a notification when a value goes above or below a limit you choose.</p>'}
    ${v.form ? alarmForm(v.form, v.keys, v.saving) : `<button type="button" class="button secondary wide-button" id="alarm-add">Add alarm</button>`}`;
}

// ---------------------------------------------------------------- picker
export type PickerOption = { value: string; label: string };
/** A choice list as a bottom sheet (the app's own, so it survives refreshes and looks the same on Android and iOS). */
export function renderPicker(title: string, options: PickerOption[], selected: string): string {
  return `<div class="sheet picker" role="dialog" aria-modal="true" aria-labelledby="picker-title">
    <div class="sheet-head"><h2 id="picker-title">${esc(title)}</h2><button type="button" class="icon-button" id="picker-close" aria-label="Close">${icon('close')}</button></div>
    <div class="picker-list">${options.map(o => `<button type="button" class="picker-option${o.value === selected ? ' active' : ''}" aria-pressed="${o.value === selected}" data-pick="${esc(o.value)}">${esc(o.label)}${o.value === selected ? icon('check') : ''}</button>`).join('')}</div>
  </div>`;
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
