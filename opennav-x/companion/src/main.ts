// SKAGER Companion controller: sign-in, foreground-only refresh, offline cache,
// notifications and navigation. Rendering is in views.ts, the map in map.ts.
import { Api, OfflineError, SignedOutError } from './lib/api.ts';
import type { Boat, BoatEvent, Boats, Cloud, Series, TrackPoint } from './lib/api.ts';
import { APP_VERSION, POLL_BOAT_MS, POLL_LIST_MS, PREVIEW } from './lib/config.ts';
import { anchorView, classify, snapshotAgeNow, valueAge } from './lib/freshness.ts';
import { authorizeUrl, newPending, readCallback } from './lib/pkce.ts';
import type { Pending } from './lib/pkce.ts';
import { cache, closeSignIn, disablePush, enablePush, onDeepLink, onForeground, openSignIn, phoneName, platform,
  pushStatus, secrets } from './lib/platform.ts';
import type { PushStatus } from './lib/platform.ts';
import { METRICS } from './lib/history.ts';
import { BoatMap } from './map.ts';
import { PALETTES, resolveTheme } from './theme.ts';
import type { ThemeChoice } from './theme.ts';
import * as view from './views.ts';
import type { Tab } from './views.ts';

type State = {
  phase: 'starting' | 'signed-out' | 'ready';
  signIn: { busy?: boolean; message?: string; error?: boolean };
  email: string; cloud: Cloud | null; push: PushStatus | 'failed';
  boats: Boats | null; selected: string | null;
  boat: Boat | null; etag: string | null; receivedLocal: number;
  offline: { since: number | null } | null;
  tab: Tab; settings: boolean; theme: ThemeChoice;
  events: BoatEvent[]; eventsMore: boolean; eventsLoading: boolean; seenEvent: number; highlight: number | null;
  track: { key: string; points: TrackPoint[] } | null; trackHours: number;
  history: { metric: string; hours: number; series: Series | null; loading: boolean; key: string };
  toast: string | null;
};
const state: State = {
  phase: 'starting', signIn: {}, email: '', cloud: null, push: 'unconfigured', boats: null, selected: null, boat: null,
  etag: null, receivedLocal: 0, offline: null, tab: 'overview', settings: false, theme: 'auto', events: [], eventsMore: false,
  eventsLoading: false, seenEvent: 0, highlight: null, track: null, trackHours: 24,
  history: { metric: METRICS[0].key, hours: 24, series: null, loading: false, key: '' }, toast: null,
};
const api = new Api(secrets);
const map = new BoatMap();
const $ = (id: string) => document.getElementById(id)!;
const dark = matchMedia('(prefers-color-scheme: dark)');
let active = document.visibilityState === 'visible';

// ---------------------------------------------------------------- rendering
const theme = () => resolveTheme(state.theme, dark.matches);
function applyTheme(): void {
  const name = theme();
  document.documentElement.dataset.theme = name;
  document.querySelector('meta[name="theme-color"]')?.setAttribute('content', PALETTES[name].background);
}
const device = () => state.boats?.devices.find(d => d.id === state.selected);
const age = () => state.boat ? snapshotAgeNow(state.boat, state.receivedLocal, Date.now()) : null;
const unread = () => state.events.filter(e => e.id > state.seenEvent && e.severity !== 'info').length;

function render(): void {
  applyTheme();
  const app = $('app');
  app.dataset.phase = state.phase;
  if (state.phase !== 'ready') {
    $('view').innerHTML = state.phase === 'signed-out' ? view.renderSignIn(state.signIn) : '<div class="spinner" aria-label="Loading"></div>';
    $('tabs').innerHTML = ''; $('banner').innerHTML = ''; $('top-title').textContent = ''; $('sheet').innerHTML = '';
    return;
  }
  const a = age();
  const anchor = state.boat?.anchor ?? device()?.anchor ?? null;
  if (state.tab === 'alarms' && state.events.length) {
    const newest = Math.max(...state.events.map(e => e.id));
    if (newest > state.seenEvent) { state.seenEvent = newest; void cache.write('seen', newest); }
  }
  $('top-title').textContent = device()?.name ?? 'SKAGER';
  $('banner').innerHTML = view.renderAlarmBanner(device(), anchor, a);
  const alarm = anchorView(anchor, a).state === 'alarm';
  $('tabs').innerHTML = view.renderTabs(state.tab, alarm, unread());
  const scroll = $('view').scrollTop;
  $('view').innerHTML = content(a);
  $('view').scrollTop = scroll;
  $('view').dataset.tab = state.tab;
  $('sheet').innerHTML = state.settings ? view.renderSettings({ email: state.email, cloud: state.cloud, push: state.push,
    theme: state.theme, version: APP_VERSION, boats: state.boats?.devices ?? [] }) : '';
  $('toast').textContent = state.toast ?? '';
  $('toast').hidden = !state.toast;
  placeMap(a);
}

function content(a: number | null): string {
  const keys = state.boats?.telemetryKeys ?? {};
  const top = view.renderBoatSwitch(state.boats, state.selected);
  if (state.cloud && !state.cloud.active) return view.renderNoCloud(state.cloud);
  if (state.boats && !state.boats.devices.length) return view.renderNoBoats();
  switch (state.tab) {
    case 'overview': return top + view.renderStatus(state.boat, device(), a, state.offline)
      + (state.push === 'prompt' ? '<section class="card notice"><p>Get the anchor alarm on this phone even when Companion is closed.</p><button type="button" class="button secondary" id="push-enable">Allow notifications</button></section>' : '')
      + view.renderOverview(state.boat, keys, a);
    case 'map': return view.renderMapPanel(state.boat, a, state.trackHours, state.track?.points.length ?? 0);
    case 'anchor': return top + view.renderAnchor(device(), state.boat?.anchor ?? device()?.anchor, a, !!state.track?.points.length);
    case 'history': {
      const available = new Set(Object.keys(state.boat?.items ?? {}).filter(k => METRICS.some(m => m.key === k)));
      return top + view.renderHistory(state.history.metric, state.history.hours, state.history.series, state.history.loading,
        available.size ? available : new Set(METRICS.map(m => m.key)), keys);
    }
    case 'alarms': return view.renderAlarms(state.events, state.eventsMore, state.eventsLoading, state.highlight,
      (state.boats?.devices.length ?? 0) > 1);
  }
}

function placeMap(a: number | null): void {
  const slot = document.querySelector('[data-map-slot]');
  if (!slot) return;
  const boat = state.boat;
  const lat = boat?.items['navigation.latitude'], lon = boat?.items['navigation.longitude'];
  const fresh = classify(lat, valueAge(lat, a));
  const anchor = boat?.anchor ?? null;
  const anchorState = anchorView(anchor, a);
  const onAnchor = slot.getAttribute('data-map-slot') === 'anchor';
  map.attach(slot, theme());
  map.update({ position: lat && lon ? [lon.v, lat.v] : null, current: fresh === 'live' || fresh === 'aging',
    track: state.track?.points ?? [], anchor, alarm: anchorState.state === 'alarm', anchorStale: anchorState.stale },
    `${state.selected}:${state.tab}:${onAnchor ? 6 : state.trackHours}:${state.track ? 1 : 0}`);
}

// ---------------------------------------------------------------- data
const trackHours = () => state.tab === 'anchor' ? 6 : state.trackHours;

async function guarded<T>(work: () => Promise<T>): Promise<T | undefined> {
  try {
    const result = await work();
    if (state.offline) { state.offline = null; }
    return result;
  } catch (error) {
    if (error instanceof SignedOutError) { await signOutLocal('You were signed out. Sign in again to see your boat.'); return undefined; }
    if (error instanceof OfflineError) { state.offline ??= { since: state.receivedLocal || null }; return undefined; }
    toast(error instanceof Error ? error.message : 'Something went wrong.');
    return undefined;
  }
}

async function loadList(): Promise<void> {
  const session = await guarded(() => api.session());
  if (session) { state.email = session.account.email; state.cloud = session.cloud; }
  const boats = await guarded(() => api.boats());
  if (boats) {
    state.boats = boats;
    state.cloud = boats.cloud ?? state.cloud;
    if (!boats.devices.some(d => d.id === state.selected)) { state.selected = boats.devices[0]?.id ?? null; state.etag = null; }
    void cache.write('boats', boats);
  }
  const events = await guarded(() => api.events({ limit: 50 }));
  if (events) {
    state.events = [...events, ...state.events.filter(e => e.id < (events.at(-1)?.id ?? 0))].slice(0, 500);
    state.eventsMore ||= events.length === 50;
    void cache.write('events', state.events.slice(0, 100));
  }
}

async function loadBoat(): Promise<void> {
  if (!state.selected) return;
  const id = state.selected;
  const result = await guarded(() => api.boat(id, state.etag ?? undefined));
  if (!result || id !== state.selected) return;
  if (result.boat) {
    state.boat = result.boat;
    state.receivedLocal = Date.now();
    void cache.write(`boat.${id}`, { boat: result.boat, receivedLocal: state.receivedLocal });
  }
  state.etag = result.etag;
}

async function loadTrack(force = false): Promise<void> {
  if (!state.selected || (state.tab !== 'map' && state.tab !== 'anchor')) return;
  const key = `${state.selected}:${trackHours()}:${Math.floor(Date.now() / POLL_LIST_MS)}`;
  if (!force && state.track?.key === key) return;
  const id = state.selected, hours = trackHours();
  const points = await guarded(() => api.track(id, hours));
  if (points && id === state.selected && hours === trackHours()) state.track = { key, points };
}

async function loadHistory(): Promise<void> {
  if (!state.selected || state.tab !== 'history') return;
  const h = state.history;
  const key = `${state.selected}:${h.metric}:${h.hours}`;
  if (h.key === key && h.series) return;
  h.loading = true; h.key = key; render();
  const series = await guarded(() => api.series(state.selected!, [h.metric], h.hours));
  if (h.key === key) { h.series = series ?? null; h.loading = false; }
}

// ---------------------------------------------------------------- refresh loop (foreground only)
let timer: ReturnType<typeof setTimeout> | null = null;
let lastList = 0;
let failures = 0;
async function tick(): Promise<void> {
  if (timer) { clearTimeout(timer); timer = null; }
  if (!active || state.phase !== 'ready') return;
  const listDue = Date.now() - lastList >= POLL_LIST_MS;
  if (listDue) { await loadList(); lastList = Date.now(); }
  await loadBoat();
  await loadTrack();
  await loadHistory();
  failures = state.offline ? Math.min(failures + 1, 3) : 0;
  render();
  // Offline: 15 → 30 → 60 s. Nothing runs while the app is in the background.
  if (active && state.phase === 'ready') timer = setTimeout(tick, POLL_BOAT_MS * 2 ** Math.max(0, failures - 1));
}
function refreshNow(): void { lastList = 0; void tick(); }

// The age counters and freshness labels move on between data refreshes.
setInterval(() => {
  if (active && state.phase === 'ready' && !state.settings && ['overview', 'anchor', 'map'].includes(state.tab)) render();
}, 5000);

// ---------------------------------------------------------------- sign-in / out
async function beginSignIn(): Promise<void> {
  const pending = newPending();
  await secrets.setPending(pending);
  state.signIn = { busy: true };
  render();
  await openSignIn(await authorizeUrl(pending));
}

async function finishSignIn(url: string): Promise<void> {
  const pending = await secrets.pending() as Pending | null;
  const result = readCallback(url, pending);
  if (!result) return;
  await secrets.clearPending();
  await closeSignIn();
  if ('error' in result) {
    state.signIn = { message: result.error === 'denied' ? 'Sign-in was cancelled.' : 'That sign-in did not work. Please try again.', error: result.error !== 'denied' };
    render();
    return;
  }
  try {
    const os = platform() === 'ios' ? 'ios' : 'android';
    const token = await api.exchange(result.code, pending!.verifier, await phoneName(), os);
    await secrets.setToken(token);
    state.signIn = {};
    await start(true);
  } catch (error) {
    state.signIn = { message: error instanceof OfflineError ? 'No connection. Check the internet and try again.'
      : 'Sign-in could not be completed. Please try again.', error: true };
    render();
  }
}

async function signOutLocal(message?: string): Promise<void> {
  await disablePush();
  await secrets.clearToken();
  await cache.clear();
  Object.assign(state, { phase: 'signed-out', signIn: message ? { message } : {}, boats: null, boat: null, selected: null,
    etag: null, events: [], track: null, settings: false, email: '', cloud: null });
  state.history.series = null; state.history.key = '';
  render();
}

async function signOut(): Promise<void> {
  await guarded(() => api.registerPush(null, null));
  await guarded(() => api.signOut());
  await signOutLocal('Signed out.');
}

// ---------------------------------------------------------------- notifications
const pushHandlers = {
  token: (service: 'fcm' | 'apns', token: string) => { void guarded(() => api.registerPush(service, token)); },
  received: (data: Record<string, string>) => { toastFor(data); refreshNow(); },
  opened: (data: Record<string, string>) => {
    if (data.deviceId && state.boats?.devices.some(d => d.id === data.deviceId)) select(data.deviceId);
    state.tab = data.kind?.startsWith('anchor') ? 'anchor' : 'alarms';
    state.highlight = Number(data.eventId) || null;
    refreshNow();
  },
  failed: () => { state.push = 'failed'; render(); },
};
function toastFor(data: Record<string, string>): void {
  toast(data.kind === 'anchor_alarm' ? 'Anchor alarm' : data.kind === 'connection_lost' ? 'No contact with the boat' : 'New boat event');
}
let toastTimer: ReturnType<typeof setTimeout> | null = null;
function toast(text: string): void {
  state.toast = text;
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { state.toast = null; render(); }, 4000);
  render();
}
async function setUpPush(ask: boolean): Promise<void> {
  state.push = await enablePush(pushHandlers, ask).catch(() => 'failed' as const);
  render();
}

// ---------------------------------------------------------------- navigation
function select(id: string): void {
  if (id === state.selected) return;
  state.selected = id; state.boat = null; state.etag = null; state.track = null; state.history.series = null; state.history.key = '';
  void cache.write('selected', id);
  void (async () => {
    const saved = await cache.read<{ boat: Boat; receivedLocal: number }>(`boat.${id}`);
    if (saved && state.selected === id && !state.boat) { state.boat = saved.boat; state.receivedLocal = saved.receivedLocal; }
    refreshNow();
  })();
  render();
}
function open(tab: Tab): void {
  state.tab = tab;
  if (tab === 'alarms') {
    state.seenEvent = Math.max(state.seenEvent, ...state.events.map(e => e.id));
    void cache.write('seen', state.seenEvent);
  }
  render();
  $('view').scrollTop = 0;
  void (async () => { await loadTrack(true); await loadHistory(); render(); })();
}

document.addEventListener('click', event => {
  const target = (event.target as Element).closest('button, [data-tab]') as HTMLElement | null;
  if (!target) return;
  const d = target.dataset;
  if (target.id === 'sign-in') void beginSignIn();
  else if (d.tab) open(d.tab as Tab);
  else if (d.boat) select(d.boat);
  else if (d.track) { state.trackHours = Number(d.track); state.track = null; void loadTrack(true).then(render); render(); }
  else if (d.metric) { state.history.metric = d.metric; state.history.series = null; void loadHistory().then(render); }
  else if (d.hours) { state.history.hours = Number(d.hours); state.history.series = null; void loadHistory().then(render); }
  else if (d.themeChoice) { state.theme = d.themeChoice as ThemeChoice; void cache.write('theme', state.theme); render(); }
  else if (target.id === 'settings-open') { state.settings = true; render(); }
  else if (target.id === 'settings-close') { state.settings = false; render(); }
  else if (target.id === 'push-enable') void setUpPush(true);
  else if (target.id === 'sign-out') void signOut();
  else if (target.id === 'events-more') void moreEvents();
});
document.addEventListener('keydown', event => { if (event.key === 'Escape' && state.settings) { state.settings = false; render(); } });

async function moreEvents(): Promise<void> {
  const before = state.events.at(-1)?.id;
  state.eventsLoading = true; render();
  const older = await guarded(() => api.events({ before, limit: 50 }));
  state.eventsLoading = false;
  if (older) { state.events.push(...older); state.eventsMore = older.length === 50; }
  render();
}

// ---------------------------------------------------------------- start
async function start(fromSignIn = false): Promise<void> {
  state.phase = 'ready';
  render();
  lastList = 0;
  await tick();
  // Ask for notification permission right after signing in, when the reason is obvious;
  // later starts re-register silently (tokens can change).
  void setUpPush(fromSignIn);
}

async function boot(): Promise<void> {
  state.theme = await cache.read<ThemeChoice>('theme') ?? 'auto';
  dark.addEventListener('change', () => render());
  onDeepLink(url => void finishSignIn(url));
  onForeground(isActive => {
    if (isActive === active) return;
    active = isActive;
    if (active) refreshNow(); else if (timer) { clearTimeout(timer); timer = null; }
  });
  addEventListener('online', () => refreshNow());
  if (PREVIEW) {
    // Browser preview only: a local test server hands over a phone token in the URL fragment.
    const token = /^#token=(skc1_[a-f0-9]{64})$/.exec(location.hash)?.[1];
    if (token) { await secrets.setToken(token); history.replaceState(null, '', location.pathname + location.search); }
    const tab = /[?&]tab=(overview|map|anchor|history|alarms)\b/.exec(location.search)?.[1] as Tab | undefined;
    if (tab) state.tab = tab;
    const look = /[?&]theme=(day|dusk|night)\b/.exec(location.search)?.[1] as ThemeChoice | undefined;
    if (look) state.theme = look;
    if (/[?&]settings=1\b/.test(location.search)) state.settings = true;
  }
  if (!await secrets.token()) { state.phase = 'signed-out'; render(); return; }
  // Show the last saved data at once, then refresh.
  state.boats = await cache.read<Boats>('boats');
  state.events = await cache.read<BoatEvent[]>('events') ?? [];
  state.seenEvent = await cache.read<number>('seen') ?? 0;
  state.selected = await cache.read<string>('selected') ?? state.boats?.devices[0]?.id ?? null;
  const saved = state.selected ? await cache.read<{ boat: Boat; receivedLocal: number }>(`boat.${state.selected}`) : null;
  if (saved) { state.boat = saved.boat; state.receivedLocal = saved.receivedLocal; }
  state.push = await pushStatus().catch(() => 'unconfigured' as const);
  await start();
}
void boot();
