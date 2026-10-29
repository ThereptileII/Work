// SKAGER Helm controller: home, pairing and the live SKAGER screen.
import { connect, HelmError, pairManually, pairWithLink } from './lib/client.ts';
import type { Boat, Connection, ControlMessage, Pilot } from './lib/client.ts';
import { Gestures } from './lib/gestures.ts';
import { formatKeyInput, parsePairingLink } from './lib/protocol.ts';
import type { PairingLink } from './lib/protocol.ts';
import { ScreenView } from './lib/screen.ts';
import { Scanner } from './lib/scanner.ts';
import { deviceName, immersive, keepAwake, native, onBack, onDeepLink, onForeground, store, tick } from './lib/platform.ts';
import type { Settings } from './lib/platform.ts';
import { APP_VERSION, PREVIEW, RECONNECT_DELAYS } from './lib/config.ts';
import { resolveTheme } from '../../../companion/src/theme.ts';
import { quality, renderBoats, renderHelp, renderKeyboard, renderManual, renderMenu, renderOverlay, renderPairing,
  renderBarHandle, renderPilotInfo, renderRotateHint, renderScan, renderSettings, renderToolbar, renderWelcome } from './views.ts';
import type { OverlayState, Stats } from './views.ts';

type Phase = 'starting' | 'home' | 'scan' | 'manual' | 'pairing' | 'helm';
type Sheet = 'settings' | 'help' | 'pilot' | null;

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;
const app = $('#app'), page = $('#page'), stage = $('#stage'), host = $('#screen-host'), canvas = $<HTMLCanvasElement>('#screen');
const toolbar = $('#toolbar'), overlay = $('#overlay'), menuSlot = $('#menu-slot'), keyboardSlot = $('#keyboard-slot');
const sheet = $('#sheet'), toast = $('#toast'), touches = $('#touches'), hint = $('#hint'), barHandle = $('#bar-handle');

const state = {
  phase: 'starting' as Phase,
  boats: [] as Boat[],
  settings: null as unknown as Settings,
  homeBusy: null as string | null,
  homeMessage: '',
  manual: { address: '', key: '', busy: false, error: '' },
  pairing: { name: '', done: false, error: '' },
  current: null as Boat | null,
  conn: null as Connection | null,
  connecting: false,
  overlay: null as OverlayState | null,
  attempt: 0,
  retry: undefined as ReturnType<typeof setTimeout> | undefined,
  resume: false,
  stats: { fps: null, kbps: null, rtt: null, width: 0, height: 0 } as Stats,
  pilot: null as Pilot | null,
  keyboard: false,
  magnifier: false,
  menu: false,
  bar: true,
  sheet: null as Sheet,
  screenAvailable: true,
};

let scanner: Scanner | null = null;
let toastTimer: ReturnType<typeof setTimeout> | undefined;
let barTimer: ReturnType<typeof setTimeout> | undefined;
const BAR_HIDE_MS = 4000;

// ---------------------------------------------------------------- theme
const dark = window.matchMedia('(prefers-color-scheme: dark)');
function applyTheme(): void {
  const theme = resolveTheme(state.settings?.theme ?? 'auto', dark.matches);
  document.documentElement.dataset.theme = theme;
  document.querySelector('meta[name="theme-color"]')?.setAttribute('content', getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#152326');
}
dark.addEventListener('change', applyTheme);

function showToast(text: string): void {
  toast.textContent = text;
  toast.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.hidden = true; }, 3200);
}

// ---------------------------------------------------------------- rendering
function render(): void {
  app.dataset.phase = state.phase;
  if (state.phase !== 'helm') {
    stage.hidden = true;
    page.hidden = false;
    if (state.phase === 'home') {
      page.innerHTML = state.boats.length ? renderBoats(state.boats, state.homeBusy, state.homeMessage) : renderWelcome(state.homeMessage);
    } else if (state.phase === 'manual') {
      page.innerHTML = renderManual(state.manual);
    } else if (state.phase === 'pairing') {
      page.innerHTML = renderPairing(state.pairing.name, state.pairing.done, state.pairing.error);
    } else if (state.phase === 'starting') {
      page.innerHTML = '<div class="spinner" aria-label="Loading"></div>';
    }
  } else {
    page.hidden = true;
    stage.hidden = false;
    renderHelm();
  }
  renderSheet();
}

function renderHelm(): void {
  const name = state.current?.name ?? 'SKAGER';
  const live = !!state.conn?.live;
  stage.dataset.orientation = window.innerWidth > window.innerHeight ? 'landscape' : 'portrait';
  const q = live ? quality(state.stats.rtt) : 'none';
  toolbar.innerHTML = renderToolbar({ name, quality: q, rtt: live ? state.stats.rtt : null,
    pilot: state.pilot, keyboard: state.keyboard, magnifier: state.magnifier, menu: state.menu, live });
  const bar = state.bar || barPinned();
  stage.dataset.bar = bar ? 'open' : 'closed';
  toolbar.inert = !bar && stage.dataset.orientation === 'landscape';
  barHandle.innerHTML = renderBarHandle(q);
  barHandle.hidden = bar;
  menuSlot.innerHTML = state.menu && state.current ? renderMenu(name, state.conn?.address ?? '', state.stats) : '';
  const shown: OverlayState | null = state.overlay ?? (!state.screenAvailable && live ? { kind: 'screen', name } : null);
  overlay.innerHTML = shown ? renderOverlay(shown) : '';
  overlay.dataset.kind = shown?.kind ?? '';
  stage.classList.toggle('dimmed', !live);
  stage.classList.toggle('typing', state.keyboard);
  // A wide SKAGER screen on an upright phone leaves room: suggest turning the phone.
  const room = host.clientHeight - screen.rect.h;
  hint.hidden = !(live && screen.hasFrame && stage.dataset.orientation === 'portrait' && room > 140);
  if (!hint.hidden && !hint.firstChild) hint.innerHTML = renderRotateHint();
  if (state.keyboard && !keyboardSlot.firstElementChild) {
    keyboardSlot.innerHTML = renderKeyboard();
    $<HTMLInputElement>('#keyboard-text').focus();
  } else if (!state.keyboard && keyboardSlot.firstElementChild) {
    keyboardSlot.innerHTML = '';
  }
}

// ---------------------------------------------------------------- the toolbar
// In landscape the toolbar slides over the left edge of the screen and hides
// itself, so SKAGER gets the whole width; a tab at the edge brings it back.
// It stays while something it opened is showing or the screen is not live.
function barPinned(): boolean {
  return state.menu || !!state.sheet || !state.conn?.live || !!state.overlay;
}

function showBar(): void {
  state.bar = true;
  clearTimeout(barTimer);
  barTimer = setTimeout(hideBar, BAR_HIDE_MS);
}

function hideBar(): void {
  clearTimeout(barTimer);
  if (barPinned()) { barTimer = setTimeout(hideBar, BAR_HIDE_MS); return; }
  if (!state.bar) return;
  state.bar = false;
  if (state.phase === 'helm') renderHelm();
}

function renderSheet(): void {
  if (!state.sheet) { sheet.innerHTML = ''; return; }
  sheet.innerHTML = state.sheet === 'settings' ? renderSettings(state.boats, state.settings, APP_VERSION, state.conn?.live ? state.current?.serverId ?? null : null)
    : state.sheet === 'help' ? renderHelp() : renderPilotInfo(state.pilot);
}

function go(phase: Phase): void {
  if (phase !== 'scan') stopScanner();
  state.phase = phase;
  render();
  if (phase === 'scan') startScanner();
}

// ---------------------------------------------------------------- pairing
async function startScanner(): Promise<void> {
  page.innerHTML = renderScan();
  const video = $<HTMLVideoElement>('#camera');
  scanner = new Scanner(video);
  try {
    await scanner.start(text => {
      const link = parsePairingLink(text);
      if (link) { stopScanner(); pairFromLink(link); return; }
      showToast('That is not a SKAGER Helm code.');
      stopScanner();
      setTimeout(() => { if (state.phase === 'scan') startScanner(); }, 1200);
    });
  } catch {
    stopScanner();
    if (state.phase === 'scan') page.innerHTML = renderScan('The camera is not available. Allow camera access for SKAGER Helm in the phone settings, or enter the key instead.');
  }
}

function stopScanner(): void {
  scanner?.stop();
  scanner = null;
}

async function remember(boat: Boat, key: Uint8Array): Promise<void> {
  await store.setKey(boat.serverId, key);
  state.boats = [boat, ...state.boats.filter(b => b.serverId !== boat.serverId)];
  await store.saveBoats(state.boats);
}

async function pairFromLink(link: PairingLink): Promise<void> {
  if (state.conn) disconnect(false);
  state.pairing = { name: link.name, done: false, error: '' };
  go('pairing');
  try {
    const { boat, key } = await pairWithLink(link, await deviceName());
    await afterPairing(boat, key);
  } catch (err) {
    state.pairing.error = pairingMessage(err);
    render();
  }
}

async function afterPairing(boat: Boat, key: Uint8Array): Promise<void> {
  await remember(boat, key);
  state.pairing = { name: boat.name, done: true, error: '' };
  render();
  setTimeout(() => openBoat(boat), 700);
}

function pairingMessage(err: unknown): string {
  if (!(err instanceof HelmError)) return 'Something went wrong. Try again.';
  switch (err.code) {
    case 'unreachable': return 'This phone could not reach the SKAGER computer. Connect the phone to the boat\'s Wi-Fi and try again.';
    case 'no_pairing': return 'Pairing is not open on SKAGER, or the code has expired. Tap Pair a phone or tablet on the SKAGER screen again.';
    case 'wrong_key': return 'The pairing key does not match. Check it on the SKAGER screen; it works once and for five minutes.';
    case 'too_many': return 'This SKAGER already has eight phones and tablets. Remove one in SKAGER (Settings → SKAGER Helm) first.';
    case 'other_boat': return 'A different SKAGER computer answered. Scan the code again.';
    default: return err.message;
  }
}

async function submitManual(form: HTMLFormElement): Promise<void> {
  const address = (form.elements.namedItem('address') as HTMLInputElement).value;
  const key = (form.elements.namedItem('key') as HTMLInputElement).value;
  state.manual = { address, key, busy: true, error: '' };
  render();
  try {
    const { boat, key: deviceKey } = await pairManually(address, key, await deviceName());
    state.manual = { address: '', key: '', busy: false, error: '' };
    state.pairing = { name: boat.name, done: true, error: '' };
    state.phase = 'pairing';
    await afterPairing(boat, deviceKey);
  } catch (err) {
    state.manual = { address, key, busy: false, error: pairingMessage(err) };
    render();
  }
}

// ---------------------------------------------------------------- the live screen
const screen: ScreenView = new ScreenView(host, canvas,
  info => {
    state.conn?.sendControl({ type: 'ack', frame: info.frame });
    if (info.width !== state.stats.width) { state.stats.width = info.width; state.stats.height = info.height; }
    if (state.overlay?.kind === 'connecting') { state.overlay = null; renderHelm(); }
  },
  frame => {
    state.conn?.sendControl({ type: 'ack', frame });
    state.conn?.sendControl({ type: 'refresh' });
  });

const gestures = new Gestures(screen.map, ev => {
  const conn = state.conn;
  if (!conn?.live) return;
  if (!conn.send(ev) && ev.type === 'pointer_down') showToast('Touch not sent: the connection is busy.');
  if (ev.type === 'pointer_down') ripple(ev.x, ev.y, ev.button === 2);
}, v => screen.magnify(v.scale, v.centre, v.dx, v.dy), () => tick());

function ripple(x: number, y: number, secondary: boolean): void {
  const r = screen.rect, box = host.getBoundingClientRect(), stageBox = stage.getBoundingClientRect();
  const dot = document.createElement('span');
  dot.className = `ripple${secondary ? ' secondary' : ''}`;
  dot.style.left = `${box.left - stageBox.left + r.x + x * r.w}px`;
  dot.style.top = `${box.top - stageBox.top + r.y + y * r.h}px`;
  touches.append(dot);
  setTimeout(() => dot.remove(), 650);
}

host.addEventListener('pointerdown', e => {
  if (!state.conn?.live || state.overlay) return;
  if (state.menu) { state.menu = false; renderHelm(); }
  if (state.bar) hideBar();
  host.setPointerCapture(e.pointerId);
  if (e.pointerType === 'mouse' && e.button === 2) {
    const at = screen.map({ x: e.clientX, y: e.clientY });
    if (at) { state.conn.send({ type: 'pointer_down', x: at.x, y: at.y, button: 2 }); state.conn.send({ type: 'pointer_up', x: at.x, y: at.y, button: 2 }); }
    return;
  }
  gestures.down(e.pointerId, { x: e.clientX, y: e.clientY }, e.timeStamp);
});
host.addEventListener('pointermove', e => gestures.move(e.pointerId, { x: e.clientX, y: e.clientY }, e.timeStamp));
host.addEventListener('pointerup', e => gestures.up(e.pointerId, { x: e.clientX, y: e.clientY }, e.timeStamp));
host.addEventListener('pointercancel', () => gestures.cancel());
host.addEventListener('contextmenu', e => e.preventDefault());
host.addEventListener('wheel', e => {
  e.preventDefault();
  const at = screen.map({ x: e.clientX, y: e.clientY });
  if (at && state.conn?.live) state.conn.send({ type: 'wheel', x: at.x, y: at.y, delta: e.deltaY < 0 ? 1 : -1 });
}, { passive: false });
window.addEventListener('resize', () => { if (state.phase === 'helm') renderHelm(); });
toolbar.addEventListener('pointerdown', () => showBar());

function onControl(msg: ControlMessage): void {
  switch (msg.type) {
    case 'hello':
      state.stats.software = msg.software;
      if (msg.pilot) state.pilot = msg.pilot;
      if (state.current && msg.name && msg.name !== state.current.name) {
        state.current = { ...state.current, name: msg.name };
        state.boats = state.boats.map(b => b.serverId === state.current!.serverId ? { ...b, name: msg.name } : b);
        store.saveBoats(state.boats);
      }
      break;
    case 'stats':
      state.stats.fps = msg.stats.fps;
      state.stats.kbps = msg.stats.kbps;
      state.stats.rtt = state.conn?.rtt ?? null;
      break;
    case 'pong':
      state.stats.rtt = state.conn?.rtt ?? null;
      break;
    case 'screen':
      state.screenAvailable = msg.available;
      break;
    case 'pilot':
      state.pilot = msg.status;
      if (state.sheet === 'pilot') renderSheet();
      break;
    case 'bye':
      return; // handled by onClosed
  }
  if (state.phase === 'helm') renderHelm();
}

function onClosed(conn: Connection, reason: string): void {
  if (state.conn !== conn) return;
  state.conn = null;
  gestures.cancel();
  state.keyboard = false;
  state.menu = false;
  const name = state.current?.name ?? 'SKAGER';
  switch (reason) {
    case 'closed_by_user':
      return;
    case 'taken_over':
      state.overlay = { kind: 'taken', name, by: conn.by || 'another phone' };
      break;
    case 'revoked':
      state.overlay = { kind: 'revoked', name };
      break;
    case 'ended_on_skager':
    case 'server_stopped':
      state.overlay = { kind: 'ended', name, reason };
      if (reason === 'server_stopped') scheduleReconnect(5000);
      break;
    default:
      // Lost Wi-Fi, PC asleep, network change: reconnect by itself.
      scheduleReconnect();
  }
  if (state.phase === 'helm') renderHelm();
}

function scheduleReconnect(delay?: number): void {
  clearTimeout(state.retry);
  if (!state.current || document.visibilityState === 'hidden') { state.resume = true; return; }
  state.attempt++;
  if (!state.overlay || state.overlay.kind === 'reconnecting') state.overlay = { kind: 'reconnecting', name: state.current.name, attempt: state.attempt };
  const wait = delay ?? RECONNECT_DELAYS[Math.min(state.attempt - 1, RECONNECT_DELAYS.length - 1)];
  state.retry = setTimeout(() => { if (state.current && !state.conn) openBoat(state.current, { reconnect: true }); }, wait);
}

async function openBoat(boat: Boat, opts: { takeover?: boolean; reconnect?: boolean } = {}): Promise<void> {
  if (state.connecting) return;
  clearTimeout(state.retry);
  const key = await store.key(boat.serverId);
  if (!key) {
    state.current = boat;
    state.overlay = { kind: 'revoked', name: boat.name };
    go('helm');
    return;
  }
  if (state.current?.serverId !== boat.serverId) { screen.clear(); state.pilot = null; state.attempt = 0; }
  state.current = boat;
  state.screenAvailable = true;
  if (!opts.reconnect || !screen.hasFrame) state.overlay = { kind: 'connecting', name: boat.name };
  if (state.phase !== 'helm') go('helm'); else renderHelm();
  state.connecting = true;
  try {
    const conn = await connect(boat, key, {
      screen: data => screen.handle(data),
      control: onControl,
      inputAck: (_seq, accepted, reason) => { if (!accepted && reason === 'stale') showToast('A touch arrived late and was not used. Try again.'); },
      closed: reason => onClosed(conn, reason),
    }, !!opts.takeover);
    state.conn = conn;
    state.attempt = 0;
    showBar();
    if (!screen.hasFrame) state.overlay = { kind: 'connecting', name: boat.name };
    else state.overlay = null;
    const updated: Boat = { ...boat, lastAddress: conn.address, lastConnected: new Date().toISOString() };
    state.current = updated;
    state.boats = [updated, ...state.boats.filter(b => b.serverId !== boat.serverId)];
    store.saveBoats(state.boats);
    if (state.settings.keepAwake) keepAwake(true);
    immersive(true);
    if (!state.settings.seenHelp) { state.settings.seenHelp = true; store.saveSettings(state.settings); state.sheet = 'help'; renderSheet(); }
  } catch (err) {
    const name = boat.name;
    if (err instanceof HelmError) {
      if (err.code === 'busy') state.overlay = { kind: 'busy', name, by: err.by ?? 'Another phone' };
      else if (err.code === 'unpaired') state.overlay = { kind: 'revoked', name };
      else if (opts.reconnect && (err.code === 'unreachable' || err.code === 'webrtc' || err.code === 'expired')) {
        state.connecting = false;
        scheduleReconnect();
        renderHelm();
        return;
      } else if (err.code === 'unreachable') state.overlay = { kind: 'unreachable', name };
      else state.overlay = { kind: 'error', name, message: err.message };
    } else {
      state.overlay = { kind: 'error', name, message: 'Something went wrong while connecting.' };
    }
  } finally {
    state.connecting = false;
  }
  renderHelm();
}

function disconnect(toHome = true): void {
  clearTimeout(state.retry);
  const conn = state.conn;
  state.conn = null;
  conn?.close();
  gestures.cancel();
  state.overlay = null;
  state.keyboard = state.menu = state.magnifier = false;
  state.bar = true;
  clearTimeout(barTimer);
  gestures.magnifier = false;
  screen.resetView();
  keepAwake(false);
  immersive(false);
  if (toHome) { state.current = null; screen.clear(); go('home'); }
}

// ---------------------------------------------------------------- events
document.addEventListener('click', async e => {
  const target = (e.target as HTMLElement).closest<HTMLElement>('[data-action], [data-key], [data-theme-choice]');
  if (!target) return;
  if (target.dataset.themeChoice) {
    state.settings.theme = target.dataset.themeChoice as Settings['theme'];
    await store.saveSettings(state.settings);
    applyTheme();
    renderSheet();
    return;
  }
  if (target.dataset.key) {
    state.conn?.send({ type: 'key', key: target.dataset.key });
    $<HTMLInputElement>('#keyboard-text')?.focus();
    return;
  }
  const boat = target.dataset.boat ? state.boats.find(b => b.serverId === target.dataset.boat) : undefined;
  switch (target.dataset.action) {
    case 'scan': state.sheet = null; disconnect(false); state.current = null; go('scan'); break;
    case 'manual': state.manual.error = ''; go('manual'); break;
    case 'home': go('home'); break;
    case 'settings': state.menu = false; state.sheet = 'settings'; render(); break;
    case 'help': state.menu = false; state.sheet = 'help'; render(); break;
    case 'pilot': state.sheet = 'pilot'; renderSheet(); break;
    case 'close-sheet': state.sheet = null; renderSheet(); break;
    case 'connect': if (boat) { state.homeMessage = ''; openBoat(boat); } break;
    case 'retry': if (state.current) { state.attempt = 0; openBoat(state.current); } break;
    case 'takeover': if (state.current) openBoat(state.current, { takeover: true }); break;
    case 'leave': case 'disconnect': disconnect(); break;
    case 'menu': state.menu = !state.menu; renderHelm(); break;
    case 'bar': showBar(); renderHelm(); break;
    case 'keyboard': state.keyboard = !state.keyboard; state.menu = false; renderHelm(); break;
    case 'magnifier':
      state.magnifier = gestures.magnifier = !state.magnifier;
      if (!state.magnifier) screen.resetView();
      showToast(state.magnifier ? 'Magnifier on: pinch with two fingers to enlarge this view.' : 'Magnifier off.');
      renderHelm();
      break;
    case 'refresh': state.menu = false; state.conn?.sendControl({ type: 'refresh' }); renderHelm(); break;
    case 'forget':
      if (boat && target.dataset.confirm !== '1') { target.dataset.confirm = '1'; target.textContent = 'Tap to confirm'; break; }
      if (boat) await forget(boat.serverId);
      break;
    case 'forget-current':
      if (state.current) await forget(state.current.serverId);
      break;
  }
});

async function forget(serverId: string): Promise<void> {
  if (state.current?.serverId === serverId) disconnect(false);
  await store.forget(serverId);
  state.boats = state.boats.filter(b => b.serverId !== serverId);
  state.sheet = state.sheet === 'settings' && state.boats.length ? 'settings' : null;
  state.current = null;
  go('home');
}

document.addEventListener('change', async e => {
  const input = e.target as HTMLInputElement;
  const key = input.dataset.setting as 'keepAwake' | 'autoConnect' | undefined;
  if (!key) return;
  state.settings[key] = input.checked;
  await store.saveSettings(state.settings);
  if (key === 'keepAwake' && state.conn) keepAwake(input.checked);
});

document.addEventListener('input', e => {
  const input = e.target as HTMLInputElement;
  if (input.id === 'manual-key') {
    const caret = input.selectionStart === input.value.length;
    input.value = formatKeyInput(input.value);
    if (caret) input.setSelectionRange(input.value.length, input.value.length);
  }
});

document.addEventListener('submit', e => {
  const form = e.target as HTMLFormElement;
  e.preventDefault();
  if (form.id === 'manual-form') submitManual(form);
  if (form.id === 'keyboard-form') {
    const input = $<HTMLInputElement>('#keyboard-text');
    const text = input.value.replace(/[\u0000-\u001f\u007f-\u009f]/g, '');
    if (text && state.conn?.send({ type: 'text', text })) input.value = '';
    input.focus();
  }
});

// Keep the soft keyboard open while tapping the key buttons.
keyboardSlot.addEventListener('pointerdown', e => { if ((e.target as HTMLElement).closest('.key')) e.preventDefault(); });

onBack(() => {
  if (state.sheet) { state.sheet = null; renderSheet(); }
  else if (state.menu) { state.menu = false; renderHelm(); }
  else if (state.keyboard) { state.keyboard = false; renderHelm(); }
  else if (state.phase === 'helm') disconnect();
  else if (state.phase !== 'home') go('home');
});

onForeground(active => {
  if (!active && state.conn) {
    // Nothing runs in the background: end the session, resume on return.
    state.resume = true;
    disconnect(false);
  } else if (active && state.resume && state.current) {
    state.resume = false;
    openBoat(state.current, { reconnect: true });
  }
});

function handleLink(url: string): void {
  const link = parsePairingLink(url);
  if (link) pairFromLink(link);
  else { state.homeMessage = 'That link is not a SKAGER Helm pairing code.'; go('home'); }
}

// ---------------------------------------------------------------- start
async function start(): Promise<void> {
  state.settings = await store.settings();
  applyTheme();
  state.boats = await store.boats();
  onDeepLink(handleLink);
  if (!native() && PREVIEW && location.hash.startsWith('#pair=')) {
    const link = decodeURIComponent(location.hash.slice(6));
    history.replaceState(null, '', location.pathname);
    state.phase = 'home';
    handleLink(link);
    return;
  }
  go('home');
  const last = state.boats[0];
  if (last && state.settings.autoConnect) openBoat(last);
}

start();
