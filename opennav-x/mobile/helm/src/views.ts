// SKAGER Helm screens as HTML-string renderers (tested in Node). Every name that
// comes from a SKAGER PC or the phone is escaped.
import type { Boat, Pilot } from './lib/client.ts';
import type { Settings } from './lib/platform.ts';

export const esc = (s: unknown) => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);

const ICON: Record<string, string> = {
  mark: '<path d="M7 19 12 7l5 12" stroke-width="2.2"/>',
  scan: '<path d="M4 8V5.5A1.5 1.5 0 0 1 5.5 4H8M16 4h2.5A1.5 1.5 0 0 1 20 5.5V8M20 16v2.5a1.5 1.5 0 0 1-1.5 1.5H16M8 20H5.5A1.5 1.5 0 0 1 4 18.5V16"/><path d="M8 8h3v3H8zM13 13h3v3h-3zM13 8h3v3M8 13v3h3"/>',
  key: '<circle cx="8" cy="15" r="4"/><path d="m11 12 8-8M16 7l2 2M14 9l2 2"/>',
  keyboard: '<rect x="3" y="6" width="18" height="12" rx="2"/><path d="M7 10h.01M10 10h.01M13 10h.01M16 10h.01M7 14h10"/>',
  magnify: '<circle cx="10.5" cy="10.5" r="6"/><path d="m15 15 5 5M10.5 8v5M8 10.5h5"/>',
  refresh: '<path d="M20 11a8 8 0 1 0-2.3 5.7"/><path d="M20 5v6h-6"/>',
  power: '<path d="M12 3v8"/><path d="M6.3 6.8a8 8 0 1 0 11.4 0"/>',
  settings: '<circle cx="12" cy="12" r="3"/><path d="M12 2.8v2.4M12 18.8v2.4M2.8 12h2.4M18.8 12h2.4M5.5 5.5l1.7 1.7M16.8 16.8l1.7 1.7M5.5 18.5l1.7-1.7M16.8 7.2l1.7-1.7"/>',
  back: '<path d="m15 6-6 6 6 6"/>',
  chevron: '<path d="m9 6 6 6-6 6"/>',
  close: '<path d="M6 6l12 12M18 6 6 18"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  wifi: '<path d="M2.5 9a14 14 0 0 1 19 0M5.5 12.5a9.5 9.5 0 0 1 13 0M8.5 16a5 5 0 0 1 7 0"/><circle cx="12" cy="19.2" r=".9"/>',
  helm: '<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 3v7M12 14v7M3 12h7M14 12h7"/>',
  help: '<circle cx="12" cy="12" r="9"/><path d="M9.5 9.5a2.5 2.5 0 1 1 3.5 2.3c-.6.3-1 .8-1 1.5V14M12 17h.01"/>',
  menu: '<path d="M5 12h.01M12 12h.01M19 12h.01" stroke-width="3"/>',
  tap: '<circle cx="12" cy="12" r="2.5" fill="currentColor"/><circle cx="12" cy="12" r="6.5" opacity=".5"/>',
  rotate: '<rect x="7" y="3" width="10" height="18" rx="2"/><path d="M20 15a8 8 0 0 1-5 5.5M19 19.5l-4 1 1-4" />',
  drag: '<path d="M5 12h14M15 8l4 4-4 4M9 8l-4 4 4 4"/>',
  hold: '<circle cx="12" cy="12" r="2.5" fill="currentColor"/><circle cx="12" cy="12" r="6" opacity=".6"/><circle cx="12" cy="12" r="9.5" opacity=".3"/>',
  pinch: '<path d="M8 8 4 4M4 4v4M4 4h4M16 16l4 4M20 20v-4M20 20h-4"/>',
  enter: '<path d="M19 6v5a3 3 0 0 1-3 3H6"/><path d="m9 10-4 4 4 4"/>',
  backspace: '<path d="M9 6h10a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H9l-6-6 6-6Z"/><path d="m12 10 4 4M16 10l-4 4"/>',
  tab: '<path d="M4 12h12M12 8l4 4-4 4M20 6v12"/>',
  left: '<path d="M19 12H5M11 6l-6 6 6 6"/>',
  right: '<path d="M5 12h14M13 6l6 6-6 6"/>',
  up: '<path d="M12 19V5M6 11l6-6 6 6"/>',
  down: '<path d="M12 5v14M6 13l6 6 6-6"/>',
  send: '<path d="M4 12 20 4l-6 16-3-7-7-1Z"/>',
};

export const icon = (name: string, label = '') =>
  `<svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" ${label ? `role="img" aria-label="${esc(label)}"` : 'aria-hidden="true"'}>${ICON[name] ?? ''}</svg>`;

const mark = '<svg class="mark" viewBox="0 0 32 32" aria-hidden="true"><path d="M7 25 16 6l9 19" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/></svg>';

export function ago(iso: string | undefined, now = Date.now()): string {
  if (!iso) return 'Not connected yet';
  const s = Math.max(0, (now - Date.parse(iso)) / 1000);
  if (s < 90) return 'Connected just now';
  if (s < 3600) return `Connected ${Math.round(s / 60)} min ago`;
  if (s < 86400 * 1.5) return `Connected ${Math.round(s / 3600)} h ago`;
  return `Connected ${new Date(iso).toLocaleDateString(undefined, { day: 'numeric', month: 'short' })}`;
}

// ---------------------------------------------------------------- home
const STEPS = `<ol class="steps">
  <li><span class="step-n">1</span><span>On the SKAGER screen, open <b>Settings → SKAGER Helm</b>.</span></li>
  <li><span class="step-n">2</span><span>Tap <b>Pair a phone or tablet</b>. A code appears.</span></li>
  <li><span class="step-n">3</span><span>Scan the code with this phone.</span></li>
</ol>`;

export function renderWelcome(message = ''): string {
  return `<section class="welcome">
    <img class="wordmark" src="assets/skager-wordmark.png" alt="SKAGER" width="170" height="54">
    <p class="eyebrow">Helm</p>
    <h1>The SKAGER screen, in your hand.</h1>
    <p class="lead">See and use SKAGER from your phone or tablet anywhere on board: in the cockpit, at the bow, in your bunk.</p>
    ${STEPS}
    <div class="actions">
      <button class="button primary" type="button" data-action="scan">${icon('scan')}<span>Scan pairing code</span></button>
      <button class="button secondary" type="button" data-action="manual">${icon('key')}<span>Enter pairing key</span></button>
    </div>
    ${message ? `<p class="message error" role="status">${esc(message)}</p>` : ''}
    <p class="note">${icon('wifi')}<span>Works over the boat's own Wi-Fi. No internet and no SKAGER account needed.</span></p>
  </section>`;
}

export function renderBoats(boats: Boat[], busy: string | null, message = ''): string {
  return `<header class="bar">${mark}<span class="bar-title">SKAGER Helm</span>
      <button type="button" class="icon-button" data-action="settings" aria-label="Settings">${icon('settings')}</button></header>
    <section class="page">
      <p class="section-title">Your boats</p>
      ${boats.map(b => `<article class="boat-card">
        <div class="boat-main"><span class="boat-mark">${icon('helm')}</span>
          <div><h2>${esc(b.name)}</h2><p class="note">${esc(ago(b.lastConnected))}</p></div></div>
        <button class="button primary" type="button" data-action="connect" data-boat="${esc(b.serverId)}" ${busy ? 'disabled' : ''}>
          ${busy === b.serverId ? '<span class="spinner small" aria-hidden="true"></span><span>Connecting…</span>' : '<span>Connect</span>'}</button>
      </article>`).join('')}
      ${message ? `<p class="message error" role="status">${esc(message)}</p>` : ''}
      <button class="button ghost wide" type="button" data-action="scan">${icon('plus')}<span>Pair another SKAGER</span></button>
      <p class="note center">Your phone needs to be on the same Wi-Fi as the SKAGER computer.</p>
    </section>`;
}

// ---------------------------------------------------------------- pairing
export function renderScan(error = ''): string {
  return `<section class="scan">
    <video id="camera" playsinline muted></video>
    <div class="scan-shade"><div class="viewfinder"><i></i><i></i><i></i><i></i></div></div>
    <header class="scan-top"><button type="button" class="icon-button on-dark" data-action="home" aria-label="Back">${icon('back')}</button></header>
    <div class="scan-bottom">
      <p class="scan-title">Point the camera at the code on the SKAGER screen</p>
      ${error ? `<p class="message error" role="status">${esc(error)}</p>` : '<p class="note">Settings → SKAGER Helm → Pair a phone or tablet</p>'}
      <button class="button secondary" type="button" data-action="manual">${icon('key')}<span>Enter pairing key instead</span></button>
    </div>
  </section>`;
}

export function renderManual(state: { address?: string; key?: string; busy?: boolean; error?: string }): string {
  return `<header class="bar"><button type="button" class="icon-button" data-action="home" aria-label="Back">${icon('back')}</button>
      <span class="bar-title">Enter pairing key</span></header>
    <form class="page form" id="manual-form" novalidate>
      <p class="lead">Use the address and key shown under the code on the SKAGER screen.</p>
      <label class="field"><span>SKAGER address</span>
        <input id="manual-address" name="address" inputmode="decimal" autocomplete="off" autocapitalize="off" spellcheck="false"
          placeholder="192.168.1.20" value="${esc(state.address ?? '')}" required></label>
      <label class="field"><span>Pairing key</span>
        <input id="manual-key" name="key" class="mono" autocomplete="one-time-code" autocapitalize="characters" spellcheck="false"
          placeholder="ABCD-EFGH-JKLM-NPQR" maxlength="19" value="${esc(state.key ?? '')}" required></label>
      ${state.error ? `<p class="message error" role="alert">${esc(state.error)}</p>` : ''}
      <button class="button primary wide" type="submit" ${state.busy ? 'disabled' : ''}>${state.busy ? '<span class="spinner small" aria-hidden="true"></span><span>Pairing…</span>' : 'Pair'}</button>
      <p class="note">The key works once and for five minutes. If it has expired, tap <b>Pair a phone or tablet</b> on SKAGER again.</p>
    </form>`;
}

export function renderPairing(name: string, done: boolean, error = ''): string {
  return `<section class="center-page">
    <div class="pair-badge${done ? ' done' : ''}${error ? ' failed' : ''}">${done ? icon('helm') : error ? icon('close') : '<span class="spinner" aria-hidden="true"></span>'}</div>
    <h1>${done ? `Paired with ${esc(name)}` : error ? 'Pairing did not work' : `Pairing with ${esc(name)}…`}</h1>
    ${error ? `<p class="message error" role="alert">${esc(error)}</p>
      <div class="actions"><button class="button primary" type="button" data-action="scan">Scan again</button>
      <button class="button secondary" type="button" data-action="manual">Enter key</button></div>`
      : `<p class="lead">${done ? 'Opening the SKAGER screen…' : 'Checking the pairing key with SKAGER.'}</p>`}
  </section>`;
}

// ---------------------------------------------------------------- helm view
export type Quality = 'good' | 'fair' | 'poor' | 'none';
/** From the round trip only: a still SKAGER screen rightly sends few frames. */
export function quality(rtt: number | null): Quality {
  if (rtt === null) return 'none';
  if (rtt > 250) return 'poor';
  if (rtt > 90) return 'fair';
  return 'good';
}

const PILOT_LABEL: Record<string, string> = { standby: 'Standby', auto: 'Auto', compass: 'Auto', track: 'Track', wind: 'Wind',
  off: 'Off', unavailable: 'Not available', unknown: 'Unknown' };

export function pilotLabel(p: Pilot | null | undefined): string | null {
  if (!p || (!p.state && !p.mode)) return null;
  const state = PILOT_LABEL[(p.mode || p.state || '').toLowerCase()] ?? (p.mode || p.state || '');
  const heading = typeof p.heading === 'number' && Number.isFinite(p.heading) ? ` ${Math.round(((p.heading % 360) + 360) % 360)}°` : '';
  return `${state}${heading}`;
}

export type ToolbarState = { name: string; quality: Quality; rtt: number | null; pilot: Pilot | null; keyboard: boolean;
  magnifier: boolean; menu: boolean; live: boolean };

export function renderToolbar(s: ToolbarState): string {
  const pilot = pilotLabel(s.pilot);
  const latency = s.rtt === null ? '' : `${Math.round(s.rtt)} ms`;
  return `<button type="button" class="tool brand" data-action="menu" aria-expanded="${s.menu}" aria-label="${esc(s.name)} — connection ${esc(s.quality)}">
      <span class="brand-mark">${mark}<span class="dot q-${s.quality}"></span></span><span class="tool-text"><b>${esc(s.name)}</b><small>${esc(latency || 'Connecting')}</small></span></button>
    <button type="button" class="tool${s.keyboard ? ' on' : ''}" data-action="keyboard" aria-pressed="${s.keyboard}" aria-label="Keyboard" ${s.live ? '' : 'disabled'}>${icon('keyboard')}</button>
    <button type="button" class="tool${s.magnifier ? ' on' : ''}" data-action="magnifier" aria-pressed="${s.magnifier}" aria-label="Magnifier: pinch to enlarge this view">${icon('magnify')}</button>
    ${pilot ? `<button type="button" class="tool pilot" data-action="pilot" aria-label="Autopilot ${esc(pilot)}, status only">${icon('helm')}<span class="tool-text"><small>Pilot</small><b>${esc(pilot)}</b></span></button>` : ''}
    <span class="tool-spacer"></span>
    <button type="button" class="tool" data-action="disconnect" aria-label="Disconnect">${icon('power')}</button>`;
}

export type Stats = { fps: number | null; kbps: number | null; rtt: number | null; width: number; height: number; software?: string };

export function renderMenu(name: string, address: string, stats: Stats): string {
  const v = (n: number | null, unit: string, digits = 0) => n === null ? '—' : `${n.toFixed(digits)} ${unit}`;
  return `<div class="menu" role="dialog" aria-label="Connection">
    <p class="eyebrow">Connected</p><h2>${esc(name)}</h2><p class="note mono">${esc(address)}${stats.software ? ` · Helm ${esc(stats.software)}` : ''}</p>
    <dl class="stats">
      <div><dt>Latency</dt><dd>${v(stats.rtt, 'ms')}</dd></div>
      <div><dt>Frames</dt><dd>${v(stats.fps, '/s', 1)}</dd></div>
      <div><dt>Data</dt><dd>${stats.kbps === null ? '—' : stats.kbps >= 1000 ? `${(stats.kbps / 1000).toFixed(1)} Mbit/s` : `${Math.round(stats.kbps)} kbit/s`}</dd></div>
    </dl>
    ${stats.width ? `<p class="fine">Screen ${stats.width} × ${stats.height}</p>` : ''}
    <div class="menu-actions">
      <button type="button" class="menu-item" data-action="refresh">${icon('refresh')}<span>Redraw the screen</span></button>
      <button type="button" class="menu-item" data-action="help">${icon('help')}<span>Touch gestures</span></button>
      <button type="button" class="menu-item" data-action="settings">${icon('settings')}<span>Settings</span></button>
      <button type="button" class="menu-item danger" data-action="disconnect">${icon('power')}<span>Disconnect</span></button>
    </div>
  </div>`;
}

export function renderKeyboard(): string {
  const key = (k: string, i: string, label: string) => `<button type="button" class="key" data-key="${k}" aria-label="${label}">${icon(i)}</button>`;
  return `<form class="keyboard" id="keyboard-form" autocomplete="off">
    <input id="keyboard-text" enterkeyhint="send" autocapitalize="sentences" spellcheck="false" placeholder="Type, then send to SKAGER" maxlength="256" aria-label="Text to type on SKAGER">
    <button type="submit" class="key send" aria-label="Send text">${icon('send')}</button>
    <div class="keys">${key('Backspace', 'backspace', 'Backspace')}${key('Enter', 'enter', 'Enter')}${key('Tab', 'tab', 'Tab')}
      ${key('ArrowLeft', 'left', 'Left')}${key('ArrowUp', 'up', 'Up')}${key('ArrowDown', 'down', 'Down')}${key('ArrowRight', 'right', 'Right')}
      <button type="button" class="key text" data-key="Escape">Esc</button></div>
  </form>`;
}

export type OverlayState =
  | { kind: 'connecting'; name: string }
  | { kind: 'reconnecting'; name: string; attempt: number }
  | { kind: 'unreachable'; name: string }
  | { kind: 'busy'; name: string; by: string }
  | { kind: 'taken'; name: string; by: string }
  | { kind: 'revoked'; name: string }
  | { kind: 'ended'; name: string; reason: string }
  | { kind: 'screen'; name: string }
  | { kind: 'error'; name: string; message: string };

export function renderOverlay(s: OverlayState): string {
  const card = (cls: string, title: string, body: string, actions: string) =>
    `<div class="overlay-card ${cls}" role="alertdialog" aria-live="assertive"><h2>${title}</h2>${body}<div class="actions">${actions}</div></div>`;
  const back = '<button class="button secondary" type="button" data-action="leave">Back to boats</button>';
  const retry = '<button class="button primary" type="button" data-action="retry">Try again</button>';
  const n = esc(s.name);
  switch (s.kind) {
    case 'connecting':
      return `<div class="overlay-card quiet" role="status"><span class="spinner" aria-hidden="true"></span><h2>Connecting to ${n}…</h2>
        <div class="actions"><button class="button secondary" type="button" data-action="leave">Cancel</button></div></div>`;
    case 'reconnecting':
      return `<div class="lost-banner" role="status"><span class="spinner small" aria-hidden="true"></span>
        <span><b>Connection lost.</b> Reconnecting${s.attempt > 1 ? ` (attempt ${s.attempt})` : ''}… Touches are not sent until ${n} is back.</span>
        <button class="button secondary small" type="button" data-action="leave">Stop</button></div>`;
    case 'unreachable':
      return card('', `Can't reach ${n}`, `<p class="note">Check that this ${'phone'} is on the boat's Wi-Fi, that the SKAGER computer is on, and that SKAGER Helm is switched on in SKAGER (Settings → SKAGER Helm).</p>`, retry + back);
    case 'busy':
      return card('', `${esc(s.by)} is using ${n}`, '<p class="note">One phone or tablet at a time can show SKAGER. Taking over disconnects the other one.</p>',
        '<button class="button primary" type="button" data-action="takeover">Take over</button>' + back);
    case 'taken':
      return card('', `${n} is now on ${esc(s.by)}`, '<p class="note">Another phone or tablet took over SKAGER Helm.</p>',
        '<button class="button primary" type="button" data-action="takeover">Take it back</button>' + back);
    case 'revoked':
      return card('alert', `This phone was removed from ${n}`, '<p class="note">To use it again, pair it from the SKAGER screen: Settings → SKAGER Helm → Pair a phone or tablet.</p>',
        '<button class="button primary" type="button" data-action="scan">Pair again</button><button class="button secondary" type="button" data-action="forget-current">Forget this boat</button>');
    case 'ended':
      return card('', s.reason === 'server_stopped' ? `SKAGER Helm stopped on ${n}` : `Disconnected on the SKAGER screen`,
        `<p class="note">${s.reason === 'server_stopped' ? 'SKAGER was closed or SKAGER Helm was switched off on the computer.' : 'Someone at the SKAGER computer ended this connection.'}</p>`,
        retry + back);
    case 'screen':
      return `<div class="lost-banner advisory" role="status">${icon('help')}<span><b>The SKAGER screen is not available.</b> SKAGER is minimized or the computer is locked. The picture resumes by itself.</span></div>`;
    case 'error':
      return card('alert', `Could not connect to ${n}`, `<p class="note">${esc(s.message)}</p>`, retry + back);
  }
}

export function renderPilotInfo(p: Pilot | null): string {
  return `<div class="sheet"><div class="sheet-head"><h2>Autopilot</h2><button type="button" class="icon-button" data-action="close-sheet" aria-label="Close">${icon('close')}</button></div>
    <p class="status-line">${icon('helm')}<span>${esc(pilotLabel(p) ?? 'No status')}</span></p>
    ${p?.message ? `<p class="note">${esc(p.message)}</p>` : ''}
    <p class="note">SKAGER Helm shows the pilot's status only. Engage, change course and stand by with the pilot controls at the helm: autopilot commands from Helm are refused by SKAGER.</p></div>`;
}

export function renderHelp(): string {
  const row = (i: string, title: string, text: string) => `<li><span class="gesture">${icon(i)}</span><div><b>${title}</b><p class="note">${text}</p></div></li>`;
  return `<div class="sheet"><div class="sheet-head"><h2>Touch gestures</h2><button type="button" class="icon-button" data-action="close-sheet" aria-label="Close">${icon('close')}</button></div>
    <ul class="gestures">
      ${row('tap', 'Tap', 'Taps the SKAGER screen where your finger is.')}
      ${row('drag', 'Drag', 'Moves the chart and sliders, as on the SKAGER screen.')}
      ${row('hold', 'Touch and hold', 'Opens the context menu (right click).')}
      ${row('pinch', 'Pinch', 'Zooms the chart in and out.')}
      ${row('magnify', 'Magnifier', 'With the magnifier on, pinch and drag with two fingers to enlarge this view only. Taps still go to SKAGER.')}
      ${row('keyboard', 'Keyboard', 'Type a waypoint name or a value: tap a field on SKAGER, then type and send.')}
    </ul>
    <button class="button primary wide" type="button" data-action="close-sheet">Got it</button></div>`;
}

export function renderSettings(boats: Boat[], settings: Settings, version: string, connected: string | null): string {
  const seg = (value: Settings['theme'], label: string) =>
    `<button type="button" data-theme-choice="${value}" class="${settings.theme === value ? 'active' : ''}" aria-pressed="${settings.theme === value}">${label}</button>`;
  return `<div class="sheet" role="dialog" aria-label="Settings"><div class="sheet-head"><h2>Settings</h2>
      <button type="button" class="icon-button" data-action="close-sheet" aria-label="Close">${icon('close')}</button></div>
    <p class="section-title">Paired boats</p>
    ${boats.length ? boats.map(b => `<div class="row"><div><b>${esc(b.name)}</b><p class="note">Paired ${esc(new Date(b.pairedAt).toLocaleDateString())}${b.serverId === connected ? ' · connected' : ''}</p></div>
      <button type="button" class="button secondary small" data-action="forget" data-boat="${esc(b.serverId)}">Forget</button></div>`).join('')
      : '<p class="note">No boats yet.</p>'}
    <p class="section-title">Appearance</p>
    <div class="segmented" role="group" aria-label="Theme">${seg('auto', 'Auto')}${seg('day', 'Day')}${seg('dusk', 'Dusk')}${seg('night', 'Night')}</div>
    <p class="fine">This app only. The SKAGER screen keeps the palette chosen on board.</p>
    <p class="section-title">Behaviour</p>
    <label class="row toggle"><span><b>Keep the screen on</b><span class="note">While the SKAGER screen is shown.</span></span>
      <input type="checkbox" data-setting="keepAwake" ${settings.keepAwake ? 'checked' : ''}></label>
    <label class="row toggle"><span><b>Connect on start</b><span class="note">Open the last boat straight away.</span></span>
      <input type="checkbox" data-setting="autoConnect" ${settings.autoConnect ? 'checked' : ''}></label>
    <p class="section-title">About</p>
    <p class="note">SKAGER Helm ${esc(version)}. Helm shows the SKAGER screen over the boat's own network, encrypted end to end between this phone and the SKAGER computer. Nothing is sent over the internet and nothing is stored on this phone except the pairing.</p>
    <p class="note">SKAGER on board stays in charge: the chart, routes and alarms run there. Autopilot commands are only available at the helm.</p>
  </div>`;
}

export const renderRotateHint = () => `${icon('rotate')}<span>Turn your phone sideways to see SKAGER larger.</span>`;
