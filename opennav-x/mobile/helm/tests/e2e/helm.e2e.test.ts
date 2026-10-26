// SKAGER Helm end to end: pairing by link, the live screen, touches arriving on
// the PC, keyboard, losing and regaining the PC, takeover and removal.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { browser, buildPreview, inputLines, pairingLink, serveApp, sleep, startHelm, touch, until, screenshot } from './harness.ts';

const appRoot = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const shots = process.env.HELM_SCREENSHOTS; // directory for review screenshots (optional)
const t0 = Date.now();
const step = (name: string) => { if (process.env.HELM_E2E_TRACE) console.error(`[${((Date.now() - t0) / 1000).toFixed(1)}s] ${name}`); };

const phase = `document.querySelector('#app').dataset.phase`;
const overlay = `document.querySelector('#overlay').dataset.kind`;
const live = `${phase} === 'helm' && !${overlay} && document.querySelector('#screen').width === 1280`;
// Canvas centre in page coordinates.
const centre = `(() => { const r = document.querySelector('#screen').getBoundingClientRect(); return { x: r.left + r.width / 2, y: r.top + r.height / 2, w: r.width, h: r.height, left: r.left, top: r.top }; })()`;

test('pair, see the screen, touch, type, reconnect, take over and remove', { timeout: 120000 }, async () => {
  step('build');
  buildPreview(appRoot);
  step('serve');
  const { server, url } = await serveApp(join(appRoot, 'dist'));
  step('helm server');
  let helm = await startHelm();
  step('browser');
  const b = await browser(844, 390);
  try {
    if (shots) {
      await b.send('Emulation.setDeviceMetricsOverride', { width: 390, height: 844, deviceScaleFactor: 3, mobile: true });
      await b.send('Page.navigate', { url });
      step('welcome');
      await until('welcome', () => b.eval(`!!document.querySelector('.welcome')`));
      await sleep(400);
      await screenshot(b, join(shots, '01-welcome.png'));
      await b.send('Emulation.setDeviceMetricsOverride', { width: 844, height: 390, deviceScaleFactor: 2, mobile: true });
    }
    const link = await pairingLink(helm);
    await b.send('Page.navigate', { url: `${url}?scan=1#pair=${encodeURIComponent(link)}` });
    step('paired and live');
    await until('paired and live', () => b.eval(live), 20000);
    assert.equal(helm.events.filter(e => e.event === 'paired').length, 1);
    assert.ok(helm.events.find(e => e.event === 'session' && e.state === 'connected'));
    // The first connection shows the gesture guide once.
    assert.equal(await b.eval(`!!document.querySelector('.gestures')`), true);
    if (shots) await screenshot(b, join(shots, '03-gestures.png'));
    await b.eval(`document.querySelector('[data-action="close-sheet"]').click()`);

    // The picture is the PC's test pattern: water colour in the top-left area.
    const pixel = await b.eval<number[]>(`Array.from(document.querySelector('#screen').getContext('2d').getImageData(100, 100, 1, 1).data)`);
    assert.ok(Math.abs(pixel[0] - 0xd5) < 12 && Math.abs(pixel[1] - 0xe5) < 12, `water pixel ${pixel}`);
    if (shots) await screenshot(b, join(shots, '04-live-landscape.png'));

    // Tap in the middle → a click in the middle of the PC frame.
    const c = await b.eval<{ x: number; y: number; w: number; h: number; left: number; top: number }>(centre);
    await touch(b, [[{ x: c.x, y: c.y }], [{ x: c.x, y: c.y }]]);
    const near = (line: string | undefined, kind: string, x: number, y: number) => {
      const m = line && new RegExp(`^${kind} (\\d+),(\\d+)$`).exec(line);
      return !!m && Math.abs(Number(m[1]) - x) <= 3 && Math.abs(Number(m[2]) - y) <= 3;
    };
    step('tap on the PC');
    await until('tap on the PC', () => {
      const l = inputLines(helm);
      return l.length === 2 && near(l[0], 'primary down', 640, 360) && near(l[1], 'primary up', 640, 360);
    });
    // Drag → press, moves, release.
    const at = (fx: number, fy: number) => ({ x: c.left + c.w * fx, y: c.top + c.h * fy });
    const before = inputLines(helm).length;
    await touch(b, [[at(0.3, 0.5)], [at(0.35, 0.5)], [at(0.45, 0.5)], [at(0.55, 0.5)], [at(0.55, 0.5)]]);
    step('drag on the PC');
    await until('drag on the PC', () => {
      const l = inputLines(helm).slice(before);
      return near(l[0], 'primary down', 384, 360) && l.includes('drag') && near(l.at(-1), 'primary up', 704, 360);
    });
    // Touch and hold → secondary click.
    const hold = inputLines(helm).length;
    await b.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ ...at(0.25, 0.25), id: 0 }] });
    await sleep(800);
    await b.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    step('secondary click');
    await until('secondary click', () => inputLines(helm).slice(hold).some(l => l.startsWith('secondary down')));
    // Pinch out → wheel up (zoom in) at the pinch centre.
    const pinch = inputLines(helm).length;
    await touch(b, [[at(0.45, 0.5), at(0.55, 0.5)], [at(0.4, 0.5), at(0.6, 0.5)], [at(0.3, 0.5), at(0.7, 0.5)], [at(0.3, 0.5), at(0.7, 0.5)]]);
    step('wheel');
    await until('wheel', () => inputLines(helm).slice(pinch).some(l => l === 'wheel 1'));

    // Keyboard: text and keys.
    await b.eval(`document.querySelector('[data-action="keyboard"]').click()`);
    step('keyboard');
    await until('keyboard', () => b.eval(`!!document.querySelector('#keyboard-text')`));
    await b.eval(`(() => { const i = document.querySelector('#keyboard-text'); i.value = 'Sandhamn'; i.form.requestSubmit(); })()`);
    await b.eval(`document.querySelector('[data-key="Enter"]').click()`);
    step('typed');
    await until('typed', () => { const l = inputLines(helm); return l.includes('text Sandhamn') && l.includes('key Enter'); });
    if (shots) await screenshot(b, join(shots, '05-keyboard.png'));
    await b.eval(`document.querySelector('[data-action="keyboard"]').click()`);

    // Menu with connection details.
    await b.eval(`document.querySelector('[data-action="menu"]').click()`);
    step('menu');
    await until('menu', () => b.eval(`!!document.querySelector('.menu')`));
    await sleep(2300); // one stats period
    await b.eval(`document.querySelector('[data-action="menu"]').click(); document.querySelector('[data-action="menu"]').click()`);
    if (shots) await screenshot(b, join(shots, '06-menu.png'));
    await b.eval(`document.querySelector('[data-action="menu"]').click()`);

    // Pilot status from SKAGER is shown, read-only.
    helm.send({ cmd: 'pilot', status: { state: 'auto', heading: 184 } });
    step('pilot chip');
    await until('pilot chip', () => b.eval(`document.querySelector('.tool.pilot')?.getAttribute('aria-label')`));
    assert.match(await b.eval(`document.querySelector('.tool.pilot').getAttribute('aria-label')`), /Auto 184°/);

    // Portrait.
    if (shots) {
      await b.send('Emulation.setDeviceMetricsOverride', { width: 390, height: 844, deviceScaleFactor: 3, mobile: true });
      await sleep(600);
      await screenshot(b, join(shots, '07-live-portrait.png'));
      await b.send('Emulation.setDeviceMetricsOverride', { width: 844, height: 390, deviceScaleFactor: 2, mobile: true });
      await sleep(400);
    }

    // SKAGER (the PC side) stops: the app says so, then reconnects by itself.
    const port = helm.port, stateDir = helm.stateDir;
    await helm.stop();
    step('stopped notice');
    await until('stopped notice', () => b.eval(`['ended', 'reconnecting'].includes(${overlay})`));
    if (shots) await screenshot(b, join(shots, '08-stopped.png'));
    helm = await startHelm({ port, stateDir });
    step('reconnected after restart');
    await until('reconnected after restart', () => b.eval(live), 30000);

    // Another phone takes over: this one is told who, and can take it back.
    const second = await browser(844, 390);
    try {
      const link2 = await pairingLink(helm);
      await second.send('Page.navigate', { url: `${url}#pair=${encodeURIComponent(link2)}` });
      step('second phone busy');
      await until('second phone busy', () => second.eval(`${overlay} === 'busy'`), 20000);
      if (shots) await screenshot(second, join(shots, '09-busy.png'));
      await second.eval(`document.querySelector('[data-action="takeover"]').click()`);
      step('second phone live');
      await until('second phone live', () => second.eval(live), 20000);
      step('first phone told');
      await until('first phone told', () => b.eval(`${overlay} === 'taken'`));
      if (shots) await screenshot(b, join(shots, '10-taken.png'));
      // No touches leave a phone that is not connected.
      const n = inputLines(helm).length;
      await touch(b, [[at(0.05, 0.08)], [at(0.05, 0.08)]]);
      await sleep(500);
      assert.equal(inputLines(helm).length, n);
    } finally {
      second.close();
    }

    // Removing the phone in SKAGER: the app explains and offers to pair again.
    const devices = helm.events.filter(e => e.event === 'devices').at(-1).devices;
    await b.eval(`document.querySelector('[data-action="takeover"]').click()`);
    step('back on first phone');
    await until('back on first phone', () => b.eval(live), 20000);
    const first = devices.find((d: any) => d.name !== helm.events.filter(e => e.event === 'paired').at(-1).device.name) ?? devices[0];
    helm.send({ cmd: 'revoke', device: first.id });
    step('revoked notice');
    await until('revoked notice', () => b.eval(`${overlay} === 'revoked'`));
    if (shots) await screenshot(b, join(shots, '11-revoked.png'));
    assert.deepEqual(b.logs.filter(l => l.startsWith('exception')), []);
  } finally {
    b.close();
    await helm.stop();
    server.close();
  }
});
