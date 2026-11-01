// The demo (for app review and curious owners): no boat, no network. Sample SKAGER
// screens come through the real screen view; taps on SKAGER's rail switch screens.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { browser, buildPreview, serveApp, sleep, touch, until, screenshot } from './harness.ts';

const appRoot = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const shots = process.env.HELM_SCREENSHOTS;

const phase = `document.querySelector('#app').dataset.phase`;
const overlay = `document.querySelector('#overlay').dataset.kind`;
const live = `${phase} === 'helm' && !${overlay} && document.querySelector('#screen').width === 1600`;
const frame = `(() => { const r = document.querySelector('#screen').getBoundingClientRect(); return { left: r.left, top: r.top, w: r.width, h: r.height }; })()`;
// A pixel of the SKAGER frame (frame pixels), as drawn on the phone.
const pixel = (x: number, y: number) => `Array.from(document.querySelector('#screen').getContext('2d').getImageData(${x}, ${y}, 1, 1).data)`;

test('demo: sample SKAGER screens, rail taps switch screens, disconnect returns home', { timeout: 60000 }, async () => {
  buildPreview(appRoot);
  const { server, url } = await serveApp(join(appRoot, 'dist'));
  const b = await browser(844, 390);
  try {
    await b.send('Page.navigate', { url });
    await until('welcome', () => b.eval(`!!document.querySelector('[data-action="demo"]')`));
    if (shots) {
      await b.send('Emulation.setDeviceMetricsOverride', { width: 390, height: 844, deviceScaleFactor: 3, mobile: true });
      await sleep(300);
      await screenshot(b, join(shots, 'demo-01-welcome.png'));
      await b.send('Emulation.setDeviceMetricsOverride', { width: 844, height: 390, deviceScaleFactor: 2, mobile: true });
    }
    await b.eval(`document.querySelector('[data-action="demo"]').click()`);
    await until('demo live', () => b.eval(live));
    assert.equal(await b.eval(`document.querySelector('#toolbar').textContent.includes('Demo boat')`), true);
    // First session: the gesture guide, as with a real boat.
    await until('gesture guide', () => b.eval(`!!document.querySelector('.gestures')`));
    await b.eval(`document.querySelector('[data-action="close-sheet"]').click()`);

    // The chart screen: water in the middle of the chart.
    const water = await b.eval<number[]>(pixel(400, 300));
    assert.ok(water[2] > 200 && water[0] > 190, `chart water ${water}`);
    if (shots) await screenshot(b, join(shots, 'demo-02-chart.png'));

    // Tap Instruments on SKAGER's rail (frame 68,530): the instruments panel replaces the chart.
    const f = await b.eval<{ left: number; top: number; w: number; h: number }>(frame);
    const at = (x: number, y: number) => ({ x: f.left + x / 1600 * f.w, y: f.top + y / 1000 * f.h });
    await touch(b, [[at(68, 530)], [at(68, 530)]]);
    await until('instruments', async () => (await b.eval<number[]>(pixel(400, 300)))[0] < 80);
    if (shots) await screenshot(b, join(shots, 'demo-03-instruments.png'));

    // Close on SKAGER's panel → the chart again; a tap elsewhere changes nothing.
    await touch(b, [[at(1150, 210)], [at(1150, 210)]]);
    await until('chart again', async () => (await b.eval<number[]>(pixel(400, 300)))[0] > 190);
    await touch(b, [[at(700, 400)], [at(700, 400)]]);
    await sleep(300);
    assert.ok((await b.eval<number[]>(pixel(400, 300)))[0] > 190);

    // Disconnect → home, and the demo boat is not remembered as a paired boat.
    await b.eval(`document.querySelector('#toolbar [data-action="disconnect"]').click()`);
    await until('home', () => b.eval(`${phase} === 'home' && !!document.querySelector('.welcome')`));
  } finally {
    b.close();
    server.close();
  }
});
