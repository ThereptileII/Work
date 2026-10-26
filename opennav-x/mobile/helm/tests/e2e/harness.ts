// End-to-end harness: the real Helm Server (test pattern display) + the app's
// browser preview in headless Chromium, driven over the DevTools protocol.
//   SKAGER_HELM_SERVER   path to a built skager-helm binary (required)
//   CHROME               browser binary (default: chromium, then google-chrome)
import { spawn, spawnSync } from 'node:child_process';
import type { ChildProcess } from 'node:child_process';
import { createServer } from 'node:http';
import type { Server } from 'node:http';
import { mkdtempSync, readFileSync, existsSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { extname, join } from 'node:path';
import { createInterface } from 'node:readline';

export const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));

export async function until<T>(what: string, f: () => Promise<T | null | undefined | false> | T | null | undefined | false, ms = 15000): Promise<T> {
  const end = Date.now() + ms;
  let last: unknown;
  while (Date.now() < end) {
    try { const v = await f(); if (v) return v as T; } catch (e) { last = e; }
    await sleep(100);
  }
  throw new Error(`timed out: ${what}${last ? ` (${last})` : ''}`);
}

// ---------------------------------------------------------------- Helm Server
export type Helm = { proc: ChildProcess; port: number; events: any[]; send(cmd: object): void; inputLog: string; stateDir: string; stop(): Promise<void> };

export async function startHelm(opts: { stateDir?: string; port?: number; name?: string } = {}): Promise<Helm> {
  const bin = process.env.SKAGER_HELM_SERVER;
  if (!bin || !existsSync(bin)) throw new Error('set SKAGER_HELM_SERVER to a built skager-helm binary');
  const stateDir = opts.stateDir ?? mkdtempSync(join(tmpdir(), 'helm-e2e-'));
  const inputLog = join(stateDir, 'input.log');
  const proc = spawn(bin, ['-source', 'test', '-test-size', '1280x720', '-control', 'stdio', '-addr', `127.0.0.1:${opts.port ?? 0}`,
    '-state-dir', stateDir, '-name', opts.name ?? 'Aurora', '-input-log', inputLog], { stdio: ['pipe', 'pipe', 'inherit'] });
  const events: any[] = [];
  createInterface({ input: proc.stdout! }).on('line', line => { try { events.push(JSON.parse(line)); } catch { /* ignore */ } });
  const ready = await until('helm ready', () => events.find(e => e.event === 'ready'));
  return {
    proc, port: ready.port, events, inputLog, stateDir,
    send: cmd => proc.stdin!.write(JSON.stringify(cmd) + '\n'),
    stop: () => new Promise(resolve => {
      if (proc.exitCode !== null || proc.signalCode !== null) return resolve();
      proc.once('exit', () => resolve());
      proc.stdin!.end();
      setTimeout(() => proc.kill(), 2000);
    }),
  };
}

/** Opens pairing and returns the QR link, pointed at the loopback listener. */
export async function pairingLink(helm: Helm): Promise<string> {
  const before = helm.events.length;
  helm.send({ cmd: 'pair' });
  const ev = await until('pairing event', () => helm.events.slice(before).find(e => e.event === 'pairing'));
  const [head, query] = ev.link.split('?');
  const q = new URLSearchParams(query);
  q.set('a', `127.0.0.1:${helm.port}`);
  return `${head}?${q}`;
}

export function inputLines(helm: Helm): string[] {
  try { return readFileSync(helm.inputLog, 'utf8').trim().split('\n').filter(Boolean); } catch { return []; }
}

// ---------------------------------------------------------------- app preview
const TYPES: Record<string, string> = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.png': 'image/png',
  '.woff2': 'font/woff2', '.txt': 'text/plain' };

export function serveApp(dist: string): Promise<{ server: Server; url: string }> {
  return new Promise(resolve => {
    const server = createServer((req, res) => {
      const path = decodeURIComponent((req.url ?? '/').split(/[?#]/)[0]);
      const file = join(dist, path === '/' ? 'index.html' : path.replace(/\.\.+/g, ''));
      if (!existsSync(file)) { res.writeHead(404); res.end(); return; }
      res.writeHead(200, { 'content-type': TYPES[extname(file)] ?? 'application/octet-stream' });
      res.end(readFileSync(file));
    }).listen(0, '127.0.0.1', () => resolve({ server, url: `http://localhost:${(server.address() as any).port}/` }));
  });
}

export function buildPreview(appRoot: string): void {
  const r = spawnSync(process.execPath, ['scripts/build.mjs'], { cwd: appRoot, env: { ...process.env, SKAGER_PREVIEW: '1' }, stdio: 'inherit' });
  if (r.status !== 0) throw new Error('preview build failed');
}

// ---------------------------------------------------------------- browser
export type Browser = { send(method: string, params?: object): Promise<any>; eval<T = any>(expr: string): Promise<T>; close(): void; logs: string[] };

export async function browser(width: number, height: number, scale = 2): Promise<Browser> {
  const bin = process.env.CHROME ?? (spawnSync('which', ['chromium']).status === 0 ? 'chromium' : 'google-chrome');
  const profile = mkdtempSync(join(tmpdir(), 'helm-chrome-'));
  const chrome = spawn(bin, ['--headless=new', '--no-sandbox', '--hide-scrollbars', '--use-angle=swiftshader', '--enable-unsafe-swiftshader',
    '--disable-features=WebRtcHideLocalIpsWithMdns', `--user-data-dir=${profile}`, '--remote-debugging-port=0',
    `--window-size=${width},${height}`, 'about:blank'], { stdio: 'ignore' });
  // Port 0: the browser picks a free port and writes it to its profile, so
  // parallel test browsers never attach to each other.
  const port = await until('chrome', () => {
    try { return readFileSync(join(profile, 'DevToolsActivePort'), 'utf8').split('\n')[0].trim() || null; } catch { return null; }
  });
  const target = await until('chrome page', async () => ((await (await fetch(`http://127.0.0.1:${port}/json`)).json()) as any[]).find(t => t.type === 'page'));
  const ws = new WebSocket(target.webSocketDebuggerUrl);
  await new Promise(r => ws.addEventListener('open', r));
  let id = 0;
  const pending = new Map<number, (v: any) => void>();
  const logs: string[] = [];
  ws.addEventListener('message', e => {
    const m = JSON.parse(String(e.data));
    if (m.id && pending.has(m.id)) { pending.get(m.id)!(m.error ? Promise.reject(new Error(m.error.message)) : m.result); pending.delete(m.id); }
    if (m.method === 'Runtime.exceptionThrown') logs.push('exception: ' + (m.params.exceptionDetails.exception?.description ?? m.params.exceptionDetails.text));
    if (m.method === 'Runtime.consoleAPICalled') logs.push(`console.${m.params.type}: ` + m.params.args.map((a: any) => a.value ?? a.description).join(' '));
  });
  // Every DevTools call has a deadline, so a stuck page fails the test with a reason instead of hanging it.
  const send = (method: string, params: object = {}) => new Promise<any>((resolve, reject) => {
    const n = ++id;
    const timer = setTimeout(() => { pending.delete(n); reject(new Error(`DevTools ${method} timed out`)); }, 10000);
    pending.set(n, v => { clearTimeout(timer); resolve(v); });
    ws.send(JSON.stringify({ id: n, method, params }));
  });
  await send('Runtime.enable');
  await send('Page.enable');
  await send('Emulation.setDeviceMetricsOverride', { width, height, deviceScaleFactor: scale, mobile: true });
  await send('Emulation.setTouchEmulationEnabled', { enabled: true, maxTouchPoints: 5 });
  return {
    send, logs,
    eval: async (expr: string) => {
      const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true });
      if (r.exceptionDetails) throw new Error(r.exceptionDetails.exception?.description ?? r.exceptionDetails.text);
      return r.result.value;
    },
    close: () => {
      ws.close();
      chrome.once('exit', () => { try { rmSync(profile, { recursive: true, force: true }); } catch { /* best effort */ } });
      chrome.kill();
    },
  };
}

export async function touch(b: Browser, points: { x: number; y: number }[][], stepMs = 30): Promise<void> {
  // points[i] = the fingers at step i; first step starts, last step ends.
  for (let i = 0; i < points.length; i++) {
    const type = i === 0 ? 'touchStart' : i === points.length - 1 ? 'touchEnd' : 'touchMove';
    await b.send('Input.dispatchTouchEvent', { type, touchPoints: type === 'touchEnd' ? [] : points[i].map((p, n) => ({ ...p, id: n })) });
    await sleep(stepMs);
  }
}

export async function screenshot(b: Browser, file: string): Promise<void> {
  const { writeFileSync } = await import('node:fs');
  const shot = await b.send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(file, Buffer.from(shot.data, 'base64'));
}
