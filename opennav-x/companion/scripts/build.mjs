// Builds the web bundle that Capacitor copies into the Android and iOS apps.
//   SKAGER_API          SKAGER Cloud origin (default https://beta.skager.app)
//   SKAGER_PUSH         comma list of platforms with push set up, e.g. "android,ios".
//                       Default: android when android/app/google-services.json exists.
//   SKAGER_PREVIEW=1    browser preview against a local server (never for phones)
import { build } from 'esbuild';
import { cpSync, existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const repo = join(root, '..');
const out = join(root, 'dist');
const pkg = JSON.parse(readFileSync(join(root, 'package.json'), 'utf8'));
const api = process.env.SKAGER_API || 'https://beta.skager.app';
const preview = process.env.SKAGER_PREVIEW === '1';
if (!/^https:\/\/[a-z0-9.-]+$/.test(api) && !(preview && /^http:\/\/(localhost|127\.0\.0\.1):\d+$/.test(api))) {
  throw new Error('SKAGER_API must be an https origin');
}
const push = (process.env.SKAGER_PUSH ?? (existsSync(join(root, 'android/app/google-services.json')) ? 'android' : ''))
  .split(',').map(s => s.trim()).filter(s => ['android', 'ios'].includes(s));

rmSync(out, { recursive: true, force: true });
mkdirSync(join(out, 'vendor'), { recursive: true });
mkdirSync(join(out, 'assets'), { recursive: true });
await build({
  entryPoints: [join(root, 'src/main.ts')], outfile: join(out, 'app.js'), bundle: true, format: 'esm', minify: true,
  sourcemap: false, target: ['es2021', 'safari15', 'chrome100'], legalComments: 'linked',
  define: { __SKAGER_API__: JSON.stringify(preview ? '' : api), __SKAGER_VERSION__: JSON.stringify(pkg.version),
    __SKAGER_PUSH__: JSON.stringify(push), __SKAGER_PREVIEW__: String(preview) },
});
const connect = [api, 'https://tiles.openfreemap.org'].join(' ');
writeFileSync(join(out, 'index.html'), readFileSync(join(root, 'src/index.html'), 'utf8').replace('__CONNECT__', preview ? 'https://tiles.openfreemap.org' : connect));
cpSync(join(root, 'src/app.css'), join(out, 'app.css'));
const maplibre = join(root, 'node_modules/maplibre-gl/dist');
cpSync(join(maplibre, 'maplibre-gl.css'), join(out, 'vendor/maplibre-gl.css'));
cpSync(join(maplibre, 'maplibre-gl-worker.mjs'), join(out, 'vendor/maplibre-gl-worker.mjs'));
cpSync(join(root, 'node_modules/maplibre-gl/LICENSE.txt'), join(out, 'vendor/maplibre-gl-LICENSE.txt'));
const marketing = join(repo, 'web/skager_web/marketing/static/marketing');
cpSync(join(marketing, 'fonts/InterVariable-latin.woff2'), join(out, 'assets/InterVariable-latin.woff2'));
cpSync(join(marketing, 'skager-wordmark.png'), join(out, 'assets/skager-wordmark.png'));
console.log(`SKAGER Companion ${pkg.version}: ${preview ? 'browser preview' : api}; push: ${push.join(', ') || 'not set up'}`);
