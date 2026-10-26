// Builds the web bundle that Capacitor copies into the Android and iOS apps.
//   SKAGER_PREVIEW=1    browser preview and end-to-end tests (accepts #pair=<link>; never for phones)
import { build } from 'esbuild';
import { cpSync, mkdirSync, readFileSync, rmSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const repo = join(root, '..', '..');
const out = join(root, 'dist');
const pkg = JSON.parse(readFileSync(join(root, 'package.json'), 'utf8'));
const preview = process.env.SKAGER_PREVIEW === '1';

rmSync(out, { recursive: true, force: true });
mkdirSync(join(out, 'assets'), { recursive: true });
await build({
  entryPoints: [join(root, 'src/main.ts')], outfile: join(out, 'app.js'), bundle: true, format: 'esm', minify: true,
  sourcemap: false, target: ['es2021', 'safari15', 'chrome100'], legalComments: 'linked',
  define: { __HELM_VERSION__: JSON.stringify(pkg.version), __HELM_PREVIEW__: String(preview) },
});
cpSync(join(root, 'src/index.html'), join(out, 'index.html'));
cpSync(join(root, 'src/app.css'), join(out, 'app.css'));
// One design system: the SKAGER tokens, font and wordmark shared with Companion and the website.
cpSync(join(repo, 'companion/src/tokens.css'), join(out, 'tokens.css'));
const marketing = join(repo, 'web/skager_web/marketing/static/marketing');
cpSync(join(marketing, 'fonts/InterVariable-latin.woff2'), join(out, 'assets/InterVariable-latin.woff2'));
cpSync(join(marketing, 'skager-wordmark.png'), join(out, 'assets/skager-wordmark.png'));
console.log(`SKAGER Helm ${pkg.version}${preview ? ' (browser preview)' : ''}`);
