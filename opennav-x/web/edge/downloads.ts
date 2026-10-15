/** SCRUM-42: private staging downloads. No release is enabled by default. */
import { csrfValid } from './platform.ts';
import { signR2Download } from './r2-downloads.ts';
export { DOWNLOAD_BUCKET, DOWNLOAD_TTL_SECONDS } from './r2-downloads.ts';
import type { Context } from './platform.ts';
import { strictJson } from './commerce.ts';

const PATH = '/api/portal/downloads';
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-8][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
const SHA256 = /^[0-9a-f]{64}$/;
const ID = /^[a-z0-9][a-z0-9_-]{0,63}$/;
const KINDS = ['installer', 'recovery', 'checksums', 'source', 'release-notes'] as const;
type ArtifactId = typeof KINDS[number];
export type DownloadArtifact = { id: ArtifactId; name: string; bytes: number; sha256: string };
export type DownloadRelease = {
  id: string; sourceCommit: string; version: string; channel: 'staging';
  qualification: { record: string; sha256: string };
  artifacts: readonly DownloadArtifact[];
};
type DownloadContext = Pick<Context, 'fetch' | 'getAccount' | 'rpc'> & {
  env: Record<string, string | undefined>;
};
const object = (value: unknown): value is Record<string, any> =>
  value !== null && typeof value === 'object' && !Array.isArray(value);
function requireValue(value: unknown): asserts value { if (!value) throw new Error('Download unavailable'); }
function reply(value: unknown, status = 200): Response {
  return new Response(JSON.stringify(value), { status, headers: {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'private, no-store, max-age=0', 'Pragma': 'no-cache',
    'Referrer-Policy': 'no-referrer', 'X-Content-Type-Options': 'nosniff',
    'X-Robots-Tag': 'noindex, nofollow',
  } });
}

// A catalog is a reviewed server source input, never environment or request JSON.
// Its qualification references do not themselves prove qualification or custody.
function snapshot(catalog: readonly DownloadRelease[]): readonly DownloadRelease[] {
  requireValue(Array.isArray(catalog) && catalog.length <= 16);
  const ids = new Set<string>();
  return Object.freeze(catalog.map(release => {
    requireValue(object(release) && typeof release.id === 'string' && ID.test(release.id) && !ids.has(release.id));
    ids.add(release.id);
    requireValue(typeof release.sourceCommit === 'string' && /^[0-9a-f]{40}$/.test(release.sourceCommit)
      && typeof release.version === 'string' && /^[0-9A-Za-z][0-9A-Za-z.+_-]{0,63}$/.test(release.version)
      && release.channel === 'staging' && object(release.qualification)
      && typeof release.qualification.record === 'string' && release.qualification.record.length <= 256
      && /^docs\/evidence\/[a-z0-9_/-]+\.(?:json|md)$/.test(release.qualification.record)
      && !release.qualification.record.includes('//') && typeof release.qualification.sha256 === 'string'
      && SHA256.test(release.qualification.sha256)
      && Array.isArray(release.artifacts) && release.artifacts.length === KINDS.length);
    const kinds = new Set<string>();
    const artifacts = release.artifacts.map(artifact => {
      requireValue(object(artifact) && KINDS.includes(artifact.id) && !kinds.has(artifact.id));
      kinds.add(artifact.id);
      requireValue(typeof artifact.name === 'string' && /^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$/.test(artifact.name)
        && !artifact.name.includes('..') && !artifact.name.endsWith('.') && typeof artifact.sha256 === 'string'
        && SHA256.test(artifact.sha256)
        && Number.isSafeInteger(artifact.bytes) && artifact.bytes > 0 && artifact.bytes <= 16 * 1024 ** 3);
      const extension = { installer: /\.exe$/, recovery: /\.zip$/, checksums: /\.(?:sha256|txt)$/,
        source: /\.(?:zip|tar\.gz)$/, 'release-notes': /\.(?:md|txt|pdf)$/ }[artifact.id];
      requireValue(extension.test(artifact.name));
      return Object.freeze({ id: artifact.id, name: artifact.name, bytes: artifact.bytes, sha256: artifact.sha256 });
    });
    return Object.freeze({ id: release.id, version: release.version, sourceCommit: release.sourceCommit,
      channel: release.channel, qualification: Object.freeze({ record: release.qualification.record,
        sha256: release.qualification.sha256 }), artifacts: Object.freeze(artifacts) });
  }));
}
function objectName(release: DownloadRelease, artifact: DownloadArtifact): string {
  return `${release.channel}/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`;
}

async function readJson(response: Response, limit: number, signal: AbortSignal): Promise<Record<string, any>> {
  const length = response.headers.get('Content-Length');
  requireValue(length === null || (/^[0-9]{1,8}$/.test(length) && Number(length) <= limit));
  const reader = response.body?.getReader();
  requireValue(reader);
  const pieces: Uint8Array[] = [];
  let size = 0;
  let abort: () => void = () => {};
  const stopped = new Promise<never>((_resolve, reject) => {
    abort = () => reject(new Error('Download unavailable'));
    signal.addEventListener('abort', abort, { once: true });
  });
  try {
    requireValue(!signal.aborted);
    for (;;) {
      const part = await Promise.race([reader.read(), stopped]);
      if (part.done) break;
      size += part.value.byteLength;
      requireValue(size <= limit);
      pieces.push(part.value);
    }
    requireValue(length === null || Number(length) === size);
    const bytes = new Uint8Array(size);
    let offset = 0;
    for (const piece of pieces) { bytes.set(piece, offset); offset += piece.byteLength; }
    return strictJson(bytes);
  } finally {
    signal.removeEventListener('abort', abort);
    void reader.cancel().catch(() => {});
    reader.releaseLock();
  }
}
async function deadline<T>(run: (signal: AbortSignal) => Promise<T>, milliseconds: number): Promise<T> {
  const controller = new AbortController();
  let timer: ReturnType<typeof setTimeout>;
  const expired = new Promise<never>((_resolve, reject) => {
    timer = setTimeout(() => { controller.abort(); reject(new Error('Download unavailable')); }, milliseconds);
  });
  try { return await Promise.race([run(controller.signal), expired]); }
  finally { clearTimeout(timer!); }
}
async function access(request: Request, ctx: DownloadContext): Promise<401 | 403 | string> {
  const account = await ctx.getAccount(request);
  if (!account || typeof account.id !== 'string' || !UUID.test(account.id)) return 401;
  if ((account as { passwordSet?: boolean }).passwordSet === false) return 403;
  const result = await ctx.rpc('store_portal_access', { p_account_id: account.id });
  requireValue(object(result) && Object.hasOwn(result, 'entitlement'));
  if (result.entitlement === null) return 403;
  requireValue(object(result.entitlement) && typeof result.entitlement.status === 'string');
  const entitlement = result.entitlement;
  return entitlement.status === 'active' && entitlement.environment === 'staging'
    && entitlement.product_code === 'skager-beta' ? account.id : 403;
}
async function sign(ctx: DownloadContext, release: DownloadRelease, artifact: DownloadArtifact) {
  return deadline(() => signR2Download(ctx.env, objectName(release, artifact), artifact.name), 8000);
}

export function createDownloadHandler(reviewedCatalog: readonly DownloadRelease[]) {
  const catalog = snapshot(reviewedCatalog);
  return async function handle(request: Request, ctx: DownloadContext): Promise<Response | null> {
    const url = new URL(request.url);
    if (url.pathname !== PATH) return null;
    if (ctx.env.SKAGER_DOWNLOADS_ENABLED !== 'true' || ctx.env.PORTAL_ENABLED !== 'true'
      || ctx.env.STRIPE_TEST_ENVIRONMENT !== 'staging') return reply({ error: 'Downloads unavailable.' }, 404);
    if (!['GET', 'POST'].includes(request.method)) return reply({ error: 'Method not allowed.' }, 405);
    if (url.search) return reply({ error: 'Invalid download request.' }, 400);
    try {
      if (request.method === 'POST' && !await csrfValid(request)) return reply({ error: 'Request rejected.' }, 403);
      const denied = await access(request, ctx);
      if (typeof denied !== 'string') return reply({ error: denied === 401 ? 'Sign in required.' : 'Active beta access required.' }, denied);
      if (request.method === 'GET') return reply({ available: catalog.length > 0,
        reason: catalog.length ? null : 'Release qualification and secure download delivery are pending.',
        releases: catalog.map(({ qualification: _qualification, ...release }) => release) });
      if (request.headers.get('Content-Type')?.split(';')[0].trim().toLowerCase() !== 'application/json')
        return reply({ error: 'Send a download selection as JSON.' }, 415);
      let selection: Record<string, any>;
      try { selection = await deadline(signal => readJson(new Response(request.body, { headers: request.headers }), 1024, signal), 1000); }
      catch { return reply({ error: 'Invalid download request.' }, 400); }
      if (Object.keys(selection).length !== 2 || typeof selection.releaseId !== 'string' || !ID.test(selection.releaseId)
        || typeof selection.artifactId !== 'string' || !KINDS.includes(selection.artifactId))
        return reply({ error: 'Invalid download request.' }, 400);
      const release = catalog.find(item => item.id === selection.releaseId);
      const artifact = release?.artifacts.find(item => item.id === selection.artifactId);
      if (!release || !artifact) return reply({ error: 'Download unavailable.' }, 404);
      const allowed = await ctx.rpc('skager_auth_throttle', {
        p_keys: ['download:account:' + denied], p_limit: 10, p_seconds: 60,
      });
      requireValue(typeof allowed === 'boolean');
      if (!allowed) return reply({ error: 'Too many download requests. Please try again shortly.' }, 429);
      const link = await sign(ctx, release, artifact);
      const revoked = await access(request, ctx);
      if (typeof revoked !== 'string') return reply({ error: revoked === 401 ? 'Sign in required.' : 'Active beta access required.' }, revoked);
      if (revoked !== denied) return reply({ error: 'Sign in required.' }, 401);
      requireValue(Date.parse(link.expiresAt) > Date.now());
      return reply({ ...link, releaseId: release.id, artifactId: artifact.id,
        name: artifact.name, bytes: artifact.bytes, sha256: artifact.sha256 });
    } catch {
      // No vendor body, credential, object token, account identifier or exception is logged or returned.
      return reply({ error: 'Downloads are temporarily unavailable. Please try again.' }, 503);
    }
  };
}

// Reviewed source catalog. Each entry names a qualified staging candidate whose exact
// bytes (length and SHA-256 from its release manifest) were uploaded to the private bucket.
// Environment configuration can never add releases.
export const RELEASES: readonly DownloadRelease[] = [{
  id: 'beta2-3', version: '0.4.0-beta2.3', channel: 'staging',
  sourceCommit: '3d7d2a44467c1b1ede917b78e5b183da2cfa023f',
  qualification: { record: 'docs/evidence/release-beta2-3/qualification.json',
    sha256: 'c0c3afc6bd327c4c9fdf8e22dd41a9aaab65a2f9b6b553606a34c70153b86457' },
  artifacts: [
    { id: 'installer', name: 'SKAGER-Beta2-Setup.exe', bytes: 185287653, sha256: '7abbf0d2d69a6a4216c545994dfdc1b82db3f9541880a9ef8a07d20dec040258' },
    { id: 'recovery', name: 'SKAGER-Beta2-Portable-Recovery.zip', bytes: 191599179, sha256: 'ec90996e2dcfa5217831622b2a74a14ba58fc166bc99d7f8710eeddc97a8dbef' },
    { id: 'checksums', name: 'SHA256SUMS.txt', bytes: 565, sha256: 'c91da6a173ddd57ae5bc10c90baa05751869f1d2a5b505f6d678aa755e4a718b' },
    { id: 'source', name: 'SKAGER-Beta2-source.zip', bytes: 440599515, sha256: 'e355a3890df827867c98da47214aa2b3c3637df3798bc505404db5d01e4f1ad1' },
    { id: 'release-notes', name: 'SKAGER-Beta2-Release-Notes.md', bytes: 8545, sha256: '9c079184900911f9aaacdc31cde742dbd7c8232ca66d11e7f99f1999020166d8' },
  ],
}];
export const handleDownloads = createDownloadHandler(RELEASES);
