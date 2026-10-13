/** SCRUM-42: private staging downloads. No release is enabled by default. */
import { csrfValid, PUBLIC_ORIGIN, SUPABASE_ORIGIN } from './platform.ts';
import type { Context } from './platform.ts';
import { strictJson } from './commerce.ts';

export const DOWNLOAD_BUCKET = 'skager-private-releases';
export const DOWNLOAD_TTL_SECONDS = 60;
const PATH = '/api/portal/downloads';
const STORAGE = SUPABASE_ORIGIN + '/storage/v1';
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
      && typeof release.qualification.record === 'string'
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
  const result = await ctx.rpc('commerce_portal_access', { p_account_id: account.id });
  requireValue(object(result) && Object.hasOwn(result, 'entitlement'));
  if (result.entitlement === null) return 403;
  requireValue(object(result.entitlement) && typeof result.entitlement.status === 'string');
  const entitlement = result.entitlement;
  return entitlement.status === 'active' && entitlement.environment === 'staging'
    && entitlement.product_code === 'skager-beta' ? account.id : 403;
}
function jwtPart(value: string): Record<string, any> {
  requireValue(/^[A-Za-z0-9_-]{1,4096}$/.test(value));
  const raw = atob(value.replace(/-/g, '+').replace(/_/g, '/') + '='.repeat((4 - value.length % 4) % 4));
  return strictJson(Uint8Array.from(raw, char => char.charCodeAt(0)));
}
function signedLink(raw: unknown, object: string, filename: string, started: number): { url: string; expiresAt: string } {
  requireValue(typeof raw === 'string' && raw.length <= 12000 && !/[^\x21-\x7e]/.test(raw));
  const relative = `/object/sign/${DOWNLOAD_BUCKET}/${object}`;
  const url = new URL(STORAGE + raw);
  requireValue(url.origin === SUPABASE_ORIGIN && url.pathname === '/storage/v1' + relative
    && !url.username && !url.password && !url.hash);
  const entries = [...url.searchParams];
  requireValue(entries.length === 1 && entries[0][0] === 'token');
  const token = entries[0][1];
  requireValue(raw === relative + '?token=' + token && /^[A-Za-z0-9_-]+\.[A-Za-z0-9_-]+\.[A-Za-z0-9_-]{16,1024}$/.test(token));
  const [headerPart, payloadPart] = token.split('.');
  const header = jwtPart(headerPart); const payload = jwtPart(payloadPart);
  requireValue(['HS256', 'HS384', 'HS512', 'RS256', 'RS384', 'RS512', 'ES256', 'ES384', 'ES512', 'EdDSA'].includes(header.alg));
  const now = Math.floor(Date.now() / 1000);
  requireValue(payload.url === DOWNLOAD_BUCKET + '/' + object && (payload.scope === undefined || payload.scope === 'download')
    && Number.isSafeInteger(payload.iat) && Math.abs(payload.iat - started) <= 5
    && Number.isSafeInteger(payload.exp) && payload.exp > now && payload.exp > payload.iat
    && payload.exp - payload.iat <= DOWNLOAD_TTL_SECONDS && payload.exp <= started + DOWNLOAD_TTL_SECONDS + 5);
  // Claims constrain an authenticated Storage response; Storage verifies its own
  // signature at redemption. Parsing here is not JWT signature authentication.
  url.searchParams.set('download', filename);
  return { url: url.href, expiresAt: new Date(payload.exp * 1000).toISOString() };
}
async function sign(ctx: DownloadContext, release: DownloadRelease, artifact: DownloadArtifact) {
  requireValue(ctx.env.SUPABASE_URL === SUPABASE_ORIGIN && ctx.env.SUPABASE_SERVICE_ROLE_KEY);
  const key = ctx.env.SUPABASE_SERVICE_ROLE_KEY;
  const object = objectName(release, artifact);
  const target = `${STORAGE}/object/sign/${DOWNLOAD_BUCKET}/${object}`;
  const started = Math.floor(Date.now() / 1000);
  return deadline(async signal => {
    const transport = ctx.fetch;
    const response = await transport(target, { method: 'POST', redirect: 'manual', signal,
      headers: { 'Content-Type': 'application/json', 'Accept-Encoding': 'identity', apikey: key, Authorization: `Bearer ${key}` },
      body: JSON.stringify({ expiresIn: DOWNLOAD_TTL_SECONDS }) });
    try {
      requireValue(response.status === 200 && !response.redirected && (!response.url || response.url === target)
        && response.headers.get('Content-Type')?.split(';')[0].trim().toLowerCase() === 'application/json'
        && ['', 'identity'].includes(response.headers.get('Content-Encoding')?.toLowerCase() ?? ''));
      const result = await readJson(response, 16384, signal);
      requireValue(Object.keys(result).length === 1 && Object.hasOwn(result, 'signedURL'));
      return signedLink(result.signedURL, object, artifact.name, started);
    } finally { void response.body?.cancel().catch(() => {}); }
  }, 8000);
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

// Qualification, immutable uploads and owner approval must precede a reviewed
// nonempty source catalog. Environment configuration alone cannot add releases.
export const handleDownloads = createDownloadHandler([]);
