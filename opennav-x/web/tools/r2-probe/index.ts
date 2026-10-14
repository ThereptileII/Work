/** Administrative one-shot R2 connectivity diagnostic; never a product route. */
import { AwsClient } from '../../edge/vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';
import { DOWNLOAD_BUCKET, R2_ACCOUNT_ID, R2_DOWNLOAD_ORIGIN } from '../../edge/r2-downloads.ts';

const PROJECT = 'kelwgubkuvditalsbqpr';
const ORIGIN = `https://${PROJECT}.supabase.co`;
const TARGET = `${R2_DOWNLOAD_ORIGIN}/${DOWNLOAD_BUCKET}`;
export const DEADLINE_MS = 5000;
type Env = Record<string, string | undefined>;
const unavailable = () => new Error('Administrative R2 probe unavailable.');
function requireValue(value: unknown): asserts value { if (!value) throw unavailable(); }
function reply(status: number, body: Record<string, unknown>) {
  return new Response(JSON.stringify(body), { status, headers: {
    'Content-Type': 'application/json', 'Cache-Control': 'private, no-store', 'X-Content-Type-Options': 'nosniff',
  } });
}

function serviceBinding(env: Env): string {
  const key = env.SUPABASE_SERVICE_ROLE_KEY;
  requireValue(env.SUPABASE_URL === ORIGIN && typeof key === 'string' && key.length <= 8192
    && key !== env.SUPABASE_ANON_KEY && /^[A-Za-z0-9_-]+\.[A-Za-z0-9_-]+\.[A-Za-z0-9_-]{43}$/.test(key));
  const decode = (part: string) => JSON.parse(atob(part.replace(/-/g, '+').replace(/_/g, '/')));
  const [header, payload] = key.split('.').slice(0, 2).map(decode);
  const now = Math.floor(Date.now() / 1000);
  // This checks the TRUSTED platform-injected binding's shape. It is NOT a JWT
  // verifier and never authorizes a caller by decoding caller-supplied claims.
  requireValue(header.alg === 'HS256' && header.typ === 'JWT' && payload.iss === 'supabase'
    && payload.ref === PROJECT && payload.role === 'service_role'
    && Number.isSafeInteger(payload.iat) && payload.iat > 0 && payload.iat <= now
    && Number.isSafeInteger(payload.exp) && payload.exp > now);
  return key;
}

async function authorized(request: Request, expected: string): Promise<boolean> {
  const header = request.headers.get('authorization');
  if (typeof header !== 'string' || header.length > 8199 || !/^Bearer [A-Za-z0-9_.-]+$/.test(header)) return false;
  // Exact possession of the genuine injected service key is required. A user
  // JWT, anon/publishable key or another project's service key cannot qualify.
  const encode = new TextEncoder();
  const [a, b] = await Promise.all([expected, header.slice(7)].map(value =>
    crypto.subtle.digest('SHA-256', encode.encode(value)).then(bytes => new Uint8Array(bytes))));
  let difference = 0;
  for (let index = 0; index < a.length; index++) difference |= a[index] ^ b[index];
  return difference === 0;
}

export async function handleProbe(request: Request, env: Env, transport: typeof fetch = fetch): Promise<Response> {
  try {
    const url = new URL(request.url);
    // The gateway may replace the public origin with an internal HTTP origin.
    // Project identity comes from the trusted service binding, not this URL;
    // the incoming origin never selects an outbound host or credential.
    if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password
      || !['/skager-r2-probe', '/functions/v1/skager-r2-probe'].includes(url.pathname)
      || url.search || url.hash) return reply(404, { error: 'Not found.' });
    if (request.method !== 'GET') return reply(405, { error: 'GET required.' });
    if (request.body !== null) {
      void request.body.cancel().catch(() => {});
      return reply(400, { error: 'Request body not supported.' });
    }
    if (!await authorized(request, serviceBinding(env))) return reply(401, { error: 'Administrative credential required.' });
    requireValue(env.SKAGER_DOWNLOAD_BACKEND === 'r2' && env.R2_ACCOUNT_ID === R2_ACCOUNT_ID
      && env.R2_DOWNLOAD_BUCKET === DOWNLOAD_BUCKET && /^[0-9a-f]{32}$/.test(env.R2_ACCESS_KEY_ID || '')
      && /^[0-9a-f]{64}$/.test(env.R2_SECRET_ACCESS_KEY || ''));
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const expired = new Promise<never>((resolve, reject) => {
      timer = setTimeout(() => { controller.abort(); reject(unavailable()); }, DEADLINE_MS);
    });
    try {
      await Promise.race([expired, (async () => {
        const client = new AwsClient({ accessKeyId: env.R2_ACCESS_KEY_ID,
          secretAccessKey: env.R2_SECRET_ACCESS_KEY, service: 's3', region: 'auto', retries: 0 });
        const signed = await client.sign(TARGET, { method: 'HEAD', redirect: 'error',
          cache: 'no-store', signal: controller.signal });
        requireValue(!controller.signal.aborted && signed.url === TARGET && signed.method === 'HEAD'
          && signed.body === null && signed.redirect === 'error' && signed.cache === 'no-store'
          && signed.headers.has('Authorization') && !signed.headers.has('apikey')
          && !signed.headers.has('X-Amz-Security-Token'));
        // sign() does local cryptography; exactly this fetch performs I/O. Never
        // use AwsClient.fetch(), which can retry. No caller headers are forwarded.
        const response = await transport(signed);
        if (response.body !== null) void response.body.cancel().catch(() => {});
        requireValue(!controller.signal.aborted && response.status === 200 && !response.redirected
          && response.body === null && (!response.url || response.url === TARGET));
      })()]);
      return reply(200, { probe: 'r2-head-bucket', authenticated: true });
    } finally { clearTimeout(timer!); controller.abort(); }
  } catch { return reply(503, { error: 'Administrative R2 probe unavailable.' }); }
}

declare const Deno: { env: { get(name: string): string | undefined }; serve(handler: (request: Request) => Promise<Response>): void };
if (typeof Deno !== 'undefined') Deno.serve(request => handleProbe(request, Object.fromEntries([
  'SUPABASE_URL', 'SUPABASE_SERVICE_ROLE_KEY', 'SUPABASE_ANON_KEY', 'SKAGER_DOWNLOAD_BACKEND',
  'R2_ACCOUNT_ID', 'R2_DOWNLOAD_BUCKET', 'R2_ACCESS_KEY_ID', 'R2_SECRET_ACCESS_KEY',
].map(name => [name, Deno.env.get(name)]))));
