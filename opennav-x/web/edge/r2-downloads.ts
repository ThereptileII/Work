/** Server-only R2 adapter. Credentials belong in Supabase Edge secret bindings. */
import { AwsClient } from './vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';

export const DOWNLOAD_BUCKET = 'skager-private-releases';
export const DOWNLOAD_TTL_SECONDS = 60;
export const R2_ACCOUNT_ID = '7869adcf376c630b175c94729321e016';
export const R2_DOWNLOAD_ORIGIN = `https://${R2_ACCOUNT_ID}.r2.cloudflarestorage.com`;
const QUERY = ['response-content-disposition', 'X-Amz-Expires', 'X-Amz-Date', 'X-Amz-Algorithm',
  'X-Amz-Credential', 'X-Amz-SignedHeaders', 'X-Amz-Signature'];
const OBJECT = /^staging\/[0-9a-f]{40}\/[a-z0-9][a-z0-9_-]{0,63}\/(?:installer|recovery|checksums|source|release-notes)\/[0-9a-f]{64}\/([A-Za-z0-9][A-Za-z0-9._-]{0,127})$/;
const unavailable = () => new Error('Download unavailable');
function requireValue(value: unknown): asserts value { if (!value) throw unavailable(); }

export async function signR2Download(env: Record<string, string | undefined>, object: string, filename: string) {
  try {
    // No endpoint override, region override, temporary credentials or provider fallback.
    requireValue(env.SKAGER_DOWNLOAD_BACKEND === 'r2' && env.R2_ACCOUNT_ID === R2_ACCOUNT_ID
      && env.R2_DOWNLOAD_BUCKET === DOWNLOAD_BUCKET && typeof env.R2_ACCESS_KEY_ID === 'string'
      && /^[0-9a-f]{32}$/.test(env.R2_ACCESS_KEY_ID) && typeof env.R2_SECRET_ACCESS_KEY === 'string'
      && /^[0-9a-f]{64}$/.test(env.R2_SECRET_ACCESS_KEY));
    const match = OBJECT.exec(object);
    requireValue(match && match[1] === filename && !filename.includes('..') && !filename.endsWith('.'));
    const path = `/${DOWNLOAD_BUCKET}/${object}`;
    const disposition = `attachment; filename="${filename}"`;
    const started = Math.floor(Date.now() / 1000) * 1000;
    const datetime = new Date(started).toISOString().replace(/[:-]|\.\d{3}/g, '');
    const target = new URL(R2_DOWNLOAD_ORIGIN + path);
    target.searchParams.set('response-content-disposition', disposition);
    target.searchParams.set('X-Amz-Expires', String(DOWNLOAD_TTL_SECONDS));
    const client = new AwsClient({ accessKeyId: env.R2_ACCESS_KEY_ID,
      secretAccessKey: env.R2_SECRET_ACCESS_KEY, service: 's3', region: 'auto', retries: 0 });
    // sign() is local cryptography only; never fetch(), HEAD, PUT or an upload signer.
    const signed = await client.sign(new Request(target, { method: 'GET' }),
      { aws: { signQuery: true, datetime } });
    const url = new URL(signed.url);
    requireValue(signed.method === 'GET' && signed.body === null && [...signed.headers].length === 0
      && url.origin === R2_DOWNLOAD_ORIGIN && url.pathname === path && !url.username && !url.password && !url.hash
      && url.href === signed.url && url.search.slice(1) === url.searchParams.toString()
      && [...url.searchParams].length === QUERY.length
      && QUERY.every(key => url.searchParams.getAll(key).length === 1)
      && [...url.searchParams.keys()].every(key => QUERY.includes(key))
      && url.searchParams.get('response-content-disposition') === disposition
      && url.searchParams.get('X-Amz-Expires') === String(DOWNLOAD_TTL_SECONDS)
      && url.searchParams.get('X-Amz-Date') === datetime
      && url.searchParams.get('X-Amz-Algorithm') === 'AWS4-HMAC-SHA256'
      && url.searchParams.get('X-Amz-Credential') === `${env.R2_ACCESS_KEY_ID}/${datetime.slice(0, 8)}/auto/s3/aws4_request`
      && url.searchParams.get('X-Amz-SignedHeaders') === 'host'
      && /^[0-9a-f]{64}$/.test(url.searchParams.get('X-Amz-Signature') || ''));
    const expires = started + DOWNLOAD_TTL_SECONDS * 1000;
    requireValue(expires > Date.now());
    return { url: url.href, expiresAt: new Date(expires).toISOString() };
  } catch { /* Never propagate an SDK exception containing credentials or bearer URLs. */ }
  throw unavailable();
}
