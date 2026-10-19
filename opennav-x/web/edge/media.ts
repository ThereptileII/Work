/** Product images in the private R2 media bucket, served same-origin through /api/media/. */
import { AwsClient } from './vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';
import { R2_ACCOUNT_ID, R2_DOWNLOAD_ORIGIN } from './r2-downloads.ts';

export const MEDIA_BUCKET = 'skager-media';
export const MEDIA_KEY = /^products\/[0-9a-f-]{36}\/[0-9a-f]{64}\.(png|jpg|webp)$/;
export const MAX_IMAGE_BYTES = 2 * 1024 * 1024;
const TYPES: Record<string, string> = { png: 'image/png', jpg: 'image/jpeg', webp: 'image/webp' };
type Env = Record<string, string | undefined>;

function client(env: Env): AwsClient {
  // This credential is scoped to the media bucket only, never the release bucket.
  if (env.R2_ACCOUNT_ID !== R2_ACCOUNT_ID || env.R2_MEDIA_BUCKET !== MEDIA_BUCKET
    || !/^[0-9a-f]{32}$/.test(env.R2_MEDIA_ACCESS_KEY_ID ?? '')
    || !/^[0-9a-f]{64}$/.test(env.R2_MEDIA_SECRET_ACCESS_KEY ?? '')) throw new Error('Media unavailable');
  return new AwsClient({ accessKeyId: env.R2_MEDIA_ACCESS_KEY_ID!, secretAccessKey: env.R2_MEDIA_SECRET_ACCESS_KEY!,
    service: 's3', region: 'auto', retries: 0 });
}

/** Recognise the image from its bytes; the declared type must agree. */
export function imageExtension(bytes: Uint8Array): 'png' | 'jpg' | 'webp' | null {
  const starts = (...values: number[]) => values.every((value, index) => bytes[index] === value);
  if (bytes.length > 8 && starts(0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a)) return 'png';
  if (bytes.length > 3 && starts(0xff, 0xd8, 0xff)) return 'jpg';
  if (bytes.length > 12 && starts(0x52, 0x49, 0x46, 0x46)
    && String.fromCharCode(...bytes.slice(8, 12)) === 'WEBP') return 'webp';
  return null;
}

export async function putImage(env: Env, productId: string, bytes: Uint8Array, fetcher: typeof fetch = fetch): Promise<string> {
  const extension = imageExtension(bytes);
  if (!extension || bytes.length > MAX_IMAGE_BYTES || !/^[0-9a-f-]{36}$/.test(productId)) throw new Error('Invalid image');
  const digest = [...new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))]
    .map(value => value.toString(16).padStart(2, '0')).join('');
  const key = `products/${productId}/${digest}.${extension}`;
  const signed = await client(env).sign(new Request(`${R2_DOWNLOAD_ORIGIN}/${MEDIA_BUCKET}/${key}`, {
    method: 'PUT', body: bytes, headers: { 'Content-Type': TYPES[extension], 'Cache-Control': 'public, max-age=31536000, immutable' },
  }));
  const response = await fetcher(signed, { signal: AbortSignal.timeout(15000), redirect: 'manual' });
  await response.body?.cancel().catch(() => {});
  if (response.status !== 200) throw new Error('Upload failed');
  return key;
}

export async function getImage(env: Env, key: string, fetcher: typeof fetch = fetch): Promise<Response> {
  const match = MEDIA_KEY.exec(key);
  if (!match) return new Response('Not found', { status: 404 });
  try {
    const signed = await client(env).sign(new Request(`${R2_DOWNLOAD_ORIGIN}/${MEDIA_BUCKET}/${key}`, { method: 'GET' }));
    const response = await fetcher(signed, { signal: AbortSignal.timeout(10000), redirect: 'manual' });
    if (response.status !== 200 || !response.body) {
      await response.body?.cancel().catch(() => {});
      return new Response('Not found', { status: 404 });
    }
    const length = Number(response.headers.get('content-length') || '0');
    if (length > MAX_IMAGE_BYTES) { await response.body.cancel().catch(() => {}); return new Response('Not found', { status: 404 }); }
    return new Response(response.body, { status: 200, headers: {
      'Content-Type': TYPES[match[1]], 'Cache-Control': 'public, max-age=86400', 'X-Content-Type-Options': 'nosniff',
    } });
  } catch { return new Response('Not found', { status: 404 }); }
}
