// R2 (S3 API) helper for SKAGER releases: the update feed, the portal download list and
// the operator state. Credentials come only from the environment (GitHub secrets
// R2_RELEASE_ACCESS_KEY_ID / R2_RELEASE_SECRET_ACCESS_KEY) and are never printed.
//   node r2.mjs list <prefix>
//   node r2.mjs get <key> <local file>          (exit 3 when the key does not exist)
//   node r2.mjs put <local file> <key>          (skips if the same bytes are already stored)
//   node r2.mjs verify <local file> <key>
import { readFileSync, writeFileSync, createReadStream } from 'node:fs';
import { createHash } from 'node:crypto';
import { AwsClient } from '../../web/edge/vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';

const ACCOUNT = '7869adcf376c630b175c94729321e016';
const BUCKET = 'skager-private-releases';
const env = process.env;
if (!/^[0-9a-f]{32}$/.test(env.R2_RELEASE_ACCESS_KEY_ID ?? '') || !/^[0-9a-f]{64}$/.test(env.R2_RELEASE_SECRET_ACCESS_KEY ?? ''))
  throw new Error('R2 release credentials missing or malformed');
const client = new AwsClient({ accessKeyId: env.R2_RELEASE_ACCESS_KEY_ID, secretAccessKey: env.R2_RELEASE_SECRET_ACCESS_KEY, service: 's3', region: 'auto' });
const base = `https://${ACCOUNT}.r2.cloudflarestorage.com/${BUCKET}`;
const url = key => base + '/' + key.split('/').map(encodeURIComponent).join('/');
const sha = buf => createHash('sha256').update(buf).digest('hex');
const type = key => key.endsWith('.json') ? 'application/json' : key.endsWith('.md') ? 'text/markdown; charset=utf-8'
  : key.endsWith('.zip') ? 'application/zip' : key.endsWith('.exe') ? 'application/vnd.microsoft.portable-executable' : 'application/octet-stream';

async function stored(key) {
  const res = await client.fetch(url(key));
  if (res.status === 404) return null;
  if (!res.ok) throw new Error(`GET ${key}: ${res.status}`);
  const h = createHash('sha256'); let n = 0;
  for await (const chunk of res.body) { h.update(chunk); n += chunk.length; }
  return { sha256: h.digest('hex'), bytes: n };
}
const [cmd, a, b] = process.argv.slice(2);
if (cmd === 'get') {
  const res = await client.fetch(url(a));
  if (res.status === 404) { console.log(`missing ${a}`); process.exit(3); }
  if (!res.ok) throw new Error(`GET ${a}: ${res.status}`);
  const data = Buffer.from(await res.arrayBuffer());
  writeFileSync(b, data, { mode: 0o600 });
  console.log(`got ${a} (${data.length} bytes, sha256 ${sha(data).slice(0, 12)})`);
} else if (cmd === 'list') {
  let token = '', keys = [];
  do {
    const q = new URLSearchParams({ 'list-type': '2', prefix: a, ...(token ? { 'continuation-token': token } : {}) });
    const res = await client.fetch(`${base}?${q}`);
    const xml = await res.text();
    if (!res.ok) throw new Error(`list ${res.status}: ${xml.slice(0, 200)}`);
    for (const m of xml.matchAll(/<Contents>(.*?)<\/Contents>/g)) {
      const f = k => (new RegExp(`<${k}>([^<]*)</${k}>`).exec(m[1]) || [])[1];
      keys.push(`${f('Key')} ${f('Size')} ${f('LastModified')}`);
    }
    token = (/<NextContinuationToken>([^<]+)</.exec(xml) || [])[1] || '';
  } while (token);
  console.log(keys.join('\n') || '(empty)');
} else if (cmd === 'put' || cmd === 'verify') {
  const data = readFileSync(a), want = sha(data);
  const have = await stored(b);
  if (cmd === 'verify') { console.log(have && have.sha256 === want && have.bytes === data.length ? `ok ${b}` : `DIFFERS ${b}`); process.exit(have?.sha256 === want ? 0 : 1); }
  if (have && have.sha256 === want) { console.log(`already stored ${b}`); process.exit(0); }
  if (data.length <= 64 << 20) {
    const res = await client.fetch(url(b), { method: 'PUT', body: data, headers: { 'content-type': type(b), 'x-amz-content-sha256': want } });
    if (!res.ok) throw new Error(`PUT ${b}: ${res.status} ${(await res.text()).slice(0, 200)}`);
  } else {
    // Multipart: 64 MB parts, each retried, so a large file never needs one long request.
    const start = await client.fetch(url(b) + '?uploads', { method: 'POST', headers: { 'content-type': type(b) } });
    const xml = await start.text();
    const id = (/<UploadId>([^<]+)</.exec(xml) || [])[1];
    if (!start.ok || !id) throw new Error(`multipart start ${b}: ${start.status}`);
    const parts = [];
    for (let i = 0, n = 1; i < data.length; i += 64 << 20, n++) {
      const chunk = data.subarray(i, Math.min(i + (64 << 20), data.length));
      for (let attempt = 1; ; attempt++) {
        try {
          const res = await client.fetch(`${url(b)}?partNumber=${n}&uploadId=${encodeURIComponent(id)}`,
            { method: 'PUT', body: chunk, headers: { 'x-amz-content-sha256': sha(chunk) } });
          if (!res.ok) throw new Error(`part ${n}: ${res.status}`);
          parts.push(`<Part><PartNumber>${n}</PartNumber><ETag>${res.headers.get('etag')}</ETag></Part>`);
          console.log(`part ${n} of ${Math.ceil(data.length / (64 << 20))}`);
          break;
        } catch (e) { if (attempt === 4) throw e; await new Promise(r => setTimeout(r, 3000 * attempt)); }
      }
    }
    const done = await client.fetch(`${url(b)}?uploadId=${encodeURIComponent(id)}`, { method: 'POST',
      body: `<CompleteMultipartUpload>${parts.join('')}</CompleteMultipartUpload>`, headers: { 'content-type': 'application/xml' } });
    if (!done.ok) throw new Error(`multipart complete ${b}: ${done.status} ${(await done.text()).slice(0, 200)}`);
  }
  const check = await stored(b);
  if (!check || check.sha256 !== want) throw new Error(`readback differs ${b}`);
  console.log(`uploaded ${b} (${data.length} bytes, sha256 ${want.slice(0, 12)})`);
} else throw new Error('usage');
