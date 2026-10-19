import test, { mock } from 'node:test';
import assert from 'node:assert/strict';
import { createHash, createHmac } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { AwsClient } from './vendor/aws4fetch-1.0.20/aws4fetch.esm.mjs';
import { signR2Download, R2_ACCOUNT_ID, R2_DOWNLOAD_ORIGIN, DOWNLOAD_BUCKET } from './r2-downloads.ts';

const env = { SKAGER_DOWNLOAD_BACKEND: 'r2', R2_ACCOUNT_ID, R2_DOWNLOAD_BUCKET: DOWNLOAD_BUCKET,
  R2_ACCESS_KEY_ID: 'f'.repeat(32), R2_SECRET_ACCESS_KEY: 'e'.repeat(64) };
const object = `staging/${'a'.repeat(40)}/fixture-beta-1/installer/${'1'.repeat(64)}/Setup.exe`;
const now = Date.parse('2026-10-14T12:00:00Z');
const originalSign = AwsClient.prototype.sign;
const generic = { message: 'Download unavailable' };

// Independent SigV4 verifier uses Node/OpenSSL, not the SDK's signing helpers.
function signatureFor(raw: string, method = 'GET') {
  const url = new URL(raw);
  const params = [...url.searchParams].filter(([key]) => key !== 'X-Amz-Signature');
  const encode = (value: string) => encodeURIComponent(value).replace(/[!'()*]/g,
    char => '%' + char.charCodeAt(0).toString(16).toUpperCase());
  const query = params.map(([key, value]) => [encode(key), encode(value)])
    .sort(([a, av], [b, bv]) => a < b ? -1 : a > b ? 1 : av < bv ? -1 : av > bv ? 1 : 0)
    .map(pair => pair.join('=')).join('&');
  const date = url.searchParams.get('X-Amz-Date')!;
  const scope = `${date.slice(0, 8)}/auto/s3/aws4_request`;
  const canonical = [method, url.pathname, query, `host:${url.host}\n`, 'host', 'UNSIGNED-PAYLOAD'].join('\n');
  const string = ['AWS4-HMAC-SHA256', date, scope, createHash('sha256').update(canonical).digest('hex')].join('\n');
  let key: Buffer = Buffer.from('AWS4' + env.R2_SECRET_ACCESS_KEY);
  for (const part of [date.slice(0, 8), 'auto', 's3', 'aws4_request']) key = createHmac('sha256', key).update(part).digest();
  return createHmac('sha256', key).update(string).digest('hex');
}

test('pinned SDK bytes and MIT notice match retained provenance', () => {
  const base = new URL('./vendor/aws4fetch-1.0.20/', import.meta.url);
  const manifest = JSON.parse(readFileSync(new URL('manifest.json', base), 'utf8'));
  assert.equal(manifest.version, '1.0.20'); assert.equal(manifest.modified, false);
  for (const [name, expected] of Object.entries(manifest.files)) {
    assert.equal(createHash('sha256').update(readFileSync(new URL(name, base))).digest('hex'), expected);
  }
  assert.match(readFileSync(new URL('LICENSE', base), 'utf8'), /Permission is hereby granted/);
});

test('GET presign matches independent known signature and binds object, TTL and attachment name', async () => {
  mock.timers.enable({ apis: ['Date'], now });
  try {
    const signed = await signR2Download(env, object, 'Setup.exe');
    const url = new URL(signed.url);
    assert.equal(url.origin, R2_DOWNLOAD_ORIGIN);
    assert.equal(url.pathname, `/${DOWNLOAD_BUCKET}/${object}`);
    assert.equal(url.searchParams.get('X-Amz-Date'), '20261014T120000Z');
    assert.equal(url.searchParams.get('X-Amz-Expires'), '60');
    assert.equal(signed.expiresAt, '2026-10-14T12:01:00.000Z');
    const signature = url.searchParams.get('X-Amz-Signature');
    // Also calculated independently with Python hashlib/hmac from this fixed fixture.
    assert.equal(signature, '75af70a972e6b4767fdc3c34d8260809c6ca5133f412a48885fc2a1d13a3e077');
    assert.equal(signatureFor(signed.url), signature);
    for (const method of ['HEAD', 'PUT', 'POST', 'DELETE']) assert.notEqual(signatureFor(signed.url, method), signature);
    for (const mutate of [
      (u: URL) => { u.pathname = u.pathname.replace('/installer/', '/source/'); },
      (u: URL) => { u.searchParams.set('X-Amz-Expires', '61'); },
      (u: URL) => { u.searchParams.set('response-content-disposition', 'attachment; filename="Other.exe"'); },
    ]) {
      const changed = new URL(signed.url); mutate(changed);
      assert.notEqual(signatureFor(changed.href), signature);
    }
    assert.equal(signed.url.includes(env.R2_SECRET_ACCESS_KEY), false);
  } finally { mock.timers.reset(); }
});

test('unconfigured backend, wrong account/bucket and malformed credentials cannot invoke SDK', async () => {
  let calls = 0;
  mock.method(AwsClient.prototype, 'sign', async () => { calls++; throw Error('SDK must not be called'); });
  try {
    for (const patch of [
      { SKAGER_DOWNLOAD_BACKEND: undefined }, { SKAGER_DOWNLOAD_BACKEND: 'supabase' }, { SKAGER_DOWNLOAD_BACKEND: 'R2' },
      { R2_ACCOUNT_ID: undefined }, { R2_ACCOUNT_ID: 'wrong-account' }, { R2_ACCOUNT_ID: R2_ACCOUNT_ID + '/' },
      { R2_DOWNLOAD_BUCKET: undefined }, { R2_DOWNLOAD_BUCKET: 'other-bucket' },
      { R2_ACCESS_KEY_ID: '' }, { R2_ACCESS_KEY_ID: 'f'.repeat(31) }, { R2_ACCESS_KEY_ID: 'f'.repeat(32) + '\n' },
      { R2_SECRET_ACCESS_KEY: undefined }, { R2_SECRET_ACCESS_KEY: 'e'.repeat(63) }, { R2_SECRET_ACCESS_KEY: 'x'.repeat(64) },
    ]) await assert.rejects(signR2Download({ ...env, ...patch }, object, 'Setup.exe'), generic);
    assert.equal(calls, 0);
  } finally { mock.restoreAll(); }
});

test('path aliases, queries, fragments, caller filenames and encoded traversal are refused before SDK', async () => {
  let calls = 0;
  mock.method(AwsClient.prototype, 'sign', async () => { calls++; throw Error('SDK must not be called'); });
  try {
    for (const path of [object + '?x=1', object + '#fragment', object.replace('/installer/', '//installer/'),
      object.replace('/installer/', '/source/../installer/'), object.replace('/installer/', '/%69nstaller/'),
      object.replace('/Setup.exe', '/%2e%2e%2fSetup.exe'), object.replace('staging/', 'production/'),
      object.replace('/Setup.exe', '/Setup..exe'), object.replace('/Setup.exe', '/Setup.exe.'),
      object.replace('/Setup.exe', '/Set up.exe'), object.replace('/Setup.exe', '/Setup+file.exe'),
      object.replace('/Setup.exe', '/Setup\\file.exe'), 'https://evil.invalid/' + object]) {
      await assert.rejects(signR2Download(env, path, path.split('/').at(-1)!), generic);
    }
    await assert.rejects(signR2Download(env, object, 'Different.exe'), generic);
    assert.equal(calls, 0);
  } finally { mock.restoreAll(); }
});

test('SDK output rejects changed origin/path/method, duplicate or extra query, credentials and invalid scope', async () => {
  mock.timers.enable({ apis: ['Date'], now });
  try {
    const good = await signR2Download(env, object, 'Setup.exe');
    for (const mutate of [
      (r: any, u: URL) => { u.hostname = 'evil.invalid'; },
      (r: any, u: URL) => { u.protocol = 'http:'; },
      (r: any, u: URL) => { u.username = 'credential'; },
      (r: any, u: URL) => { u.hash = 'fragment'; },
      (r: any, u: URL) => { u.pathname = u.pathname.replace('/installer/', '/source/'); },
      (r: any, u: URL) => { u.pathname = u.pathname.replace('/Setup.exe', '/%53etup.exe'); },
      (r: any, u: URL) => { u.search = u.search.replace('X-Amz-Date', '%58-Amz-Date'); },
      (r: any, u: URL) => { u.searchParams.append('X-Amz-Signature', 'duplicate'); },
      (r: any, u: URL) => { u.searchParams.set('redirect', 'https://evil.invalid'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Security-Token', 'unapproved'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Expires', '61'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Date', '20261014T115900Z'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Credential', 'wrong/20261014/auto/s3/aws4_request'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-SignedHeaders', 'content-type;host'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Algorithm', 'none'); },
      (r: any, u: URL) => { u.searchParams.set('X-Amz-Signature', 'not-a-signature'); },
      (r: any, u: URL) => { u.searchParams.set('response-content-disposition', 'inline'); },
      (r: any) => { r.method = 'PUT'; }, (r: any) => { r.body = 'upload'; },
      (r: any) => { r.headers.set('Authorization', env.R2_SECRET_ACCESS_KEY); },
    ]) {
      const returned = { method: 'GET', body: null, headers: new Headers(), url: good.url };
      const url = new URL(returned.url); mutate(returned, url); returned.url = url.href;
      mock.method(AwsClient.prototype, 'sign', async () => returned);
      await assert.rejects(signR2Download(env, object, 'Setup.exe'), generic);
      mock.restoreAll();
    }
  } finally { mock.restoreAll(); mock.timers.reset(); }
});

test('expired output and SDK exceptions are redacted without logging', async () => {
  const recorded: unknown[][] = [];
  for (const method of ['log', 'warn', 'error'] as const) mock.method(console, method, (...values) => recorded.push(values));
  mock.timers.enable({ apis: ['Date'], now });
  try {
    mock.method(AwsClient.prototype, 'sign', async function(input: Request, init: any) {
      const signed = await originalSign.call(this, input, init);
      mock.timers.tick(60000);
      return signed;
    });
    await assert.rejects(signR2Download(env, object, 'Setup.exe'), generic);
    mock.method(AwsClient.prototype, 'sign', async () => { throw Error(env.R2_SECRET_ACCESS_KEY + '?X-Amz-Signature=private'); });
    await assert.rejects(signR2Download(env, object, 'Setup.exe'), generic);
    assert.deepEqual(recorded, []);
  } finally { mock.restoreAll(); mock.timers.reset(); }
});
