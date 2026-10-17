import test from 'node:test';
import assert from 'node:assert/strict';
import { downloadSteps, downloadTarget, renderDashboard, validateDownloads } from './static/portal.js';

const artifact = { id: 'installer', name: 'SKAGER-Setup.exe', bytes: 1200, sha256: 'b'.repeat(64) };
const release = { id: 'staging-test', version: '0.4.0-beta2.5', sourceCommit: 'a'.repeat(40), channel: 'staging', artifacts: [artifact] };
const catalog = { available: true, releases: [release] };
const account = { betaAccess: 'active', account: { email: 'test@example.test' }, boats: [], orders: [], checkoutAvailable: false };
const now = Date.parse('2026-10-13T20:00:00Z');
const target = new URL(`https://7869adcf376c630b175c94729321e016.r2.cloudflarestorage.com/skager-private-releases/staging/${release.sourceCommit}/${release.id}/${artifact.id}/${artifact.sha256}/${artifact.name}`);
for (const [key, value] of Object.entries({ 'response-content-disposition': `attachment; filename="${artifact.name}"`,
  'X-Amz-Expires': '60', 'X-Amz-Date': '20261013T200000Z', 'X-Amz-Algorithm': 'AWS4-HMAC-SHA256',
  'X-Amz-Credential': 'f'.repeat(32) + '/20261013/auto/s3/aws4_request', 'X-Amz-SignedHeaders': 'host',
  'X-Amz-Signature': 'e'.repeat(64) })) target.searchParams.set(key, value);
const signed = {
  releaseId: release.id, artifactId: artifact.id, name: artifact.name, bytes: artifact.bytes, sha256: artifact.sha256,
  expiresAt: new Date(now + 60000).toISOString(), url: target.href,
};

test('downloads remain closed for missing catalog, disabled distribution and temporary failure', () => {
  for (const data of [undefined, null, { available: false }, { available: false, releases: [release] }]) {
    const result = validateDownloads(data);
    const html = renderDashboard(account, 'downloads', result);
    assert.match(html, /Release delivery is closed/);
    assert.doesNotMatch(html, /class="button button-secondary portal-download"/);
    assert.doesNotMatch(html, /SKAGER-Setup\.exe/);
    assert.match(html, /\/open-source\//);
  }
  const html = renderDashboard(account, 'downloads', { available: false, reason: 'Release information is temporarily unavailable.' });
  assert.match(html, /temporarily unavailable/);
});

test('authorized catalog shows exact release/source/file identity without pre-minted links', () => {
  const html = renderDashboard(account, 'downloads', validateDownloads(catalog));
  for (const value of [release.version, release.sourceCommit, artifact.name, artifact.sha256, '1,200 bytes']) assert.ok(html.includes(value));
  assert.match(html, /data-release-id="staging-test" data-artifact-id="installer"/);
  assert.doesNotMatch(html, /X-Amz-Signature|r2\.cloudflarestorage/);
  assert.match(html, /OpenCPN is installed separately/);
  assert.match(html, /open-source licenses/);
});

test('remote catalog markup is escaped and ambiguous or malformed selections are refused', () => {
  const special = { ...catalog, releases: [{ ...release, version: '<img src=x onerror=alert(1)>', artifacts: [{ ...artifact, name: '<script>alert(1)</script>' }] }] };
  const html = renderDashboard(account, 'downloads', validateDownloads(special));
  assert.doesNotMatch(html, /<img|<script/);
  assert.match(html, /&lt;script&gt;/);
  const badReleases = [
    { ...release, id: '" onclick="alert(1)' }, { ...release, sourceCommit: 'wrong' }, { ...release, channel: 'production' },
    { ...release, artifacts: [artifact, artifact] }, { ...release, artifacts: [{ ...artifact, bytes: Infinity }] },
    { ...release, artifacts: [{ ...artifact, sha256: '' }] },
  ];
  for (const row of badReleases) assert.throws(() => validateDownloads({ available: true, releases: [row] }));
  assert.throws(() => validateDownloads({ available: true, releases: [release, release] }));
  assert.throws(() => validateDownloads({ available: true, releases: [] }));
});

test('download navigation requires same selected identity and a short-lived fixed storage URL', () => {
  assert.equal(downloadTarget(signed, release, artifact, now), signed.url);
  for (const patch of [
    { releaseId: 'other' }, { artifactId: 'other' }, { name: 'changed.exe' }, { bytes: 2 }, { sha256: 'c'.repeat(64) },
    { expiresAt: new Date(now).toISOString() }, { expiresAt: new Date(now + 90001).toISOString() }, { expiresAt: 'invalid' },
    { url: 'javascript:alert(1)' }, { url: signed.url.replace('https:', 'http:') },
    { url: signed.url.replace('7869adcf376c630b175c94729321e016.r2.cloudflarestorage.com', 'untrusted.example.test') },
    { url: signed.url.replace('://', '://user@') }, { url: signed.url + '#fragment' },
    { url: signed.url.replace('skager-private-releases', 'other-bucket') },
    { url: signed.url.replace('X-Amz-Date', '%58-Amz-Date') },
    { url: signed.url + '&X-Amz-Signature=second' }, { url: signed.url + '&response-content-disposition=second.exe' },
    { url: signed.url.replace('X-Amz-Expires=60', 'X-Amz-Expires=61') },
    { url: signed.url.replace('SignedHeaders=host', 'SignedHeaders=content-type') },
    { url: signed.url.replace('/staging/', '/other-channel/') },
    { url: signed.url.replace('/installer/', '/source/') },
    { url: signed.url.replace('/staging-test/', '/wrong-release/') },
    { url: signed.url + '&redirect=https://untrusted.example.test' },
    { url: signed.url.replace('SKAGER-Setup.exe', 'changed.exe') },
  ]) assert.throws(() => downloadTarget({ ...signed, ...patch }, release, artifact, now));
});

test('download steps come one at a time before SKAGER; text is escaped and only https links are used', () => {
  const steps = downloadSteps({ steps: [
    { flow: 'download', title: 'Install <OpenCPN>', summary: 'First "this"', skipLabel: 'I already have OpenCPN installed',
      linkUrl: 'https://github.com/OpenCPN/OpenCPN/releases/tag/Release_5.12.4', linkLabel: 'Download OpenCPN' },
    { flow: 'checkout', title: 'GPS', skipLabel: 'I already have a GPS' },
    { flow: 'download', title: 'Charts', skipLabel: 'I have charts', linkUrl: 'javascript:alert(1)', linkLabel: 'Get charts' },
  ] });
  assert.equal(steps.length, 2);
  const first = renderDashboard(account, 'downloads', validateDownloads(catalog), { steps, current: 0, used: [] });
  assert.match(first, /Step 1 of 3/);
  assert.match(first, /Install &lt;OpenCPN&gt;/);
  assert.match(first, /href="https:\/\/github\.com\/OpenCPN\/OpenCPN\/releases\/tag\/Release_5\.12\.4" target="_blank" rel="noopener noreferrer"/);
  assert.match(first, /data-setup-step="1">I already have OpenCPN installed</);
  assert.doesNotMatch(first, /SKAGER-Setup\.exe|portal-download/);
  assert.match(renderDashboard(account, 'downloads', validateDownloads(catalog), { steps, current: 0, used: [0] }), /Done, next step →/);
  const second = renderDashboard(account, 'downloads', validateDownloads(catalog), { steps, current: 1, used: [] });
  assert.doesNotMatch(second, /javascript:/);
  assert.match(second, /data-setup-step="2">I have charts</);
  const last = renderDashboard(account, 'downloads', validateDownloads(catalog), { steps, current: 2, used: [] });
  assert.match(last, /Step 3 of 3/);
  assert.match(last, /data-artifact-id="installer"/);
  assert.match(renderDashboard(account, 'downloads', validateDownloads(catalog), { steps: [], current: 0, used: [] }), /data-artifact-id="installer"/);
});
