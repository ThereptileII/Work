import test from 'node:test';
import assert from 'node:assert/strict';
import { downloadTarget, renderDashboard, validateDownloads } from './static/portal.js';

const artifact = { id: 'installer', name: 'SKAGER-Setup.exe', bytes: 1200, sha256: 'b'.repeat(64) };
const release = { id: 'staging-test', version: '0.4.0-beta2.5', sourceCommit: 'a'.repeat(40), channel: 'staging', artifacts: [artifact] };
const catalog = { available: true, releases: [release] };
const account = { betaAccess: 'active', account: { email: 'test@example.test' }, boats: [], orders: [], checkoutAvailable: false };
const now = Date.parse('2026-10-13T20:00:00Z');
const signed = {
  releaseId: release.id, artifactId: artifact.id, name: artifact.name, bytes: artifact.bytes, sha256: artifact.sha256,
  expiresAt: new Date(now + 60000).toISOString(),
  url: 'https://kelwgubkuvditalsbqpr.supabase.co/storage/v1/object/sign/skager-private-releases/release/installer?token=synthetic-token&download=SKAGER-Setup.exe',
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
  assert.doesNotMatch(html, /token=|object\/sign/);
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
    { url: signed.url.replace('kelwgubkuvditalsbqpr.supabase.co', 'untrusted.example.test') },
    { url: signed.url.replace('://', '://user@') }, { url: signed.url + '#fragment' },
    { url: signed.url.replace('skager-private-releases', 'other-bucket') },
    { url: signed.url + '&token=second' }, { url: signed.url + '&download=second.exe' },
    { url: signed.url + '&redirect=https://untrusted.example.test' },
    { url: signed.url.replace('SKAGER-Setup.exe', 'changed.exe') },
  ]) assert.throws(() => downloadTarget({ ...signed, ...patch }, release, artifact, now));
});
