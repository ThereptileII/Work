#!/usr/bin/env python3
"""Fetch the exact accepted Beta 1 installer for a real Beta-1-to-Beta-2 upgrade.

No source rewriting or version relabeling. The archive and extracted installer
are hash-pinned, and an expired/missing artifact fails the qualification gate.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import urllib.error
import urllib.request
import zipfile

if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
    raise SystemExit('Prior-version fixture requires disposable native CI')
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--early-beta2-layout', action='store_true',
                    help='Exact early Beta 2 historical-layout regression fixture, never a release allowlist')
args = parser.parse_args()
fixture = 'early Beta 2 layout' if args.early_beta2_layout else 'accepted Beta 1'
lock_name = 'early-beta2-layout.lock.json' if args.early_beta2_layout else 'accepted-beta1.lock.json'
lock = json.loads((root / 'tools' / lock_name).read_text())
output = root / ('build/prior-beta2-layout-fixture' if args.early_beta2_layout else 'build/prior-alpha-fixture')
output.mkdir()
api = 'https://api.github.com/repos/ThereptileII/Work/actions/artifacts/' + str(lock['artifactId'])
token = os.environ.get('OPENNAV_ARTIFACT_TOKEN')
if not token:
    raise SystemExit('Read-only Actions token required for the fixed prior installer fixture')
headers = {'Accept': 'application/vnd.github+json', 'User-Agent': 'OpenNav-qualification',
           'Authorization': 'Bearer ' + token, 'X-GitHub-Api-Version': '2022-11-28'}
with urllib.request.urlopen(urllib.request.Request(api, headers=headers), timeout=60) as response:
    metadata = json.load(response)
assert not metadata['expired'], 'Prior installer artifact expired; restore exact hash-pinned archive'
assert metadata['workflow_run']['head_sha'] == lock['commit']
assert metadata['workflow_run']['id'] == lock['runId']
assert metadata['digest'] == 'sha256:' + lock['archiveSha256']
# Never forward GitHub authorization to the signed blob-storage redirect.
class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, hdrs, newurl):
        return None
try:
    urllib.request.build_opener(NoRedirect).open(
        urllib.request.Request(api + '/zip', headers=headers), timeout=60)
except urllib.error.HTTPError as redirect:
    if redirect.code != 302:
        raise SystemExit('Prior installer archive request failed: HTTP ' + str(redirect.code)) from None
    location = redirect.headers['Location']
else:
    raise SystemExit('Expected authenticated artifact redirect')
assert location.startswith('https://'), 'Artifact transport must use TLS'
archive = output / 'verified-prior-installer.zip'
with urllib.request.urlopen(location, timeout=120) as response, archive.open('wb') as target:
    remaining = lock['maximumArchiveBytes']
    while block := response.read(1024 * 1024):
        remaining -= len(block)
        if remaining < 0:
            raise SystemExit('Prior installer archive exceeds its bound')
        target.write(block)
assert hashlib.sha256(archive.read_bytes()).hexdigest() == lock['archiveSha256']
setup = output / 'setup' / lock['setupName']
setup.parent.mkdir()
with zipfile.ZipFile(archive) as source:
    entry = source.getinfo(lock['setupName'])
    assert entry.file_size == lock['setupBytes']
    data = source.read(entry)
    assert hashlib.sha256(data).hexdigest() == lock['setupSha256']
    setup.write_bytes(data)
evidence_name = 'installer-prior-beta2-layout.json' if args.early_beta2_layout else 'installer-prior-release.json'
(root / 'evidence/local' / evidence_name).write_text(json.dumps({
    'source': lock, 'verified': True,
    'scope': 'Exact '+fixture+' installer; subsequent real upgrade and rollback gates required'
}, indent=2) + '\n')
print('Exact '+fixture+' installer verified; candidate executable and source untouched')
