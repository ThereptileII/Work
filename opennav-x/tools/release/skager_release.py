#!/usr/bin/env python3
"""Release one qualified Staging build to the three places it belongs.

A build reaches this script only after every Staging job passed. It is then:
  1. the update in SKAGER's auto-updater (signed update feed, beta channel),
  2. the download in the account portal at app.skager.app,
  3. a source release in the public repository (source, notes, checksums;
     the ready-to-install setup stays behind the account login).

Subcommands (the workflow calls them in this order):
  prepare  --staging DIR --out DIR --run-id N   verify the build, write policy, selection,
                                                portal entry and public release notes
  publish  --repository DIR --assets DIR --out DIR   sign the feed (keys from the environment)
  portal   --current FILE|- --entry FILE --out FILE  newest-first portal download list
Signing keys come only from SKAGER_UPDATE_{TARGETS,SNAPSHOT,TIMESTAMP}_KEY and reach
skager-repository on stdin; they are never written to disk or printed.
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import zipfile
from pathlib import Path

# Signed artifacts stay on the address every installed SKAGER trusts for updates
# (installer/windows/production-update-trust.json); customers use app.skager.app.
ORIGIN = 'https://beta.skager.app'
FEED_FILES = {  # policy role -> staging file name
    'releaseNotes': 'SKAGER-Beta2-Release-Notes.md',
    'installer': 'SKAGER-Beta2-Setup.exe',
    'recovery': 'SKAGER-Beta2-Portable-Recovery.zip',
    'correspondingSource': 'SKAGER-Beta2-source.zip',
    'notices': 'SOURCE_AND_LICENSES.md',
}
PORTAL_FILES = {  # portal artifact id -> staging file name
    'installer': 'SKAGER-Beta2-Setup.exe',
    'recovery': 'SKAGER-Beta2-Portable-Recovery.zip',
    'checksums': 'SHA256SUMS.txt',
    'source': 'SKAGER-Beta2-source.zip',
    'release-notes': 'SKAGER-Beta2-Release-Notes.md',
}
NOTICES = 'SOURCE_AND_LICENSES.md'  # taken from the verified recovery ZIP (docs/)
GATES = ('linux', 'windows', 'installer', 'package', 'restart')
COMMIT = re.compile(r'^[0-9a-f]{40}$')
VERSION = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.]+)?$')


def fail(message):
    sys.exit(f'release refused: {message}')


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def dump(value):
    return (json.dumps(value, indent=2) + '\n').encode()


def verify(staging: Path):
    """The exact download set Staging qualified: every gate passed and every byte matches."""
    try:
        release = json.loads((staging / 'RELEASE.json').read_text())
        qualification = json.loads((staging / 'QUALIFICATION.json').read_text())
    except (OSError, ValueError):
        fail('RELEASE.json or QUALIFICATION.json missing')
    commit, version = release.get('commit', ''), release.get('version', '')
    if not COMMIT.match(commit) or not VERSION.match(version):
        fail('release manifest has no exact commit and version')
    if qualification.get('commit') != commit or qualification.get('channel') != 'staging':
        fail('qualification belongs to another build')
    if any(qualification.get('gates', {}).get(g) != 'passed' for g in GATES):
        fail('not every Staging gate passed')
    # RELEASE.json (bound into the update selection) lists every file with size and hash;
    # SHA256SUMS.txt, where it lists a file, must agree with it.
    manifest = {f.get('name'): f for f in release.get('files', []) if isinstance(f, dict)}
    sums = {}
    for line in (staging / 'SHA256SUMS.txt').read_text().splitlines():
        m = re.match(r'^([0-9a-f]{64})  ([A-Za-z0-9._-]+)$', line.strip())
        if line.strip() and not m:
            fail('malformed SHA256SUMS.txt')
        if m:
            sums[m.group(2)] = m.group(1)
    for name in (set(FEED_FILES.values()) | set(PORTAL_FILES.values()) | {'QUALIFICATION.json'}) - {NOTICES}:
        entry, data = manifest.get(name), staging / name
        if not entry or not data.is_file() or sha(data) != entry.get('sha256') or data.stat().st_size != entry.get('size'):
            fail(f'{name} does not match RELEASE.json')
        if name in sums and sums[name] != entry['sha256']:
            fail(f'{name} differs between RELEASE.json and SHA256SUMS.txt')
    # The licence notices travel inside the verified recovery package; extract that exact file.
    with zipfile.ZipFile(staging / PORTAL_FILES['recovery']) as z:
        names = [n for n in z.namelist() if n.endswith('docs/' + NOTICES)]
        if len(names) != 1:
            fail('recovery package has no single docs/SOURCE_AND_LICENSES.md')
        (staging / NOTICES).write_bytes(z.read(names[0]))
    return release, qualification, commit, version


def record(staging, commit, name):
    data = staging / name
    path = f'artifacts/{commit}/{name}'
    return {'path': path, 'url': f'{ORIGIN}/{path}', 'sha256': sha(data), 'bytes': data.stat().st_size}


def prepare(args):
    staging, out = Path(args.staging), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    release, qualification, commit, version = verify(staging)
    compat = json.loads(Path(args.compatibility).read_text())
    if compat.get('openNavVersion') != version:
        fail(f'compatibility.json says {compat.get("openNavVersion")}, the build is {version}')
    supported = [{k: e[k] for k in ('version', 'arch', 'executableSha256', 'upstreamCommit')}
                 for e in compat['supportedOpenCpn']]
    policy = {'schema': 1, 'product': 'skager', 'channel': 'beta', 'version': version, 'commit': commit,
              **{role: record(staging, commit, name) for role, name in FEED_FILES.items()},
              'supportedOpenCpn': supported}
    policy_bytes = dump(policy)
    selection = {'schema': 1, 'version': version, 'commit': commit,
                 'releaseManifestSha256': sha(staging / 'RELEASE.json'),
                 'qualificationSha256': sha(staging / 'QUALIFICATION.json'),
                 'policySha256': hashlib.sha256(policy_bytes).hexdigest(),
                 'files': [{k: policy[role][k] for k in ('path', 'sha256', 'bytes')} for role in FEED_FILES]}
    (out / 'policy.json').write_bytes(policy_bytes)
    (out / 'selection.json').write_bytes(dump(selection))
    # Portal: the same files, served from the feed's own objects (artifacts/<commit>/<name>).
    entry = {'id': re.sub(r'[^a-z0-9]+', '-', version.lower()).strip('-')[:64], 'version': version,
             'channel': 'staging', 'sourceCommit': commit, 'location': 'feed',
             'qualification': {'record': f'actions/runs/{args.run_id}', 'sha256': sha(staging / 'QUALIFICATION.json')},
             'artifacts': [{'id': aid, 'name': name, 'bytes': (staging / name).stat().st_size,
                            'sha256': sha(staging / name)} for aid, name in PORTAL_FILES.items()]}
    (out / 'portal-entry.json').write_bytes(dump(entry))
    sums = (staging / 'SHA256SUMS.txt').read_text()
    notes = (staging / 'SKAGER-Beta2-Release-Notes.md').read_text()
    (out / 'public-notes.md').write_text(
        f'SKAGER {version} (beta), built from commit `{commit}` after passing every Staging check.\n\n'
        'This release carries the corresponding source, its licence notices and the release notes. '
        'The ready-to-install Windows setup is available with a SKAGER account at '
        '[app.skager.app](https://app.skager.app/portal/downloads/) and through the update in SKAGER.\n\n'
        f'SHA-256 of every file of this version, including the setup:\n\n```\n{sums.strip()}\n```\n\n---\n\n{notes}')
    (out / 'release.env').write_text(f'VERSION={version}\nCOMMIT={commit}\nTAG=v{version}\n')
    print(f'prepared {version} ({commit[:8]})')


def publish(args):
    keys = {role: os.environ.get(f'SKAGER_UPDATE_{role.upper()}_KEY', '').strip()
            for role in ('targets', 'snapshot', 'timestamp')}
    if not all(keys.values()):
        fail('update signing keys are not configured')
    out = Path(args.out)
    envelope = json.dumps({'schema': 1, 'keys': keys}, separators=(',', ':')).encode()
    keys.clear()
    run = subprocess.run([args.tool, 'publish', '--repository', args.repository, '--assets', args.assets,
                          '--policy', str(out / 'policy.json'), '--selection', str(out / 'selection.json'),
                          '--selection-sha256', sha(out / 'selection.json'), '--keys-stdin'],
                         input=envelope, capture_output=True)
    envelope = b''
    if run.returncode:
        # The tool's own messages never contain key material.
        sys.stderr.write(run.stderr.decode()[-2000:])
        fail('signing the update feed failed')
    print(run.stdout.decode()[-500:])


def portal(args):
    current = []
    if args.current != '-' and Path(args.current).is_file():
        current = json.loads(Path(args.current).read_text())
        if not isinstance(current, list):
            fail('portal list is not a list')
    entry = json.loads(Path(args.entry).read_text())
    # Apps (tools/release/publish_apps.py) keep their place; older Windows releases make room.
    apps = [r for r in current if 'app' in r]
    windows = [entry] + [r for r in current if r.get('id') != entry['id'] and 'app' not in r]
    releases = windows[:16 - len(apps)] + apps
    Path(args.out).write_bytes(dump(releases))
    print(f'portal list: {", ".join(r["version"] for r in releases)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--staging', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--run-id', required=True, type=int)
    p.add_argument('--compatibility', default=str(Path(__file__).resolve().parents[2] / 'installer/windows/compatibility.json'))
    p.set_defaults(func=prepare)
    p = sub.add_parser('publish')
    p.add_argument('--repository', required=True)
    p.add_argument('--assets', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--tool', default='skager-repository')
    p.set_defaults(func=publish)
    p = sub.add_parser('portal')
    p.add_argument('--current', required=True)
    p.add_argument('--entry', required=True)
    p.add_argument('--out', required=True)
    p.set_defaults(func=portal)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
