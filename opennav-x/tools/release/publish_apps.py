#!/usr/bin/env python3
"""Adds apps to the account downloads next to SKAGER for Windows.

Apps are SKAGER Companion and SKAGER Helm for Android (APKs signed with the owner's
upload keys, on the owner's computer) and SKAGER for Raspberry Pi (the .deb from
.github/workflows/skager-raspberry-pi.yml). Each app keeps one entry, its newest.

  publish_apps.py --current current.json --out portal.json --stage DIR \\
      --app companion-android=FILE,VERSION,COMMIT,RUN [--app ...]

DIR receives every file under its storage key (staging/<commit>/<id>/<kind>/<sha256>/<name>,
web/edge/downloads.ts objectName) for upload to the private bucket; portal.json replaces
portal/beta-releases.json. Upload the files first, then the list.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

KIND = {'companion-android': 'android', 'helm-android': 'android', 'raspberry-pi': 'linux'}
EXTENSION = {'android': '.apk', 'linux': '.deb'}
LIMIT = 16


def entry(app, spec, stage):
    file, version, commit, run = spec.split(',')
    path = Path(file)
    kind = KIND[app]
    if path.suffix != EXTENSION[kind]:
        raise SystemExit(f'{app}: expected a {EXTENSION[kind]} file, got {path.name}')
    if not re.fullmatch(r'[0-9a-f]{40}', commit) or not re.fullmatch(r'[1-9][0-9]{0,19}', run):
        raise SystemExit(f'{app}: commit must be 40 hex digits and run a GitHub run id')
    if not re.fullmatch(r'[0-9A-Za-z][0-9A-Za-z.+_-]{0,63}', version):
        raise SystemExit(f'{app}: version {version!r} is not allowed in the download list')
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    release_id = f'{app}-' + re.sub(r'[^a-z0-9_-]+', '-', version.lower()).strip('-')
    key = f'staging/{commit}/{release_id}/{kind}/{digest}/{path.name}'
    target = Path(stage) / key
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(path, target)
    return {'id': release_id, 'sourceCommit': commit, 'version': version, 'channel': 'staging', 'app': app,
            'qualification': {'record': f'actions/runs/{run}', 'sha256': digest},
            'artifacts': [{'id': kind, 'name': path.name, 'bytes': len(data), 'sha256': digest}]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--current', required=True, help="the published list ('-' if none)")
    parser.add_argument('--out', required=True)
    parser.add_argument('--stage', required=True)
    parser.add_argument('--app', action='append', required=True, metavar='APP=FILE,VERSION,COMMIT,RUN')
    args = parser.parse_args()
    current = json.loads(Path(args.current).read_text()) if args.current != '-' else []
    if not isinstance(current, list):
        raise SystemExit('the published list is not a list')
    added = []
    for item in args.app:
        app, _, spec = item.partition('=')
        if app not in KIND:
            raise SystemExit(f'unknown app {app!r}; one of {", ".join(KIND)}')
        added.append(entry(app, spec, args.stage))
    apps = {e['app'] for e in added}
    # SKAGER for Windows first (newest first, as the release workflow keeps them), then the apps.
    kept = [r for r in current if r.get('app') not in apps]
    releases = [r for r in kept if 'app' not in r] + [r for r in kept if 'app' in r] + added
    if len(releases) > LIMIT:
        raise SystemExit(f'the list would hold {len(releases)} entries; the portal accepts {LIMIT}')
    Path(args.out).write_text(json.dumps(releases, indent=2) + '\n')
    for e in added:
        print(f"{e['id']}: {e['artifacts'][0]['name']} {e['artifacts'][0]['sha256']}")


if __name__ == '__main__':
    main()
