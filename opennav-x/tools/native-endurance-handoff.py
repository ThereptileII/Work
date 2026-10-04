#!/usr/bin/env python3
"""Closed same-run Win32 runtime transfer; never builds or launches an application."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import unicodedata

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'ThereptileII/Work'
PRODUCER = 'windows-integration'
CONSUMER = 'windows-endurance'
RUNTIME = 'build/xnav-install'
CONFIG = 'build/xnav-windows/include/config.h'
EXE = RUNTIME + '/opencpn.exe'
SOURCES = (
    'tools/native-endurance-handoff.py', 'tools/soak-runtime.py',
    'tools/diagnostic_snapshot.py', 'tools/windows-ui.py',
    'tools/chart-render-check.py', 'tools/prepare-test-profile.py',
    'tools/profile-fixtures.py', 'tests/fixtures/mode-persistence.gpx',
    'docs/design/prototype-tokens.json', 'release/qualification.json',
)
MAX_BYTES = 4 * 1024**3
PROFILE_NAMES = {'profile', 'profiles', 'opencpn.conf', 'opencpn.ini',
                 'navobj.xml', 'navobj.xml.changes', 'navobj.db',
                 'opennav_test_profile', 'opennav_portable_preview'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON key')
        result[key] = value
    return result


def read_json(path):
    plain(path)
    require(path.stat().st_size <= 16 * 1024**2, 'Oversized JSON')
    return json.loads(path.read_text(encoding='utf-8-sig'), object_pairs_hook=unique,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON number')))


def record(path):
    plain(path)
    require(path.is_file(), 'Required regular file missing: ' + str(path))
    with path.open('rb') as stream:
        return {'bytes': path.stat().st_size,
                'sha256': hashlib.file_digest(stream, 'sha256').hexdigest()}


def plain(path):
    for entry in (path, *path.parents):
        if os.path.lexists(entry):
            info = entry.lstat()
            require(not stat.S_ISLNK(info.st_mode) and
                    not (getattr(info, 'st_file_attributes', 0) & 0x400),
                    'Symlink/reparse path refused: ' + str(entry))


def safe_name(name):
    require(isinstance(name, str) and name and
            unicodedata.normalize('NFC', name) == name, 'Unsafe relative path')
    parts = name.split('/')
    require(str(PurePosixPath(name)) == name and not name.startswith('/') and
            all(part not in ('', '.', '..') and not part.endswith((' ', '.')) and
                not re.search(r'[\\:\x00-\x1f<>"|?*]', part) and
                not re.fullmatch(r'(?i)(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', part)
                for part in parts), 'Unsafe relative path')
    return name


def scan_tree(root):
    """Include empty directories, and reject Windows aliases even on Linux."""
    plain(root)
    require(root.is_dir(), 'Required directory missing: ' + str(root))
    files, directories, aliases = {}, [], set()
    total = 0
    for base, dirs, names in os.walk(root, followlinks=False):
        for name in sorted(dirs + names):
            path = Path(base) / name
            relative = safe_name(path.relative_to(root).as_posix())
            require(relative.casefold() not in aliases, 'Case-aliased path')
            aliases.add(relative.casefold())
            require(len(aliases) <= 100000, 'Too many runtime entries')
            plain(path)
            mode = path.lstat().st_mode
            if stat.S_ISDIR(mode):
                directories.append(relative)
            else:
                require(stat.S_ISREG(mode), 'Nonregular runtime entry')
                files[relative] = record(path)
                total += files[relative]['bytes']
                require(total <= MAX_BYTES, 'Runtime size exceeds bound')
    return dict(sorted(files.items())), sorted(directories)


def identity_values(repository, commit, run_id, run_attempt):
    require(repository == REPOSITORY and re.fullmatch(r'[0-9a-f]{40}', commit) and
            re.fullmatch(r'[1-9][0-9]{0,19}', run_id) and
            re.fullmatch(r'[1-9][0-9]{0,9}', run_attempt), 'Invalid CI identity')
    return {'repository': repository, 'commit': commit, 'run_id': run_id,
            'run_attempt': run_attempt, 'producer_job': PRODUCER, 'architecture': 'Win32'}


def ci_identity(root, command):
    expected_job = PRODUCER if command == 'prepare' else CONSUMER
    require(sys.platform == 'win32' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_OS') == 'Windows' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
            os.environ.get('GITHUB_JOB') == expected_job, 'Wrong native CI job')
    identity = identity_values(*(os.environ.get(key, '') for key in
        ('GITHUB_REPOSITORY', 'GITHUB_SHA', 'GITHUB_RUN_ID', 'GITHUB_RUN_ATTEMPT')))
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    require(head == identity['commit'], 'Checkout commit differs from CI identity')
    subprocess.run(['git', '-C', str(root), 'diff', '--exit-code', 'HEAD', '--', *SOURCES],
                   check=True, stdout=subprocess.DEVNULL)
    return identity


def source_records(root):
    return {name: record(root / name) for name in SOURCES}


def validate_records(records):
    require(isinstance(records, dict), 'Invalid file inventory')
    for name, value in records.items():
        safe_name(name)
        require(isinstance(value, dict) and set(value) == {'bytes', 'sha256'} and
                type(value['bytes']) is int and 0 <= value['bytes'] <= MAX_BYTES and
                isinstance(value['sha256'], str) and
                re.fullmatch(r'[0-9a-f]{64}', value['sha256']), 'Invalid size/hash record')


def restore_manifest_directories(stage, manifest):
    """Artifact transport omits empty directories; restore only closed safe records."""
    files, directories = manifest['files'], manifest['directories']
    require(isinstance(directories, list) and all(isinstance(n, str) for n in directories),
            'Invalid directory inventory')
    fixed = {'build', RUNTIME, 'build/xnav-windows', 'build/xnav-windows/include'}
    require(directories == sorted(set(directories)) and fixed <= set(directories),
            'Incomplete/duplicate directory inventory')
    aliases = set()
    for name in [*files, *directories]:
        safe_name(name)
        require(name.casefold() not in aliases, 'Case alias or file/directory collision')
        aliases.add(name.casefold())
        require(len(aliases) <= 100000, 'Too many handoff entries')
        require(name in fixed or name == CONFIG or name.startswith(RUNTIME + '/'),
                'Manifest path outside runtime/config roots')
        require(not any(part.casefold() in PROFILE_NAMES for part in name.split('/')),
                'User/portable profile entry refused')
        parent = str(PurePosixPath(name).parent)
        require(parent == '.' or parent in directories, 'Missing manifest parent directory')
    require(all(name == CONFIG or name.startswith(RUNTIME + '/') for name in files) and
            CONFIG in files and EXE in files, 'Invalid runtime file inventory')
    require(all(name in fixed or name.startswith(RUNTIME + '/') for name in directories),
            'Invalid runtime directory inventory')
    all_files, all_dirs = scan_tree(stage)
    expected_dirs = {'payload', *('payload/' + name for name in directories)}
    require(set(all_files) == {'manifest.json', *('payload/' + name for name in files)} and
            set(all_dirs) <= expected_dirs, 'Unlisted or missing handoff entry')
    for name in directories:
        path = stage / 'payload' / name
        plain(path)
        path.mkdir(parents=True, exist_ok=True)


def duration(root):
    seconds = read_json(root / 'release/qualification.json').get('enduranceSeconds')
    require(type(seconds) is int and 10800 <= seconds <= 21600, 'Invalid release endurance duration')
    return seconds


def win32_executable(path):
    with path.open('rb') as stream:
        header = stream.read(64)
        require(len(header) == 64 and header[:2] == b'MZ', 'Missing Win32 executable')
        offset = int.from_bytes(header[60:64], 'little')
        require(64 <= offset <= path.stat().st_size - 6, 'Invalid PE offset')
        stream.seek(offset)
        require(stream.read(6) == b'PE\0\0\x4c\x01', 'Executable is not Win32/x86')


def runtime_inventory(root):
    files, dirs = scan_tree(root / RUNTIME)
    require('opencpn.exe' in files, 'Missing runtime executable')
    for name in (*files, *dirs):
        require(not any(part.casefold() in PROFILE_NAMES for part in name.split('/')),
                'User/portable profile entry refused')
    result = {RUNTIME + '/' + name: value for name, value in files.items()}
    result[CONFIG] = record(root / CONFIG)
    require(result[CONFIG]['bytes'] > 0, 'Empty native build config')
    win32_executable(root / EXE)
    directories = ['build', RUNTIME, 'build/xnav-windows', 'build/xnav-windows/include']
    directories += [RUNTIME + '/' + name for name in dirs]
    return dict(sorted(result.items())), sorted(directories)


def write_new(path, value):
    plain(path)
    with path.open('x', encoding='utf-8', newline='\n') as output:
        json.dump(value, output, indent=2, sort_keys=True, allow_nan=False)
        output.write('\n')


def prepare(root, stage, identity):
    plain(stage)
    require(not os.path.lexists(stage), 'Stage must be new')
    require(not stage.resolve().is_relative_to((root / RUNTIME).resolve()) and
            not stage.resolve().is_relative_to((root / 'build/xnav-windows').resolve()),
            'Stage cannot be inside runtime inputs')
    files, directories = runtime_inventory(root)
    manifest = {'schema': 1, 'owner': 'SKAGER.NativeEnduranceHandoff.1',
                'identity': identity, 'required_seconds': duration(root),
                'sources': source_records(root), 'files': files, 'directories': directories}
    stage.mkdir(parents=True, exist_ok=False)
    payload = stage / 'payload'
    for name in directories:
        (payload / name).mkdir(parents=True, exist_ok=True)
    for name in files:
        shutil.copy2(root / name, payload / name)
    require(scan_tree(payload) == (files, directories), 'Runtime changed during handoff copy')
    require(runtime_inventory(root) == (files, directories) and
            source_records(root) == manifest['sources'], 'Producer inputs changed during handoff')
    write_new(stage / 'manifest.json', manifest)
    return manifest


def validate_stage(root, stage, identity):
    manifest = read_json(stage / 'manifest.json')
    require(set(manifest) == {'schema', 'owner', 'identity', 'required_seconds',
                              'sources', 'files', 'directories'} and
            type(manifest['schema']) is int and manifest['schema'] == 1 and
            manifest['owner'] == 'SKAGER.NativeEnduranceHandoff.1' and
            manifest['identity'] == identity, 'Handoff identity/schema mismatch')
    validate_records(manifest['sources'])
    validate_records(manifest['files'])
    require(manifest['sources'] == source_records(root), 'Harness/tool/fixture source mismatch')
    require(type(manifest['required_seconds']) is int and
            manifest['required_seconds'] == duration(root), 'Qualification duration mismatch')
    restore_manifest_directories(stage, manifest)
    files, directories = runtime_inventory(stage / 'payload')
    require(files == manifest['files'] and directories == manifest['directories'], 'Handoff payload changed')
    all_files, all_dirs = scan_tree(stage)
    require(set(all_files) == {'manifest.json', *('payload/' + n for n in files)} and
            all_dirs == sorted(['payload', *('payload/' + n for n in directories)]),
            'Unlisted handoff entry')
    return manifest


def verify(root, stage, identity):
    manifest = validate_stage(root, stage, identity)
    destinations = [root / RUNTIME, root / 'build/xnav-windows']
    for path in destinations:
        plain(path)
        require(not os.path.lexists(path), 'Consumer build destination must be absent')
    for name in manifest['directories']:
        (root / name).mkdir(parents=True, exist_ok=True)
    for name in manifest['files']:
        shutil.copy2(stage / 'payload' / name, root / name)
    verify_installed(root, manifest)
    return manifest


def verify_installed(root, manifest):
    require(runtime_inventory(root) == (manifest['files'], manifest['directories']),
            'Installed endurance runtime changed')
    config_files, config_dirs = scan_tree(root / 'build/xnav-windows')
    require(config_files == {'include/config.h': manifest['files'][CONFIG]} and
            config_dirs == ['include'], 'Unlisted consumer build input')
    require(source_records(root) == manifest['sources'], 'Endurance source closure changed')


def validate_report(report, manifest, identity):
    """Pure gate also usable by the promotion consumer; no elapsed-time inference."""
    require(manifest['identity'] == identity, 'Result identity mismatch')
    requested, elapsed = report.get('requested_seconds'), report.get('elapsed_seconds')
    required = manifest['required_seconds']
    require(type(required) is int and 10800 <= required <= 21600 and
            type(requested) is int and required <= requested <= 21600 and
            type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= requested,
            'Required actual endurance duration not met')
    require(report.get('status') == 'passed' and report.get('authority') == 'native Windows' and
            report.get('release_duration') is True and
            report.get('harness_commit') == identity['commit'] and
            report.get('build_commit') == identity['commit'] and
            report.get('binary_matches_harness_commit') is True and
            report.get('harness_sha256') == manifest['sources']['tools/soak-runtime.py']['sha256'],
            'Native endurance status/source identity rejected')
    executable = report.get('executable', {})
    require({key: executable.get(key) for key in ('bytes', 'sha256')} == manifest['files'][EXE] and
            type(executable.get('bytes')) is int, 'Soak executable identity mismatch')
    return {'status': 'passed', 'authority': 'native Windows', 'identity': identity,
            'consumer_job': CONSUMER, 'required_seconds': required,
            'requested_seconds': requested, 'elapsed_seconds': elapsed,
            'harness_commit': report['harness_commit'], 'build_commit': report['build_commit'],
            'harness_sha256': report['harness_sha256'], 'executable': manifest['files'][EXE],
            'binary_matches_harness_commit': True}


def result(root, stage, identity, report_path, output):
    manifest = validate_stage(root, stage, identity)
    verify_installed(root, manifest)
    receipt = validate_report(read_json(report_path), manifest, identity)
    receipt.update(schema=1, owner='SKAGER.NativeEnduranceResult.1', runtime_unchanged=True,
                   stage_manifest=record(stage / 'manifest.json'), report=record(report_path))
    write_new(output, receipt)
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'verify', 'result'))
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    identity = ci_identity(ROOT, args.command)
    stage = args.stage.absolute()
    if args.command == 'result':
        require(args.report is not None and args.output is not None, 'Result needs --report and --output')
        value = result(ROOT, stage, identity, args.report.absolute(), args.output.absolute())
    else:
        require(args.report is None and args.output is None, 'Report/output only apply to result')
        value = (prepare if args.command == 'prepare' else verify)(ROOT, stage, identity)
    print(json.dumps({'status': 'passed', 'command': args.command, 'identity': identity,
                      'required_seconds': value['required_seconds']}, sort_keys=True))


if __name__ == '__main__':
    main()
