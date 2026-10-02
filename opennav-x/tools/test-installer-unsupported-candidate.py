#!/usr/bin/env python3
"""SCRUM-22: actual authenticated Setup rejection on disposable hosted Windows.

The caller authenticates the artifact; this consumer rechecks its bound bytes.
No installer/application compilation and no supported installation are performed.
"""
import argparse
import csv
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
import uuid

ROOT = Path(__file__).resolve().parents[1]
STOCK_SHA = '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
OFFICIAL_SHA = 'e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
OFFICIAL_URL = 'https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe'
MARKER = b'Disposable disconnected UI test.\n'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def plain(path):
    for part in (path, *path.parents):
        if part.exists() or part.is_symlink():
            require(not part.is_symlink() and not (getattr(part.lstat(), 'st_file_attributes', 0) & 0x400),
                    'Reparse/symlink path refused')


def snapshot(path):
    plain(path)
    if not path.exists():
        return None
    require(path.is_dir(), 'Expected directory')
    result = {}; total = 0
    for base, dirs, files in os.walk(path, followlinks=False):
        for name in dirs + files:
            item = Path(base) / name
            plain(item)
            total += 0 if item.is_dir() else item.stat().st_size
            require(total <= 512 * 1024 * 1024, 'Profile/tree byte snapshot exceeds bound')
            result[item.relative_to(path).as_posix()] = None if item.is_dir() else sha(item)
            require(len(result) <= 20000, 'Profile/tree snapshot exceeds bound')
    return result


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


class ProcessTreeTerminationError(RuntimeError):
    """A timed-out process tree may still reach the normal profile path."""


def normal_profile_path(raw, program_data):
    require(isinstance(raw, str) and raw and
            all(part not in ('.', '..') for part in raw.replace('\\', '/').split('/')),
            'Unexpected normal profile root')
    profile = Path(raw)
    plain(program_data)
    plain(profile)
    expected = program_data / 'opencpn'
    require(profile.is_absolute() and profile.resolve() == expected.resolve(),
            'Unexpected normal profile root')
    return profile


@contextmanager
def preserve_profile(profile, report):
    original = snapshot(profile)
    backup = profile.with_name(profile.name + '.scrum22-backup-' + uuid.uuid4().hex)
    require(not os.path.lexists(backup), 'Backup already exists')
    if original is not None:
        profile.rename(backup)  # Same-volume rename preserves original metadata/ACLs.
        report['originalProfileBackup'] = str(backup)
    report['originalProfilePresent'] = original is not None
    safe_to_restore = True
    try:
        yield
    except ProcessTreeTerminationError:
        safe_to_restore = False
        raise
    finally:
        if not safe_to_restore:
            # Never expose the original profile to a possibly running Setup.
            report['profileRestoration'] = 'unverified; process tree termination failed; backup retained'
        else:
            if original is not None:
                require(snapshot(backup) == original, 'Original profile backup changed; retained')
            if os.path.lexists(profile):
                # Even a malformed fixture or redirected descendant is retained,
                # not recursively deleted. Rename the entry without traversing it.
                plain(profile.parent)
                quarantine = profile.with_name(profile.name + '.scrum22-quarantine-' + uuid.uuid4().hex)
                require(not os.path.lexists(quarantine), 'Quarantine already exists')
                profile.rename(quarantine)
                report['quarantinedProfile'] = str(quarantine)
            if original is not None:
                backup.rename(profile)
            require(snapshot(profile) == original, 'Original profile restoration mismatch')
            report['profileRestoration'] = 'verified'


def run(command, log, timeout=60, report=None):
    with log.open('wb') as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            return child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            event = {'phase': log.name, 'pid': child.pid, 'timeoutSeconds': timeout,
                     'processTreeTermination': 'unverified'}
            if report is not None:
                report.setdefault('processTimeouts', []).append(event)
            try:
                killed = subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
                event['taskkillExitCode'] = killed.returncode
                require(killed.returncode == 0, 'Process tree termination was not confirmed')
                child.wait(timeout=30)
                event['processTreeTermination'] = 'verified'
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                raise ProcessTreeTerminationError('Timed-out process tree may still be running') from error
            raise


def registry_state():
    import winreg
    def tree(hive, path, view):
        try:
            key = winreg.OpenKey(hive, path, 0, winreg.KEY_READ | view)
        except FileNotFoundError:
            return None
        with key:
            children, values, _ = winreg.QueryInfoKey(key)
            require(children + values <= 5000, 'Registry snapshot exceeds bound')
            return {'values': sorted((name, kind, repr(value)) for name, value, kind in
                                     (winreg.EnumValue(key, i) for i in range(values))),
                    'children': {name: tree(hive, path + '\\' + name, view) for name in
                                 (winreg.EnumKey(key, i) for i in range(children))}}
    uninstall = r'Software\Microsoft\Windows\CurrentVersion\Uninstall'
    result = {'userUninstall': tree(winreg.HKEY_CURRENT_USER, uninstall, 0),
              'userOpenCPN': tree(winreg.HKEY_CURRENT_USER, r'Software\OpenCPN', 0)}
    for view in (winreg.KEY_WOW64_32KEY, winreg.KEY_WOW64_64KEY):
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, uninstall, 0, winreg.KEY_READ | view) as key:
            names = [winreg.EnumKey(key, i) for i in range(winreg.QueryInfoKey(key)[0])]
        result[str(view)] = {name: tree(winreg.HKEY_LOCAL_MACHINE, uninstall + '\\' + name, view)
                             for name in names if name.lower().startswith(('opencpn', 'opennav', 'skager'))}
    return result


def assert_closed():
    output = subprocess.check_output(['tasklist', '/FI', 'IMAGENAME eq opencpn.exe', '/FO', 'CSV', '/NH'],
                                     timeout=30).decode(errors='replace')
    require(not any(row and row[0].lower() == 'opencpn.exe' for row in csv.reader(output.splitlines())),
            'OpenCPN process present before normal-profile isolation')


def main(args):
    started = time.monotonic()
    require(sys.platform == 'win32' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'Disposable hosted Windows required')
    require(not os.environ.get('GH_TOKEN') and not os.environ.get('GITHUB_TOKEN'), 'Actions token must not reach Setup')
    args.evidence.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'commit': args.commit, 'setupSha256': args.setup_sha256,
              'packageManifestSha256': args.manifest_sha256, 'testSourceSha256': sha(Path(__file__)),
              'processLimitsSeconds': {'loaderAndPreparation': 60, 'setup': 180, 'treeKill': 30, 'reap': 30},
              'registryScope': 'HKCU uninstall and OpenCPN settings; stock/product HKLM uninstall in both views',
              'scope': 'Unsupported-build preservation only; not clean-install or release acceptance'}
    try:
        report['sourceFilesSha256'] = {name: sha(ROOT / name) for name in (
            'tools/test-installer-unsupported-candidate.py', 'tools/test-plugin-download-guard-windows.py',
            'tools/prepare-test-profile.py', 'tools/profile-fixtures.py', 'tests/fixtures/mode-persistence.gpx')}
        validator = module('test-plugin-download-guard-windows')
        validator.validate_runtime(args.runtime.resolve(), args.manifest, args.manifest_sha256, args.commit)
        require(sha(args.setup) == args.setup_sha256, 'Authenticated Setup bytes changed')
        require(sha(args.runtime / 'profile-adapter/include/config.h') == args.adapter_sha256, 'Profile adapter changed')
        roots = [Path(os.environ['LOCALAPPDATA']) / 'OpenNavXAlpha1']
        programs = Path(os.environ['APPDATA']) / 'Microsoft/Windows/Start Menu/Programs'
        roots += [programs / name for name in ('SKAGER', 'OpenNav X', 'OpenNav X Alpha 1')]
        require(all(snapshot(p) is None for p in roots), 'Runner already contains installation/shortcut roots')
        registry_before = registry_state()
        assert_closed()
        with tempfile.TemporaryDirectory(prefix='scrum22-', dir=os.environ['RUNNER_TEMP']) as raw:
            work = Path(raw)
            location = work / 'locations.json'
            require(run([str(args.runtime / 'app/opencpn.exe'), '--opennav-self-test', str(location)],
                        args.evidence / 'selftest.log', report=report) == 0, 'Candidate loader self-test failed')
            info = validator.read_json(location)
            require(info.get('contract') == 'OpenNavX.LoaderSelfTest.1' and
                    info.get('profile_initialized') is False and info.get('plugins_loaded') is False and
                    info.get('passed') is True and info.get('commit') == args.commit and
                    info.get('test_fixtures') is False and info.get('xnav_hardware_output_policy') == 'status-only',
                    'Running candidate identity/policy mismatch')
            profile = normal_profile_path(info['normal_config_directory'], Path(os.environ['PROGRAMDATA']))
            official = work / 'official.exe'
            request = urllib.request.Request(OFFICIAL_URL, headers={'User-Agent': 'Mozilla/5.0'})
            download_started = time.monotonic()
            with urllib.request.urlopen(request, timeout=60) as incoming, official.open('xb') as output:
                total = 0
                while block := incoming.read(65536):
                    total += len(block)
                    require(total <= 256 * 1024 * 1024 and time.monotonic() - download_started <= 120,
                            'Official archive exceeds byte/time bound')
                    output.write(block)
            require(sha(official) == OFFICIAL_SHA, 'Official stock archive mismatch')
            listing = subprocess.check_output([str(args.seven), 'l', '-slt', str(official)],
                                              timeout=60).decode('utf-8', 'strict')
            require('Type = Nsis' in listing, 'Official stock archive is not NSIS')
            members = [line[7:].replace('\\', '/') for line in listing.splitlines() if line.startswith('Path = ')]
            require(members.count('opencpn.exe') == 1 and
                    sum(Path(name).name.casefold() == 'opencpn.exe' for name in members) == 1,
                    'Official executable member is missing, ambiguous or at an unexpected path')
            listing_path = args.evidence / 'official-nsis-listing.txt'
            listing_path.write_text(listing, encoding='utf-8')
            report['officialArchive'] = {'setupSha256': OFFICIAL_SHA, 'executableSha256': STOCK_SHA,
                                         'member': 'opencpn.exe', 'listingSha256': sha(listing_path)}
            # Extract one real official executable, without executing stock Setup
            # or changing its registrations; its exact accepted hash is required.
            bad = work / 'unsupported'; bad.mkdir()
            with (bad / 'opencpn.exe').open('xb') as output:
                subprocess.run([str(args.seven), 'e', '-so', str(official), 'opencpn.exe'],
                               stdout=output, stderr=subprocess.PIPE, check=True, timeout=60)
            require(sha(bad / 'opencpn.exe') == STOCK_SHA, 'Extracted official executable mismatch')
            with (bad / 'opencpn.exe').open('ab') as output:
                output.write(b'SCRUM-22 unsupported build\n')
            (bad / 'user-owned').mkdir(); (bad / 'user-owned/keep.txt').write_bytes(b'Preserve this file.\n')
            bad_before = snapshot(bad)
            assert_closed()
            # Refuse a slow prerequisite phase before moving a real profile.
            # Child waits own termination/cleanup; the parent must not kill this
            # helper on a separate deadline and bypass its restoration finally.
            require(time.monotonic() - started < 240, 'Insufficient remaining profile-restoration budget')
            with preserve_profile(profile, report):
                require(run([sys.executable, str(ROOT / 'tools/prepare-test-profile.py'), '--build',
                             str(args.runtime / 'profile-adapter'), '--profile', str(profile)],
                            args.evidence / 'profile-prepare.log', report=report) == 0, 'Test profile preparation failed')
                module('profile-fixtures').seed(profile)
                shutil.copyfile(profile / 'opencpn.conf', profile / 'opencpn.ini')
                profile_before = snapshot(profile)
                result = args.evidence / 'setup-rejection.json'
                require(all('"' not in str(v) for v in (args.setup, bad, result)), 'Unsafe command argument')
                command = subprocess.list2cmdline([str(args.setup)]) + ' /S /ACTION=Install /OPENCPN="' + str(bad / 'opencpn.exe') + '" /REPORT="' + str(result) + '"'
                require(run(command, args.evidence / 'setup.log', timeout=180, report=report) == 1, 'Setup did not reject unsupported input')
                failure = validator.read_json(result)
                require(failure.get('status') == 'failed' and 'Unsupported OpenCPN executable.' in failure.get('error', ''),
                        'Setup failed for a different reason')
                require(snapshot(bad) == bad_before, 'Rejected installation tree changed')
                require(snapshot(profile) == profile_before, 'Existing shared test profile changed')
                require(all(snapshot(p) is None for p in roots), 'Install/shortcut root created during rejection')
                require(registry_state() == registry_before, 'Installer or stock registry state changed')
                report.update(rejectedTreeSha256=digest(bad_before), profileSha256=digest(profile_before),
                              registrySha256=digest(registry_before), installAndShortcutRootsAbsent=True,
                              rejection='unsupported executable hash')
        require(sha(args.setup) == args.setup_sha256, 'Setup changed during probe')
        validator.validate_runtime(args.runtime.resolve(), args.manifest, args.manifest_sha256, args.commit)
        report['status'] = 'passed'
    finally:
        (args.evidence / 'unsupported-results.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('setup', 'runtime', 'manifest', 'seven', 'evidence'):
        parser.add_argument('--' + name, type=Path, required=True)
    for name in ('setup-sha256', 'manifest-sha256', 'adapter-sha256', 'commit'):
        parser.add_argument('--' + name, required=True)
    main(parser.parse_args())
