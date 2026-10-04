#!/usr/bin/env python3
"""Probe-only native consumer of a caller-authenticated frozen runtime package."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

from openssl_package import _require_win32_pe

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def record(path):
    return {'sha256': sha(path), 'bytes': Path(path).stat().st_size}


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON key')
        result[key] = value
    return result


def read_json(path):
    require(Path(path).stat().st_size <= 16 * 1024 * 1024, 'oversized JSON')
    return json.loads(Path(path).read_text(encoding='utf-8-sig'), object_pairs_hook=unique)


def matches(path, expected):
    require(isinstance(expected, dict) and set(expected) == {'sha256', 'bytes'} and
            re.fullmatch('[a-f0-9]{64}', expected['sha256']) is not None and
            type(expected['bytes']) is int and expected['bytes'] > 0, 'invalid file record')
    require(Path(path).is_file() and not Path(path).is_symlink() and record(path) == expected,
            'missing or changed file: ' + str(path))


def validate_runtime(root, manifest_path, expected_hash, commit):
    require(re.fullmatch('[a-f0-9]{64}', expected_hash) is not None and
            re.fullmatch('[a-f0-9]{40}', commit) is not None, 'invalid expected identity')
    require(sha(manifest_path) == expected_hash, 'caller-bound package manifest changed')
    package = read_json(manifest_path)
    require(package.get('schema') == 1 and package.get('commit') == commit, 'wrong package identity')
    inventory = {}
    names = set()
    for item in package['files']:
        name = item['path']
        require(isinstance(name, str) and name.startswith(('app/', 'docs/')) and
                '\\' not in name and ':' not in name and '\x00' not in name and
                all(p not in ('', '.', '..') and p[-1] not in '. ' for p in name.split('/')) and
                name.casefold() not in names, 'unsafe or duplicate package path')
        path = root / name
        require(path.resolve().is_relative_to(root) and path.is_file() and not path.is_symlink(), 'runtime path escaped')
        require(re.fullmatch('[a-f0-9]{64}', item['sha256']) is not None and sha(path) == item['sha256'], 'runtime file changed')
        inventory[name] = item['sha256']
        names.add(name.casefold())
    actual = {p.relative_to(root).as_posix() for sub in ('app', 'docs') for p in (root / sub).rglob('*') if p.is_file()}
    require(actual == set(inventory), 'runtime contains missing or unlisted package files')
    product = read_json(root / 'docs/PRODUCT_BUILD.json')
    require(product.get('commit') == commit and product.get('test_fixtures') is False and
            product.get('build_purpose') == 'INSTALLED PRODUCT' and
            product.get('xnav_hardware_output_policy') == 'status-only' and
            sha(root / 'app/opencpn.exe') == product.get('executable_sha256'), 'product policy mismatch')
    return inventory


def validate_dependencies(app):
    manifests = {name: read_json(app / (name + '-build.json')) for name in ('curl', 'openssl', 'zlib')}
    for name, version in (('curl', '8.22.0'), ('openssl', '3.5.9'), ('zlib', '1.3.2')):
        m = manifests[name]
        lock = read_json(ROOT / 'tools' / ('windows-' + name + '.lock.json'))
        require(m.get('version') == version and m.get('architecture') == 'Win32' and m.get('abi') == 'x86' and
                m.get('source') == {k: lock[k] for k in ('url', 'archive', 'bytes', 'sha256', 'signingPrimaryFingerprint')} and
                all(m.get('buildSteps', {}).get(k) == 'passed' for k in ('configure', 'compile', 'test', 'install')),
                'wrong/incomplete producer identity: ' + name)
    curl = manifests['curl']
    require(curl.get('runtime') == 'MultiThreadedDLL (/MD)' and curl['buildSteps']['testsPassed'] == curl['buildSteps']['testsReported'] > 0,
            'curl producer tests/runtime mismatch')
    for name, version in (('openssl', '3.5.9'), ('zlib', '1.3.2')):
        dep = curl['dependencies'][name]
        require(dep['version'] == version and dep['manifestSha256'] == sha(app / (name + '-build.json')), 'curl dependency manifest binding failed')
    for name, library in (('libcurl.dll', 'curl'), ('libssl-3.dll', 'openssl'), ('libcrypto-3.dll', 'openssl'), ('zlib1.dll', 'zlib')):
        matches(app / name, manifests[library]['outputs']['bin/' + name])
        _require_win32_pe(app / name)
    return manifests


def fetch(url, dest, expected=None, limit=64 * 1024 * 1024):
    dest.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'}), timeout=60) as source, dest.open('xb') as out:
        total = 0
        while block := source.read(65536):
            total += len(block)
            require(total <= limit, 'download exceeded bound')
            out.write(block)
    if expected:
        matches(dest, expected)


def run(args, log, timeout=120):
    result = subprocess.run(list(map(str, args)), capture_output=True, text=True, timeout=timeout)
    log.write_text(result.stdout + result.stderr, encoding='utf-8')
    require(result.returncode == 0, 'command failed; see ' + str(log))
    return result.stdout


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir', type=Path, required=True)
    parser.add_argument('--package-manifest-path', type=Path, required=True)
    parser.add_argument('--expected-manifest-sha256', required=True)
    parser.add_argument('--expected-commit', required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--curl-import-lib', type=Path, help='optional exact producer import library; mismatch fails, never falls back')
    args = parser.parse_args()
    require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_OS') == 'Windows' and os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'disposable GitHub-hosted Windows required')
    runtime = args.runtime_dir.resolve()
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': 'isolated consumer proof; not producer-library or same-job reuse acceptance',
              'commit': args.expected_commit, 'packageManifestSha256': args.expected_manifest_sha256}
    try:
        inventory = validate_runtime(runtime, args.package_manifest_path, args.expected_manifest_sha256, args.expected_commit)
        app = runtime / 'app'
        manifests = validate_dependencies(app)
        report['runtimeFiles'] = inventory
        probe = module('plugin_guard', ROOT / 'tools/test-plugin-download-guard.py')
        tls = module('downloader_tls', ROOT / 'tools/test-downloader-trust.py')
        with tempfile.TemporaryDirectory(prefix='plugin-guard-native-', dir=os.environ['RUNNER_TEMP']) as raw:
            work = Path(raw)
            sdk = work / 'sdk'
            sdk.mkdir()
            wx = sdk / 'wx'
            seven = shutil.which('7z')
            require(seven is not None, '7z unavailable')
            for item in read_json(ROOT / 'tools/windows-wx.lock.json')['archives']:
                archive = sdk / item['file']
                fetch(item['url'], archive, {k: item[k] for k in ('sha256', 'bytes')})
                run([seven, 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
            for name in ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'):
                require(sha(wx / 'lib/vc14x_dll' / name) == sha(app / name), 'pinned wx SDK does not match candidate runtime')
            archive_root = sdk / 'archive'
            for item in read_json(ROOT / 'tools/windows-plugin-archive-sdk.lock.json')['files']:
                relative = PurePosixPath(item['path']).relative_to('buildwin')
                fetch(item['url'], archive_root / relative, {k: item[k] for k in ('sha256', 'bytes')})
            require(sha(archive_root / 'archive.dll') == sha(app / 'archive.dll'), 'pinned archive SDK does not match candidate runtime')
            _require_win32_pe(app / 'archive.dll')
            curl_lock = read_json(ROOT / 'tools/windows-curl.lock.json')
            curl_archive = sdk / curl_lock['archive']
            fetch(curl_lock['url'], curl_archive, {k: curl_lock[k] for k in ('sha256', 'bytes')})
            curl = sdk / 'curl'
            with tarfile.open(curl_archive) as tar:
                for name, rec in manifests['curl']['outputs'].items():
                    if not name.startswith('include/curl/'):
                        continue
                    require(re.fullmatch(r'include/curl/[a-z0-9_-]+\.h', name) is not None, 'unexpected curl header path')
                    info = tar.getmember('curl-8.22.0/' + name)
                    require(info.isfile() and info.size <= 1024 * 1024, 'unexpected curl source member')
                    path = curl / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(tar.extractfile(info).read())
                    matches(path, rec)
            source = work / 'source'
            for name in probe.EXPECTED:
                fetch('https://raw.githubusercontent.com/OpenCPN/OpenCPN/' + probe.PIN + '/' + name, source / name, limit=1024 * 1024)
            patch = Path(os.environ['ProgramFiles']) / 'Git/usr/bin/patch.exe'
            require(patch.is_file(), 'reviewed Git patch executable unavailable')
            report['source'] = probe.patch_and_slice(source, patch)
            # Retain exact generated source; no vendor/application build occurs.
            shutil.copytree(source, evidence / 'source')
            vswhere = Path(os.environ['ProgramFiles(x86)']) / 'Microsoft Visual Studio/Installer/vswhere.exe'
            vs = Path(run([vswhere, '-latest', '-products', '*', '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], evidence / 'vswhere.log').strip())
            version = (vs / 'VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt').read_text().strip()
            require(re.fullmatch(r'[0-9.]+', version) is not None, 'unexpected MSVC version')
            tool_dir = vs / 'VC/Tools/MSVC' / version / 'bin/Hostx64/x86'
            lib, dumpbin = tool_dir / 'lib.exe', tool_dir / 'dumpbin.exe'
            report['tools'] = {name: record(path) for name, path in (('lib.exe', lib), ('dumpbin.exe', dumpbin), ('patch.exe', patch))}
            curl_import = sdk / 'libcurl.lib'
            if args.curl_import_lib:
                matches(args.curl_import_lib, manifests['curl']['outputs']['lib/libcurl.lib'])
                shutil.copyfile(args.curl_import_lib, curl_import)
                report['importLibrary'] = {'kind': 'verified-producer', **record(curl_import)}
            else:
                exports = run([dumpbin, '/nologo', '/exports', app / 'libcurl.dll'], evidence / 'curl-exports.txt')
                rows = re.findall(r'^\s*\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]+\s+(curl_[A-Za-z0-9_]+)\s*$', exports, re.M)
                count = re.search(r'^\s*(\d+)\s+number of names\s*$', exports, re.M)
                require(count is not None and len(rows) == int(count[1]) and len(rows) == len(set(rows)) and len(rows) > 20,
                        'unknown, forwarded, decorated or ambiguous curl export table')
                definition = evidence / 'probe-only-libcurl.def'
                definition.write_text('LIBRARY libcurl.dll\nEXPORTS\n' + '\n'.join(sorted(rows)) + '\n', encoding='ascii')
                run([lib, '/nologo', '/machine:x86', '/def:' + str(definition), '/out:' + str(curl_import)], evidence / 'import-library-build.log')
                report['importLibrary'] = {'kind': 'probe-only-reconstructed-not-producer', 'sourceDll': record(app / 'libcurl.dll'),
                    'exports': record(evidence / 'curl-exports.txt'), 'definition': record(definition), 'generatedLibrary': record(curl_import),
                    'tool': record(lib), 'exportNames': sorted(rows)}
            shutil.copyfile(curl_import, evidence / 'probe-libcurl.lib')
            build = work / 'build'
            run(['cmake', '-S', ROOT / 'tests/plugin_download_guard', '-B', build, '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DSOURCE_DIR:PATH=' + source.as_posix(), '-DTOOLS_DIR:PATH=' + (ROOT / 'tools').as_posix(), '-DCURL_INCLUDE:PATH=' + (curl / 'include').as_posix(),
                 '-DCURL_IMPORT:FILEPATH=' + curl_import.as_posix(), '-DARCHIVE_INCLUDE:PATH=' + (archive_root / 'include').as_posix(), '-DARCHIVE_IMPORT:FILEPATH=' + (archive_root / 'archive.lib').as_posix(),
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(), '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(), '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
            run(['cmake', '--build', build, '--config', 'Release', '--parallel', '2'], evidence / 'build.log', timeout=300)
            execution = work / 'runtime'
            execution.mkdir()
            for dll in app.glob('*.dll'):
                shutil.copyfile(dll, execution / dll.name)
                require(sha(execution / dll.name) == inventory['app/' + dll.name], 'runtime copy changed')
            binary = execution / 'plugin-download-guard-probe.exe'
            shutil.copyfile(build / 'Release/plugin-download-guard-probe.exe', binary)
            report['probeExecutable'] = record(binary)
            shutil.copyfile(binary, evidence / binary.name)
            certs = work / 'certificates'
            certs.mkdir()
            key, ca = tls.make_ca(certs, 'plugin-probe-owned-ca')
            other_key, other_ca = tls.make_ca(certs, 'plugin-probe-untrusted-ca')
            valid_key, valid_cert = tls.make_leaf(certs, 'valid', key, ca, 'localhost')
            bad_key, bad_cert = tls.make_leaf(certs, 'bad', other_key, other_ca, 'localhost')
            receipt = work / 'owned-ca.txt'
            trust = ROOT / 'tools/plugin-probe-owned-trust.ps1'
            trust_args = ['pwsh', '-NoProfile', '-File', trust, '-Certificate', ca, '-Receipt', receipt]
            try:
                run(trust_args + ['-Operation', 'Import'], evidence / 'ca-import.log')
                probe.run_cases(binary, evidence, None, valid_cert, valid_key, bad_cert, bad_key, report)
            finally:
                if receipt.exists():
                    run(trust_args + ['-Operation', 'Remove'], evidence / 'ca-cleanup.log')
                    report['ownedCaCleanup'] = 'verified absent'
            require(validate_runtime(runtime, args.package_manifest_path, args.expected_manifest_sha256, args.expected_commit) == inventory,
                    'candidate runtime changed during probe')
            report['status'] = 'passed'
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        (evidence / 'native-results.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
