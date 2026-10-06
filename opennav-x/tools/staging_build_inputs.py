#!/usr/bin/env python3
"""Seal compiled Staging inputs before UI qualification; restore without building.

The supplied archive digest and producer identity must come from authenticated
Actions metadata/job outputs. A manifest inside an untrusted ZIP is not authority.
These receipts prove retained input integrity, never Staging/Production acceptance.
"""
import argparse
from hardware_output_policy import require_product_output_policy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import shutil
import stat
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile

ARCHIVE = 'STAGING_BUILD_INPUTS.zip'
MANIFEST = 'STAGING_BUILD_INPUTS.json'
KIND = 'skager-staging-compiled-inputs'
MAX_FILES = 100000
MAX_FILE = 4 * 1024**3
MAX_TOTAL = 12 * 1024**3
MAX_MANIFEST = 32 * 1024**2
INSTALLS = ('build/xnav-install', 'build/production-install')
VARIANTS = ('xnav', 'production')
PACKAGE_ROOT = 'build/developer-preview/SKAGER-Beta2-Portable-Recovery'
FEEDBACK_MANIFEST = 'build/xnav-windows/Release/boat-feedback-tests.json'
FEEDBACK_TESTS = {
    'chart_info_tests', 'pilot_status_tests', 'anchor_route_transition_tests',
    'navigation_naming_tests', 'route_context_tests', 'chart_info_drawer_test',
    'navigation_name_editor_test', 'route_context_card_test',
    'chart_light_hover_tests', 'ais_drawer_scroll_test', 'online_ais_radius_test',
    'chart_anchor_watch_renderer_test', 'route_activation_callbacks_test',
}
FEEDBACK_BINARIES = {name: 'build/xnav-windows/Release/' + name + '.exe'
                     for name in FEEDBACK_TESTS}
FIXED_FILES = {
    FEEDBACK_MANIFEST,
    'build/beta-installer/SKAGER-Beta2-Setup.exe',
    'build/beta-installer/package.json', 'build/beta-installer/payload.zip',
    'build/beta-installer/SKAGER-Beta2-Setup.exe.sha256',
    'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip',
    'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip.sha256',
    'build/developer-preview/SKAGER-Beta2-source.zip',
    'build/developer-preview/production-package-selftest.json',
    'build/developer-preview/production-restart-selftest.json',
    'evidence/local/windows-xnav-tests.xml', 'evidence/local/windows-production-tests.xml',
    'evidence/local/windows-peer-cli-receipt.json',
    'evidence/local/peer-buffer-windows.log',
    'evidence/local/production-package-selftest.json',
    'evidence/local/production-restart-selftest.json',
    'evidence/local/preview-dll-audit.json',
    'evidence/local/installer-skager-pe-brand.json',
    'evidence/local/windows-xnav-executable-sha256.txt',
    'evidence/local/windows-production-executable-sha256.txt',
}
EVIDENCE_TREES = ('evidence/local/ais-native-runtime',
                  'evidence/local/downloader-trust-windows',
                  'evidence/local/ocharts-private-wxcurl-trust-windows')
REQUIRED = {
    'build/beta-installer/SKAGER-Beta2-Setup.exe', 'build/beta-installer/package.json',
    'build/beta-installer/payload.zip',
    'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip',
    'build/developer-preview/SKAGER-Beta2-source.zip',
    PACKAGE_ROOT + '/docs/PRODUCT_BUILD.json', PACKAGE_ROOT + '/app/opencpn.exe',
    'evidence/local/windows-xnav-tests.xml', 'evidence/local/windows-production-tests.xml',
    'evidence/local/windows-peer-cli-receipt.json', 'evidence/local/peer-buffer-windows.log',
    'evidence/local/ais-native-runtime/report.json',
    'evidence/local/downloader-trust-windows/summary.json',
    'evidence/local/ocharts-private-wxcurl-trust-windows/summary.json',
}
for variant in VARIANTS:
    REQUIRED.update({f'build/{variant}-install/opencpn.exe',
                     f'build/{variant}-install/opennav-restart.exe',
                     f'build/{variant}-windows/include/config.h',
                     f'build/{variant}-windows/include/OpenNavBuild.h'})
REQUIRED.add('build/xnav-install/opencpn-cmd.exe')
REQUIRED.add(FEEDBACK_MANIFEST)
REQUIRED.update(FEEDBACK_BINARIES.values())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'Duplicate JSON field')
            result[key] = value
        return result
    return json.loads(data, object_pairs_hook=pairs)


def producer(commit, run_id, run_attempt, job='windows-integration'):
    require(isinstance(commit, str) and re.fullmatch('[0-9a-f]{40}', commit), 'Exact producer commit required')
    require(all(isinstance(v, str) and re.fullmatch('[1-9][0-9]{0,19}', v)
                for v in (run_id, run_attempt)), 'Exact producer run/attempt required')
    require(job == 'windows-integration', 'Unapproved producer job')
    return dict(repository='ThereptileII/Work', commit=commit, runId=run_id,
                runAttempt=run_attempt, job=job, platform='windows-2022', architecture='Win32')


def safe_name(name):
    require(isinstance(name, str) and name and '\\' not in name and ':' not in name and '\x00' not in name,
            'Unsafe archive path')
    path = PurePosixPath(name)
    require(not path.is_absolute() and path.as_posix() == name and
            all(part not in ('.', '..', '') and not part.endswith((' ', '.')) and
                not re.fullmatch(r'(?i)(con|prn|aux|nul|com[0-9]|lpt[0-9])(?:\..*)?', part)
                for part in name.split('/')), 'Unsafe archive path')
    return path


def allowed(name):
    path = safe_name(name)
    if name in FIXED_FILES:
        return True
    if any(name.startswith(prefix + '/') for prefix in (*INSTALLS, PACKAGE_ROOT, *EVIDENCE_TREES)):
        return True
    for variant in VARIANTS:
        base = f'build/{variant}-windows/'
        if name.startswith(base + 'include/') and path.suffix in ('.h', '.hpp'):
            return True
        if name.startswith(base + 'opennav-chart-style/v1/'):
            return True
        if name.startswith((base + 'Release/', base + 'test/')) and path.suffix.lower() in ('.exe', '.dll'):
            return True
    return False


def plain_file(root, name):
    path = root
    for part in safe_name(name).parts:
        path = path / part
        require(not path.is_symlink(), 'Linked retained input refused')
    require(stat.S_ISREG(path.stat().st_mode), 'Nonregular retained input refused')
    return path


def inventory(root):
    roots = [*INSTALLS, PACKAGE_ROOT, *EVIDENCE_TREES]
    for variant in VARIANTS:
        roots.extend(f'build/{variant}-windows/{part}' for part in ('include', 'Release', 'test', 'opennav-chart-style/v1'))
    names = {name for name in FIXED_FILES if (root / name).exists() or (root / name).is_symlink()}
    for directory in roots:
        base = root / directory
        require(not base.is_symlink(), 'Linked retained input directory refused')
        if not base.exists():
            continue
        for file in base.rglob('*'):
            require(not file.is_symlink(), 'Linked retained input refused')
            if file.is_file():
                name = file.relative_to(root).as_posix()
                if allowed(name):
                    names.add(name)
    return sorted(names)


def validate_inventory(names):
    require(0 < len(names) <= MAX_FILES and len(names) == len(set(names)), 'Invalid retained file count')
    require(REQUIRED <= set(names), 'Missing required compiled/package/security input: ' + ', '.join(sorted(REQUIRED - set(names))))
    folded = set()
    for name in names:
        require(allowed(name), 'Input outside approved retained paths')
        require(name.casefold() not in folded, 'Case-colliding retained path')
        folded.add(name.casefold())
    # Preserve actual upstream unit-test binaries, not just their XML receipt.
    for variant in VARIANTS:
        for binary in ('tests.exe', 'buffer_tests.exe'):
            require(any(name.startswith(f'build/{variant}-windows/') and
                        PurePosixPath(name).name == binary for name in names),
                    f'Missing compiled {variant} test executable: {binary}')


def feedback_binding(root, expected_commit):
    """Bind original producer paths to fixed retained files; never execute paths from JSON."""
    manifest_path = plain_file(root, FEEDBACK_MANIFEST)
    require(manifest_path.stat().st_size <= MAX_MANIFEST, 'Feedback manifest exceeds bound')
    manifest = strict_json(manifest_path.read_bytes())
    require(set(manifest) == {'schema', 'commit', 'tests'} and
            type(manifest['schema']) is int and manifest['schema'] == 1 and
            manifest['commit'] == expected_commit, 'Feedback manifest producer differs')
    entries = manifest['tests']
    require(isinstance(entries, list) and len(entries) == len(FEEDBACK_TESTS) and
            all(isinstance(entry, dict) and set(entry) == {'name', 'path'} and
                isinstance(entry['name'], str) and isinstance(entry['path'], str)
                for entry in entries), 'Invalid feedback manifest entries')
    require({entry['name'] for entry in entries} == FEEDBACK_TESTS,
            'Feedback manifest must contain each fixed component once')
    prefixes = set()
    for entry in entries:
        path = entry['path']
        suffix = '/' + FEEDBACK_BINARIES[entry['name']]
        require(path.endswith(suffix) and '\\' not in path and '\x00' not in path,
                'Feedback executable outside fixed retained paths')
        prefix = path[:-len(suffix)]
        windows = PureWindowsPath(prefix)
        require(windows.is_absolute() and re.fullmatch('[A-Za-z]:', windows.drive) and
                prefix == windows.as_posix(), 'Invalid feedback producer root')
        safe_name(prefix[3:])
        prefixes.add(prefix)
    require(len(prefixes) == 1, 'Feedback executable producer roots differ')
    return dict(manifestSha256=sha(manifest_path), binaries=[
        dict(name=name, path=FEEDBACK_BINARIES[name],
             sha256=sha(plain_file(root, FEEDBACK_BINARIES[name])))
        for name in sorted(FEEDBACK_TESTS)])


def restored_feedback(root, manifest_path, receipt_path, product_commit, harness_commit):
    """Rebase only sealed, restored component identities for a known CI harness."""
    root = Path(root).resolve()
    require(Path(manifest_path).resolve() == root / FEEDBACK_MANIFEST,
            'Only the retained original feedback manifest may be used')
    receipt = strict_json(Path(receipt_path).read_bytes())
    if receipt.get('kind') == 'skager-recovered-prepackage-staging-inputs':
        from recovered_staging import validated_receipt
        receipt = validated_receipt(root, receipt_path, product_commit, harness_commit)
        return dict(schema=1, commit=product_commit, tests=[
            dict(name=name, path=str(root / FEEDBACK_BINARIES[name]))
            for name in sorted(FEEDBACK_TESTS)])
    expected = receipt.get('producer', {})
    require(expected == producer(expected.get('commit'), expected.get('runId'),
                                 expected.get('runAttempt'), expected.get('job')) and
            expected['commit'] == product_commit and
            receipt.get('schema') == 1 and receipt.get('kind') == KIND and
            receipt.get('status') == 'restored' and receipt.get('qualification') == 'not-run' and
            receipt.get('harnessCommit') == harness_commit and
            receipt.get('workspaceRoot') == str(root) and
            re.fullmatch('[0-9a-f]{64}', receipt.get('archiveSha256', '')),
            'Feedback restore receipt producer/harness/workspace differs')
    require(receipt.get('boatFeedback') == feedback_binding(root, product_commit),
            'Feedback manifest or executable differs from sealed restore receipt')
    return dict(schema=1, commit=product_commit, tests=[
        dict(name=name, path=str(root / FEEDBACK_BINARIES[name]))
        for name in sorted(FEEDBACK_TESTS)])


def validate_content(root, names, expected):
    feedback_binding(root, expected['commit'])
    def read(name):
        return plain_file(root, name).read_bytes()
    def digest(name):
        return sha(plain_file(root, name))
    for variant in VARIANTS:
        header = read(f'build/{variant}-windows/include/OpenNavBuild.h').decode('utf-8-sig')
        match = re.search(r'#define OPENNAV_BUILD_COMMIT "([0-9a-f]{40})"', header)
        require(match and match[1] == expected['commit'], 'Compiled header belongs to another commit')
        document = ET.fromstring(read(f'evidence/local/windows-{variant}-tests.xml'))
        cases = list(document.iter('testcase'))
        require(cases and any(case.find('skipped') is None for case in cases) and
                not list(document.iter('failure')) and not list(document.iter('error')),
                'Native test evidence is empty or failed')
        for suite in document.iter('testsuite'):
            require(int(suite.get('failures', '0')) == 0 and int(suite.get('errors', '0')) == 0,
                    'Native test suite reports failure')
    peer = strict_json(read('evidence/local/windows-peer-cli-receipt.json'))
    require(peer.get('status') == 'success' and
            all(peer.get(key) == expected[key] for key in ('commit', 'runId', 'runAttempt', 'job')) and
            peer.get('executableSha256') == digest('build/xnav-install/opencpn-cmd.exe'),
            'Peer CLI evidence differs from producer or executable')
    ais = strict_json(read('evidence/local/ais-native-runtime/report.json'))
    require(ais.get('passed') is True and
            all(ais.get(key) == expected[key] for key in ('commit', 'runId', 'runAttempt')),
            'AIS runtime evidence missing or belongs to another producer')
    for directory in EVIDENCE_TREES[1:]:
        record = strict_json(read(directory + '/summary.json'))
        require(record.get('status') == 'passed' and record.get('cleanup') == 'verified',
                'Native TLS trust check did not pass with verified cleanup')
    product = strict_json(read(PACKAGE_ROOT + '/docs/PRODUCT_BUILD.json'))
    exe_digest = digest('build/production-install/opencpn.exe')
    require_product_output_policy(product)
    require(product.get('commit') == expected['commit'] and product.get('test_fixtures') is False and
            product.get('build_purpose') == 'INSTALLED PRODUCT' and
            product.get('executable_sha256') == exe_digest and
            digest(PACKAGE_ROOT + '/app/opencpn.exe') == exe_digest,
            'Retained product identity or versioned output boundary differs')
    package = strict_json(read('build/beta-installer/package.json'))
    require(package.get('commit') == expected['commit'] and
            package.get('payloadSha256') == digest('build/beta-installer/payload.zip'),
            'Installer payload belongs to different bytes or revision')
    for name, member, field in (
        ('build/developer-preview/SKAGER-Beta2-source.zip', 'SOURCE_REFERENCE.json', 'productCommit'),
        ('build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip',
         'SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json', 'commit')):
        with zipfile.ZipFile(plain_file(root, name)) as archive:
            matches = [entry for entry in archive.infolist() if entry.filename == member]
            require(len(matches) == 1 and matches[0].orig_filename == member and
                    not matches[0].is_dir() and stat.S_IFMT(matches[0].external_attr >> 16) in (0, stat.S_IFREG) and
                    matches[0].file_size <= 16 * 1024**2, 'Missing or unsafe packaged identity')
            metadata = strict_json(archive.read(matches[0]))
            require(metadata.get(field) == expected['commit'], 'Retained source/recovery archive revision differs')
            if field == 'commit':
                require(metadata == product, 'Recovery archive differs from extracted product identity')


def archive_entry(name):
    # Apply the same canonical metadata to payloads AND the generated manifest.
    # writestr(name, ...) otherwise inserts the current local timestamp.
    entry = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    entry.compress_type = zipfile.ZIP_STORED if name.endswith('.zip') else zipfile.ZIP_DEFLATED
    return entry


def seal(root, output, expected):
    require(not Path(root).is_symlink(), 'Seal root must be regular')
    root, output = Path(root).resolve(), Path(output).resolve()
    require(not output.exists(), 'Seal output must be fresh')
    names = inventory(root)
    validate_inventory(names)
    validate_content(root, names, expected)
    output.mkdir(parents=True)
    archive = output / ARCHIVE
    records = []
    total = 0
    try:
        with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=1) as target:
            for name in names:
                source = plain_file(root, name)
                before = source.stat()
                require(0 <= before.st_size <= MAX_FILE, 'Retained input exceeds individual bound')
                total += before.st_size
                require(total <= MAX_TOTAL, 'Retained inputs exceed total bound')
                entry = archive_entry(name)
                digest = hashlib.sha256()
                with source.open('rb') as stream, target.open(entry, 'w', force_zip64=True) as destination:
                    for block in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(block)
                        destination.write(block)
                after = source.stat()
                require((before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns),
                        'Retained input changed during sealing')
                records.append(dict(path=name, size=before.st_size, sha256=digest.hexdigest()))
            manifest = dict(schema=1, kind=KIND, producer=expected, qualification='not-run', files=records)
            encoded = canonical(manifest)
            require(len(encoded) <= MAX_MANIFEST, 'Manifest exceeds bound')
            target.writestr(archive_entry(MANIFEST), encoded, compresslevel=1)
        receipt = dict(schema=1, kind=KIND, status='sealed', qualification='not-run', producer=expected,
                       archive=ARCHIVE, archiveSha256=sha(archive),
                       manifestSha256=hashlib.sha256(encoded).hexdigest(), files=len(records), bytes=total)
        (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        return receipt
    except BaseException:
        shutil.rmtree(output)
        raise


def restore(root, archive, archive_sha256, expected, harness_commit, receipt_path):
    require(not Path(root).is_symlink(), 'Restore root must be regular')
    root, archive, receipt_path = Path(root).resolve(), Path(archive), Path(receipt_path)
    require(re.fullmatch('[0-9a-f]{64}', archive_sha256 or ''), 'Authenticated archive SHA256 required')
    require(re.fullmatch('[0-9a-f]{40}', harness_commit or ''), 'Exact harness commit required')
    require(not archive.is_symlink() and archive.is_file() and sha(archive) == archive_sha256,
            'Retained archive differs from authenticated producer digest')
    require(not receipt_path.exists(), 'Restore receipt must be fresh')
    for directory in (*INSTALLS, 'build/xnav-windows', 'build/production-windows',
                      'build/developer-preview', 'build/beta-installer'):
        require(not (root / directory).exists() and not (root / directory).is_symlink(),
                'Restore requires fresh build input directories')
    with zipfile.ZipFile(archive) as source:
        entries = source.infolist()
        require(0 < len(entries) <= MAX_FILES + 1, 'Archive entry count exceeds bound')
        seen = set()
        total = 0
        for entry in entries:
            require(entry.orig_filename == entry.filename, 'Unsafe normalized archive path')
            safe_name(entry.filename)
            require(not entry.is_dir() and stat.S_IFMT(entry.external_attr >> 16) in (0, stat.S_IFREG) and not entry.flag_bits & 1,
                    'Archive links, special files or encrypted inputs refused')
            require(entry.filename.casefold() not in seen, 'Duplicate or case-colliding archive member')
            seen.add(entry.filename.casefold())
            limit = MAX_MANIFEST if entry.filename == MANIFEST else MAX_FILE
            require(0 <= entry.file_size <= limit, 'Archive member exceeds bound')
            total += entry.file_size
            require(total <= MAX_TOTAL + MAX_MANIFEST, 'Archive exceeds total bound')
        require(MANIFEST in source.namelist(), 'Missing retained manifest')
        encoded = source.read(MANIFEST)
        manifest = strict_json(encoded)
        require(set(manifest) == {'schema', 'kind', 'producer', 'qualification', 'files'} and
                type(manifest['schema']) is int and manifest['schema'] == 1 and manifest['kind'] == KIND and
                manifest['producer'] == expected and manifest['qualification'] == 'not-run',
                'Retained producer identity or scope differs')
        records = manifest['files']
        require(isinstance(records, list) and all(isinstance(r, dict) and set(r) == {'path','size','sha256'}
                                                 for r in records), 'Invalid retained file records')
        names = [r['path'] for r in records]
        validate_inventory(names)
        require(set(source.namelist()) == set(names) | {MANIFEST}, 'Archive inventory differs from manifest')
        for record in records:
            require(type(record['size']) is int and 0 <= record['size'] <= MAX_FILE and
                    re.fullmatch('[0-9a-f]{64}', record['sha256'] or '') and
                    source.getinfo(record['path']).file_size == record['size'], 'Invalid retained record size/hash')
        # Refuse every pre-existing destination before writing any restored file.
        for name in names:
            destination = root / name
            require(not destination.exists() and not destination.is_symlink(), 'Restore requires fresh input files')
            for parent in destination.parents:
                if parent == root:
                    break
                require(not parent.is_symlink(), 'Linked restore ancestor refused')
        with tempfile.TemporaryDirectory(prefix='skager-staging-restore-', dir=root.parent) as temporary:
            staging = Path(temporary)
            for record in records:
                destination = staging / record['path']
                destination.parent.mkdir(parents=True, exist_ok=True)
                digest = hashlib.sha256()
                with source.open(record['path']) as stream, destination.open('xb') as target:
                    for block in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(block)
                        target.write(block)
                require(digest.hexdigest() == record['sha256'], 'Retained file checksum mismatch')
            validate_content(staging, names, expected)
            # Only approved names can be published. No extractall/build tree or
            # source checkout overwrite is permitted by this boundary.
            for name in names:
                destination = root / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                with (staging / name).open('rb') as stream, destination.open('xb') as target:
                    shutil.copyfileobj(stream, target, 1024 * 1024)
    receipt = dict(schema=1, kind=KIND, status='restored', qualification='not-run', producer=expected,
                   harnessCommit=harness_commit, workspaceRoot=str(root), archiveSha256=archive_sha256,
                   sourceArchiveSha256=next(record['sha256'] for record in records
                       if record['path']=='build/developer-preview/SKAGER-Beta2-source.zip'),
                   manifestSha256=hashlib.sha256(encoded).hexdigest(), files=len(names),
                   boatFeedback=feedback_binding(root, expected['commit']))
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    with receipt_path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(receipt, indent=2) + '\n')
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('seal', 'restore'):
        command = commands.add_parser(name)
        command.add_argument('--root', type=Path, required=True)
        for key in ('commit', 'run-id', 'run-attempt'):
            command.add_argument('--producer-' + key, required=True)
        command.add_argument('--producer-job', default='windows-integration')
    commands.choices['seal'].add_argument('--output', type=Path, required=True)
    restore_parser = commands.choices['restore']
    restore_parser.add_argument('--archive', type=Path, required=True)
    restore_parser.add_argument('--archive-sha256', required=True)
    restore_parser.add_argument('--harness-commit', required=True)
    restore_parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args(argv)
    expected = producer(args.producer_commit, args.producer_run_id, args.producer_run_attempt, args.producer_job)
    # Public library functions support inert policy tests on Linux. CLI sealing
    # is restricted to the named disposable native producer, never a laptop.
    if args.command == 'seal':
        require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
                os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
                os.environ.get('GITHUB_REPOSITORY') == expected['repository'] and
                all(os.environ.get(env) == expected[key] for env, key in
                    (('GITHUB_SHA','commit'), ('GITHUB_RUN_ID','runId'),
                     ('GITHUB_RUN_ATTEMPT','runAttempt'), ('GITHUB_JOB','job'))),
                'Sealing requires the exact disposable native producer')
        head = subprocess.check_output(['git','-C',str(args.root),'rev-parse','HEAD'], text=True).strip()
        require(head == expected['commit'], 'Producer checkout differs')
        result = seal(args.root, args.output, expected)
        if os.environ.get('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as stream:
                for name, value in (('archive_sha256',result['archiveSha256']),
                                    ('manifest_sha256',result['manifestSha256'])):
                    stream.write(name + '=' + value + '\n')
    else:
        head = subprocess.check_output(['git','-C',str(args.root),'rev-parse','HEAD'], text=True).strip()
        require(head == args.harness_commit, 'Harness checkout differs')
        result = restore(args.root, args.archive, args.archive_sha256, expected, args.harness_commit, args.receipt)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
