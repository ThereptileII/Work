#!/usr/bin/env python3
"""Create/verify immutable Staging release sets; promotion receipts live elsewhere.

This is an offline integrity check, not a signature or release qualification.
The publisher must establish trusted CI provenance before using these records.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import zipfile

MANIFEST = 'RELEASE.json'
PRODUCT_FILES = frozenset({
    'SKAGER-Beta2-Setup.exe', 'SKAGER-Beta2-Portable-Recovery.zip',
    'SKAGER-Beta2-source.zip', 'SKAGER-Beta2-Install-Guide.md',
    'SKAGER-Beta2-Test-Guide.md', 'SKAGER-Beta2-Release-Notes.md',
})
RELEASE_FILES = PRODUCT_FILES | {'SHA256SUMS.txt', 'QUALIFICATION.txt'}
OPTIONAL_FILES = frozenset({'QUALIFICATION.json', 'RETEST_SUPPORT.json'})
UPSTREAM = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
FIELDS = {'schemaVersion', 'channel', 'version', 'commit', 'runId', 'runAttempt',
          'candidateId', 'designReview', 'files', 'inventorySha256'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()


def strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'Duplicate JSON key: ' + key)
            result[key] = value
        return result
    return json.loads(data, object_pairs_hook=pairs)


def matches(pattern, value):
    return isinstance(value, str) and re.fullmatch(pattern, value) is not None


def identity(commit, run_id, run_attempt, version):
    require(matches(r'[0-9a-f]{40}', commit), 'Exact lowercase commit SHA required')
    require(matches(r'[1-9][0-9]{0,19}', run_id) and
            matches(r'[1-9][0-9]{0,9}', run_attempt),
            'Run identity must be canonical positive decimal strings')
    require(matches(r'[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9]+(?:[.-][A-Za-z0-9]+)*)?', version)
            and len(version) <= 80, 'Safe product version required')
    return f'{version}-run{run_id}-attempt{run_attempt}-{commit[:12]}'


def safe_name(name):
    require(matches(r'[A-Za-z0-9][A-Za-z0-9._-]{0,199}', name) and
            not name.endswith('.') and name.split('.')[0].upper() not in
            {'CON', 'PRN', 'AUX', 'NUL', *(f'COM{i}' for i in range(10)),
             *(f'LPT{i}' for i in range(10))}, 'Unsafe release basename')


def directory_files(directory, with_manifest):
    directory = Path(directory)
    require(not directory.is_symlink() and directory.is_dir(), 'Regular release directory required')
    files = {}
    folded = set()
    for path in directory.iterdir():
        safe_name(path.name)
        require(path.name.casefold() not in folded, 'Case-colliding release files')
        folded.add(path.name.casefold())
        require(stat.S_ISREG(path.lstat().st_mode), 'Release links/nonregular files refused')
        files[path.name] = path
    expected = RELEASE_FILES | ({MANIFEST} if with_manifest else set())
    require(expected <= set(files) <= expected | OPTIONAL_FILES,
            'Release inventory differs: missing/extra files')
    return files


def file_record(path):
    hasher = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            size += len(chunk)
            hasher.update(chunk)
    require(size > 0, 'Empty release file: ' + path.name)
    return {'name': path.name, 'size': size, 'sha256': hasher.hexdigest()}


def check_sums(files, records):
    sums = {}
    for line in files['SHA256SUMS.txt'].read_text(encoding='utf-8').splitlines():
        match = re.fullmatch(r'([a-f0-9]{64})  ([A-Za-z0-9_.-]+)', line)
        require(match is not None, 'Malformed SHA256SUMS.txt')
        safe_name(match[2])
        require(match[2] not in sums, 'Duplicate checksum entry')
        sums[match[2]] = match[1]
    expected = {item['name']: item['sha256'] for item in records if item['name'] in PRODUCT_FILES}
    require(sums == expected, 'SHA256SUMS.txt differs from complete product inventory')


def archive_records(archive):
    names = set()
    for entry in archive.infolist():
        name = entry.filename
        path = PurePosixPath(name)
        require(name and not path.is_absolute() and '\\' not in name and ':' not in name and
                '..' not in path.parts and path.as_posix() == name.rstrip('/') and
                all(part not in {'.', ''} for part in name.rstrip('/').split('/')),
                'Unsafe archive path')
        require(name.casefold() not in names, 'Duplicate/case-colliding archive entry')
        names.add(name.casefold())
    return names


def read_metadata(archive, name):
    entry = archive.getinfo(name)
    require(not entry.is_dir() and stat.S_IFMT(entry.external_attr >> 16) != stat.S_IFLNK
            and entry.file_size <= 16 * 1024 * 1024, 'Unsafe archive metadata')
    value = strict_json(archive.read(name))
    require(isinstance(value, dict), 'Archive metadata must be an object')
    return value


def check_product(files, commit, version):
    notes_name = 'SKAGER-Beta2-Release-Notes.md'
    notes = files[notes_name].read_bytes()
    with zipfile.ZipFile(files['SKAGER-Beta2-Portable-Recovery.zip']) as archive:
        archive_records(archive)
        require(all(stat.S_IFMT(item.external_attr >> 16) != stat.S_IFLNK
                    for item in archive.infolist()), 'Portable archive links refused')
        prefix = 'SKAGER-Beta2-Portable-Recovery/'
        build = read_metadata(archive, prefix + 'docs/PRODUCT_BUILD.json')
        require(build.get('commit') == commit and build.get('version') == version and
                build.get('test_fixtures') is False and build.get('build_purpose') == 'INSTALLED PRODUCT'
                and build.get('xnav_hardware_output_policy') == 'status-only',
                'Portable product identity/policy differs')
        with archive.open(prefix + 'app/opencpn.exe') as executable:
            hasher = hashlib.sha256()
            for chunk in iter(lambda: executable.read(1024 * 1024), b''):
                hasher.update(chunk)
        require(hasher.hexdigest() == build.get('executable_sha256'), 'Portable executable differs')
        hashes = read_metadata(archive, prefix + 'FILE_SHA256.json')
        require(archive.read(prefix + 'docs/' + notes_name) == notes and
                hashes.get('docs/' + notes_name) == digest(notes), 'Portable release notes differ')
    with zipfile.ZipFile(files['SKAGER-Beta2-source.zip']) as archive:
        archive_records(archive)
        source = read_metadata(archive, 'SOURCE_REFERENCE.json')
        require(source.get('productCommit') == commit and source.get('upstreamCommit') == UPSTREAM
                and source.get('openCpnVersion') == '5.12.4', 'Corresponding source identity differs')
        notes_path = 'opennav-x/docs/beta2/' + notes_name
        source_files = source.get('files')
        require(isinstance(source_files, dict) and isinstance(source_files.get(notes_path), dict)
                and source_files[notes_path].get('sha256') == digest(notes)
                and archive.read(notes_path) == notes, 'Corresponding source release notes differ')


def design_review(files):
    if 'QUALIFICATION.json' not in files:
        return 'not-requested'
    path = files['QUALIFICATION.json']
    require(path.stat().st_size <= 1024 * 1024, 'Oversized qualification metadata')
    qualification = strict_json(path.read_bytes())
    require(isinstance(qualification, dict) and
            qualification.get('designReview') in ('not-requested', 'requested'),
            'Qualification must record whether design review was requested, not claim a pass')
    return qualification['designReview']


def create(directory, commit, run_id, run_attempt, version):
    candidate = identity(commit, run_id, run_attempt, version)
    files = directory_files(directory, False)
    records = [file_record(files[name]) for name in sorted(files)]
    check_sums(files, records)
    check_product(files, commit, version)
    record = dict(schemaVersion=1, channel='staging', version=version, commit=commit,
                  runId=run_id, runAttempt=run_attempt, candidateId=candidate,
                  designReview=design_review(files), files=records, inventorySha256=digest(canonical(records)))
    # Exclusive create avoids changing an existing release identity on retry.
    with (Path(directory) / MANIFEST).open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(record, indent=2, sort_keys=True) + '\n')
    return record


def verify(directory, commit=None):
    files = directory_files(directory, True)
    require(files[MANIFEST].stat().st_size <= 1024 * 1024, 'Oversized release manifest')
    record = strict_json(files[MANIFEST].read_bytes())
    require(isinstance(record, dict) and set(record) == FIELDS, 'Unknown manifest fields')
    require(type(record['schemaVersion']) is int and record['schemaVersion'] == 1,
            'Unknown release schema')
    require(record['channel'] == 'staging' and record['designReview'] in ('not-requested', 'requested'),
            'Release manifest must retain Staging origin; qualification belongs in a separate receipt')
    candidate = identity(record['commit'], record['runId'], record['runAttempt'], record['version'])
    require(record['candidateId'] == candidate, 'Candidate identity differs')
    if commit is not None:
        require(matches(r'[0-9a-f]{40}', commit) and record['commit'] == commit, 'Requested commit differs')
    records = record['files']
    require(isinstance(records, list) and len(records) == len(files) - 1, 'Invalid file inventory')
    seen = set()
    for item in records:
        require(isinstance(item, dict) and set(item) == {'name', 'size', 'sha256'}, 'Invalid file record')
        safe_name(item['name'])
        require(item['name'].casefold() not in seen, 'Duplicate/case-colliding file record')
        seen.add(item['name'].casefold())
        require(type(item['size']) is int and item['size'] > 0 and
                matches(r'[a-f0-9]{64}', item['sha256']), 'Invalid file size/hash')
    require(record['inventorySha256'] == digest(canonical(records)), 'Inventory digest differs')
    actual = [file_record(files[name]) for name in sorted(files) if name != MANIFEST]
    require(records == actual, 'File inventory hash/size/order differs')
    require(record['designReview'] == design_review(files), 'Design review scheduling metadata differs')
    check_sums(files, actual)
    check_product(files, record['commit'], record['version'])
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    create_parser = sub.add_parser('create')
    create_parser.add_argument('--directory', required=True, type=Path)
    for name in ('commit', 'run-id', 'run-attempt', 'version'):
        create_parser.add_argument('--' + name, required=True)
    verify_parser = sub.add_parser('verify')
    verify_parser.add_argument('--directory', required=True, type=Path)
    verify_parser.add_argument('--commit')
    args = vars(parser.parse_args())
    command = args.pop('command')
    try:
        result = create(**args) if command == 'create' else verify(**args)
    except (ValueError, OSError, KeyError, zipfile.BadZipFile) as error:
        parser.exit(1, f'Release manifest refused: {error}\n')
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
