#!/usr/bin/env python3
"""Fail-closed OpenSSL provenance checks for native Windows packaging."""
import hashlib
import json
from pathlib import Path
import struct

OPENSSL_FILES = ('bin/libssl-3.dll', 'bin/libcrypto-3.dll')
OPENSSL_OUTPUTS = ('include/openssl/opensslv.h', 'lib/libssl.lib', 'lib/libcrypto.lib',
                   'bin/openssl.exe') + OPENSSL_FILES
OPENSSL_CACHE_PATHS = {
    'include/openssl/opensslv.h': 'include/openssl/opensslv.h',
    'lib/libssl.lib': 'libssl.lib',
    'lib/libcrypto.lib': 'libcrypto.lib',
    'bin/libssl-3.dll': 'libssl-3.dll',
    'bin/libcrypto-3.dll': 'libcrypto-3.dll',
}
REQUIRED_NOTICE_FILES = ('LICENSE.txt', 'provenance.json')
EXPECTED_VERSION = '3.5.9'
EXPECTED_ARCHIVE = 'openssl-3.5.9.tar.gz'
EXPECTED_SOURCE_URL = 'https://github.com/openssl/openssl/releases/download/openssl-3.5.9/openssl-3.5.9.tar.gz'
EXPECTED_SOURCE_SHA256 = '603f5602e2eef00d77fbd429d34dcd5822bb301757a1bc9cdb24c670f1eb859a'
EXPECTED_SOURCE_BYTES = 53279637
EXPECTED_SIGNING_FINGERPRINT = 'B146647E45A7B33947AB226B2A2C87D161692D40'


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path, label):
    path = Path(path)
    if not path.is_file() or path.is_symlink():
        raise ValueError(label + ' missing or not a regular file: ' + str(path))
    try:
        value = json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(label + ' is not valid UTF-8 JSON') from error
    if not isinstance(value, dict):
        raise ValueError(label + ' must be a JSON object')
    return value


def _require_win32_pe(path):
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 64 or data[:2] != b'MZ':
        raise ValueError('OpenSSL DLL is not a PE file: ' + path.name)
    pe_offset = struct.unpack_from('<I', data, 0x3c)[0]
    if pe_offset > len(data) - 6 or data[pe_offset:pe_offset + 4] != b'PE\0\0':
        raise ValueError('OpenSSL DLL has an invalid PE header: ' + path.name)
    machine = struct.unpack_from('<H', data, pe_offset + 4)[0]
    if machine != 0x14c:
        raise ValueError('OpenSSL DLL is not the required Win32/x86 architecture: ' + path.name)


def verify_openssl_package_inputs(install, lock_path, source_archive, notice_dir):
    """Verify installed binaries and return the inert source-archive inventory."""
    install = Path(install).resolve()
    lock = _read_json(lock_path, 'OpenSSL lock')
    manifest = _read_json(install / 'openssl-build.json', 'installed OpenSSL manifest')
    lock_keys = {'version', 'configuration', 'archive', 'url', 'sha256', 'bytes',
                 'signingPrimaryFingerprint', 'buildTools'}
    if set(lock) != lock_keys:
        raise ValueError('OpenSSL lock has an unsupported schema')
    build_tools = lock.get('buildTools')
    if (not isinstance(build_tools, dict) or set(build_tools) != {'nasm'} or
            not isinstance(build_tools['nasm'], dict) or set(build_tools['nasm']) != {
                'version', 'archive', 'url', 'sha256', 'bytes', 'provenance'}):
        raise ValueError('OpenSSL lock omits pinned build-tool provenance')
    manifest_keys = {'schemaVersion', 'library', 'version', 'configuration', 'architecture', 'abi',
                     'source', 'outputs', 'cacheBuildwin', 'toolchain', 'buildSteps', 'versionOutput'}
    if set(manifest) != manifest_keys or manifest.get('schemaVersion') != 1 or manifest.get('library') != 'OpenSSL':
        raise ValueError('Installed OpenSSL manifest has an unsupported schema')
    if (lock.get('version') != EXPECTED_VERSION or lock.get('configuration') != 'VC-WIN32 shared' or
            manifest.get('version') != lock['version'] or manifest.get('configuration') != lock['configuration'] or
            manifest.get('architecture') != 'Win32' or manifest.get('abi') != 'x86'):
        raise ValueError('OpenSSL lock does not require the supported Win32 architecture')
    if (manifest.get('toolchain', {}).get('nasmArchiveSha256') != build_tools['nasm']['sha256'] or
            not isinstance(manifest.get('buildSteps'), dict) or
            any(manifest['buildSteps'].get(step) != 'passed'
                for step in ('configure', 'compile', 'test', 'install')) or
            'OpenSSL 3.5.9' not in manifest.get('versionOutput', '')):
        raise ValueError('Installed OpenSSL build attestation is incomplete')
    source = manifest.get('source')
    reviewed_source = {key: lock[key] for key in (
        'url', 'archive', 'sha256', 'bytes', 'signingPrimaryFingerprint')}
    outputs = manifest.get('outputs')
    if not isinstance(source, dict) or set(source) != {
            'url', 'archive', 'sha256', 'bytes', 'signingPrimaryFingerprint'}:
        raise ValueError('OpenSSL lock source provenance is incomplete')
    if source != reviewed_source:
        raise ValueError('Installed OpenSSL source provenance does not match the reviewed lock')
    if not isinstance(outputs, dict) or set(outputs) != set(OPENSSL_OUTPUTS):
        raise ValueError('OpenSSL lock output inventory is incomplete')
    cache = manifest.get('cacheBuildwin')
    if not isinstance(cache, dict) or set(cache) != set(OPENSSL_CACHE_PATHS.values()):
        raise ValueError('OpenSSL cache mapping inventory is incomplete')
    for relative, output_record in outputs.items():
        if (not isinstance(output_record, dict) or set(output_record) != {'sha256', 'bytes'} or
                not isinstance(output_record['sha256'], str) or len(output_record['sha256']) != 64 or
                not isinstance(output_record['bytes'], int) or output_record['bytes'] <= 0):
            raise ValueError('OpenSSL output record is invalid: ' + relative)
        if relative in OPENSSL_CACHE_PATHS:
            cache_record = cache[OPENSSL_CACHE_PATHS[relative]]
            if (not isinstance(cache_record, dict) or set(cache_record) != {'source', 'sha256', 'bytes'} or
                    cache_record['source'] != relative or
                    {key: cache_record[key] for key in ('sha256', 'bytes')} != output_record):
                raise ValueError('OpenSSL cache mapping differs from build output: ' + relative)
    archive_name = lock.get('archive')
    if (not isinstance(archive_name, str) or Path(archive_name).name != archive_name or
            archive_name in ('', '.', '..')):
        raise ValueError('OpenSSL source archive name is unsafe')
    expected_source_sha = lock.get('sha256')
    if (archive_name != EXPECTED_ARCHIVE or lock.get('url') != EXPECTED_SOURCE_URL or
            expected_source_sha != EXPECTED_SOURCE_SHA256 or
            lock.get('bytes') != EXPECTED_SOURCE_BYTES or
            lock.get('signingPrimaryFingerprint') != EXPECTED_SIGNING_FINGERPRINT):
        raise ValueError('OpenSSL lock does not identify the reviewed upstream source')
    source_archive = Path(source_archive)
    if (not source_archive.is_file() or source_archive.is_symlink() or
            source_archive.name != archive_name or source_archive.stat().st_size != lock.get('bytes') or
            _sha256(source_archive) != expected_source_sha.lower()):
        raise ValueError('Verified OpenSSL source archive is missing, renamed, or has the wrong digest')
    source_archive = source_archive.resolve()
    for relative in OPENSSL_FILES:
        record = manifest['cacheBuildwin'][Path(relative).name]
        dll = install / Path(relative).name
        if not dll.is_file() or dll.is_symlink():
            raise ValueError('Installed OpenSSL DLL missing: ' + relative)
        if dll.stat().st_size != record['bytes'] or _sha256(dll) != record['sha256']:
            raise ValueError('Installed OpenSSL DLL differs from the reviewed lock: ' + relative)
        _require_win32_pe(dll)
    notice_dir = Path(notice_dir)
    for name in REQUIRED_NOTICE_FILES:
        notice = notice_dir / name
        if not notice.is_file() or notice.is_symlink() or not notice.read_bytes():
            raise ValueError('OpenSSL notice missing or empty: ' + name)
    provenance = _read_json(notice_dir / 'provenance.json', 'OpenSSL notice provenance')
    if (provenance.get('library') != 'OpenSSL ' + lock['version'] or
            provenance.get('sourceArchiveSha256') != expected_source_sha.lower() or
            provenance.get('sourceArchiveBytes') != lock['bytes'] or
            provenance.get('signatureVerification', {}).get('primaryFingerprint') !=
            lock['signingPrimaryFingerprint'] or
            provenance.get('licenseSha256') != _sha256(notice_dir / 'LICENSE.txt')):
        raise ValueError('OpenSSL notices do not match the reviewed source lock')
    source_bundle = {
        'archive': source_archive,
        'path': 'third-party-sources/' + archive_name,
        'sha256': expected_source_sha.lower(),
        'reference': {
            'library': 'OpenSSL', 'version': lock['version'],
            'url': lock['url'], 'signingPrimaryFingerprint': lock['signingPrimaryFingerprint'],
        },
    }
    return {'sourceBundle': source_bundle, 'manifest': manifest}


def verify_packaged_openssl(package_app, manifest):
    """Recheck final package bytes after all dependency-copy operations."""
    package_app = Path(package_app)
    for name in ('libssl-3.dll', 'libcrypto-3.dll'):
        record = manifest['cacheBuildwin'][name]
        dll = package_app / name
        if (not dll.is_file() or dll.is_symlink() or dll.stat().st_size != record['bytes'] or
                _sha256(dll) != record['sha256']):
            raise ValueError('Packaged OpenSSL DLL differs from the verified build: ' + name)
        _require_win32_pe(dll)
