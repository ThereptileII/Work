"""Fail-closed maintained curl/zlib source and installed-output package boundary."""
import re
from pathlib import Path

from openssl_package import _read_json, _sha256, _require_win32_pe

SOURCES = {
    'curl': {
        'version': '8.22.0', 'configuration': 'Win32 shared OpenSSL',
        'archive': 'curl-8.22.0.tar.xz',
        'url': 'https://curl.se/download/curl-8.22.0.tar.xz',
        'sha256': 'f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7',
        'bytes': 2953092,
        'signingPrimaryFingerprint': '27EDEAF22F3ABCEB50DB9A125CC908FDB71E12C2',
    },
    'zlib': {
        'version': '1.3.2', 'configuration': 'Win32 shared',
        'archive': 'zlib-1.3.2.tar.gz', 'url': 'https://github.com/madler/zlib/releases/download/v1.3.2/zlib-1.3.2.tar.gz',
        'sha256': 'bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16',
        'bytes': 1502830,
        'signingPrimaryFingerprint': '5ED46A6721D365587791E2AA783FCD8E58BCAFBA',
    },
}
SOURCE_KEYS = {'url', 'archive', 'sha256', 'bytes', 'signingPrimaryFingerprint'}
ZLIB_OUTPUTS = {'include/zlib.h', 'include/zconf.h', 'lib/zlib1.lib', 'bin/zlib1.dll'}
CURL_HEADERS = {'curl.h', 'curlver.h', 'easy.h', 'header.h', 'mprintf.h', 'multi.h',
                'options.h', 'stdcheaders.h', 'system.h', 'typecheck-gcc.h', 'urlapi.h',
                'websockets.h'}
CURL_OUTPUTS = {'bin/libcurl.dll', 'lib/libcurl.lib'} | {'include/curl/' + p for p in CURL_HEADERS}
BASE_KEYS = {'schemaVersion', 'library', 'version', 'configuration', 'architecture',
             'abi', 'runtime', 'source', 'buildSteps', 'outputs'}


def require_record(record, label):
    if (not isinstance(record, dict) or set(record) != {'sha256', 'bytes'} or
            not isinstance(record['sha256'], str) or
            re.fullmatch('[0-9a-f]{64}', record['sha256']) is None or
            type(record['bytes']) is not int or record['bytes'] <= 0):
        raise ValueError('Invalid output record: ' + label)


def verify_file(path, record):
    path = Path(path)
    require_record(record, path.name)
    if (not path.is_file() or path.is_symlink() or
            path.stat().st_size != record['bytes'] or _sha256(path) != record['sha256']):
        raise ValueError('Missing or changed dependency file: ' + path.name)


def verify_manifest(install, library):
    manifest = _read_json(install / (library + '-build.json'), library + ' build manifest')
    pin = SOURCES[library]
    keys = BASE_KEYS | ({'dependencies', 'options', 'versionOutput', 'importOutput',
                         'cacheBuildwin'} if library == 'curl' else set())
    if (set(manifest) != keys or manifest.get('schemaVersion') != 1 or
            manifest.get('library') != library or manifest.get('version') != pin['version'] or
            manifest.get('configuration') != pin['configuration'] or
            manifest.get('architecture') != 'Win32' or manifest.get('abi') != 'x86' or
            manifest.get('runtime') != 'MultiThreadedDLL (/MD)' or
            manifest.get('source') != {k: pin[k] for k in SOURCE_KEYS}):
        raise ValueError('Unsupported dependency build identity: ' + library)
    steps = manifest['buildSteps']
    if (not isinstance(steps, dict) or
            any(steps.get(s) != 'passed' for s in ('configure', 'compile', 'test', 'install'))):
        raise ValueError('Incomplete dependency build: ' + library)
    expected = CURL_OUTPUTS if library == 'curl' else ZLIB_OUTPUTS
    outputs = manifest['outputs']
    if not isinstance(outputs, dict) or set(outputs) != expected:
        raise ValueError('Unexpected dependency output inventory: ' + library)
    for name, record in outputs.items():
        require_record(record, name)
    if library == 'curl':
        count = steps.get('testsReported')
        if (steps.get('testTarget') != 'tests' or type(count) is not int or count <= 0 or
                type(steps.get('testsPassed')) is not int or steps['testsPassed'] != count or
                steps.get('log') != 'evidence/local/windows-curl-native-output.log' or
                not isinstance(steps.get('logSha256'), str) or
                re.fullmatch('[0-9a-f]{64}', steps['logSha256']) is None):
            raise ValueError('curl has no successful upstream execution evidence')
        mappings = manifest['cacheBuildwin']
        expected_mapping = {Path(n).name if n.startswith(('bin/', 'lib/')) else n:
                            {'source': n, **r} for n, r in outputs.items()}
        if mappings != expected_mapping:
            raise ValueError('curl cache mapping differs from build outputs')
        dependencies = manifest['dependencies']
        if not isinstance(dependencies, dict) or set(dependencies) != {'openssl', 'zlib'}:
            raise ValueError('curl dependency identity missing')
        for dep, version in (('openssl', '3.5.9'), ('zlib', '1.3.2')):
            value = dependencies[dep]
            if (not isinstance(value, dict) or
                    set(value) != {'version', 'manifestSha256', 'prefix'} or
                    value['version'] != version or
                    value['manifestSha256'] != _sha256(install / (dep + '-build.json'))):
                raise ValueError('curl linked dependency manifest differs: ' + dep)
    return manifest


def verify_curl_package_inputs(install, source_cache, notice_root):
    """Requires OpenSSL's own package validator to have passed separately."""
    install, source_cache, notice_root = map(Path, (install, source_cache, notice_root))
    manifests = {name: verify_manifest(install, name) for name in ('zlib', 'curl')}
    bundles = []
    for library, pin in SOURCES.items():
        archive = source_cache / pin['archive']
        verify_file(archive, {k: pin[k] for k in ('sha256', 'bytes')})
        notice = notice_root / (library + '-' + pin['version'])
        provenance = _read_json(notice / 'provenance.json', library + ' notice')
        license_file = notice / 'LICENSE.txt'
        fingerprint = provenance.get('signatureVerification', {}).get('primary_fingerprint', '')
        if (not license_file.is_file() or license_file.is_symlink() or
                provenance.get('library') != library + ' ' + pin['version'] or
                provenance.get('sourceArchive') != pin['url'] or
                provenance.get('sourceArchiveSha256') != pin['sha256'] or
                provenance.get('sourceArchiveBytes') != pin['bytes'] or
                fingerprint.replace(' ', '') != pin['signingPrimaryFingerprint'] or
                provenance.get('licenseSha256') != _sha256(license_file)):
            raise ValueError('Dependency notice/source provenance differs: ' + library)
        bundles.append({'archive': archive, 'path': 'third-party-sources/' + pin['archive'],
                        'sha256': pin['sha256'], 'reference': {
                            'library': library, 'version': pin['version'], 'url': pin['url'],
                            'signingPrimaryFingerprint': pin['signingPrimaryFingerprint']}})
    verify_packaged_curl(install, manifests)
    return {'manifests': manifests, 'sourceBundles': bundles}


def verify_packaged_curl(app, manifests):
    """Recheck output after every copy; never silently ship legacy SSL runtimes."""
    app = Path(app)
    for path in app.rglob('*'):
        if path.name.lower() in {'libeay32.dll', 'ssleay32.dll'}:
            raise ValueError('Legacy OpenSSL runtime remains in package: ' + path.name)
    for library, name in (('curl', 'libcurl.dll'), ('zlib', 'zlib1.dll')):
        path = app / name
        verify_file(path, manifests[library]['outputs']['bin/' + name])
        _require_win32_pe(path)
