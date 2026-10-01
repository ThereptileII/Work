#!/usr/bin/env python3
"""Focused fail-closed tests for the maintained curl/zlib package boundary."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import curl_package


class CurlPackageBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='curl-package-test-')
        self.root = Path(self.tmp.name)
        self.install = self.root / 'install'
        self.cache = self.root / 'source-cache'
        self.notices = self.root / 'notices'
        self.install.mkdir()
        self.cache.mkdir()
        self.notices.mkdir()
        self._old_sources = curl_package.SOURCES
        self.sources = {
            'curl': {
                'version': 'test-curl', 'configuration': 'Win32 shared OpenSSL',
                'archive': 'test-curl.tar.xz', 'url': 'https://example.invalid/test-curl.tar.xz',
                'sha256': '', 'bytes': 0, 'signingPrimaryFingerprint': 'CURLTEST',
            },
            'zlib': {
                'version': 'test-zlib', 'configuration': 'Win32 shared',
                'archive': 'test-zlib.tar.gz', 'url': 'https://example.invalid/test-zlib.tar.gz',
                'sha256': '', 'bytes': 0, 'signingPrimaryFingerprint': 'ZLIBTEST',
            },
        }
        for library, pin in self.sources.items():
            source = (library + '-source-bytes').encode()
            pin['sha256'] = hashlib.sha256(source).hexdigest()
            pin['bytes'] = len(source)
            (self.cache / pin['archive']).write_bytes(source)
        curl_package.SOURCES = self.sources
        self._make_notices()
        self._make_install()

    def tearDown(self):
        curl_package.SOURCES = self._old_sources
        self.tmp.cleanup()

    @staticmethod
    def _pe(machine=0x14C, payload=b'fixture-pe'):
        data = bytearray(128 + len(payload))
        data[:2] = b'MZ'
        data[0x3C:0x40] = (64).to_bytes(4, 'little')
        data[64:68] = b'PE\0\0'
        data[68:70] = machine.to_bytes(2, 'little')
        data[128:] = payload
        return bytes(data)

    @staticmethod
    def _write_json(path, value, bom=True):
        path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(value, sort_keys=True).encode('utf-8')
        path.write_bytes((b'\xef\xbb\xbf' if bom else b'') + text)

    def _make_notices(self):
        for library, pin in self.sources.items():
            directory = self.notices / (library + '-' + pin['version'])
            directory.mkdir()
            license_bytes = (library + ' license').encode()
            (directory / 'LICENSE.txt').write_bytes(license_bytes)
            self._write_json(directory / 'provenance.json', {
                'library': library + ' ' + pin['version'],
                'sourceArchive': pin['url'],
                'sourceArchiveSha256': pin['sha256'],
                'sourceArchiveBytes': pin['bytes'],
                'signatureVerification': {'primary_fingerprint': pin['signingPrimaryFingerprint']},
                'licenseSha256': hashlib.sha256(license_bytes).hexdigest(),
            })

    def _record(self, value=b'placeholder'):
        return {'sha256': hashlib.sha256(value).hexdigest(), 'bytes': len(value)}

    def _make_install(self):
        self.libcurl = self._pe(payload=b'libcurl')
        self.zlib = self._pe(payload=b'zlib')
        zlib_outputs = {
            name: (self._record(self.zlib) if name == 'bin/zlib1.dll' else self._record(name.encode()))
            for name in curl_package.ZLIB_OUTPUTS
        }
        curl_outputs = {
            name: (self._record(self.libcurl) if name == 'bin/libcurl.dll' else self._record(name.encode()))
            for name in curl_package.CURL_OUTPUTS
        }
        zlib_manifest = {
            'schemaVersion': 1, 'library': 'zlib', 'version': self.sources['zlib']['version'],
            'configuration': self.sources['zlib']['configuration'], 'architecture': 'Win32', 'abi': 'x86',
            'runtime': 'MultiThreadedDLL (/MD)',
            'source': {k: self.sources['zlib'][k] for k in curl_package.SOURCE_KEYS},
            'buildSteps': {'configure': 'passed', 'compile': 'passed', 'test': 'passed', 'install': 'passed'},
            'outputs': zlib_outputs,
        }
        self._write_json(self.install / 'zlib-build.json', zlib_manifest)
        self._write_json(self.install / 'openssl-build.json', {'fixture': 'openssl manifest'})
        openssl_hash = curl_package._sha256(self.install / 'openssl-build.json')
        zlib_hash = curl_package._sha256(self.install / 'zlib-build.json')
        curl_manifest = {
            'schemaVersion': 1, 'library': 'curl', 'version': self.sources['curl']['version'],
            'configuration': self.sources['curl']['configuration'], 'architecture': 'Win32', 'abi': 'x86',
            'runtime': 'MultiThreadedDLL (/MD)',
            'source': {k: self.sources['curl'][k] for k in curl_package.SOURCE_KEYS},
            'buildSteps': {
                'configure': 'passed', 'compile': 'passed', 'test': 'passed', 'install': 'passed',
                'testTarget': 'tests', 'testsReported': 11, 'testsPassed': 11,
                'log': 'evidence/local/windows-curl-native-output.log',
                'logSha256': 'a' * 64,
            },
            'outputs': curl_outputs,
            'dependencies': {
                'openssl': {'version': '3.5.9', 'manifestSha256': openssl_hash, 'prefix': 'openssl'},
                'zlib': {'version': '1.3.2', 'manifestSha256': zlib_hash, 'prefix': 'zlib'},
            },
            'options': {}, 'versionOutput': 'curl test version', 'importOutput': 'libssl-3.dll',
            'cacheBuildwin': {},
        }
        curl_manifest['cacheBuildwin'] = {
            Path(name).name if name.startswith(('bin/', 'lib/')) else name:
            {'source': name, **record} for name, record in curl_outputs.items()
        }
        self._write_json(self.install / 'curl-build.json', curl_manifest)
        (self.install / 'libcurl.dll').write_bytes(self.libcurl)
        (self.install / 'zlib1.dll').write_bytes(self.zlib)

    def _verify(self):
        return curl_package.verify_curl_package_inputs(self.install, self.cache, self.notices)

    def test_valid_bom_manifests_and_inputs(self):
        result = self._verify()
        self.assertEqual(set(result['manifests']), {'curl', 'zlib'})
        self.assertEqual([bundle['reference']['library'] for bundle in result['sourceBundles']], ['curl', 'zlib'])

    def test_changed_or_missing_output_dll_is_rejected(self):
        (self.install / 'libcurl.dll').write_bytes(self.libcurl + b'changed')
        with self.assertRaisesRegex(ValueError, 'Missing or changed dependency file'):
            self._verify()
        (self.install / 'libcurl.dll').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing or changed dependency file'):
            self._verify()

    def test_wrong_dependency_manifest_hash_is_rejected(self):
        path = self.install / 'curl-build.json'
        manifest = json.loads(path.read_text(encoding='utf-8-sig'))
        manifest['dependencies']['zlib']['manifestSha256'] = 'b' * 64
        self._write_json(path, manifest)
        with self.assertRaisesRegex(ValueError, 'linked dependency manifest'):
            self._verify()

    def test_absent_or_altered_source_archive_is_rejected(self):
        archive = self.cache / self.sources['curl']['archive']
        archive.unlink()
        with self.assertRaisesRegex(ValueError, 'test-curl.tar.xz'):
            self._verify()
        archive.write_bytes(b'altered')
        with self.assertRaisesRegex(ValueError, 'test-curl.tar.xz'):
            self._verify()

    def test_wrong_architecture_is_rejected_even_when_hash_matches(self):
        wrong = self._pe(machine=0x8664, payload=b'wrong-arch')
        record = {'sha256': hashlib.sha256(wrong).hexdigest(), 'bytes': len(wrong)}
        path = self.install / 'libcurl.dll'
        path.write_bytes(wrong)
        manifest = json.loads((self.install / 'curl-build.json').read_text(encoding='utf-8-sig'))
        manifest['outputs']['bin/libcurl.dll'] = record
        manifest['cacheBuildwin']['libcurl.dll'] = {'source': 'bin/libcurl.dll', **record}
        self._write_json(self.install / 'curl-build.json', manifest)
        with self.assertRaisesRegex(ValueError, 'Win32/x86'):
            self._verify()

    def test_malformed_or_missing_output_inventory_is_rejected(self):
        path = self.install / 'zlib-build.json'
        manifest = json.loads(path.read_text(encoding='utf-8-sig'))
        manifest['outputs']['bin/zlib1.dll'] = {'sha256': 'bad'}
        self._write_json(path, manifest)
        with self.assertRaisesRegex(ValueError, 'Invalid output record'):
            self._verify()
        manifest['outputs'].pop('bin/zlib1.dll')
        self._write_json(path, manifest)
        with self.assertRaisesRegex(ValueError, 'output inventory'):
            self._verify()
        path.unlink()
        with self.assertRaisesRegex(ValueError, 'missing or not a regular file'):
            self._verify()

    def test_wrong_notice_is_rejected(self):
        path = self.notices / ('curl-' + self.sources['curl']['version']) / 'provenance.json'
        provenance = json.loads(path.read_text(encoding='utf-8-sig'))
        provenance['sourceArchiveSha256'] = 'c' * 64
        self._write_json(path, provenance)
        with self.assertRaisesRegex(ValueError, 'provenance'):
            self._verify()

    def test_failed_or_zero_upstream_tests_are_rejected(self):
        path = self.install / 'curl-build.json'
        original = json.loads(path.read_text(encoding='utf-8-sig'))
        for change in ({'test': 'failed'}, {'testsReported': 0, 'testsPassed': 0}):
            manifest = json.loads(json.dumps(original))
            manifest['buildSteps'].update(change)
            self._write_json(path, manifest)
            with self.assertRaisesRegex(ValueError, 'Incomplete dependency build|successful upstream execution'):
                self._verify()

    def test_post_copy_overwrite_is_rejected(self):
        result = self._verify()
        (self.install / 'zlib1.dll').write_bytes(b'overwrite')
        with self.assertRaisesRegex(ValueError, 'Missing or changed dependency file'):
            curl_package.verify_packaged_curl(self.install, result['manifests'])

    def test_nested_legacy_ssl_runtime_is_rejected(self):
        nested = self.install / 'plugins' / 'example'
        nested.mkdir(parents=True)
        (nested / 'ssleay32.dll').write_bytes(b'legacy')
        with self.assertRaisesRegex(ValueError, 'Legacy OpenSSL runtime'):
            self._verify()


if __name__ == '__main__':
    unittest.main()
