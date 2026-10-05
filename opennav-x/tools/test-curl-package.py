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
        self.openssl_prefix = self.root / 'openssl-prefix'
        (self.openssl_prefix / 'bin').mkdir(parents=True)
        self.openssl_exe = self.openssl_prefix / 'bin/openssl.exe'
        self.openssl_exe.write_bytes(self._pe(payload=b'openssl.exe'))
        self.openssl_version = 'OpenSSL 3.5.9 fixture version'
        self._write_json(self.install / 'openssl-build.json', {
            'versionOutput': self.openssl_version,
            'outputs': {'bin/openssl.exe': self._record(self.openssl_exe.read_bytes())},
        })
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
                'testHost': {'path': 'C:/msys64/usr/bin/perl.exe', 'sha256': 'b' * 64,
                             'bytes': 100, 'os': 'cygwin',
                             'runtimePath': 'C:/msys64/usr/bin/msys-2.0.dll',
                             'runtimeSha256': 'c' * 64, 'runtimeBytes': 100},
                'log': 'evidence/local/windows-curl-native-output.log',
                'logSha256': 'a' * 64,
                'certificatePatch': {
                    **curl_package.CURL_CERTIFICATE_PATCH,
                    'helperSha256': curl_package._sha256(Path(__file__).with_name('patch-curl-test-openssl.py')),
                },
                'certificateTool': {
                    'path': str(self.openssl_exe), 'sha256': curl_package._sha256(self.openssl_exe),
                    'bytes': self.openssl_exe.stat().st_size,
                    'versionOutput': self.openssl_version,
                },
                'certificateProbe': 'passed',
            },
            'outputs': curl_outputs,
            'dependencies': {
                'openssl': {'version': '3.5.9', 'manifestSha256': openssl_hash, 'prefix': str(self.openssl_prefix)},
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

    def _producer_layout(self):
        """Actual separate-prefix layout, with inert fixture outputs only."""
        import importlib.util
        spec = importlib.util.spec_from_file_location('adapter_prepare', Path(__file__).with_name('prepare-ocharts-adapter.py'))
        prep = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(prep)
        self.zlib_prefix = self.root / 'zlib-prefix'
        self.zlib_prefix.mkdir()
        openssl = json.loads((self.install / 'openssl-build.json').read_text(encoding='utf-8-sig'))
        for name in curl_package.OPENSSL_OUTPUTS:
            path = self.openssl_prefix / name
            path.parent.mkdir(parents=True, exist_ok=True)
            if name != 'bin/openssl.exe':
                path.write_bytes(name.encode())
            openssl['outputs'][name] = self._record(path.read_bytes())
        self._write_json(self.openssl_prefix / 'openssl-build.json', openssl)
        zlib = json.loads((self.install / 'zlib-build.json').read_text(encoding='utf-8-sig'))
        self._write_json(self.zlib_prefix / 'zlib-build.json', zlib)
        curl = json.loads((self.install / 'curl-build.json').read_text(encoding='utf-8-sig'))
        for library, prefix, manifest in [('curl', self.install, curl), ('zlib', self.zlib_prefix, zlib)]:
            for name in manifest['outputs']:
                path = prefix / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(self.libcurl if name == 'bin/libcurl.dll' else self.zlib if name == 'bin/zlib1.dll' else name.encode())
        for library, prefix in [('openssl', self.openssl_prefix), ('zlib', self.zlib_prefix)]:
            curl['dependencies'][library]['prefix'] = str(prefix)
            curl['dependencies'][library]['manifestSha256'] = curl_package._sha256(prefix / (library + '-build.json'))
            (self.install / (library + '-build.json')).unlink()
        self._write_json(self.install / 'curl-build.json', curl)
        return lambda: prep.verify_producer_dependencies(self.install, self.openssl_prefix, self.zlib_prefix)

    def test_separate_producer_prefixes_and_default_package_refusal(self):
        verify = self._producer_layout()
        self.assertEqual(set(verify()), {'curl','zlib'})
        # The original call still refuses this layout: no automatic prefix trust.
        with self.assertRaises((ValueError, FileNotFoundError)):
            curl_package.verify_manifest(self.install, 'curl')

    def test_producer_wrong_expected_or_recorded_prefix_rejected(self):
        verify = self._producer_layout()
        for value in ({}, {'openssl':self.openssl_prefix}, {'openssl':self.install,'zlib':self.zlib_prefix}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                curl_package.verify_manifest(self.install,'curl',dependency_prefixes=value)
        path = self.install / 'curl-build.json'
        original = path.read_bytes()
        for library in ('openssl','zlib'):
            manifest = json.loads(original.decode('utf-8-sig'))
            manifest['dependencies'][library]['prefix'] = str(self.install)
            self._write_json(path, manifest)
            with self.subTest(library=library), self.assertRaisesRegex(ValueError,'prefix differs'):
                verify()
        path.write_bytes(original)

    def test_producer_missing_changed_manifests_and_outputs_rejected(self):
        verify = self._producer_layout()
        paths = [self.openssl_prefix/'openssl-build.json', self.zlib_prefix/'zlib-build.json',
                 self.openssl_prefix/'bin/openssl.exe', self.openssl_prefix/'lib/libssl.lib',
                 self.zlib_prefix/'bin/zlib1.dll', self.install/'lib/libcurl.lib',
                 self.install/'include/curl/curl.h']
        for path in paths:
            original = path.read_bytes()
            for missing in (False,True):
                if missing: path.unlink()
                else: path.write_bytes(original+b'changed')
                with self.subTest(path=path.name,missing=missing), self.assertRaises((ValueError,FileNotFoundError)):
                    verify()
                path.write_bytes(original)
        self.assertEqual(set(verify()), {'curl','zlib'})

    def test_producer_no_colocated_decoy_or_relative_prefix_fallback(self):
        verify = self._producer_layout()
        for library,prefix in [('openssl',self.openssl_prefix),('zlib',self.zlib_prefix)]:
            source=prefix/(library+'-build.json')
            original=source.read_bytes()
            (self.install/source.name).write_bytes(original)
            source.unlink()
            with self.subTest(library=library), self.assertRaises(ValueError):
                verify()
            source.write_bytes(original)
        path=self.install/'curl-build.json'
        manifest=json.loads(path.read_text(encoding='utf-8-sig'))
        manifest['dependencies']['openssl']['prefix']='openssl-prefix'
        self._write_json(path,manifest)
        with self.assertRaisesRegex(ValueError,'prefix differs'):
            verify()

    def _verify(self):
        return curl_package.verify_curl_package_inputs(self.install, self.cache, self.notices)

    def test_valid_bom_manifests_and_inputs(self):
        result = self._verify()
        self.assertEqual(set(result['manifests']), {'curl', 'zlib'})
        self.assertEqual([bundle['reference']['library'] for bundle in result['sourceBundles']], ['curl', 'zlib'])

    def test_reviewed_zlib_lock_manifest_is_accepted_and_source_mutations_rejected(self):
        lock = json.loads((Path(__file__).with_name('windows-zlib.lock.json')).read_text())
        source = {key: lock[key] for key in curl_package.SOURCE_KEYS}
        production_source = {key: self._old_sources['zlib'][key] for key in curl_package.SOURCE_KEYS}
        self.assertEqual(production_source, source)
        provenance = json.loads((Path(__file__).parents[1] / 'docs/third-party/zlib-1.3.2/provenance.json').read_text())
        self.assertEqual(provenance['sourceArchive'], lock['url'])
        self.assertEqual(provenance['sourceArchiveSha256'], lock['sha256'])
        self.assertEqual(provenance['sourceArchiveBytes'], lock['bytes'])
        manifest = {
            'schemaVersion': 1, 'library': 'zlib', 'version': lock['version'],
            'configuration': lock['configuration'], 'architecture': 'Win32', 'abi': 'x86',
            'runtime': lock['runtime'], 'source': source,
            'buildSteps': {'configure': 'passed', 'compile': 'passed', 'test': 'passed', 'install': 'passed'},
            'outputs': {name: self._record(name.encode()) for name in curl_package.ZLIB_OUTPUTS},
        }
        manifest_path = self.install / 'zlib-build.json'
        self._write_json(manifest_path, manifest)
        with mock.patch.object(curl_package, 'SOURCES', self._old_sources):
            self.assertEqual(curl_package.verify_manifest(self.install, 'zlib')['source'], source)
            for key, value in (('url', 'https://example.invalid/other.tar.gz'),
                               ('sha256', '0' * 64), ('bytes', lock['bytes'] + 1)):
                changed = json.loads(json.dumps(manifest))
                changed['source'][key] = value
                self._write_json(manifest_path, changed)
                with self.assertRaisesRegex(ValueError, 'Unsupported dependency build identity: zlib'):
                    curl_package.verify_manifest(self.install, 'zlib')

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

    def test_curl_test_host_identity_is_required(self):
        path = self.install / 'curl-build.json'
        original = json.loads(path.read_text(encoding='utf-8-sig'))
        for field, value in (
            ('os', 'MSWin32'), ('os', []), ('os', {}),
            ('path', 'usr/bin/perl.exe'),
            ('runtimePath', 'C:/other/msys-2.0.dll'),
            ('runtimeSha256', 'invalid'), ('bytes', 0),
        ):
            with self.subTest(field=field, value=value):
                changed = json.loads(json.dumps(original))
                changed['buildSteps']['testHost'][field] = value
                self._write_json(path, changed)
                with self.assertRaisesRegex(ValueError, 'test-host|output record'):
                    self._verify()
        changed = json.loads(json.dumps(original))
        del changed['buildSteps']['testHost']
        self._write_json(path, changed)
        with self.assertRaisesRegex(ValueError, 'test-host'):
            self._verify()

    def test_certificate_patch_tool_and_probe_tampering_are_rejected(self):
        path = self.install / 'curl-build.json'
        original = json.loads(path.read_text(encoding='utf-8-sig'))
        for field, value in (
            ('originalSha256', '0' * 64), ('patchedSha256', '0' * 64),
            ('helperSha256', '0' * 64),
        ):
            with self.subTest(field=field):
                changed = json.loads(json.dumps(original))
                changed['buildSteps']['certificatePatch'][field] = value
                self._write_json(path, changed)
                with self.assertRaisesRegex(ValueError, 'patch or probe provenance'):
                    self._verify()
        for field, value in (('certificateProbe', 'failed'), ('certificatePatch', None)):
            with self.subTest(field=field):
                changed = json.loads(json.dumps(original))
                changed['buildSteps'][field] = value
                self._write_json(path, changed)
                with self.assertRaisesRegex(ValueError, 'patch or probe provenance'):
                    self._verify()
        self._write_json(path, original)
        self.openssl_exe.write_bytes(self.openssl_exe.read_bytes() + b'changed')
        with self.assertRaisesRegex(ValueError, 'Missing or changed dependency file'):
            self._verify()
        self.openssl_exe.write_bytes(self._pe(payload=b'openssl.exe'))
        openssl_path = self.install / 'openssl-build.json'
        openssl_manifest = json.loads(openssl_path.read_text(encoding='utf-8-sig'))
        openssl_manifest['outputs']['bin/openssl.exe']['sha256'] = '0' * 64
        self._write_json(openssl_path, openssl_manifest)
        original['dependencies']['openssl']['manifestSha256'] = curl_package._sha256(openssl_path)
        self._write_json(path, original)
        with self.assertRaisesRegex(ValueError, 'verified OpenSSL producer'):
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
