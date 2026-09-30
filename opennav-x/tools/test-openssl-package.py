#!/usr/bin/env python3
"""Owned inert fixtures for the OpenSSL native package gate."""
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import openssl_package
from openssl_package import verify_openssl_package_inputs, verify_packaged_openssl


def digest(data):
    return hashlib.sha256(data).hexdigest()


def pe(machine=0x14c):
    data = bytearray(128)
    data[:2] = b'MZ'
    struct.pack_into('<I', data, 0x3c, 64)
    data[64:68] = b'PE\0\0'
    struct.pack_into('<H', data, 68, machine)
    return bytes(data)


class OpenSslPackageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='opennav-openssl-package-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.install = self.root / 'install'
        self.notice = self.root / 'notice'
        self.notice.mkdir()
        archive_bytes = b'inert verified OpenSSL source tar fixture\n'
        self.archive = self.root / 'openssl-3.5.9.tar.gz'
        self.archive.write_bytes(archive_bytes)
        self.source_sha_patch = patch.object(openssl_package, 'EXPECTED_SOURCE_SHA256', digest(archive_bytes))
        self.source_bytes_patch = patch.object(openssl_package, 'EXPECTED_SOURCE_BYTES', len(archive_bytes))
        self.source_sha_patch.start()
        self.source_bytes_patch.start()
        self.addCleanup(self.source_bytes_patch.stop)
        self.addCleanup(self.source_sha_patch.stop)
        outputs = {}
        for relative in ('bin/libssl-3.dll', 'bin/libcrypto-3.dll'):
            binary = pe()
            path = self.install / Path(relative).name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(binary)
            outputs[relative] = {'sha256': digest(binary), 'bytes': len(binary)}
        for relative in ('include/openssl/opensslv.h', 'lib/libssl.lib', 'lib/libcrypto.lib'):
            outputs[relative] = {'sha256': digest(relative.encode()), 'bytes': len(relative)}
        self.lock = {
            'version': '3.5.9', 'configuration': 'VC-WIN32 shared',
            'archive': self.archive.name, 'url': openssl_package.EXPECTED_SOURCE_URL,
            'sha256': digest(archive_bytes), 'bytes': len(archive_bytes),
            'signingPrimaryFingerprint': openssl_package.EXPECTED_SIGNING_FINGERPRINT,
            'buildTools': {'nasm': {'version': '3.02', 'archive': 'nasm.zip',
                'url': 'https://example.invalid/nasm.zip', 'sha256': '1' * 64,
                'bytes': 10, 'provenance': 'fixture'}},
        }
        source = {key: self.lock[key] for key in (
            'url', 'archive', 'sha256', 'bytes', 'signingPrimaryFingerprint')}
        cache = {openssl_package.OPENSSL_CACHE_PATHS[relative]: {'source': relative, **record}
                 for relative, record in outputs.items()}
        self.manifest = {
            'schemaVersion': 1, 'library': 'OpenSSL', 'version': self.lock['version'],
            'configuration': self.lock['configuration'], 'architecture': 'Win32', 'abi': 'x86',
            'source': source, 'outputs': outputs, 'cacheBuildwin': cache,
            'toolchain': {'compiler': 'fixture', 'nasmArchiveSha256': '1' * 64},
            'buildSteps': {'configure': 'passed', 'compile': 'passed', 'test': 'passed', 'install': 'passed'},
            'versionOutput': 'OpenSSL 3.5.9 fixture',
        }
        self.assertIn('include/openssl/opensslv.h', self.manifest['cacheBuildwin'])
        self.assertNotIn('opensslv.h', self.manifest['cacheBuildwin'])
        self.lock_path = self.root / 'windows-openssl.lock.json'
        self.write_json(self.lock_path, self.lock)
        self.write_json(self.install / 'openssl-build.json', self.manifest, bom=True)
        license_bytes = b'Apache License fixture\n'
        (self.notice / 'LICENSE.txt').write_bytes(license_bytes)
        self.write_json(self.notice / 'provenance.json', {
            'library': 'OpenSSL 3.5.9', 'sourceArchiveSha256': digest(archive_bytes),
            'sourceArchiveBytes': len(archive_bytes), 'licenseSha256': digest(license_bytes),
            'signatureVerification': {
                'primaryFingerprint': openssl_package.EXPECTED_SIGNING_FINGERPRINT},
        })

    @staticmethod
    def write_json(path, value, bom=False):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding='utf-8-sig' if bom else 'utf-8')

    def verify(self):
        return verify_openssl_package_inputs(
            self.install, self.lock_path, self.archive, self.notice)

    def test_verified_inputs_produce_bounded_source_inventory(self):
        source = self.verify()
        self.assertEqual(source['sourceBundle']['path'], 'third-party-sources/openssl-3.5.9.tar.gz')
        self.assertEqual(source['sourceBundle']['sha256'], self.lock['sha256'])

    def test_tampered_dll_is_refused(self):
        (self.install / 'libssl-3.dll').write_bytes(pe() + b'tampered')
        with self.assertRaisesRegex(ValueError, 'differs from'):
            self.verify()

    def test_missing_dll_is_refused(self):
        (self.install / 'libcrypto-3.dll').unlink()
        with self.assertRaisesRegex(ValueError, 'DLL missing'):
            self.verify()

    def test_wrong_installed_source_manifest_is_refused(self):
        manifest = json.loads((self.install / 'openssl-build.json').read_text(encoding='utf-8-sig'))
        manifest['source']['sha256'] = '0' * 64
        self.write_json(self.install / 'openssl-build.json', manifest)
        with self.assertRaisesRegex(ValueError, 'does not match'):
            self.verify()

    def test_missing_or_inconsistent_notice_is_refused(self):
        (self.notice / 'LICENSE.txt').unlink()
        with self.assertRaisesRegex(ValueError, 'notice missing'):
            self.verify()

    def test_changed_or_missing_source_archive_is_refused(self):
        self.archive.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'source archive'):
            self.verify()

    def test_x64_dll_is_refused_even_if_hash_matches_lock(self):
        binary = pe(0x8664)
        path = self.install / 'libssl-3.dll'
        path.write_bytes(binary)
        record = {'sha256': digest(binary), 'bytes': len(binary)}
        self.manifest['outputs']['bin/libssl-3.dll'] = record
        self.manifest['cacheBuildwin']['libssl-3.dll'] = {'source': 'bin/libssl-3.dll', **record}
        self.write_json(self.install / 'openssl-build.json', self.manifest, bom=True)
        with self.assertRaisesRegex(ValueError, 'Win32'):
            self.verify()

    def test_post_copy_packaged_dll_tampering_is_refused(self):
        checked = self.verify()
        package = self.root / 'package'
        package.mkdir()
        for name in ('libssl-3.dll', 'libcrypto-3.dll'):
            (package / name).write_bytes((self.install / name).read_bytes())
        verify_packaged_openssl(package, checked['manifest'])
        (package / 'libssl-3.dll').write_bytes(pe() + b'runtime overwrite')
        with self.assertRaisesRegex(ValueError, 'Packaged OpenSSL DLL differs'):
            verify_packaged_openssl(package, checked['manifest'])


if __name__ == '__main__':
    unittest.main()
