#!/usr/bin/env python3
"""Disposable copied-artifact verification; no external process or full build."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock
import zipfile

from updater_package import verify_updater_package, GO_VERSION, MAIN_MODULE, SOURCE_RELATIVE, BUNDLE_PATH


class UpdaterPackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='skager-updater-package-')
        self.addCleanup(self.temp.cleanup)
        self.app = Path(self.temp.name)
        self.commit = 'a' * 40
        self.archive = self.app / SOURCE_RELATIVE
        self.archive.parent.mkdir(parents=True)
        with zipfile.ZipFile(self.archive, 'x') as source:
            source.writestr('LICENSE', b'Original upstream license\r\n')
        self.binary = self.app / 'skager-start.exe'
        pe = bytearray(128)
        pe[:2] = b'MZ'
        struct.pack_into('<I', pe, 60, 64)
        pe[64:68] = b'PE\0\0'
        struct.pack_into('<H', pe, 68, 0x14c)
        struct.pack_into('<H', pe, 88, 0x10b)
        self.binary.write_bytes(pe)
        checksum = 'h1:' + base64.b64encode(bytes(32)).decode()
        self.record = {
            'schema': 1, 'productCommit': self.commit, 'goVersion': GO_VERSION,
            'binary': {'path': 'skager-start.exe', 'sha256': self.sha(self.binary)},
            'sourceBundle': {'archive': SOURCE_RELATIVE, 'path': BUNDLE_PATH, 'sha256': self.sha(self.archive),
                             'reference': {'schema': 1, 'productCommit': self.commit, 'goVersion': GO_VERSION,
                                           'mainModule': 'tools/update-verifier',
                                           'modules': [{'path': 'public.example/module', 'version': 'v1.0.0',
                                                        'sum': checksum, 'goModSum': checksum,
                                                        'sourcePrefix': 'modules/public.example/module@v1.0.0'}],
                                           'build': 'CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build -mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start'}},
            'buildInfo': f'skager-start.exe: {GO_VERSION}\n\tpath\t{MAIN_MODULE}/cmd/skager-start\n\tbuild\t-trimpath=true\n\tbuild\tCGO_ENABLED=0\n\tbuild\tGOARCH=386\n\tbuild\tGOOS=windows\n\tbuild\tvcs.revision={self.commit}\n\tbuild\tvcs.modified=false\n',
        }
        self.record_file = self.archive.parent / 'build.json'
        self.write()

    @staticmethod
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write(self, record=None):
        self.record_file.write_text(json.dumps(record or self.record), encoding='utf-8')

    def test_returns_actual_copied_source_descriptor_without_archive_expansion(self):
        with mock.patch.object(zipfile, 'ZipFile', side_effect=AssertionError('Must not expand dependency source again')):
            result = verify_updater_package(self.app, self.commit)
        self.assertEqual(result['archive'], self.archive)
        self.assertEqual(result['path'], BUNDLE_PATH)
        self.assertEqual(result['sha256'], self.sha(self.archive))
        self.assertEqual(set(result), {'archive', 'path', 'sha256', 'reference'})

    def test_changed_source_or_binary_rejected(self):
        for file in (self.archive, self.binary):
            with self.subTest(file=file.name):
                original = file.read_bytes()
                file.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
                with self.assertRaisesRegex(ValueError, 'changed'):
                    verify_updater_package(self.app, self.commit)
                file.write_bytes(original)

    def test_exact_commit_compiler_and_schema(self):
        mutations = [
            lambda x: x.update(productCommit='b' * 40),
            lambda x: x.update(goVersion='go1.27.0'),
            lambda x: x.update(schema=True),
            lambda x: x.update(extra='unreviewed'),
            lambda x: x['sourceBundle']['reference'].update(productCommit='b' * 40),
            lambda x: x['sourceBundle']['reference'].update(goVersion='go1.27.0'),
            lambda x: x['sourceBundle']['reference']['modules'][0].update(sum='unverified'),
            lambda x: x['sourceBundle']['reference']['modules'].append(x['sourceBundle']['reference']['modules'][0].copy()),
            lambda x: x.update(buildInfo=x['buildInfo'].replace('GOARCH=386', 'GOARCH=amd64')),
            lambda x: x.update(buildInfo=x['buildInfo'].replace('vcs.modified=false', 'vcs.modified=true')),
            lambda x: x.update(buildInfo=x['buildInfo'] + '\tbuild\tGOARCH=386\n'),
        ]
        for mutate in mutations:
            record = copy.deepcopy(self.record)
            mutate(record)
            self.write(record)
            with self.subTest(record=record), self.assertRaises(ValueError):
                verify_updater_package(self.app, self.commit)
        with self.assertRaisesRegex(ValueError, 'Exact product commit'):
            verify_updater_package(self.app, 'unverified')

    def test_ambiguous_json_rejected(self):
        text = self.record_file.read_text(encoding='utf-8')
        self.record_file.write_text('{"schema":1,' + text[1:], encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            verify_updater_package(self.app, self.commit)

    def test_fixed_relative_paths_only(self):
        for field, value in [('archive', '../outside.zip'), ('archive', str(self.archive)),
                             ('archive', 'opennav\\third-party\\updater\\updater-source.zip'),
                             ('path', 'third-party-sources/../../escape.zip')]:
            record = copy.deepcopy(self.record)
            record['sourceBundle'][field] = value
            self.write(record)
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'fixed relative'):
                verify_updater_package(self.app, self.commit)
        record = copy.deepcopy(self.record)
        record['binary']['path'] = '../skager-start.exe'
        self.write(record)
        with self.assertRaisesRegex(ValueError, 'fixed relative'):
            verify_updater_package(self.app, self.commit)

    def test_pe_architecture_verified_independently_of_hash(self):
        pe = bytearray(self.binary.read_bytes())
        struct.pack_into('<H', pe, 68, 0x8664)
        self.binary.write_bytes(pe)
        self.record['binary']['sha256'] = self.sha(self.binary)
        self.write()
        with self.assertRaisesRegex(ValueError, 'Win32'):
            verify_updater_package(self.app, self.commit)

    def test_redirected_source_refused(self):
        original = self.app / 'outside-source.zip'
        self.archive.rename(original)
        try:
            self.archive.symlink_to(original)
        except OSError:
            self.skipTest('Host cannot create disposable symlink')
        with self.assertRaisesRegex(ValueError, 'reparse'):
            verify_updater_package(self.app, self.commit)

    def test_default_trust_fixture_and_oversized_record_refused(self):
        trust = self.app / 'update-trust.json'
        trust.write_text('{"bootstrapRoot":"fixture"}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'must not bundle update trust'):
            verify_updater_package(self.app, self.commit)
        trust.unlink()
        self.record_file.write_bytes(b'x' * ((1 << 20) + 1))
        with self.assertRaisesRegex(ValueError, 'type or size'):
            verify_updater_package(self.app, self.commit)


if __name__ == '__main__':
    unittest.main()
