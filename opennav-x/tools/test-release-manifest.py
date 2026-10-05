#!/usr/bin/env python3
"""Offline release-boundary tests with inert archive fixtures; no app execution."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import warnings
import zipfile

import release_manifest as release

COMMIT = '1' * 40
VERSION = '0.4.0-beta2'


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in release.PRODUCT_FILES:
            (self.root / name).write_bytes(b'Inert test fixture\n')
        (self.root / 'QUALIFICATION.txt').write_text('Test-only pending qualification\n')
        self.notes = (self.root / 'SKAGER-Beta2-Release-Notes.md').read_bytes()
        self.build = dict(version=VERSION, commit=COMMIT, test_fixtures=False,
                          build_purpose='INSTALLED PRODUCT', xnav_hardware_output_policy='status-only',
                          executable_sha256=release.digest(b'inert exe'))
        self.source = dict(productCommit=COMMIT, upstreamCommit=release.UPSTREAM,
                           openCpnVersion='5.12.4', files={
                               'opennav-x/docs/beta2/SKAGER-Beta2-Release-Notes.md':
                                   {'sha256': release.digest(self.notes)}})
        self.archives()

    def archives(self, portable_extra=()):
        prefix = 'SKAGER-Beta2-Portable-Recovery/'
        with zipfile.ZipFile(self.root / 'SKAGER-Beta2-Portable-Recovery.zip', 'w') as archive:
            for name, data in [
                (prefix + 'docs/PRODUCT_BUILD.json', json.dumps(self.build)),
                (prefix + 'app/opencpn.exe', b'inert exe'),
                (prefix + 'docs/SKAGER-Beta2-Release-Notes.md', self.notes),
                (prefix + 'FILE_SHA256.json', json.dumps({
                    'docs/SKAGER-Beta2-Release-Notes.md': release.digest(self.notes)})),
                *portable_extra,
            ]:
                archive.writestr(name, data)
        with zipfile.ZipFile(self.root / 'SKAGER-Beta2-source.zip', 'w') as archive:
            archive.writestr('SOURCE_REFERENCE.json', json.dumps(self.source))
            archive.writestr('opennav-x/docs/beta2/SKAGER-Beta2-Release-Notes.md', self.notes)
        self.sums()

    def sums(self):
        (self.root / 'SHA256SUMS.txt').write_text(''.join(
            release.digest((self.root / name).read_bytes()) + '  ' + name + '\n'
            for name in sorted(release.PRODUCT_FILES)))

    def create(self, **kwargs):
        values = dict(directory=self.root, commit=COMMIT, run_id='42', run_attempt='1', version=VERSION)
        return release.create(**dict(values, **kwargs))

    def rewrite(self, record):
        (self.root / release.MANIFEST).write_text(json.dumps(record))

    def test_default_staging_and_readonly_verification(self):
        record = self.create()
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        self.assertEqual(release.verify(self.root, COMMIT), record)
        self.assertEqual(record['channel'], 'staging')
        self.assertEqual(record['designReview'], 'not-requested')
        self.assertEqual(record['candidateId'], VERSION + '-run42-attempt1-' + COMMIT[:12])
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.root.iterdir()})
        with self.assertRaises(ValueError):
            self.create()

    def test_cli_roundtrip_and_wrong_commit(self):
        tool = str(Path(release.__file__))
        result = subprocess.run([sys.executable, tool, 'create', '--directory', str(self.root),
                                 '--commit', COMMIT, '--run-id', '42', '--run-attempt', '1',
                                 '--version', VERSION], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), release.verify(self.root))
        result = subprocess.run([sys.executable, tool, 'verify', '--directory', str(self.root),
                                 '--commit', '2' * 40], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Requested commit differs', result.stderr)

    def test_optional_receipts_are_hashed_and_closed(self):
        for name in release.OPTIONAL_FILES:
            (self.root / name).write_text('{"status":"test-only","designReview":"not-requested"}')
        record = self.create()
        self.assertEqual(len(record['files']), 10)
        release.verify(self.root)
        (self.root / 'RETEST_SUPPORT.json').write_text('{}')
        with self.assertRaises(ValueError):
            release.verify(self.root)
        (self.root / 'RETEST_SUPPORT.json').unlink()
        with self.assertRaises(ValueError):
            release.verify(self.root)

    def test_requested_design_review_metadata_alignment(self):
        path = self.root / 'QUALIFICATION.json'
        path.write_text(json.dumps({'designReview': 'requested'}))
        record = self.create()
        self.assertEqual(record['designReview'], 'requested')
        self.assertEqual(release.verify(self.root), record)
        self.rewrite(dict(record, designReview='not-requested'))
        with self.assertRaisesRegex(ValueError, 'scheduling metadata differs'):
            release.verify(self.root)
        (self.root / release.MANIFEST).unlink()
        path.write_text(json.dumps({'designReview': 'passed'}))
        with self.assertRaises(ValueError):
            self.create()
        self.assertFalse((self.root / release.MANIFEST).exists())

    def test_tampering_and_missing_files(self):
        self.create()
        for name in release.RELEASE_FILES:
            with self.subTest(name=name):
                path = self.root / name
                content = path.read_bytes()
                path.write_bytes(content + b'tamper')
                with self.assertRaises(ValueError):
                    release.verify(self.root)
                path.unlink()
                with self.assertRaises(ValueError):
                    release.verify(self.root)
                path.write_bytes(content)

    def test_extra_case_collision_and_unsafe_paths(self):
        for name in ['extra.txt', '.hidden']:
            with self.subTest(name=name):
                path = self.root / name
                path.write_text('extra')
                with self.assertRaises(ValueError):
                    self.create()
                path.unlink()
        # Windows cannot create these names as single regular files. Exercise
        # the same basename boundary without relying on host filesystem rules.
        for name in ['bad\\path', 'CON.txt']:
            with self.subTest(name=name), self.assertRaises(ValueError):
                release.safe_name(name)
        # A case-insensitive filesystem cannot hold both names. Supply the
        # conflicting directory listing without overwriting the Setup fixture.
        entries = [*self.root.iterdir(), self.root / 'skager-beta2-setup.exe']
        with patch.object(Path, 'iterdir', return_value=iter(entries)):
            with self.assertRaisesRegex(ValueError, 'Case-colliding'):
                self.create()
        (self.root / 'folder').mkdir()
        with self.assertRaises(ValueError):
            self.create()

    def test_empty_required_file(self):
        for name in release.RELEASE_FILES:
            with self.subTest(name=name):
                path = self.root / name
                content = path.read_bytes()
                path.write_bytes(b'')
                with self.assertRaises(ValueError):
                    self.create()
                path.write_bytes(content)

    def test_symlinks_refused(self):
        path = self.root / 'QUALIFICATION.txt'
        path.unlink()
        path.symlink_to(self.root / 'SKAGER-Beta2-Setup.exe')
        with self.assertRaises(ValueError):
            self.create()
        path.unlink()
        path.symlink_to(self.root / 'missing')
        with self.assertRaises(ValueError):
            self.create()

    def test_manifest_symlink_refused(self):
        self.create()
        content = (self.root / release.MANIFEST).read_bytes()
        with tempfile.TemporaryDirectory() as external:
            target = Path(external) / 'original.json'
            target.write_bytes(content)
            (self.root / release.MANIFEST).unlink()
            (self.root / release.MANIFEST).symlink_to(target)
            with self.assertRaises(ValueError):
                release.verify(self.root)

    def test_noncanonical_identity(self):
        for field in ('run_id', 'run_attempt'):
            for value in (True, 1, 1.0, None, '0', '-1', '01', '1\n', '1.0', '1e3', '../42'):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.create(**{field: value})
        for value in ('../version', '-option', '1', '', '0.4.0 beta2'):
            with self.assertRaises(ValueError):
                self.create(version=value)
        with self.assertRaises(ValueError):
            self.create(commit='2' * 40)
        self.assertFalse((self.root / release.MANIFEST).exists())

    def test_manifest_identity_and_schema_tampering(self):
        original = self.create()
        for key, value in [('runId', 42), ('runAttempt', True), ('schemaVersion', True),
                           ('channel', 'production'), ('designReview', 'passed'),
                           ('candidateId', 'different'), ('commit', '2' * 40),
                           ('version', '0.4.1'), ('inventorySha256', '0' * 64)]:
            with self.subTest(key=key):
                self.rewrite(dict(original, **{key: value}))
                with self.assertRaises(ValueError):
                    release.verify(self.root)
        self.rewrite(original)
        content = (self.root / release.MANIFEST).read_text()
        (self.root / release.MANIFEST).write_text(content[:-1] + ',"channel":"staging"}')
        with self.assertRaises(ValueError):
            release.verify(self.root)

    def test_manifest_paths_duplicates_and_numeric_sizes(self):
        original = self.create()
        for name in ('../outside', '/absolute', 'sub/file', 'back\\file', 'CON', '.hidden'):
            record = copy.deepcopy(original)
            record['files'][0]['name'] = name
            self.rewrite(record)
            with self.assertRaises(ValueError):
                release.verify(self.root)
        for change in ('duplicate', 'case', 'boolean', 'float'):
            record = copy.deepcopy(original)
            if change in ('duplicate', 'case'):
                record['files'][0]['name'] = record['files'][1]['name']
                if change == 'case':
                    record['files'][0]['name'] = record['files'][0]['name'].upper()
            else:
                record['files'][0]['size'] = True if change == 'boolean' else 1.5
            record['inventorySha256'] = release.digest(release.canonical(record['files']))
            self.rewrite(record)
            with self.assertRaises(ValueError):
                release.verify(self.root)

    def test_checksum_manifest_closed_and_unique(self):
        path = self.root / 'SHA256SUMS.txt'
        original = path.read_text()
        for value in (original + original.splitlines()[0] + '\n',
                      original + '0' * 64 + '  ../outside\n',
                      '\n'.join(original.splitlines()[1:]), original.replace('  SKAGER', ' *SKAGER')):
            path.write_text(value)
            with self.assertRaises(ValueError):
                self.create()

    def test_embedded_product_and_source_identity(self):
        for target, key, value in [(self.build, 'test_fixtures', True),
                                    (self.build, 'xnav_hardware_output_policy', 'enabled'),
                                    (self.build, 'version', '0.4.1'),
                                    (self.build, 'executable_sha256', '0' * 64),
                                    (self.source, 'productCommit', '2' * 40),
                                    (self.source, 'upstreamCommit', '2' * 40)]:
            previous = target[key]
            target[key] = value
            self.archives()
            with self.assertRaises(ValueError):
                self.create()
            target[key] = previous

    def test_windows_zip_separator_normalization_is_rejected(self):
        entry = zipfile.ZipInfo()
        entry.filename = entry.orig_filename = 'back\\path'
        self.archives([(entry, 'extra')])
        # Exercise the Windows stdlib behavior even on the Linux test runner.
        with patch.object(zipfile.os, 'sep', '\\'):
            with zipfile.ZipFile(self.root / 'SKAGER-Beta2-Portable-Recovery.zip') as archive:
                normalized = archive.infolist()[-1]
                self.assertEqual(normalized.orig_filename, 'back\\path')
                self.assertEqual(normalized.filename, 'back/path')
                with self.assertRaisesRegex(ValueError, 'Normalized archive path'):
                    release.archive_records(archive)

    def test_malicious_archive_names_and_links(self):
        prefix = 'SKAGER-Beta2-Portable-Recovery/'
        for name in ('../outside', '/absolute', 'back\\path', prefix + 'app/opencpn.exe',
                     prefix + 'app/OPENCPN.EXE', prefix + './extra', prefix + 'nul\0hidden'):
            with self.subTest(name=name):
                # ZipInfo(name) normalizes Windows separators and truncates
                # NULs. Assign afterwards to preserve the malicious wire name.
                entry = zipfile.ZipInfo()
                entry.filename = entry.orig_filename = name
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', UserWarning)
                    self.archives([(entry, 'extra')])
                with zipfile.ZipFile(self.root / 'SKAGER-Beta2-Portable-Recovery.zip') as archive:
                    self.assertEqual(archive.infolist()[-1].orig_filename, name)
                with self.assertRaises(ValueError):
                    self.create()
        link = zipfile.ZipInfo(prefix + 'link')
        link.create_system = 3
        link.external_attr = 0o120777 << 16
        self.archives([(link, 'app/opencpn.exe')])
        with self.assertRaises(ValueError):
            self.create()


if __name__ == '__main__':
    unittest.main()
