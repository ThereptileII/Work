#!/usr/bin/env python3
"""Inert archive regressions using the current checked-in license text bytes."""
import copy
import json
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch
import warnings
import zipfile

import packaged_notice_gate as gate

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '1' * 40


def fixture_entries(commit=COMMIT):
    portable, source = {}, {}
    def add(packaged, original, content):
        portable[packaged] = source[original] = content
    add('docs/licenses/SKAGER-COPYING.txt', 'opennav-x/LICENSE', (ROOT / 'LICENSE').read_bytes())
    # Actual root notice names at the pinned OpenCPN revision; no bare COPYING.
    for name in ('COPYING.gplv2', 'COPYING.gplv3', 'COPYING.lgplv2', 'COPYING.lgplv3'):
        add('docs/licenses/OpenCPN/' + name, 'OpenCPN-5.12.4-integrated/' + name,
            ('Inert pinned upstream notice: ' + name + '\n').encode())
    bundles = []
    for library, names in gate.LIBRARIES.items():
        directory = ROOT / 'docs/third-party' / library
        provenance = json.loads((directory / 'provenance.json').read_bytes())
        if library != 'wxWidgets-3.2.8':
            component, version = library.rsplit('-', 1)
            payload = ('Inert dependency source for ' + library).encode()
            path = 'third-party-sources/' + provenance['sourceArchive'].rsplit('/', 1)[1]
            source[path] = payload
            provenance.update(sourceArchiveSha256=gate.digest(payload), sourceArchiveBytes=len(payload))
            bundles.append(dict(path=path, sha256=gate.digest(payload), bytes=len(payload),
                                reference=dict(library=component, version=version, url=provenance['sourceArchive'])))
        for name in (*names, 'provenance.json'):
            data = json.dumps(provenance).encode() if name == 'provenance.json' else (directory / name).read_bytes()
            add('docs/licenses/' + library + '/' + name, 'opennav-x/docs/third-party/' + library + '/' + name, data)
    portable['docs/SOURCE_AND_LICENSES.md'] = (
        'Full project source: https://github.com/ThereptileII/Work/tree/' + commit + '/opennav-x\n'
        'Pinned OpenCPN source: https://github.com/OpenCPN/OpenCPN/tree/' + gate.UPSTREAM + '\n').encode()
    reference = dict(schema=1, productCommit=commit,
        productSource='https://github.com/ThereptileII/Work/tree/' + commit + '/opennav-x',
        openCpnVersion='5.12.4', upstreamCommit=gate.UPSTREAM, integratedTree='OpenCPN-5.12.4-integrated',
        files={name: dict(sha256=gate.digest(data), gitMode='100644') for name, data in source.items()},
        bundledDependencySources=bundles)
    return portable, source, reference


class NoticeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.portable, self.source, self.reference = fixture_entries()
        self.hashes = {name: gate.digest(data) for name, data in self.portable.items()}
        self.portable_extra, self.source_extra = [], []

    def validate(self):
        recovery, source = self.root / 'recovery.zip', self.root / 'source.zip'
        with zipfile.ZipFile(recovery, 'w', zipfile.ZIP_DEFLATED) as archive:
            for name, data in self.portable.items():
                archive.writestr(gate.PREFIX + name, data)
            archive.writestr(gate.PREFIX + 'FILE_SHA256.json', json.dumps(self.hashes))
            for name, data in self.portable_extra:
                archive.writestr(name, data)
        with zipfile.ZipFile(source, 'w', zipfile.ZIP_DEFLATED) as archive:
            for name, data in self.source.items():
                archive.writestr(name, data)
            archive.writestr('SOURCE_REFERENCE.json', json.dumps(self.reference))
            for name, data in self.source_extra:
                archive.writestr(name, data)
        return gate.validate(recovery, source, COMMIT)

    def test_current_notice_texts_and_deterministic_report(self):
        first = self.validate()
        self.assertEqual(first, self.validate())
        self.assertEqual(len(first['notices']), 15)
        self.assertEqual(list(first['notices']), sorted(first['notices']))

    def test_missing_each_packaged_notice(self):
        for name in list(self.portable):
            with self.subTest(name=name):
                data = self.portable.pop(name)
                with self.assertRaisesRegex(ValueError, 'Missing notice/source'):
                    self.validate()
                self.portable[name] = data

    def test_missing_each_pinned_upstream_license_reference(self):
        for name in ('COPYING.gplv2', 'COPYING.gplv3', 'COPYING.lgplv2', 'COPYING.lgplv3'):
            with self.subTest(name=name):
                path = 'OpenCPN-5.12.4-integrated/' + name
                record = self.reference['files'].pop(path)
                with self.assertRaisesRegex(ValueError, 'Missing pinned OpenCPN license reference'):
                    self.validate()
                self.reference['files'][path] = record

    def test_rehashed_tampered_packaged_notice(self):
        name = 'docs/licenses/curl-8.22.0/LICENSE.txt'
        self.portable[name] += b'Tampering even with matching package inventory'
        self.hashes[name] = gate.digest(self.portable[name])
        with self.assertRaisesRegex(ValueError, 'differs from corresponding source'):
            self.validate()

    def test_tampered_packaged_hash(self):
        self.hashes['docs/licenses/SKAGER-COPYING.txt'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'Packaged notice digest differs'):
            self.validate()

    def test_missing_each_source_notice(self):
        for name in list(self.source):
            with self.subTest(name=name):
                data = self.source.pop(name)
                with self.assertRaisesRegex(ValueError, 'Missing notice/source'):
                    self.validate()
                self.source[name] = data

    def test_source_commit_url_and_upstream_identity(self):
        for field in ('schema', 'productCommit', 'productSource', 'upstreamCommit', 'integratedTree'):
            with self.subTest(field=field):
                original = self.reference[field]
                self.reference[field] = True if field == 'schema' else 'other'
                with self.assertRaisesRegex(ValueError, 'identity differs'):
                    self.validate()
                self.reference[field] = original

    def test_nonregular_source_reference(self):
        self.reference['files']['opennav-x/LICENSE']['gitMode'] = '120000'
        with self.assertRaisesRegex(ValueError, 'nonregular notice source'):
            self.validate()

    def test_source_hash_mismatch(self):
        self.reference['files']['opennav-x/LICENSE']['sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'Notice source digest differs'):
            self.validate()

    def test_ambiguous_reference_case(self):
        self.reference['files']['opennav-x/license'] = self.reference['files']['opennav-x/LICENSE']
        with self.assertRaisesRegex(ValueError, 'case-colliding source/hash reference'):
            self.validate()

    def test_unsafe_reference(self):
        self.reference['files']['opennav-x/../LICENSE'] = {}
        with self.assertRaisesRegex(ValueError, 'Unsafe notice/source path'):
            self.validate()

    def test_duplicate_json_keys(self):
        for data in (b'{"files":{},"files":{}}', b'{"files":{"a":1,"a":2}}'):
            with self.assertRaisesRegex(ValueError, 'Duplicate notice/source JSON key'):
                gate.object_json(data)

    def test_duplicate_and_case_archive_entries(self):
        for name in ('opennav-x/LICENSE', 'opennav-x/license'):
            self.source_extra = [(name, b'ambiguous')]
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                with self.assertRaisesRegex(ValueError, 'Duplicate/case-colliding'):
                    self.validate()

    def test_special_notice_file_modes(self):
        data = self.portable.pop('docs/licenses/SKAGER-COPYING.txt')
        for mode in (stat.S_IFLNK, stat.S_IFIFO, stat.S_IFCHR, stat.S_IFSOCK):
            entry = zipfile.ZipInfo(gate.PREFIX + 'docs/licenses/SKAGER-COPYING.txt')
            entry.create_system = 3
            entry.external_attr = (mode | 0o644) << 16
            self.portable_extra = [(entry, data)]
            with self.assertRaisesRegex(ValueError, 'Unsafe or oversized'):
                self.validate()

    def test_notice_and_aggregate_read_bounds(self):
        with patch.object(gate, 'MAX_NOTICE', 10):
            # read defaults bind at definition time; validate source bundle entry
            # sizes and aggregate counters separately below.
            with zipfile.ZipFile(self.root / 'bound.zip', 'w') as archive:
                archive.writestr('notice', b'12345678901')
            with zipfile.ZipFile(self.root / 'bound.zip') as archive:
                with self.assertRaisesRegex(ValueError, 'oversized'):
                    gate.Archive(archive).read('notice', gate.MAX_NOTICE)
        with patch.object(gate, 'MAX_READ', 32):
            with self.assertRaisesRegex(ValueError, 'read budget'):
                self.validate()
        with patch.object(gate, 'MAX_ENTRIES', 1):
            with self.assertRaisesRegex(ValueError, 'entry bound'):
                self.validate()
        with patch.object(gate, 'MAX_ARCHIVE', 1):
            with self.assertRaisesRegex(ValueError, 'bounded notice/source archive'):
                self.validate()

    def test_component_provenance_hash_even_after_rehashing_both_archives(self):
        relative = 'curl-8.22.0/LICENSE.txt'
        target, original = 'docs/licenses/' + relative, 'opennav-x/docs/third-party/' + relative
        self.portable[target] = self.source[original] = b'Altered source and package notice'
        self.hashes[target] = self.reference['files'][original]['sha256'] = gate.digest(self.source[original])
        with self.assertRaisesRegex(ValueError, 'component provenance'):
            self.validate()

    def test_ambiguous_wx_provenance(self):
        relative = 'wxWidgets-3.2.8/provenance.json'
        target, original = 'docs/licenses/' + relative, 'opennav-x/docs/third-party/' + relative
        value = json.loads(self.source[original])
        value['files'][1] = value['files'][0]
        self.portable[target] = self.source[original] = json.dumps(value).encode()
        self.hashes[target] = self.reference['files'][original]['sha256'] = gate.digest(self.source[original])
        with self.assertRaisesRegex(ValueError, 'Ambiguous wxWidgets'):
            self.validate()

    def test_missing_tampered_and_ambiguous_dependency_reference(self):
        bundles = copy.deepcopy(self.reference['bundledDependencySources'])
        for mutation in ('missing', 'hash', 'duplicate', 'component', 'identity'):
            self.reference['bundledDependencySources'] = copy.deepcopy(bundles)
            items = self.reference['bundledDependencySources']
            if mutation == 'missing':
                items.pop(0)
            elif mutation == 'hash':
                items[0]['sha256'] = '0' * 64
            elif mutation == 'duplicate':
                items.append(copy.deepcopy(items[0]))
            elif mutation == 'component':
                items.append(copy.deepcopy(items[0]))
                items[-1]['path'] = 'third-party-sources/other.tar.gz'
            else:
                items[0]['reference']['version'] = 'wrong'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.validate()

    def test_extra_packaged_notice_without_source_reference(self):
        self.portable['docs/licenses/OpenCPN/COPYING.extra'] = b'Unattributed extra notice'
        with self.assertRaisesRegex(ValueError, 'notice inventory differs'):
            self.validate()

    def test_tampered_dependency_payload(self):
        name = self.reference['bundledDependencySources'][0]['path']
        self.source[name] = b'x' * len(self.source[name])
        with self.assertRaisesRegex(ValueError, 'Dependency source bytes differ'):
            self.validate()

    def test_missing_wrong_or_duplicate_notice_source_url(self):
        name = 'docs/SOURCE_AND_LICENSES.md'
        original = self.portable[name]
        for contents in (b'No source', original.replace(COMMIT.encode(), b'2' * 40), original + original):
            self.portable[name] = contents
            self.hashes[name] = gate.digest(contents)
            with self.assertRaisesRegex(ValueError, 'same-commit source URL'):
                self.validate()


if __name__ == '__main__':
    unittest.main()
