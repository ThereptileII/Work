#!/usr/bin/env python3
"""Exercise inventory provenance failures using inert, deterministic package bytes."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from component_license_inventory import (DEPENDENCIES, INVENTORY, NOTICES, digest,
                                         generate_inventory, write_inventory)
from source_package import PINNED_UPSTREAM

COMMIT = 'a' * 40


class InventoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.package = self.root / 'package'
        self.archive = self.root / 'source.zip'
        self.entries = {}
        self.manifests = {}
        self.refs = {'productCommit': COMMIT, 'upstreamCommit': PINNED_UPSTREAM,
                     'files': {}, 'bundledDependencySources': []}
        self.notice('SKAGER-COPYING.txt', 'opennav-x/LICENSE', b'product terms')
        self.notice('OpenCPN/COPYING', 'OpenCPN-5.12.4-integrated/COPYING', b'upstream terms')
        self.notice('OpenCPN/libs/foo/LICENSE', 'OpenCPN-5.12.4-integrated/libs/foo/LICENSE', b'embedded terms')
        for library, (version, dlls) in DEPENDENCIES.items():
            prefix = library + '-' + version + '/'
            source_prefix = 'opennav-x/docs/third-party/' + prefix
            source = ('source ' + library).encode()
            source_path = 'third-party-sources/' + library + '.tar.gz'
            source_identity = {'sha256': digest(source), 'bytes': len(source),
                               'url': 'https://example.invalid/' + library}
            self.add_source(source_path, source)
            self.refs['bundledDependencySources'].append({
                'path': source_path, **source_identity,
                'reference': {'library': library, 'version': version, 'url': source_identity['url']}})
            license_text = (library + ' notice').encode()
            provenance = {'library': library + ' ' + version, 'license': 'fixture-terms',
                          'licenseSha256': digest(license_text),
                          'sourceArchiveSha256': digest(source), 'sourceArchiveBytes': len(source),
                          'sourceArchive': source_identity['url']}
            self.notice(prefix + 'provenance.json', source_prefix + 'provenance.json',
                        json.dumps(provenance).encode())
            self.notice(prefix + 'LICENSE.txt', source_prefix + 'LICENSE.txt', license_text)
            manifest = {'library': library, 'version': version, 'source': source_identity, 'outputs': {}}
            for dll in dlls:
                content = ('inert DLL ' + dll).encode()
                self.put('app/' + dll, content)
                manifest['outputs']['bin/' + dll] = {'sha256': digest(content), 'bytes': len(content)}
            self.manifests[library] = manifest
            self.put('app/' + library.lower() + '-build.json', json.dumps(manifest).encode())
        wx = {'version': '3.2.8', 'commit': 'b' * 40, 'files': []}
        for name in ('licence.txt', 'lgpl.txt', 'gpl.txt'):
            data = ('wx terms ' + name).encode()
            wx['files'].append({'file': name, 'sha256': digest(data)})
            self.notice('wxWidgets-3.2.8/' + name, 'opennav-x/docs/third-party/wxWidgets-3.2.8/' + name, data)
        self.notice('wxWidgets-3.2.8/provenance.json',
                    'opennav-x/docs/third-party/wxWidgets-3.2.8/provenance.json', json.dumps(wx).encode())
        for name in ('app/skager.ttf', 'app/icon.png', 'app/msvcp140.dll', 'app/opencpn.exe'):
            self.put(name, b'unattributed fixture')
        self.save_archive()

    def put(self, name, data):
        path = self.package / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)

    def add_source(self, name, data):
        self.entries[name] = data
        self.refs['files'][name] = {'sha256': digest(data), 'gitMode': '100644'}

    def notice(self, name, source, data):
        self.put('docs/licenses/' + name, data)
        self.add_source(source, data)

    def save_archive(self):
        with zipfile.ZipFile(self.archive, 'w') as archive:
            for name, data in sorted(self.entries.items()):
                archive.writestr(name, data)
            archive.writestr('SOURCE_REFERENCE.json', json.dumps(self.refs))

    def generate(self):
        return generate_inventory(self.package, self.archive, COMMIT, self.manifests)

    def test_deterministic_and_unresolved_coverage(self):
        inventory = write_inventory(self.package, self.archive, COMMIT, self.manifests)
        original = [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)]
        write_inventory(self.package, self.archive, COMMIT, self.manifests)
        self.assertEqual(original, [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)])
        self.assertEqual(inventory['status'], 'incomplete-review-required')
        unresolved = {item['path']: item['category'] for item in inventory['unresolvedFiles']}
        self.assertEqual(unresolved, {'app/skager.ttf': 'font', 'app/icon.png': 'asset',
                                     'app/msvcp140.dll': 'runtime', 'app/opencpn.exe': 'runtime'})
        self.assertEqual(len(inventory['components']), 6)
        self.assertEqual(inventory['files']['app/libcurl.dll']['componentId'], 'curl')
        self.assertIn('COMPONENT_LICENSE_INVENTORY.json', original[1].decode())
        self.assertNotIn(INVENTORY, inventory['files'])

    def test_missing_and_modified_packaged_notices_rejected(self):
        path = self.package / 'docs/licenses/curl-8.22.0/LICENSE.txt'
        path.write_bytes(b'replaced terms')
        with self.assertRaisesRegex(ValueError, 'notice differs'):
            self.generate()
        path.unlink()
        with self.assertRaisesRegex(ValueError, 'evidence missing'):
            self.generate()

    def test_missing_embedded_opencpn_notice_rejected(self):
        (self.package / 'docs/licenses/OpenCPN/libs/foo/LICENSE').unlink()
        with self.assertRaisesRegex(ValueError, 'notice inventory differs'):
            self.generate()

    def test_runtime_substitution_rejected(self):
        self.put('app/libcurl.dll', b'substituted runtime')
        with self.assertRaisesRegex(ValueError, 'runtime differs'):
            self.generate()

    def test_manifest_substitution_rejected(self):
        self.put('app/curl-build.json', b'{}')
        with self.assertRaisesRegex(ValueError, 'manifest differs'):
            self.generate()

    def test_duplicate_json_key_rejected(self):
        self.put('app/curl-build.json', b'{"library":"curl","library":"curl"}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            self.generate()

    def test_source_revision_mismatch_rejected(self):
        self.refs['productCommit'] = 'f' * 40
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'identity differs'):
            self.generate()

    def test_source_content_tampering_rejected(self):
        self.entries['third-party-sources/curl.tar.gz'] = b'tampered archive'
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'digest differs'):
            self.generate()

    def test_source_hash_relabeling_rejected(self):
        self.add_source('third-party-sources/curl.tar.gz', b'x' * len(b'source curl'))
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Bundled dependency source differs'):
            self.generate()

    def test_missing_and_ambiguous_source_records_rejected(self):
        original = copy.deepcopy(self.refs['bundledDependencySources'])
        self.refs['bundledDependencySources'] = original[1:]
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Missing or ambiguous'):
            self.generate()
        self.refs['bundledDependencySources'] = original + [original[0]]
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Missing or ambiguous'):
            self.generate()

    def test_wx_provenance_hash_rejected(self):
        name = 'opennav-x/docs/third-party/wxWidgets-3.2.8/provenance.json'
        wx = json.loads(self.entries[name])
        wx['files'][0]['sha256'] = '0' * 64
        self.notice('wxWidgets-3.2.8/provenance.json', name, json.dumps(wx).encode())
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'wxWidgets notice provenance differs'):
            self.generate()

    def test_windows_case_alias_rejected(self):
        self.put('app/LIBCURL.dll', b'alias')
        with self.assertRaisesRegex(ValueError, 'Ambiguous Windows package path'):
            self.generate()

    def test_symlink_rejected(self):
        (self.package / 'app/linked.ttf').symlink_to('skager.ttf')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.generate()

    def test_duplicate_source_member_rejected(self):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            with zipfile.ZipFile(self.archive, 'a') as archive:
                archive.writestr('SOURCE_REFERENCE.json', json.dumps(self.refs))
        with self.assertRaisesRegex(ValueError, 'duplicate corresponding-source member'):
            self.generate()


if __name__ == '__main__':
    unittest.main()
