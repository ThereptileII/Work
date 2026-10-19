#!/usr/bin/env python3
"""Exercise inventory provenance failures using inert, deterministic package bytes."""
import copy
import base64
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import font_inventory as fonts
import font_payload_test_fixture
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
        self.runtime_sources = []
        self.wx_receipt = None
        self.msvc_receipt = None
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
        # Inert stand-ins exercise byte provenance, not font rendering/licensing.
        freesans = b'SSFN' + b'font fixture'
        helvetica = b'\xfftxf' + b'texture fixture'
        self.add_source(fonts.FREESANS_SOURCE, freesans)
        self.add_source(fonts.HELVETICA_SOURCE, helvetica)
        self.put(fonts.FREESANS_PACKAGE, freesans)
        for name, data in (('FREESANS_SHA256', freesans), ('HELVETICA_SHA256', helvetica)):
            override = patch.object(fonts, name, digest(data))
            override.start()
            self.addCleanup(override.stop)
        self.add_source('OpenCPN-5.12.4-integrated/CMakeLists.txt', b'''
set(ssfnData data/ssfn/FreeSans.sfn)
set(s57data data/s57data/chartsymbols.xml)
install(FILES ${ssfnData} DESTINATION ${PREFIX_PKGDATA}/ssfndata)
install(FILES ${s57data} DESTINATION ${PREFIX_PKGDATA}/s57data)
''')
        font_payload_test_fixture.install(self, lambda name, data: self.add_source('opennav-x/' + name, data), self.put)
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
        return generate_inventory(self.package, self.archive, COMMIT, self.manifests,
                                  verified_runtime_sources=self.runtime_sources, verified_wx=self.wx_receipt,
                                  verified_msvc=self.msvc_receipt)

    def runtime_bundle(self, component, members, reference):
        raw = io.BytesIO()
        with zipfile.ZipFile(raw, 'w') as archive:
            for name, data in sorted(members.items()):
                archive.writestr(name, data)
        data = raw.getvalue()
        updater = component == 'updater'
        packaged = 'app/opennav/third-party/' + ('updater/updater-source.zip' if updater else 'ocharts/corresponding-source.zip')
        source = 'third-party-sources/skager-' + ('updater' if updater else 'ocharts') + '-source.zip'
        self.put(packaged, data)
        self.add_source(source, data)
        self.refs['bundledDependencySources'].append({
            'path': source, 'sha256': digest(data), 'bytes': len(data), 'reference': reference})
        descriptor = {'archive': self.package / packaged, 'path': source,
                      'sha256': digest(data), 'reference': reference}
        self.runtime_sources.append(descriptor)
        return descriptor, packaged

    def updater(self):
        import struct
        from updater_package import GO_VERSION, MAIN_MODULE, verify_updater_package
        checksum = 'h1:' + base64.b64encode(bytes(32)).decode()
        reference = {'schema': 1, 'productCommit': COMMIT, 'goVersion': GO_VERSION,
                     'mainModule': 'tools/update-verifier',
                     'modules': [{'path': 'example.test/module', 'version': 'v1.0.0',
                                  'sum': checksum, 'goModSum': checksum,
                                  'sourcePrefix': 'modules/example.test/module@v1.0.0'}],
                     'build': 'CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build -mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start'}
        members = {'project/LICENSE': b'product terms', GO_VERSION + '/LICENSE': b'Go notice',
                   'modules/example.test/module@v1.0.0/LICENSE': b'module notice',
                   'modules/example.test/module@v1.0.0/source.go': b'package inert'}
        indexed = {name: {'sha256': digest(data), 'bytes': len(data)} for name, data in members.items()}
        members['SOURCE_REFERENCE.json'] = json.dumps({**reference, 'files': indexed}).encode()
        descriptor, packaged = self.runtime_bundle('updater', members, reference)
        pe = bytearray(128)
        pe[:2] = b'MZ'
        struct.pack_into('<I', pe, 60, 64)
        pe[64:68] = b'PE\0\0'
        struct.pack_into('<H', pe, 68, 0x14c)
        struct.pack_into('<H', pe, 88, 0x10b)
        self.put('app/skager-start.exe', pe)
        manifest = {'schema': 1, 'productCommit': COMMIT, 'goVersion': GO_VERSION,
                    'binary': {'path': 'skager-start.exe', 'sha256': digest(pe)},
                    'sourceBundle': {**descriptor, 'archive': packaged.removeprefix('app/')},
                    'buildInfo': f'skager-start.exe: {GO_VERSION}\n\tpath\t{MAIN_MODULE}/cmd/skager-start\n\tbuild\t-trimpath=true\n\tbuild\tCGO_ENABLED=0\n\tbuild\tGOARCH=386\n\tbuild\tGOOS=windows\n\tbuild\tvcs.revision={COMMIT}\n\tbuild\tvcs.modified=false\n'}
        self.put('app/opennav/third-party/updater/build.json', json.dumps(manifest).encode())
        # Exercise the actual existing updater validator before aggregation.
        self.assertEqual(verify_updater_package(self.package / 'app', COMMIT), descriptor)
        self.save_archive()
        return members

    def adapter(self):
        def blob(data):
            return {'bytes': len(data), 'gitBlob': __import__('hashlib').sha1(
                b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()}
        original = b'adapter upstream terms'
        embedded = b'embedded upstream terms'
        lock = {'source': {'repository': 'example.test/adapter', 'commit': 'c' * 40,
                           'files': {'COPYING': blob(original)}},
                'gitlink': {'commit': 'd' * 40, 'files': {'LICENSE': blob(embedded)}}}
        lock_path = 'tools/ocharts-adapter-source.lock.json'
        lock_data = json.dumps(lock).encode()
        self.add_source('opennav-x/' + lock_path, lock_data)
        reference = {'repository': lock['source']['repository'], 'commit': lock['source']['commit'],
                     'gitlinkCommit': lock['gitlink']['commit'], 'manifestSha256': digest(lock_data)}
        descriptor, packaged = self.runtime_bundle('adapter', {
            'product/' + lock_path: lock_data, 'source/COPYING': original,
            'source/opencpn-libs/LICENSE': embedded}, reference)
        runtime = b'inert adapter bytes'
        self.put('app/skager-ocharts-adapter.dll', runtime)
        manifest = {'schema': 1, 'kind': 'skager-ocharts-adapter', 'bindingVersion': 1,
                    'source': reference, 'inputs': {lock_path: {'sha256': digest(lock_data), 'bytes': len(lock_data)}},
                    'chartResourceManifest': {}, 'chartResourceHeader': {},
                    'dll': {'name': 'skager-ocharts-adapter.dll', 'machine': 'I386',
                            'sha256': digest(runtime), 'bytes': len(runtime)},
                    'imports': [], 'exports': [], 'dependencies': {},
                    'correspondingSource': {'path': 'corresponding-source.zip',
                        'sha256': descriptor['sha256'], 'bytes': (self.package / packaged).stat().st_size},
                    'qualification': 'fixture identity only'}
        self.put('app/opennav/third-party/ocharts/manifest.json', json.dumps(manifest).encode())
        self.save_archive()

    def wx_runtime(self):
        archive = {'file': 'wxMSW-3.2.8_vc14x_ReleaseDLL.7z', 'url': 'https://example.invalid/wx',
                   'sha256': 'e' * 64, 'bytes': 100}
        self.add_source('opennav-x/tools/windows-wx.lock.json', json.dumps({
            'version': '3.2.8', 'abi': 'vc14x x86 DLL', 'archives': [archive]}).encode())
        runtime = b'wx runtime fixture'
        self.put('app/wxbase32u_vc14x.dll', runtime)
        self.wx_receipt = {'schema': 1, 'component': 'wxWidgets', 'version': '3.2.8',
                           'abi': 'vc14x x86 DLL', 'archive': archive,
                           'scope': 'prebuilt-runtime-archive-members-only',
                           'files': [{'path': 'wxbase32u_vc14x.dll', 'member': 'lib/vc14x_dll/wxbase32u_vc14x.dll',
                                      'sha256': digest(runtime), 'bytes': len(runtime)}]}
        self.put('app/wx-runtime-receipt.json', json.dumps(self.wx_receipt).encode())
        self.save_archive()

    def msvc_runtime(self):
        files = []
        for name in ('msvcp140.dll', 'vcruntime140.dll'):
            data = ('inert MSVC ' + name).encode()
            self.put('app/' + name, data)
            files.append({'name': name, 'bytes': len(data), 'sha256': digest(data),
                          'fileVersion': '14.44.1.0', 'productVersion': '14.44.1.0'})
        self.msvc_receipt = {'schemaVersion': 2, 'component': 'Microsoft Visual C++ runtime',
                             'architecture': 'x86', 'family': 'Microsoft.VC143.CRT',
                             'visualStudio': {'installationDirectory': r'C:\BuildTools', 'installationVersion': '17.14.1',
                                              'productId': 'Microsoft.VisualStudio.Product.BuildTools',
                                              'catalog': {'productDisplayVersion': '17.14'}},
                             'redistributable': {'directory': r'C:\BuildTools\VC\Redist\MSVC\14.44.1\x86\Microsoft.VC143.CRT',
                                                  'version': '14.44.1'}, 'files': files,
                             'sourceCopyEquality': 'verified',
                             'legalStatus': 'redistribution-terms-unresolved-review-required'}
        self.save_msvc_receipt()

    def save_msvc_receipt(self):
        self.put('docs/MSVC_RUNTIME_RECEIPT.json', json.dumps(self.msvc_receipt).encode())

    def test_font_source_substitution_despite_rehashed_reference(self):
        import freefont_payload
        for relative in freefont_payload.SOURCE_FILES:
            name = 'opennav-x/' + relative
            original = self.entries[name]
            with self.subTest(name=name):
                self.add_source(name, b'substituted source')
                self.save_archive()
                with self.assertRaisesRegex(ValueError, 'Pinned GNU FreeFont'):
                    self.generate()
                self.add_source(name, original)

    def replace_runtime_source(self, members):
        descriptor = self.runtime_sources[0]
        updater = 'updater' in descriptor['path']
        packaged = 'app/opennav/third-party/' + ('updater/updater-source.zip' if updater else 'ocharts/corresponding-source.zip')
        raw = io.BytesIO()
        with zipfile.ZipFile(raw, 'w') as archive:
            for name, data in sorted(members.items()):
                archive.writestr(name, data)
        data = raw.getvalue()
        self.put(packaged, data)
        self.add_source(descriptor['path'], data)
        descriptor['sha256'] = digest(data)
        bundle = self.refs['bundledDependencySources'][-1]
        bundle.update(sha256=digest(data), bytes=len(data))
        manifest_path = 'app/opennav/third-party/' + ('updater/build.json' if updater else 'ocharts/manifest.json')
        manifest = json.loads((self.package / manifest_path).read_bytes())
        if updater:
            manifest['sourceBundle']['sha256'] = digest(data)
        else:
            manifest['correspondingSource'].update(sha256=digest(data), bytes=len(data))
        self.put(manifest_path, json.dumps(manifest).encode())
        self.save_archive()

    def test_updater_verified_runtime_source_and_notices_are_linked(self):
        self.updater()
        inventory = self.generate()
        component = next(item for item in inventory['components'] if item['id'] == 'updater')
        self.assertEqual(component['evidenceStatus'], 'runtime-source-notice-matched')
        self.assertEqual(component['licenseStatus'], 'unresolved-review-required')
        self.assertEqual(len(component['notices']), 3)
        self.assertEqual(inventory['files']['app/skager-start.exe']['componentId'], 'updater')
        self.assertEqual(inventory['status'], 'incomplete-review-required')

    def test_updater_changed_runtime_and_unverified_descriptor_refused(self):
        self.updater()
        self.put('app/skager-start.exe', b'changed runtime')
        with self.assertRaisesRegex(ValueError, 'Updater runtime/source manifest differs'):
            self.generate()
        self.runtime_sources.clear()
        with self.assertRaisesRegex(ValueError, 'requires verified source descriptor'):
            self.generate()

    def test_updater_rehashed_source_must_match_its_inner_index(self):
        members = self.updater()
        members['modules/example.test/module@v1.0.0/source.go'] = b'wrong source'
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'nested source bytes differ'):
            self.generate()

    def test_runtime_nested_source_case_alias_refused(self):
        members = self.updater()
        members['project/license'] = members['project/LICENSE']
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'Ambiguous or oversized runtime source archive'):
            self.generate()

    def test_updater_product_source_relabeling_refused(self):
        members = self.updater()
        members['project/LICENSE'] = b'relabeled product terms'
        index = json.loads(members['SOURCE_REFERENCE.json'])
        index['files']['project/LICENSE'] = {'sha256': digest(members['project/LICENSE']),
                                            'bytes': len(members['project/LICENSE'])}
        members['SOURCE_REFERENCE.json'] = json.dumps(index).encode()
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'Updater corresponding product source differs'):
            self.generate()

    def test_updater_required_original_notice_missing_refused(self):
        members = self.updater()
        index = json.loads(members['SOURCE_REFERENCE.json'])
        del members['project/LICENSE']
        del index['files']['project/LICENSE']
        members['SOURCE_REFERENCE.json'] = json.dumps(index).encode()
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'Runtime source notice missing'):
            self.generate()

    def test_updater_rehashed_source_requires_module_notice_and_exact_reference(self):
        members = self.updater()
        index = json.loads(members['SOURCE_REFERENCE.json'])
        name = 'modules/example.test/module@v1.0.0/LICENSE'
        del members[name]
        del index['files'][name]
        members['SOURCE_REFERENCE.json'] = json.dumps(index).encode()
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'module source notice missing'):
            self.generate()
        index['productCommit'] = 'f' * 40
        members['SOURCE_REFERENCE.json'] = json.dumps(index).encode()
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'nested source inventory differs'):
            self.generate()

    def test_runtime_outer_source_payload_and_ambiguous_descriptors_refused(self):
        self.updater()
        self.entries[self.runtime_sources[0]['path']] = b'wrong retained payload'
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Runtime source payload differs'):
            self.generate()
        self.runtime_sources.append(copy.deepcopy(self.runtime_sources[0]))
        with self.assertRaisesRegex(ValueError, 'duplicate verified runtime source'):
            self.generate()

    def test_adapter_source_blob_notice_and_runtime_are_linked(self):
        self.adapter()
        inventory = self.generate()
        component = next(item for item in inventory['components'] if item['id'] == 'o-charts-adapter')
        self.assertEqual(component['evidenceStatus'], 'runtime-source-notice-matched')
        self.assertEqual({item['sourceMember'] for item in component['notices']},
                         {'source/COPYING', 'source/opencpn-libs/LICENSE'})
        self.put('app/skager-ocharts-adapter.dll', b'wrong adapter')
        with self.assertRaisesRegex(ValueError, 'Adapter runtime/source manifest differs'):
            self.generate()

    def test_adapter_rehashed_source_must_match_committed_git_blobs(self):
        self.adapter()
        descriptor = self.runtime_sources[0]
        with zipfile.ZipFile(descriptor['archive']) as archive:
            members = {name: archive.read(name) for name in archive.namelist()}
        members['source/COPYING'] = b'relabeled terms'
        self.replace_runtime_source(members)
        with self.assertRaisesRegex(ValueError, 'corresponding upstream source differs'):
            self.generate()

    def test_adapter_source_lock_substitution_refused(self):
        self.adapter()
        source_path = 'opennav-x/tools/ocharts-adapter-source.lock.json'
        lock = json.loads(self.entries[source_path])
        lock['source']['commit'] = 'f' * 40
        self.add_source(source_path, json.dumps(lock).encode())
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'identity differs from committed lock'):
            self.generate()

    def test_wx_receipt_links_archive_members_without_source_rebuild_claim(self):
        self.wx_runtime()
        inventory = self.generate()
        component = next(item for item in inventory['components'] if item['id'] == 'wxWidgets')
        self.assertEqual(component['evidenceStatus'], 'runtime-archive-notice-matched')
        self.assertEqual(component['licenseStatus'], 'unresolved-review-required')
        self.assertEqual(inventory['files']['app/wxbase32u_vc14x.dll']['componentId'], 'wxWidgets')
        self.assertIn('does not establish source rebuild', component['limitations'][0])

    def test_wx_changed_runtime_receipt_and_locked_archive_refused(self):
        self.wx_runtime()
        path = 'app/wxbase32u_vc14x.dll'
        original = (self.package / path).read_bytes()
        self.put(path, b'wrong wx runtime')
        with self.assertRaisesRegex(ValueError, 'packaged runtime differs'):
            self.generate()
        self.put(path, original)
        self.put('app/wx-runtime-receipt.json', b'{"schema":1,"schema":1}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            self.generate()
        self.wx_receipt['archive']['sha256'] = '0' * 64
        self.put('app/wx-runtime-receipt.json', json.dumps(self.wx_receipt).encode())
        with self.assertRaisesRegex(ValueError, 'archive identity differs'):
            self.generate()

    def test_wx_unverified_or_extra_runtime_refused(self):
        self.wx_runtime()
        self.wx_receipt = None
        with self.assertRaisesRegex(ValueError, 'requires verified receipt'):
            self.generate()
        self.wx_runtime()
        self.put('app/wxdebug_vc14x.dll', b'unknown wx DLL')
        with self.assertRaisesRegex(ValueError, 'runtime inventory differs'):
            self.generate()

    def test_msvc_copy_identity_keeps_terms_and_entitlement_unresolved(self):
        self.msvc_runtime()
        inventory = self.generate()
        component = next(item for item in inventory['components'] if item['id'] == 'MSVC-runtime')
        self.assertEqual(component['evidenceStatus'], 'runtime-copy-identity-matched')
        self.assertEqual(component['licenseStatus'], 'redistribution-terms-and-entitlement-unresolved-review-required')
        self.assertEqual(component['notices'], [])
        self.assertNotIn('source', component)
        self.assertEqual(inventory['files']['app/msvcp140.dll']['componentId'], 'MSVC-runtime')
        self.assertEqual(inventory['status'], 'incomplete-review-required')

    def test_combined_runtime_inventory_is_deterministic_and_keeps_review_status(self):
        self.updater()
        self.adapter()
        self.wx_runtime()
        self.msvc_runtime()
        options = {'verified_runtime_sources': self.runtime_sources,
                   'verified_wx': self.wx_receipt, 'verified_msvc': self.msvc_receipt}
        write_inventory(self.package, self.archive, COMMIT, self.manifests, **options)
        original = [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)]
        write_inventory(self.package, self.archive, COMMIT, self.manifests, **options)
        self.assertEqual(original, [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)])
        self.assertIn('redistribution-terms-and-entitlement-unresolved', original[1].decode())
        self.assertIn('../app/opennav/third-party/updater/updater-source.zip', original[1].decode())

    def test_msvc_invalid_identity_catalog_and_legal_upgrade_refused(self):
        self.msvc_runtime()
        original = copy.deepcopy(self.msvc_receipt)
        for mutate in (lambda r: r['visualStudio'].update(productId='OtherVendor.Product'),
                       lambda r: r['visualStudio']['catalog'].update(entitlement='licensed'),
                       lambda r: r['redistributable'].update(directory=r'C:\Other\CRT'),
                       lambda r: r.update(schemaVersion=1),
                       lambda r: r.update(legalStatus='accepted')):
            self.msvc_receipt = copy.deepcopy(original)
            mutate(self.msvc_receipt)
            self.save_msvc_receipt()
            with self.assertRaises(ValueError):
                self.generate()

    def test_msvc_copy_runtime_and_receipt_changes_refused(self):
        self.msvc_runtime()
        self.put('app/vcruntime140.dll', b'changed runtime')
        with self.assertRaisesRegex(ValueError, 'packaged runtime differs'):
            self.generate()
        self.put('docs/MSVC_RUNTIME_RECEIPT.json', b'{}')
        with self.assertRaisesRegex(ValueError, 'copy receipt differs'):
            self.generate()

    def test_msvc_missing_unverified_duplicate_and_boolean_file_records_refused(self):
        self.msvc_runtime()
        original = copy.deepcopy(self.msvc_receipt)
        self.msvc_receipt = None
        with self.assertRaisesRegex(ValueError, 'requires verified copy receipt'):
            self.generate()
        for mutate in (lambda r: r['files'].pop(),
                       lambda r: r['files'].append(r['files'][0].copy()),
                       lambda r: r['files'][0].update(bytes=True)):
            self.msvc_receipt = copy.deepcopy(original)
            mutate(self.msvc_receipt)
            self.save_msvc_receipt()
            with self.assertRaises(ValueError):
                self.generate()

    def test_deterministic_and_unresolved_coverage(self):
        inventory = write_inventory(self.package, self.archive, COMMIT, self.manifests)
        original = [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)]
        write_inventory(self.package, self.archive, COMMIT, self.manifests)
        self.assertEqual(original, [(self.package / name).read_bytes() for name in (INVENTORY, NOTICES)])
        self.assertEqual(inventory['status'], 'incomplete-review-required')
        unresolved = {item['path']: item['category'] for item in inventory['unresolvedFiles']}
        self.assertEqual(unresolved, {'app/skager.ttf': 'font', 'app/icon.png': 'asset',
                                     'app/msvcp140.dll': 'runtime', 'app/opencpn.exe': 'runtime',
                                     fonts.FREESANS_PACKAGE: 'font'})
        self.assertEqual(len(inventory['components']), 7)
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
        runtime = self.package / 'app/libcurl.dll'
        original = runtime.read_bytes()

        # On case-insensitive filesystems, writing the alias would replace
        # the valid runtime and test hash rejection instead.
        # Inject only the extra directory entry; its metadata and bytes still
        # come from the real, unchanged file, and inventory checks stay real.
        class CaseAliasPath(type(runtime)):
            def is_symlink(self):
                return runtime.is_symlink()

            def is_file(self):
                return runtime.is_file()

            def open(self, *args, **kwargs):
                return runtime.open(*args, **kwargs)

        entries = list(self.package.rglob('*'))
        entries.append(CaseAliasPath(self.package / 'app/LIBCURL.dll'))
        with patch.object(Path, 'rglob', return_value=iter(entries)):
            with self.assertRaisesRegex(ValueError, 'Ambiguous Windows package path'):
                self.generate()
        self.assertEqual(runtime.read_bytes(), original)

    def test_symlink_rejected(self):
        (self.package / 'app/linked.ttf').symlink_to('skager.ttf')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.generate()

    def test_freesans_source_mapping_keeps_license_unresolved(self):
        inventory = self.generate()
        component = next(c for c in inventory['components'] if c['id'] == 'FreeSans-SSFN')
        self.assertEqual(component['licenseStatus'], 'unresolved')
        self.assertNotIn('declaredLicense', component)
        self.assertEqual(component['source']['path'], fonts.FREESANS_SOURCE)
        self.assertEqual(inventory['fontEvidence']['reviewedInstallLists']['sourceOnlyUnderReviewedList'],
                         'data/s57data/Helvetica.txf')
        self.assertNotIn('app/s57data/Helvetica.txf', inventory['files'])

    def test_freesans_missing_or_modified_rejected(self):
        self.put(fonts.FREESANS_PACKAGE, b'SSFNwrong font')
        with self.assertRaisesRegex(ValueError, 'Packaged FreeSans font differs'):
            self.generate()
        (self.package / fonts.FREESANS_PACKAGE).unlink()
        with self.assertRaisesRegex(ValueError, 'evidence missing'):
            self.generate()

    def test_font_source_relabeling_rejected(self):
        self.add_source(fonts.FREESANS_SOURCE, b'SSFNchanged source')
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Pinned font source differs'):
            self.generate()

    def test_texture_font_install_list_expansion_requires_review(self):
        path = 'OpenCPN-5.12.4-integrated/CMakeLists.txt'
        self.add_source(path, self.entries[path].replace(b'data/s57data/chartsymbols.xml',
                                                       b'data/s57data/Helvetica.txf'))
        self.save_archive()
        with self.assertRaisesRegex(ValueError, 'Texture font has entered'):
            self.generate()

    def test_legacy_and_renamed_fonts_remain_explicit_candidates(self):
        self.put('app/s57data/Helvetica.txf', b'\xfftxf' + b'fixture')
        self.put('app/font-data.bin', b'SSFN' + b'fixture')
        self.put('app/legacy.fnt', b'not a validated font')
        self.put('app/plain.txt', b'this text mentions a font')
        inventory = self.generate()
        candidates = {c['path']: c for c in inventory['fontEvidence']['packageCandidates']}
        self.assertEqual(candidates['app/font-data.bin']['format'], 'SSFN')
        self.assertEqual(candidates['app/s57data/Helvetica.txf']['format'], 'TXF')
        self.assertEqual(candidates['app/legacy.fnt']['basis'], 'extension-only')
        self.assertNotIn('app/plain.txt', candidates)
        unresolved = {i['path']: i['category'] for i in inventory['unresolvedFiles']}
        for name in ('app/font-data.bin', 'app/s57data/Helvetica.txf', 'app/legacy.fnt'):
            self.assertEqual(unresolved[name], 'font')

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
