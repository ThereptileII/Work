#!/usr/bin/env python3
"""Inert subject-role contracts; never native Windows/runtime acceptance."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import plugin_subject_sdk as api

ROOT = Path(__file__).resolve().parents[1]


class SubjectSdk(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'root'; self.root.mkdir()
        self.subject = Path(self.temp.name) / 'subject'; self.subject.mkdir()
        self.payload = self.subject / 'payload'; self.payload.mkdir()
        self.provenance = Path(self.temp.name) / 'provenance.json'
        self.lock = json.loads((ROOT / 'tools/windows-dependency-bundle.lock.json').read_text())
        self.host_entry, self.subject_entry = self.lock['images']
        self.write(self.root / 'tools/windows-dependency-bundle.lock.json', self.lock)
        for name in api.bundle_api.INPUTS:
            path = self.payload / name; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((ROOT / name).read_bytes())
        path = self.payload / api.bundle_api.WORKFLOW; path.parent.mkdir(parents=True)
        path.write_bytes((ROOT.parent / api.bundle_api.WORKFLOW).read_bytes())
        for kind, prefix in api.stage.PREFIX.items():
            for name in ('include/fixture.h', 'lib/fixture.lib', 'bin/fixture.dll'):
                path = self.payload / prefix / name; path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes((kind + name).encode())
        for kind, name in api.reuse.PRODUCER_FACTS.items():
            self.write(self.payload / name, {'schemaVersion': 1, 'kind': kind})
        self.roots = [*api.bundle_api.INPUTS, api.bundle_api.WORKFLOW,
                      *api.stage.PREFIX.values(), *api.reuse.PRODUCER_FACTS.values()]
        selection = self.subject_entry['selection']
        producer = {key: selection[key] for key in ('repository', 'runId', 'runAttempt', 'headSha')}
        producer.update(job=api.bundle_api.JOB, workflowPath=api.bundle_api.WORKFLOW)
        self.doc = dict(schemaVersion=1, producer=producer, workspaceRoot='D:\\a\\Work\\Work\\opennav-x',
            fingerprint=api.bundle_api.fingerprint(self.payload), roots=self.roots,
            files=api.receipt._inventory(self.payload, self.roots), runner=self.subject_entry['runner'],
            python={'executable': 'C:\\Python\\python.exe', 'sha256': 'e' * 64})
        for entry in self.lock['images']:
            selected = entry['selection']
            selected['artifactName'] = f"windows-dependencies-{self.doc['fingerprint']['sha256']}-run{selected['runId']}-attempt{selected['runAttempt']}"
        self.write(self.root / 'tools/windows-dependency-bundle.lock.json', self.lock)
        self.doc['toolchainSha256'] = api.bundle_api._toolchain_digest(self.doc)
        self.captured = api.dependency_authority(self.doc)
        self.host = copy.deepcopy(self.doc)
        self.host['runner'] = self.host_entry['runner']
        # Distinct host runtime bytes are allowed, but never bind the subject.
        self.host['files'][api.stage.PREFIX['curl'] + '/bin/fixture.dll']['sha256'] = 'f' * 64
        self.seal()
        self.native = patch.object(api.bundle_api, 'verify_restored', return_value=self.host).start()
        self.addCleanup(patch.stopall)
        self.producer_checks = patch.object(api, 'verify_subject_producers', return_value={}).start()
        patch.object(api.bundle_api, '_roots', return_value=self.roots).start()
        patch.dict(os.environ, self.host_entry['runner']).start()

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def seal(self):
        self.write(self.subject / 'bundle.json', self.doc)
        authority = dict(self.subject_entry['selection'], schemaVersion=1, workflowPath=api.bundle_api.WORKFLOW,
                         job=api.bundle_api.JOB, conclusion='success',
                         bundleSha256=api.receipt._digest(self.subject / 'bundle.json'))
        self.write(self.provenance, authority)

    def verify(self):
        return api.verify_subject(self.root, self.subject, self.provenance,
                                  Path('host-bundle'), Path('host-provenance'), self.captured)

    def test_two_roles_keep_original_subject_and_current_host(self):
        self.assertEqual(self.verify(), self.doc)
        self.native.assert_called_once_with(self.root, Path('host-bundle'), Path('host-provenance'))
        self.producer_checks.assert_called_once_with(self.payload, self.doc)
        self.assertNotEqual(self.host['runner'], self.doc['runner'])
        self.assertNotEqual(self.host['files'], self.doc['files'])

    def test_current_host_and_original_producer_checks_mandatory(self):
        for boundary in (self.native, self.producer_checks):
            with self.subTest(boundary=boundary):
                boundary.side_effect = ValueError('mandatory native or producer check')
                with self.assertRaisesRegex(ValueError, 'mandatory'): self.verify()
                boundary.side_effect = None

    def test_capture_cannot_select_unreviewed_subject(self):
        self.captured['producer']['runId'] = '1'
        with self.assertRaisesRegex(ValueError, 'unique reviewed'): self.verify()

    def test_unknown_current_host_refused(self):
        with patch.dict(os.environ, {'ImageVersion': '999.1'}):
            with self.assertRaisesRegex(ValueError, 'exact current Windows runner'): self.verify()

    def test_original_manifest_digest_required(self):
        with (self.subject / 'bundle.json').open('a') as stream: stream.write(' ')
        with self.assertRaisesRegex(ValueError, 'manifest differs'): self.verify()

    def test_external_envelope_cannot_change_pin_or_success(self):
        original = json.loads(self.provenance.read_text())
        for key, value in [('artifactId', '1'), ('artifactDigest', 'sha256:' + 'f' * 64),
                           ('conclusion', 'failure'), ('runAttempt', '2'), ('headSha', 'f' * 40)]:
            with self.subTest(key=key):
                self.write(self.provenance, dict(original, **{key: value}))
                with self.assertRaisesRegex(ValueError, 'envelope'): self.verify()
        self.write(self.provenance, original)
        inside = self.subject / 'authority.json'; inside.write_bytes(self.provenance.read_bytes())
        self.provenance = inside
        with self.assertRaisesRegex(ValueError, 'outside'): self.verify()

    def test_original_runner_capture_and_toolchain_digest_mandatory(self):
        original = copy.deepcopy(self.doc)
        for key, value, error in [('runner', self.host_entry['runner'], 'runner differs'),
                                  ('toolchainSha256', 'f' * 64, 'capture authority')]:
            with self.subTest(key=key):
                self.doc = copy.deepcopy(original); self.doc[key] = value; self.seal()
                with self.assertRaisesRegex(ValueError, error): self.verify()
        self.captured = api.dependency_authority(self.doc)
        with self.assertRaisesRegex(ValueError, 'toolchain digest'): self.verify()

    def test_payload_extra_changed_missing_and_symlink_refused(self):
        path = self.payload / api.stage.PREFIX['curl'] / 'include/fixture.h'
        original = path.read_bytes()
        for mode in ('changed', 'missing', 'extra', 'symlink'):
            with self.subTest(mode=mode):
                if mode == 'changed': path.write_bytes(b'changed')
                elif mode == 'missing': path.unlink()
                elif mode == 'extra': (self.payload / 'undeclared').write_text('extra')
                else: path.unlink(); path.symlink_to(self.provenance)
                with self.assertRaises(ValueError): self.verify()
                if path.is_symlink(): path.unlink()
                path.write_bytes(original)
                (self.payload / 'undeclared').unlink(missing_ok=True)

    def test_equal_source_python_abi_fact_and_interface_contract(self):
        original = copy.deepcopy(self.host)
        cases = [('workspaceRoot', 'elsewhere'), ('python', {'executable': 'changed'}),
                 ('fingerprint', dict(self.host['fingerprint'], sha256='f' * 64))]
        for key, value in cases:
            with self.subTest(key=key):
                self.host.clear(); self.host.update(copy.deepcopy(original)); self.host[key] = value
                with self.assertRaises(ValueError): self.verify()
        paths = [*api.reuse.PRODUCER_FACTS.values(),
                 *api.interface_files(self.host)]
        for name in paths:
            with self.subTest(name=name):
                self.host.clear(); self.host.update(copy.deepcopy(original))
                self.host['files'][name]['sha256'] = '0' * 64
                with self.assertRaises(ValueError): self.verify()
        self.host.clear(); self.host.update(copy.deepcopy(original))
        self.host['files'][api.stage.PREFIX['curl'] + '/include/extra.h'] = {'bytes': 1, 'sha256': '0' * 64}
        with self.assertRaisesRegex(ValueError, 'headers or import libraries'): self.verify()

    def test_actual_selected_tool_path_bytes_and_count_bound(self):
        paths = []
        for name in ('cl.exe', 'dumpbin.exe', 'cmake.exe', 'python.exe', 'pwsh.exe'):
            path = Path(self.temp.name) / name; path.write_bytes(name.encode()); paths.append(path)
        records = [{'path': str(path), 'bytes': path.stat().st_size, 'sha256': api.receipt._digest(path)} for path in paths]
        for name in api.reuse.PRODUCER_FACTS.values(): self.write(self.payload / name, {'tools': records})
        self.doc['python'] = {'executable': str(paths[3]), 'sha256': api.receipt._digest(paths[3])}
        api.verify_probe_tools(paths, self.doc, self.payload)
        for path in paths:
            original = path.read_bytes(); path.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'differs'): api.verify_probe_tools(paths, self.doc, self.payload)
            path.write_bytes(original)
        with self.assertRaisesRegex(ValueError, 'Expected compiler'): api.verify_probe_tools(paths[:-1], self.doc, self.payload)
        renamed = Path(self.temp.name) / 'renamed.exe'; renamed.write_bytes(paths[0].read_bytes())
        with self.assertRaisesRegex(ValueError, 'differs'): api.verify_probe_tools([renamed, *paths[1:]], self.doc, self.payload)



class ProducerValidationView(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.payload = Path(self.temp.name) / 'payload'; self.payload.mkdir()
        self.document = {'workspaceRoot': 'D:/a/Work/Work/opennav-x'}
        self.manifests = {}
        for kind, prefix in api.stage.PREFIX.items():
            path = self.payload / prefix / 'bin' / (kind + '.dll')
            path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(kind.encode())
            self.manifests[kind] = {'outputs': {'bin/' + path.name:
                {'bytes': path.stat().st_size, 'sha256': api.receipt._digest(path)}}}
        curl = self.manifests['curl']
        curl.update(dependencies={kind: {'prefix': self.document['workspaceRoot'] + '/' + api.stage.PREFIX[kind],
                                        'manifestSha256': 'unchanged original dependency hash'}
                                  for kind in ('openssl', 'zlib')},
                    buildSteps={'certificateTool': {'path': self.document['workspaceRoot'] + '/' +
                        api.stage.PREFIX['openssl'] + '/bin/openssl.exe', 'sha256': 'unchanged original executable hash'}},
                    importOutput='libssl-3.dll\nlibcrypto-3.dll\nzlib1.dll\n')
        for kind, name in api.reuse.PRODUCER_FACTS.items():
            path = self.payload / name; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({'schemaVersion': 1, 'kind': kind}))
        self.write()
        self.opened = patch.object(api.bundle_api.openssl_package, 'verify_openssl_package_inputs').start()
        self.curled = patch.object(api.bundle_api.curl_package, 'verify_curl_package_inputs').start()
        self.manifest_check = patch.object(api.bundle_api.curl_package, 'verify_manifest').start()
        self.upstream = patch.object(api.evidence, '_require_upstream_tests').start()
        self.addCleanup(patch.stopall)

    def write(self):
        for kind, prefix in api.stage.PREFIX.items():
            (self.payload / prefix / (kind + '-build.json')).write_text(json.dumps(self.manifests[kind]))

    def verify(self):
        api.verify_subject_producers(self.payload, self.document)

    def test_only_three_validated_paths_move_in_temporary_view(self):
        original = copy.deepcopy(self.manifests['curl'])
        def inspect(flat, source, notices):
            derived = json.loads((flat / 'curl-build.json').read_text())
            for kind in ('openssl', 'zlib'):
                self.assertEqual(derived['dependencies'][kind]['prefix'], str(self.payload / api.stage.PREFIX[kind]))
                derived['dependencies'][kind]['prefix'] = original['dependencies'][kind]['prefix']
            self.assertEqual(derived['buildSteps']['certificateTool']['path'],
                             str(self.payload / api.stage.PREFIX['openssl'] / 'bin/openssl.exe'))
            derived['buildSteps']['certificateTool']['path'] = original['buildSteps']['certificateTool']['path']
            self.assertEqual(derived, original)
            self.flat = flat
        self.curled.side_effect = inspect
        self.verify()
        self.assertFalse(self.flat.exists())
        self.opened.assert_called_once(); self.manifest_check.assert_called_once(); self.upstream.assert_called_once()
        self.assertEqual(self.manifest_check.call_args.kwargs['dependency_prefixes'],
                         {kind: self.payload / api.stage.PREFIX[kind] for kind in ('openssl', 'zlib')})
        self.assertEqual(json.loads((self.payload / api.stage.PREFIX['curl'] / 'curl-build.json').read_text()), original)

    def test_original_paths_cannot_redirect_subject_validation(self):
        original = copy.deepcopy(self.manifests['curl'])
        for key in ('openssl', 'zlib', 'certificateTool'):
            with self.subTest(key=key):
                self.manifests['curl'] = copy.deepcopy(original)
                if key == 'certificateTool': self.manifests['curl']['buildSteps'][key]['path'] = 'C:/foreign/openssl.exe'
                else: self.manifests['curl']['dependencies'][key]['prefix'] = 'C:/foreign'
                self.write()
                with self.assertRaisesRegex(ValueError, 'Subject original'): self.verify()
        self.curled.assert_not_called()

    def test_original_output_import_and_upstream_checks_remain(self):
        target = self.payload / api.stage.PREFIX['curl'] / 'bin/curl.dll'
        target.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'Missing or changed'): self.verify()
        target.write_bytes(b'curl')
        self.manifests['curl']['importOutput'] += 'libeay32.dll\n'; self.write()
        with self.assertRaisesRegex(ValueError, 'closure incomplete or legacy'): self.verify()
        self.manifests['curl']['importOutput'] = 'libssl-3.dll\nlibcrypto-3.dll\nzlib1.dll\n'; self.write()
        self.upstream.side_effect = ValueError('upstream must pass')
        with self.assertRaisesRegex(ValueError, 'upstream must pass'): self.verify()


if __name__ == '__main__':
    unittest.main()
