#!/usr/bin/env python3
"""Inert artifact-boundary tests; no application, compiler or Windows execution."""
import hashlib
import io
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile
import staging_build_inputs as inputs

COMMIT = 'a' * 40
HARNESS = 'b' * 40
PRODUCER = inputs.producer(COMMIT, '100', '2')


def zipped(entries):
    output = io.BytesIO()
    with zipfile.ZipFile(output, 'w') as archive:
        for name, value in entries.items():
            entry = zipfile.ZipInfo()
            entry.filename = entry.orig_filename = name
            archive.writestr(entry, value)
    return output.getvalue()


class Boundary(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.producer = self.base / 'producer'
        self.consumer = self.base / 'consumer'
        self.producer.mkdir(); self.consumer.mkdir()
        self.output = self.base / 'sealed'
        self.receipt = self.consumer / 'evidence/local/staging-build-restore.json'
        files = {name: b'inert compiled fixture\n' for name in inputs.REQUIRED}
        for variant in inputs.VARIANTS:
            files[f'build/{variant}-windows/include/OpenNavBuild.h'] = (
                '#define OPENNAV_BUILD_COMMIT "' + COMMIT + '"\n').encode()
            files[f'evidence/local/windows-{variant}-tests.xml'] = (
                '<testsuite tests="1" failures="0" errors="0"><testcase name="native-contract"/></testsuite>').encode()
            for name in ('tests.exe', 'buffer_tests.exe'):
                files[f'build/{variant}-windows/test/Release/{name}'] = b'inert test PE'
        files[inputs.FEEDBACK_MANIFEST] = json.dumps(dict(schema=1, commit=COMMIT, tests=[
            dict(name=name, path='D:/a/Work/Work/opennav-x/' + path)
            for name, path in sorted(inputs.FEEDBACK_BINARIES.items())])).encode()
        files['build/production-install/opencpn.exe'] = b'inert product PE'
        files[inputs.PACKAGE_ROOT + '/app/opencpn.exe'] = b'inert product PE'
        files['build/xnav-install/opencpn-cmd.exe'] = b'inert peer CLI'
        product = dict(commit=COMMIT, test_fixtures=False, build_purpose='INSTALLED PRODUCT',
                       xnav_hardware_output_policy='status-only',
                       executable_sha256=hashlib.sha256(b'inert product PE').hexdigest())
        files[inputs.PACKAGE_ROOT + '/docs/PRODUCT_BUILD.json'] = json.dumps(product).encode()
        files['build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip'] = zipped({
            'SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json': json.dumps(product)})
        files['build/developer-preview/SKAGER-Beta2-source.zip'] = zipped({
            'SOURCE_REFERENCE.json': json.dumps(dict(productCommit=COMMIT))})
        files['build/beta-installer/package.json'] = json.dumps(dict(commit=COMMIT,
            payloadSha256=hashlib.sha256(files['build/beta-installer/payload.zip']).hexdigest())).encode()
        files['evidence/local/windows-peer-cli-receipt.json'] = json.dumps(dict(
            status='success', commit=COMMIT, runId='100', runAttempt='2', job='windows-integration',
            executableSha256=hashlib.sha256(b'inert peer CLI').hexdigest())).encode()
        files['evidence/local/ais-native-runtime/report.json'] = json.dumps(dict(
            passed=True, commit=COMMIT, runId='100', runAttempt='2')).encode()
        for tree in inputs.EVIDENCE_TREES[1:]:
            files[tree + '/summary.json'] = b'{"status":"passed","cleanup":"verified"}'
        # Unrelated build trees must never enter the retained artifact.
        files['build/integration-source/do-not-restore.cpp'] = b'unrelated source'
        for name, value in files.items():
            path = self.producer / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(value)

    def tearDown(self):
        self.temporary.cleanup()

    def seal(self):
        return inputs.seal(self.producer, self.output, PRODUCER)

    def restore(self, digest=None, producer=PRODUCER):
        return inputs.restore(self.consumer, self.output / inputs.ARCHIVE,
                              digest or inputs.sha(self.output / inputs.ARCHIVE),
                              producer, HARNESS, self.receipt)

    def rewrite(self, change):
        archive = self.output / inputs.ARCHIVE
        with zipfile.ZipFile(archive) as source:
            entries = {entry.filename: source.read(entry) for entry in source.infolist()}
        change(entries)
        archive.write_bytes(zipped(entries))

    def test_roundtrip_is_exact_and_separate_from_qualification(self):
        sealed = self.seal()
        restored = self.restore(sealed['archiveSha256'])
        self.assertEqual(restored['producer'], PRODUCER)
        self.assertEqual(restored['harnessCommit'], HARNESS)
        self.assertEqual(restored['workspaceRoot'], str(self.consumer.resolve()))
        self.assertEqual(restored['sourceArchiveSha256'], inputs.sha(self.consumer / 'build/developer-preview/SKAGER-Beta2-source.zip'))
        self.assertEqual(restored['qualification'], 'not-run')
        self.assertEqual(restored['manifestSha256'], sealed['manifestSha256'])
        self.assertFalse((self.consumer / 'build/integration-source').exists())
        for name in inputs.inventory(self.producer):
            self.assertEqual((self.consumer / name).read_bytes(), (self.producer / name).read_bytes())

    def test_feedback_paths_rebase_without_relabeling_or_executing(self):
        self.seal(); restored = self.restore()
        original = self.producer / inputs.FEEDBACK_MANIFEST
        self.assertEqual((self.consumer / inputs.FEEDBACK_MANIFEST).read_bytes(), original.read_bytes())
        manifest = inputs.restored_feedback(self.consumer, self.consumer / inputs.FEEDBACK_MANIFEST,
                                            self.receipt, COMMIT, HARNESS)
        self.assertEqual(manifest['commit'], COMMIT)
        self.assertEqual(restored['harnessCommit'], HARNESS)
        self.assertEqual(restored['boatFeedback']['manifestSha256'], inputs.sha(original))
        self.assertEqual(len(manifest['tests']), 13)
        for entry in manifest['tests']:
            self.assertEqual(entry['path'], str(self.consumer / inputs.FEEDBACK_BINARIES[entry['name']]))

    def test_feedback_manifest_rejects_arbitrary_paths_duplicate_names_and_wrong_commit(self):
        path = self.producer / inputs.FEEDBACK_MANIFEST
        original = path.read_bytes()
        mutations = [
            lambda m: m.update(commit=HARNESS),
            lambda m: m['tests'].pop(),
            lambda m: m['tests'].__setitem__(0, m['tests'][1]),
            lambda m: m['tests'][0].update(path='D:/foreign/arbitrary.exe'),
            lambda m: m['tests'][0].update(path='D:/a/../escape/' + inputs.FEEDBACK_BINARIES[m['tests'][0]['name']]),
            lambda m: m['tests'][0].update(path='D:/another/root/' + inputs.FEEDBACK_BINARIES[m['tests'][0]['name']]),
            lambda m: m['tests'][0].update(path='//server/share/' + inputs.FEEDBACK_BINARIES[m['tests'][0]['name']]),
        ]
        for index, mutation in enumerate(mutations):
            manifest = json.loads(original); mutation(manifest); path.write_text(json.dumps(manifest))
            with self.subTest(index=index), self.assertRaises(ValueError): self.seal()
            self.assertFalse(self.output.exists())
        path.write_bytes(original)

    def test_missing_feedback_manifest_or_any_component_never_seals(self):
        for name in [inputs.FEEDBACK_MANIFEST, *inputs.FEEDBACK_BINARIES.values()]:
            path = self.producer / name; saved = path.read_bytes(); path.unlink()
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'Missing required'): self.seal()
            path.write_bytes(saved)

    def test_feedback_tampering_after_restore_is_refused(self):
        self.seal(); self.restore()
        for name in [inputs.FEEDBACK_MANIFEST, *inputs.FEEDBACK_BINARIES.values()]:
            path = self.consumer / name; saved = path.read_bytes()
            path.write_bytes(saved + b' ')
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'sealed restore receipt'):
                inputs.restored_feedback(self.consumer, self.consumer / inputs.FEEDBACK_MANIFEST,
                                         self.receipt, COMMIT, HARNESS)
            path.write_bytes(saved)

    def test_feedback_wrong_receipt_identity_and_manifest_location_are_refused(self):
        self.seal(); self.restore()
        for product, harness, manifest in [
            (HARNESS, HARNESS, self.consumer / inputs.FEEDBACK_MANIFEST),
            (COMMIT, COMMIT, self.consumer / inputs.FEEDBACK_MANIFEST),
            (COMMIT, HARNESS, self.producer / inputs.FEEDBACK_MANIFEST),
        ]:
            with self.subTest(product=product, harness=harness, manifest=manifest), self.assertRaises(ValueError):
                inputs.restored_feedback(self.consumer, manifest, self.receipt, product, harness)
        record = json.loads(self.receipt.read_bytes()); record['workspaceRoot'] = str(self.producer)
        self.receipt.write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, 'workspace differs'):
            inputs.restored_feedback(self.consumer, self.consumer / inputs.FEEDBACK_MANIFEST,
                                     self.receipt, COMMIT, HARNESS)

    def test_relocated_runner_preserves_failure_gate_and_distinct_identities(self):
        self.seal(); self.restore()
        spec = importlib.util.spec_from_file_location('feedback_runner',
            Path(__file__).with_name('test-boat-feedback-widgets.py'))
        runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
        commands = []
        def process(command, **kwargs):
            commands.append(command)
            child = mock.MagicMock()
            child.__enter__.return_value = child
            child.communicate.return_value = (b'fixture output', b'')
            child.returncode = 1 if len(commands) == 1 else 0
            return child
        output = self.consumer / 'evidence/local/components'
        argv = ['runner', '--manifest', str(self.consumer / inputs.FEEDBACK_MANIFEST),
                '--compiled-input-receipt', str(self.receipt), '--expected-commit', COMMIT,
                '--output', str(output)]
        with mock.patch.object(runner, 'ROOT', self.consumer), \
             mock.patch.object(runner.sys, 'argv', argv), \
             mock.patch.object(runner.sys, 'platform', 'win32'), \
             mock.patch.dict(os.environ, {'GITHUB_ACTIONS': 'true'}), \
             mock.patch.object(runner.subprocess, 'check_output', side_effect=[HARNESS, '']), \
             mock.patch.object(runner.subprocess, 'Popen', side_effect=process), \
             mock.patch('builtins.print'):
            self.assertEqual(runner.main(), 1)
        self.assertEqual(len(commands), 13)
        self.assertEqual({command[0] for command in commands},
                         {str(self.consumer / path) for path in inputs.FEEDBACK_BINARIES.values()})
        result = json.loads((output / 'result.json').read_bytes())
        self.assertFalse(result['passed'])
        self.assertEqual(result['source_commit'], COMMIT)
        self.assertEqual(result['harness_commit'], HARNESS)
        self.assertIsNone(result['source_dirty'])
        self.assertEqual(len(result['tests']), 13)
        self.assertEqual(result['manifest_sha256'], inputs.sha(self.consumer / inputs.FEEDBACK_MANIFEST))

    def test_gui_gate_occurs_only_after_restore_and_never_compiles(self):
        repo = Path(__file__).resolve().parents[1]
        workflow = (repo / '.github/workflows/opennav-baseline.yml').read_text()
        producer = workflow.split('  windows-integration:', 1)[1].split('  windows-qualification:', 1)[0]
        self.assertNotIn('test-boat-feedback-widgets.py', producer)
        qualification = (repo / 'tools/qualify-staging-windows.ps1').read_text()
        self.assertIn("Check 'test-boat-feedback-widgets.py'", qualification)
        self.assertIn("'--compiled-input-receipt'", qualification)
        self.assertNotIn('cmake --build', qualification)
        self.assertNotIn('build-pristine-windows', qualification)

    def test_sealing_is_deterministic(self):
        # Force different ZIP clock ticks without sleeping. A filename passed
        # directly to writestr otherwise inherits wall time and makes this test
        # pass accidentally when both seals finish within the same two seconds.
        with mock.patch.object(zipfile.time, 'localtime',
                               return_value=(2025, 1, 2, 3, 4, 6, 3, 2, 0)):
            first = self.seal()
        for name in inputs.inventory(self.producer):
            os.utime(self.producer / name, (1800000000, 1800000000))
        with mock.patch.object(zipfile.time, 'localtime',
                               return_value=(2026, 7, 8, 9, 10, 12, 2, 189, 0)):
            other = inputs.seal(self.producer, self.base / 'second', PRODUCER)
        self.assertEqual(first['archiveSha256'], other['archiveSha256'])
        self.assertEqual((self.output / inputs.ARCHIVE).read_bytes(),
                         (self.base / 'second' / inputs.ARCHIVE).read_bytes())
        with zipfile.ZipFile(self.output / inputs.ARCHIVE) as archive:
            for entry in archive.infolist():
                self.assertEqual(entry.date_time, (1980, 1, 1, 0, 0, 0))
                self.assertEqual(entry.create_system, 3)
                self.assertEqual(entry.external_attr, 0o100644 << 16)

    def test_corrupted_download_is_refused_before_restore(self):
        sealed = self.seal()
        with (self.output / inputs.ARCHIVE).open('ab') as stream:
            stream.write(b'tampered')
        with self.assertRaisesRegex(ValueError, 'authenticated producer digest'):
            self.restore(sealed['archiveSha256'])
        self.assertFalse((self.consumer / 'build').exists())

    def test_wrong_producer_run_attempt_or_commit_is_refused(self):
        self.seal()
        for key, value in (('runId', '101'), ('runAttempt', '3'), ('commit', 'c'*40), ('job', 'other-job')):
            wrong = dict(PRODUCER, **{key: value})
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'producer identity'):
                self.restore(producer=wrong)
        self.assertFalse((self.consumer / 'build').exists())

    def test_missing_required_file_never_seals(self):
        (self.producer / 'build/production-install/opencpn.exe').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing required'):
            self.seal()
        self.assertFalse(self.output.exists())

    def test_failed_native_evidence_never_seals(self):
        path = self.producer / 'evidence/local/windows-production-tests.xml'
        path.write_text('<testsuite tests="1" failures="1"><testcase><failure/></testcase></testsuite>')
        with self.assertRaisesRegex(ValueError, 'empty or failed'):
            self.seal()

    def test_failed_cleanup_never_seals(self):
        (self.producer / inputs.EVIDENCE_TREES[1] / 'summary.json').write_text('{"status":"passed","cleanup":"pending"}')
        with self.assertRaisesRegex(ValueError, 'verified cleanup'):
            self.seal()

    def test_stale_binary_or_source_identity_never_seals(self):
        (self.producer / 'build/production-install/opencpn.exe').write_bytes(b'another product')
        with self.assertRaisesRegex(ValueError, 'product identity'):
            self.seal()

    def test_missing_manifest_file_or_checksum_mismatch_is_refused(self):
        self.seal()
        self.rewrite(lambda entries: entries.__setitem__('build/production-install/opencpn.exe', b'inert product XX'))
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            self.restore()
        self.assertFalse((self.consumer / 'build').exists())

    def test_extra_sdk_source_and_unsafe_archive_names_are_refused(self):
        self.seal()
        original = (self.output / inputs.ARCHIVE).read_bytes()
        for name in ('../escape', 'build\\evil', 'C:/escape', 'build/app/NUL.txt',
                     'build/app/trailing.', 'build/./ambiguous', 'build/integration-source/new.cpp'):
            (self.output / inputs.ARCHIVE).write_bytes(original)
            self.rewrite(lambda entries: entries.__setitem__(name, b'injected'))
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.restore()
        self.assertFalse((self.consumer / 'build').exists())

    def test_case_collision_is_refused(self):
        self.seal()
        self.rewrite(lambda entries: entries.__setitem__('BUILD/production-install/opencpn.exe', b'injected'))
        with self.assertRaisesRegex(ValueError, 'case-colliding'):
            self.restore()

    def test_linked_archive_member_is_refused(self):
        self.seal()
        archive = self.output / inputs.ARCHIVE
        with zipfile.ZipFile(archive) as source:
            entries = {entry.filename: source.read(entry) for entry in source.infolist()}
        with zipfile.ZipFile(archive, 'w') as target:
            for name, value in entries.items():
                entry = zipfile.ZipInfo(name)
                if name == 'build/production-install/opencpn.exe':
                    entry.external_attr = (0o120777 << 16)
                target.writestr(entry, value)
        with self.assertRaisesRegex(ValueError, 'links, special files'):
            self.restore()
        self.assertFalse((self.consumer / 'build').exists())

    def test_existing_installation_tree_is_never_merged(self):
        self.seal()
        existing = self.consumer / 'build/production-install'
        existing.mkdir(parents=True)
        (existing / 'foreign.dll').write_bytes(b'untrusted preexisting DLL')
        with self.assertRaisesRegex(ValueError, 'fresh build input directories'):
            self.restore()
        self.assertEqual((existing / 'foreign.dll').read_bytes(), b'untrusted preexisting DLL')
        self.assertFalse(self.receipt.exists())


if __name__ == '__main__':
    unittest.main()
