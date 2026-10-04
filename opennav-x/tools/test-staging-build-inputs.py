#!/usr/bin/env python3
"""Inert artifact-boundary tests; no application, compiler or Windows execution."""
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
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

    def test_sealing_is_deterministic(self):
        first = self.seal()
        other = inputs.seal(self.producer, self.base / 'second', PRODUCER)
        self.assertEqual(first['archiveSha256'], other['archiveSha256'])

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
