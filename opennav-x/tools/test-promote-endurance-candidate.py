#!/usr/bin/env python3
"""Tiny inert release-boundary fixtures; no application, CI or network access."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('promotion', Path(__file__).with_name('promote-endurance-candidate.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def zipped(values):
    output = io.BytesIO()
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name, data in values.items():
            archive.writestr(name, data)
    return output.getvalue()


class PromotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.review = self.root/'review'
        self.review.mkdir()
        self.commit = 'a'*40
        self.identity = p.expected_identity({'GITHUB_REPOSITORY': p.REPOSITORY, 'GITHUB_SHA': self.commit,
                                             'GITHUB_RUN_ID': '7', 'GITHUB_RUN_ATTEMPT': '1'})
        self.payload = {name: b'inert payload' for name in p.FILES}
        self.build = {'version': p.VERSION, 'commit': self.commit, 'test_fixtures': False,
                      'build_purpose': 'INSTALLED PRODUCT', 'xnav_hardware_output_policy': 'status-only',
                      'executable_sha256': p.fact(b'inert executable')['sha256']}
        self.executable = b'inert executable'
        self.refresh()

    def refresh(self):
        prefix = 'SKAGER-Beta2-Portable-Recovery/'
        notes = self.payload[p.RELEASE_NOTES]
        self.payload['SKAGER-Beta2-Portable-Recovery.zip'] = zipped({
            prefix+'docs/PRODUCT_BUILD.json': json.dumps(self.build), prefix+'app/opencpn.exe': self.executable,
            prefix+'docs/'+p.RELEASE_NOTES: notes,
            prefix+'FILE_SHA256.json': json.dumps({'docs/'+p.RELEASE_NOTES: p.fact(notes)['sha256']})})
        name = 'opennav-x/docs/beta2/'+p.RELEASE_NOTES
        self.payload['SKAGER-Beta2-source.zip'] = zipped({name: notes, 'SOURCE_REFERENCE.json': json.dumps({
            'productCommit': self.commit, 'upstreamCommit': p.UPSTREAM, 'openCpnVersion': '5.12.4',
            'files': {name: {'sha256': p.fact(notes)['sha256']}}})})
        self.payload['SHA256SUMS.txt'] = ''.join(p.fact(self.payload[name])['sha256']+'  '+name+'\n'
                                               for name in sorted(p.FILES)).encode()
        self.payload['QUALIFICATION.txt'] = p.PENDING
        for name, data in self.payload.items():
            (self.review/name).write_bytes(data)

    def test_unchanged_payloads_and_new_output(self):
        self.payload['QUALIFICATION.txt'] = p.PENDING.replace(b'\n', b'\r\n')
        (self.review/'QUALIFICATION.txt').write_bytes(self.payload['QUALIFICATION.txt'])
        payload = p.verify_payloads(self.review, self.commit)
        output = self.root/'output'
        p.promote(self.review, output, payload)
        self.assertEqual({x.name for x in output.iterdir()}, set(self.payload))
        for name in p.FILES | {'SHA256SUMS.txt'}:
            self.assertEqual((output/name).read_bytes(), self.payload[name])
        self.assertEqual((output/'QUALIFICATION.txt').read_bytes(), p.CANDIDATE.replace(b'\n', b'\r\n'))
        self.assertEqual((self.review/'QUALIFICATION.txt').read_bytes(), self.payload['QUALIFICATION.txt'])
        with self.assertRaisesRegex(ValueError, 'new'):
            p.promote(self.review, output, payload)

    def test_payload_tamper_and_missing_inventory(self):
        setup = self.review/'SKAGER-Beta2-Setup.exe'
        setup.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'hash changed'):
            p.verify_payloads(self.review, self.commit)
        setup.unlink()
        with self.assertRaisesRegex(ValueError, 'eight-file'):
            p.verify_payloads(self.review, self.commit)

    def test_wrong_product_commit_fixtures_hardware_and_executable(self):
        for key, value in [('commit', 'b'*40), ('test_fixtures', True),
                           ('xnav_hardware_output_policy', 'enabled'), ('executable_sha256', '0'*64)]:
            with self.subTest(key=key):
                original = self.build[key]
                self.build[key] = value
                self.refresh()
                with self.assertRaises(ValueError):
                    p.verify_payloads(self.review, self.commit)
                self.build[key] = original

    def test_qualification_and_checksum_set_cannot_be_relabelled(self):
        (self.review/'QUALIFICATION.txt').write_bytes(p.CANDIDATE)
        with self.assertRaisesRegex(ValueError, 'pending'):
            p.verify_payloads(self.review, self.commit)
        self.refresh()
        sums = self.review/'SHA256SUMS.txt'
        sums.write_bytes(sums.read_bytes().split(b'\n', 1)[1])
        with self.assertRaisesRegex(ValueError, 'six payloads'):
            p.verify_payloads(self.review, self.commit)

    def test_source_mutation_during_promotion_is_refused_without_output(self):
        payload = p.verify_payloads(self.review, self.commit)
        (self.review/'SKAGER-Beta2-Setup.exe').write_bytes(b'changed after validation')
        output = self.root/'output'
        with self.assertRaisesRegex(ValueError, 'changed during'):
            p.promote(self.review, output, payload)
        self.assertFalse(output.exists())

    def soak_fixture(self):
        directory = self.root/'soak'
        directory.mkdir()
        (self.root/'tools').mkdir()
        (self.root/'release').mkdir()
        sources = {'tools/soak-runtime.py': b'# inert harness fixture\n',
                   'release/qualification.json': b'{"enduranceSeconds":10800}\n'}
        for name, data in sources.items():
            (self.root/name).write_bytes(data)
        manifest = {'schema': 1, 'owner': 'SKAGER.NativeEnduranceHandoff.1', 'identity': self.identity,
                    'required_seconds': 10800,
                    'sources': {name: p.fact(data.replace(b'\n', b'\r\n')) for name, data in sources.items()},
                    'files': {'build/xnav-install/opencpn.exe': p.fact(b'inert fixture exe')}}
        report = {'status': 'passed', 'release_duration': True, 'authority': 'native Windows',
                  'binary_matches_harness_commit': True, 'requested_seconds': 10800, 'elapsed_seconds': 10800.2,
                  'harness_commit': self.commit, 'build_commit': self.commit,
                  'harness_sha256': manifest['sources']['tools/soak-runtime.py']['sha256'],
                  'executable': manifest['files']['build/xnav-install/opencpn.exe']}
        return directory, manifest, report

    def write_soak(self, directory, manifest, report):
        manifest_bytes, report_bytes = json.dumps(manifest).encode(), json.dumps(report).encode()
        receipt = {'schema': 1, 'owner': 'SKAGER.NativeEnduranceResult.1', 'identity': self.identity,
                   'consumer_job': 'windows-endurance', 'required_seconds': 10800, 'runtime_unchanged': True,
                   'stage_manifest': p.fact(manifest_bytes), 'report': p.fact(report_bytes)}
        receipt.update({key: report[key] for key in ('status', 'authority', 'requested_seconds', 'elapsed_seconds',
                        'harness_commit', 'build_commit', 'harness_sha256', 'binary_matches_harness_commit', 'executable')})
        for name, value in [('runtime-manifest.json', manifest_bytes), ('results.json', report_bytes),
                            ('handoff-qualified.json', json.dumps(receipt).encode())]:
            (directory/name).write_bytes(value)

    def test_exact_native_soak_receipt_and_crlf_sources_pass(self):
        directory, manifest, report = self.soak_fixture()
        self.write_soak(directory, manifest, report)
        self.assertEqual(p.validate_soak(directory, self.identity, self.root)['status'], 'passed')

    def test_soak_rejects_wrong_commit_short_duration_and_non_native(self):
        directory, manifest, report = self.soak_fixture()
        for key, value in [('build_commit', 'b'*40), ('harness_commit', 'b'*40),
                           ('elapsed_seconds', 10799.9), ('requested_seconds', 120),
                           ('authority', 'Linux development'), ('release_duration', False),
                           ('harness_sha256', '0'*64), ('binary_matches_harness_commit', False)]:
            with self.subTest(key=key):
                changed = dict(report, **{key: value})
                self.write_soak(directory, manifest, changed)
                with self.assertRaises(ValueError):
                    p.validate_soak(directory, self.identity, self.root)

    def test_soak_rejects_missing_tampered_and_wrong_attempt_receipts(self):
        directory, manifest, report = self.soak_fixture()
        self.write_soak(directory, manifest, report)
        (directory/'results.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'identity changed'):
            p.validate_soak(directory, self.identity, self.root)
        self.write_soak(directory, manifest, report)
        for key, value in [('run_attempt', '2'), ('run_id', '8'), ('repository', 'another/repository'), ('commit', 'b'*40)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                p.validate_soak(directory, dict(self.identity, **{key: value}), self.root)
        (directory/'handoff-qualified.json').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing'):
            p.validate_soak(directory, self.identity, self.root)

    def test_failed_soak_creates_no_candidate_directory(self):
        directory, manifest, report = self.soak_fixture()
        report['elapsed_seconds'] = 10799
        self.write_soak(directory, manifest, report)
        output = self.root/'candidate'
        environment = {'GITHUB_REPOSITORY': p.REPOSITORY, 'GITHUB_SHA': self.commit,
                       'GITHUB_RUN_ID': '7', 'GITHUB_RUN_ATTEMPT': '1'}
        with patch.object(p, 'ROOT', self.root), patch.dict(p.os.environ, environment), patch('sys.argv', [
                'promotion', '--review-dir', str(self.review), '--soak-dir', str(directory), '--output', str(output)]):
            with self.assertRaisesRegex(ValueError, 'duration'):
                p.main()
        self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
