"""Offline release boundary tests; no GitHub, boat or executable is contacted."""
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
import warnings
import zipfile

import beta2_handoff as handoff


def zipped(entries):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            for name, value in entries:
                archive.writestr(name, value)
    return stream.getvalue()


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'docs').mkdir()
        (self.root / 'docs/review.md').write_text('Explicit test-only review fixture')
        evidence = {'status': 'passed', 'path': 'docs/review.md',
                    'sha256': handoff.digest((self.root / 'docs/review.md').read_bytes())}
        self.record = dict(schema=1, owner='OpenNavX.Beta2.BoatAcceptance.1',
                           version=handoff.VERSION, status='accepted', physicalCommands=0,
                           commit='1' * 40, runId=7, runAttempt=1,
                           boatGates={key: dict(evidence) for key in handoff.BOAT_GATES},
                           limitations=['No physical actuator test'])
        self.build = dict(version=handoff.VERSION, commit=self.record['commit'],
                          test_fixtures=False, build_purpose='INSTALLED PRODUCT',
                          executable_sha256=handoff.digest(b'inert test bytes'))
        self.source = dict(productCommit=self.record['commit'], upstreamCommit=handoff.UPSTREAM,
                           openCpnVersion='5.12.4')
        self.payload = {name: b'test-only content' for name in handoff.FILES}
        self.refresh_inner()
        self.run = dict(id=7, run_attempt=1, head_sha=self.record['commit'],
                        status='completed', conclusion='success',
                        path='.github/workflows/opennav-baseline.yml',
                        repository={'full_name': handoff.REPOSITORY})
        self.jobs = {'total_count': len(handoff.JOBS), 'jobs': [
            dict(name=name, status='completed', conclusion='success',
                 head_sha=self.record['commit'], run_id=7, run_attempt=1)
            for name in sorted(handoff.JOBS)]}
        self.artifact = dict(id=9, expired=False, name='beta-candidate-' + self.record['commit'],
                             size_in_bytes=self.record['artifact']['bytes'],
                             digest='sha256:' + self.record['artifact']['sha256'],
                             workflow_run={'id': 7, 'head_sha': self.record['commit']})

    def refresh_inner(self):
        prefix = 'OpenNavX-Beta2-Portable-Recovery/'
        notes = self.payload[handoff.RELEASE_NOTES]
        notes_hash = handoff.digest(notes)
        self.source['files'] = {'opennav-x/docs/beta2/' + handoff.RELEASE_NOTES: {'sha256': notes_hash}}
        self.payload['OpenNavX-Beta2-Portable-Recovery.zip'] = zipped([
            (prefix + 'docs/' + handoff.RELEASE_NOTES, notes),
            (prefix + 'FILE_SHA256.json', json.dumps({'docs/' + handoff.RELEASE_NOTES: notes_hash})),
            (prefix + 'docs/PRODUCT_BUILD.json', json.dumps(self.build)),
            (prefix + 'app/opencpn.exe', b'inert test bytes')])
        self.payload['OpenNavX-Beta2-source.zip'] = zipped([
            ('SOURCE_REFERENCE.json', json.dumps(self.source)),
            ('opennav-x/docs/beta2/' + handoff.RELEASE_NOTES, notes)])
        self.repack()

    def repack(self, extra=()):
        self.record['payloadSha256'] = {name: handoff.digest(self.payload[name])
                                       for name in handoff.FILES}
        self.payload['SHA256SUMS.txt'] = ''.join(value + '  ' + name + '\n' for name, value in
                                               sorted(self.record['payloadSha256'].items())).encode()
        self.payload['QUALIFICATION.txt'] = b'Original candidate note'
        self.archive = zipped(list(self.payload.items()) + list(extra))
        self.record['artifact'] = dict(id=9, bytes=len(self.archive), sha256=handoff.digest(self.archive))

    def test_complete_unchanged_set(self):
        handoff.validate_acceptance(self.record, self.root)
        handoff.validate_ci(self.record, self.run, self.jobs, self.artifact)
        self.assertEqual(handoff.verify_payload(self.record, self.archive), self.payload)

    def test_incomplete_or_unreviewed_boat_gate(self):
        for change in ('missing', 'pending', 'changed'):
            with self.subTest(change=change):
                record = copy.deepcopy(self.record)
                if change == 'missing': del record['boatGates']['maintenance']
                elif change == 'pending': record['boatGates']['maintenance']['status'] = 'pending'
                else: record['boatGates']['maintenance']['sha256'] = '0' * 64
                with self.assertRaises(ValueError): handoff.validate_acceptance(record, self.root)

    def test_external_evidence_path(self):
        for path in ('../review.md', 'docs/../review.md', '/tmp/review.md', 'docs\\review.md'):
            with self.subTest(path=path):
                self.record['boatGates']['feedback']['path'] = path
                with self.assertRaises(ValueError): handoff.validate_acceptance(self.record, self.root)

    def test_wrong_stage_control_or_identity(self):
        for key, value in [('status', 'pending'), ('version', '0.3.0-beta1'),
                           ('physicalCommands', 1), ('physicalCommands', False),
                           ('commit', 'latest'), ('runId', True), ('runAttempt', 0)]:
            with self.subTest(key=key, value=value):
                record = dict(self.record, **{key: value})
                with self.assertRaises(ValueError): handoff.validate_acceptance(record, self.root)

    def test_run_not_fully_qualified(self):
        for key, value in [('status', 'in_progress'), ('conclusion', 'failure'),
                           ('head_sha', '2' * 40), ('run_attempt', 2),
                           ('path', '.github/workflows/opennav-boat-tools.yml')]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    handoff.validate_ci(self.record, dict(self.run, **{key: value}), self.jobs, self.artifact)

    def test_missing_or_duplicate_gate(self):
        for duplicate in (False, True):
            jobs = copy.deepcopy(self.jobs)
            jobs['jobs'].pop()
            if duplicate: jobs['jobs'].append(dict(jobs['jobs'][0]))
            jobs['total_count'] = len(jobs['jobs'])
            with self.assertRaises(ValueError):
                handoff.validate_ci(self.record, self.run, jobs, self.artifact)

    def test_partial_job_failed_skipped_or_different_attempt(self):
        for key, value in [('conclusion', 'failure'), ('conclusion', 'skipped'),
                           ('status', 'in_progress'), ('run_attempt', 2), ('head_sha', '2' * 40)]:
            with self.subTest(key=key, value=value):
                jobs = copy.deepcopy(self.jobs); jobs['jobs'][0][key] = value
                with self.assertRaises(ValueError):
                    handoff.validate_ci(self.record, self.run, jobs, self.artifact)

    def test_artifact_expired_replaced_or_early(self):
        for key, value in [('expired', True), ('id', 10), ('size_in_bytes', 1),
                           ('digest', 'sha256:' + '0' * 64),
                           ('name', 'beta2-boat-review-pending-endurance-' + self.record['commit'])]:
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    handoff.validate_ci(self.record, self.run, self.jobs, dict(self.artifact, **{key: value}))

    def test_corrupt_transferred_archive(self):
        with self.assertRaises(ValueError): handoff.verify_payload(self.record, self.archive + b'x')

    def test_unexpected_traversal_and_duplicate_entries(self):
        for name in ('../bad.exe', 'folder/file', 'QUALIFICATION.txt'):
            with self.subTest(name=name):
                self.repack([(name, b'unsafe')])
                with self.assertRaises(ValueError): handoff.verify_payload(self.record, self.archive)

    def test_manifest_fails_to_match_reviewed_bytes(self):
        self.record['payloadSha256']['OpenNavX-Beta2-Setup.exe'] = '0' * 64
        with self.assertRaises(ValueError): handoff.verify_payload(self.record, self.archive)

    def test_fixture_or_wrong_product_identity(self):
        for key, value in [('test_fixtures', True), ('test_fixtures', 'false'),
                           ('commit', '2' * 40), ('build_purpose', 'CI FIXTURE'),
                           ('executable_sha256', '0' * 64)]:
            with self.subTest(key=key):
                old = self.build[key]; self.build[key] = value; self.refresh_inner()
                with self.assertRaises(ValueError): handoff.verify_payload(self.record, self.archive)
                self.build[key] = old

    def test_release_notes_are_required_reviewed_payload(self):
        record = copy.deepcopy(self.record)
        del record['payloadSha256'][handoff.RELEASE_NOTES]
        with self.assertRaisesRegex(ValueError, 'six reviewed payload hashes'):
            handoff.validate_acceptance(record, self.root)

    def test_outer_release_notes_must_match_packaged_copy(self):
        self.payload[handoff.RELEASE_NOTES] = b'changed notes after package assembly'
        self.repack()
        with self.assertRaisesRegex(ValueError, 'Portable release notes differ'):
            handoff.verify_payload(self.record, self.archive)

    def test_packaged_release_notes_missing_changed_or_wrong_hash(self):
        original = dict(self.payload)
        for archive_name, notes_path, manifest_name in (
            ('OpenNavX-Beta2-Portable-Recovery.zip',
             'OpenNavX-Beta2-Portable-Recovery/docs/' + handoff.RELEASE_NOTES,
             'OpenNavX-Beta2-Portable-Recovery/FILE_SHA256.json'),
            ('OpenNavX-Beta2-source.zip', 'opennav-x/docs/beta2/' + handoff.RELEASE_NOTES,
             'SOURCE_REFERENCE.json')):
            for mutation in ('missing', 'changed', 'hash'):
                with self.subTest(archive=archive_name, mutation=mutation):
                    self.payload = dict(original)
                    with zipfile.ZipFile(io.BytesIO(self.payload[archive_name])) as inner:
                        entries = {name: inner.read(name) for name in inner.namelist()}
                    if mutation == 'missing':
                        del entries[notes_path]
                    elif mutation == 'changed':
                        entries[notes_path] = b'different package notes'
                    else:
                        manifest = json.loads(entries[manifest_name])
                        if manifest_name == 'SOURCE_REFERENCE.json':
                            manifest['files'][notes_path]['sha256'] = '0' * 64
                        else:
                            manifest['docs/' + handoff.RELEASE_NOTES] = '0' * 64
                        entries[manifest_name] = json.dumps(manifest).encode()
                    self.payload[archive_name] = zipped(entries.items())
                    self.repack()
                    with self.assertRaisesRegex(ValueError, 'release notes'):
                        handoff.verify_payload(self.record, self.archive)

    def test_wrong_corresponding_source(self):
        for key, value in [('productCommit', '2' * 40), ('upstreamCommit', '2' * 40),
                           ('openCpnVersion', '5.12.2')]:
            with self.subTest(key=key):
                old = self.source[key]; self.source[key] = value; self.refresh_inner()
                with self.assertRaises(ValueError): handoff.verify_payload(self.record, self.archive)
                self.source[key] = old


if __name__ == '__main__':
    unittest.main()
