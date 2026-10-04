#!/usr/bin/env python3
"""Authenticate inert CI artifacts with transport-only GitHub mocks; no network."""
import hashlib
import io
import json
from pathlib import Path
import tempfile
import struct
import unittest
import zipfile

import fetch_ci_inputs as fetcher
from github_release_delivery import GitHub

SHA = 'a' * 40
REPO = 'ThereptileII/Work'


def zipped(entries, symlink=None):
    output = io.BytesIO()
    with zipfile.ZipFile(output, 'w') as archive:
        for name, value in entries.items():
            entry = zipfile.ZipInfo()
            entry.filename = entry.orig_filename = name
            if name == symlink:
                entry.external_attr = 0o120777 << 16
            archive.writestr(entry, value)
    return output.getvalue()


class TransportGitHub(GitHub):
    """Retain the actual JSON/API/pagination implementation above this seam."""
    def __init__(self, kind='staging', entries=None):
        super().__init__(REPO)
        self.calls = []
        self.kind = kind
        self.payload = zipped(entries or ({'STAGING_BUILD_INPUTS.zip': b'inert inner ZIP',
                                           'receipt.json': b'{}'} if kind == 'staging' else
                                          {'bundle.json': b'{}', 'payload/build/runtime.dll': b'inert DLL'}))
        workflow = fetcher.STAGING if kind == 'staging' else fetcher.DEPENDENCIES
        job = 'windows-integration' if kind == 'staging' else 'windows-dependencies'
        prefix = 'staging-build-' + SHA if kind == 'staging' else 'windows-dependencies-' + 'b' * 64
        self.selection = dict(repository=REPO, runId='123', runAttempt='2', headSha=SHA,
            artifactId='456', artifactName=prefix + '-run123-attempt2',
            artifactDigest='sha256:' + hashlib.sha256(self.payload).hexdigest())
        self.run = dict(id=123, run_attempt=2, head_sha=SHA, path=workflow, event='workflow_dispatch',
                        head_repository=dict(full_name=REPO), status='completed', conclusion='success')
        self.jobs = [dict(name=job, status='completed', conclusion='success', head_sha=SHA,
                          run_id=123, run_attempt=2)]
        self.artifact = dict(id=456, name=self.selection['artifactName'], expired=False,
            digest=self.selection['artifactDigest'], size_in_bytes=len(self.payload),
            workflow_run=dict(id=123, head_sha=SHA))

    def _run(self, arguments, *, data=None, output=None):
        self.calls.append(arguments)
        assert data is None and arguments[0] == 'api'
        endpoint = arguments[1]
        if endpoint == self.base + '/actions/artifacts/456/zip':
            assert output is not None
            output.write(self.payload)
            return None
        assert output is None
        if endpoint == self.base + '/actions/runs/123/attempts/2':
            result = self.run
        elif endpoint.startswith(self.base + '/actions/runs/123/attempts/2/jobs?per_page=100&page='):
            page = int(endpoint.rsplit('=', 1)[1])
            result = dict(jobs=self.jobs[(page - 1) * 100:page * 100])
        elif endpoint == self.base + '/actions/artifacts/456':
            result = self.artifact
        else:
            raise AssertionError('Unexpected transport endpoint: ' + endpoint)
        return json.dumps(result).encode()


class Authentication(unittest.TestCase):
    def authority(self, gh):
        return fetcher.authenticated_artifact(gh, gh.selection, kind=gh.kind)

    def test_staging_accepts_passed_producer_from_failed_overall_run(self):
        gh = TransportGitHub()
        gh.run['conclusion'] = 'failure'
        result = self.authority(gh)
        self.assertEqual(result['job'], 'windows-integration')
        self.assertEqual(result['runAttempt'], '2')

    def test_dependencies_require_successful_run_and_job(self):
        for key, value in (('conclusion', 'failure'), ('status', 'in_progress')):
            gh = TransportGitHub('dependencies')
            gh.run[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'completed successfully'):
                self.authority(gh)
        self.assertEqual(self.authority(TransportGitHub('dependencies'))['job'], 'windows-dependencies')

    def test_actual_pagination_finds_producer_on_second_page(self):
        gh = TransportGitHub()
        gh.jobs = [dict(name='unrelated-' + str(i)) for i in range(100)] + gh.jobs
        self.assertEqual(self.authority(gh)['conclusion'], 'success')
        self.assertTrue(any('page=2' in call[1] for call in gh.calls))

    def test_wrong_run_provenance_is_refused(self):
        cases = dict(id=124, run_attempt=3, head_sha='b'*40, path=fetcher.DEPENDENCIES,
                     event='pull_request', head_repository=dict(full_name='attacker/fork'))
        for key, value in cases.items():
            gh = TransportGitHub(); gh.run[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'provenance differs'):
                self.authority(gh)

    def test_wrong_selection_repository_and_unpinned_fields_are_refused(self):
        for change in ({'repository':'attacker/fork'}, {'headSha':'latest'}, {'runAttempt':'02'},
                       {'artifactDigest':'not-a-digest'}, {'latest':True}):
            gh = TransportGitHub(); gh.selection.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.authority(gh)
            self.assertFalse(gh.calls)

    def test_wrong_failed_or_ambiguous_job_is_refused(self):
        for change in ({'name':'Friendly display name'}, {'status':'in_progress'},
                       {'conclusion':'failure'}, {'head_sha':'b'*40},
                       {'run_id':124}, {'run_attempt':3}):
            gh = TransportGitHub(); gh.jobs[0].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'Producer job'):
                self.authority(gh)
        gh = TransportGitHub(); gh.jobs *= 2
        with self.assertRaisesRegex(ValueError, 'Producer job'):
            self.authority(gh)

    def test_wrong_artifact_identity_or_expiry_is_refused(self):
        cases = dict(id=457, name='other', expired=True, digest='sha256:'+'0'*64,
                     workflow_run=dict(id=124, head_sha=SHA), size_in_bytes=fetcher.MAX_BYTES+1)
        for key, value in cases.items():
            gh = TransportGitHub(); gh.artifact[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Artifact provenance'):
                self.authority(gh)
        gh = TransportGitHub(); gh.artifact['workflow_run']['head_sha'] = 'b'*40
        with self.assertRaisesRegex(ValueError, 'Artifact provenance'):
            self.authority(gh)

    def test_artifact_name_must_bind_exact_attempt_even_when_metadata_agrees(self):
        gh = TransportGitHub()
        gh.artifact['name'] = gh.selection['artifactName'] = 'staging-build-' + SHA + '-run123-attempt1'
        with self.assertRaisesRegex(ValueError, 'exact producer attempt'):
            self.authority(gh)


class Download(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.output = self.root / 'artifact'
        self.provenance = self.root / 'authority.json'

    def tearDown(self):
        self.temp.cleanup()

    def fetch(self, gh):
        return fetcher.fetch(gh, gh.selection, self.output, self.provenance, kind=gh.kind)

    def test_download_hashes_inner_input_only_after_authenticated_outer_digest(self):
        gh = TransportGitHub()
        result = self.fetch(gh)
        self.assertEqual(result['archiveSha256'], hashlib.sha256(b'inert inner ZIP').hexdigest())
        self.assertEqual(json.loads(self.provenance.read_text()), result)
        self.assertEqual(gh.calls[-1], ['api', gh.base + '/actions/artifacts/456/zip'])

    def test_downloaded_bytes_must_match_the_authenticated_artifact_digest(self):
        gh = TransportGitHub(); gh.payload += b'tampering after metadata'
        with self.assertRaisesRegex(ValueError, 'Downloaded artifact digest'):
            self.fetch(gh)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.provenance.exists())

    def test_dependency_payload_is_bound_by_downloaded_outer_digest(self):
        result = self.fetch(TransportGitHub('dependencies'))
        self.assertEqual(result['bundleSha256'], hashlib.sha256(b'{}').hexdigest())

    def test_provenance_cannot_overwrite_input_or_be_written_inside_it(self):
        self.provenance = self.output / 'authority.json'
        with self.assertRaisesRegex(ValueError, 'external provenance'):
            self.fetch(TransportGitHub())
        self.assertFalse(self.output.exists())

    def test_archive_rejects_unsafe_aliases_before_creating_destination(self):
        for name in ('../escape', '/absolute', 'payload\\evil', 'C:/escape',
                     'payload/./evil', 'payload//evil', 'payload/trailing.', 'payload/trailing ',
                     'payload/CON.txt', 'payload/COM1', 'payload/NUL', 'payload/name:ads',
                     'payload/a\x00ignored', 'payload/question?', 'payload/asterisk*'):
            with self.subTest(name=name):
                archive = self.root / 'unsafe.zip'
                archive.write_bytes(zipped({'bundle.json':b'{}', name:b'bad'}))
                with zipfile.ZipFile(archive) as wire:
                    self.assertEqual(wire.infolist()[-1].orig_filename, name)
                with self.assertRaises(ValueError):
                    fetcher.unpack(archive, self.output, 'dependencies')
                self.assertFalse(self.output.exists())

    def test_archive_rejects_case_alias_links_extras_and_file_directory_collision(self):
        cases = [({'STAGING_BUILD_INPUTS.zip':b'zip','receipt.json':b'{}','Receipt.json':b'extra'}, None, 'staging'),
                 ({'STAGING_BUILD_INPUTS.zip':b'zip','receipt.json':b'{}'}, 'receipt.json', 'staging'),
                 ({'STAGING_BUILD_INPUTS.zip':b'zip','receipt.json':b'{}','extra':b'bad'}, None, 'staging'),
                 ({'bundle.json':b'{}','payload/a':b'file','payload/a/child':b'bad'}, None, 'dependencies')]
        for entries, link, kind in cases:
            archive = self.root / 'bad.zip'; archive.write_bytes(zipped(entries, symlink=link))
            with self.subTest(entries=list(entries)), self.assertRaises(ValueError):
                fetcher.unpack(archive, self.output, kind)
            self.assertFalse(self.output.exists())

    def test_encrypted_zip_is_refused_before_writing_any_member(self):
        raw = bytearray(zipped({'bundle.json':b'{}', 'payload/data':b'data'}))
        for marker, offset in ((b'PK\x03\x04',6), (b'PK\x01\x02',8)):
            location = raw.index(marker) + offset
            struct.pack_into('<H', raw, location, struct.unpack_from('<H', raw, location)[0] | 1)
        archive = self.root / 'encrypted.zip'; archive.write_bytes(raw)
        with self.assertRaisesRegex(ValueError, 'encrypted'):
            fetcher.unpack(archive, self.output, 'dependencies')
        self.assertFalse(self.output.exists())

    def test_unknown_kind_is_not_an_alias_for_staging(self):
        gh = TransportGitHub()
        with self.assertRaisesRegex(ValueError, 'kind'):
            fetcher.authenticated_artifact(gh, gh.selection, kind='typo')


if __name__ == '__main__':
    unittest.main()
