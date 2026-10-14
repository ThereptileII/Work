"""Inert API/ZIP composition tests. No credentials, network, approval or native actions."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from datetime import timedelta
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import stat
import sys
import unittest
from unittest.mock import patch
import warnings
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import installed_updater_approval_transport as transport

spec = importlib.util.spec_from_file_location('inert_approval_fixture',
    Path(__file__).with_name('test-installed-updater-approval.py'))
fixture_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture_module)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture_module.HandoffTests()
        self.f.setUp()
        self.arguments = {key: value for key, value in self.f.inputs().items() if key in
            ('request_bytes', 'client_request', 'declaration', 'release_policy', 'expected_setup',
             'expected_runner', 'expected_challenge', 'expected_candidate_artifact', 'expected_operator_kit_artifact')}
        self.members = {'request.json': self.f.request_bytes, 'approval.json': self.f.approval_bytes,
                        'approval-pins.json': fixture_module.encode(dict(schema=1, purpose=transport.PINS_PURPOSE,
                            requestSha256=transport.binding.sha256(self.f.request_bytes),
                            approvalSha256=transport.binding.sha256(self.f.approval_bytes), pins=self.f.pins))}
        self.source = transport.ApprovalSource(transport.binding.REPOSITORY, '9' * 40, '700', '1',
            '10', '10', '701', '702',
            transport.artifact_name('700', '1', transport.binding.sha256(self.f.request_bytes)), '', 1)
        self.run = dict(id=700, run_attempt=1, head_sha='9' * 40, path=transport.WORKFLOW,
                        head_repository={'full_name': transport.binding.REPOSITORY},
                        repository={'full_name': transport.binding.REPOSITORY}, event='workflow_dispatch',
                        actor={'id': 10}, triggering_actor={'id': 10}, status='completed', conclusion='success',
                        created_at='2026-10-13T12:00:30Z')
        self.job = dict(id=701, name=transport.JOB, run_id=700, run_attempt=1, head_sha='9' * 40,
                        status='completed', conclusion='success', started_at='2026-10-13T12:01:00Z',
                        completed_at='2026-10-13T12:01:30Z')
        self.artifact = dict(id=702, name=self.source.artifact_name, expired=False,
                             workflow_run={'id': 700, 'head_sha': '9' * 40}, created_at='2026-10-13T12:01:10Z')
        self.set_archive(self.make_archive())
        self.calls, self.downloads, self.sleeps = [], [], []
        self.tick, self.cancelled, self.pending_reads = 0, False, 0

    def make_archive(self, members=None):
        data = io.BytesIO()
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            with zipfile.ZipFile(data, 'w', compression=zipfile.ZIP_STORED) as archive:
                for name, raw in (self.members.items() if members is None else members):
                    archive.writestr(name, raw)
        return data.getvalue()

    def set_archive(self, raw):
        self.raw = raw
        digest = 'sha256:' + hashlib.sha256(raw).hexdigest()
        self.source = replace(self.source, artifact_digest=digest, artifact_bytes=len(raw))
        self.artifact.update(digest=digest, size_in_bytes=len(raw))

    def read_api(self, endpoint, *, timeout_seconds, max_bytes):
        self.assertGreater(timeout_seconds, 0)
        self.assertLessEqual(timeout_seconds, 30)
        self.assertEqual(max_bytes, transport.MAX_API_BYTES)
        self.calls.append(endpoint)
        if endpoint.endswith('/attempts/1'):
            if self.pending_reads:
                self.pending_reads -= 1
                return dict(self.run, status='in_progress', conclusion=None)
            return deepcopy(self.run)
        if endpoint.endswith('/jobs?per_page=100&page=1'):
            return dict(total_count=1, jobs=[deepcopy(self.job)])
        if endpoint.endswith('/artifacts/702'):
            return deepcopy(self.artifact)
        self.fail('Unexpected API endpoint: ' + endpoint)

    def download(self, endpoint, *, timeout_seconds, max_bytes):
        self.assertEqual(endpoint, 'repos/ThereptileII/Work/actions/artifacts/702/zip')
        self.assertEqual(max_bytes, self.source.artifact_bytes)
        self.assertGreater(timeout_seconds, 0)
        self.assertLessEqual(timeout_seconds, 30)
        self.downloads.append(endpoint)
        return self.raw

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.tick += seconds

    def receive(self, **changes):
        args = dict(source=self.source, read_api=self.read_api, download_archive=self.download,
                    bind_arguments=self.arguments, timeout_seconds=30, poll_seconds=5,
                    cancelled=lambda: self.cancelled, monotonic=lambda: self.tick,
                    sleep=self.sleep, utcnow=lambda: self.f.now + timedelta(seconds=self.tick))
        args.update(changes)
        return transport.receive_approval(**args)

    def test_exact_operator_artifact_returns_original_owned_bound_approval_without_writes(self):
        with patch('builtins.open', side_effect=AssertionError('unexpected file I/O')):
            result = self.receive()
        self.assertEqual(result.original_approval_bytes, self.f.approval_bytes)
        self.assertEqual(result.original_client_request_bytes, self.f.client_bytes)
        self.assertEqual(result.original_declaration_bytes, self.f.declaration_bytes)
        self.assertEqual(result.original_release_policy_bytes, self.f.policy_bytes)
        self.assertEqual(dict(result.request.pins), self.f.pins)
        self.assertEqual(len(self.downloads), 1)
        self.assertEqual(len(self.calls), 4)
        with self.assertRaises(FrozenInstanceError):
            result.setup.size = 99

    def test_pending_producer_is_polled_only_until_success(self):
        self.pending_reads = 2
        self.receive()
        self.assertEqual(self.sleeps, [5, 5])
        self.assertEqual(len(self.downloads), 1)
        self.assertTrue(all('/actions/' in path for path in self.calls))

    def test_run_repository_attempt_workflow_commit_event_and_both_root_actors_required(self):
        mutations = [('id', 999), ('run_attempt', 2), ('head_sha', '0' * 40),
                     ('path', '.github/workflows/other.yml'), ('event', 'push'),
                     ('head_repository', {'full_name': 'other/Work'}),
                     ('repository', {'full_name': 'other/Work'}), ('actor', {'id': 11}),
                     ('triggering_actor', {'id': 11}), ('created_at', '2026-10-13T12:00:00Z')]
        original = deepcopy(self.run)
        for key, value in mutations:
            self.run = dict(original, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.receive()
        self.assertFalse(self.downloads)

    def test_failed_or_cancelled_producer_is_not_retried(self):
        for status, conclusion in [('completed', 'failure'), ('completed', 'cancelled'),
                                   ('completed', None), ('unknown', None), ('in_progress', 'failure')]:
            self.run.update(status=status, conclusion=conclusion)
            with self.subTest(status=status, conclusion=conclusion), self.assertRaises(ValueError):
                self.receive()
        self.assertFalse(self.downloads)
        self.assertFalse(self.sleeps)

    def test_exact_single_successful_job_required(self):
        original = deepcopy(self.job)
        for key, value in [('id', 999), ('name', 'Other job'), ('run_id', 999), ('run_attempt', 2),
                           ('head_sha', '0' * 40), ('status', 'in_progress'), ('conclusion', 'failure')]:
            self.job = dict(original, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.receive()
        self.job = original
        original_read = self.read_api
        for jobs in [[], [self.job, self.job]]:
            def read(endpoint, **limits):
                return dict(total_count=len(jobs), jobs=jobs) if '/jobs?' in endpoint else original_read(endpoint, **limits)
            with self.subTest(count=len(jobs)), self.assertRaises(ValueError):
                self.receive(read_api=read)
        self.assertFalse(self.downloads)

    def test_artifact_identity_expiry_size_and_attempt_interval_required(self):
        original = deepcopy(self.artifact)
        for key, value in [('id', 999), ('name', 'other'), ('expired', True),
                           ('digest', 'sha256:' + '0' * 64), ('size_in_bytes', self.source.artifact_bytes + 1),
                           ('workflow_run', {'id': 999, 'head_sha': self.source.commit}),
                           ('workflow_run', {'id': 700, 'head_sha': '0' * 40}),
                           ('created_at', '2026-10-13T12:00:59Z'), ('created_at', '2026-10-13T12:01:31Z')]:
            self.artifact = dict(original, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.receive()
        self.assertFalse(self.downloads)

    def test_runner_artifact_or_noncanonical_source_cannot_supply_operator_authority(self):
        for change in [dict(run_id=self.f.runner['runId']), dict(repository='other/Work'),
                       dict(actor_id='010'), dict(job_id='0'), dict(artifact_bytes=True),
                       dict(artifact_bytes=transport.MAX_ARCHIVE_BYTES + 1),
                       dict(artifact_name='name-only-approval'), dict(artifact_digest='0' * 64)]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.receive(source=replace(self.source, **change))
        self.assertFalse(self.calls)

    def test_original_zip_digest_and_length_are_checked_before_any_binding(self):
        for raw in [self.raw + b'!', self.raw[:-1], b'x' * len(self.raw)]:
            with self.subTest(length=len(raw)), self.assertRaises(ValueError):
                self.receive(download_archive=lambda *args, **kwargs: raw)

    def test_zip_crc_is_consumed_even_when_original_archive_digest_matches(self):
        corrupted = bytearray(self.raw)
        offset = self.raw.index(self.f.request_bytes)
        corrupted[offset] ^= 1
        self.set_archive(bytes(corrupted))
        with self.assertRaises(ValueError):
            self.receive()

    def test_zip_requires_exact_bounded_regular_member_allowlist(self):
        base = list(self.members.items())
        linked = zipfile.ZipInfo('request.json')
        linked.create_system = 3
        linked.external_attr = (stat.S_IFLNK | 0o777) << 16
        cases = [base + [('extra.json', b'{}')], base[:-1], base + [base[0]],
                 [('../request.json', base[0][1]), *base[1:]],
                 [('REQUEST.json', base[0][1]), *base[1:]],
                 [('request.json/', base[0][1]), *base[1:]],
                 [(linked, base[0][1]), *base[1:]],
                 [('request.json', b'x' * (transport.binding.MAX_RECORD_BYTES + 1)), *base[1:]]]
        for index, members in enumerate(cases):
            self.set_archive(self.make_archive(members))
            with self.subTest(case=index), self.assertRaises(ValueError):
                self.receive()

    def test_operator_request_must_be_the_exact_retained_pending_original(self):
        members = dict(self.members, **{'request.json': self.f.request_bytes + b'\n'})
        self.set_archive(self.make_archive(members.items()))
        with self.assertRaises(ValueError):
            self.receive()

    def test_root_authenticated_pins_still_cannot_bypass_original_byte_bindings(self):
        original = json.loads(self.members['approval-pins.json'])
        for key in ('requestSha256', 'approvalSha256', *transport.binding.PIN_FIELDS):
            pins = deepcopy(original)
            if key in pins:
                pins[key] = '0' * 64
            else:
                pins['pins'][key] = '0' * 64
            members = dict(self.members, **{'approval-pins.json': fixture_module.encode(pins)})
            self.set_archive(self.make_archive(members.items()))
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.receive()

    def test_pins_envelope_is_canonical_exact_and_bounded(self):
        value = json.loads(self.members['approval-pins.json'])
        for raw in [json.dumps(value).encode(), fixture_module.encode(dict(value, unknown=1)),
                    fixture_module.encode(dict(value, schema=True)),
                    fixture_module.encode(dict(value, purpose='other'))]:
            self.set_archive(self.make_archive(dict(self.members, **{'approval-pins.json': raw}).items()))
            with self.subTest(raw=raw[:30]), self.assertRaises(ValueError):
                self.receive()

    def test_original_approval_expiry_and_challenge_are_enforced_after_transport(self):
        for key, value in [('challenge', '0' * 64), ('expiresAt', '2026-10-13T12:02:00Z')]:
            approval = fixture_module.encode(dict(self.f.approval, **{key: value}))
            pins = json.loads(self.members['approval-pins.json'])
            pins['approvalSha256'] = transport.binding.sha256(approval)
            members = dict(self.members, **{'approval.json': approval, 'approval-pins.json': fixture_module.encode(pins)})
            self.set_archive(self.make_archive(members.items()))
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.receive()

    def test_timeout_request_expiry_and_cancellation_never_download(self):
        self.pending_reads = 100
        with self.assertRaises(ValueError):
            self.receive(timeout_seconds=6)
        self.assertEqual(self.sleeps, [5, 1])
        self.assertFalse(self.downloads)
        self.tick = 0
        with self.assertRaises(ValueError):
            self.receive(utcnow=lambda: self.f.now + timedelta(hours=1))
        self.cancelled = True
        with self.assertRaises(ValueError):
            self.receive()
        self.assertFalse(self.downloads)

    def test_io_callback_timeout_or_failure_is_not_retried(self):
        def failed(*args, **kwargs):
            raise TimeoutError('inert callback deadline')
        with self.assertRaises(TimeoutError):
            self.receive(read_api=failed)
        with self.assertRaises(TimeoutError):
            self.receive(download_archive=failed)
        self.assertFalse(self.sleeps)

    def test_cancel_or_elapsed_deadline_after_download_refuses_return(self):
        original = self.download
        for cancel in [True, False]:
            self.cancelled, self.tick = False, 0
            def download(*args, **kwargs):
                raw = original(*args, **kwargs)
                if cancel:
                    self.cancelled = True
                else:
                    self.tick = 31
                return raw
            with self.subTest(cancel=cancel), self.assertRaises(ValueError):
                self.receive(download_archive=download)

    def test_root_actor_is_rechecked_after_download(self):
        original = self.download
        def download(*args, **kwargs):
            raw = original(*args, **kwargs)
            self.run['triggering_actor']['id'] = 999
            return raw
        with self.assertRaises(ValueError):
            self.receive(download_archive=download)

    def test_cancellation_during_final_binding_does_not_return_approval(self):
        original = transport.binding.bind_approval
        def bind(**arguments):
            result = original(**arguments)
            self.cancelled = True
            return result
        with patch.object(transport.binding, 'bind_approval', side_effect=bind):
            with self.assertRaises(ValueError):
                self.receive()

    def test_api_response_byte_cap_is_checked_and_binding_inputs_are_owned(self):
        original = self.read_api
        with self.assertRaises(ValueError):
            self.receive(read_api=lambda *args, **kwargs: dict(self.run, padding='x' * transport.MAX_API_BYTES))
        def read(*args, **kwargs):
            self.arguments['expected_challenge'] = '0' * 64
            self.arguments['expected_setup']['bytes'] = 1
            return original(*args, **kwargs)
        result = self.receive(read_api=read)
        self.assertEqual(result.request.challenge, 'f' * 64)
        self.assertEqual(result.setup.size, 1234)


if __name__ == '__main__':
    unittest.main()
