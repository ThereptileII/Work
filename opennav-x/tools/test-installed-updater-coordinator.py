"""Inert live-composition tests; no approval, network, Windows or product effects."""
from copy import deepcopy
from datetime import timedelta
import importlib.util
from pathlib import Path
import sys
import tempfile
from dataclasses import replace
import subprocess
import os
import json
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import installed_updater_coordinator as coordinator

spec = importlib.util.spec_from_file_location('transport_fixture', Path(__file__).with_name('test-installed-updater-approval-transport.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
originals_check = coordinator._originals
original_reader = coordinator._original_reader


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture.TransportTests()
        self.f.setUp()
        self.reads = coordinator.http.GitHubReads('inert-token')
        self.reads.read_api, self.reads.download_archive = self.f.read_api, self.f.download
        self.owner = Mock(sid='original-sid')
        self.owner.actual_sid.return_value = 'original-sid'
        self.owner.profile.return_value = {'original': 'profile'}
        self.consumer = coordinator.NativeConsumption.__new__(coordinator.NativeConsumption)
        self.consumer.pin, self.consumer.consume = Mock(), Mock()
        self.consumer.check, self.consumer.close = Mock(), Mock()
        self.initial = {'request': {'clientSid': 'original-sid'}}
        self.calls = []
        self.cancelled = False
        self.now = self.f.f.now
        self.native = patch.object(coordinator.journey.admission, 'BeforeEnrollment', return_value=self.owner)
        self.originals = patch.object(coordinator, '_originals', side_effect=lambda *a: self.calls.append('originals'))
        self.profile = patch.object(coordinator.journey.phase, 'profile_preserved', return_value=True)
        self.run = patch.object(coordinator.journey, 'run_joined', return_value={'state': 'observed'})
        for p in (self.native, self.originals, self.profile, self.run):
            p.start()
            self.addCleanup(p.stop)

    def reset_scenario(self):
        self.f.setUp()
        self.owner.reset_mock(side_effect=True)
        self.owner.actual_sid.return_value = 'original-sid'
        self.owner.profile.return_value = {'original': 'profile'}
        self.consumer.pin.reset_mock(side_effect=True)
        self.consumer.consume.reset_mock(side_effect=True)
        coordinator._originals.reset_mock(side_effect=True)
        coordinator._originals.side_effect = lambda *a: self.calls.append('originals')
        coordinator.journey.run_joined.reset_mock(side_effect=True)
        coordinator.journey.run_joined.return_value = {'state': 'observed'}
        coordinator.journey.phase.profile_preserved.return_value = True
        self.calls, self.cancelled, self.now = [], False, self.f.f.now

    def make(self, **options):
        return coordinator.Coordinator(source=self.f.source, bind_arguments=self.f.arguments,
            initial=self.initial, phases={}, output=Path('/unused/result.json'),
            handoff=lambda *a: None, consumer=self.consumer, reads=self.reads,
            cancelled=lambda: self.cancelled, utcnow=lambda: self.now, **options)

    def external(self):
        f = self.f.f
        target = dict(kind='interactive-self-hosted', runnerId='901', runnerGroupId='17',
            labels=['self-hosted','Windows','X64','skager-disposable-interactive'], clientSid=f.client['clientSid'])
        declared = dict(f.declaration, schema=2, hostAdmission=target)
        request = f.prepare_originals(declaration=declared)
        approval = fixture.fixture_module.encode(dict(f.approval, requestSha256=request.request_sha256))
        self.f.arguments.update(request_bytes=request.original_bytes, declaration=json.dumps(declared, indent=2).encode())
        self.f.members = {'request.json':request.original_bytes, 'approval.json':approval,
            'approval-pins.json':fixture.fixture_module.encode(dict(schema=1, purpose=coordinator.transport.PINS_PURPOSE,
                requestSha256=request.request_sha256, approvalSha256=coordinator.binding.sha256(approval), pins=dict(request.pins)))}
        self.f.source = replace(self.f.source, artifact_name=coordinator.transport.artifact_name('700','1',request.request_sha256))
        self.f.artifact['name'] = self.f.source.artifact_name
        self.f.set_archive(self.f.make_archive())
        self.initial = dict(request=dict(clientSid=target['clientSid']), hostAdmission=target)
        self.owner.sid = target['clientSid']; self.owner.actual_sid.return_value = target['clientSid']
        environment = dict(GITHUB_ACTIONS='true', RUNNER_OS='Windows', RUNNER_ARCH='X64', RUNNER_ENVIRONMENT='self-hosted',
            GITHUB_REPOSITORY='ThereptileII/Work', GITHUB_EVENT_NAME='workflow_dispatch', GITHUB_SHA=f.runner['headSha'],
            GITHUB_RUN_ID=f.runner['runId'], GITHUB_RUN_ATTEMPT=f.runner['runAttempt'])
        workflow = '.github/workflows/installed-journey.yml'
        run = dict(id=123, run_attempt=2, head_sha=f.runner['headSha'], path=workflow, event='workflow_dispatch',
            status='in_progress', conclusion=None, head_repository=dict(full_name='ThereptileII/Work'))
        job = dict(id=456, name=f.runner['jobName'], run_id=123, run_attempt=2, head_sha=f.runner['headSha'],
            status='in_progress', conclusion=None, runner_id=901, runner_group_id=17, labels=target['labels'])
        def read(endpoint, **kwargs):
            if '/runs/123/attempts/2' in endpoint:
                return dict(total_count=1,jobs=[job]) if '/jobs?' in endpoint else run
            return self.f.read_api(endpoint, **kwargs)
        self.reads.read_api = read
        capability = dict(state='available',normalTokenAvailable=True,consentPromptAvailable=True,
            nativeCustomerAcceptance=False,consentPromptAcceptance=False,observations=dict(current=dict(sid=target['clientSid'])))
        for p in (patch.dict(os.environ, environment, clear=True), patch.object(coordinator.sys,'platform','win32'),
                  patch.object(coordinator.host_admission.customer,'inspect_customer_capability',return_value=capability)):
            p.start(); self.addCleanup(p.stop)
        return workflow, job

    def test_schema2_preserves_real_approval_and_single_consumption_after_three_host_checks(self):
        workflow, job = self.external()
        self.assertEqual(self.make(pending_workflow=workflow).run(), {'state':'observed'})
        coordinator.host_admission.customer.inspect_customer_capability.assert_called()
        self.assertEqual(coordinator.host_admission.customer.inspect_customer_capability.call_count, 3)
        self.consumer.consume.assert_called_once()
        coordinator.journey.run_joined.assert_called_once()

    def test_bad_external_host_has_no_receipt_child_owner_or_install_effect(self):
        workflow, job = self.external(); job['runner_id'] = 902
        with self.assertRaises(ValueError): self.make(pending_workflow=workflow).run()
        coordinator.journey.admission.BeforeEnrollment.assert_not_called()
        self.consumer.pin.assert_not_called()
        self.consumer.consume.assert_not_called()
        coordinator.journey.run_joined.assert_not_called()

    def test_changed_external_runner_during_approval_never_consumes(self):
        workflow, job = self.external()
        actual = coordinator.host_admission.validate_live
        calls = []
        def check(*args):
            calls.append(None)
            if len(calls) == 2: job['runner_id'] = 902
            return actual(*args)
        with patch.object(coordinator.host_admission,'validate_live',side_effect=check), self.assertRaises(ValueError):
            self.make(pending_workflow=workflow).run()
        self.consumer.pin.assert_called_once()
        self.consumer.consume.assert_not_called()
        coordinator.journey.run_joined.assert_not_called()

    def test_real_transport_binding_reaches_exact_one_consumption_and_join(self):
        c = self.make()
        self.assertEqual(c.run(), {'state': 'observed'})
        self.assertEqual(len(self.f.downloads), 1)
        record = self.consumer.consume.call_args.args[0]
        self.assertEqual(record['requestSha256'], coordinator.binding.sha256(self.f.f.request_bytes))
        self.assertEqual(record['approvalSha256'], coordinator.binding.sha256(self.f.f.approval_bytes))
        self.assertEqual(record['challenge'], 'f' * 64)
        self.assertEqual(record['runId'], self.f.f.runner['runId'])
        self.assertEqual(self.calls, ['originals', 'originals'])
        args = coordinator.journey.run_joined.call_args
        self.assertEqual(args.args[2], dict(request=self.f.f.pins['clientRequestSha256'],
            policy=self.f.f.pins['bootstrapPolicySha256'], provisioner='c' * 64, broker='d' * 64, timestamp='e' * 64))
        self.owner.close.assert_called_once()
        with self.assertRaises(ValueError):
            c.run()
        self.assertEqual(self.consumer.consume.call_count, 1)
        coordinator.journey.run_joined.assert_called_once()

    def test_unapproved_wrong_context_or_expired_records_never_consume(self):
        for kind in ('challenge', 'runner', 'expiry', 'actor', 'digest'):
            with self.subTest(kind=kind):
                self.reset_scenario()
                c = self.make()
                if kind == 'challenge': c.arguments['expected_challenge'] = '0' * 64
                if kind == 'runner': c.arguments['expected_runner']['runId'] = '999'
                if kind == 'expiry': self.now += timedelta(hours=2)
                if kind == 'actor': self.f.run['actor']['id'] = 999
                if kind == 'digest': self.f.raw += b'x'
                with self.assertRaises(ValueError): c.run()
                self.consumer.consume.assert_not_called()
                coordinator.journey.run_joined.assert_not_called()
                self.owner.close.assert_called_once()

    def test_changed_account_profile_or_originals_refuse_before_consumption(self):
        for kind in ('account', 'profile', 'original'):
            with self.subTest(kind=kind):
                self.reset_scenario()
                if kind == 'account': self.owner.actual_sid.return_value = 'foreign'
                if kind == 'profile': coordinator.journey.phase.profile_preserved.return_value = False
                if kind == 'original': coordinator._originals.side_effect = ValueError('changed')
                with self.assertRaises(ValueError): self.make().run()
                self.consumer.consume.assert_not_called()
                coordinator.journey.run_joined.assert_not_called()

    def test_uncertain_consumption_permanently_invalidates_attempt(self):
        self.consumer.consume.side_effect = ValueError('uncertain')
        c = self.make()
        for _ in range(2):
            with self.assertRaises(ValueError): c.run()
        self.consumer.consume.assert_called_once()
        coordinator.journey.run_joined.assert_not_called()
        self.owner.close.assert_called_once()

    def test_cancel_or_expiry_after_consumption_prevents_journey_and_replay(self):
        for kind in ('cancel', 'expiry'):
            with self.subTest(kind=kind):
                self.reset_scenario()
                def consumed(_):
                    if kind == 'cancel': self.cancelled = True
                    else: self.now += timedelta(hours=2)
                self.consumer.consume.side_effect = consumed
                c = self.make()
                for _ in range(2):
                    with self.assertRaises(ValueError): c.run()
                self.consumer.consume.assert_called_once()
                coordinator.journey.run_joined.assert_not_called()

    def test_journey_or_owner_cleanup_failure_is_not_retried(self):
        for kind in ('journey', 'cleanup'):
            with self.subTest(kind=kind):
                self.reset_scenario()
                if kind == 'journey': coordinator.journey.run_joined.side_effect = ValueError('partial')
                else: self.owner.close.side_effect = ValueError('cleanup')
                c = self.make()
                for _ in range(2):
                    with self.assertRaises(ValueError): c.run()
                self.consumer.consume.assert_called_once()
                coordinator.journey.run_joined.assert_called_once()

    def deferred(self):
        c = self.make()
        c.source = coordinator.rendezvous.TrustedStatusLocator('88', '9' * 40, '10', '10')
        request = coordinator.binding._request(c.arguments['request_bytes'])
        status = dict(id=900, context=coordinator.rendezvous.status_context(request), state='success',
            creator=dict(id=88), created_at='2026-10-13T12:01:40Z', updated_at='2026-10-13T12:01:40Z',
            target_url=coordinator.rendezvous.source_target_url(self.f.source))
        old = self.reads.read_api
        self.status_calls = 0
        def read(endpoint, **kwargs):
            self.assertTrue(self.consumer.pin.called, 'source resolved before native READY')
            if '/statuses?' in endpoint:
                self.status_calls += 1
                return [deepcopy(status)]
            return old(endpoint, **kwargs)
        self.reads.read_api = read
        return c, status

    def test_deferred_locator_resolves_inside_ready_owner_and_rechecks_before_consume(self):
        c, _ = self.deferred()
        def consume(_):
            self.assertEqual(self.status_calls, 2)
            self.owner.close.assert_not_called()
            self.consumer.close.assert_not_called()
        self.consumer.consume.side_effect = consume
        self.assertEqual(c.run(), {'state': 'observed'})
        self.assertEqual(self.status_calls, 2)
        self.owner.close.assert_called_once()
        self.consumer.close.assert_called_once()

    def test_changed_status_after_authenticated_download_prevents_consumption(self):
        c, status = self.deferred()
        old = self.reads.download_archive
        def download(*args, **kwargs):
            result = old(*args, **kwargs)
            status['id'] = 901
            return result
        self.reads.download_archive = download
        with self.assertRaises(ValueError): c.run()
        self.consumer.consume.assert_not_called()
        coordinator.journey.run_joined.assert_not_called()
        self.owner.close.assert_called_once()
        self.consumer.close.assert_called_once()

    def test_locator_time_counts_against_same_overall_receiver_budget(self):
        c, _ = self.deferred()
        tick = [0]
        old = self.reads.read_api
        def read(*args, **kwargs):
            result = old(*args, **kwargs)
            if '/statuses?' in args[0]: tick[0] = 300
            return result
        self.reads.read_api = read
        original = coordinator.transport.receive_approval
        with patch.object(coordinator.time, 'monotonic', side_effect=lambda: tick[0]), \
             patch.object(coordinator.transport, 'receive_approval', wraps=original) as receive:
            c.run()
            self.assertEqual(receive.call_args.kwargs['timeout_seconds'], 3300)

    def test_approval_expiry_during_status_recheck_prevents_native_effect(self):
        c, _ = self.deferred()
        old = self.reads.read_api
        def read(*args, **kwargs):
            result = old(*args, **kwargs)
            if self.status_calls == 2:
                self.now += timedelta(minutes=30)
            return result
        self.reads.read_api = read
        with self.assertRaises(ValueError): c.run()
        self.consumer.consume.assert_not_called()
        coordinator.journey.run_joined.assert_not_called()

    def test_original_native_owner_loss_during_locator_refuses_before_download(self):
        c, _ = self.deferred()
        self.consumer.check.side_effect = ValueError('original child lost')
        with self.assertRaises(ValueError): c.run()
        self.assertEqual(self.f.downloads, [])
        self.consumer.consume.assert_not_called()
        self.owner.close.assert_called_once()


class OriginalBytesTests(unittest.TestCase):
    def test_exact_approved_originals_and_changed_file_refusal(self):
        f = fixture.TransportTests()
        f.setUp()
        approval = f.receive()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            declaration, client, policy, setup = [root / n for n in ('declaration', 'client', 'policy', 'setup')]
            for path, raw in ((declaration, approval.original_declaration_bytes),
                              (client, approval.original_client_request_bytes),
                              (policy, approval.original_release_policy_bytes), (setup, b'setup')):
                path.write_bytes(raw)
            approval = replace(approval, setup=replace(approval.setup, path=str(setup), size=5))
            initial = dict(retained=[(declaration, None, dict(approval.request.pins)['declarationSha256'])],
                requestPath=client, b0=dict(paths=dict(policy=policy)), setup=setup,
                setupSha256=approval.setup.sha256,
                brokerPolicy=dict(provisioner=dict(sha256='c' * 64), broker=dict(sha256='d' * 64)))
            with patch.object(coordinator.journey, '_same_inputs'), \
                 patch.object(coordinator.journey.admission, 'unchanged'), \
                 patch.object(coordinator.journey.phase, 'unchanged'):
                originals_check(approval, initial, {})
                for path in (declaration, client, policy, setup):
                    with self.subTest(changed=path.name):
                        raw = path.read_bytes()
                        path.write_bytes(raw + b'changed')
                        with self.assertRaises(ValueError): originals_check(approval, initial, {})
                        path.write_bytes(raw)


class NativeConsumerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.parent = self.root / 'protected-parent'
        self.parent.mkdir()
        self.helper = self.root / 'approved-native.exe'
        self.helper.write_bytes(b'inert')
        self.owner = Mock()
        self.record = dict(schema=1, purpose='installed-updater-b0-consumption',
            repository='ThereptileII/Work', headSha='a' * 40, runId='1', runAttempt='1',
            jobId='2', jobName='Installed updater', challenge='b' * 64,
            requestSha256='c' * 64, approvalSha256='d' * 64)
        identity = dict(volumeSerial='1', fileID='1' * 32, creationTime='2', descriptorSha256='e' * 64)
        self.ready = dict(schema=1, state='ready', parent=identity)
        self.receipt = dict(schema=1, state='consumed',
            recordSha256=coordinator.binding.sha256(coordinator.binding._encode(self.record)),
            parent=identity, record=dict(identity, fileID='2' * 32))
        self.process = Mock()
        self.process.poll.return_value = None
        self.process.stdin.closed = False
        self.process.stdin.close.side_effect = lambda: setattr(self.process.stdin, 'closed', True)
        self.process.wait.side_effect = self.finish
        self.consumer = coordinator.NativeConsumption(self.helper, 'a' * 64,
            self.parent, self.root / 'receipt.json')
        self.addCleanup(self.consumer.close)

    def finish(self, **kwargs):
        if not self.process.stdin.closed:
            return 1
        if self.process.stdin.write.called:
            self.stream.write(coordinator.binding._encode(self.receipt))
        self.process.poll.return_value = 0
        return 0

    def popen(self, command, **options):
        self.command, self.options = command, options
        self.stream = options['stdout']
        self.stream.write(coordinator.binding._encode(self.ready))
        return self.process

    def reader(self, stream):
        if os.name == 'nt':
            return original_reader(stream)
        # Inert Linux fixture only: a same-object fd reference gives independent
        # cursors. Production accepts Windows ReOpenFile, never this fallback.
        return open('/proc/self/fd/' + str(stream.fileno()), 'rb', buffering=0)

    def start(self):
        with patch.object(coordinator.subprocess, 'Popen', side_effect=self.popen), \
             patch.object(coordinator, '_original_reader', side_effect=self.reader):
            self.consumer.pin(self.owner)

    def test_fixed_command_ready_exact_stdin_original_receipt_and_drain(self):
        self.start()
        self.assertEqual(self.consumer.parent_native, self.ready['parent'])
        self.assertEqual(self.command, [str(self.helper), '--consume-installed-b0-challenge=' + str(self.parent)])
        self.owner.pin_image.assert_called_once_with(self.helper, 'a' * 64)
        self.consumer.consume(self.record)
        self.process.stdin.write.assert_called_once_with(coordinator.binding._encode(self.record))
        self.process.wait.assert_called_once_with(timeout=60)
        with self.assertRaises(ValueError): self.consumer.consume(self.record)
        self.consumer.close()
        self.process.kill.assert_not_called()
        self.process.terminate.assert_not_called()

    def test_receipt_path_replacement_cannot_substitute_original_stream(self):
        self.start()
        original = self.consumer.receipt.with_name('moved-receipt')
        try:
            self.consumer.receipt.rename(original)
        except PermissionError:
            return  # Native handle denies replacement: equivalent safety.
        self.consumer.receipt.write_bytes(b'foreign pathname bytes')
        self.consumer.consume(self.record)
        self.assertEqual(self.consumer.receipt.read_bytes(), b'foreign pathname bytes')
        self.assertIn(b'consumed', original.read_bytes())

    def test_parent_ready_identity_mismatch_refuses_consumed_receipt(self):
        self.start()
        self.receipt = dict(self.receipt, parent=dict(self.ready['parent'], creationTime='99'))
        with self.assertRaises(ValueError): self.consumer.consume(self.record)

    def test_zero_or_foreign_native_id_refuses(self):
        self.start()
        self.receipt = dict(self.receipt, record=dict(self.receipt['record'], fileID='0' * 32))
        with self.assertRaises(ValueError): self.consumer.consume(self.record)

    def test_existing_receipt_never_launches(self):
        self.consumer.receipt.write_bytes(b'prior')
        with patch.object(coordinator.subprocess, 'Popen') as popen:
            with self.assertRaises(FileExistsError): self.consumer.pin(self.owner)
            popen.assert_not_called()

    def test_eof_before_record_drains_without_consuming(self):
        self.start()
        self.consumer.close()
        self.process.stdin.write.assert_not_called()
        self.process.stdin.close.assert_called_once()
        self.process.wait.assert_called_once_with()

    def test_timeout_drain_preserves_failure_without_kill_or_late_success(self):
        self.start()
        self.process.wait.side_effect = [subprocess.TimeoutExpired('original', 60), 0]
        with self.assertRaises(subprocess.TimeoutExpired): self.consumer.consume(self.record)
        with self.assertRaises(ValueError): self.consumer.consume(self.record)
        self.consumer.close()
        self.assertEqual(self.process.wait.call_count, 2)
        self.process.kill.assert_not_called()
        self.process.terminate.assert_not_called()

    def test_stdin_close_failure_still_drains_original_child_before_observer_close(self):
        self.start()
        self.process.stdin.close.side_effect = BrokenPipeError('original child')
        with self.assertRaises(BrokenPipeError): self.consumer.close()
        self.process.wait.assert_called_once_with()
        self.assertTrue(self.consumer.reader.closed)
        self.assertTrue(self.consumer.output.closed)

    def test_unexpected_wait_error_keeps_owners_until_verified_exit_and_refuses(self):
        self.start()
        self.process.wait.side_effect = [OSError('unobserved exit'), 0]
        with patch.object(coordinator.time, 'sleep'):
            with self.assertRaises(OSError): self.consumer.close()
        self.assertEqual(self.process.wait.call_count, 2)
        self.assertTrue(self.consumer.reader.closed)
        self.process.kill.assert_not_called()

    def test_parent_path_replacement_while_ready_refuses_before_record(self):
        self.start()
        displaced = self.parent.with_name('displaced-parent')
        try:
            self.parent.rename(displaced)
        except PermissionError:
            return  # Real native READY ancestry would deny this replacement.
        self.parent.mkdir()
        with self.assertRaises(ValueError): self.consumer.consume(self.record)
        self.process.stdin.write.assert_not_called()

    def test_extra_record_framing_refuses(self):
        self.start()
        def finish(**kwargs):
            self.finish(**kwargs)
            self.stream.write(b'extra\n')
            return 0
        self.process.wait.side_effect = finish
        with self.assertRaises(ValueError): self.consumer.consume(self.record)


if __name__ == '__main__':
    unittest.main()
