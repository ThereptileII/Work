"""Inert host admission contracts; no API, Windows effect or approval grant."""
from copy import deepcopy
import unittest
from unittest.mock import patch

import installed_updater_host as host


class HostTests(unittest.TestCase):
    def setUp(self):
        self.target = dict(kind=host.MODE, runnerId='901', runnerGroupId='17',
            labels=['self-hosted', 'Windows', 'X64', 'skager-disposable-interactive'],
            clientSid='S-1-5-21-1-2-3-4')
        self.runner = dict(repository=host.REPOSITORY, headSha='a' * 40, runId='123',
                           runAttempt='2', jobId='456', jobName='Installed journey')
        self.workflow = '.github/workflows/installed-journey.yml'
        self.environment = dict(GITHUB_ACTIONS='true', RUNNER_OS='Windows', RUNNER_ENVIRONMENT='self-hosted',
            GITHUB_REPOSITORY=host.REPOSITORY, GITHUB_EVENT_NAME='workflow_dispatch',
            GITHUB_SHA='a' * 40, GITHUB_RUN_ID='123', GITHUB_RUN_ATTEMPT='2')
        self.run = dict(id=123, run_attempt=2, head_sha='a' * 40, path=self.workflow,
            event='workflow_dispatch', status='in_progress', conclusion=None,
            head_repository=dict(full_name=host.REPOSITORY))
        self.job = dict(id=456, name='Installed journey', status='in_progress', conclusion=None,
            run_id=123, run_attempt=2, head_sha='a' * 40, runner_id=901, runner_group_id=17,
            labels=deepcopy(self.target['labels']))
        self.capability = dict(state='available', normalTokenAvailable=True, consentPromptAvailable=True,
            nativeCustomerAcceptance=False, consentPromptAcceptance=False,
            observations=dict(current=dict(sid=self.target['clientSid'])))
        self.calls = []

    def read(self, endpoint, **kwargs):
        self.calls.append(endpoint)
        self.assertTrue(endpoint.startswith('repos/ThereptileII/Work/actions/runs/123/attempts/2'))
        self.assertEqual(kwargs, dict(timeout_seconds=30, max_bytes=256 << 10))
        return dict(total_count=1, jobs=[self.job]) if '/jobs?' in endpoint else self.run

    def check(self):
        with patch.object(host.customer, 'inspect_customer_capability', return_value=self.capability) as native:
            host.validate_live(self.target, self.runner, self.workflow, self.environment, 'win32', self.read)
        return native

    def test_exact_private_runner_and_native_consent_capability_are_admitted_without_acceptance(self):
        self.check().assert_called_once()
        self.assertEqual(len(self.calls), 2)
        self.assertIs(self.capability['nativeCustomerAcceptance'], False)
        self.assertIs(self.capability['consentPromptAcceptance'], False)

    def test_wrong_missing_changed_runner_group_labels_and_job_tuple_refuse(self):
        for key, value in [('runner_id', 902), ('runner_id', None), ('runner_id', True),
            ('runner_group_id', 18), ('runner_group_id', None), ('labels', []),
            ('labels', self.target['labels'] + ['foreign']), ('labels', ['windows-2022']),
            ('id', 457), ('name', 'Other job'), ('run_id', 124), ('run_attempt', 1),
            ('head_sha', 'b' * 40), ('status', 'completed'), ('conclusion', 'failure')]:
            original = deepcopy(self.job)
            self.job[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError): self.check()
            self.job = original

    def test_stale_completed_foreign_or_unreviewed_run_refuses(self):
        for key, value in [('id', 124), ('run_attempt', 1), ('head_sha', 'b' * 40),
            ('path', '.github/workflows/other.yml'), ('event', 'push'), ('status', 'completed'),
            ('conclusion', 'success'), ('head_repository', dict(full_name='foreign/repo'))]:
            original = deepcopy(self.run); self.run[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): self.check()
            self.run = original

    def test_environment_spoof_is_never_identity_authority(self):
        self.environment.update(RUNNER_NAME='approved-private-host', SKAGER_APPROVED_RUNNER_ID='901')
        self.job['runner_id'] = 902
        with self.assertRaises(ValueError): self.check()
        self.job['runner_id'] = 901
        for key, value in [('RUNNER_ENVIRONMENT', 'github-hosted'), ('GITHUB_RUN_ID', '124'),
                           ('GITHUB_ACTIONS', 'false'), ('GITHUB_REPOSITORY', 'foreign/repo')]:
            original = dict(self.environment); self.environment[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): self.check()
            self.environment = original

    def test_intended_sid_normal_tokens_and_real_prompt_policy_are_required(self):
        for key, value in [('state', 'blocked'), ('normalTokenAvailable', False),
            ('consentPromptAvailable', False), ('nativeCustomerAcceptance', True),
            ('consentPromptAcceptance', True), ('observations', dict(current=dict(sid='S-1-5-21-9-8-7-6')))]:
            original = deepcopy(self.capability); self.capability[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError): self.check()
            self.capability = original

    def test_closed_target_fields_and_schema_keep_legacy_hosted_only(self):
        legacy = {key: None for key in host.DECLARATION_KEYS}; legacy['schema'] = 1
        self.assertIsNone(host.declaration(legacy))
        current = dict(legacy, schema=2, hostAdmission=self.target)
        self.assertEqual(host.declaration(current), self.target)
        for changed in (dict(legacy, hostAdmission=self.target), dict(current, schema=True),
                        dict(current, schema=3), dict(current, unknown=True)):
            with self.subTest(changed=changed), self.assertRaises(ValueError): host.declaration(changed)
        for field, value in [('runnerId', '0'), ('runnerId', 901), ('runnerGroupId', '01'),
            ('clientSid', 'S-1-5-18'), ('labels', self.target['labels'] * 2), ('kind', 'localhost')]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                host.target(dict(self.target, **{field:value}))


if __name__ == '__main__':
    unittest.main()
