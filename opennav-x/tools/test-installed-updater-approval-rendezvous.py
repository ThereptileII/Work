"""Inert authenticated-channel fixtures; no status writes/network/approval grant."""
from copy import deepcopy
from dataclasses import replace
from datetime import timedelta
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import installed_updater_approval_rendezvous as rendezvous

spec = importlib.util.spec_from_file_location('transport_fixture', Path(__file__).with_name('test-installed-updater-approval-transport.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class RendezvousTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture.TransportTests()
        self.f.setUp()
        self.request = rendezvous.binding._request(self.f.arguments['request_bytes'])
        self.policy = rendezvous.TrustedStatusLocator('88', '9' * 40, '10', '10')
        self.status = dict(id=900, context=rendezvous.status_context(self.request), state='success',
            creator=dict(id=88), created_at='2026-10-13T12:01:40Z', updated_at='2026-10-13T12:01:40Z',
            target_url=rendezvous.source_target_url(self.f.source))
        self.rows = [self.status]
        self.tick, self.cancelled, self.calls, self.checks = 0, False, [], 0

    def read(self, endpoint, *, timeout_seconds, max_bytes):
        self.assertGreater(timeout_seconds, 0)
        self.assertLessEqual(timeout_seconds, 30)
        self.assertEqual(max_bytes, rendezvous.transport.MAX_API_BYTES)
        self.calls.append(endpoint)
        return deepcopy(self.rows)

    def check(self):
        self.checks += 1

    def sleep(self, value):
        self.tick += value

    def resolve(self, **changes):
        args = dict(locator=self.policy, request=self.request, read_api=self.read,
                    timeout_seconds=30, poll_seconds=5, cancelled=lambda: self.cancelled,
                    monotonic=lambda: self.tick, sleep=self.sleep,
                    utcnow=lambda: self.f.f.now + timedelta(seconds=self.tick), check=self.check)
        args.update(changes)
        return rendezvous.resolve_source(**args)

    def test_exact_fixed_locator_owns_descriptor_and_rechecks_original_status(self):
        result = self.resolve()
        self.assertEqual(result.source, self.f.source)
        self.assertEqual(self.calls, [f'repos/ThereptileII/Work/commits/{self.request.runner.head_sha}/statuses?per_page=100&page=1'])
        self.assertGreater(self.checks, 0)
        self.assertEqual(self.resolve(expected=result), result)
        self.assertLessEqual(len(self.status['context']), 100)
        self.assertLessEqual(len(self.status['target_url']), 2048)

    def test_fixed_context_changes_for_job_attempt_challenge_and_original_request(self):
        original = rendezvous.status_context(self.request)
        for request in (replace(self.request, challenge='0' * 64),
                        replace(self.request, request_sha256='0' * 64),
                        replace(self.request, runner=replace(self.request.runner, run_attempt='99')),
                        replace(self.request, runner=replace(self.request.runner, job_id='99'))):
            self.assertNotEqual(rendezvous.status_context(request), original)

    def test_foreign_creator_wrong_state_age_case_or_missing_fields_refuse(self):
        mutations = dict(creator=lambda s: s.update(creator=dict(id=99)),
            state=lambda s: s.update(state='pending'), old=lambda s: s.update(created_at='2026-10-13T11:59:00Z'),
            future=lambda s: s.update(created_at='2026-10-13T12:03:00Z', updated_at='2026-10-13T12:03:00Z'),
            edited=lambda s: s.update(updated_at='2026-10-13T12:01:41Z'),
            case=lambda s: s.update(context=s['context'].upper()),
            missing=lambda s: s.pop('target_url'), boolID=lambda s: s.update(id=True))
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                self.rows = [deepcopy(self.status)]
                mutate(self.rows[0])
                with self.assertRaises(ValueError): self.resolve()

    def test_descriptor_cannot_change_trusted_source_or_infer_artifact_pins(self):
        for field, value in (('commit', '8' * 40), ('actor_id', '99'),
                             ('triggering_actor_id', '99'), ('repository', 'other/repo'),
                             ('artifact_digest', 'invalid'), ('run_id', self.request.runner.run_id)):
            with self.subTest(field=field):
                changed = replace(self.f.source, **{field: value})
                self.rows = [dict(self.status, target_url=rendezvous.source_target_url(changed))]
                with self.assertRaises(ValueError): self.resolve()

    def test_url_is_data_only_and_malformed_truncated_or_foreign_urls_refuse(self):
        for url in ('https://evil.invalid/a', self.status['target_url'] + '=',
                    self.status['target_url'].replace('/700#', '/701#'),
                    self.status['target_url'][:-3], 'x' * 2049):
            with self.subTest(url=url[:60]):
                self.rows = [dict(self.status, target_url=url)]
                with self.assertRaises(ValueError): self.resolve()
        self.assertTrue(all('/statuses?' in endpoint for endpoint in self.calls))

    def test_duplicate_conflicting_or_foreign_matching_entries_refuse(self):
        for other in (deepcopy(self.status), dict(self.status, id=901),
                      dict(self.status, creator=dict(id=99))):
            with self.subTest(other=other['id']):
                self.rows = [self.status, other]
                with self.assertRaises(ValueError): self.resolve()

    def test_complete_pagination_required_before_accepting_match(self):
        filler = [dict(context='unrelated')] * 99
        def read(endpoint, **kwargs):
            self.calls.append(endpoint)
            return [self.status, *filler] if endpoint.endswith('&page=1') else []
        self.assertEqual(self.resolve(read_api=read).source, self.f.source)
        self.assertEqual(len(self.calls), 2)
        def partial(endpoint, **kwargs):
            if endpoint.endswith('&page=1'): return [self.status, *filler]
            raise ValueError('page unavailable')
        with self.assertRaises(ValueError): self.resolve(read_api=partial)
        with self.assertRaises(ValueError): self.resolve(read_api=lambda *a, **k: [dict(context='other')] * 100)

    def test_missing_locator_waits_only_within_deadline_cancel_and_original_expiry(self):
        self.rows = []
        with self.assertRaises(ValueError): self.resolve(timeout_seconds=10)
        self.assertEqual(self.tick, 10)
        self.cancelled = True
        with self.assertRaises(ValueError): self.resolve()
        self.cancelled = False
        self.tick = 7200
        with self.assertRaises(ValueError): self.resolve()

    def test_recheck_refuses_missing_changed_or_conflicting_attestation_without_wait(self):
        result = self.resolve()
        for rows in ([], [dict(self.status, id=901)], [self.status, dict(self.status, id=901)]):
            with self.subTest(rows=len(rows)):
                self.rows = rows
                before = self.tick
                with self.assertRaises(ValueError): self.resolve(expected=result)
                self.assertEqual(before, self.tick)


if __name__ == '__main__':
    unittest.main()
