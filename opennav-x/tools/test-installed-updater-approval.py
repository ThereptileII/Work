"""Inert byte-contract checks; no Windows, network, key or approval is used."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import installed_updater_approval as handoff


def encode(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True) + '\n').encode()


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.runner = dict(repository=handoff.REPOSITORY, headSha='a' * 40, runId='123',
                           runAttempt='2', jobId='456', jobName='Installed protected updater')
        self.artifact = dict(repository=handoff.REPOSITORY, headSha='b' * 40, runId='120',
                             runAttempt='1', jobId='450', artifactId='800',
                             artifactName='staging-release-original-attempt1',
                             artifactDigest='sha256:' + 'c' * 64, artifactBytes=12345)
        self.kit = dict(self.artifact, jobId='451', artifactId='801',
                        artifactName='protected-operator-kit-original', artifactBytes=2345)
        self.client = dict(schema=1, purpose='client-registration', clientSid='S-1-5-21-1-2-3-4',
                           installed=dict(version='0.4.0-beta2.4', commit='b' * 40, generation='d' * 32,
                                          packageSha256='1' * 64, productBuildSha256='2' * 64,
                                          executableSha256='3' * 64, inventorySha256='4' * 64),
                           stock=dict(path=r'C:\stock\opencpn.exe', version='5.12.4', arch='x86',
                                      executableSha256='5' * 64, upstreamCommit='e' * 40),
                           trustConfiguration={'schema': 1}, trustConfigSha256='6' * 64)
        self.client_bytes = json.dumps(self.client, indent=2).encode()  # Original bytes are not canonicalized.
        file = lambda name, digest: dict(path='C:\\approved\\' + name, sha256=digest)
        self.setup = dict(path=r'C:\approved\SKAGER-Beta2-Setup.exe', sha256='0' * 64, bytes=1234)
        self.policy = dict(schema=1, product='skager', channel='beta', version='0.4.0-beta2.4', commit='b' * 40,
                           installer=dict(path='artifacts/' + 'b' * 40 + '/SKAGER-Beta2-Setup.exe',
                                          url='https://approved.invalid/Setup.exe', sha256=self.setup['sha256'], bytes=1234),
                           releaseNotes={}, recovery={}, correspondingSource={}, notices={}, supportedOpenCpn=[])
        self.policy_bytes = json.dumps(self.policy, indent=2).encode()
        self.declaration = dict(schema=1, setup=dict(path=self.setup['path'], sha256=self.setup['sha256']), b0=dict(version='0.4.0-beta2.4', commit='b' * 40,
                                                recovery=file('b0.zip', '7' * 64), source=file('source.zip', '8' * 64),
                                                policy=file('beta.json', handoff.sha256(self.policy_bytes))),
                                operatorKit=file('kit.zip', 'a' * 64),
                                clientRequest=file('client.json', handoff.sha256(self.client_bytes)),
                                brokerBootstrapPolicy=file('broker-beta.json', 'b' * 64),
                                approvedBaseline=file('approved-installed-baseline.zip', '7' * 64),
                                trustConfig=file('trust.json', '6' * 64), bootstrapCache=r'C:\ProtectedCache',
                                artifactDirectory=r'C:\ProtectedAcquisition',
                                preserveSettings=[dict(section='Settings', key='StableKey')])
        self.declaration_bytes = json.dumps(self.declaration, indent=2).encode()
        self.pins = dict(clientRequestSha256=handoff.sha256(self.client_bytes),
                         declarationSha256=handoff.sha256(self.declaration_bytes), bootstrapPolicySha256='b' * 64,
                         provisionerSha256='c' * 64, brokerSha256='d' * 64, timestampSha256='e' * 64)
        self.request = dict(schema=1, purpose=handoff.REQUEST_PURPOSE, runner=self.runner,
                            challenge='f' * 64, createdAt='2026-10-13T12:00:00Z', expiresAt='2026-10-13T13:00:00Z',
                            candidate=dict(artifact=self.artifact, releaseManifestSha256='1' * 64, qualificationSha256='2' * 64),
                            operatorKit=dict(artifact=self.kit, sha256='a' * 64),
                            b0=dict(version='0.4.0-beta2.4', commit='b' * 40, recoverySha256='7' * 64,
                                    sourceSha256='8' * 64, policySha256=handoff.sha256(self.policy_bytes), trustSha256='6' * 64), pins=self.pins)
        self.request_bytes = encode(self.request)
        self.approval = dict(schema=1, purpose=handoff.APPROVAL_PURPOSE, requestSha256=handoff.sha256(self.request_bytes),
                             runner=self.runner, challenge='f' * 64, issuedAt='2026-10-13T12:01:00Z',
                             expiresAt='2026-10-13T12:20:00Z')
        self.approval_bytes = encode(self.approval)
        self.now = datetime(2026, 10, 13, 12, 2, tzinfo=timezone.utc)

    def inputs(self):
        return dict(request_bytes=self.request_bytes, approval_bytes=self.approval_bytes,
                    client_request=self.client_bytes, declaration=self.declaration_bytes,
                    release_policy=self.policy_bytes, expected_setup=deepcopy(self.setup),
                    approved_request_sha256=handoff.sha256(self.request_bytes),
                    approved_approval_sha256=handoff.sha256(self.approval_bytes), approved_pins=deepcopy(self.pins),
                    expected_runner=deepcopy(self.runner), expected_challenge='f' * 64,
                    expected_candidate_artifact=deepcopy(self.artifact), expected_operator_kit_artifact=deepcopy(self.kit),
                    now=self.now)

    def bind(self, **changes):
        inputs = self.inputs()
        inputs.update(changes)
        return handoff.bind_approval(**inputs)

    def rejected_request(self, changed):
        raw = encode(changed)
        # Match the changed envelope's new hash in both independent fixture pins
        # and approval. Refusal must come from the attacked boundary, not merely
        # from a stale approval pointing at the old request hash.
        approval = encode(dict(self.approval, requestSha256=handoff.sha256(raw)))
        with self.assertRaises(ValueError):
            self.bind(request_bytes=raw, approved_request_sha256=handoff.sha256(raw),
                      approval_bytes=approval, approved_approval_sha256=handoff.sha256(approval))

    def rejected_approval(self, changed):
        raw = encode(changed)
        with self.assertRaises(ValueError):
            self.bind(approval_bytes=raw, approved_approval_sha256=handoff.sha256(raw))

    def test_exact_originals_bind_and_owned_return_is_deeply_immutable(self):
        result = self.bind()
        self.assertEqual(result.original_client_request_bytes, self.client_bytes)
        self.assertEqual(result.original_declaration_bytes, self.declaration_bytes)
        self.assertEqual(result.original_release_policy_bytes, self.policy_bytes)
        self.assertEqual((result.setup.path, result.setup.sha256, result.setup.size),
                         (self.setup['path'], self.setup['sha256'], self.setup['bytes']))
        with self.assertRaises(FrozenInstanceError):
            result.setup.size = 99
        self.assertEqual(result.original_approval_bytes, self.approval_bytes)
        self.assertEqual(result.request.original_bytes, self.request_bytes)
        self.assertEqual(result.request.runner.job_id, '456')
        with self.assertRaises(FrozenInstanceError):
            result.request.runner.job_id = '999'
        with self.assertRaises(TypeError):
            result.request.pins[0] = ('clientRequestSha256', '0' * 64)

    def prepare_originals(self, declaration=None, policy=None):
        # Re-pin inert changed originals to exercise semantic binding, rather
        # than getting a refusal only because an earlier hash became stale.
        declared = deepcopy(self.declaration if declaration is None else declaration)
        policy_bytes = json.dumps(self.policy if policy is None else policy, indent=2).encode()
        declared['b0']['policy']['sha256'] = handoff.sha256(policy_bytes)
        declaration_bytes = json.dumps(declared, indent=2).encode()
        request = deepcopy(self.request)
        request['b0']['policySha256'] = handoff.sha256(policy_bytes)
        request['pins']['declarationSha256'] = handoff.sha256(declaration_bytes)
        return handoff.prepare_request(request, client_request=self.client_bytes, declaration=declaration_bytes,
                                       release_policy=policy_bytes, expected_setup=self.setup)

    def test_setup_first_declaration_requires_exact_original_setup_record(self):
        self.assertEqual(self.prepare_originals().policy_sha256, handoff.sha256(self.policy_bytes))
        changes = [None, {}, {'path': self.setup['path']},
                   dict(self.declaration['setup'], unknown=1),
                   dict(self.declaration['setup'], path=r'C:\approved\other.exe'),
                   dict(self.declaration['setup'], sha256='f' * 64)]
        for setup in changes:
            declaration = deepcopy(self.declaration)
            if setup is None:
                del declaration['setup']  # Former declaration schema must refuse.
            else:
                declaration['setup'] = setup
            with self.subTest(setup=setup), self.assertRaises(ValueError):
                self.prepare_originals(declaration=declaration)

    def test_independent_setup_metadata_must_match_policy_size_hash_and_declared_path(self):
        for field, value in (('path', r'C:\approved\other.exe'), ('path', 'bad\npath'),
                             ('sha256', 'f' * 64), ('bytes', 1233), ('bytes', True),
                             ('bytes', 0), ('bytes', (1 << 30) + 1)):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.bind(expected_setup=dict(self.setup, **{field: value}))
        for record in ({}, dict(self.setup, unknown=1)):
            with self.subTest(record=record), self.assertRaises(ValueError):
                self.bind(expected_setup=record)

    def test_policy_installer_binding_refuses_changed_original_and_wrong_fixed_target(self):
        with self.assertRaises(ValueError):
            self.bind(release_policy=self.policy_bytes + b'\n')
        for key, value in (('path', 'artifacts/' + 'b' * 40 + '/other.exe'),
                           ('path', 'artifacts/' + 'a' * 40 + '/SKAGER-Beta2-Setup.exe'),
                           ('sha256', 'f' * 64), ('bytes', 1233), ('bytes', True),
                           ('bytes', 0), ('bytes', (1 << 30) + 1)):
            policy = deepcopy(self.policy)
            policy['installer'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.prepare_originals(policy=policy)
        policy = deepcopy(self.policy)
        policy['installer']['unknown'] = 1
        with self.assertRaises(ValueError):
            self.prepare_originals(policy=policy)
        for key, value in (('schema', True), ('product', 'other'), ('channel', 'stable'),
                           ('version', '0.4.0-beta2.5'), ('commit', 'a' * 40)):
            policy = dict(self.policy, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.prepare_originals(policy=policy)

    def test_setup_metadata_is_owned_and_file_io_is_not_performed(self):
        with patch('builtins.open', side_effect=AssertionError('unexpected file access')):
            bound = self.bind()
        self.setup['path'] = r'C:\changed-after-binding.exe'
        self.setup['sha256'] = 'f' * 64
        self.setup['bytes'] = 1
        self.assertEqual(bound.setup, handoff.Setup(r'C:\approved\SKAGER-Beta2-Setup.exe', '0' * 64, 1234))

    def test_prepare_returns_review_data_without_approval_and_owns_nested_inputs(self):
        result = handoff.prepare_request(self.request, client_request=self.client_bytes, declaration=self.declaration_bytes,
                                         release_policy=self.policy_bytes, expected_setup=self.setup)
        self.runner['jobId'] = '999'
        self.pins['brokerSha256'] = '0' * 64
        self.assertEqual(result.runner.job_id, '456')
        self.assertEqual(dict(result.pins)['brokerSha256'], 'd' * 64)
        self.assertFalse(hasattr(result, 'approved'))
        self.assertFalse(hasattr(result, 'nativeProductAcceptance'))

    def test_no_self_hash_can_replace_separate_original_and_approval_pins(self):
        for name in ('approved_request_sha256', 'approved_approval_sha256'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.bind(**{name: '0' * 64})
        for name in handoff.PIN_FIELDS:
            pins = deepcopy(self.pins)
            pins[name] = '0' * 64
            with self.subTest(pin=name), self.assertRaises(ValueError):
                self.bind(approved_pins=pins)

    def test_tampered_originals_refuse_without_reserializing(self):
        for name, raw in (('client_request', self.client_bytes), ('declaration', self.declaration_bytes)):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.bind(**{name: raw + b'\n'})

    def test_wrong_live_run_attempt_job_commit_or_repository_refuses(self):
        for key, value in (('repository', 'attacker/Work'), ('headSha', '0' * 40), ('runId', '124'),
                           ('runAttempt', '3'), ('jobId', '457'), ('jobName', 'Other job')):
            expected = dict(self.runner, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.bind(expected_runner=expected)

    def test_previous_job_nonce_refuses(self):
        with self.assertRaises(ValueError):
            self.bind(expected_challenge='0' * 64)
        changed = deepcopy(self.approval)
        changed['challenge'] = '0' * 64
        self.rejected_approval(changed)

    def test_each_artifact_provenance_field_is_bound_to_external_observation(self):
        for parameter, original in (('expected_candidate_artifact', self.artifact), ('expected_operator_kit_artifact', self.kit)):
            for key, value in (('repository', 'other/Work'), ('headSha', '0' * 40), ('runId', '999'),
                               ('runAttempt', '2'), ('jobId', '999'), ('artifactId', '999'),
                               ('artifactName', 'other'), ('artifactDigest', 'sha256:' + '0' * 64), ('artifactBytes', 1)):
                with self.subTest(parameter=parameter, key=key), self.assertRaises(ValueError):
                    self.bind(**{parameter: dict(original, **{key: value})})

    def test_approval_cannot_target_another_request_or_runner(self):
        changed = deepcopy(self.approval)
        changed['requestSha256'] = '0' * 64
        self.rejected_approval(changed)
        for key in ('runId', 'runAttempt', 'jobId'):
            changed = deepcopy(self.approval)
            changed['runner'][key] = '999'
            self.rejected_approval(changed)

    def test_request_expiry_and_future_creation(self):
        for now in (datetime(2026, 10, 13, 11, 59, tzinfo=timezone.utc),
                    datetime(2026, 10, 13, 13, 0, tzinfo=timezone.utc)):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.bind(now=now)
        for expiry in ('2026-10-13T12:00:00Z', '2026-10-13T14:00:01Z'):
            changed = deepcopy(self.request)
            changed['expiresAt'] = expiry
            self.rejected_request(changed)

    def test_approval_must_follow_production_challenge_and_fit_both_expiries(self):
        for issued, expires in (('2026-10-13T12:00:00Z', '2026-10-13T12:20:00Z'),
                                ('2026-10-13T11:59:59Z', '2026-10-13T12:20:00Z'),
                                ('2026-10-13T12:03:00Z', '2026-10-13T12:20:00Z'),
                                ('2026-10-13T12:01:00Z', '2026-10-13T12:31:01Z'),
                                ('2026-10-13T12:01:00Z', '2026-10-13T13:00:01Z'),
                                ('2026-10-13T12:01:00Z', '2026-10-13T12:02:00Z')):
            changed = dict(self.approval, issuedAt=issued, expiresAt=expires)
            with self.subTest(issued=issued, expires=expires):
                self.rejected_approval(changed)

    def test_clock_requires_aware_utc(self):
        for now in (self.now.replace(tzinfo=None), self.now.astimezone(timezone(timedelta(hours=2))), 'now'):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.bind(now=now)

    def test_noncanonical_whitespace_order_bom_and_trailing_bytes_refuse(self):
        for parameter, value in (('request_bytes', self.request), ('approval_bytes', self.approval)):
            for raw in (json.dumps(value).encode(), encode(value).rstrip(b'\n'),
                        b'\xef\xbb\xbf' + encode(value), encode(value) + b'\n', encode(value) + b'{}'):
                changes = {parameter: raw, 'approved_' + ('request' if parameter == 'request_bytes' else 'approval') + '_sha256': handoff.sha256(raw)}
                with self.subTest(parameter=parameter, raw=raw[:20]), self.assertRaises(ValueError):
                    self.bind(**changes)

    def test_duplicate_keys_at_root_and_nested_levels_refuse_even_with_new_pin(self):
        for parameter, raw in (('request_bytes', self.request_bytes), ('approval_bytes', self.approval_bytes)):
            for altered in (raw.replace(b'{', b'{"schema":1,', 1),
                            raw.replace(b'"jobId":"456"', b'"jobId":"456","jobId":"456"')):
                changes = {parameter: altered, 'approved_' + ('request' if parameter == 'request_bytes' else 'approval') + '_sha256': handoff.sha256(altered)}
                with self.subTest(parameter=parameter), self.assertRaises(ValueError):
                    self.bind(**changes)

    def test_unknown_missing_bool_float_and_numeric_alias_fields_refuse(self):
        for original, reject in ((self.request, self.rejected_request), (self.approval, self.rejected_approval)):
            for field in original:
                changed = deepcopy(original)
                del changed[field]
                with self.subTest(missing=field):
                    reject(changed)
            for value in (True, 1.0, None, '1'):
                with self.subTest(schema=value):
                    reject(dict(original, schema=value))
            reject(dict(original, approved=True))
        for key in ('runId', 'runAttempt', 'jobId'):
            for value in ('01', '0', '-1', 1, True, '1' * 21):
                changed = deepcopy(self.request)
                changed['runner'][key] = value
                with self.subTest(key=key, value=value):
                    self.rejected_request(changed)

    def test_unknown_nested_request_records_refuse(self):
        for path in (('runner',), ('candidate',), ('candidate', 'artifact'), ('operatorKit',),
                     ('operatorKit', 'artifact'), ('b0',), ('pins',)):
            changed = deepcopy(self.request)
            current = changed
            for key in path:
                current = current[key]
            current['unknown'] = 1
            with self.subTest(path=path):
                self.rejected_request(changed)

    def test_digest_case_length_algorithm_and_challenge_bounds(self):
        for value in ('F' * 64, 'f' * 63, 'f' * 65, 'sha256:' + 'f' * 64, None):
            self.rejected_request(dict(self.request, challenge=value))
        for value in ('sha512:' + 'f' * 64, 'sha256:' + 'F' * 64, 'f' * 64):
            changed = deepcopy(self.request)
            changed['operatorKit']['artifact']['artifactDigest'] = value
            self.rejected_request(changed)

    def test_oversized_empty_deep_and_invalid_json_refuse(self):
        for raw in (b'', b' ' * (handoff.MAX_RECORD_BYTES + 1), b'[' * 1500 + b']' * 1500,
                    b'{"schema":NaN}\n', b'{"schema":Infinity}\n', b'\xff', b'[]\n'):
            with self.subTest(raw=raw[:20]), self.assertRaises(ValueError):
                self.bind(approval_bytes=raw, approved_approval_sha256=handoff.sha256(raw))
        for value in (True, 0, -1, (16 << 30) + 1):
            changed = deepcopy(self.request)
            changed['candidate']['artifact']['artifactBytes'] = value
            self.rejected_request(changed)

    def test_unbounded_originals_and_envelopes_refuse_before_hashing(self):
        for parameter, limit in (('request_bytes', handoff.MAX_RECORD_BYTES),
                                 ('approval_bytes', handoff.MAX_RECORD_BYTES)):
            with self.subTest(parameter=parameter), patch.object(handoff, 'sha256', side_effect=AssertionError('unexpected hash')):
                with self.assertRaises(ValueError):
                    inputs = self.inputs_without_hashes()
                    inputs[parameter] = b'x' * (limit + 1)
                    handoff.bind_approval(**inputs)
        prepared = handoff.prepare_request(self.request, client_request=self.client_bytes, declaration=self.declaration_bytes,
                                         release_policy=self.policy_bytes, expected_setup=self.setup)
        for parameter, limit in (('client_request', 1 << 20), ('declaration', 64 << 10), ('release_policy', 32 << 10)):
            inputs = dict(client_request=self.client_bytes, declaration=self.declaration_bytes,
                                         release_policy=self.policy_bytes, expected_setup=self.setup)
            inputs[parameter] = b'x' * (limit + 1)
            with self.subTest(parameter=parameter), patch.object(handoff, 'sha256', side_effect=AssertionError('unexpected hash')):
                with self.assertRaises(ValueError):
                    handoff._originals(prepared, **inputs)

    def inputs_without_hashes(self):
        # Known fixture pins avoid accidentally hashing in the test itself.
        return dict(request_bytes=self.request_bytes, approval_bytes=self.approval_bytes,
                    client_request=self.client_bytes, declaration=self.declaration_bytes,
                    release_policy=self.policy_bytes, expected_setup=deepcopy(self.setup),
                    approved_request_sha256='0' * 64, approved_approval_sha256='0' * 64,
                    approved_pins=self.pins, expected_runner=self.runner, expected_challenge='f' * 64,
                    expected_candidate_artifact=self.artifact, expected_operator_kit_artifact=self.kit, now=self.now)

    def test_mixed_product_commits_reused_artifact_id_and_unsupported_version_refuse(self):
        for target in ('candidate', 'operatorKit'):
            changed = deepcopy(self.request)
            changed[target]['artifact']['headSha'] = '0' * 40
            self.rejected_request(changed)
        changed = deepcopy(self.request)
        changed['operatorKit']['artifact']['artifactId'] = self.artifact['artifactId']
        self.rejected_request(changed)
        changed = deepcopy(self.request)
        changed['b0']['version'] = '0.4.0-beta2.04'
        self.rejected_request(changed)

    def test_original_declaration_mismatches_cannot_be_hidden_by_new_local_hash(self):
        for key in ('recovery', 'source', 'policy'):
            changed = deepcopy(self.declaration)
            changed['b0'][key]['sha256'] = '0' * 64
            raw = encode(changed)
            request = deepcopy(self.request)
            request['pins']['declarationSha256'] = handoff.sha256(raw)
            with self.subTest(key=key), self.assertRaises(ValueError):
                handoff.prepare_request(request, client_request=self.client_bytes, declaration=raw,
                                        release_policy=self.policy_bytes, expected_setup=self.setup)

    def test_duplicate_or_unknown_originals_refuse_after_pin_matching(self):
        for raw in (self.client_bytes.replace(b'{', b'{"schema":1,', 1),
                    encode(dict(self.client, unknown=1))):
            declaration = deepcopy(self.declaration)
            declaration['clientRequest']['sha256'] = handoff.sha256(raw)
            declaration_raw = encode(declaration)
            request = deepcopy(self.request)
            request['pins']['clientRequestSha256'] = handoff.sha256(raw)
            request['pins']['declarationSha256'] = handoff.sha256(declaration_raw)
            with self.assertRaises(ValueError):
                handoff.prepare_request(request, client_request=raw, declaration=declaration_raw,
                                        release_policy=self.policy_bytes, expected_setup=self.setup)


if __name__ == '__main__':
    unittest.main()
