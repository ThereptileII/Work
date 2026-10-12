"""Offline refusal contracts for portable read-only AIS attribution evidence."""
import copy
from datetime import datetime, timezone
import importlib.util
import ipaddress
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('ais_connection_binding', ROOT / 'tools/ais_connection_binding.py')
binding = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(binding)
NOW = '2026-10-12T12:00:05Z'


def address(text):
    ip = ipaddress.ip_address(text)
    return (ip.packed + (bytes(12) if ip.version == 4 else b'')).hex()


def fixture(ipv6=False):
    family = 'ipv6' if ipv6 else 'ipv4'
    local = {'address': address('2001:4860:1::2' if ipv6 else '192.168.1.12'),
             'port': 50321, 'scope': 0}
    remote = {'address': address('2606:4700::1111' if ipv6 else '1.1.1.1'),
              'port': 443, 'scope': 0}
    observation = {'schema': 1, 'source': 'AISSTREAM_ONLINE', 'available': True,
                   'reason': 'observed', 'process_id': 42,
                   'observed_utc': '2026-10-12T12:00:01Z', 'family': family,
                   'generation': '1', 'captured_monotonic_ns': '1000000000',
                   'observed_monotonic_ns': '2000000000', 'age_ms': '1000',
                   'local': local, 'remote': remote}
    expected = {'schema': 1, 'image_path': r'C:\Program Files\SKAGER\SKAGER.exe',
                'image_sha256': 'a' * 64,
                'modules': [{'path': r'C:\Windows\System32\ws2_32.dll', 'sha256': 'b' * 64},
                            {'path': r'C:\Program Files\SKAGER\curl.dll', 'sha256': 'c' * 64}]}
    process = {'pid': 42, 'creation_filetime': '1234567890',
               'image_path': expected['image_path'], 'image_sha256': expected['image_sha256'],
               'modules': copy.deepcopy(expected['modules']),
               'same_image_processes': [{'pid': 42, 'creation_filetime': '1234567890'}]}
    tcp = {'pid': 42, 'state': 'ESTABLISHED', 'family': family,
           'local': copy.deepcopy(local), 'remote': copy.deepcopy(remote)}
    dns = {'host': 'stream.aisstream.io',
           'addresses': [{'family': family, 'address': remote['address'], 'scope': 0}]}
    evidence = {'schema': 1, 'purpose': 'read-only-ais-attribution',
                'started_utc': '2026-10-12T12:00:00Z', 'finished_utc': '2026-10-12T12:00:03Z',
                'observation_before': observation, 'observation_after': copy.deepcopy(observation),
                'process_before': process, 'process_after': copy.deepcopy(process),
                'tcp_before': [tcp], 'tcp_after': [copy.deepcopy(tcp)],
                'dns_before': dns, 'dns_after': copy.deepcopy(dns)}
    evidence['observation_after'].update(observed_utc='2026-10-12T12:00:02Z',
                                          observed_monotonic_ns='3000000000', age_ms='2000')
    return evidence, expected


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.evidence, self.expected = fixture()

    def validate(self, now=NOW):
        return binding.validate(self.evidence, self.expected, now)

    def refused(self, now=NOW):
        with self.assertRaises(binding.EvidenceError) as caught:
            self.validate(now)
        self.assertEqual(str(caught.exception), binding.ERROR)

    def test_valid_ipv4_and_ipv6_are_only_observations(self):
        for ipv6 in (False, True):
            self.evidence, self.expected = fixture(ipv6)
            self.assertEqual(self.validate(), {'schema': 1, 'state': 'OBSERVATION_MATCHED',
                                              'authorizes_filter': False})

    def test_explicit_utc_datetime_is_supported(self):
        self.assertFalse(self.validate(datetime(2026, 10, 12, 12, 0, 5,
                                               tzinfo=timezone.utc))['authorizes_filter'])
        self.refused(datetime(2026, 10, 12, 12, 0, 5))

    def test_missing_or_unavailable_observation_is_refused(self):
        for key in ('observation_before', 'observation_after'):
            for missing in (True, False):
                with self.subTest(key=key, missing=missing):
                    self.evidence, self.expected = fixture()
                    if missing:
                        del self.evidence[key]
                    else:
                        self.evidence[key] = {'schema': 1, 'source': 'AISSTREAM_ONLINE',
                            'available': False, 'reason': 'not-connected', 'process_id': 42,
                            'observed_utc': '2026-10-12T12:00:01Z'}
                    self.refused()

    def test_stale_future_and_collection_deadline(self):
        self.refused('2026-10-12T12:00:12Z')
        self.refused('2026-10-12T12:00:02Z')
        self.evidence['started_utc'] = '2026-10-12T11:59:40Z'
        self.refused()

    def test_timestamp_order_and_format(self):
        for field, value in [('started_utc', '2026-10-12T12:00:04Z'),
                             ('finished_utc', '2026-10-12T12:00:01Z'),
                             ('started_utc', '2026-10-12T12:00:00+00:00'),
                             ('started_utc', '2026-02-30T12:00:00Z')]:
            with self.subTest(field=field, value=value):
                self.evidence, self.expected = fixture()
                self.evidence[field] = value
                self.refused()
        self.evidence, self.expected = fixture()
        self.evidence['observation_after']['observed_utc'] = '2026-10-12T12:00:00Z'
        self.refused()

    def test_connection_capture_can_be_old_without_refresh(self):
        for row in ('observation_before', 'observation_after'):
            self.evidence[row]['captured_monotonic_ns'] = '1'
            self.evidence[row]['observed_monotonic_ns'] = str(86400 * 1000000000 +
                (1000000000 if row.endswith('before') else 2000000000))
            self.evidence[row]['age_ms'] = str((int(self.evidence[row]['observed_monotonic_ns']) - 1) // 1000000)
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')

    def test_exact_freshness_boundary_floor_age_and_same_second_reads(self):
        self.assertEqual(self.validate('2026-10-12T12:00:11Z')['state'], 'OBSERVATION_MATCHED')
        self.evidence['observation_after']['observed_utc'] = self.evidence['observation_before']['observed_utc']
        self.evidence['observation_before']['observed_monotonic_ns'] = '2000999999'
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')
        self.evidence['observation_before']['observed_monotonic_ns'] = '1000000000'
        self.evidence['observation_before']['age_ms'] = '0'
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')

    def test_capture_refresh_generation_tuple_or_pid_drift(self):
        mutations = [('generation', '2'), ('captured_monotonic_ns', '2000000000'),
                     ('process_id', 43), ('family', 'ipv6'),
                     ('observed_monotonic_ns', '2000000000')]
        for field, value in mutations:
            with self.subTest(field=field):
                self.evidence, self.expected = fixture()
                self.evidence['observation_after'][field] = value
                if field == 'captured_monotonic_ns':
                    self.evidence['observation_after']['age_ms'] = '1000'
                if field == 'observed_monotonic_ns':
                    self.evidence['observation_after']['age_ms'] = '1000'
                self.refused()
        self.evidence, self.expected = fixture()
        self.evidence['observation_after']['local']['port'] += 1
        self.refused()

    def test_monotonic_age_must_be_exact_canonical_decimal(self):
        for field, value in [('age_ms', '1001'), ('age_ms', '01000'), ('age_ms', 1000),
                             ('generation', '0'), ('generation', True),
                             ('captured_monotonic_ns', '3000000000'),
                             ('observed_monotonic_ns', '-1'),
                             ('observed_monotonic_ns', str(1 << 64))]:
            with self.subTest(field=field, value=value):
                self.evidence, self.expected = fixture()
                self.evidence['observation_before'][field] = value
                self.refused()

    def test_wrong_pid_reused_pid_and_ambiguous_process(self):
        for where, field, value in [('process_before', 'pid', 43),
                                    ('process_after', 'creation_filetime', '1234567891')]:
            with self.subTest(where=where, field=field):
                self.evidence, self.expected = fixture()
                self.evidence[where][field] = value
                self.refused()
        for peers in ([], [{'pid': 43, 'creation_filetime': '1234567890'}],
                      [{'pid': 42, 'creation_filetime': '1234567890'},
                       {'pid': 43, 'creation_filetime': '1234567891'}]):
            self.evidence, self.expected = fixture()
            self.evidence['process_before']['same_image_processes'] = peers
            self.refused()

    def test_tcp_missing_wrong_pid_duplicate_and_competing_owner(self):
        for rows in ([], 'wrong-pid', 'duplicate', 'competing-owner'):
            with self.subTest(rows=rows):
                self.evidence, self.expected = fixture()
                if rows == []:
                    self.evidence['tcp_before'] = []
                elif rows == 'wrong-pid':
                    self.evidence['tcp_before'][0]['pid'] = 43
                else:
                    self.evidence['tcp_before'].append(copy.deepcopy(self.evidence['tcp_before'][0]))
                    if rows == 'competing-owner':
                        self.evidence['tcp_before'][1]['pid'] = 43
                self.refused()

    def test_other_established_connections_are_permitted(self):
        row = copy.deepcopy(self.evidence['tcp_before'][0])
        row['local']['port'] += 1
        row['remote'].update(address=address('192.168.1.14'), port=22)
        self.evidence['tcp_before'].append(row)
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')

    def test_module_hash_path_inventory_and_image_changes(self):
        for change in ('image', 'module-hash', 'module-path', 'module-missing', 'pin'):
            with self.subTest(change=change):
                self.evidence, self.expected = fixture()
                process = self.evidence['process_after']
                if change == 'image':
                    process['image_sha256'] = 'd' * 64
                elif change == 'module-hash':
                    process['modules'][0]['sha256'] = 'd' * 64
                elif change == 'module-path':
                    process['modules'][0]['path'] = r'C:\Other\ws2_32.dll'
                elif change == 'module-missing':
                    process['modules'].pop()
                else:
                    self.expected['image_sha256'] = 'd' * 64
                self.refused()

    def test_module_inventory_order_is_not_identity(self):
        self.evidence['process_after']['modules'].reverse()
        self.expected['modules'].reverse()
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')

    def test_casefold_duplicates_and_noncanonical_paths(self):
        self.expected['modules'].append({'path': self.expected['modules'][0]['path'].upper(),
                                         'sha256': 'b' * 64})
        self.refused()
        for path in (r'\\server\share\app.exe', r'C:app.exe', r'C:\app.exe:secret',
                     r'C:\folder\..\app.exe', r'C:\folder.\app.exe',
                     r'C:\PROGRA~1\app.exe', r'C:\NUL.txt', r'C:\\app.exe'):
            with self.subTest(path=path):
                self.evidence, self.expected = fixture()
                self.expected['image_path'] = path
                self.refused()

    def test_dns_change_missing_remote_duplicate_and_scope(self):
        for change in ('change', 'missing', 'duplicate', 'scope', 'host'):
            with self.subTest(change=change):
                self.evidence, self.expected = fixture()
                dns = self.evidence['dns_after']
                if change == 'change':
                    dns['addresses'].append({'family': 'ipv4', 'address': address('8.8.8.8'), 'scope': 0})
                elif change == 'missing':
                    for key in ('dns_before', 'dns_after'):
                        self.evidence[key]['addresses'][0]['address'] = address('8.8.8.8')
                elif change == 'duplicate':
                    dns['addresses'].append(copy.deepcopy(dns['addresses'][0]))
                elif change == 'scope':
                    dns['addresses'][0]['scope'] = 1
                else:
                    dns['host'] = 'other.example'
                self.refused()

    def test_public_remote_address_and_port_required(self):
        for ip in ('127.0.0.1', '192.168.1.1', '169.254.1.1', '0.0.0.0', '224.0.0.1', '255.255.255.255'):
            with self.subTest(ip=ip):
                self.evidence, self.expected = fixture()
                self.evidence['observation_before']['remote']['address'] = address(ip)
                self.refused()
        self.evidence, self.expected = fixture()
        self.evidence['observation_before']['remote']['port'] = 80
        self.refused()

    def test_ipv6_scope_mapped_and_link_local_rules(self):
        self.evidence, self.expected = fixture(True)
        self.evidence['observation_before']['local'].update(address=address('fe80::1'), scope=0)
        self.refused()
        for ip in ('::1', '::', '::ffff:1.1.1.1', 'ff02::1', 'fe80::1', 'fc00::1'):
            with self.subTest(ip=ip):
                self.evidence, self.expected = fixture(True)
                self.evidence['observation_before']['remote']['address'] = address(ip)
                self.refused()
        self.evidence, self.expected = fixture(True)
        for key in ('observation_before', 'observation_after'):
            self.evidence[key]['local'].update(address=address('fe80::1'), scope=3)
        for key in ('tcp_before', 'tcp_after'):
            self.evidence[key][0]['local'].update(address=address('fe80::1'), scope=3)
        self.assertEqual(self.validate()['state'], 'OBSERVATION_MATCHED')

    def test_boolean_numerics_unknown_keys_and_credentials_refused(self):
        for target, field in [('root', 'schema'), ('observation_before', 'schema'),
                              ('observation_before', 'process_id'), ('process_before', 'pid')]:
            with self.subTest(target=target, field=field):
                self.evidence, self.expected = fixture()
                row = self.evidence if target == 'root' else self.evidence[target]
                row[field] = True
                self.refused()
        self.evidence, self.expected = fixture()
        for row in (self.evidence, self.expected, self.evidence['dns_before'],
                    self.evidence['tcp_before'][0], self.evidence['observation_before']['local']):
            row['api_key'] = 'DO-NOT-LEAK-super-secret'
            self.refused()
            del row['api_key']

    def test_direct_api_resource_limits_and_cycles(self):
        for count in (binding.MAX_MODULES + 1, binding.MAX_TCP_ROWS + 1):
            self.evidence, self.expected = fixture()
            if count == binding.MAX_MODULES + 1:
                self.expected['modules'] *= count
            else:
                self.evidence['tcp_before'] *= count
            self.refused()
        self.evidence, self.expected = fixture()
        self.evidence['secret'] = 'x' * (binding.MAX_STRING + 1)
        self.refused()
        self.evidence, self.expected = fixture()
        self.evidence['cycle'] = self.evidence
        self.refused()


class JsonAndCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / 'input.json'

    def test_strict_json_refusals(self):
        cases = ['{"schema":1,"schema":1}', '{"secret":NaN}', '{"schema":1.0}',
                 '{"schema":-1}', '{"schema":18446744073709551616}',
                 '[' * 1000 + '0' + ']' * 1000, '{"secret":"' + 'x' * 1025 + '"}',
                 '{"secret":' + '1' * 5000 + '}', '{"secret":"\\ud800"}']
        for source in cases:
            with self.subTest(source=source[:40]):
                self.path.write_text(source, encoding='utf-8')
                with self.assertRaises(binding.EvidenceError):
                    binding.read_json(self.path)
        self.path.write_bytes(b'\xff')
        with self.assertRaises(binding.EvidenceError):
            binding.read_json(self.path)

    def test_diagnostic_float_mode_is_finite_and_not_evidence_permission(self):
        raw = b'{"latitude":-20.5,"offset":-4,"nested":{"valid":true}}'
        with self.assertRaises(binding.EvidenceError):
            binding.strict_json_loads(raw)
        decoded = binding.strict_json_loads(raw, allow_floats=True)
        self.assertEqual(decoded['latitude'], -20.5)
        for raw in (b'{"n":NaN}', b'{"n":Infinity}', b'{"n":1e999}',
                    b'{"n":1,"n":2}', b'{"n":-18446744073709551616}'):
            with self.assertRaises(binding.EvidenceError):
                binding.strict_json_loads(raw, allow_floats=True)
        evidence, expected = fixture()
        evidence['schema'] = 1.0
        with self.assertRaises(binding.EvidenceError):
            binding.validate(evidence, expected, NOW)

    def test_expected_preflight_refuses_unreviewable_shapes(self):
        _, expected = fixture()
        self.assertIs(binding.validate_expected(expected), expected)
        for field, value in (('schema', True), ('modules', []),
                             ('image_path', r'C:\PROGRA~1\SKAGER.exe'),
                             ('image_sha256', 'a' * 63)):
            bad = copy.deepcopy(expected)
            bad[field] = value
            with self.assertRaises(binding.EvidenceError):
                binding.validate_expected(bad)

    def test_image_module_entry_must_have_same_spelling_and_digest(self):
        _, expected = fixture()
        expected['modules'].append({'path': expected['image_path'],
                                    'sha256': expected['image_sha256']})
        binding.validate_expected(expected)
        expected['modules'][-1]['sha256'] = 'd' * 64
        with self.assertRaises(binding.EvidenceError):
            binding.validate_expected(expected)
        expected['modules'][-1] = {'path': expected['image_path'].upper(),
                                    'sha256': expected['image_sha256']}
        with self.assertRaises(binding.EvidenceError):
            binding.validate_expected(expected)

    def test_bounds_apply_to_dns_modules_and_endpoint_types(self):
        evidence, expected = fixture()
        evidence['dns_before']['addresses'] *= binding.MAX_DNS_ADDRESSES + 1
        with self.assertRaises(binding.EvidenceError):
            binding.validate(evidence, expected, NOW)
        for field, value in (('scope', True), ('scope', -1), ('scope', 1 << 32),
                             ('port', True), ('port', 65536), ('port', 0),
                             ('address', '01010101' + '0' * 23 + '1'),
                             ('address', 'G' * 32)):
            evidence, expected = fixture()
            evidence['observation_before']['local'][field] = value
            with self.assertRaises(binding.EvidenceError):
                binding.validate(evidence, expected, NOW)

    def test_read_is_bounded_before_decoding(self):
        self.path.write_bytes(b' ' * 33)
        with self.assertRaises(binding.EvidenceError):
            binding.read_json(self.path, limit=32)

    def test_valid_json_round_trip(self):
        evidence, expected = fixture()
        self.path.write_text(json.dumps(evidence), encoding='utf-8')
        self.assertEqual(binding.validate(binding.read_json(self.path), expected, NOW)['state'],
                         'OBSERVATION_MATCHED')

    def test_cli_success_uses_safe_bounded_status(self):
        evidence, expected = fixture()
        evidence_path = self.root / 'evidence.json'
        expected_path = self.root / 'expected.json'
        evidence_path.write_text(json.dumps(evidence), encoding='utf-8')
        expected_path.write_text(json.dumps(expected), encoding='utf-8')
        with patch.object(binding, 'datetime') as date, patch('builtins.print') as output:
            date.now.return_value = datetime(2026, 10, 12, 12, 0, 5, tzinfo=timezone.utc)
            date.strptime.side_effect = datetime.strptime
            # String now avoids replacing the datetime type check in validate.
            with patch.object(binding, '_now', return_value=date.now.return_value):
                self.assertEqual(binding.main(['--evidence', str(evidence_path),
                                                '--expected', str(expected_path)]), 0)
        self.assertEqual(json.loads(output.call_args.args[0]),
                         {'schema': 1, 'state': 'OBSERVATION_MATCHED', 'authorizes_filter': False})

    def test_cli_error_never_echoes_credentials_or_input_paths(self):
        secret = 'DO-NOT-LEAK-super-secret'
        self.path.write_text(json.dumps({'api_key': secret}), encoding='utf-8')
        for arguments in (['--api-key', secret],
                          ['--evidence', str(self.path), '--expected', str(self.path)],
                          ['--evidence', str(self.root / secret), '--expected', str(self.path)]):
            run = subprocess.run([sys.executable, str(ROOT / 'tools/ais_connection_binding.py'),
                                  *arguments], capture_output=True, text=True, check=False)
            self.assertEqual(run.returncode, 2)
            self.assertEqual(run.stdout, '')
            self.assertEqual(run.stderr, binding.ERROR + '\n')
            self.assertNotIn(secret, run.stderr)


if __name__ == '__main__':
    unittest.main()
