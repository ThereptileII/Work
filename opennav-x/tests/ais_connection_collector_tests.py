"""Inert collector sequencing, privacy and lifetime tests; no Windows/DNS calls."""
import copy
import ctypes
from datetime import datetime, timedelta, timezone
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import struct
import socket
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
SPEC = importlib.util.spec_from_file_location('ais_collector', ROOT / 'tools/collect-ais-connection.py')
collector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(collector)
binding = collector.binding
ORIGINAL_UNLINK = os.unlink


def fixture():
    expected = {'schema': 1, 'image_path': r'C:\Program Files\SKAGER\opencpn.exe',
                'image_sha256': 'a' * 64,
                'modules': [{'path': r'C:\Program Files\SKAGER\opencpn.exe', 'sha256': 'a' * 64},
                            {'path': r'C:\Windows\System32\ws2_32.dll', 'sha256': 'b' * 64}]}
    local = {'address': 'c0a8010c' + '0' * 24, 'port': 50321, 'scope': 0}
    remote = {'address': '01010101' + '0' * 24, 'port': 443, 'scope': 0}
    process = {'pid': 42, 'creation_filetime': '1234567890', 'image_path': expected['image_path'],
               'image_sha256': expected['image_sha256'], 'modules': copy.deepcopy(expected['modules']),
               'same_image_processes': [{'pid': 42, 'creation_filetime': '1234567890'}]}
    tcp = {'pid': 42, 'state': 'ESTABLISHED', 'family': 'ipv4', 'local': local, 'remote': remote}
    dns = {'host': collector.HOST,
           'addresses': [{'family': 'ipv4', 'address': remote['address'], 'scope': 0}]}
    observation = {'schema': 1, 'source': 'AISSTREAM_ONLINE', 'available': True,
                   'reason': 'observed', 'process_id': 42, 'observed_utc': '2026-10-12T12:00:01Z',
                   'family': 'ipv4', 'generation': '1', 'captured_monotonic_ns': '100000000',
                   'observed_monotonic_ns': '1100000000', 'age_ms': '1000',
                   'local': local, 'remote': remote}
    report = {'build_purpose': 'INSTALLED PRODUCT', 'data_mode': 'OPENCPN selected navigation',
              'test_fixtures': False, 'build_commit': 'c' * 40,
              'runtime': {'online_ais': {'connection_observation': observation},
                          'pilot': {'private_synthetic_fixture': 'PRIVATE-PILOT-FIXTURE'}},
              'vessel': {'latitude': -54.34, 'heading': 51.25, 'profile': 'PRIVATE-VESSEL-FIXTURE'}}
    return expected, process, tcp, dns, report


class FakeClock:
    def __init__(self):
        self.seconds, self.events = 0, []
    def monotonic(self):
        return self.seconds
    def utc(self):
        return datetime(2026, 10, 12, 12, tzinfo=timezone.utc) + timedelta(seconds=self.seconds)
    def sleep(self, seconds):
        self.events.append('sleep')
        self.seconds += seconds


class FakeProvider:
    def __init__(self, budget):
        self.budget, self.events, self.closed, self.phase = budget, [], False, -1
        self.expected, self.process, self.tcp, self.dns, self.report = fixture()
        self.fail = None
    def event(self, name):
        self.events.append(name)
        if name == self.fail:
            raise OSError('PRIVATE-FAILURE-FIXTURE')
    def expected_bytes(self, path):
        self.event('expected')
        return json.dumps(self.expected).encode()
    def prepare(self, pid, expected):
        self.event('prepare')
        self.asserted_expected = copy.deepcopy(expected)
    def process_sample(self):
        self.phase += 1
        self.event('process')
        return copy.deepcopy(self.process)
    def tcp_sample(self, pid):
        self.event('tcp')
        return copy.deepcopy(self.tcp if isinstance(self.tcp, list) else [self.tcp])
    def dns_sample(self):
        self.event('dns')
        return copy.deepcopy(self.dns)
    def diagnostics_bytes(self, path):
        self.event('diagnostics')
        report = copy.deepcopy(self.report)
        if self.phase == 1:
            report['runtime']['online_ais']['connection_observation'].update(
                observed_utc='2026-10-12T12:00:02Z', observed_monotonic_ns='2200000000', age_ms='2100')
        return json.dumps(report).encode()
    def verify_retained(self):
        self.event('verify')
    def close(self):
        self.closed = True
        self.event('close')


class CollectorTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.provider = FakeClock(), None
    def factory(self, budget):
        self.provider = FakeProvider(budget)
        return self.provider
    def collect(self, mutation=None):
        def factory(budget):
            provider = self.factory(budget)
            if mutation:
                mutation(provider)
            return provider
        return collector.collect(42, r'C:\evidence\diagnostics.json', r'C:\review\pins.json',
                                 provider_factory=factory, clock=self.clock)
    def test_success_drains_owners_before_binding_validation(self):
        validate = binding.validate
        def check(*args):
            self.assertTrue(self.provider.closed)
            return validate(*args)
        with patch.object(binding, 'validate', side_effect=check):
            encoded = self.collect()
        evidence = json.loads(encoded)
        self.assertEqual(self.clock.events, ['sleep', 'sleep'])
        self.assertEqual(self.provider.events, ['expected', 'prepare', 'process', 'tcp', 'dns',
                         'diagnostics', 'process', 'tcp', 'dns', 'diagnostics', 'verify', 'close'])
        self.assertEqual(evidence['started_utc'], '2026-10-12T12:00:00Z')
        self.assertEqual(evidence['finished_utc'], '2026-10-12T12:00:02Z')
        self.assertEqual(evidence['observation_after']['captured_monotonic_ns'], '100000000')
        self.assertEqual(evidence['observation_after']['age_ms'], '2100')
        self.assertEqual(binding.validate(evidence, self.provider.expected, self.clock.utc()),
                         {'schema': 1, 'state': 'OBSERVATION_MATCHED', 'authorizes_filter': False})
    def test_unrelated_finite_sensor_data_is_accepted_and_never_output(self):
        encoded = self.collect()
        for forbidden in (b'PRIVATE', b'latitude', b'pilot', b'profile', b'build_commit', b'build_purpose'):
            self.assertNotIn(forbidden, encoded)
        self.assertEqual(set(json.loads(encoded)), binding.ROOT_KEYS)
    def test_every_provider_failure_drains_owners(self):
        for step in ('expected', 'prepare', 'process', 'tcp', 'dns', 'diagnostics', 'verify', 'close'):
            with self.subTest(step=step):
                self.clock = FakeClock()
                with self.assertRaises((OSError, ValueError)):
                    self.collect(lambda provider: setattr(provider, 'fail', step))
                self.assertTrue(self.provider.closed)
    def test_expected_validation_precedes_prepare(self):
        with self.assertRaises(binding.EvidenceError):
            self.collect(lambda p: p.expected.update(image_sha256='PRIVATE-INVALID'))
        self.assertEqual(self.provider.events, ['expected', 'close'])
    def test_runtime_principal_module_and_tcp_ambiguity_fail_closed(self):
        mutations = [lambda p: p.process.update(creation_filetime='999'),
                     lambda p: p.process['modules'][0].update(sha256='f' * 64),
                     lambda p: p.process['same_image_processes'].append({'pid': 43, 'creation_filetime': '2'}),
                     lambda p: setattr(p, 'tcp', [p.tcp, copy.deepcopy(p.tcp)]),
                     lambda p: p.report['runtime']['online_ais']['connection_observation'].update(process_id=43)]
        # A changed creation time in both samples is a different (but internally
        # stable) principal; test a stale/missing peer creation identity instead.
        for mutation in mutations:
            self.clock = FakeClock()
            with self.assertRaises((binding.EvidenceError, collector.CollectorError)):
                self.collect(mutation)
            self.assertTrue(self.provider.closed)
    def test_installed_product_markers_and_canonical_commit_required(self):
        for field, value in [('build_purpose', 'TEST'), ('data_mode', 'Demo'),
                             ('test_fixtures', True), ('build_commit', 'c' * 39),
                             ('build_commit', 'C' * 40)]:
            with self.subTest(field=field):
                self.clock = FakeClock()
                with self.assertRaises(collector.CollectorError):
                    self.collect(lambda p: p.report.update({field: value}))
    def test_full_diagnostics_strict_duplicate_nonfinite_and_bounds(self):
        for raw in (b'{"runtime":{},"runtime":{}}', b'{"vessel":NaN}',
                    b'{"vessel":1e999}', b'\xff', b' ' * (collector.MAX_DIAGNOSTICS + 1)):
            with self.subTest(raw=raw[:40]):
                with self.assertRaises((ValueError, UnicodeError)):
                    collector.diagnostic_json(raw)
        self.assertEqual(collector.diagnostic_json(b'{"vessel":-1.5}'), {'vessel': -1.5})
    def test_collection_deadline_and_clock_drift_refuse(self):
        def overdue(provider):
            original = provider.dns_sample
            def delayed():
                self.clock.seconds += 16
                return original()
            provider.dns_sample = delayed
        with self.assertRaises(collector.CollectorError):
            self.collect(overdue)
        self.assertTrue(self.provider.closed)
        budget = collector.Budget(FakeClock())
        budget.clock.utc = lambda: datetime(2026, 10, 12, 11, tzinfo=timezone.utc)
        with self.assertRaises(collector.CollectorError):
            budget.check()
    def test_cli_refusal_is_redacted_and_does_not_open_output(self):
        for extra in ([], ['--process-id', 'PRIVATE-INVALID']):
            stderr, stdout, calls = io.StringIO(), io.StringIO(), []
            with patch('sys.stderr', stderr), patch('sys.stdout', stdout):
                result = collector.main(extra, windows=True, writer=lambda *args: calls.append(args))
            self.assertEqual(result, 1)
            self.assertEqual(stderr.getvalue(), collector.REFUSAL)
            self.assertEqual(stdout.getvalue(), '')
            self.assertFalse(calls)
    def arguments(self):
        return ['--process-id', '42', '--diagnostics', r'C:\evidence\diagnostics.json',
                '--expected', r'C:\review\pins.json', '--output', r'C:\evidence\new.json']
    def test_cli_failed_capture_never_invokes_writer(self):
        calls, stderr = [], io.StringIO()
        def fail(*args):
            raise OSError('PRIVATE-FAILURE-FIXTURE')
        with patch('sys.stderr', stderr):
            result = collector.main(self.arguments(), runner=fail, windows=True,
                                    writer=lambda *args: calls.append(args))
        self.assertEqual(result, 1)
        self.assertFalse(calls)
        self.assertEqual(stderr.getvalue(), collector.REFUSAL)
    def test_cli_success_writes_only_after_capture_returns(self):
        encoded, events = self.collect(), []
        def run(*args):
            events.append('capture-drained')
            return encoded
        def write(path, raw):
            events.append('write')
            self.assertEqual(raw, encoded)
        with patch('sys.stdout', io.StringIO()) as stdout:
            self.assertEqual(collector.main(self.arguments(), runner=run, writer=write, windows=True), 0)
        self.assertEqual(events, ['capture-drained', 'write'])
        self.assertEqual(stdout.getvalue(), '')
    def test_nonwindows_and_path_aliases_refuse_before_capture(self):
        for path in (r'\\server\share\evidence.json', r'C:\..\evidence.json',
                     r'C:\NUL.json', r'C:\PROGRA~1\file.json', r'C:\file.json:ads',
                     r'C:relative.json', '/tmp/evidence.json'):
            args, called = self.arguments(), []
            args[-1] = path
            with patch('sys.stderr', io.StringIO()):
                self.assertEqual(collector.main(args, windows=True, runner=lambda *a: called.append(a)), 1)
            self.assertFalse(called)
        with patch('sys.stderr', io.StringIO()):
            self.assertEqual(collector.main(self.arguments(), windows=False), 1)
    def test_native_inventory_must_match_pins_before_any_binary_file_open(self):
        expected, _, _, _, _ = fixture()
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.kernel = types.SimpleNamespace(OpenProcess=object())
        native.open_handle = lambda *args: 1
        native.identity = lambda *args: {'pid': 42, 'creation_filetime': '1', 'image_path': expected['image_path']}
        native.inventory = lambda: [('c:\\unreviewed.dll', r'C:\unreviewed.dll', 8)]
        calls = []
        native.open_plain = lambda *args, **kwargs: calls.append(args)
        with self.assertRaises(collector.CollectorError):
            native.prepare(42, expected)
        self.assertFalse(calls)
    def test_native_open_owns_handle_before_post_call_deadline(self):
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.handles = []
        checks = []
        def check():
            checks.append(1)
            if len(checks) == 2:
                raise collector.CollectorError()
        native.budget = types.SimpleNamespace(check=check)
        with self.assertRaises(collector.CollectorError):
            native.open_handle(lambda: 123)
        self.assertEqual(native.handles, [123])
        closed = []
        native.kernel = types.SimpleNamespace(CloseHandle=lambda handle: closed.append(handle) or True)
        native.close()
        self.assertEqual(closed, [123])
    def test_native_release_closes_before_expired_budget_check(self):
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.handles, closed = [123], []
        native.kernel = types.SimpleNamespace(CloseHandle=lambda h: closed.append(h) or True)
        def expired():
            raise collector.CollectorError()
        native.budget = types.SimpleNamespace(check=expired)
        with self.assertRaises(collector.CollectorError):
            native.release(123)
        self.assertEqual(closed, [123])
        self.assertFalse(native.handles)


class NativeBoundaryTests(unittest.TestCase):
    def test_diagnostics_cap_above_evidence_cap_is_explicit(self):
        raw = b' ' * (collector.MAX_EVIDENCE + 1) + b'{"vessel":-1.5}'
        self.assertEqual(collector.diagnostic_json(raw), {'vessel': -1.5})
        with self.assertRaises(binding.EvidenceError):
            binding.strict_json_loads(raw)

    def test_only_reviewed_binary_handles_allow_stable_hardlinks(self):
        class FileTime(ctypes.Structure):
            _fields_ = [('dwLowDateTime', ctypes.c_uint32), ('dwHighDateTime', ctypes.c_uint32)]
        class Info(ctypes.Structure):
            _fields_ = [('attributes', ctypes.c_uint32), ('creation', FileTime),
                ('write', FileTime), ('volume', ctypes.c_uint32), ('size_high', ctypes.c_uint32),
                ('size_low', ctypes.c_uint32), ('links', ctypes.c_uint32),
                ('index_high', ctypes.c_uint32), ('index_low', ctypes.c_uint32)]
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.w, native.Info = None, Info
        links = [2]
        def information(handle, pointer):
            info = pointer._obj
            info.links, info.volume, info.index_low = links[0], 10, 20
            return True
        def final_path(handle, buffer, size, flags):
            buffer.value = '\\\\?\\C:\\Windows\\System32\\ws2_32.dll'
            return len(buffer.value)
        native.kernel = types.SimpleNamespace(GetFileType=lambda handle: 1,
            GetFileInformationByHandle=information, GetFinalPathNameByHandleW=final_path)
        native.call = lambda function, *args: function(*args)
        path = r'C:\Windows\System32\ws2_32.dll'
        with self.assertRaises(collector.CollectorError):
            native.file_identity(1, path)
        before = native.file_identity(1, path, pinned_binary=True)
        self.assertEqual(before[4], 2)
        native.pid, native.process = 42, 2
        native.original_identity, native.original_inventory = {}, []
        native.identity, native.inventory = lambda *args: {}, lambda: []
        native.retained = {'binary': (1, path, before, 'a' * 64, True)}
        native.verify_retained()
        links[0] = 3
        with self.assertRaises(collector.CollectorError):
            native.verify_retained()

    def test_tcp_retains_only_selected_rows_and_exact_competing_owners(self):
        u32 = ctypes.c_uint32
        class Tcp4(ctypes.Structure):
            _fields_ = [(name, u32) for name in ('state', 'local_address', 'local_port',
                                                'remote_address', 'remote_port', 'pid')]
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.w, native.Tcp4, native.Tcp6 = types.SimpleNamespace(DWORD=u32), Tcp4, Tcp4
        local = int.from_bytes(bytes.fromhex('c0a8010c'), 'little')
        remote = int.from_bytes(bytes.fromhex('01010101'), 'little')
        rows = [(5, local, socket.htons(50321), remote, socket.htons(443), 42),
                (5, local, socket.htons(50321), remote, socket.htons(443), 43),
                (5, local, socket.htons(50322), remote, socket.htons(443), 44),
                (5, local, socket.htons(50323), remote, socket.htons(22), 42)]
        def table(buffer, size, order, family, table_class, reserved):
            raw = struct.pack('<I', len(rows) if family == socket.AF_INET else 0)
            if family == socket.AF_INET:
                raw += b''.join(struct.pack('<6I', *row) for row in rows)
            size._obj.value = len(raw)
            if buffer is None:
                return 122
            ctypes.memmove(buffer, raw, len(raw))
            return 0
        native.iphlp = types.SimpleNamespace(GetExtendedTcpTable=table)
        native.call = lambda function, *args: function(*args)
        result = native.tcp_sample(42)
        self.assertEqual([row['pid'] for row in result], [42, 43])
        self.assertEqual(result[0]['remote']['address'], '01010101' + '0' * 24)

    def test_close_attempts_every_owner_and_retains_failed_close_for_retry(self):
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.handles, attempts = [1, 2, 3], []
        def close(handle):
            attempts.append(handle)
            return handle != 2
        native.kernel = types.SimpleNamespace(CloseHandle=close)
        with self.assertRaises(collector.CollectorError):
            native.close()
        self.assertEqual(attempts, [3, 2, 1])
        self.assertEqual(native.handles, [2])
        native.kernel.CloseHandle = lambda handle: True
        native.close()
        self.assertFalse(native.handles)

    def test_partial_frame_watchdog_terminates_only_own_child_and_drains_reader(self):
        ready, received = threading.Event(), threading.Event()
        class Receiver:
            def recv_bytes(self, limit):
                # Simulate a readable header whose body never completes until
                # the sender child is terminated. The reader itself is inert.
                ready.wait(0.5)
                received.set()
                raise EOFError()
            def close(self):
                self.closed = True
        receiver = Receiver()
        sender = types.SimpleNamespace(closed=False)
        sender.close = lambda: setattr(sender, 'closed', True)
        class Child:
            pid, exitcode, alive, terminated, closed = None, None, False, False, False
            def start(self):
                self.pid, self.alive = 123, True
            def is_alive(self):
                return self.alive
            def terminate(self):
                self.terminated, self.alive, self.exitcode = True, False, 1
                ready.set()
            kill = terminate
            def join(self, timeout):
                pass
            def close(self):
                self.closed = True
        child = Child()
        context = types.SimpleNamespace(Pipe=lambda **kwargs: (receiver, sender),
                                        Process=lambda **kwargs: child)
        started = time.monotonic()
        with patch.object(collector.multiprocessing, 'get_context', return_value=context), \
             patch.object(collector, 'WATCHDOG_SECONDS', 0.03):
            with self.assertRaises(collector.CollectorError):
                collector.run_with_watchdog(42, r'C:\diagnostics.json', r'C:\pins.json')
        self.assertLess(time.monotonic() - started, 0.3)
        self.assertTrue(child.terminated and child.closed and receiver.closed and sender.closed and received.is_set())


class OutputLifetimeTests(unittest.TestCase):
    """Emulate only file handle semantics; never load native Windows APIs."""
    def writer_fixture(self, actual, mode=None):
        native = collector.NativeWindows.__new__(collector.NativeWindows)
        native.handles, native.closed, native.marked, native.identity_calls = [], False, [], 0
        native.w = types.SimpleNamespace(HANDLE=ctypes.c_void_p)
        state = {'late': False, 'replace': False, 'delete_inode': None}
        def check():
            if state['late']:
                raise collector.CollectorError()
        native.budget = types.SimpleNamespace(check=check)
        def create(*args):
            fd = os.open(actual, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            if mode == 'late-open':
                state['late'] = True
            return fd
        def duplicate(process, handle, target, pointer, access, inherit, flags):
            fd = os.dup(handle)
            ctypes.cast(pointer, ctypes.POINTER(ctypes.c_void_p))[0] = fd
            if mode == 'late-duplicate':
                state['late'] = True
            return True
        def close(handle):
            os.close(handle)
            return True
        def disposition(handle, info_class, pointer, size):
            native.marked.append(handle)
            self.assertEqual(info_class, 4)
            self.assertEqual(size, 1)
            self.assertIn(handle, native.handles)
            # Windows keeps deletion pending until the final open handle closes.
            # POSIX unlink works here; Windows may reject this open file, so
            # model the pending disposition and apply it in drain below.
            state['delete_inode'] = os.fstat(handle).st_ino
            return True
        native.kernel = types.SimpleNamespace(CreateFileW=create, DuplicateHandle=duplicate,
            CloseHandle=close, GetCurrentProcess=lambda: -1, SetFileInformationByHandle=disposition)
        native.lock_parents = lambda *args, **kwargs: None
        def identity(handle, path):
            native.identity_calls += 1
            if mode == 'identity' or (mode == 'replace' and native.identity_calls == 3):
                if mode == 'replace':
                    # Simulate a later namespace occupant after handles drain;
                    # the retained original handle must never delete it.
                    state['replace'] = True
                raise collector.CollectorError()
            stat = os.fstat(handle)
            return (1, stat.st_ino, 0, 0, 1, stat.st_size, 123, 0)
        native.file_identity = identity
        original_close = native.close
        def drain():
            original_close()
            if state['replace']:
                ORIGINAL_UNLINK(actual)
                actual.write_bytes(b'REPLACEMENT-FIXTURE')
            if (state['delete_inode'] is not None and not state['replace'] and
                    actual.exists() and
                    actual.stat().st_ino == state['delete_inode']):
                ORIGINAL_UNLINK(actual)
            native.closed = True
        native.close = drain
        return native
    def run_writer(self, actual, mode=None):
        native = self.writer_fixture(actual, mode)
        fake_crt = types.SimpleNamespace(open_osfhandle=lambda handle, flags: handle)
        with patch.object(collector, 'NativeWindows', return_value=native), \
             patch.dict(sys.modules, {'msvcrt': fake_crt}), \
             patch.object(os, 'O_BINARY', 0, create=True), \
             patch.object(collector.os, 'unlink', side_effect=AssertionError('pathname deletion forbidden')):
            if mode:
                with self.assertRaises((ValueError, OSError)):
                    collector.write_new_output(r'C:\evidence\new.json', b'{"schema":1}')
            else:
                collector.write_new_output(r'C:\evidence\new.json', b'{"schema":1}')
        self.assertTrue(native.closed)
        self.assertFalse(native.handles)
        return native
    def test_success_exclusive_output_and_existing_file_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            actual = Path(directory) / 'evidence.json'
            native = self.run_writer(actual)
            self.assertEqual(actual.read_bytes(), b'{"schema":1}\n')
            self.assertFalse(native.marked)
            native = self.run_writer(actual, 'exists')
            self.assertEqual(actual.read_bytes(), b'{"schema":1}\n')
            self.assertFalse(native.marked)
    def test_late_acquisition_and_failed_identity_mark_owned_output_for_deletion(self):
        for mode in ('late-open', 'late-duplicate', 'identity'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                actual = Path(directory) / 'evidence.json'
                native = self.run_writer(actual, mode)
                self.assertFalse(actual.exists())
                self.assertEqual(len(native.marked), 1)
    def test_failure_never_deletes_replacement_namespace_occupant(self):
        with tempfile.TemporaryDirectory() as directory:
            actual = Path(directory) / 'evidence.json'
            native = self.run_writer(actual, 'replace')
            self.assertEqual(actual.read_bytes(), b'REPLACEMENT-FIXTURE')
            self.assertEqual(len(native.marked), 1)


if __name__ == '__main__':
    unittest.main()
