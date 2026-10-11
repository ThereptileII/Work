#!/usr/bin/env python3
"""Fail-closed harness contracts; no application, display or peer is launched."""
import importlib.util
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
import zipfile
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    "peer_two", Path(__file__).with_name("smoke-peer-two-instances.py"))
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class PeerTwoContracts(unittest.TestCase):
    def test_cli_failure_never_renders_secret_exception_or_chain(self):
        # A real child interpreter captures the outward CLI streams and status;
        # only run() is replaced, so no application or peer is launched.
        script = Path(peer.__file__).resolve()
        for exception_type in ('AssertionError', 'KeyboardInterrupt', 'SystemExit'):
            code = f'''import importlib.util, sys
sys.path.insert(0, {str(script.parent)!r})
spec = importlib.util.spec_from_file_location("peer_cli", {str(script)!r})
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)
def fail(args):
    secret = b"\\n".join(peer.SECRETS).decode()
    try:
        raise ValueError(secret)
    except ValueError as cause:
        raise {exception_type}(secret) from cause
peer.run = fail
sys.exit(peer.main([]))
'''
            with self.subTest(exception_type=exception_type):
                child = subprocess.run([sys.executable, '-c', code],
                                       capture_output=True, timeout=20)
                self.assertEqual(child.returncode, 1)
                self.assertEqual(child.stdout, b'')
                self.assertIn(b'Peer containment gate failed', child.stderr)
                self.assertNotIn(b'Traceback', child.stderr)
                for secret in peer.SECRETS:
                    self.assertNotIn(secret, child.stdout + child.stderr)

    def test_cli_success_returns_zero(self):
        with patch.object(peer, 'run') as run:
            self.assertEqual(peer.main([]), 0)
        run.assert_called_once()

    def test_native_peer_topology_preserves_ordinary_and_portable_paths(self):
        self.assertEqual(peer.launch_plan('nt'), ((1, False), (2, True)))
        self.assertEqual(peer.launch_plan('posix'), ((1, False), (2, False)))
        exe, first, second = Path('opencpn.exe'), Path('profile 1'), Path('profile 2')
        ordinary = peer.launch_arguments(exe, first)
        portable = peer.launch_arguments(exe, second, portable=True)
        self.assertEqual(ordinary, ['opencpn.exe', '--configdir', str(first),
                                    '--no_opengl', '--xnav'])
        self.assertEqual(portable, ['opencpn.exe', '--portable', '--configdir',
                                    str(second), '--no_opengl', '--xnav'])
        self.assertNotIn('--xnav-preview', portable)
        self.assertNotIn('--portable', ordinary)
        self.assertIsNone(peer.launch_working_directory(first))
        self.assertEqual(peer.launch_working_directory(second, portable=True),
                         str(second))

    def failure_ui(self, title='OpenCPN', caption='Existing instance is too busy'):
        def native_class(handle, buffer, length):
            buffer.value = {1: '#32770', 2: 'Static', 3: 'Edit'}[handle]
            return len(buffer.value)
        return types.SimpleNamespace(C=ctypes,
            windows=Mock(return_value=[(1, 123, title)]),
            children=Mock(return_value=[(2, caption), (3, 'editable private text')]),
            GetClassNameW=native_class, IsWindowEnabled=lambda handle: handle != 1)

    def test_failure_observes_owned_modal_and_missing_diagnostics_without_input(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            (profile / 'opencpn.log').write_bytes(b'')
            process = Mock(pid=123)
            process.poll.return_value = None
            ui = self.failure_ui()
            result = peer.windows_startup_failure(process, profile, ui)
            self.assertIsNone(result['exit_code_before_cleanup'])
            self.assertEqual(result['files']['opencpn.log']['bytes'], 0)
            self.assertEqual(result['diagnostic_error'], 'FileNotFoundError')
            self.assertEqual(result['windows'][0]['class'], '#32770')
            self.assertFalse(result['windows'][0]['enabled'])
            self.assertEqual(len(result['windows'][0]['controls']), 1)
            self.assertNotIn('editable private text', json.dumps(result))
            process.terminate.assert_not_called()
            ui.windows.assert_called_once_with(123)

    def test_failure_state_redacts_secrets_and_keeps_diagnostic_allowlist(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            secret = peer.SECRETS[0].decode()
            diagnostic = {'build_commit': secret, 'publication_monotonic_ms': 42,
                          'runtime': {'ui_update': {'ticks': 3}}, 'private': 'discarded'}
            path = profile / 'opennav-diagnostics.json'
            path.write_text(json.dumps(diagnostic))
            original = path.read_bytes()
            process = Mock(pid=123); process.poll.return_value = None
            result = peer.windows_startup_failure(process, profile, self.failure_ui(secret, secret))
            self.assertNotIn(secret, json.dumps(result))
            self.assertNotIn('discarded', json.dumps(result))
            self.assertEqual(result['diagnostic']['ui_ticks'], 3)
            self.assertEqual(path.read_bytes(), original)
            path.write_text('{invalid')
            self.assertEqual(peer.windows_startup_failure(process, profile, self.failure_ui())['diagnostic_error'], 'JSONDecodeError')

    def test_unready_live_process_still_fails_at_original_deadline(self):
        process = Mock(pid=123); process.poll.return_value = None
        with tempfile.TemporaryDirectory() as temporary, \
             patch.object(peer.time, 'monotonic', side_effect=[0, 0, 75]), \
             patch.object(peer.time, 'sleep'), self.assertRaisesRegex(RuntimeError, 'did not finish startup'):
            peer.wait_ready(process, Path(temporary))

    def retained_fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name).resolve()
        product, harness = 'a' * 40, 'b' * 40
        exe = root / 'build/xnav-install/opencpn.exe'
        exe.parent.mkdir(parents=True)
        exe.write_bytes(b'inert retained executable; never launched')
        build = root / 'build/xnav-windows'
        build.mkdir()
        identity = peer.staging_inputs.producer(product, '123', '1')
        manifest = json.dumps(dict(producer=identity, files=[dict(
            path='build/xnav-install/opencpn.exe', size=exe.stat().st_size,
            sha256=peer.staging_inputs.sha(exe))])).encode()
        archive = root / 'build/retained-staging-download/STAGING_BUILD_INPUTS.zip'
        archive.parent.mkdir()
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr(peer.staging_inputs.MANIFEST, manifest)
        receipt = root / 'evidence/local/staging-inputs.json'
        receipt.parent.mkdir(parents=True)
        record = dict(schema=1, kind=peer.staging_inputs.KIND, producer=identity,
                      status='restored', qualification='not-run', workspaceRoot=str(root),
                      harnessCommit=harness, archiveSha256=peer.staging_inputs.sha(archive),
                      manifestSha256=hashlib.sha256(manifest).hexdigest())
        receipt.write_text(json.dumps(record))
        return root, exe, build, product, harness, receipt, record, archive

    def test_same_commit_default_and_explicit_identity_pass(self):
        commit = 'a' * 40
        for requested in (None, commit):
            self.assertEqual(peer.product_identity(Path('unused'), Path('unused'), requested,
                                                   None, commit), (commit, None))
        for requested in ('b' * 40, '', 'a' * 8):
            with self.subTest(requested=requested), self.assertRaises(ValueError):
                peer.product_identity(Path('unused'), Path('unused'), requested, None, commit)

    def test_retained_producer_passes_with_separate_harness_identity(self):
        root, exe, build, product, harness, receipt, _, _ = self.retained_fixture()
        expected, digest = peer.product_identity(exe, build, product, receipt, harness, root)
        self.assertEqual(expected, product)
        self.assertEqual(digest, peer.staging_inputs.sha(receipt))
        peer.require_build({'build_commit': product}, expected)
        with self.assertRaises(AssertionError):
            peer.require_build({'build_commit': harness}, expected)

    def test_retained_receipt_mismatches_are_refused(self):
        root, exe, build, product, harness, receipt, record, _ = self.retained_fixture()
        for mutation in (dict(schema=True), dict(status='sealed'), dict(qualification='passed'),
                         dict(kind='other'), dict(harnessCommit=product), dict(workspaceRoot='/other'),
                         dict(producer=peer.staging_inputs.producer('c' * 40, '123', '1')),
                         dict(archiveSha256='bad'), dict(manifestSha256='0' * 64)):
            receipt.write_text(json.dumps({**record, **mutation}))
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                peer.product_identity(exe, build, product, receipt, harness, root)

    def test_missing_malformed_and_linked_receipts_are_refused(self):
        root, exe, build, product, harness, receipt, record, _ = self.retained_fixture()
        for raw in ('{broken', '[]', '{"producer": {}, "producer": {}}'):
            receipt.write_text(raw)
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                peer.product_identity(exe, build, product, receipt, harness, root)
        receipt.unlink()
        with self.assertRaises(FileNotFoundError):
            peer.product_identity(exe, build, product, receipt, harness, root)
        target = receipt.with_name('other.json')
        target.write_text(json.dumps(record))
        with self.assertRaises((ValueError, FileNotFoundError)):
            peer.product_identity(exe, build, product, target, harness, root)
        try:
            receipt.symlink_to(target)
        except OSError:
            return  # Windows may deny unprivileged symlink creation.
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)

    def test_retained_executable_and_paths_must_match_authenticated_manifest(self):
        root, exe, build, product, harness, receipt, _, archive = self.retained_fixture()
        with self.assertRaises(ValueError):
            peer.product_identity(exe, root / 'other-build', product, receipt, harness, root)
        exe.write_bytes(b'changed retained executable')
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)
        with zipfile.ZipFile(archive, 'w') as out:
            out.writestr(peer.staging_inputs.MANIFEST, b'{}')
        with self.assertRaises(ValueError):
            peer.product_identity(exe, build, product, receipt, harness, root)

    def test_only_exact_binary_identity_is_accepted(self):
        expected = "a" * 40
        peer.require_build({"build_commit": expected}, expected)
        for diagnostic in ({}, {"build_commit": ""}, {"build_commit": "b" * 40},
                           {"build_commit": expected[:8]}):
            with self.subTest(diagnostic=diagnostic), self.assertRaises(AssertionError):
                peer.require_build(diagnostic, expected)

    def test_windows_refuses_non_disposable_hosts(self):
        for env in ({}, {"GITHUB_ACTIONS": "true"},
                    {"GITHUB_ACTIONS": "true", "RUNNER_ENVIRONMENT": "self-hosted",
                     "RUNNER_TEMP": "unused"}):
            with patch.object(peer, "os", types.SimpleNamespace(name="nt", environ=env)):
                with self.assertRaises(RuntimeError):
                    peer.native_temp_root()

    def test_all_supported_outputs_reject_each_sentinel(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            outputs = [profile / name for name in ("stdout", "stderr", "opencpn.log")]
            for target in outputs:
                for secret in peer.SECRETS:
                    for output in outputs:
                        output.write_bytes(b"ordinary output\n")
                    target.write_bytes(b"before " + secret + b" after\n")
                    with self.subTest(output=target.name), self.assertRaises(AssertionError):
                        peer.assert_no_secrets(profile, outputs[0], outputs[1])
                    sanitized = peer.redact(target.read_bytes())
                    self.assertFalse(any(value in sanitized for value in peer.SECRETS))
                    self.assertIn(b"before [REDACTED] after", sanitized)
                    target.write_bytes(sanitized)
                    peer.assert_no_secrets(profile, outputs[0], outputs[1])
            outputs[0].unlink()
            with self.assertRaises(RuntimeError):
                peer.assert_no_secrets(profile, outputs[0], outputs[1])

    def test_failed_gate_keeps_original_error_and_sanitizes_each_available_stream(self):
        faults = ('none', 'cleanup', 'xserver-cleanup', 'cert.pem', 'key.pem',
                  'opencpn.log', 'peer-two-1.stdout.log', 'peer-two-1.stderr.log',
                  'export', 'report')
        for fault in faults:
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                # Exercise a noncanonical spelling on every host, like a Windows
                # temporary path whose short name/junction resolves differently.
                (root / 'path-alias').mkdir()
                root = root / 'path-alias' / '..'
                # run() resolves its evidence destination before exporting. The
                # injector must compare the same paths or its failure is missed.
                evidence = (root / 'evidence').resolve()
                exe = root / 'inert.exe'; exe.write_bytes(b'never launched')
                build = root / 'build'; build.mkdir()
                raw = b'prefix\n' + b'\n'.join(peer.SECRETS) + b'\nsuffix'
                original = AssertionError('original failed gate: ' + peer.SECRETS[0].decode())
                children = []

                def prepare(command, **kwargs):
                    profile = Path(command[command.index('--profile') + 1])
                    profile.mkdir()
                    (profile / 'opencpn.log').write_bytes(raw)
                    (profile / 'cert.pem').write_bytes(peer._CERT_BYTES)
                    (profile / 'key.pem').write_bytes(peer._KEY_BYTES)

                def launch(command, **kwargs):
                    for name in ('stdout', 'stderr'):
                        stream = kwargs[name]
                        self.assertFalse(Path(stream.name).is_relative_to(evidence))
                        stream.write(raw)
                    child = Mock(pid=100 + len(children))
                    child.poll.return_value = None
                    child.wait.return_value = 0
                    if ((fault == 'cleanup' and len(children) == 1) or
                            (fault == 'xserver-cleanup' and not children)):
                        child.terminate.side_effect = OSError('cleanup failed')
                    children.append(child)
                    return child

                read_bytes, write_bytes, write_text = Path.read_bytes, Path.write_bytes, Path.write_text

                def read(path):
                    if path.name == fault:
                        raise OSError('injected read failure')
                    return read_bytes(path)

                def write(path, data):
                    if fault == 'export' and path == evidence / 'peer-two-1.opencpn.log':
                        raise OSError('injected export failure')
                    return write_bytes(path, data)

                def write_report(path, data, **kwargs):
                    if fault == 'report' and path.name == 'peer-two-instances-results.json':
                        raise OSError('injected report failure')
                    return write_text(path, data, **kwargs)

                args = types.SimpleNamespace(exe=exe, build=build, evidence=evidence,
                    expected_commit=None, compiled_input_receipt=None)
                with patch.object(peer, 'os', types.SimpleNamespace(name='posix', environ={})), \
                     patch.object(peer, 'native_temp_root', return_value=root), \
                     patch.object(peer, 'product_identity', return_value=('a' * 40, None)), \
                     patch.object(peer.subprocess, 'check_output', return_value='a' * 40), \
                     patch.object(peer.subprocess, 'run', side_effect=prepare), \
                     patch.object(peer.subprocess, 'Popen', side_effect=launch), \
                     patch.object(peer.time, 'sleep'), \
                     patch.object(peer, 'fixtures', Mock()), \
                     patch.object(peer, 'PeerBoundary', Mock()), \
                     patch.object(peer, 'wait_ready', side_effect=original), \
                     patch.object(Path, 'read_bytes', read), \
                     patch.object(Path, 'write_bytes', write), \
                     patch.object(Path, 'write_text', write_report):
                    with self.assertRaises(AssertionError) as caught:
                        peer.run(args)
                self.assertIs(caught.exception, original)
                self.assertEqual(len(children), 3)
                for child in children:
                    child.terminate.assert_called_once()
                for path in evidence.iterdir():
                    content = path.read_bytes()
                    self.assertFalse(any(secret in content for secret in peer.SECRETS), path.name)
                # The second profile and display streams still export when any
                # first-profile cleanup, certificate, read or write fails.
                for name in ('peer-two-2.stdout.log', 'peer-two-2.stderr.log',
                             'peer-two-xvfb.stdout.log', 'peer-two-xvfb.stderr.log'):
                    self.assertIn(b'[REDACTED]', (evidence / name).read_bytes())
                if fault != 'report':
                    report = json.loads((evidence / 'peer-two-instances-results.json').read_text())
                    self.assertEqual(report['result'], 'FAILED')
                    self.assertIn('original failed gate: [REDACTED]', report['error'])
                    if fault != 'none':
                        self.assertTrue(report['finalization_errors'])
                    if fault == 'export':
                        self.assertIn({'stage': 'peer-two-1.opencpn.log', 'error': 'OSError'},
                                      report['finalization_errors'])
                        self.assertFalse((evidence / 'peer-two-1.opencpn.log').exists())

    def test_finalization_failure_invalidates_pass_and_redacts_multiline_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / 'raw'; destination = root / 'sanitized'
            source.write_bytes(b'ordinary output')
            report = {'result': 'PASS', 'error': peer._KEY_BYTES.decode()}
            child = Mock(); child.poll.return_value = None
            child.terminate.side_effect = OSError('stop failed')
            errors = peer.finalize_evidence([], child, [(source, destination)], root, report)
            self.assertEqual(errors, [{'stage': 'xserver-cleanup', 'error': 'OSError'}])
            retained = json.loads((root / 'peer-two-instances-results.json').read_text())
            self.assertEqual(retained['result'], 'FAILED')
            self.assertNotIn(peer._KEY_BYTES.strip().decode(), retained['error'])
            self.assertIn('[REDACTED]', retained['error'])
            self.assertEqual(destination.read_bytes(), b'ordinary output')

    def test_absent_optional_application_log_does_not_fail_finalization(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / 'stdout'; source.write_bytes(b'ordinary output')
            report = {'result': 'PASS', 'profiles': []}
            errors = peer.finalize_evidence([], None, [
                (source, root / 'stdout-retained'),
                (root / 'opencpn.log', root / 'opencpn-retained'),
            ], root, report)
            self.assertEqual(errors, [])
            self.assertEqual((root / 'stdout-retained').read_bytes(), b'ordinary output')
            self.assertFalse((root / 'opencpn-retained').exists())
            self.assertEqual(json.loads((root / 'peer-two-instances-results.json').read_text())['result'], 'PASS')

    def test_malformed_startup_evidence_is_not_retried_as_a_race(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)
            (profile / "opencpn.log").write_text("OnInitTimer...Finalize Canvases")
            (profile / "opennav-diagnostics.json").write_text("{broken")
            process = Mock(pid=123)
            process.poll.return_value = None
            with self.assertRaises(json.JSONDecodeError):
                peer.wait_ready(process, profile)

    def test_terminated_process_cannot_supply_startup_evidence(self):
        process = Mock(pid=123, returncode=1)
        process.poll.return_value = 1
        with tempfile.TemporaryDirectory() as temporary, self.assertRaises(RuntimeError):
            peer.wait_ready(process, Path(temporary))


if __name__ == "__main__":
    unittest.main(verbosity=2)
