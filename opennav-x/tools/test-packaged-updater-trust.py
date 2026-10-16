"""Inert checks for exact packaged trust and actual legacy initialization."""
import copy
import hashlib
import importlib.util
import json
import io
import os
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('packaged_trust_fixture', Path(__file__).with_name('test-packaged-updater-windows.py'))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class TrustTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.app = self.root / 'app'
        self.app.mkdir()
        self.root_raw = b'{ "signed": { "version": 1 } }'
        self.raw = b'{"bootstrapRoot":' + self.root_raw + b',"metadataUrl":"https://inert.invalid","channel":"beta"}\n'
        self.pin = hashlib.sha256(self.raw).hexdigest()
        self.product = dict(commit='a' * 40, update_trust=dict(schema=1, productCommit='a' * 40, channel='beta',
                            sourcePath='opennav-x/installer/windows/staging-update-trust.json',
                            sourceSha256=self.pin, configSha256=self.pin))
        self.path = self.app / 'update-trust.json'
        self.path.write_bytes(self.raw)

    def initialize(self, generation='a' * 32, version=1):
        identity = helper.packaged_trust_identity(self.app, self.product)
        (self.root / 'update-artifacts').mkdir(exist_ok=True)
        channel = self.root / 'update-trust/beta'
        channel.mkdir(parents=True, exist_ok=True)
        cache = channel / generation / 'metadata'
        cache.mkdir(parents=True)
        root = self.root_raw if version == 1 else json.dumps({'signed': {'version': version}}).encode()
        (cache / 'root.json').write_bytes(root)
        (self.root / 'update-trust-initialized-beta').write_bytes(json.dumps(
            dict(schema=1, channel='beta', configSha256=self.pin), separators=(',', ':')).encode())
        state = dict(schema=1, **{k: identity[k] for k in ('bootstrapSha256', 'metadataUrl', 'channel')},
                     generation=generation, metadata={'root.json': hashlib.sha256(root).hexdigest()})
        (channel / 'state.json').write_text(json.dumps(state))
        return identity

    def test_exact_selected_original_trust_and_intentional_initialization_are_retained(self):
        before = self.path.read_bytes()
        identity = self.initialize()
        self.assertEqual(identity['state'], 'packaged-public-trust')
        self.assertEqual(identity['configSha256'], self.pin)
        self.assertEqual(identity['bootstrapSha256'], hashlib.sha256(self.root_raw).hexdigest())
        self.assertNotEqual(identity['bootstrapSha256'], hashlib.sha256(json.dumps(json.loads(self.root_raw)).encode()).hexdigest())
        self.assertEqual(self.path.read_bytes(), before)
        original = helper.legacy_trust_observation(self.root, identity, initial=True)
        self.assertEqual(original['state'], 'initialized-original-identity')
        self.assertTrue(helper.legacy_trust_preserved(original, helper.legacy_trust_observation(self.root, identity)))

    def test_changed_missing_undeclared_mixed_commit_and_source_pins_refuse(self):
        for key, value in (('schema', True), ('productCommit', 'b' * 40), ('channel', 'stable'),
                           ('sourcePath', 'unreviewed.json'), ('sourceSha256', '0' * 64),
                           ('configSha256', self.pin.upper())):
            bad = copy.deepcopy(self.product)
            bad['update_trust'][key] = value
            with self.subTest(key=key), self.assertRaises(AssertionError):
                helper.packaged_trust_identity(self.app, bad)
        self.path.write_bytes(self.raw + b' ')
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, {'commit': self.product['commit']})
        self.path.unlink()
        with self.assertRaises(FileNotFoundError):
            helper.packaged_trust_identity(self.app, self.product)
        self.assertEqual(helper.packaged_trust_identity(self.app, {'commit': self.product['commit']}), {'state': 'unconfigured'})

    def test_linked_or_oversized_public_trust_refuses(self):
        other = self.root / 'original.json'
        self.path.rename(other)
        self.path.symlink_to(other)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        self.path.unlink()
        self.path.hardlink_to(other)
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)
        self.path.unlink()
        self.path.write_bytes(b'x' * ((576 << 10) + 1))
        with self.assertRaises(AssertionError):
            helper.packaged_trust_identity(self.app, self.product)

    def test_unconfigured_packages_refuse_any_legacy_initialization_output_without_reset(self):
        for name in ('update-trust', 'update-artifacts', 'update-trust-initialized-beta', 'update-trust-initialized-stable'):
            path = self.root / name
            path.write_bytes(b'original state must survive')
            with self.subTest(name=name), self.assertRaises(AssertionError):
                helper.legacy_trust_observation(self.root, {'state': 'unconfigured'})
            self.assertEqual(path.read_bytes(), b'original state must survive')
            path.unlink()
        (self.root / 'update-trust').symlink_to(self.root / 'missing')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, {'state': 'unconfigured'})

    def test_legitimate_cache_refresh_advances_but_downgrade_or_replaced_marker_refuses(self):
        identity = self.initialize()
        before = helper.legacy_trust_observation(self.root, identity)
        # Real refresh atomically replaces state.json and selects a new cache;
        # the original marker/store directory objects remain intact.
        self.initialize('b' * 32, 2)
        after = helper.legacy_trust_observation(self.root, identity)
        self.assertTrue(helper.legacy_trust_preserved(before, after))
        self.assertFalse(helper.legacy_trust_preserved(after, before))
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity, initial=True)
        marker = self.root / 'update-trust-initialized-beta'
        original = marker.read_bytes()
        marker.rename(self.root / 'original-marker-retained')
        marker.write_bytes(original)
        self.assertFalse(helper.legacy_trust_preserved(after, helper.legacy_trust_observation(self.root, identity)))

    def test_self_consistent_reserialized_initial_root_refuses(self):
        identity = self.initialize()
        channel = self.root / 'update-trust/beta'
        root = json.dumps(json.loads(self.root_raw)).encode()
        (channel / ('a' * 32) / 'metadata/root.json').write_bytes(root)
        state_path = channel / 'state.json'
        state = json.loads(state_path.read_bytes())
        state['metadata']['root.json'] = hashlib.sha256(root).hexdigest()
        state_path.write_text(json.dumps(state))
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity, initial=True)

    def test_missing_corrupt_changed_identity_pending_and_metadata_refuse(self):
        identity = self.initialize()
        channel = self.root / 'update-trust/beta'
        state_path = channel / 'state.json'
        original = state_path.read_bytes()
        for key, value in (('schema', True), ('bootstrapSha256', '0' * 64), ('metadataUrl', 'https://other.invalid'),
                           ('channel', 'stable'), ('generation', '../escape')):
            state = json.loads(original)
            state[key] = value
            state_path.write_text(json.dumps(state))
            with self.subTest(key=key), self.assertRaises(AssertionError):
                helper.legacy_trust_observation(self.root, identity)
        state_path.write_bytes(original)
        pending = channel / 'refresh.pending'
        pending.write_bytes(b'uncertain original refresh')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)
        self.assertEqual(pending.read_bytes(), b'uncertain original refresh')
        pending.unlink()
        metadata = channel / ('a' * 32) / 'metadata/root.json'
        metadata.write_bytes(b'changed')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)
        metadata.unlink()
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)

    def test_public_trust_duplicate_fields_and_changed_once_marker_refuse(self):
        with self.assertRaises(AssertionError):
            helper.public_trust_identity(self.raw.replace(b'"channel":"beta"', b'"channel":"beta","channel":"beta"'))
        identity = self.initialize()
        marker = self.root / 'update-trust-initialized-beta'
        marker.write_bytes(marker.read_bytes() + b'\n')
        with self.assertRaises(AssertionError):
            helper.legacy_trust_observation(self.root, identity)


class StartupDiagnosticTests(unittest.TestCase):
    def test_wow64_query_rejects_empty_relative_mismatched_and_oversized_results(self):
        for text, length in (('', 20), ('relative', 8), ('/absolute', 2), ('', 0), ('', 32768)):
            query = mock.Mock(side_effect=lambda buffer, size: (setattr(buffer, 'value', text) or length))
            with self.subTest(text=text, length=length), \
                 mock.patch.object(helper.ctypes, 'WinDLL', create=True, return_value=types.SimpleNamespace(GetSystemWow64DirectoryW=query)):
                with self.assertRaises(AssertionError): helper.wow64_system_directory()
            self.assertEqual(query.call_args.args[1], 260)

    def test_wow64_query_uses_one_bounded_required_capacity_retry(self):
        text = str(Path(tempfile.gettempdir()).resolve() / ('x'*280))
        calls = []
        def query_path(buffer, capacity):
            calls.append(capacity)
            if len(calls) == 1: return len(text) + 1
            buffer.value = text
            return len(text)
        query = mock.Mock(side_effect=query_path)
        with mock.patch.object(helper.ctypes, 'WinDLL', create=True, return_value=types.SimpleNamespace(GetSystemWow64DirectoryW=query)):
            self.assertEqual(helper.wow64_system_directory(), Path(text))
            self.assertEqual(calls, [260, len(text) + 2])
            query.side_effect = lambda buffer, size: size + 1
            with self.assertRaisesRegex(AssertionError, 'bounded retry'): helper.wow64_system_directory()

    def test_bounded_capture_retains_original_error_and_never_exports_challenge(self):
        for code, raw in ((0, b'current-qualified\n'), (1, b'Receiver: connect-timeout\n'),
                          (None, b'CloseOwned failed\n'), (1, b'x' * 65537)):
            process = mock.Mock(pid=123, stdout=io.BytesIO(b'original output\n'), stderr=io.BytesIO(raw))
            process.wait.side_effect = subprocess.TimeoutExpired(['original'], 240) if code is None else None
            process.wait.return_value = code; process.poll.return_value = code
            owned = set()
            with mock.patch.object(helper.subprocess, 'Popen', return_value=process) as popen, \
                 mock.patch.object(helper.subprocess, 'STARTUPINFO', create=True, return_value=types.SimpleNamespace(dwFlags=0)), \
                 mock.patch.object(helper.subprocess, 'STARTF_USESHOWWINDOW', 1, create=True), \
                 mock.patch.dict(os.environ, {'Skager_Update_Challenge': 'a'*64, 'KEEP': 'ordinary'}, clear=True):
                result = helper.capture_supervisor_diagnostic(['original'], Path('app'), owned)
            self.assertEqual(popen.call_count, 1); process.wait.assert_called_once_with(timeout=240)
            self.assertEqual(popen.call_args.kwargs['env'], {'KEEP': 'ordinary'})
            self.assertEqual(popen.call_args.kwargs['cwd'], Path('app'))
            self.assertEqual(result['exitCode'], code); self.assertEqual(result['timedOut'], code is None)
            self.assertEqual(owned, {123} if code is None else set())
            self.assertTrue(result['stderr']['complete'])
            self.assertEqual(result['stderr']['truncated'], len(raw) > 65536)
            self.assertIn('withheld' if len(raw) > 65536 else raw.decode().strip(), result['stderr']['text'])
        raw = b'SKAGER_UPDATE_CHALLENGE=' + b'a'*64 + b'\nSKAGER-UPDATE-READY/1 secret-frame\nother ' + b'b'*64
        clean = helper.diagnostic_text(raw)
        self.assertNotIn('a'*64, clean); self.assertNotIn('b'*64, clean); self.assertNotIn('secret-frame', clean)

    def test_wow64_host_and_both_pe32_images_are_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); launcher = root/'skager-start.exe'
            host = root/'SysWOW64/WindowsPowerShell/v1.0/powershell.exe'; host.parent.mkdir(parents=True)
            data = bytearray(90); data[:2] = b'MZ'; data[60:64] = (64).to_bytes(4, 'little')
            data[64:70] = b'PE\0\0\x4c\x01'; data[88:90] = b'\x0b\x01'
            launcher.write_bytes(data); host.write_bytes(data)
            query = mock.Mock(side_effect=lambda buffer, size: (setattr(buffer, 'value', str(root/'SysWOW64')) or len(buffer.value)))
            with mock.patch.object(helper.ctypes, 'WinDLL', create=True, return_value=types.SimpleNamespace(GetSystemWow64DirectoryW=query)):
                self.assertEqual(helper.diagnostic_host(launcher), host)
                data[68:70] = b'\x64\x86'; host.write_bytes(data)
                with self.assertRaisesRegex(AssertionError, 'I386/PE32'): helper.diagnostic_host(launcher)

    def test_original_supervisor_once_and_diagnostic_always_refuses_qualification(self):
        for code in (0, 1, None):
            with self.subTest(exit_code=code), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary); install = root/'OpenNavXAlpha1'; initial = 'a'*32
                first = install/'generations'/initial; (first/'app').mkdir(parents=True)
                profile=root/'profile'; profile.mkdir(); evidence=root/'evidence'; evidence.mkdir()
                source=root/'source'; source.mkdir(); stock=root/'stock/opencpn.exe'; stock.parent.mkdir(); stock.write_bytes(b'stock')
                setup=root/'Setup.exe'; setup.write_bytes(b'original setup')
                paths=('app/opencpn.exe','app/skager-start.exe','UpdateSupervisor.ps1','UpdateTransaction.ps1','Lifecycle.ps1')
                for path in paths: (first/path).write_bytes(('original '+path).encode())
                commit=helper.STARTUP_FAILURE['productCommit']; exe_hash=helper.digest(first/'app/opencpn.exe')
                owner='OpenNavX.Alpha1.SideBySide.1'
                (first/'ownership.json').write_text(json.dumps(dict(owner=owner,commit=commit,updateStartupHealth=1,
                    xnavHardwareOutputPolicy='status-only',managedFiles=[dict(path=p,sha256=helper.digest(first/p)) for p in paths])))
                (install/'state.json').write_text(json.dumps(dict(owner=owner,schema=1,current=initial)))
                host=root/'powershell.exe'; host.write_bytes(b'original host')
                ui=mock.Mock(); ui.windows.return_value=[]
                result=dict(exitCode=code,timedOut=code is None,stdout=dict(text=''),stderr=dict(text='original receiver or cleanup error'))
                with mock.patch.dict(os.environ, dict(LOCALAPPDATA=str(root),GITHUB_ACTIONS='true',RUNNER_ENVIRONMENT='github-hosted'), clear=True), \
                     mock.patch.object(helper.sys,'platform','win32'), \
                     mock.patch.object(helper,'diagnostic_host',return_value=host), \
                     mock.patch.object(helper,'capture_supervisor_diagnostic',return_value=result) as capture, \
                     mock.patch.object(helper.subprocess,'Popen') as forbidden_later_launch:
                    with self.assertRaisesRegex(RuntimeError,'original launcher failure remains unresolved'):
                        helper.qualify(install=install,setup=setup,stock=stock,profile=profile,evidence=evidence,
                            powershell=Path('wrong-64-bit-host'),commit=commit,executable_sha256=exe_hash,
                            ui=ui,owned=set(),inventory=lambda path:{},navigation_matches=lambda:True,
                            product=dict(commit=commit,executable_sha256=exe_hash),source_app=source,startup_diagnostic=True)
                capture.assert_called_once()
                self.assertEqual(capture.call_args.args[:2], ([str(host),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
                    '-File',str(first/'UpdateSupervisor.ps1'),'-InstallationRoot',str(install),'-Action','QualifyCurrent'], first/'app'))
                forbidden_later_launch.assert_not_called()
                report=json.loads((evidence/'packaged-updater.json').read_text())
                self.assertEqual(report['status'],'failed'); self.assertFalse(report['releaseQualification'])
                self.assertTrue(report['diagnosticOnly']); self.assertEqual(report['originalFailure'],helper.STARTUP_FAILURE)
                self.assertEqual(report['supervisorDiagnostic']['exitCode'],code)
                self.assertTrue(report['supervisorDiagnostic']['inputsUnchanged'])
                self.assertFalse(report['supervisorDiagnostic']['acceptance'])


if __name__ == '__main__':
    unittest.main()
