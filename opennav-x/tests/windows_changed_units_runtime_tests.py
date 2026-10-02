"""Offline runtime staging contracts; does not simulate native UI acceptance."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class SettingsRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.build, self.wx, self.evidence = (self.root / p for p in ('build', 'wx', 'evidence'))
        self.client = self.build / 'Release/settings_drawer_test.exe'
        self.write(self.client, b'component test only')
        self.evidence.mkdir()
        for name in ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'wxmsw32u_aui_vc14x.dll'):
            self.write(self.wx / 'lib/vc14x_dll' / name, name.encode())
        self.install = self.root / 'VS'
        for version in ('14.9.1', '14.40.1'):
            for arch in ('x86', 'x64'):
                for name in ('msvcp140.dll', 'vcruntime140.dll'):
                    self.write(self.install / 'VC/Redist/MSVC' / version / arch / 'Microsoft.VC143.CRT' / name,
                               (version + arch).encode())

    def write(self, path, content):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)

    def invoke(self, sha=None):
        def capture(command, log, **kwargs):
            self.assertEqual(command[1], ROOT / 'tools/prototype/capture-ais-component.py')
            self.assertEqual(command[2:6], ['--component', 'settings', '--client', self.client])
            output = command[-1]
            output.mkdir()
            (output / 'capture.json').write_text(json.dumps({
                'platform': 'win32', 'executable_sha256': sha or gate.record(self.client)['sha256']}))
        with patch.dict(gate.os.environ, {'ProgramFiles(x86)': str(self.root)}), \
             patch.object(gate.subprocess, 'check_output', return_value=str(self.install)), \
             patch.object(gate, 'run', side_effect=capture):
            return gate.settings_component(self.build, self.wx, self.evidence)

    def test_app_local_runtime_uses_newest_x86_crt_and_existing_capture(self):
        result = self.invoke()
        self.assertEqual(len(result['runtime']), 5)
        self.assertEqual((self.client.parent / 'msvcp140.dll').read_bytes(), b'14.40.1x86')
        for name, item in result['runtime'].items():
            self.assertEqual(gate.record(self.client.parent / name)['sha256'], item['sha256'])
        self.assertTrue((self.evidence / 'settings-runtime.json').is_file())

    def test_missing_pinned_wx_runtime_fails_before_capture(self):
        (self.wx / 'lib/vc14x_dll/wxmsw32u_core_vc14x.dll').unlink()
        with self.assertRaises(FileNotFoundError):
            self.invoke()
        self.assertFalse((self.evidence / 'settings-component').exists())

    def test_capture_from_different_executable_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'native tested executable'):
            self.invoke(sha='0' * 64)

    def test_x64_runtime_cannot_substitute_for_missing_x86_runtime(self):
        for path in (self.install / 'VC/Redist/MSVC').glob('*/x86'):
            shutil.rmtree(path)
        with self.assertRaisesRegex(ValueError, 'x86 VC143 runtime missing'):
            self.invoke()
        self.assertFalse((self.evidence / 'settings-component').exists())


class FloatingRuntimeTests(unittest.TestCase):
    write = SettingsRuntimeTests.write

    def setUp(self):
        SettingsRuntimeTests.setUp(self)
        self.client = self.build / 'Release/floating_surface_test.exe'
        self.write(self.client, b'exact floating fixture')

    def invoke(self, text='12 floating-surface lifecycle checks passed\n', mutate=None):
        def capture(command, log, **kwargs):
            self.assertEqual(command, [self.client])
            self.assertEqual(kwargs['timeout'], 30)
            log.write_text(text)
            if mutate == 'executable':
                self.client.write_bytes(b'changed')
            if mutate == 'runtime':
                (self.client.parent / 'msvcp140.dll').write_bytes(b'changed')
        with patch.dict(gate.os.environ, {'ProgramFiles(x86)': str(self.root)}), \
             patch.object(gate.subprocess, 'check_output', return_value=str(self.install)), \
             patch.object(gate, 'run', side_effect=capture):
            return gate.floating_surface_component(self.build, self.wx, self.evidence)

    def test_exact_fixture_with_pinned_wx_and_x86_runtime(self):
        result = self.invoke()
        self.assertEqual(result['checks'], 12)
        self.assertEqual(result['exitCode'], 0)
        self.assertEqual(len(result['runtime']), 4)
        self.assertEqual((self.client.parent / 'msvcp140.dll').read_bytes(), b'14.40.1x86')

    def test_incomplete_duplicate_or_gtk_result_cannot_pass_native_gate(self):
        for text in ('', '11 floating-surface lifecycle checks passed\n',
                     '17 floating-surface lifecycle checks passed\n',
                     '12 floating-surface lifecycle checks passed\n' * 2):
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'all 12 Windows'):
                self.invoke(text)

    def test_changed_executable_or_runtime_rejected(self):
        for mutation in ('executable', 'runtime'):
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, 'changed during proof'):
                self.invoke(mutate=mutation)

    def test_original_missing_main_error_and_unrelated_failures(self):
        original = json.loads((ROOT / 'docs/evidence/scrum-245-floating-entry/original-failure.json').read_text())
        text = '\n'.join(original['errors'])
        gate.require_missing_main_failure(text)
        for other in ('', text.replace('_main referenced', '_other referenced'),
                      text + '\nerror C2065: invalid identifier',
                      text + '\nerror LNK2019: unresolved external symbol _other',
                      text.replace('LNK1120', 'LNK2001')):
            with self.subTest(other=other), self.assertRaisesRegex(ValueError, 'different reason'):
                gate.require_missing_main_failure(other)


class FloatingWorkflowLayoutTests(unittest.TestCase):
    relative = Path('.github/workflows/skager-windows-changed-units.yml')

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name) / 'repository'
        self.repo.mkdir()
        subprocess.run(['git', 'init', '-q', self.repo], check=True)
        self.workflow = self.repo / self.relative
        self.workflow.parent.mkdir(parents=True)
        self.workflow.write_bytes(b'exact workflow bytes')

    def test_standalone_layout_binds_actual_workflow_hash(self):
        key, identity = gate.floating_workflow_input(self.repo)
        self.assertEqual(key, self.relative.as_posix())
        self.assertEqual(identity, gate.record(self.workflow))
        self.assertEqual(gate.record(self.repo / key), identity)

    def test_monorepo_layout_binds_parent_workflow_hash_and_detects_changes(self):
        product = self.repo / 'opennav-x'
        product.mkdir()
        key, identity = gate.floating_workflow_input(product)
        self.assertEqual(key, '../' + self.relative.as_posix())
        self.assertEqual(gate.record(product / key), identity)
        self.workflow.write_bytes(b'changed workflow bytes')
        self.assertNotEqual(gate.floating_workflow_input(product), (key, identity))

    def test_ambiguous_monorepo_workflow_is_rejected(self):
        product = self.repo / 'opennav-x'
        shadow = product / self.relative
        shadow.parent.mkdir(parents=True)
        shadow.write_bytes(b'shadow workflow')
        with self.assertRaisesRegex(ValueError, 'Ambiguous'):
            gate.floating_workflow_input(product)

    def test_missing_or_unknown_layout_does_not_search_other_parents(self):
        product = self.repo / 'other-product'
        product.mkdir()
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            gate.floating_workflow_input(product)
        self.workflow.unlink()
        with self.assertRaisesRegex(ValueError, 'Missing or redirected'):
            gate.floating_workflow_input(self.repo)


if __name__ == '__main__':
    unittest.main()
