"""Offline runtime staging contracts; does not simulate native UI acceptance."""
import importlib.util
import json
from pathlib import Path
import shutil
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


if __name__ == '__main__':
    unittest.main()
