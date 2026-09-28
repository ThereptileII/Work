"""Portable tests of native dependency selection; no PE execution or network."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('package_probe', Path(__file__).with_name('package-ais-live-probe.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class DependencySelection(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.install = base / 'install'
        self.runtime = base / 'Microsoft.VC143.CRT'
        self.install.mkdir(); self.runtime.mkdir()
        for n in ['msvcp140.dll', 'vcruntime140.dll', 'vccorlib140.dll']:
            (self.runtime / n).write_bytes(b'licensed-current')
            (self.install / n).write_bytes(b'upstream-older')
        (self.install / 'libssl-3.dll').write_bytes(b'pinned-tls')

    def test_current_licensed_crt_wins_and_other_dependencies_stay_pinned(self):
        result = m.dependency_candidates(self.install, self.runtime)
        self.assertEqual(result['msvcp140.dll'], self.runtime / 'msvcp140.dll')
        self.assertEqual(result['vcruntime140.dll'], self.runtime / 'vcruntime140.dll')
        self.assertEqual(result['vccorlib140.dll'], self.runtime / 'vccorlib140.dll')
        self.assertEqual(result['libssl-3.dll'], self.install / 'libssl-3.dll')

    def test_unknown_conflicting_non_crt_is_refused(self):
        (self.runtime / 'libssl-3.dll').write_bytes(b'unknown')
        with self.assertRaises(ValueError): m.dependency_candidates(self.install, self.runtime)

    def test_incomplete_runtime_is_refused(self):
        (self.runtime / 'vcruntime140.dll').unlink()
        with self.assertRaises(ValueError): m.dependency_candidates(self.install, self.runtime)

    def test_unidentified_runtime_directory_is_refused(self):
        with self.assertRaises(ValueError): m.dependency_candidates(self.install, self.install)


if __name__ == '__main__': unittest.main()
