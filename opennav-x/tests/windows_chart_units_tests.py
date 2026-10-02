"""Offline refusal checks for the compile-only preflight; no native execution."""
import hashlib
import importlib.util
import io
from pathlib import Path
import struct
import subprocess
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('chart_units', ROOT / 'tools/windows_chart_units.py')
chart = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chart)


def record(path):
    data = path.read_bytes()
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


class ChartPreflightGuards(unittest.TestCase):
    def extract(self, members):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / 'source.tar'
            with tarfile.open(archive, 'w') as tar:
                for name, kind in members:
                    item = tarfile.TarInfo(name)
                    item.type = kind
                    item.size = 1 if kind == tarfile.REGTYPE else 0
                    tar.addfile(item, io.BytesIO(b'x') if item.size else None)
            chart.extract_locked_tree(archive, root / 'headers', 'source-1')
            return {p.relative_to(root / 'headers').as_posix(): p.read_bytes()
                    for p in (root / 'headers').rglob('*') if p.is_file()}

    def test_real_relative_header_extracts(self):
        self.assertEqual(self.extract([('source-1/include/a.h', tarfile.REGTYPE)]),
                         {'include/a.h': b'x'})

    def test_unsafe_paths_rejected(self):
        for name in ('../a', '/source-1/a', 'other/a', 'source-1/../a',
                     'source-1/C:a', 'source-1/a\\b'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.extract([(name, tarfile.REGTYPE)])

    def test_links_and_case_duplicates_rejected(self):
        for kind in (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.FIFOTYPE):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.extract([('source-1/include/a.h', kind)])
        with self.assertRaises(ValueError):
            self.extract([('source-1/a.h', tarfile.REGTYPE), ('source-1/A.h', tarfile.REGTYPE)])

    def object_fixture(self, root, machine=0x14c, big=False):
        path = root / 'check_chart_ChartPresentation.dir/Release/ChartPresentation.obj'
        path.parent.mkdir(parents=True)
        data = bytearray(56)
        if big:
            data[:4] = b'\0\0\xff\xff'
            struct.pack_into('<H', data, 6, machine)
        else:
            struct.pack_into('<H', data, 0, machine)
        path.write_bytes(data)
        return path

    def test_standard_and_bigobj_x86_accepted(self):
        for big in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                path = self.object_fixture(root, big=big)
                self.assertEqual(chart.verify_objects(root, ('ChartPresentation.cpp',), record),
                                 {'check_chart_ChartPresentation': record(path)})

    def test_missing_wrong_arch_empty_and_extra_objects_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(ValueError):
                chart.verify_objects(root, ('ChartPresentation.cpp',), record)
            path = self.object_fixture(root, machine=0x8664)
            with self.assertRaises(ValueError):
                chart.verify_objects(root, ('ChartPresentation.cpp',), record)
            path.write_bytes(b'')
            with self.assertRaises(ValueError):
                chart.verify_objects(root, ('ChartPresentation.cpp',), record)
            path.rename(path.with_name('Unexpected.obj'))
            with self.assertRaises(ValueError):
                chart.verify_objects(root, ('ChartPresentation.cpp',), record)

    def test_required_real_units_unique(self):
        units = chart.LOCAL_UNITS + chart.UPSTREAM_UNITS
        self.assertEqual(len(units), 16)
        self.assertEqual(len({Path(p).stem for p in units}), len(units))
        for name in ('chcanv', 'glChartCanvas', 'route_gui', 'route_point_gui', 'waypointman_gui', 'ais', 'piano', 's52plib', 'DepthFont'):
            self.assertIn(name, {Path(p).stem for p in chart.UPSTREAM_UNITS})
        for path in chart.LOCAL_UNITS:
            self.assertTrue((ROOT / path).is_file(), path)

    def test_mode_conflicts_rejected_before_platform_or_side_effects(self):
        for flag in ('--floating-surface-only', '--ui', '--prototype-proof',
                     '--legacy-control', '--settings-touch-only', '--settings-component',
                     '--chart-presentation-component', '--energy-component'):
            result = subprocess.run([sys.executable, ROOT / 'tools/test-windows-changed-units.py',
                                     '--chart-units-only', flag], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('--chart-units-only cannot combine', result.stderr)


if __name__ == '__main__':
    unittest.main()
