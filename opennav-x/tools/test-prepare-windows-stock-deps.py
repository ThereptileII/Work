#!/usr/bin/env python3
"""Disposable cache correction; no download, SDK build or application process."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location('stock', Path(__file__).with_name('prepare-windows-stock-deps.py'))
stock = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stock)


class StockTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        data = bytearray(128)
        data[:2] = b'MZ'
        struct.pack_into('<I', data, 60, 64)
        data[64:68] = b'PE\0\0'
        struct.pack_into('<H', data, 68, 0x14c)
        self.expected = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        for prefix in stock.stage.PREFIX.values():
            (self.root / prefix).mkdir(parents=True)
            (self.root / prefix / 'keep').write_bytes(b'authenticated prefix')
        self.prefix = self.write(stock.stage.PREFIX['curl'] + '/bin/libcurl.dll', data)
        self.sentinel = self.write(stock.stage.CACHE + '/libcurl.dll', data)
        self.other = self.write(stock.stage.CACHE + '/custom.dll', b'untouched cache')
        self.manifest = self.write(stock.stage.PREFIX['curl'] + '/curl-build.json', json.dumps({'outputs': {'bin/libcurl.dll': self.expected}}).encode())
        self.batch = self.write(stock.BATCH, b'pinned fixture sentinel logic\r\n')
        patch = mock.patch.object(stock, 'BATCH_SHA256', hashlib.sha256(self.batch.read_bytes().replace(b'\r\n', b'\n')).hexdigest())
        patch.start(); self.addCleanup(patch.stop)
        patch = mock.patch.object(stock.bundle_api, 'verify_restored', return_value={'producer': {'runId': 'fixture'}, 'fingerprint': {'sha256': 'a'*64}})
        self.verify = patch.start(); self.addCleanup(patch.stop)

    def write(self, name, data):
        target = self.root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return target

    def prepare(self):
        return stock.prepare(self.root, self.root / 'bundle', self.root / 'provenance')

    def test_only_verified_disposable_sentinel_removed(self):
        before = stock.prefix_inventory(self.root)
        report = self.prepare()
        self.assertTrue(report['removed'])
        self.assertTrue(report['prefixesUnchanged'])
        self.assertEqual(report['verifiedSentinel'], self.expected)
        self.assertFalse(self.sentinel.exists())
        self.assertEqual(stock.prefix_inventory(self.root), before)
        self.assertEqual(self.other.read_bytes(), b'untouched cache')
        self.verify.assert_called_once()

    def test_complete_stock_closure_keeps_sentinel(self):
        for name in (*stock.STOCK_FILES, 'archive.lib'):
            self.write(stock.stage.CACHE + '/' + name, b'stock presence fixture')
        self.assertFalse(self.prepare()['removed'])
        self.assertTrue(self.sentinel.exists())

    def test_unknown_authority_batch_prefix_cache_or_architecture_refuses(self):
        for failure in ('authority', 'batch', 'prefix', 'cache', 'missing', 'architecture'):
            with self.subTest(failure=failure):
                originals = {p: p.read_bytes() for p in (self.batch, self.prefix, self.sentinel, self.manifest)}
                try:
                    if failure == 'authority': self.verify.side_effect = ValueError('untrusted SDK')
                    elif failure == 'batch': self.batch.write_bytes(b'changed batch')
                    elif failure == 'missing': self.sentinel.unlink()
                    elif failure == 'architecture':
                        data = bytearray(self.prefix.read_bytes()); struct.pack_into('<H', data, 68, 0x8664)
                        self.prefix.write_bytes(data); self.sentinel.write_bytes(data)
                        self.manifest.write_text(json.dumps({'outputs': {'bin/libcurl.dll': {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}}}))
                    else: (self.prefix if failure == 'prefix' else self.sentinel).write_bytes(b'unknown')
                    with self.assertRaises((ValueError, OSError)): self.prepare()
                    if failure != 'missing': self.assertTrue(self.sentinel.exists())
                    self.assertEqual(self.other.read_bytes(), b'untouched cache')
                finally:
                    self.verify.side_effect = None
                    for path, data in originals.items(): path.write_bytes(data)

    def test_native_preflight_orders_stock_restage_configure_and_preserves_prefixes(self):
        spec = importlib.util.spec_from_file_location('native_stock', Path(__file__).with_name('test-windows-stock-configure.py'))
        probe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(probe)
        evidence = self.root / 'evidence/local'
        evidence.mkdir(parents=True)
        lock = {'archives': [{'file': 'wx.7z', 'url': 'https://fixture.invalid/wx',
                             'bytes': 2, 'sha256': hashlib.sha256(b'wx').hexdigest()}]}
        self.write('tools/windows-wx.lock.json', json.dumps(lock).encode())
        calls = []
        def native(command, timeout, log):
            args = [str(v) for v in command]
            calls.append(args[0])
            if args[0] == 'curl.exe': Path(args[args.index('--output') + 1]).write_bytes(b'wx')
            elif args[0] == '7z': self.write('build/integration-source/cache/wxWidgets-3.2.8/include/wx/version.h', b'wx')
            elif any(arg.endswith('windows_gettext.py') for arg in args):
                Path(args[args.index('--receipt') + 1]).write_text(json.dumps({'directory': str(self.root / 'gettext')}))
            elif args[0].endswith('cmd.exe'):
                self.assertFalse(self.sentinel.exists())
                self.assertEqual(args[-1], 'buildwin\\win_deps.bat')
                self.sentinel.write_bytes(b'upstream stock curl')
                for name in (*stock.STOCK_FILES, 'archive.lib'):
                    self.write(stock.stage.CACHE + '/' + name, self.prefix.read_bytes())
                self.write('build/integration-source/cache/OCPNWindowsCoreBuildSupport.zip', b'stock archive fixture')
            elif args[0] == 'cmake':
                self.assertEqual(self.sentinel.read_bytes(), self.prefix.read_bytes())
                self.assertIn('-DSKAGER_OCHARTS_PACKAGE:PATH=', args)
                self.assertEqual(args[args.index('-A')+1], 'Win32')
                self.assertNotIn('--build', args); self.assertNotIn('--install', args)
                build = Path(args[args.index('-B')+1]); build.mkdir()
                (build / 'CMakeCache.txt').write_text('LibArchive_LIBRARY:FILEPATH='+str(self.sentinel.parent / 'archive.lib')+'\nLibArchive_INCLUDE_DIR:PATH='+str(self.sentinel.parent / 'include')+'\nCMAKE_GENERATOR_PLATFORM:INTERNAL=Win32\n')
            else: self.fail('Unexpected native command')
            return {'exitCode': 0}, ''
        def restage(*args):
            calls.append('authenticated-restage')
            self.sentinel.write_bytes(self.prefix.read_bytes())
        before = stock.prefix_inventory(self.root)
        original_path = os.environ.get('PATH')
        arguments = ['probe', '--root', str(self.root), '--bundle', str(self.root/'bundle'), '--provenance', str(self.root/'provenance')]
        with mock.patch.object(probe, 'stock', stock), mock.patch.object(probe.sys, 'platform', 'win32'), mock.patch.object(probe.sys, 'argv', arguments), mock.patch.dict(os.environ, {'GITHUB_ACTIONS': 'true', 'SystemRoot': str(self.root)}), mock.patch.object(probe.windows_gettext, 'native', side_effect=native), mock.patch.object(stock.bundle_api, 'stage_bundle', side_effect=restage):
            probe.main()
        report = json.loads((evidence/'windows-stock-configure/report.json').read_text())
        self.assertTrue(report['passed']); self.assertTrue(report['prefixesUnchanged'])
        self.assertFalse(report['applicationBuilt']); self.assertFalse(report['applicationInstalled'])
        self.assertEqual(calls[-2:], ['authenticated-restage', 'cmake'])
        self.assertEqual(stock.prefix_inventory(self.root), before)
        self.assertEqual(os.environ.get('PATH'), original_path)

    def test_redirected_sentinel_refused(self):
        self.sentinel.unlink()
        try: self.sentinel.symlink_to(self.prefix)
        except OSError: self.skipTest('Disposable symlink unavailable')
        with self.assertRaises(ValueError): self.prepare()
        self.assertTrue(self.sentinel.is_symlink())
        self.assertEqual(stock.receipt._digest(self.prefix), self.expected['sha256'])


if __name__ == '__main__':
    unittest.main()
