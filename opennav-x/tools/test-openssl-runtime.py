#!/usr/bin/env python3
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('probe', Path(__file__).with_name('verify-openssl-runtime.py'))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
CPU_A = '0xfffa32035f8bffff:0x1a415fe6f1bf2fb9:0x00001c3023d04010:0x0000000000000000:0x0000000000000000'
CPU_B = '0xfeda3203078bffff:0x00400684219c07a9:0x0000000000000010:0x0000000000000000:0x0000000000000000'
VERSION = '\r\n'.join(('OpenSSL 3.5.9 29 Sep 2026 (Library: OpenSSL 3.5.9 29 Sep 2026)',
    'built on: Sun Oct  4 20:07:01 2026 UTC', 'platform: VC-WIN32', 'options: bn(64,32)',
    'compiler: cl /MD /O2', 'OPENSSLDIR: "original/ssl"', 'ENGINESDIR: "original/engines"',
    'MODULESDIR: "original/modules"', 'Seeding source: os-specific',
    'CPUINFO: OPENSSL_ia32cap=' + CPU_A))


class RuntimeIdentity(unittest.TestCase):
    def test_original_and_host_cpu_only_change_keep_original_evidence(self):
        self.assertFalse(probe.compare(VERSION, VERSION, {})['cpuChanged'])
        changed = VERSION.replace(CPU_A, CPU_B)
        report = probe.compare(VERSION, changed, {})
        self.assertTrue(report['cpuChanged'])
        self.assertEqual(report['producerVersionOutput'], VERSION)
        self.assertEqual(report['consumerVersionOutput'], changed)

    def test_every_immutable_field_drift_is_refused(self):
        for line in VERSION.splitlines()[:-1]:
            with self.subTest(line=line), self.assertRaises(ValueError):
                probe.compare(VERSION, VERSION.replace(line, line + ' changed'), {})

    def test_missing_duplicate_reordered_unknown_and_malformed_cpu_refused(self):
        variants = [VERSION.rsplit('\r\n', 1)[0], VERSION + '\r\n' + VERSION.splitlines()[-1],
                    '\r\n'.join(reversed(VERSION.splitlines())), VERSION + '\r\nunknown',
                    VERSION.replace(CPU_A, '0x123'), VERSION.replace(CPU_A, CPU_A + ' env:0'),
                    VERSION + '\r\n', VERSION.replace('VC-WIN32', 'VC-WIN64')]
        for value in variants:
            with self.subTest(value=value), self.assertRaises(ValueError):
                probe.compare(VERSION, value, {})

    def test_cpu_environment_override_refused_even_empty_or_mixed_case(self):
        for key in ('OPENSSL_ia32cap', 'openssl_IA32CAP'):
            for value in ('', '0'):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    probe.compare(VERSION, VERSION, {key: value})

    def test_probe_checks_hashes_before_execution_and_bounds_native_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'bin').mkdir()
            outputs = {}
            for name in ('openssl.exe', 'libcrypto-3.dll', 'libssl-3.dll'):
                data = ('inert fixture ' + name).encode()
                (root / 'bin' / name).write_bytes(data)
                outputs['bin/' + name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
            manifest = root / 'openssl-build.json'
            manifest.write_text(json.dumps(dict(versionOutput=VERSION, outputs=outputs)))
            exe = root / 'bin/openssl.exe'
            with mock.patch.object(probe.subprocess, 'run') as run:
                run.return_value = subprocess.CompletedProcess([], 0, (VERSION + '\r\n').encode(), b'')
                self.assertEqual(probe.verify(manifest, exe, {})['status'], 'passed')
                self.assertEqual(run.call_args.kwargs['timeout'], 10)
                for result in (subprocess.CompletedProcess([], 1, b'', b''),
                               subprocess.CompletedProcess([], 0, b'', b'warning'),
                               subprocess.CompletedProcess([], 0, b'x' * 16385, b'')):
                    run.return_value = result
                    with self.assertRaises(ValueError): probe.verify(manifest, exe, {})
                run.reset_mock()
                exe.write_bytes(b'tampered')
                with self.assertRaises(ValueError): probe.verify(manifest, exe, {})
                run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
