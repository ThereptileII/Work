#!/usr/bin/env python3
"""Small offline guards for the same-job AIS runner; not native runtime proof."""
import importlib.util
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('ais_runtime', Path(__file__).with_name('test-ais-runtime-windows.py'))
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)


class GateTests(unittest.TestCase):
    def test_consumer_requires_all_three_private_imports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / 'private/cache/buildwin'
            project = root / 'consumer.vcxproj'
            def write(paths):
                project.write_text('<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003">'
                    '<ItemDefinitionGroup><Link><AdditionalDependencies>' + ';'.join(map(str, paths)) +
                    '</AdditionalDependencies></Link></ItemDefinitionGroup></Project>')
            good = [cache / name for name in ('libssl.lib', 'libcrypto.lib', 'zlib1.lib')]
            write(good)
            GATE.private_links(project, cache)
            for bad in (good[:-1], good + [root / 'old-sdk/libssl.lib'],
                        ['libssl.lib', *good[1:]], [*good[:2], root / 'old-sdk/zlib1.lib']):
                write(bad)
                with self.assertRaisesRegex(ValueError, 'exclusively private'):
                    GATE.private_links(project, cache)

    def test_child_environment_excludes_inherited_tls_paths(self):
        with patch.dict(os.environ, {'SystemRoot': '/Windows', 'PATH': '/old-sdk:/user-tls',
                                     'OPENSSL_CONF': '/old-config', 'OPENSSL_MODULES': '/old-modules'}):
            before = dict(os.environ)
            env = GATE.child_environment(Path('/private'), Path('/runtime'))
            self.assertEqual(env['PATH'].split(os.pathsep),
                list(map(str, map(Path, ['/private/openssl/bin', '/runtime', '/Windows/System32', '/Windows']))))
            self.assertEqual(env['OPENSSL_CONF'], str(Path('/private/openssl/ssl/openssl.cnf')))
            self.assertEqual(env['OPENSSL_MODULES'], str(Path('/private/openssl/lib/ossl-modules')))
            self.assertEqual(dict(os.environ), before)

    def test_pe_architecture_guard_rejects_x64_and_invalid_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'probe.exe'
            data = bytearray(100); data[:2] = b'MZ'
            struct.pack_into('<I', data, 60, 64); data[64:68] = b'PE\0\0'
            struct.pack_into('<H', data, 68, 0x14c)
            path.write_bytes(data); GATE.win32(path)
            struct.pack_into('<H', data, 68, 0x8664)
            path.write_bytes(data)
            with self.assertRaisesRegex(ValueError, 'Not native Win32'): GATE.win32(path)
            path.write_bytes(b'not a binary')
            with self.assertRaisesRegex(ValueError, 'Missing PE'): GATE.win32(path)


if __name__ == '__main__':
    unittest.main()
