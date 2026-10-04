#!/usr/bin/env python3
"""Offline AIS authority/runtime guards; these are not native runtime proof."""
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


class DependencyAuthorityTests(unittest.TestCase):
    def test_default_preserves_same_job_receipt_authority(self):
        root = Path('/workspace')
        files = {'prefix/file': {'sha256': 'a' * 64, 'bytes': 7}}
        with patch.object(GATE.reuse, 'verify_same_job') as verify, \
                patch.object(GATE.receipt, '_read_json', return_value={'files': files}) as read, \
                patch.object(GATE.bundle_api, 'verify_restored') as cross_run:
            observed, authority = GATE.dependency_inputs(root)
            self.assertEqual(observed, files)
            self.assertEqual(authority, {'mode': 'same-job-receipt'})
            verify.assert_called_once_with(root)
            read.assert_called_once_with(root / GATE.reuse.RECEIPT)
            cross_run.assert_not_called()

    def test_cross_run_records_distinct_authority_without_same_job_receipt(self):
        root, bundle, provenance = map(Path, ('/workspace', '/bundle', '/authenticated.json'))
        document = {'files': {'prefix/file': {'sha256': 'a' * 64, 'bytes': 7}},
                    'producer': {'runId': '123', 'headSha': 'b' * 40},
                    'fingerprint': {'sha256': 'c' * 64}, 'toolchainSha256': 'd' * 64}
        with patch.object(GATE.bundle_api, 'verify_restored', return_value=document) as verify, \
                patch.object(GATE.reuse, 'verify_same_job') as same_job, \
                patch.object(GATE.receipt, '_read_json') as receipt_read:
            files, authority = GATE.dependency_inputs(root, bundle, provenance)
            self.assertEqual(files, document['files'])
            self.assertEqual(authority['mode'], 'cross-run-bundle')
            self.assertEqual(authority['producer'], document['producer'])
            self.assertEqual(authority['fingerprint'], document['fingerprint'])
            verify.assert_called_once_with(root, bundle, provenance)
            same_job.assert_not_called()
            receipt_read.assert_not_called()

    def test_bundle_failure_never_falls_back_to_same_job(self):
        for message in ('artifact provenance invalid', 'toolchain changed', 'restored inventory changed'):
            with self.subTest(message=message), \
                    patch.object(GATE.bundle_api, 'verify_restored', side_effect=ValueError(message)), \
                    patch.object(GATE.reuse, 'verify_same_job') as same_job:
                with self.assertRaisesRegex(ValueError, message):
                    GATE.dependency_inputs(Path('/workspace'), Path('/bundle'), Path('/authority'))
                same_job.assert_not_called()

    def test_restored_prefix_tampering_is_rejected_before_compilation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            prefix = root / 'build/windows-openssl-3.5.9/install'
            prefix.mkdir(parents=True)
            source = prefix / 'openssl-build.json'
            source.write_text('original')
            workflow = root / GATE.bundle_api.WORKFLOW
            workflow.parent.mkdir(parents=True)
            workflow.write_text('inert producer workflow')
            roots = [prefix.relative_to(root).as_posix(), GATE.bundle_api.WORKFLOW]
            expected = GATE.receipt._inventory(root, roots)
            document = {'roots': roots, 'files': expected}
            source.write_text('modified')
            with patch.object(GATE.bundle_api, 'verify_bundle', return_value=document), \
                    patch.object(GATE.bundle_api, 'verify_producers') as producers:
                with self.assertRaisesRegex(ValueError, 'payload changed'):
                    GATE.dependency_inputs(root, Path('/bundle'), Path('/authority'))
                producers.assert_not_called()

    def test_bundle_arguments_are_paired_and_default_is_unchanged(self):
        args = GATE.arguments([])
        self.assertIsNone(args.dependency_bundle)
        self.assertIsNone(args.dependency_bundle_provenance)
        args = GATE.arguments(['--dependency-bundle', 'bundle',
                               '--dependency-bundle-provenance', 'authority'])
        self.assertEqual(args.dependency_bundle, Path('bundle'))
        for argv in (['--dependency-bundle', 'bundle'],
                     ['--dependency-bundle-provenance', 'authority']):
            with self.subTest(argv=argv), patch('sys.stderr'), self.assertRaises(SystemExit):
                GATE.arguments(argv)
        with self.assertRaisesRegex(ValueError, 'supplied together'):
            GATE.dependency_inputs(Path('/workspace'), Path('/bundle'))

    def test_native_reprobe_uses_driver_verify_only_without_gui_options(self):
        bundle, provenance, log = map(Path, ('/bundle', '/authority', '/reprobe.log'))
        with patch.object(GATE, 'run') as run:
            GATE.reprobe_bundle_inputs(bundle, provenance, log)
        command = run.call_args.args[0]
        self.assertEqual(command[:3], ['pwsh', '-NoProfile', '-File'])
        self.assertEqual(command[3], GATE.ROOT / 'tools/build-pristine-windows.ps1')
        self.assertIn('-VerifyDependencyBundleOnly', command)
        self.assertIn('-Integration', command)
        self.assertNotIn('-Production', command)
        self.assertNotIn('-ReuseVerifiedDependencies', command)
        self.assertEqual(command[-4:], ['-DependencyBundle', bundle,
                                        '-DependencyBundleProvenance', provenance])


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
