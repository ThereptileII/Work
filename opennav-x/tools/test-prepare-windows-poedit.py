#!/usr/bin/env python3
"""Inert acquisition-policy tests: no network, installer or tool execution."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('poedit_preparation', Path(__file__).with_name('prepare-windows-poedit.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


class PreparationTests(unittest.TestCase):
    def setUp(self):
        work = tempfile.TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name).resolve()
        self.evidence = self.root / 'evidence'
        self.curl = self.root / 'Windows/System32/curl.exe'
        self.curl.parent.mkdir(parents=True)
        self.curl.write_bytes(b'inert curl fixture')
        self.payload = b'inert installer fixture'
        self.expected = {'bytes': len(self.payload), 'sha256': hashlib.sha256(self.payload).hexdigest()}
        for fixture in (patch.object(p, 'require_ci'), patch.object(p, 'INSTALLER', self.expected),
                        patch.dict(os.environ, {'SystemRoot': str(self.root / 'Windows')})):
            fixture.start()
            self.addCleanup(fixture.stop)
        self.discovery = patch.object(p.gettext, 'discover', return_value=(None, ['missing pair'])).start()
        self.addCleanup(patch.stopall)
        self.calls = []

    def native(self, command, timeout, log):
        self.calls.append((command, timeout, log))
        if Path(command[0]).name == 'curl.exe':
            Path(command[command.index('--output') + 1]).write_bytes(self.payload)
        return {'exitCode': 0, 'timedOut': False}, ''

    def ensure(self, receipt, allow_install):
        self.assertFalse(allow_install)
        receipt.write_text('{"status":"passed"}\n')

    def run_prepare(self, native=None, ensure=None):
        with patch.object(p.gettext, 'native', side_effect=native or self.native), \
                patch.object(p.gettext, 'ensure', side_effect=ensure or self.ensure) as final_probe:
            try:
                return p.prepare(self.evidence)
            finally:
                self.final_probe_count = final_probe.call_count

    def failed_receipt(self, phase):
        receipt = json.loads((self.evidence / 'receipt.json').read_text())
        self.assertEqual(receipt['status'], 'failed')
        self.assertEqual(receipt['phase'], phase)
        self.assertFalse(receipt['nativeProductAcceptance'])
        self.assertNotIn('gettextReceipt', receipt)

    def test_verified_installer_then_fresh_native_probe(self):
        state = self.run_prepare()
        self.assertEqual(state['status'], 'passed')
        self.assertEqual(self.final_probe_count, 1)
        self.assertEqual(state['installer']['sha256'], self.expected['sha256'])
        download, install = self.calls
        self.assertEqual(download[0][0], self.curl)
        self.assertIn('--fail', download[0])
        for key, value in (('--proto', '=https'), ('--proto-redir', '=https'),
                           ('--connect-timeout', '20'), ('--max-time', '120'),
                           ('--max-filesize', str(len(self.payload)))):
            self.assertEqual(download[0][download[0].index(key) + 1], value)
        self.assertEqual(download[1], 150)
        self.assertEqual(install[0][1:], ['/verysilent', '/norestart'])
        self.assertEqual(install[1], 180)

    def test_usable_known_path_skips_download_and_install(self):
        self.discovery.return_value = ({'directory': 'known Poedit pair'}, [])
        state = self.run_prepare()
        self.assertEqual(state['origin'], 'preinstalled-known-Poedit-path')
        self.assertEqual(self.calls, [])
        self.assertEqual(self.final_probe_count, 1)

    def test_bad_digest_never_executes_installer(self):
        self.payload = b'x' * len(self.payload)
        with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
            self.run_prepare()
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.final_probe_count, 0)
        self.failed_receipt('download')

    def test_wrong_size_never_executes_installer(self):
        self.payload += b'truncated or excessive payload'
        with self.assertRaisesRegex(RuntimeError, 'size/checksum'):
            self.run_prepare()
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.final_probe_count, 0)

    def test_failed_download_never_accepts_matching_leftover(self):
        def native(command, timeout, log):
            self.native(command, timeout, log)
            return {'exitCode': 22}, ''
        with self.assertRaisesRegex(RuntimeError, 'download failed'):
            self.run_prepare(native)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.final_probe_count, 0)
        self.failed_receipt('download')

    def test_download_timeout_has_no_install_or_acceptance(self):
        def native(command, timeout, log):
            self.native(command, timeout, log)
            raise p.gettext.PrerequisiteDeadline('owned download timed out')
        with self.assertRaises(p.gettext.PrerequisiteDeadline):
            self.run_prepare(native)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.final_probe_count, 0)
        self.failed_receipt('download')

    def test_nonzero_installer_including_reboot_code_never_probes_leftovers(self):
        for exit_code in (1, 3010):
            with self.subTest(exit_code=exit_code):
                self.evidence = self.root / ('exit-' + str(exit_code))
                def native(command, timeout, log):
                    result = self.native(command, timeout, log)
                    return result if Path(command[0]).name == 'curl.exe' else ({'exitCode': exit_code}, '')
                with self.assertRaisesRegex(RuntimeError, 'nonzero exit'):
                    self.run_prepare(native)
                self.assertEqual(self.final_probe_count, 0)
                self.failed_receipt('install')

    def test_installer_timeout_has_no_retry_or_leftover_acceptance(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            if Path(command[0]).name != 'curl.exe':
                # The reused native helper retains this record before raising.
                log.with_suffix('.json').write_text('{"timedOut":true}\n')
                raise p.gettext.PrerequisiteDeadline('owned installer timed out')
            return result
        with self.assertRaises(p.gettext.PrerequisiteDeadline):
            self.run_prepare(native)
        self.assertEqual(len(self.calls), 2)
        self.assertEqual(self.final_probe_count, 0)
        self.assertTrue(json.loads((self.evidence / 'install.json').read_text())['timedOut'])
        self.failed_receipt('install')

    def test_initial_tool_timeout_never_starts_acquisition(self):
        self.discovery.side_effect = p.gettext.PrerequisiteDeadline('existing tool timed out')
        with self.assertRaises(p.gettext.PrerequisiteDeadline):
            self.run_prepare()
        self.assertEqual(self.calls, [])
        self.assertEqual(self.final_probe_count, 0)
        self.failed_receipt('initial-probe')

    def test_installer_change_after_execution_is_rejected(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            if Path(command[0]).name != 'curl.exe':
                Path(command[0]).write_bytes(b'changed')
            return result
        with self.assertRaisesRegex(RuntimeError, 'size/checksum'):
            self.run_prepare(native)
        self.assertEqual(self.final_probe_count, 0)
        self.failed_receipt('install')

    def test_zero_install_exit_without_usable_tools_still_fails(self):
        def missing(receipt, allow_install):
            self.assertFalse(allow_install)
            raise RuntimeError('usable tool pair missing')
        with self.assertRaisesRegex(RuntimeError, 'tool pair missing'):
            self.run_prepare(ensure=missing)
        self.failed_receipt('native-verification')

    def test_existing_evidence_is_not_overwritten_or_reused(self):
        self.evidence.mkdir()
        with self.assertRaises(FileExistsError):
            self.run_prepare()
        self.assertEqual(self.calls, [])
        self.assertFalse((self.evidence / 'receipt.json').exists())


class PlatformTests(unittest.TestCase):
    def test_refuses_non_ci_non_windows_or_wrong_host_architecture(self):
        expected = {'GITHUB_ACTIONS': 'true', 'RUNNER_OS': 'Windows', 'RUNNER_ARCH': 'X64'}
        with patch.dict(os.environ, expected, clear=True), patch.object(p.sys, 'platform', 'win32'):
            p.require_ci()
            for key in expected:
                with patch.dict(os.environ, {key: 'unqualified'}):
                    with self.assertRaisesRegex(RuntimeError, 'disposable Windows'):
                        p.require_ci()
        with patch.dict(os.environ, expected, clear=True), patch.object(p.sys, 'platform', 'linux'):
            with self.assertRaisesRegex(RuntimeError, 'disposable Windows'):
                p.require_ci()


if __name__ == '__main__':
    unittest.main()
