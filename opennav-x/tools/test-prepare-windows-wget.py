#!/usr/bin/env python3
"""Inert Wget acquisition tests; no network or native executable runs."""
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import stat
import struct
import tempfile
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('wget_preparation', Path(__file__).with_name('prepare-windows-wget.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def identity(data):
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


class PreparationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.evidence = self.root / 'evidence'
        self.runner_temp = self.root / 'runner-temp'
        self.runner_temp.mkdir()
        self.github_path = self.root / 'github-path'
        self.github_path.write_text('existing-entry\n')
        self.chocolatey_bin = self.root / 'chocolatey/bin'
        self.chocolatey_bin.mkdir(parents=True)
        (self.chocolatey_bin / 'choco.exe').write_bytes(b'inert existing host tool')
        self.target = self.chocolatey_bin / 'wget.exe'
        self.original_path = str(self.chocolatey_bin) + ';' + str(self.root / 'other-bin')
        self.curl = self.root / 'Windows/System32/curl.exe'
        self.curl.parent.mkdir(parents=True)
        self.curl.write_bytes(b'inert curl fixture')
        self.where = self.curl.with_name('where.exe')
        self.where.write_bytes(b'inert where fixture')
        executable = bytearray(128)
        executable[:2] = b'MZ'
        struct.pack_into('<I', executable, 0x3c, 64)
        executable[64:68] = b'PE\0\0'
        struct.pack_into('<H', executable, 68, 0x8664)
        self.executable = bytes(executable)
        self.archive = self.make_archive()
        for fixture in (patch.object(p, 'require_ci'),
                        patch.object(p, 'CHOCOLATEY_BIN', self.chocolatey_bin),
                        patch.object(p, 'EXECUTABLE', identity(self.executable)),
                        patch.object(p, 'ARCHIVE', identity(self.archive)),
                        patch.dict(os.environ, {'SystemRoot': str(self.root / 'Windows'), 'PATH': self.original_path,
                            'RUNNER_TEMP': str(self.runner_temp), 'GITHUB_PATH': str(self.github_path)})):
            fixture.start()
            self.addCleanup(fixture.stop)
        self.calls = []

    def make_archive(self, names=None, special=None):
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, 'w') as archive:
            for name in names if names is not None else sorted(p.MEMBERS):
                info = zipfile.ZipInfo(name)
                if name == special:
                    info.create_system = 3
                    info.external_attr = (stat.S_IFLNK | 0o777) << 16
                archive.writestr(info, self.executable if name == 'wget.exe' else b'inert vendor document')
        return buffer.getvalue()

    def native(self, command, timeout, log):
        self.calls.append((command, timeout, log))
        if Path(command[0]).name == 'curl.exe':
            Path(command[command.index('--output') + 1]).write_bytes(self.archive)
            return {'exitCode': 0, 'timedOut': False}, ''
        if Path(command[0]).name == 'where.exe':
            return {'exitCode': 0, 'timedOut': False}, str(self.target) + '\n'
        return {'exitCode': 0, 'timedOut': False}, 'GNU Wget 1.25.0 built on mingw32.\n'

    def run_prepare(self, native=None):
        with patch.object(p.native_tools, 'native', side_effect=native or self.native):
            return p.prepare(self.evidence)

    def failed_receipt(self, phase):
        state = json.loads((self.evidence / 'receipt.json').read_text())
        self.assertEqual(state['status'], 'failed')
        self.assertEqual(state['phase'], phase)
        self.assertFalse(state['nativeProductAcceptance'])
        self.assertNotIn('executablePublished', state)
        self.assertEqual(self.github_path.read_text(), 'existing-entry\n')
        self.assertEqual(os.environ['PATH'], self.original_path)

    def test_exclusive_verified_copy_resolves_without_changing_path(self):
        state = self.run_prepare()
        directory = Path(state['directory'])
        self.assertEqual(directory.parent, self.runner_temp)
        self.assertEqual(set(item.name for item in directory.iterdir()), p.MEMBERS)
        self.assertEqual(state['files']['wget.exe'], identity(self.executable))
        self.assertEqual(state['status'], 'passed')
        self.assertEqual(self.github_path.read_text(), 'existing-entry\n')
        self.assertEqual(os.environ['PATH'], self.original_path)
        self.assertTrue(state['pathUnchanged'])
        self.assertEqual(state['inheritedPathSha256'], hashlib.sha256(self.original_path.encode()).hexdigest())
        self.assertEqual(self.target.read_bytes(), self.executable)
        self.assertFalse(list(self.evidence.glob('*.exe')))
        download, probe, resolution = self.calls
        self.assertEqual(download[0][0], self.curl)
        for key, value in (('--proto', '=https'), ('--proto-redir', '=https'),
                           ('--connect-timeout', '20'), ('--max-time', '120'),
                           ('--max-filesize', str(len(self.archive)))):
            self.assertEqual(download[0][download[0].index(key) + 1], value)
        self.assertIn('--fail', download[0])
        self.assertEqual(download[1], 150)
        self.assertEqual(probe[0], [self.target, '--version'])
        self.assertEqual(probe[1], 30)
        self.assertEqual(resolution[0], [self.where, 'wget'])
        self.assertEqual(resolution[1], 30)

    def test_bad_archive_digest_and_size_never_execute_or_publish(self):
        for payload in (b'x' * len(self.archive), self.archive + b'extra'):
            with self.subTest(size=len(payload)):
                self.evidence = self.root / ('bad-' + str(len(payload)))
                def native(command, timeout, log):
                    result = self.native(command, timeout, log)
                    Path(command[command.index('--output') + 1]).write_bytes(payload)
                    return result
                with self.assertRaisesRegex(RuntimeError, 'size/checksum'):
                    self.run_prepare(native)
                self.failed_receipt('extraction')
        self.assertEqual(len(self.calls), 2)

    def test_failed_download_does_not_accept_matching_leftover(self):
        def native(command, timeout, log):
            self.native(command, timeout, log)
            return {'exitCode': 22}, ''
        with self.assertRaisesRegex(RuntimeError, 'download failed'):
            self.run_prepare(native)
        self.assertEqual(len(self.calls), 1)
        self.failed_receipt('download')

    def test_download_timeout_has_no_probe_or_path_publication(self):
        def native(command, timeout, log):
            self.native(command, timeout, log)
            raise p.native_tools.PrerequisiteDeadline('owned download timed out')
        with self.assertRaises(p.native_tools.PrerequisiteDeadline):
            self.run_prepare(native)
        self.assertEqual(len(self.calls), 1)
        self.failed_receipt('download')

    def test_traversal_duplicate_case_collision_and_missing_members_fail(self):
        cases = [sorted(p.MEMBERS) + ['../escape'], sorted(p.MEMBERS) + ['wget.exe'],
                 sorted(p.MEMBERS) + ['WGET.EXE'], sorted(p.MEMBERS - {'COPYING'})]
        for index, names in enumerate(cases):
            with self.subTest(names=names):
                self.evidence = self.root / ('inventory-' + str(index))
                self.archive = self.make_archive(names)
                with patch.object(p, 'ARCHIVE', identity(self.archive)), \
                        self.assertRaisesRegex(RuntimeError, 'member inventory'):
                    self.run_prepare()
                self.failed_receipt('extraction')
        self.assertEqual(len(self.calls), len(cases))

    def test_archive_link_rejected_before_any_member_is_written(self):
        self.archive = self.make_archive(special='COPYING')
        with patch.object(p, 'ARCHIVE', identity(self.archive)), \
                self.assertRaisesRegex(RuntimeError, 'nonregular'):
            self.run_prepare()
        state = json.loads((self.evidence / 'receipt.json').read_text())
        self.assertEqual(list(Path(state['directory']).iterdir()), [])
        self.failed_receipt('extraction')

    def test_changed_extracted_executable_rejected_before_probe(self):
        self.executable += b'changed'
        self.archive = self.make_archive()
        with patch.object(p, 'ARCHIVE', identity(self.archive)), \
                self.assertRaisesRegex(RuntimeError, 'size/checksum'):
            self.run_prepare()
        self.assertEqual(len(self.calls), 1)
        self.failed_receipt('extraction')

    def test_wrong_architecture_rejected_before_probe(self):
        executable = bytearray(self.executable)
        struct.pack_into('<H', executable, 68, 0x14c)
        self.executable = bytes(executable)
        self.archive = self.make_archive()
        with patch.object(p, 'ARCHIVE', identity(self.archive)), \
                patch.object(p, 'EXECUTABLE', identity(self.executable)), \
                self.assertRaisesRegex(RuntimeError, 'Windows x64 PE'):
            self.run_prepare()
        self.assertEqual(len(self.calls), 1)
        self.failed_receipt('extraction')

    def test_wrong_version_and_nonzero_probe_never_publish(self):
        for index, result in enumerate((({'exitCode': 0}, 'GNU Wget 1.21.4 built on mingw32.\n'),
                                       ({'exitCode': 1}, 'GNU Wget 1.25.0 built on mingw32.\n'))):
            with self.subTest(result=result):
                self.evidence = self.root / ('probe-' + str(index))
                def native(command, timeout, log):
                    normal = self.native(command, timeout, log)
                    return normal if Path(command[0]).name == 'curl.exe' else result
                with self.assertRaisesRegex(RuntimeError, 'version probe failed'):
                    self.run_prepare(native)
                self.assertFalse(self.target.exists())
                self.failed_receipt('native-verification')

    def test_probe_timeout_does_not_publish(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            if Path(command[0]).name == 'wget.exe':
                raise p.native_tools.PrerequisiteDeadline('owned probe timed out')
            return result
        with self.assertRaises(p.native_tools.PrerequisiteDeadline):
            self.run_prepare(native)
        self.failed_receipt('native-verification')

    def test_executable_changed_during_probe_is_rejected(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            if Path(command[0]).name == 'wget.exe':
                Path(command[0]).write_bytes(b'changed')
            return result
        with self.assertRaisesRegex(RuntimeError, 'size/checksum'):
            self.run_prepare(native)
        self.failed_receipt('native-verification')

    def test_existing_evidence_is_not_overwritten(self):
        self.evidence.mkdir()
        with self.assertRaises(FileExistsError):
            self.run_prepare()
        self.assertEqual(self.calls, [])

    def test_existing_target_is_never_overwritten_even_when_hash_matches(self):
        for index, data in enumerate((b'unknown existing tool', self.executable)):
            with self.subTest(data=data[:10]):
                self.evidence = self.root / ('existing-' + str(index))
                self.target.write_bytes(data)
                with self.assertRaisesRegex(RuntimeError, 'overwrite an existing'):
                    self.run_prepare()
                self.assertEqual(self.target.read_bytes(), data)
                self.failed_receipt('preparation')
        self.assertEqual(self.calls, [])

    def test_target_created_after_preflight_is_not_overwritten(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            self.target.write_bytes(b'concurrent existing tool')
            return result
        with self.assertRaises(FileExistsError):
            self.run_prepare(native)
        self.assertEqual(self.target.read_bytes(), b'concurrent existing tool')
        self.failed_receipt('exclusive-copy')

    def test_missing_chocolatey_host_tool_stops_before_download(self):
        (self.chocolatey_bin / 'choco.exe').unlink()
        with self.assertRaises(FileNotFoundError):
            self.run_prepare()
        self.assertEqual(self.calls, [])
        self.failed_receipt('preparation')

    def test_existing_directory_must_already_be_on_inherited_path(self):
        with patch.dict(os.environ, {'PATH': str(self.root / 'other-bin')}), \
                self.assertRaisesRegex(RuntimeError, 'not on inherited PATH'):
            self.run_prepare()
        self.assertEqual(self.calls, [])

    def test_other_command_resolution_is_rejected_and_owned_copy_removed(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            return ({'exitCode': 0}, str(self.root / 'shadow/wget.cmd')) if command[0] == self.where else result
        with self.assertRaisesRegex(RuntimeError, 'resolution does not select'):
            self.run_prepare(native)
        self.assertFalse(self.target.exists())
        self.failed_receipt('command-resolution')

    def test_path_mutation_during_probe_is_rejected(self):
        def native(command, timeout, log):
            result = self.native(command, timeout, log)
            if command[0] == self.where:
                os.environ['PATH'] = 'changed-path'
            return result
        with self.assertRaisesRegex(RuntimeError, 'Inherited PATH changed'):
            self.run_prepare(native)
        self.assertFalse(self.target.exists())
        os.environ['PATH'] = self.original_path
        self.failed_receipt('command-resolution')

    def test_relative_temp_directory_is_refused(self):
        with patch.dict(os.environ, {'RUNNER_TEMP': 'relative'}), \
                self.assertRaisesRegex(RuntimeError, 'absolute and plain'):
            self.run_prepare()
        self.assertEqual(self.calls, [])
        self.failed_receipt('preparation')


class PlatformTests(unittest.TestCase):
    def test_refuses_non_ci_non_windows_or_wrong_host_architecture(self):
        expected = {'GITHUB_ACTIONS': 'true', 'RUNNER_OS': 'Windows', 'RUNNER_ARCH': 'X64'}
        with patch.dict(os.environ, expected, clear=True), patch.object(p.sys, 'platform', 'win32'):
            p.require_ci()
            for key in expected:
                with patch.dict(os.environ, {key: 'unqualified'}), \
                        self.assertRaisesRegex(RuntimeError, 'disposable Windows'):
                    p.require_ci()
        with patch.dict(os.environ, expected, clear=True), patch.object(p.sys, 'platform', 'linux'), \
                self.assertRaisesRegex(RuntimeError, 'disposable Windows'):
            p.require_ci()


if __name__ == '__main__':
    unittest.main()
