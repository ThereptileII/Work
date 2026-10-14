#!/usr/bin/env python3
"""Portable inert tests; never create the real Windows ProgramData profile."""
from contextlib import redirect_stderr, redirect_stdout
import configparser
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location('prepare_profile',
    Path(__file__).with_name('prepare-installed-updater-profile-windows.py'))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class InertNative:
    profile = helper.phase.Native.profile

    def __init__(self, root):
        self.profile_root = root / 'opencpn'
        self.p = {'settings': helper.SETTINGS}
        self.events, self.blocked = [], False

    def no_existing_clients(self):
        helper.require(not self.blocked)

    def check(self):
        self.no_existing_clients()
        helper.phase.updater._plain(self.profile_root, allow_missing=True)

    def create_profile(self):
        self.check()
        self.profile_root.mkdir()
        self.events.append('create')

    def write(self, name, raw):
        self.check()
        with (self.profile_root / name).open('xb') as stream:
            stream.write(raw)
        self.events.append(name)


class Tests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.native = InertNative(self.root)
        self.files = helper.contents()
        self.environment = dict(GITHUB_ACTIONS='true', RUNNER_OS='Windows',
            RUNNER_ENVIRONMENT='github-hosted', GITHUB_REPOSITORY='ThereptileII/Work',
            GITHUB_EVENT_NAME='workflow_dispatch')

    def prepare(self):
        return helper._prepare(self.native, self.files)

    def test_new_profile_uses_real_ini_observer_and_controlled_navigation(self):
        result = self.prepare()
        root = self.native.profile_root
        self.assertEqual(result['state'], 'disposable-profile-created')
        self.assertEqual(result['preservation'], self.native.profile())
        self.assertEqual(result['preservation']['navigationStore'], 'xml')
        self.assertEqual(result['preservation']['navigationSha256'], helper.phase.CONTROLLED_NAV_SHA256)
        self.assertEqual(result['preservation']['navigationMigrationFixtureSha256'], helper.phase.CONTROLLED_XML_SHA256)
        self.assertEqual((root / 'navobj.xml').read_bytes(), self.files['navobj.xml'])
        self.assertFalse((root / 'opencpn.conf').exists())
        self.assertFalse((root / 'navobj.db').exists())
        self.assertEqual(result['preserveSettings'], helper.SETTINGS)
        for name in ('nativeInstalledAcceptance', 'nativeProductAcceptance', 'customerUacAcceptance'):
            self.assertIs(result[name], False)

    def test_connections_plugins_online_sources_disabled_without_fake_version(self):
        self.prepare()
        config = configparser.RawConfigParser(interpolation=None)
        config.read(self.native.profile_root / 'opencpn.ini')
        connection = config.get('Settings/NMEADataSource', 'DataConnections')
        self.assertEqual(connection, helper.fixtures.CONNECTION)
        self.assertEqual(connection.split(';')[17], '0')
        self.assertNotIn('|', connection)
        self.assertFalse(config.has_option('Settings', 'ConfigVersionString'))
        self.assertFalse(config.has_section('OpenNav'))  # No configured vessel/control settings.
        for name in helper.PLUGINS:
            self.assertFalse(config.getboolean('PlugIns/' + name, 'bEnabled'))
        for name in ('OpenNav/OnlineAIS/v1', 'OpenNav/Weather/v1'):
            self.assertFalse(config.getboolean(name, 'Enabled'))

    def test_existing_empty_profile_refuses_without_writing(self):
        self.native.profile_root.mkdir()
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(list(self.native.profile_root.iterdir()), [])
        self.assertEqual(self.native.events, [])

    def test_real_existing_profile_all_bytes_preserved(self):
        root = self.native.profile_root
        root.mkdir()
        originals = {'opencpn.ini': b'private settings', 'navobj.db': b'private navigation',
                     'route.gpx': b'private route', 'OPENNAV_TEST_PROFILE': b'old fixture'}
        for name, raw in originals.items():
            (root / name).write_bytes(raw)
        before = {p.name: (p.stat().st_ino, p.read_bytes()) for p in root.iterdir()}
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(before, {p.name: (p.stat().st_ino, p.read_bytes()) for p in root.iterdir()})
        self.assertEqual(self.native.events, [])

    def test_existing_file_or_dangling_profile_link_refuses(self):
        for kind in ('file', 'dangling-link'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                self.native = InertNative(Path(temporary))
                path = self.native.profile_root
                if kind == 'file':
                    path.write_bytes(b'preserved')
                else:
                    path.symlink_to(Path(temporary) / 'absent', target_is_directory=True)
                with self.assertRaises(ValueError):
                    self.prepare()
                self.assertTrue(os.path.lexists(path))
                self.assertEqual(self.native.events, [])

    def test_redirected_parent_and_profile_preserve_foreign_tree(self):
        target = self.root / 'foreign'
        target.mkdir()
        (target / 'secret').write_bytes(b'preserved')
        for kind in ('parent', 'profile'):
            with self.subTest(kind=kind):
                link = self.root / ('alias' if kind == 'parent' else 'opencpn')
                link.symlink_to(target, target_is_directory=True)
                self.native = InertNative(link if kind == 'parent' else self.root)
                with self.assertRaises(ValueError):
                    self.prepare()
                self.assertEqual({p.name for p in target.iterdir()}, {'secret'})
                self.assertEqual((target / 'secret').read_bytes(), b'preserved')
                self.assertEqual(self.native.events, [])

    def test_reparse_flag_refuses_before_create(self):
        real_lstat = Path.lstat
        def lstat(path, *args, **kwargs):
            result = real_lstat(path, *args, **kwargs)
            if path == self.root:
                class Reparse:
                    st_mode = result.st_mode
                    st_file_attributes = 0x400
                return Reparse()
            return result
        with patch.object(Path, 'lstat', lstat), self.assertRaises(ValueError):
            self.prepare()
        self.assertFalse(self.native.profile_root.exists())

    def test_running_client_refuses_before_create(self):
        self.native.blocked = True
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertFalse(self.native.profile_root.exists())

    def test_native_check_uses_stable_identity_not_directory_size(self):
        native = object.__new__(helper.Native)  # No Windows API is loaded/called.
        native.root = self.root
        identity = (123, b'original-file-id')
        native.directories = [(self.root, 42, identity)]
        with patch.object(helper.phase.Native, 'folder', return_value=self.root), \
             patch.object(native, 'no_existing_clients'), \
             patch.object(native, '_check_file', side_effect=[(identity, 4096), (identity, 8192)]), \
             patch.object(native, '_open', return_value=43), \
             patch.object(native, 'close_handle', return_value=True, create=True) as close:
            native.check()
            close.assert_called_once_with(43)
        with patch.object(helper.phase.Native, 'folder', return_value=self.root), \
             patch.object(native, 'no_existing_clients'), \
             patch.object(native, '_check_file', return_value=((123, b'replaced-file-id'), 4096)), \
             patch.object(native, '_open') as reopen, self.assertRaises(ValueError):
            native.check()
        reopen.assert_not_called()

    def test_native_directory_reader_refusal_is_not_projected_away(self):
        native = object.__new__(helper.Native)
        with patch.object(native, '_check_file', side_effect=ValueError('reparse/type/path refusal')), \
             self.assertRaises(ValueError):
            native._directory_identity(42, self.root)

    def test_partial_failure_preserves_created_state_and_never_retries(self):
        original = self.native.write
        def fail(name, raw):
            if name == 'navobj.xml':
                raise OSError('private failure')
            original(name, raw)
        with patch.object(self.native, 'write', fail), self.assertRaises(OSError):
            self.prepare()
        root = self.native.profile_root
        before = {p.name: p.read_bytes() for p in root.iterdir()}
        self.assertEqual(set(before), {'OPENNAV_TEST_PROFILE'})
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(before, {p.name: p.read_bytes() for p in root.iterdir()})

    def test_changed_or_hardlinked_fixture_refuses(self):
        source = self.root / 'fixture.gpx'
        fixture_root = self.root / 'tests/fixtures'
        fixture_root.mkdir(parents=True)
        source.write_bytes(self.files['navobj.xml'])
        fixture = fixture_root / 'mode-persistence.gpx'
        os.link(source, fixture)
        with patch.object(helper.fixtures, 'ROOT', self.root), self.assertRaises(ValueError):
            helper.contents()
        fixture.unlink()
        fixture.write_bytes(b'<gpx/>')
        with patch.object(helper.fixtures, 'ROOT', self.root), self.assertRaises(ValueError):
            helper.contents()

    def test_host_guards_before_files_or_native_effects(self):
        cases = [(dict(self.environment), 'linux', helper.OPT_IN),
                 (dict(self.environment), 'win32', 'wrong')]
        for key in self.environment:
            changed = dict(self.environment)
            changed[key] = 'wrong'
            cases.append((changed, 'win32', helper.OPT_IN))
        for environment, platform, opt_in in cases:
            with self.subTest(platform=platform, environment=environment, opt_in=opt_in), \
                 patch.dict(helper.os.environ, environment, clear=True), \
                 patch.object(helper.sys, 'platform', platform), \
                 patch.object(helper, 'contents') as contents, \
                 patch.object(helper, 'Native') as native, self.assertRaises(ValueError):
                helper.prepare(opt_in)
            contents.assert_not_called()
            native.assert_not_called()

    def test_cli_has_no_path_override_duplicate_or_failure_details(self):
        for args in (['--profile', str(self.root), '--disposable-opt-in', helper.OPT_IN],
                     ['--disposable-opt-in', helper.OPT_IN, '--disposable-opt-in', helper.OPT_IN],
                     ['--disposable-opt-in', 'wrong']):
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr), patch.object(helper, 'Native') as native:
                self.assertEqual(helper.main(args), 1)
            native.assert_not_called()
            receipt = json.loads(stdout.getvalue())
            self.assertEqual(receipt['state'], 'refused')
            self.assertIn(receipt['stage'], helper.STAGES)
            self.assertIsNone(receipt['win32Error'])
            self.assertIsNone(receipt['ntstatus'])
            self.assertLessEqual(len(stdout.getvalue()), 1025)
            self.assertEqual(stderr.getvalue(), helper.FAILURE + '\n')

    def test_failure_receipt_contains_only_stage_unsigned_codes_and_false_flags(self):
        error = OSError('PRIVATE PATH AND CREDENTIAL')
        error.winerror = 5
        for stage in helper.STAGES:
            refusal = helper.Refused(stage, error, ntstatus=0xc0000022)
            stdout, stderr = io.StringIO(), io.StringIO()
            with patch.object(helper, 'prepare', side_effect=refusal), redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(helper.main(['--disposable-opt-in', helper.OPT_IN]), 1)
            self.assertEqual(json.loads(stdout.getvalue()), dict(schema=1, state='refused', stage=stage,
                win32Error=5, ntstatus=0xc0000022, nativeInstalledAcceptance=False,
                customerUacAcceptance=False, nativeProductAcceptance=False))
            self.assertNotIn('PRIVATE', stdout.getvalue() + stderr.getvalue())
            self.assertEqual(len(stdout.getvalue().splitlines()), 1)
        for value in (-1, 1 << 32, True, 'PRIVATE'):
            error.winerror = value
            record = helper.failure_receipt(helper.Refused('PRIVATE', error, ntstatus=value))
            self.assertEqual(record['stage'], 'arguments')
            self.assertIsNone(record['win32Error'])
            self.assertIsNone(record['ntstatus'])

    def test_native_create_refusal_retains_only_unsigned_ntstatus(self):
        native = object.__new__(helper.Native)
        native.nt, native.handles = object(), []
        with patch.object(native, '_fn', return_value=lambda *args: -1073741790), \
             self.assertRaises(helper.Refused) as refused:
            native._create(42, 'opencpn', True)
        self.assertEqual(helper.failure_receipt(refused.exception)['ntstatus'], 0xc0000022)
        self.assertEqual(refused.exception.stage, 'native-create')

    def test_cleanup_cannot_replace_original_failure_stage_or_emit_success(self):
        error = OSError('PRIVATE OBSERVER FAILURE')
        error.winerror = 5
        def observe(native, files):
            native.stage = 'fixture-observation'
            raise error
        def close():
            self.native.stage = 'cleanup'
        with patch.object(helper, 'host'), patch.object(helper, 'contents', return_value=self.files), \
             patch.object(helper, 'Native', return_value=self.native), patch.object(helper, '_prepare', observe), \
             patch.object(self.native, 'close', side_effect=close, create=True), \
             self.assertRaises(helper.Refused) as refused:
            helper.prepare(helper.OPT_IN)
        self.assertEqual(refused.exception.stage, 'fixture-observation')
        self.assertEqual(refused.exception.winerror, 5)
        with patch.object(helper, 'host'), patch.object(helper, 'contents', return_value=self.files), \
             patch.object(helper, 'Native', return_value=self.native), patch.object(helper, '_prepare', return_value={}), \
             patch.object(self.native, 'close', side_effect=error, create=True), \
             self.assertRaises(helper.Refused) as refused:
            helper.prepare(helper.OPT_IN)
        self.assertEqual(refused.exception.stage, 'cleanup')


if __name__ == '__main__':
    unittest.main()
