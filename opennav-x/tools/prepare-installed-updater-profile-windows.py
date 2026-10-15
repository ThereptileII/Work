#!/usr/bin/env python3
"""Create only a fresh disposable hosted Windows broker profile; never repair one."""
import argparse
import configparser
import ctypes as C
from ctypes import wintypes as W
import hashlib
import importlib.util
import json
import ntpath
import os
from pathlib import Path
import re
import sys

import installed_customer_launch as launch


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


phase = module('test-installed-protected-updater-windows')
fixtures = module('profile-fixtures')
OPT_IN = 'SCRUM-311-disposable-create-installed-profile'
FAILURE = 'Disposable installed profile refused; preserve existing or partial state. No retry or cleanup.'
SETTINGS = [{'section': 'Settings', 'key': 'ShowStatusBar'},
            {'section': 'Settings/NMEADataSource', 'key': 'DataConnections'}]
PLUGINS = ('chartdldr_pi.dll', 'dashboard_pi.dll', 'grib_pi.dll', 'wmm_pi.dll')
STAGES = frozenset(('arguments', 'host-guard', 'fixture-input', 'known-folder',
    'client-observation', 'canonical-directory', 'profile-absence', 'native-create',
    'file-handle-conversion', 'fixture-write', 'fixture-observation',
    'fixture-verification', 'cleanup'))


class Refused(ValueError):
    def __init__(self, stage, error=None, *, ntstatus=None):
        super().__init__(FAILURE)
        self.stage = stage if type(stage) is str and stage in STAGES else 'arguments'
        value = getattr(error, 'winerror', None)
        self.winerror = value if type(value) is int and 0 <= value <= 0xffffffff else None
        value = ntstatus if ntstatus is not None else getattr(error, 'ntstatus', None)
        self.ntstatus = value if type(value) is int and 0 <= value <= 0xffffffff else None


def failure_receipt(error):
    refused = error if isinstance(error, Refused) else Refused('arguments')
    return dict(schema=1, state='refused', stage=refused.stage,
                win32Error=refused.winerror, ntstatus=refused.ntstatus,
                nativeInstalledAcceptance=False, customerUacAcceptance=False,
                nativeProductAcceptance=False)


def require(value):
    if not value:
        raise ValueError(FAILURE)


def host(environment, platform, opt_in):
    require(platform == 'win32' and opt_in == OPT_IN and
            environment.get('GITHUB_ACTIONS') == 'true' and
            environment.get('RUNNER_OS') == 'Windows' and
            environment.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
            environment.get('GITHUB_REPOSITORY') == 'ThereptileII/Work' and
            environment.get('GITHUB_EVENT_NAME') == 'workflow_dispatch')


def contents():
    source = fixtures.ROOT / 'tests/fixtures/mode-persistence.gpx'
    phase.updater._plain(source)
    entry = source.stat()
    require(source.is_file() and entry.st_nlink == 1 and 0 < entry.st_size <= 16384)
    raw = source.read_bytes()
    require(hashlib.sha256(raw).hexdigest() in
            (phase.CONTROLLED_XML_SHA256, phase.CONTROLLED_XML_CRLF_SHA256))
    # No version stamp suppresses the real application's changed-version warning.
    ini = ('[Settings]\nNavMessageShown=1\nShowStatusBar=1\nShowMenuBar=1\n'
           '[Settings/NMEADataSource]\nDataConnections=' + fixtures.CONNECTION + '\n'
           '[OpenNav/OnlineAIS/v1]\nEnabled=0\n'
           '[OpenNav/Weather/v1]\nEnabled=0\n')
    for plugin in PLUGINS:
        ini += '[PlugIns/' + plugin + ']\nbEnabled=0\n'
    return {'OPENNAV_TEST_PROFILE': b'Disposable disconnected installed updater test.\n',
            'navobj.xml': raw, 'opencpn.ini': ini.encode('ascii')}


class Native:
    """Retain real known-folder ancestry; create children relative to its HANDLE.

    Reuse the maintained native directory identity/canonical-path reader only,
    without constructing its diagnostic-file writer or touching account policy.
    """
    _fn = staticmethod(launch._Native._fn)
    _ok = staticmethod(launch._Native._ok)
    _open = launch._ProfileCapabilityFile._open
    _check_file = launch._ProfileCapabilityFile._check_file
    no_existing_clients = phase.Native.no_existing_clients
    profile = phase.Native.profile

    def __init__(self):
        self.stage = 'known-folder'
        self.handles, self.streams, self.directories = [], [], []
        self.kernel = C.WinDLL('kernel32.dll', use_last_error=True, winmode=0x800)
        self.nt = C.WinDLL('ntdll.dll', use_last_error=True, winmode=0x800)
        self.w, self.ui = W, phase.module('windows-ui')
        self.p = {'settings': SETTINGS}
        self.close_handle = self._fn(self.kernel, 'CloseHandle', W.BOOL, W.HANDLE)
        try:
            self.root = phase.Native.folder(self, phase.PROGRAM_DATA)
            text = str(self.root)
            require(re.fullmatch(r'[A-Za-z]:\\[^\r\n\x00:/]+', text) and
                    ntpath.normpath(text) == text)
            for path in reversed((self.root, *self.root.parents)):
                self._pin(path)
            self.profile_root = self.root / 'opencpn'
            self.check()
        except BaseException as error:
            failure = Refused(self.stage, error)
            try:
                self.close()
            except BaseException as cleanup:
                raise Refused('cleanup', cleanup) from None
            raise failure from None

    def _pin(self, path, handle=None):
        self.stage = 'canonical-directory'
        if handle is None:
            handle = self._open(str(path), 0x80, 3, 3, 0x02200000)
        self.handles.append(handle)
        identity = self._directory_identity(handle, path)
        self.directories.append((path, handle, identity))

    def _directory_identity(self, handle, path):
        # The maintained reader still verifies type, reparse/delete state and
        # canonical path. Its second return value is content size, not identity.
        return self._check_file(handle, str(path), directory=True)[0]

    def check(self):
        self.stage = 'known-folder'
        require(phase.Native.folder(self, phase.PROGRAM_DATA) == self.root)
        self.stage = 'client-observation'
        self.no_existing_clients()
        self.stage = 'canonical-directory'
        for path, handle, identity in self.directories:
            require(self._directory_identity(handle, path) == identity)
            current = self._open(str(path), 0x80, 3, 3, 0x02200000)
            try:
                require(self._directory_identity(current, path) == identity)
            finally:
                require(self.close_handle(current))

    def _create(self, parent, name, directory):
        # NtCreateFile FILE_CREATE is an atomic create-only operation relative to
        # the retained original directory, even if a DOS mapping changes.
        require(name in ('opencpn', 'OPENNAV_TEST_PROFILE', 'navobj.xml', 'opencpn.ini'))
        class Unicode(C.Structure):
            _fields_ = [('length', W.USHORT), ('maximum', W.USHORT), ('buffer', W.LPWSTR)]
        class Attributes(C.Structure):
            _fields_ = [('length', W.ULONG), ('root', W.HANDLE), ('name', C.POINTER(Unicode)),
                        ('flags', W.ULONG), ('security', C.c_void_p), ('qos', C.c_void_p)]
        class Status(C.Structure):
            _fields_ = [('status', C.c_void_p), ('information', C.c_size_t)]
        text = C.create_unicode_buffer(name)
        string = Unicode(len(name) * 2, (len(name) + 1) * 2, C.cast(text, W.LPWSTR))
        attributes = Attributes(C.sizeof(Attributes), parent, C.pointer(string), 0x40, None, None)
        status, handle = Status(), W.HANDLE()
        call = self._fn(self.nt, 'NtCreateFile', C.c_long, C.POINTER(W.HANDLE), W.DWORD,
                        C.POINTER(Attributes), C.POINTER(Status), C.c_void_p, W.DWORD,
                        W.DWORD, W.DWORD, W.DWORD, C.c_void_p, W.DWORD)
        access = 0x100081 if directory else 0xc0100080
        # FILE_CREATE refuses every existing leaf, including a reparse point;
        # the relative name has no intermediate components to redirect.
        options = 0x20 | (1 if directory else 0x40)
        self.stage = 'native-create'
        result = call(C.byref(handle), access, C.byref(attributes), C.byref(status), None,
                      0x10 if directory else 0x80, 3 if directory else 1, 2, options, None, 0)
        if handle.value:
            self.handles.append(handle.value)
        if result != 0 or not handle.value or status.information != 2:
            raise Refused(self.stage, ntstatus=result & 0xffffffff if result != 0 else None)
        return handle.value

    def create_profile(self):
        self.check()
        self.stage = 'profile-absence'
        phase.updater._plain(self.profile_root, allow_missing=True)
        require(not os.path.lexists(self.profile_root))
        handle = self._create(self.directories[-1][1], 'opencpn', True)
        # _create already owns this original handle, including on later refusal.
        self.stage = 'canonical-directory'
        self.directories.append((self.profile_root, handle,
                                 self._directory_identity(handle, self.profile_root)))
        self.check()
        require(not any(self.profile_root.iterdir()))

    def write(self, name, raw):
        import msvcrt
        self.check()
        handle = self._create(self.directories[-1][1], name, False)
        self.stage = 'file-handle-conversion'
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDWR | os.O_BINARY)
        self.handles.remove(handle)  # descriptor owns it from this point.
        try:
            stream = os.fdopen(descriptor, 'w+b')
        except BaseException:
            os.close(descriptor)
            raise
        self.streams.append(stream)
        self.stage = 'fixture-write'
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
        self.check()

    def close(self):
        self.stage = 'cleanup'
        failed = False
        for stream in reversed(self.streams):
            try:
                stream.close()
            except OSError:
                failed = True
        self.streams.clear()
        for handle in reversed(self.handles):
            failed = not self.close_handle(handle) or failed
        self.handles.clear()
        require(not failed)


def _prepare(native, files):
    """Private seam for inert filesystem tests; production takes no path override."""
    root = native.profile_root
    native.stage = 'profile-absence'
    phase.updater._plain(root.parent)
    phase.updater._plain(root, allow_missing=True)
    require(not os.path.lexists(root))  # Even an existing empty profile refuses.
    native.create_profile()
    for name, raw in files.items():
        native.check()
        native.write(name, raw)
    native.check()
    native.stage = 'fixture-verification'
    require({p.name for p in root.iterdir()} == set(files))
    for name, raw in files.items():
        path = phase.updater._plain(root / name)
        require(path.is_file() and path.stat().st_nlink == 1 and path.read_bytes() == raw)
    native.stage = 'fixture-observation'
    observed = native.profile()  # Existing read-only navigation and .ini observer.
    native.stage = 'fixture-verification'
    require(observed['navigationMigrationFixtureSha256'] == phase.CONTROLLED_XML_SHA256)
    config = configparser.RawConfigParser(strict=True, interpolation=None)
    config.read_string((root / 'opencpn.ini').read_text(encoding='ascii'))
    require(config.get('Settings/NMEADataSource', 'DataConnections') == fixtures.CONNECTION and
            fixtures.CONNECTION.split(';')[17] == '0' and
            all(not config.getboolean('PlugIns/' + p, 'bEnabled') for p in PLUGINS) and
            not config.getboolean('OpenNav/OnlineAIS/v1', 'Enabled') and
            not config.getboolean('OpenNav/Weather/v1', 'Enabled'))
    native.check()
    return dict(schema=1, state='disposable-profile-created', preservation=observed,
                preserveSettings=SETTINGS, nativeInstalledAcceptance=False,
                customerUacAcceptance=False, nativeProductAcceptance=False)


def prepare(opt_in, *, host_mode='github-hosted', journey_config=None, journey_config_sha256=None):
    native, stage = None, 'host-guard'
    try:
        if host_mode == 'github-hosted':
            require(journey_config is None and journey_config_sha256 is None)
            host(os.environ, sys.platform, opt_in)  # Before any reads or native effects.
        else:
            require(opt_in == OPT_IN)
            cli = module('run-installed-updater-journey-windows')
            cli.host(os.environ, sys.platform, host_mode)
            require(journey_config is not None and journey_config_sha256 is not None)
            config = cli.binding._json(cli.original(dict(path=journey_config, sha256=journey_config_sha256), 64 << 10), 64 << 10)
            cli.validate_profile_host(config, os.environ)  # Before Native()/any filesystem effect.
        stage = 'fixture-input'
        files = contents()
        stage = 'known-folder'
        native = Native()
        return _prepare(native, files)
    except BaseException as error:
        if isinstance(error, Refused):
            raise
        raise Refused(native.stage if native is not None else stage, error) from None
    finally:
        if native is not None:
            try:
                native.close()
            except BaseException as error:
                raise Refused('cleanup', error) from None


def main(arguments=None):
    try:
        class Parser(argparse.ArgumentParser):
            def error(self, message):
                raise ValueError(FAILURE)
        parser = Parser(allow_abbrev=False)
        parser.add_argument('--disposable-opt-in', required=True, action=phase.Once)
        parser.add_argument('--host-mode', default='github-hosted', action=phase.Once)
        parser.add_argument('--journey-config', action=phase.Once)
        parser.add_argument('--journey-config-sha256', action=phase.Once)
        args = parser.parse_args(arguments)
        if args.host_mode == 'github-hosted' and args.journey_config is None and args.journey_config_sha256 is None:
            result = prepare(args.disposable_opt_in)
        else:
            result = prepare(args.disposable_opt_in, host_mode=args.host_mode,
                             journey_config=args.journey_config, journey_config_sha256=args.journey_config_sha256)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (Exception, KeyboardInterrupt, SystemExit) as error:
        # Fixed enum and bounded unsigned numbers only; never exception text,
        # account/profile paths, environment, native buffers or submitted inputs.
        receipt = json.dumps(failure_receipt(error), sort_keys=True)
        require(len(receipt) <= 1024)
        print(receipt)
        print(FAILURE, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
