#!/usr/bin/env python3
"""Read-only Windows module paths and separate on-disk identities; no closure verdict."""
import argparse
from contextlib import contextmanager
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import sys

from win32_plugin_inventory import plain_path, read_file, stamp

LIMITS = [
    'OS module paths are observations, not plugin activation, provenance/trust or loader-closure acceptance.',
    'SHA-256 identifies separately captured ON-DISK bytes, never mapped image bytes or bytes originally loaded.',
    'Two matching snapshots are not an atomic snapshot or proof that no load/unload occurred between them.',
    'Toolhelp can omit datafile loads, manually mapped images, transient modules and child-process modules.',
    'No dependency resolution, dynamic-call coverage, signatures, ABI, TLS policy or installer ownership is inferred.',
]


def path_key(value):
    return os.path.normcase(os.path.abspath(value))


@contextmanager
def windows_process(pid):
    """Hold one query-only process handle throughout both snapshots and disk reads."""
    if sys.platform != 'win32':
        raise ValueError('Native Windows required; no emulated/native success fallback')
    from ctypes import wintypes as w

    class Module(ctypes.Structure):
        _fields_ = [('dwSize', w.DWORD), ('th32ModuleID', w.DWORD),
                    ('th32ProcessID', w.DWORD), ('GlblcntUsage', w.DWORD),
                    ('ProccntUsage', w.DWORD), ('modBaseAddr', ctypes.c_void_p),
                    ('modBaseSize', w.DWORD), ('hModule', w.HMODULE),
                    ('szModule', w.WCHAR * 256), ('szExePath', w.WCHAR * 260)]

    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    for name, result, args in (
        ('OpenProcess', w.HANDLE, [w.DWORD, w.BOOL, w.DWORD]),
        ('CloseHandle', w.BOOL, [w.HANDLE]),
        ('WaitForSingleObject', w.DWORD, [w.HANDLE, w.DWORD]),
        ('QueryFullProcessImageNameW', w.BOOL, [w.HANDLE, w.DWORD, w.LPWSTR, ctypes.POINTER(w.DWORD)]),
        ('GetProcessTimes', w.BOOL, [w.HANDLE] + [ctypes.POINTER(w.FILETIME)] * 4),
        ('CreateToolhelp32Snapshot', w.HANDLE, [w.DWORD, w.DWORD]),
        ('Module32FirstW', w.BOOL, [w.HANDLE, ctypes.POINTER(Module)]),
        ('Module32NextW', w.BOOL, [w.HANDLE, ctypes.POINTER(Module)]),
    ):
        fn = getattr(kernel, name)
        fn.restype, fn.argtypes = result, args

    def failure(label):
        return OSError(label + ': ' + str(ctypes.WinError(ctypes.get_last_error())))

    handle = kernel.OpenProcess(0x1000 | 0x100000, False, pid)  # QUERY_LIMITED_INFORMATION | SYNCHRONIZE
    if not handle:
        raise failure('OpenProcess')

    class Process:
        source = 'Windows Toolhelp module snapshot'

        def identity(self):
            if kernel.WaitForSingleObject(handle, 0) != 0x102:  # WAIT_TIMEOUT: still running
                raise ValueError('Selected process exited or its liveness cannot be confirmed')
            path = ctypes.create_unicode_buffer(32768)
            size = w.DWORD(len(path))
            if not kernel.QueryFullProcessImageNameW(handle, 0, path, ctypes.byref(size)):
                raise failure('QueryFullProcessImageNameW')
            times = [w.FILETIME() for _ in range(4)]
            if not kernel.GetProcessTimes(handle, *(ctypes.byref(item) for item in times)):
                raise failure('GetProcessTimes')
            return {'pid': pid, 'executablePath': path.value,
                    'creationTime100ns': (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime}

        def modules(self, limit):
            invalid = ctypes.c_void_p(-1).value
            for _ in range(3):  # Bound loader-churn retries; exhaustion stays incomplete.
                snapshot = kernel.CreateToolhelp32Snapshot(0x8 | 0x10, pid)
                if snapshot != invalid:
                    break
                if ctypes.get_last_error() != 24:  # ERROR_BAD_LENGTH
                    raise failure('CreateToolhelp32Snapshot')
            else:
                raise failure('CreateToolhelp32Snapshot retry budget exhausted')
            try:
                entry = Module()
                entry.dwSize = ctypes.sizeof(entry)
                if not kernel.Module32FirstW(snapshot, ctypes.byref(entry)):
                    raise failure('Module32FirstW')
                rows = []
                while True:
                    if len(rows) >= limit:
                        raise ValueError('Module count budget exceeded')
                    path = entry.szExePath
                    if not path or len(path) >= 259 or not os.path.isabs(path):
                        raise ValueError('Missing, possibly truncated or nonabsolute module path')
                    if entry.th32ProcessID != pid:
                        raise ValueError('Module snapshot PID differs from selected process')
                    rows.append({'path': path, 'baseAddress': entry.modBaseAddr,
                                 'imageBytes': entry.modBaseSize})
                    if not kernel.Module32NextW(snapshot, ctypes.byref(entry)):
                        if ctypes.get_last_error() != 18:  # ERROR_NO_MORE_FILES
                            raise failure('Module32NextW')
                        break
                return rows
            finally:
                kernel.CloseHandle(snapshot)

    try:
        yield Process()
    finally:
        kernel.CloseHandle(handle)


def observe(process, expected_exe, expected_sha256, required_modules=(), *,
            max_modules=1024, max_bytes=1024 * 1024 * 1024):
    """Reconcile OS observations and disk reads; process may be a fixture in tests."""
    if not re.fullmatch('[0-9a-fA-F]{64}', expected_sha256):
        raise ValueError('Expected executable SHA-256 must be 64 hexadecimal digits')
    if any(type(n) is not int or n < 1 for n in (max_modules, max_bytes)):
        raise ValueError('Positive integer budgets required')
    report = {'schema': 'SKAGER.WindowsModuleObservation.1',
              'startedUtc': datetime.now(timezone.utc).isoformat(),
              'processObservationSource': process.source,
              'runtimeEnforcement': False, 'loaderClosureAccepted': False,
              'mappedImageIdentityEstablished': False, 'limits': LIMITS[:],
              'expectedExecutable': {'path': str(expected_exe), 'diskSha256': expected_sha256.lower()},
              'requiredModulePaths': [str(p) for p in required_modules],
              'budgets': {'modules': max_modules, 'diskBytes': max_bytes},
              'modules': [], 'errors': [], 'observedDiskBytes': 0}
    observed = {}

    def capture(path):
        key = path_key(path)
        if key not in observed:
            actual, data, before = read_file(path, max_bytes - report['observedDiskBytes'])
            report['observedDiskBytes'] += len(data)
            record = {'path': str(actual), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
            observed[key] = (actual, before, record)
        return observed[key][2].copy()

    def snapshot():
        rows = process.modules(max_modules)
        if not rows or len(rows) > max_modules:
            raise ValueError('Empty module snapshot or module count budget exceeded')
        keys = [(path_key(row['path']), row['baseAddress'], row['imageBytes']) for row in rows]
        if len(keys) != len(set(keys)):
            raise ValueError('Duplicate module snapshot records')
        return sorted(rows, key=lambda row: (path_key(row['path']), row['baseAddress'])), set(keys)

    try:
        before_identity = process.identity()
        report['processBefore'] = before_identity
        if path_key(before_identity['executablePath']) != path_key(expected_exe):
            raise ValueError('Selected process executable path differs from expected path')
        report['executableDisk'] = capture(expected_exe)
        if report['executableDisk']['sha256'] != expected_sha256.lower():
            raise ValueError('Executable disk SHA-256 differs from expected identity')
        rows, before_modules = snapshot()
        paths = {path_key(row['path']) for row in rows}
        if path_key(expected_exe) not in paths:
            report['errors'].append('Executable missing from module snapshot')
        for path in required_modules:
            if path_key(path) not in paths:
                report['errors'].append('Required module absent: ' + str(path))
        for row in rows:
            record = dict(row, pathSource=process.source)
            report['modules'].append(record)
            try:
                record['disk'] = capture(row['path'])
            except (OSError, ValueError) as error:
                record['diskError'] = str(error)
                report['errors'].append('Module disk identity unavailable: ' + row['path'])
        _, after_modules = snapshot()
        if before_modules != after_modules:
            report['errors'].append('Module paths, base addresses or image sizes changed between snapshots')
        report['processAfter'] = process.identity()
        if before_identity != report['processAfter']:
            report['errors'].append('Process identity changed during observation (possible PID reuse)')
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
    for path, before, _ in observed.values():
        try:
            if stamp(plain_path(path).stat()) != before:
                report['errors'].append('File changed after disk observation: ' + str(path))
        except (OSError, ValueError) as error:
            report['errors'].append(str(error))
    report['status'] = 'observation-incomplete' if report['errors'] else 'module-paths-and-disk-files-observed'
    report['finishedUtc'] = datetime.now(timezone.utc).isoformat()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--expected-exe', type=Path, required=True)
    parser.add_argument('--expected-sha256', required=True)
    parser.add_argument('--require-module', type=Path, action='append', default=[])
    args = parser.parse_args()
    if not 0 < args.pid <= 0xffffffff:
        parser.error('PID must be a positive DWORD')
    try:
        # Record helper provenance separately; reject source mutation during this run.
        sources = [Path(__file__).resolve(), Path(__file__).with_name('win32_plugin_inventory.py').resolve()]
        captured = [read_file(path) for path in sources]
        with windows_process(args.pid) as process:
            report = observe(process, args.expected_exe, args.expected_sha256, args.require_module)
        report['observer'] = {'platform': sys.platform, 'pythonVersion': sys.version,
                              'pointerBits': ctypes.sizeof(ctypes.c_void_p) * 8,
                              'sourceSha256': {path.name: hashlib.sha256(data).hexdigest()
                                               for path, data, _ in captured}}
        for path, data, before in captured:
            _, after_data, after_stamp = read_file(path)
            if data != after_data or before != after_stamp:
                report['errors'].append('Observer source changed during capture: ' + str(path))
                report['status'] = 'observation-incomplete'
    except (OSError, ValueError) as error:
        report = {'schema': 'SKAGER.WindowsModuleObservation.1', 'status': 'observation-incomplete',
                  'errors': [str(error)], 'runtimeEnforcement': False, 'loaderClosureAccepted': False,
                  'mappedImageIdentityEstablished': False}
    json.dump(report, sys.stdout, indent=2, ensure_ascii=True)
    print()
    return 1 if report['status'] == 'observation-incomplete' else 0


if __name__ == '__main__':
    raise SystemExit(main())
