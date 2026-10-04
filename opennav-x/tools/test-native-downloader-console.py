#!/usr/bin/env python3
"""Bounded native wx console diagnosis; no TLS, navigation app or boat access."""
import argparse
import ctypes
from ctypes import wintypes
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ('tools/test-native-downloader-console.py', 'tools/TrustProbeConsole.h',
          'tests/native_downloader_console/CMakeLists.txt',
          'tests/native_downloader_console/probe.cpp',
          'tools/test-windows-changed-units.py', 'tools/windows-wx.lock.json')


def owned_windows(pid):
    user = ctypes.WinDLL('user32', use_last_error=True)
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user.EnumWindows.argtypes = [callback_type, wintypes.LPARAM]
    user.EnumChildWindows.argtypes = [wintypes.HWND, callback_type, wintypes.LPARAM]
    user.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user.GetClassNameW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user.IsWindowVisible.argtypes = [wintypes.HWND]
    windows = []

    def item(hwnd):
        owner = wintypes.DWORD()
        user.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
        if owner.value != pid:
            return None
        title = ctypes.create_unicode_buffer(2048)
        cls = ctypes.create_unicode_buffer(128)
        user.GetWindowTextW(hwnd, title, len(title))
        user.GetClassNameW(hwnd, cls, len(cls))
        return {'handle': int(hwnd), 'class': cls.value, 'text': title.value,
                'visible': bool(user.IsWindowVisible(hwnd))}

    @callback_type
    def visit(hwnd, _):
        entry = item(hwnd)
        if entry is not None:
            children = []

            @callback_type
            def child(child_hwnd, _):
                value = item(child_hwnd)
                if value is not None:
                    children.append(value)
                return True

            user.EnumChildWindows(hwnd, child, 0)
            entry['children'] = children
            windows.append(entry)
        return True

    if not user.EnumWindows(visit, 0):
        raise ctypes.WinError(ctypes.get_last_error())
    return windows


def probe(executable, mode, evidence):
    destination = evidence / (mode + '.output')
    destination.write_bytes(b'pre-existing destination\n')
    process = subprocess.Popen([str(executable), mode, str(destination)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    timed_out = False
    windows = []
    try:
        stdout, stderr = process.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        timed_out = True
        try:
            windows = owned_windows(process.pid)
        finally:
            # This exact single-purpose child never spawns another process.
            process.kill()
            stdout, stderr = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
    (evidence / (mode + '.stdout.txt')).write_bytes(stdout)
    (evidence / (mode + '.stderr.txt')).write_bytes(stderr)
    record = {'mode': mode, 'pid': process.pid, 'timeoutSeconds': 5,
              'timedOut': timed_out, 'exitCode': process.returncode,
              'ownedWindowsAtTimeout': windows}
    (evidence / (mode + '.process.json')).write_text(json.dumps(record, indent=2) + '\n')
    return record, stdout.decode('utf-8', errors='replace'), stderr.decode('utf-8', errors='replace'), destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted' or
            os.environ.get('GITHUB_REPOSITORY') != 'ThereptileII/Work'):
        raise SystemExit('Exact disposable hosted GitHub Windows repository required')
    for name in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(name):
            raise SystemExit('Inherited compiler override refused: ' + name)
    spec = importlib.util.spec_from_file_location('changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': __doc__, 'applicationBuild': False,
              'tlsAcceptance': False, 'boatAccess': False,
              'candidate': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'runId': os.environ.get('GITHUB_RUN_ID'), 'runAttempt': os.environ.get('GITHUB_RUN_ATTEMPT'),
              'sources': {name: api.record(ROOT / name) for name in INPUTS}, 'cases': []}
    try:
        sdk = evidence / 'sdk'
        wx = sdk / 'wx'
        lock = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        if lock['version'] != '3.2.8' or len(lock['archives']) != 3:
            raise ValueError('Exact three locked wx3.2.8 archives required')
        report['wxLock'] = lock
        for item in lock['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/native_downloader_console', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log', timeout=90)
        api.run(['cmake', '--build', build, '--config', 'Release', '--parallel', '2',
                 '--', '/verbosity:normal'], evidence / 'compile.log', timeout=120)
        executable = build / 'Release/native-downloader-console.exe'
        report['executable'] = api.record(executable)
        report['runtime'] = api.stage_native_runtime(executable, wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        for mode in ('original-log', 'startup-log-ownership', 'fixed', 'fixed-lifecycle', 'fixed-assert'):
            record, stdout, stderr, destination = probe(executable, mode, evidence)
            report['cases'].append(record)
            if mode == 'original-log':
                dialogs = [window for window in record['ownedWindowsAtTimeout']
                           if window['class'] == '#32770' and window['text'] == 'Message' and
                           window['visible'] and any('native-original-log' in child['text']
                                                     for child in window['children'])]
                if (not record['timedOut'] or len(dialogs) != 1 or
                        'probe_stage=before-original-log' not in stderr or
                        'probe_stage=after-original-log' in stderr or stdout):
                    raise ValueError('Original no-init log did not reproduce exact owned Message dialog')
            elif mode == 'startup-log-ownership':
                stages = ('probe_stage=before-wx-initialization',
                          'probe_stage=startup-log-destroyed-by-wx',
                          'probe_stage=after-wx-initialization',
                          'probe_stage=former-owner-would-delete-again')
                if (record['timedOut'] or record['exitCode'] != 87 or
                        any(stderr.count(stage) != 1 for stage in stages) or
                        [stderr.index(stage) for stage in stages] !=
                        sorted(stderr.index(stage) for stage in stages)):
                    raise ValueError('Native wx startup ownership deletion was not observed')
            elif mode == 'fixed-lifecycle':
                teardown = ('probe_stage=console-logger-delete-begin',
                            'probe_stage=console-logger-delete-complete',
                            'probe_stage=console-wx-cleanup-complete',
                            'probe_stage=fixed-scope-destroyed')
                if (record['timedOut'] or record['exitCode'] != 0 or
                        [line for line in stderr.splitlines() if line.startswith('probe_stage=')] !=
                        list(teardown) * 16 or
                        stderr.count('native-fixed-lifecycle ') != 16):
                    raise ValueError('Repeated actual helper lifecycle failed')
            elif mode == 'fixed':
                if (record['timedOut'] or record['exitCode'] != 0 or
                        any(value not in stderr for value in ('native-fixed-message', 'native-fixed-warning',
                            'probe_stage=staging-written', 'probe_stage=rename-complete')) or
                        destination.read_bytes() != b'native console staging payload\n' or
                        list(evidence.glob('.ocpn-download-*'))):
                    raise ValueError('Fixed console log/staging/rename proof failed')
            elif (record['timedOut'] or record['exitCode'] != 86 or
                  'probe_assertion_failed' not in stderr or 'native-console-assert' not in stderr or
                  'probe_stage=after-assert' in stderr):
                raise ValueError('Actual assertion did not fail promptly through stderr')
            if mode != 'fixed' and destination.read_bytes() != b'pre-existing destination\n':
                raise ValueError('Non-file probe changed its destination')
        if api.record(executable) != report['executable']:
            raise ValueError('Executable changed during proof')
        for name, identity in report['runtime'].items():
            if api.record(executable.parent / name) != {k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during proof: ' + name)
        report['status'] = 'passed'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in report['sources'].items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native console proof failed')


if __name__ == '__main__':
    main()
