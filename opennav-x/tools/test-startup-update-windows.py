#!/usr/bin/env python3
"""Build/run the startup update model and popup on disposable native Windows."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_COMMIT = '0123456789012345678901234567890123456789'

RECEIPT_HARNESS = r'''
param([string]$Module,[string]$Worker,[string]$Evidence,[string]$Commit)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. $Module
function Check([bool]$Passed,[string]$Message) { if(-not $Passed){throw $Message} }
function Identity([char]$Digit) {
 return [pscustomobject]@{generation=([string]$Digit)*32;commit=([string]$Digit)*40;
  packageSha256=([string]$Digit)*64;executableSha256=([string]$Digit)*64}
}
function HashWorker([string]$Path) {
 # The native host may inherit pwsh's PSModulePath; do not require module
 # auto-discovery to authenticate the actual compiled sender executable.
 $sha=[Security.Cryptography.SHA256]::Create(); $stream=$null
 try {
  $stream=[IO.File]::OpenRead($Path)
  return ([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant()
 } finally { if($stream){$stream.Dispose()};$sha.Dispose() }
}
$hash=HashWorker $Worker
foreach($case in @('healthy','no-marker','interrupted','invalid-request','wrong-expected-commit')) {
 $record=New-UpdatePendingRecord (Identity 'a') (Identity 'b')
 $record.candidate.commit=$Commit
 if($case -eq 'wrong-expected-commit'){$record.candidate.commit='e'*40}
 $record.candidate.executableSha256=$hash
 $session=New-UpdateStartupSession $record
 $process=$null
 try {
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$Worker; $start.UseShellExecute=$false; $start.CreateNoWindow=$true
  $start.RedirectStandardOutput=$true; $start.RedirectStandardError=$true
  $start.EnvironmentVariables['SKAGER_UPDATE_PIPE']=$session.pipe
  $start.EnvironmentVariables['SKAGER_UPDATE_GENERATION']=$record.candidate.generation
  $start.EnvironmentVariables['SKAGER_UPDATE_CHALLENGE']=$session.challenge
  if($case -eq 'invalid-request'){$start.EnvironmentVariables['SKAGER_UPDATE_CHALLENGE']='invalid'}
  # An attacker-selected environment commit must never replace compiled code identity.
  $start.EnvironmentVariables['SKAGER_UPDATE_COMMIT']='f'*40
  $start.EnvironmentVariables['SKAGER_RECEIPT_FIXTURE_MODE']=$(if($case -in @('no-marker','interrupted')){$case}else{'healthy'})
  $process=[Diagnostics.Process]::Start($start)
  $accepted=Wait-UpdateStartupSuccess $record $session $process $Worker 3000
  Check ($accepted -eq ($case -eq 'healthy')) ('Unexpected receipt decision: '+$case)
 } finally {
  $session.server.Dispose()
  if($process) {
   if(-not $process.HasExited){$process.Kill();$process.WaitForExit()}
   $output=$process.StandardOutput.ReadToEnd()+$process.StandardError.ReadToEnd()
   [IO.File]::WriteAllText((Join-Path $Evidence ('receipt-'+$case+'.log')),$output)
   Check ($output.Contains('PASS Win32 and CRT updater environment cleared; compiled commit '+$Commit)) ('Native custody/compiled-commit fixture did not run: '+$case)
   $process.Dispose()
  }
 }
 Write-Host ('PASS actual sender + authenticated receiver: '+$case)
}
'''


def test_prompt(client, evidence):
    """Exercise the installed executable protocol and actual native controls."""
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.WinDLL('user32', use_last_error=True)
    callback = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user32.EnumWindows.argtypes = [callback, wintypes.LPARAM]
    user32.EnumChildWindows.argtypes = [wintypes.HWND, callback, wintypes.LPARAM]
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
    user32.SendMessageTimeoutW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM,
        wintypes.LPARAM, wintypes.UINT, wintypes.UINT, ctypes.POINTER(ctypes.c_size_t)]
    user32.SendMessageTimeoutW.restype = wintypes.LPARAM
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    valid = (('a' * 64) + '\n0.6.0-beta.1\n' + ('b' * 40) + '\n').encode('ascii')
    results = []
    for name, data in (
        ('empty', b''), ('missing-final-lf', valid[:-1]), ('extra-line', valid + b'\n'),
        ('crlf', valid.replace(b'\n', b'\r\n')), ('non-ascii', valid + b'\xff'),
        ('oversized', b'a' * 257), ('nul', valid.replace(b'0.6', b'0\x006')),
        ('url', valid.replace(b'0.6.0-beta.1', b'https://example.invalid/setup.exe')),
        ('uppercase-hash', valid.replace(b'a', b'A', 1)),
    ):
        child = subprocess.run([client], input=data, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=8)
        if child.returncode != 2:
            raise ValueError('Malformed prompt request did not fail closed: ' + name)
        results.append({'case': name, 'exit': child.returncode})
    child = subprocess.Popen([client], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
    try:
        child.stdin.write(valid)
        child.stdin.flush()  # Deliberately withhold EOF: no UI is permitted.
        if child.wait(timeout=8) != 2:
            raise ValueError('Prompt accepted input without bounded EOF')
        results.append({'case': 'missing-eof', 'exit': child.returncode})
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate()
    disk = evidence / 'prompt-input.txt'
    disk.write_bytes(valid)
    with disk.open('rb') as source:
        child = subprocess.run([client], stdin=source, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=8)
    if child.returncode != 2:
        raise ValueError('Prompt accepted file instead of pipe')
    results.append({'case': 'disk-input', 'exit': child.returncode})

    def title(handle):
        value = ctypes.create_unicode_buffer(512)
        user32.GetWindowTextW(handle, value, len(value))
        return value.value

    def find_dialog(child, caption):
        selected = []
        until = time.monotonic() + 8
        while time.monotonic() < until and child.poll() is None and not selected:
            @callback
            def window(handle, unused):
                owner = wintypes.DWORD()
                user32.GetWindowThreadProcessId(handle, ctypes.byref(owner))
                if (owner.value == child.pid and title(handle) == caption
                        and user32.IsWindowVisible(handle)):
                    selected.append(handle)
                return True
            user32.EnumWindows(window, 0)
            if not selected:
                time.sleep(.02)
        if len(selected) != 1:
            raise ValueError('Installed prompt did not show its unique native dialog: ' + caption)
        return selected[0]

    def click_button(dialog, label):
        buttons = []
        @callback
        def control(handle, unused):
            if title(handle) == label and user32.IsWindowVisible(handle):
                buttons.append(handle)
            return True
        user32.EnumChildWindows(dialog, control, 0)
        if len(buttons) != 1:
            raise ValueError('Native prompt button not uniquely accessible: ' + label)
        for message, flags in ((0x0201, 1), (0x0202, 0)):
            if not user32.PostMessageW(buttons[0], message, flags, (20 << 16) | 20):
                raise ValueError('Could not deliver native prompt button input')

    for choice, expected in (('LATER', 0), ('UPDATE NOW', 10), ('close', 0)):
        child = subprocess.Popen([client], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        try:
            child.stdin.write(valid)
            child.stdin.close()
            child.stdin = None
            dialog = find_dialog(child, 'SKAGER software update')
            if choice == 'close':
                if not user32.PostMessageW(dialog, 0x0010, 0, 0):
                    raise ValueError('Could not close installed popup')
            else:
                click_button(dialog, choice)
            child.communicate(timeout=8)
            if child.returncode != expected:
                raise ValueError('Installed prompt returned incorrect choice: ' + choice)
            results.append({'case': choice, 'exit': child.returncode})
        finally:
            if child.poll() is None:
                child.kill()
                child.communicate()

    progress = [client, '--download-progress']
    for name, args, data, expected in (
        ('progress-immediate-eof', progress, b'', 0),
        ('progress-content', progress, b'x', 2),
        ('progress-nul-content', progress, b'\0', 2),
        ('progress-extra-argument', progress + ['unexpected'], b'', 2),
    ):
        child = subprocess.run(args, input=data, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=8)
        if child.returncode != expected:
            raise ValueError('Incorrect download-progress protocol decision: ' + name)
        results.append({'case': name, 'exit': child.returncode})
    with disk.open('rb') as source:
        child = subprocess.run(progress, stdin=source, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=8)
    if child.returncode != 2:
        raise ValueError('Download progress accepted non-pipe stdin')
    results.append({'case': 'progress-disk-input', 'exit': child.returncode})

    for action, expected in (('eof', 0), ('cancel', 1), ('escape', 1),
                             ('close', 1), ('content', 2), ('cancel-burst', 1),
                             ('close-then-eof', 1)):
        child = subprocess.Popen(progress, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        try:
            dialog = find_dialog(child, 'SKAGER update download')
            # Empty but still-open stdin must keep the visible window alive.
            time.sleep(.1)
            if child.poll() is not None or not user32.IsWindowVisible(dialog):
                raise ValueError('Progress disappeared before EOF or a user decision')
            started = time.monotonic()
            if action == 'eof':
                child.stdin.close()
                child.stdin = None
            elif action == 'content':
                child.stdin.write(b'x')
                child.stdin.flush()
            elif action == 'escape':
                for message in (0x0100, 0x0101):
                    if not user32.PostMessageW(dialog, message, 0x1B, 0):
                        raise ValueError('Could not send Escape to download progress')
            elif action == 'close':
                if not user32.PostMessageW(dialog, 0x0010, 0, 0):
                    raise ValueError('Could not close download progress')
            elif action == 'close-then-eof':
                handled = ctypes.c_size_t()
                # Synchronous close delivers the cancellation handler first;
                # later EOF must never overwrite the accepted cancellation.
                if not user32.SendMessageTimeoutW(dialog, 0x0010, 0, 0, 3, 2000,
                                                  ctypes.byref(handled)):
                    raise ValueError('Could not deliver ordered progress cancellation')
                child.stdin.close()
                child.stdin = None
            else:
                click_button(dialog, 'CANCEL')
                if action == 'cancel-burst':
                    # Queue a second close while the first cancellation drains.
                    user32.PostMessageW(dialog, 0x0010, 0, 0)
            # Keep the parent's pipe open while waiting: cancelling must stop
            # the worker independently, never wait for the downloader's EOF.
            child.wait(timeout=3)
            elapsed = time.monotonic() - started
            child.communicate()
            if child.returncode != expected:
                raise ValueError('Incorrect native download-progress result: ' + action)
            results.append({'case': 'progress-' + action, 'exit': child.returncode,
                            'exitSeconds': elapsed})
        finally:
            if child.poll() is None:
                child.kill()
                child.communicate()
    (evidence / 'prompt-protocol.json').write_text(json.dumps(results, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true'
            or os.environ.get('RUNNER_ENVIRONMENT') != 'github-hosted'):
        raise SystemExit('Disposable native Windows CI required; never the boat')
    for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(key):
            raise SystemExit('Refusing inherited compiler override: ' + key)
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location(
        'changed_units', ROOT / 'tools/test-windows-changed-units.py')
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    paths = [ROOT / name for name in (
        'tools/test-startup-update-windows.py', 'tools/test-windows-changed-units.py',
        'tools/windows-wx.lock.json', 'tests/startup_update/CMakeLists.txt',
        'tests/startup_update_tests.cpp', 'tests/startup_update_dialog_test.cpp',
        'tests/update_startup_receipt_tests.cpp', 'tests/update_startup_receipt_native_test.cpp',
        'src/integration/UpdateStartupReceipt.cpp', 'src/integration/OpenNavBuild.h.in',
        'installer/windows/UpdateTransaction.ps1', 'cmake/StartupUpdate.cmake',
        'src/platform/windows/StartupUpdatePrompt.cpp',
        'src/application/StartupUpdate.cpp', 'src/ui/StartupUpdateDialog.cpp',
        'src/ui/Controls.cpp', 'src/vessel/VesselState.cpp')]
    paths += sorted((ROOT / 'src').rglob('*.h'))
    sources = {str(p.relative_to(ROOT)): api.record(p) for p in paths}
    report = {
        'status': 'failed',
        'scope': 'isolated model, installed prompt and receipt sender/receiver; no installer, product, '
                 'network, boat or visual-design acceptance',
        'commit': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'receiptFixtureCompiledCommit': FIXTURE_COMMIT,
        'sources': sources, 'nativeProductAcceptance': False,
    }
    try:
        sdk = ROOT / 'build/startup-update-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive],
                    evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/startup_update', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target',
                 'startup_update_tests', 'startup_update_dialog_test', 'update_startup_receipt_tests',
                 'update_startup_receipt_native_test', 'skager-update-prompt', '--parallel', '2'],
                evidence / 'compile.log', timeout=180)
        clients = {name: build / ('Release/' + name + '.exe') for name in
                   ('startup_update_tests', 'startup_update_dialog_test', 'update_startup_receipt_tests',
                    'update_startup_receipt_native_test', 'skager-update-prompt')}
        report['executables'] = {name: api.record(path) for name, path in clients.items()}
        report['runtime'] = api.stage_native_runtime(clients['startup_update_dialog_test'], wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'))
        for name, client in clients.items():
            if name in ('update_startup_receipt_native_test', 'skager-update-prompt'):
                continue
            api.run([client], evidence / (name + '.log'), timeout=60)
            if api.record(client) != report['executables'][name]:
                raise ValueError('Fixture executable changed during replay: ' + name)
        test_prompt(clients['skager-update-prompt'], evidence)
        harness = evidence / 'receipt-native.ps1'
        harness.write_text(RECEIPT_HARNESS)
        api.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                 '-File', harness, '-Module', ROOT / 'installer/windows/UpdateTransaction.ps1',
                 '-Worker', clients['update_startup_receipt_native_test'],
                 '-Evidence', evidence, '-Commit', FIXTURE_COMMIT],
                evidence / 'receipt-native.log', timeout=45)
        for name, client in clients.items():
            if api.record(client) != report['executables'][name]:
                raise ValueError('Fixture executable changed during replay: ' + name)
        for name, identity in report['runtime'].items():
            if api.record(build / 'Release' / name) != {
                    k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during replay: ' + name)
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    if report['status'] != 'passed':
        raise RuntimeError('Native startup update fixture did not pass')


if __name__ == '__main__':
    main()
