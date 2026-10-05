#!/usr/bin/env python3
"""Real installed updater qualification, called only by disposable installer smoke.

Same-package transitions exercise installation/health/recovery mechanics. They
are deliberately not evidence of release-policy upgrade selection or trust setup.
No standalone destructive entrypoint, build, download, or product trust fixture.
"""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def qualify(*, install, setup, stock, profile, evidence, powershell, commit,
            executable_sha256, ui, owned, inventory, navigation_matches):
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise RuntimeError('Real package qualification requires disposable Windows CI')
    assert install == Path(os.environ['LOCALAPPDATA']) / 'OpenNavXAlpha1'
    assert re.fullmatch('[a-f0-9]{40}', commit)
    assert re.fullmatch('[a-f0-9]{64}', executable_sha256)
    assert install.is_dir() and profile.is_dir() and setup.is_file() and stock.is_file()
    output = evidence / 'packaged-updater.json'
    assert not output.exists(), 'Keep each qualification attempt separately'
    report = {'status': 'running', 'scope': 'real package, same-version supervised installation and authenticated startup; no release-selection or TUF acceptance',
              'commit': commit, 'setupSha256': digest(setup), 'executableSha256': executable_sha256,
              'testSourceSha256': digest(Path(__file__)), 'operations': [], 'checks': []}
    stock_before = inventory(stock.parent)
    initial_profile = inventory(profile)
    pending_path = install / 'update-pending.json'
    assert not pending_path.exists()

    def state():
        value = read(install / 'state.json')
        assert value['owner'] == 'OpenNavX.Alpha1.SideBySide.1' and value['schema'] == 1
        assert re.fullmatch('[a-f0-9]{32}', value['current'])
        return value

    def generation(identifier):
        assert re.fullmatch('[a-f0-9]{32}', identifier)
        directory = install / 'generations' / identifier
        record = read(directory / 'ownership.json')
        assert record['owner'] == 'OpenNavX.Alpha1.SideBySide.1'
        assert record['commit'] == commit and record['updateStartupHealth'] == 1
        assert record['xnavHardwareOutputPolicy'] == 'status-only'
        assert not (directory / 'app/update-trust.json').exists(), 'No product trust provisioning in this fixture'
        assert digest(directory / 'app/opencpn.exe') == executable_sha256
        for path in ('app/opencpn.exe', 'app/skager-start.exe', 'UpdateSupervisor.ps1', 'UpdateTransaction.ps1', 'Lifecycle.ps1'):
            matches = [entry for entry in record['managedFiles'] if entry['path'] == path]
            assert len(matches) == 1 and digest(directory / path) == matches[0]['sha256']
        return directory

    def check(message):
        report['checks'].append(message)
        print('Packaged updater: ' + message, flush=True)

    def command(arguments, phase, timeout=240):
        log = evidence / ('packaged-updater-' + phase + '.log')
        assert not log.exists()
        start = time.monotonic()
        with log.open('xb') as stream:
            process = subprocess.Popen(arguments, stdout=stream, stderr=subprocess.STDOUT)
            owned.add(process.pid)
            try:
                # A timeout does not kill an installer/supervisor in flight.
                # The outer disposable smoke owner retains failure evidence and
                # performs its existing process-tree cleanup after reporting.
                code = process.wait(timeout=timeout)
                assert code == 0, (phase, code, str(log))
            finally:
                if process.poll() is not None:
                    owned.discard(process.pid)
        report['operations'].append({'phase': phase, 'seconds': round(time.monotonic()-start, 3), 'log': log.name})

    def process_image(pid):
        handle = ui.monitor_process(pid)
        try:
            query = ui.declare(ui.kernel, 'QueryFullProcessImageNameW', ctypes.c_int,
                               ctypes.c_void_p, ctypes.c_ulong, ctypes.c_wchar_p,
                               ctypes.POINTER(ctypes.c_ulong))
            buffer = ctypes.create_unicode_buffer(32768)
            size = ctypes.c_ulong(len(buffer))
            assert query(handle, 0, buffer, ctypes.byref(size)), 'Cannot verify actual application process'
            return Path(buffer.value)
        finally:
            ui.CloseHandle(handle)

    def launch_and_close(arguments, identifier, phase, authenticate):
        directory = generation(identifier)
        executable = directory / 'app/opencpn.exe'
        assert not any(title == 'SKAGER / OpenCPN' for _, _, title in ui.windows())
        log = evidence / ('packaged-updater-' + phase + '.log')
        assert not log.exists()
        start = time.monotonic()
        with log.open('xb') as stream:
            parent = subprocess.Popen(arguments, stdout=stream, stderr=subprocess.STDOUT)
            owned.add(parent.pid)
            try:
                # Locate the actual app child, not the launcher/supervisor PID.
                # Its final image must match the exact generation before any UI action.
                window, pid = ui.wait_window('SKAGER / OpenCPN', timeout=90)
                assert process_image(pid).samefile(executable), 'Wrong generation opened the application window'
                owned.add(pid)
                assert parent.wait(timeout=max(1, 240-(time.monotonic()-start))) == 0, str(log)
                assert ui.IsWindowEnabled(window), 'Startup remains blocked by a modal dialog'
                if authenticate:
                    audit_receipt(identifier, phase)
                    assert time.monotonic()-start >= 30, 'Real continuous startup health checkpoint did not elapse'
                # The production CloseMainWindow path remains separately tested;
                # this fixture requests the normal visible application close.
                monitor = ui.monitor_process(pid)
                ui.close(window)
                ui.wait_clean_exit(monitor, timeout_ms=30000)
                owned.discard(pid)
            finally:
                if parent.poll() is not None:
                    owned.discard(parent.pid)
        assert navigation_matches(), 'Shared navigation fixtures changed across supervised startup'
        assert inventory(stock.parent) == stock_before, 'Supported stock installation changed'
        report['operations'].append({'phase': phase, 'generation': identifier,
                                     'seconds': round(time.monotonic()-start, 3), 'log': log.name})

    audit = evidence / 'packaged-updater-receipt-audit.ps1'
    assert not audit.exists()
    audit.write_text(r'''param([string]$Root,[string]$Generation)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if ($Generation -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid receipt generation.' }
. (Join-Path (Join-Path (Join-Path $Root 'generations') $Generation) 'UpdateSupervisor.ps1')
$record=Get-SupervisedGeneration $Root $Generation
Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $Root $record.identity) $record.identity
Write-Output 'Exact-generation DPAPI startup receipt authenticated.'
''', encoding='utf-8')

    def audit_receipt(identifier, phase):
        command([str(powershell), '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                 '-File', str(audit), '-Root', str(install), '-Generation', identifier], phase+'-receipt', 60)
        report['operations'].append({'phase': phase+'-receipt-proof', 'generation': identifier,
                                     'receiptSha256': digest(install / 'known-good' / (identifier+'.receipt'))})

    def supervised_setup(previous, phase):
        before = inventory(profile)
        result = evidence / ('packaged-updater-' + phase + '-setup.json')
        assert not result.exists()
        assert all('"' not in str(value) for value in (setup, stock, result))
        arguments = (subprocess.list2cmdline([str(setup)]) + ' /S /ACTION=Update /SUPERVISED=1'
                     + ' /OPENCPN="' + str(stock) + '" /REPORT="' + str(result) + '"')
        command(arguments, phase+'-setup', 240)
        assert read(result)['status'] == 'passed'
        assert inventory(profile) == before, 'Supervised Setup modified the existing profile'
        assert inventory(stock.parent) == stock_before
        after = state()
        assert after['current'] != previous and after['previous'] == previous
        generation(after['current'])
        pending = read(pending_path)
        assert pending['attempts'] == 0 and pending['session'] == ''
        assert pending['candidate']['generation'] == after['current'] and pending['previous']['generation'] == previous
        assert pending['candidate']['commit'] == pending['previous']['commit'] == commit
        assert pending['candidate']['executableSha256'] == pending['previous']['executableSha256'] == executable_sha256
        assert re.fullmatch('[a-f0-9]{32}', pending['transaction'])
        report['operations'].append({'phase': phase+'-pending', 'record': pending})
        return after['current'], pending

    def supervisor(identifier, transaction):
        return [str(powershell), '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                '-File', str(install / 'generations' / identifier / 'UpdateSupervisor.ps1'),
                '-InstallationRoot', str(install), '-Action', 'LaunchPending', '-Transaction', transaction]

    def archive(pending, outcome, attempts):
        assert not pending_path.exists(), 'Pending was not finalized'
        path = install / 'update-history' / (pending['transaction']+'-'+outcome+'.json')
        record = read(path)
        assert record['candidate'] == pending['candidate'] and record['previous'] == pending['previous']
        assert record['attempts'] == attempts
        if attempts:
            assert re.fullmatch('[a-f0-9]{32}', record['session']) and record['processId'] > 0
        report['operations'].append({'phase': outcome+'-archive', 'transaction': pending['transaction'],
                                     'attempts': attempts, 'archiveSha256': digest(path)})
        return record

    try:
        initial = state()['current']
        first = generation(initial)
        assert not (install / 'known-good' / (initial+'.receipt')).exists(), 'Bootstrap must establish new actual proof'
        assert inventory(profile) == initial_profile
        launch_and_close([str(first / 'app/skager-start.exe'), '--xnav'], initial, 'bootstrap', True)
        check('Actual installed launcher bootstraps real application and exact authenticated health receipt')

        healthy, pending = supervised_setup(initial, 'healthy')
        launch_and_close(supervisor(healthy, pending['transaction']), healthy, 'healthy-startup', True)
        assert state()['current'] == healthy
        archive(pending, 'healthy', 1)
        check('Exact Setup supervised update publishes pending candidate; real startup authenticates and finalizes one attempt')

        broken, pending = supervised_setup(healthy, 'broken')
        target = generation(broken) / 'app/opencpn.exe'
        for path in (target, *target.parents):
            assert not path.is_symlink() and not (getattr(path.lstat(), 'st_file_attributes', 0) & 0x400), 'Fault injection refuses redirected paths'
            if path == install:
                break
        ownership_hash = digest(target.parents[1] / 'ownership.json')
        assert not any(title == 'SKAGER / OpenCPN' for _, _, title in ui.windows())
        before = inventory(profile)
        # Explicit fault injection only in the newly installed disposable copy.
        # Keep Setup, source executable, publisher identity and ownership intact.
        # An invalid MZ header cannot execute even if an integrity check regresses.
        with target.open('r+b') as stream:
            original = stream.read(2)
            assert original == b'MZ'
            stream.seek(0); stream.write(b'\0\0'); stream.flush(); os.fsync(stream.fileno())
        fault = {'kind': 'disposable installed candidate invalid PE header', 'generation': broken,
                 'expectedSha256': executable_sha256, 'corruptSha256': digest(target),
                 'ownershipSha256': ownership_hash, 'path': 'app/opencpn.exe', 'restored': False}
        report['faultInjection'] = fault
        try:
            command(supervisor(broken, pending['transaction']), 'broken-candidate-recovery', 240)
            assert state()['current'] == healthy and state()['previous'] == ''
            failed = archive(pending, 'restored', 0)
            assert failed['processId'] == 0 and failed['session'] == '', 'Corrupt candidate was attempted'
            assert not (install / 'known-good' / (broken+'.receipt')).exists()
            assert inventory(profile) == before, 'Prelaunch integrity fallback modified the profile'
            assert inventory(stock.parent) == stock_before
            assert digest(target.parents[1] / 'ownership.json') == ownership_hash
            fault['recovery'] = 'LaunchPending rejected candidate integrity before process creation; actual guarded Lifecycle restored known-good previous'
            check('Fault-injected candidate is refused before launch; actual guarded rollback restores verified previous without profile changes')
        finally:
            # Restore only our two-byte fault, and only while the rest of the
            # exact file still matches the fault hash. Preserve unexpected changes.
            if digest(target) == fault['corruptSha256']:
                with target.open('r+b') as stream:
                    stream.write(original); stream.flush(); os.fsync(stream.fileno())
                assert digest(target) == executable_sha256
                fault['restored'] = True
        assert fault['restored'] and digest(target) == executable_sha256, 'Faulted copy was not restored to its original bytes'
        previous = generation(healthy)
        launch_and_close([str(previous / 'app/skager-start.exe'), '--xnav'], healthy, 'restored-startup', False)
        audit_receipt(healthy, 'restored')
        check('Restored actual generation launches and closes cleanly with retained authenticated receipt')
        assert digest(setup) == report['setupSha256']
        assert navigation_matches() and inventory(stock.parent) == stock_before
        report['status'] = 'passed'
        return report
    except BaseException as error:
        report['status'] = 'failed'; report['error'] = repr(error)
        for name in ('state.json', 'update-pending.json', 'transaction.json'):
            path = install / name
            if path.is_file() and path.stat().st_size <= 32768:
                try:
                    report[name] = read(path)
                except (OSError, ValueError) as record_error:
                    report[name] = {'readError': repr(record_error)}
        raise
    finally:
        output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    raise SystemExit('Run through smoke-installer-windows.py with its disposable package/profile fixture')
