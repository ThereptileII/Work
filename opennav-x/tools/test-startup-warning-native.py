#!/usr/bin/env python3
"""Exact pinned navigation-warning boundary on disposable native Windows only."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

MODAL_HARNESS = r'''
param([string]$Module,[string]$Worker,[string]$Evidence,[string]$Commit)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. $Module
function Check([bool]$Passed,[string]$Message) { if(-not $Passed){throw $Message} }
$sha=[Security.Cryptography.SHA256]::Create()
$stream=[IO.File]::OpenRead($Worker)
try {$hash=([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant()}
finally {$stream.Dispose();$sha.Dispose()}
$results=@()
foreach($case in @('agree','cancel','timeout','fast')) {
 $identity=[pscustomobject]@{generation=('a'*32);commit=$Commit;packageSha256=('b'*64);executableSha256=$hash}
 $session=New-UpdateHealthSession $identity
 $process=$null
 $clock=[Diagnostics.Stopwatch]::StartNew()
 try {
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$Worker;$start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $start.RedirectStandardOutput=$true;$start.RedirectStandardError=$true
  $start.EnvironmentVariables['SKAGER_UPDATE_PIPE']=$session.pipe
  $start.EnvironmentVariables['SKAGER_UPDATE_GENERATION']=$identity.generation
  $start.EnvironmentVariables['SKAGER_UPDATE_CHALLENGE']=$session.challenge
  $start.EnvironmentVariables['SKAGER_WARNING_FIXTURE_MODE']=$case
  $start.EnvironmentVariables['SKAGER_WARNING_FIXTURE_PROFILE']=Join-Path $Evidence ('modal-'+$case+'.ini')
  $process=[Diagnostics.Process]::Start($start)
  $accepted=$session.server.Receive($process,$Worker,$hash,(Get-UpdateReadyFrame $identity $session),3000,7000)
  Check ($accepted -eq ($case -in @('agree','fast'))) ('Unexpected exact-modal decision: '+$case+'; '+$session.server.FailureReason)
  if($case -in @('agree','cancel')) {
   Check ($clock.ElapsedMilliseconds -ge 3500) 'Actual modal did not remain unqualified beyond initial startup deadline.'
  }
  if($case -eq 'cancel') {
   Check ($session.server.Phase -ceq 'cancelled' -and $session.server.FailureReason -ceq 'human-cancelled') 'Actual Cancel did not remain cancellation.'
  }
  if($case -eq 'timeout') {
   Check ($session.server.Phase -ceq 'awaiting-human' -and $session.server.FailureReason -ceq 'human-wait-timeout') 'Unanswered actual warning did not exhaust its finite human deadline.'
  }
  if(-not $accepted) { Check ($null -eq $session.server.VerifiedFrame) 'Rejected modal case produced health proof.' }
  $results+=@{case=$case;accepted=$accepted;phase=$session.server.Phase;reason=$session.server.FailureReason;elapsedMs=$clock.ElapsedMilliseconds}
 } finally {
  $session.server.Dispose()
  if($process) {
   if(-not $process.HasExited){$process.Kill();$process.WaitForExit()}
   $output=$process.StandardOutput.ReadToEnd()+$process.StandardError.ReadToEnd()
   [IO.File]::WriteAllText((Join-Path $Evidence ('modal-'+$case+'.log')),$output)
   $process.Dispose()
   Check ($output.Contains('PASS exact upstream modal visible; no consent yet') -and -not $output.Contains('FAIL')) ('Actual upstream modal failed: '+$case)
   if($case -ne 'timeout') {
    $choice=if($case -eq 'cancel'){'cancel'}else{'agree'}
    Check ($output.Contains('PASS exact dialog result '+$choice+'; original nav-message branch preserved')) 'Original dialog and call-site result were not observed.'
    Check ($output.Contains('PASS synthetic post-dialog checkpoint probe; not installed health')) 'Component-only health probe did not complete.'
   }
  }
  [IO.File]::WriteAllText((Join-Path $Evidence 'modal-results.json'),(ConvertTo-Json -InputObject @($results) -Depth 5))
 }
 Write-Host ('PASS exact upstream modal and authenticated phase protocol: '+$case)
}
'''


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def extract_boundaries(upstream, generated):
    """Extract exact pinned function and reconstruct the reviewed patch hunk."""
    frame = (upstream / 'gui/src/ocpn_frame.cpp').read_text(encoding='utf-8')
    begin = frame.index('bool ShowNavWarning() {\n')
    end = frame.index('\nbool isSingleChart(', begin)
    function = frame[begin:end]
    if function.count('info_dlg.ShowModal()') != 1 or 'return agreed == wxID_OK;' not in function:
        raise ValueError('Pinned warning function boundary changed')
    patch = (ROOT / 'patches/opencpn-5.12.4-xnav.patch').read_text(encoding='utf-8')
    hunks = re.split(r'^@@[^\n]*@@[^\n]*\n', patch, flags=re.M)
    selected = [h for h in hunks if '+      opennav::integration::NotifyUpdateNavigationWarning(true, false);' in h]
    if len(selected) != 1:
        raise ValueError('Expected exactly one reviewed navigation-warning hook hunk')
    lines = selected[0].splitlines(keepends=True)
    original = ''.join(line[1:] for line in lines if line.startswith((' ', '-')))
    updated = ''.join(line[1:] for line in lines if line.startswith((' ', '+')))
    app = (upstream / 'gui/src/ocpn_app.cpp').read_text(encoding='utf-8')
    if app.count(original) != 1 or updated.count('ShowNavWarning()') != 1:
        raise ValueError('Reviewed warning call site does not match exact pinned source')
    if 'const bool xnav_nav_warning = opennav::IsXNav();' not in updated:
        raise ValueError('Warning hook lost explicit XNav gate')
    generated.mkdir(parents=True, exist_ok=False)
    (generated / 'exact_warning.inc').write_text(function, encoding='utf-8')
    (generated / 'exact_warning_call.inc').write_text(updated, encoding='utf-8')


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
    api = load('warning_changed_units', ROOT / 'tools/test-windows-changed-units.py')
    receipts = load('warning_receipts', ROOT / 'tools/test-startup-update-windows.py')
    paths = [ROOT / 'tools/test-startup-warning-native.py',
             ROOT / 'tools/test-startup-update-windows.py', ROOT / 'tools/test-windows-changed-units.py',
             ROOT / 'tools/windows-wx.lock.json', ROOT / 'upstream.lock.json',
             ROOT / 'patches/opencpn-5.12.4-xnav.patch',
             ROOT / 'installer/windows/UpdateTransaction.ps1',
             ROOT / 'src/integration/UpdateStartupReceipt.cpp',
             ROOT / 'src/integration/UpdateStartupReceipt.h',
             ROOT / 'src/integration/OpenNavBuild.h.in',
             ROOT / 'tests/update_startup_receipt_tests.cpp',
             ROOT / 'tests/update_startup_receipt_native_test.cpp']
    paths += sorted((ROOT / 'tests/startup_warning').glob('*'))
    sources = {p.relative_to(ROOT).as_posix(): api.record(p) for p in paths if p.is_file()}
    report = {'status': 'failed', 'commit': subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'scope': 'exact upstream native modal and authenticated receipt phase boundary; '
                 'synthetic health clock/checkpoint; no installed application health, full application, '
                 'installer, plugins, navigation connections, boat, or visual-design acceptance',
        'nativeProductAcceptance': False, 'receiptFixtureCompiledCommit': receipts.FIXTURE_COMMIT,
        'sources': sources}
    try:
        lock = json.loads((ROOT / 'tests/startup_warning/upstream.lock.json').read_text())
        if lock['commit'] != json.loads((ROOT / 'upstream.lock.json').read_text())['commit']:
            raise ValueError('Warning source lock differs from supported OpenCPN pin')
        upstream = evidence / 'upstream'
        for item in lock['files']:
            api.fetch(item, upstream / item['path'])
        report['upstream'] = lock
        generated = evidence / 'extracted'
        extract_boundaries(upstream, generated)
        report['extracted'] = {p.name: api.record(p) for p in generated.iterdir()}
        sdk = ROOT / 'build/startup-warning-sdk'
        wx = sdk / 'wx'
        report['wxLock'] = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
        for item in report['wxLock']['archives']:
            archive = sdk / item['file']
            api.fetch(item, archive)
            api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        build = evidence / 'build'
        api.run(['cmake', '-S', ROOT / 'tests/startup_warning', '-B', build,
                 '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DWARNING_UPSTREAM:PATH=' + upstream.as_posix(),
                 '-DWARNING_GENERATED:PATH=' + generated.as_posix(),
                 '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                 '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                 '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
        targets = ('startup_warning_native', 'update_startup_receipt_tests', 'update_startup_receipt_native_test')
        api.run(['cmake', '--build', build, '--config', 'Release', '--target', *targets,
                 '--parallel', '2'], evidence / 'compile.log', timeout=180)
        clients = {name: build / 'Release' / (name + '.exe') for name in targets}
        report['executables'] = {name: api.record(path) for name, path in clients.items()}
        report['runtime'] = api.stage_native_runtime(clients['startup_warning_native'], wx,
            ('wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll', 'wxmsw32u_html_vc14x.dll'))
        api.run([clients['update_startup_receipt_tests']], evidence / 'receipt-policy.log', timeout=30)
        for name, body, worker, timeout in (
            ('receipt-native', receipts.RECEIPT_HARNESS, clients['update_startup_receipt_native_test'], 90),
            ('exact-modal', MODAL_HARNESS, clients['startup_warning_native'], 90),
        ):
            harness = evidence / (name + '.ps1')
            harness.write_text(body, encoding='utf-8')
            api.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                     '-File', harness, '-Module', ROOT / 'installer/windows/UpdateTransaction.ps1',
                     '-Worker', worker, '-Evidence', evidence, '-Commit', receipts.FIXTURE_COMMIT],
                    evidence / (name + '.log'), timeout=timeout)
        report['modalResults'] = json.loads((evidence / 'modal-results.json').read_text(encoding='utf-8-sig'))
        for name, client in clients.items():
            if api.record(client) != report['executables'][name]:
                raise ValueError('Fixture executable changed during replay: ' + name)
        for name, identity in report['runtime'].items():
            if api.record(build / 'Release' / name) != {k: identity[k] for k in ('bytes', 'sha256')}:
                raise ValueError('Runtime changed during replay: ' + name)
        if any(api.record(upstream / item['path']) != {k: item[k] for k in ('bytes', 'sha256')}
               for item in lock['files']):
            raise ValueError('Pinned upstream inputs changed during replay')
        if any(api.record(generated / name) != identity for name, identity in report['extracted'].items()):
            raise ValueError('Extracted exact boundary changed during replay')
        report['status'] = 'passed'
    finally:
        if any(api.record(ROOT / name) != identity for name, identity in sources.items()):
            report['status'] = 'failed'
            report['sourceDrift'] = True
        (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if report['status'] != 'passed':
        raise RuntimeError('Exact native warning boundary did not pass')


if __name__ == '__main__':
    main()
