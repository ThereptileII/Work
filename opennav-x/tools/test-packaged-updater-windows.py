#!/usr/bin/env python3
"""Real installed updater qualification, called only by disposable installer smoke.

Same-package transitions exercise installation/health/recovery mechanics. They
are deliberately not evidence of release-policy upgrade selection or trust setup.
No standalone destructive entrypoint, build, download, or product trust fixture.
"""
import ctypes
from hardware_output_policy import require_product_output_policy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import threading
import time


_startup_spec = importlib.util.spec_from_file_location(
    'packaged_updater_startup_log', Path(__file__).with_name('startup-log.py'))
_startup = importlib.util.module_from_spec(_startup_spec)
_startup_spec.loader.exec_module(_startup)


STARTUP_FAILURE = dict(productCommit='854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc',
                       runId='38765418126', runAttempt='1', status='failed',
                       artifactSha256='9ae383747ad38c25b700e72ad1ae57a7fbae17c153b4d80310eefeabdfc97353',
                       stage='packaged-updater-bootstrap/launcher-completion')


def diagnostic_text(raw):
    # Never persist environment dumps, wire frames or the 64-hex challenge.
    # Exact binary/source hashes are recorded separately from trusted files.
    text = raw.decode('utf-8', errors='replace').replace('\x00', '')
    text = re.sub(r'(?im)^.*(?:SKAGER_UPDATE_|SKAGER-UPDATE-(?:READY|WAIT|CONTINUE|CANCEL)/1).*$', '[redacted startup protocol/environment]', text)
    return re.sub(r'(?i)[a-f0-9]{64}', '[redacted 64-hex value]', text)


def wow64_system_directory():
    query = ctypes.WinDLL('kernel32', use_last_error=True).GetSystemWow64DirectoryW
    query.argtypes = (ctypes.POINTER(ctypes.c_wchar), ctypes.c_uint); query.restype = ctypes.c_uint
    capacity = 260  # Conventional MAX_PATH; never pass a 65536-byte WCHAR buffer.
    for attempt in range(2):
        system = ctypes.create_unicode_buffer(capacity)
        length = query(system, capacity)
        assert length > 0, 'Windows WOW64 system directory query failed'
        if length >= capacity:
            assert attempt == 0 and length < 32767, 'Windows WOW64 system directory exceeds the bounded retry'
            capacity = length + 1
            continue
        value = system.value
        assert value and len(value.encode('utf-16-le')) // 2 == length and Path(value).is_absolute(), 'Windows WOW64 directory must be nonempty, absolute and match the returned length'
        return Path(value)
    raise AssertionError('Windows WOW64 system directory query did not complete')


def diagnostic_host(launcher=None):
    # The packaged GOARCH=386 launcher calls GetSystemDirectory and leaves WOW64
    # redirection enabled. Explicitly select that x86 host from this x64 harness.
    host = wow64_system_directory() / 'WindowsPowerShell/v1.0/powershell.exe'
    if launcher is not None: bounded_plain(launcher, 64 * 1024 * 1024)
    for path in ((launcher, host) if launcher is not None else (host,)):
        # Serviced OS files may legitimately have WinSxS hard links.
        for ancestor in (path, *path.parents):
            assert not ancestor.is_symlink() and not (getattr(ancestor.lstat(), 'st_file_attributes', 0) & 0x400), 'Redirected diagnostic host refuses'
        assert path.is_file() and 64 <= path.stat().st_size <= 64 * 1024 * 1024
        with path.open('rb') as stream:
            header = stream.read(64)
            assert len(header) == 64 and header[:2] == b'MZ', 'Expected native x86 image'
            offset = int.from_bytes(header[60:64], 'little')
            assert 64 <= offset <= path.stat().st_size - 26, 'Invalid native PE header offset'
            stream.seek(offset); pe = stream.read(26)
            assert pe[:6] == b'PE\0\0\x4c\x01' and pe[24:26] == b'\x0b\x01', 'Diagnostic must use the original I386/PE32 host contract'
    return host


def capture_supervisor_diagnostic(arguments, directory, owned):
    """One original supervisor invocation; bounded memory, no raw output file."""
    environment = {key: value for key, value in os.environ.items() if not key.upper().startswith('SKAGER_UPDATE_')}
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    process = subprocess.Popen(arguments, cwd=directory, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               env=environment, startupinfo=startup)
    owned.add(process.pid)
    captures = {}
    lock = threading.Lock()
    def drain(name, stream):
        try:
            while chunk := stream.read1(4096):
                with lock:
                    record = captures[name]
                    record['bytes'] += len(chunk)
                    record['raw'].extend(chunk[:max(0, 65536 - len(record['raw']))])
        except OSError:
            with lock: captures[name]['readError'] = True
        finally:
            stream.close()
            with lock: captures[name]['complete'] = True
    readers = []
    for name, stream in (('stdout', process.stdout), ('stderr', process.stderr)):
        captures[name] = dict(raw=bytearray(), bytes=0, complete=False)
        reader = threading.Thread(target=drain, args=(name, stream), daemon=True)
        reader.start(); readers.append(reader)
    result = dict(exitCode=None, timedOut=False, timeoutSeconds=240)
    try:
        result['exitCode'] = process.wait(timeout=240)
    except subprocess.TimeoutExpired:
        result['timedOut'] = True  # Existing outer owned-process cleanup remains authoritative.
    finally:
        for reader in readers: reader.join(.25)
        if process.poll() is not None: owned.discard(process.pid)
    with lock:
        for name, record in captures.items():
            complete = record['complete'] and not record.get('readError', False)
            truncated = record['bytes'] > 65536
            raw = bytes(record['raw'])
            # A surviving app may inherit an output handle. Keep complete lines
            # already received without waiting for its exit or exporting a
            # partial final line/challenge. Oversized output is withheld.
            if not complete: raw = raw[:raw.rfind(b'\n') + 1]
            text = diagnostic_text(raw) if not truncated else '[output exceeds 65536-byte bound; withheld]'
            result[name] = dict(text=text, bytes=record['bytes'], complete=complete, truncated=truncated)
    return result


def startup_log(profile):
    path = profile / 'opencpn.log'
    if not path.exists():
        return b''
    with path.open('rb') as stream:
        content = stream.read(4 * 1024 * 1024 + 1)
    if len(content) > 4 * 1024 * 1024:
        raise RuntimeError('Installed startup log exceeds the observation bound')
    return content


def wait_startup_ready(profile, before):
    # Same fresh append/rotation policy and deadline as installer smoke. A
    # visible shell and an exited launcher do not prove deferred init finished:
    # pinned OpenCPN explicitly ignores WM_CLOSE until g_bDeferredInitDone.
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        if _startup.initialized_since(before, startup_log(profile)):
            time.sleep(.6)
            return
        time.sleep(.1)
    raise RuntimeError('Installed app did not complete fresh startup before close')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def strict_pairs(pairs):
    result = {}
    for key, value in pairs:
        assert key not in result, 'Duplicate trust/state field refuses'
        result[key] = value
    return result


def bounded_plain(path, limit, directory=False):
    for ancestor in (path, *path.parents):
        info = ancestor.lstat()
        assert not ancestor.is_symlink() and not (getattr(info, 'st_file_attributes', 0) & 0x400), 'Redirected trust/state object refuses'
    info = path.stat()
    if directory:
        assert path.is_dir(), 'Expected original trust/state directory'
    else:
        assert stat.S_ISREG(info.st_mode) and info.st_nlink == 1 and 0 < info.st_size <= limit, 'Bounded original trust/state file required'
    return info.st_dev, info.st_ino


def public_trust_identity(raw):
    # Go's json.RawMessage keeps this exact root-value span. Reserializing the
    # parsed object would produce a different bootstrap identity for indented JSON.
    text = raw.decode('utf-8')
    decoder = json.JSONDecoder(object_pairs_hook=strict_pairs)
    value = decoder.decode(text)
    assert isinstance(value, dict) and isinstance(value.get('metadataUrl'), str) and value.get('channel') == 'beta'
    cursor = text.index('{') + 1
    root = None
    while True:
        while text[cursor].isspace(): cursor += 1
        if text[cursor] == '}': break
        key, cursor = decoder.raw_decode(text, cursor)
        while text[cursor].isspace(): cursor += 1
        assert text[cursor] == ':'
        cursor += 1
        while text[cursor].isspace(): cursor += 1
        start = cursor
        _, cursor = decoder.raw_decode(text, cursor)
        if key == 'bootstrapRoot': root = text[start:cursor].encode('utf-8')
        while text[cursor].isspace(): cursor += 1
        if text[cursor] == '}': break
        assert text[cursor] == ','
        cursor += 1
    assert root is not None
    return {'bootstrapSha256': hashlib.sha256(root).hexdigest(), 'metadataUrl': value['metadataUrl'], 'channel': value['channel']}


def packaged_trust_identity(app, product):
    """Check the exact producer-selected trust without initializing or changing it."""
    path = app / 'update-trust.json'
    selected = product.get('update_trust')
    if selected is None:
        assert not path.exists() and not path.is_symlink(), 'Undeclared packaged trust refuses'
        return {'state': 'unconfigured'}
    assert isinstance(selected, dict) and type(selected.get('schema')) is int and selected['schema'] == 1
    assert selected.get('productCommit') == product['commit'] and selected.get('channel') == 'beta'
    assert selected.get('sourcePath') == 'opennav-x/installer/windows/staging-update-trust.json'
    expected = selected.get('configSha256')
    assert isinstance(expected, str) and re.fullmatch('[a-f0-9]{64}', expected)
    assert selected.get('sourceSha256') == expected, 'Packaged trust differs from committed source selection'
    bounded_plain(path, 576 << 10)
    assert digest(path) == expected, 'Original packaged public trust changed'
    return {'state': 'packaged-public-trust', 'configSha256': expected, **public_trust_identity(path.read_bytes())}


def legacy_trust_observation(install, identity, *, initial=False):
    if identity['state'] == 'unconfigured':
        for name in ('update-trust', 'update-artifacts', 'update-trust-initialized-beta', 'update-trust-initialized-stable'):
            path = install / name
            assert not path.exists() and not path.is_symlink(), 'Undeclared legacy trust state refuses'
        return {'state': 'unconfigured'}
    # Lifecycle intentionally invokes --initialize-trust after each committed
    # health-capable generation. Verify that original initialization, never undo it.
    marker = install / 'update-trust-initialized-beta'
    marker_id = bounded_plain(marker, 4096)
    expected = json.dumps(dict(schema=1, channel='beta', configSha256=identity['configSha256']), separators=(',', ':')).encode()
    assert marker.read_bytes() == expected, 'Original trust once marker changed'
    roots = {name: bounded_plain(install / name, 0, directory=True) for name in
             ('update-trust', 'update-artifacts', 'update-trust/beta')}
    channel = install / 'update-trust/beta'
    pending = channel / 'refresh.pending'
    assert not pending.exists() and not pending.is_symlink(), 'Interrupted trust refresh refuses'
    state_path = channel / 'state.json'
    bounded_plain(state_path, 32 << 10)
    state = json.loads(state_path.read_bytes(), object_pairs_hook=strict_pairs)
    assert set(state) == {'schema', 'bootstrapSha256', 'metadataUrl', 'channel', 'generation', 'metadata'}
    assert type(state['schema']) is int and state['schema'] == 1
    assert all(state[key] == identity[key] for key in ('bootstrapSha256', 'metadataUrl', 'channel')), 'Installed trust identity differs from original public bytes'
    assert isinstance(state['generation'], str) and re.fullmatch('[a-f0-9]{32}', state['generation'])
    metadata = state['metadata']
    assert isinstance(metadata, dict) and 'root.json' in metadata and 1 <= len(metadata) <= 128
    if initial:
        assert set(metadata) == {'root.json'} and metadata['root.json'] == identity['bootstrapSha256'], 'Initial retained metadata differs from original bootstrap bytes'
    cache = channel / state['generation'] / 'metadata'
    bounded_plain(cache, 0, directory=True)
    assert {p.name for p in cache.iterdir()} == set(metadata), 'Original retained metadata inventory differs'
    total, floors = 0, {}
    for name, pin in metadata.items():
        assert isinstance(name, str) and re.fullmatch('[A-Za-z0-9_%.-]+\\.json', name)
        assert isinstance(pin, str) and re.fullmatch('[a-f0-9]{64}', pin)
        path = cache / name
        bounded_plain(path, 5 << 20)
        total += path.stat().st_size
        assert total <= 32 << 20 and digest(path) == pin, 'Retained metadata changed'
        role = name[:-5].rsplit('.', 1)[-1]
        if role in ('root', 'timestamp', 'snapshot', 'targets'):
            signed = json.loads(path.read_bytes(), object_pairs_hook=strict_pairs)['signed']
            version = signed['version']
            assert type(version) is int and version > 0
            floors[role] = max(floors.get(role, 0), version)
    # Cache generations/metadata can legitimately advance after --xnav checks.
    # Only the once marker and original store roots must retain physical identity.
    return {'state': 'initialized-original-identity', 'markerIdentity': marker_id, 'rootIdentities': roots, 'floors': floors}


def legacy_trust_preserved(before, after):
    if before['state'] == 'unconfigured':
        return before == after
    return (all(before[key] == after[key] for key in ('state', 'markerIdentity', 'rootIdentities')) and
            all(after['floors'].get(role, 0) >= version for role, version in before['floors'].items()))


def qualify(*, install, setup, stock, profile, evidence, powershell, commit,
            executable_sha256, ui, owned, inventory, navigation_matches, product, source_app,
            startup_diagnostic=False):
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise RuntimeError('Real package qualification requires disposable Windows CI')
    assert install == Path(os.environ['LOCALAPPDATA']) / 'OpenNavXAlpha1'
    assert re.fullmatch('[a-f0-9]{40}', commit)
    assert re.fullmatch('[a-f0-9]{64}', executable_sha256)
    assert install.is_dir() and profile.is_dir() and setup.is_file() and stock.is_file()
    assert product['commit'] == commit and product['executable_sha256'] == executable_sha256
    trust_identity = packaged_trust_identity(source_app, product)
    output = evidence / 'packaged-updater.json'
    assert not output.exists(), 'Keep each qualification attempt separately'
    report = {'status': 'running', 'scope': 'real package, same-version supervised installation and authenticated startup; original selected legacy trust identity retained; no protected B0/B1 or signed-offer acceptance',
              'commit': commit, 'setupSha256': digest(setup), 'executableSha256': executable_sha256,
              'trustConfiguration': {k: v for k, v in trust_identity.items() if k in ('state', 'configSha256')},
              'legacyTrustState': 'not-verified',
              'nativeProtectedUpdaterAcceptance': False, 'protectedUpdaterGate': 'pending-real-B0-B1',
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

    original_legacy_trust, last_legacy_trust = None, None

    def generation(identifier):
        nonlocal last_legacy_trust
        assert re.fullmatch('[a-f0-9]{32}', identifier)
        directory = install / 'generations' / identifier
        record = read(directory / 'ownership.json')
        assert record['owner'] == 'OpenNavX.Alpha1.SideBySide.1'
        assert record['commit'] == commit and record['updateStartupHealth'] == 1
        require_product_output_policy({'xnav_hardware_output_policy': record['xnavHardwareOutputPolicy'],
                                       'xnav_manual_control_contract': record.get('xnavManualControlContract', 0)})
        assert packaged_trust_identity(directory / 'app', product) == trust_identity
        if trust_identity['state'] == 'packaged-public-trust':
            matches = [entry for entry in record['managedFiles'] if entry['path'] == 'app/update-trust.json']
            assert len(matches) == 1 and matches[0]['sha256'] == trust_identity['configSha256'], 'Installed ownership does not bind original public trust'
        observed_trust = legacy_trust_observation(install, trust_identity)
        assert legacy_trust_preserved(original_legacy_trust, observed_trust) and legacy_trust_preserved(last_legacy_trust, observed_trust), 'Original legacy trust identity or metadata floor was reset'
        last_legacy_trust = observed_trust
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
        before_log = startup_log(profile)
        start = time.monotonic()
        operation = {'phase': phase, 'generation': identifier, 'log': log.name,
                     'status': 'running', 'lastStage': 'launch'}
        report['operations'].append(operation)
        with log.open('xb') as stream:
            parent = subprocess.Popen(arguments, stdout=stream, stderr=subprocess.STDOUT)
            owned.add(parent.pid)
            try:
                # Locate the actual app child, not the launcher/supervisor PID.
                # Its final image must match the exact generation before any UI action.
                operation['lastStage'] = 'window-discovery'
                window, pid = ui.wait_window('SKAGER / OpenCPN', timeout=90)
                assert process_image(pid).samefile(executable), 'Wrong generation opened the application window'
                owned.add(pid)
                operation['lastStage'] = 'launcher-completion'
                assert parent.wait(timeout=max(1, 240-(time.monotonic()-start))) == 0, str(log)
                operation['lastStage'] = 'fresh-startup-readiness'
                wait_startup_ready(profile, before_log)
                operation['startupReadySeconds'] = round(time.monotonic()-start, 3)
                assert ui.IsWindowEnabled(window), 'Startup remains blocked by a modal dialog'
                if authenticate:
                    operation['lastStage'] = 'receipt-authentication'
                    audit_receipt(identifier, phase)
                    assert time.monotonic()-start >= 30, 'Real continuous startup health checkpoint did not elapse'
                # The production CloseMainWindow path remains separately tested;
                # this fixture requests the normal visible application close.
                monitor = ui.monitor_process(pid)
                operation['lastStage'] = 'normal-close'
                operation['closeRequestedSeconds'] = round(time.monotonic()-start, 3)
                ui.close(window)
                ui.wait_clean_exit(monitor, timeout_ms=30000)
                owned.discard(pid)
            except BaseException as error:
                operation.update(status='failed', error=repr(error),
                                 seconds=round(time.monotonic()-start, 3))
                # Keep the actual disposable app's bounded startup log, even if
                # stdout is empty. Capture failure must not replace the cause.
                try:
                    content = startup_log(profile)
                    saved = evidence / ('packaged-updater-' + phase + '-startup.log')
                    with saved.open('xb') as diagnostic:
                        diagnostic.write(content)
                    operation['startupLog'] = {'path': saved.name, 'bytes': len(content),
                                               'sha256': hashlib.sha256(content).hexdigest(),
                                               'freshInitialization': _startup.initialized_since(before_log, content)}
                except (OSError, RuntimeError) as capture_error:
                    operation['startupLogError'] = repr(capture_error)
                raise
            finally:
                if parent.poll() is not None:
                    owned.discard(parent.pid)
        assert navigation_matches(), 'Shared navigation fixtures changed across supervised startup'
        assert inventory(stock.parent) == stock_before, 'Supported stock installation changed'
        operation.update(status='passed', lastStage='complete',
                         seconds=round(time.monotonic()-start, 3))

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
        original_legacy_trust = legacy_trust_observation(install, trust_identity, initial=True)
        last_legacy_trust = original_legacy_trust
        initial = state()['current']
        first = generation(initial)
        assert not (install / 'known-good' / (initial+'.receipt')).exists(), 'Bootstrap must establish new actual proof'
        assert inventory(profile) == initial_profile
        if startup_diagnostic:
            assert commit == STARTUP_FAILURE['productCommit'] and os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted'
            assert not any(os.environ.get(key) for key in ('GH_TOKEN', 'GITHUB_TOKEN', 'OPENNAV_ARTIFACT_TOKEN'))
            assert not any(title == 'SKAGER / OpenCPN' for _, _, title in ui.windows())
            report.update(diagnosticOnly=True, releaseQualification=False, originalFailure=STARTUP_FAILURE,
                          scope='Original installed QualifyCurrent stderr diagnosis only; no launcher, installer or updater acceptance')
            host = diagnostic_host(first / 'app/skager-start.exe')
            paths = ('ownership.json', 'app/opencpn.exe', 'app/skager-start.exe', 'UpdateSupervisor.ps1', 'UpdateTransaction.ps1', 'Lifecycle.ps1')
            before = {path: digest(first / path) for path in paths}
            # The authenticated installer payload supplies these exact original scripts.
            # Do not dot-source, instrument, rewrite or substitute harness copies.
            arguments = [str(host), '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
                         '-File', str(first / 'UpdateSupervisor.ps1'), '-InstallationRoot', str(install),
                         '-Action', 'QualifyCurrent']
            observation = dict(generation=initial, hostPath=str(host), hostSha256=digest(host),
                               hostMachine='I386', workingDirectory=str(first / 'app'), inputs=before,
                               cleanupMayMaskReceiverReason=True, acceptance=False)
            report['supervisorDiagnostic'] = observation
            observation.update(capture_supervisor_diagnostic(arguments, first / 'app', owned))
            # Register any surviving visible exact app for the existing failure
            # screenshots and disposable runner cleanup; perform no UI action.
            for _, pid, _ in ui.windows():
                try:
                    if process_image(pid).samefile(first / 'app/opencpn.exe'): owned.add(pid)
                except OSError: pass
            observation['receiptPresent'] = (install / 'known-good' / (initial+'.receipt')).exists()
            observation['inputsUnchanged'] = before == {path: digest(first / path) for path in paths}
            assert observation['inputsUnchanged'], 'Original installed diagnostic inputs changed'
            assert generation(initial) == first and state()['current'] == initial and not pending_path.exists()
            assert digest(setup) == report['setupSha256'] and inventory(stock.parent) == stock_before
            assert navigation_matches(), 'Shared navigation fixtures changed during diagnosis'
            raise RuntimeError('Diagnostic completed without qualification; original launcher failure remains unresolved')
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
        assert packaged_trust_identity(source_app, product) == trust_identity, 'Original package trust changed during qualification'
        assert navigation_matches() and inventory(stock.parent) == stock_before
        assert legacy_trust_preserved(last_legacy_trust, legacy_trust_observation(install, trust_identity))
        report['legacyTrustState'] = original_legacy_trust['state']
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
    if sys.argv[1:] == ['--diagnostic-host-preflight']:
        assert sys.platform == 'win32' and os.environ.get('GITHUB_ACTIONS') == 'true' and os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'Disposable native CI required'
        host = diagnostic_host()
        print(json.dumps(dict(scope='Read-only WOW64 host preflight; no supervisor or application execution',
                              hostPath=str(host), hostSha256=digest(host), hostMachine='I386',
                              testSourceSha256=digest(Path(__file__)), acceptance=False), indent=2))
        raise SystemExit(0)
    raise SystemExit('Run through smoke-installer-windows.py with its disposable package/profile fixture')
