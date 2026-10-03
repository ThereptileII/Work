#!/usr/bin/env python3
"""Prepare locally, then explicitly stage/run an offline boat HTML reference.

Never installs a browser or fonts, accesses a normal browser profile, starts
OpenCPN, changes display settings, or registers global Python packages. The
Windows run command is a separate action, not a side effect of preparation.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import stat
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def plain(path):
    for part in (path, *path.parents):
        info = part.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
            raise ValueError('Linked/reparse staging path refused')


def inventory(root):
    result = {}
    for path in sorted(root.rglob('*')):
        plain(path)
        if path.is_file():
            result[path.relative_to(root).as_posix()] = digest(path.read_bytes())
        elif not path.is_dir():
            raise ValueError('Non-regular runtime path')
    return result


def prepare(wheels, output):
    """No downloads: consume only locally inspected, hash-locked wheel bytes."""
    spec = importlib.util.spec_from_file_location('reference_render', HERE/'render.py')
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    original = renderer.verify_original()
    lock = json.loads((HERE/'boat-reference-wheels.json').read_text())
    paths = ['tools/prototype/render.py', 'tools/prototype/boat-reference.py',
             'tools/prototype/boat-reference-wheels.json', 'docs/design/prototype-original.json']
    paths += ['docs/design/prototype/'+item['path'] for item in original['files']]
    source_commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    payload = {'source/'+name: (ROOT/name).read_bytes() for name in paths}
    for name in paths:
        committed = subprocess.check_output(['git', 'show', source_commit+':'+name], cwd=ROOT)
        if committed != payload['source/'+name]:
            raise ValueError('Bundle source must be committed byte-exact: '+name)
    requirements = []
    for item in lock['packages']:
        path = wheels/item['filename']
        plain(path)
        data = path.read_bytes()
        if len(data) != item['bytes'] or digest(data) != item['sha256']:
            raise ValueError('Wheel differs from reviewed lock: '+item['filename'])
        payload['wheels/'+item['filename']] = data
        requirements.append(f"{item['name']}=={item['version']} --hash=sha256:{item['sha256']}")
    payload['requirements.lock'] = ('\n'.join(requirements)+'\n').encode()
    record = {'schema': 1, 'tooling_commit': source_commit,
              'scope': 'Immutable HTML and private reference tools, not an application package',
              'files': {name: digest(data) for name, data in sorted(payload.items())}}
    payload['bundle-manifest.json'] = (json.dumps(record, indent=2)+'\n').encode()
    with zipfile.ZipFile(output, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(payload.items()):
            item = zipfile.ZipInfo(name, date_time=(2026, 10, 3, 0, 0, 0))
            item.external_attr = (stat.S_IFREG | 0o644) << 16
            archive.writestr(item, data, compress_type=zipfile.ZIP_DEFLATED)
    return {'archive': str(output.resolve()), 'sha256': digest(output.read_bytes()),
            'bytes': output.stat().st_size, 'files': len(payload), 'tooling_commit': source_commit}


def unpack(bundle, expected, destination):
    plain(bundle)
    if not re.fullmatch('[0-9a-f]{64}', expected) or digest(bundle.read_bytes()) != expected:
        raise ValueError('Bundle differs from independently approved SHA256')
    plain(destination.parent)
    if any(p.name.casefold() == destination.name.casefold() for p in destination.parent.iterdir()):
        raise ValueError('Fresh destination or case alias already exists')
    with zipfile.ZipFile(bundle) as archive:
        names = set()
        for item in archive.infolist():
            rel = PurePosixPath(item.filename)
            if (not rel.parts or rel.is_absolute() or rel.as_posix() != item.filename or
                    any(p in ('.', '..') or ':' in p or p.endswith((' ', '.')) for p in rel.parts) or
                    '\\' in item.filename or item.filename.casefold() in names or
                    stat.S_IFMT(item.external_attr >> 16) != stat.S_IFREG or item.external_attr & (0x10 | 0x400)):
                raise ValueError('Unsafe/duplicate/nonregular bundle entry')
            names.add(item.filename.casefold())
        if archive.testzip() is not None:
            raise ValueError('Bundle CRC failed')
        record = json.loads(archive.read('bundle-manifest.json'))
        if record['schema'] != 1 or set(archive.namelist()) != set(record['files']) | {'bundle-manifest.json'}:
            raise ValueError('Bundle inventory differs')
        for name, expected_hash in record['files'].items():
            if digest(archive.read(name)) != expected_hash:
                raise ValueError('Bundle payload differs: '+name)
        destination.mkdir()
        archive.extractall(destination)
    return record


def logged_run(command, log, environment):
    """Native Windows job: descendants cannot run before ownership is established.

    This wrapper is used only by the explicitly invoked CPython 3.13 Windows
    staging process. Popen's retained handle identifies the exact owned child;
    no process-name/PID lookup can accidentally target an unrelated process.
    """
    import ctypes
    from ctypes import wintypes as w
    class BasicLimits(ctypes.Structure):
        _fields_ = [('PerProcessUserTimeLimit',ctypes.c_longlong),('PerJobUserTimeLimit',ctypes.c_longlong),
                    ('LimitFlags',w.DWORD),('MinimumWorkingSetSize',ctypes.c_size_t),
                    ('MaximumWorkingSetSize',ctypes.c_size_t),('ActiveProcessLimit',w.DWORD),
                    ('Affinity',ctypes.c_size_t),('PriorityClass',w.DWORD),('SchedulingClass',w.DWORD)]
    class IoCounters(ctypes.Structure):
        _fields_ = [(name,ctypes.c_ulonglong) for name in
                    ('ReadOperationCount','WriteOperationCount','OtherOperationCount',
                     'ReadTransferCount','WriteTransferCount','OtherTransferCount')]
    class ExtendedLimits(ctypes.Structure):
        _fields_ = [('BasicLimitInformation',BasicLimits),('IoInfo',IoCounters),
                    ('ProcessMemoryLimit',ctypes.c_size_t),('JobMemoryLimit',ctypes.c_size_t),
                    ('PeakProcessMemoryUsed',ctypes.c_size_t),('PeakJobMemoryUsed',ctypes.c_size_t)]
    api=ctypes.WinDLL('kernel32',use_last_error=True)
    api.CreateJobObjectW.argtypes=[ctypes.c_void_p,w.LPCWSTR];api.CreateJobObjectW.restype=w.HANDLE
    api.SetInformationJobObject.argtypes=[w.HANDLE,ctypes.c_int,ctypes.c_void_p,w.DWORD]
    api.SetInformationJobObject.restype=w.BOOL
    api.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE];api.AssignProcessToJobObject.restype=w.BOOL
    api.CloseHandle.argtypes=[w.HANDLE];api.CloseHandle.restype=w.BOOL
    job=api.CreateJobObjectW(None,None)
    if not job: raise ctypes.WinError(ctypes.get_last_error())
    child=None
    try:
        limits=ExtendedLimits();limits.BasicLimitInformation.LimitFlags=0x2000  # KILL_ON_JOB_CLOSE
        if not api.SetInformationJobObject(job,9,ctypes.byref(limits),ctypes.sizeof(limits)):
            raise ctypes.WinError(ctypes.get_last_error())
        gate="import subprocess,sys; go=sys.stdin.buffer.read(1); sys.exit(subprocess.call(sys.argv[1:]) if go==b'G' else 125)"
        with log.open('wb') as stream:
            child=subprocess.Popen([sys.executable,'-B','-c',gate,*command],stdin=subprocess.PIPE,
                                   stdout=stream,stderr=subprocess.STDOUT,env=environment,close_fds=True)
            if not api.AssignProcessToJobObject(job,int(child._handle)):
                raise ctypes.WinError(ctypes.get_last_error())
            child.stdin.write(b'G');child.stdin.close()
            result=child.wait(timeout=180)
        if result:
            raise RuntimeError(f'Owned reference command failed ({result}); retained {log.name}')
    finally:
        # Closing the job kills any remaining owned child/driver/browser even
        # when the root command timed out or exited with residual descendants.
        closed=api.CloseHandle(job)
        if child is not None:
            if child.stdin and not child.stdin.closed: child.stdin.close()
            try: child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()  # Exact retained child handle; never image-name kill.
                child.wait(timeout=10)
        if not closed: raise ctypes.WinError(ctypes.get_last_error())


def run(args):
    if sys.platform != 'win32' or sys.version_info[:2] != (3, 13) or platform.architecture()[0] != '64bit':
        raise ValueError('This locked runtime requires native Windows CPython 3.13 x64')
    destination = args.destination.absolute()
    if destination.parent != Path('C:/XNav/tools') or not re.fullmatch(r'prototype-reference-[a-z0-9-]+', destination.name):
        raise ValueError('Fresh private C:\\XNav\\tools\\prototype-reference-* destination required')
    # Creating this one dedicated tools parent is the only shared-directory write.
    plain(destination.parent.parent)
    if not destination.parent.exists():
        destination.parent.mkdir()
    record = unpack(args.bundle.absolute(), args.expected_bundle_sha256, destination)
    if record['files']['source/tools/prototype/boat-reference.py'] != digest(Path(__file__).read_bytes()):
        raise ValueError('Executing staging tool differs from the approved bundle')
    receipt = {'schema': 1, 'bundle_sha256': args.expected_bundle_sha256, 'bundle': record,
               'python': {'path': sys.executable, 'version': sys.version,
                          'sha256': digest(Path(sys.executable).read_bytes())},
               'result': 'started', 'process_ownership': 'Windows Job Object, KILL_ON_JOB_CLOSE; gated child before spawn',
               'scope': 'Headless same-machine font reference; not physical display acceptance'}
    receipt_path = destination/'runtime-receipt.json'
    environment = os.environ.copy()
    environment.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
                       PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD='1', PIP_DISABLE_PIP_VERSION_CHECK='1')
    environment.pop('PYTHONPATH', None)
    try:
        # No dependency resolution or network; every transitive distribution is
        # already pinned by version and wheel hash, including licenses/metadata.
        logged_run([sys.executable, '-B', '-m', 'pip', '--isolated', 'install', '--no-index', '--no-deps',
                    '--no-cache-dir', '--no-compile', '--require-hashes', '--only-binary=:all:',
                    '--find-links', str(destination/'wheels'), '--target', str(destination/'runtime'),
                    '-r', str(destination/'requirements.lock')], destination/'runtime-install.log', environment)
        receipt['runtime_files'] = inventory(destination/'runtime')
        receipt['dependency_note'] = ('Pinned distributions unmodified, including unused recorder/trace-viewer Codicon assets; '
                                      'no Windows fonts installed and reference page never loads these assets')
        receipt_path.write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
        environment['PYTHONPATH'] = str(destination/'runtime')
        logged_run([sys.executable, '-B', str(destination/'source/tools/prototype/render.py'),
                    '--output', str(destination/'capture'), '--states', 'navigation',
                    '--themes', 'day', 'dusk', 'night', '--scale', '1', '--browser-channel', 'msedge',
                    '--browser-executable', str(args.browser_executable),
                    '--expected-browser-sha256', args.expected_browser_sha256,
                    '--runtime-receipt', str(receipt_path)], destination/'render.log', environment)
        if inventory(destination/'runtime') != receipt['runtime_files']:
            raise ValueError('Private runtime changed during capture')
        for name, expected in record['files'].items():
            if digest((destination/name).read_bytes()) != expected:
                raise ValueError('Staged input changed during capture: '+name)
        receipt['result'] = 'PASS'
    except Exception as error:
        receipt['result'], receipt['error'] = 'FAILED', str(error)
        raise
    finally:
        # Keep runtime receipt stable: capture.json embeds its exact pre-run hash.
        (destination/'completion.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    prepare_parser = commands.add_parser('prepare')
    prepare_parser.add_argument('--wheels', type=Path, required=True)
    prepare_parser.add_argument('--output', type=Path, required=True)
    run_parser = commands.add_parser('run')
    run_parser.add_argument('--bundle', type=Path, required=True)
    run_parser.add_argument('--expected-bundle-sha256', required=True)
    run_parser.add_argument('--destination', type=Path, required=True)
    run_parser.add_argument('--browser-executable', type=Path, required=True)
    run_parser.add_argument('--expected-browser-sha256', required=True)
    args = parser.parse_args()
    if args.command == 'prepare':
        print(json.dumps(prepare(args.wheels, args.output), indent=2))
    else:
        run(args)


if __name__ == '__main__':
    main()
