#!/usr/bin/env python3
"""Capture CI-only plugin-guard SDK/runtime inputs; this is not qualification.

The generated inventory hash is informational. A later runner still requires
an independently reviewed external pin and authenticated dependency bundle.
No compiler, candidate DLL, application, downloader or trust tool is executed.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

import windows_dependency_receipt as receipt
import windows_dependency_stage as stage

ROOT = Path(__file__).resolve().parents[1]
MAX_FILES = 20000
MAX_ENTRIES = 40000
MAX_FILE_BYTES = 256 * 1024 * 1024
MAX_TOTAL_BYTES = 2 * 1024 * 1024 * 1024
MAX_RUNTIME = 128
SDK_SUFFIXES = {'.h', '.hpp', '.hxx', '.inl', '.inc', '.rc', '.manifest',
                '.ico', '.cur', '.bmp', '.xpm', '.png', '.xml'}
EXTRA_SYSTEM = {'comdlg32.dll', 'comctl32.dll', 'winspool.drv', 'uxtheme.dll',
                'version.dll', 'winmm.dll', 'imm32.dll', 'oleacc.dll', 'dwmapi.dll',
                'msimg32.dll', 'rpcrt4.dll', 'usp10.dll'}
WX = 'build/integration-source/cache/wxWidgets-3.2.8'
BUILD = 'build/production-windows'
INSTALL = 'build/production-install'


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


runner = module('retained_plugin_runner', 'test-plugin-download-guard-windows.py')
native = module('retained_plugin_native', 'test-ais-runtime-windows.py')


def stamp(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_size,
            info.st_mtime_ns, info.st_ctime_ns, info.st_nlink,
            getattr(info, 'st_file_attributes', 0))


def common_stamp(info):
    # Windows path stat infers executable suffix bits and may expose birth time
    # as ctime, while descriptor stat reports handle mode/change time. Match
    # physical identity here, then recheck each complete source baseline below.
    return (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode), info.st_size,
            info.st_mtime_ns, info.st_nlink,
            bool(getattr(info, 'st_file_attributes', 0) & 0x400))


def plain(root, relative):
    path = receipt._plain_path(root, relative)
    if path.is_file() and path.stat().st_nlink != 1:
        raise ValueError('Hardlinked source refused: ' + relative)
    return path


def record(root, relative):
    path = plain(root, relative)
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_FILE_BYTES:
        raise ValueError('Non-file or oversized source: ' + relative)
    digest = hashlib.sha256()
    count = 0
    with path.open('rb') as stream:
        opened = os.fstat(stream.fileno())
        if common_stamp(opened) != common_stamp(before):
            raise ValueError('Source changed before read: ' + relative)
        opened_identity = stamp(opened)
        while block := stream.read(1024 * 1024):
            count += len(block)
            if count > before.st_size:
                raise ValueError('Source grew during read: ' + relative)
            digest.update(block)
        if stamp(os.fstat(stream.fileno())) != opened_identity:
            raise ValueError('Source changed during read: ' + relative)
    if stamp(plain(root, relative).lstat()) != stamp(before) or count != before.st_size:
        raise ValueError('Source changed after read: ' + relative)
    return {'bytes': count, 'sha256': digest.hexdigest()}, stamp(before)


def tree(root, relative, suffixes):
    base = plain(root, relative)
    if not base.is_dir():
        raise ValueError('Missing SDK directory: ' + relative)
    result = []
    pending = [relative]
    entries = 0
    while pending:
        directory = pending.pop()
        children = sorted(plain(root, directory).iterdir())
        entries += len(children)
        if entries > MAX_ENTRIES:
            raise ValueError('SDK entry limit exceeded')
        if len({p.name.casefold() for p in children}) != len(children):
            raise ValueError('Case-aliased SDK entries: ' + directory)
        for child in children:
            name = directory + '/' + child.name
            receipt._relative(name)
            selected = plain(root, name)
            if selected.is_dir():
                pending.append(name)
            elif selected.suffix.lower() in suffixes:
                result.append(name)
            else:
                raise ValueError('Unexpected SDK input type: ' + name)
    if not result:
        raise ValueError('Empty SDK directory: ' + relative)
    return sorted(result)


def production_selection(root, commit):
    cache_name = BUILD + '/CMakeCache.txt'
    cache = plain(root, cache_name).read_text(encoding='utf-8')
    def value(key, kinds):
        found = re.findall(r'^' + re.escape(key) + r':([^=]+)=(.*)$', cache, re.M)
        if len(found) != 1 or found[0][0] not in kinds.split('|') or not found[0][1]:
            raise ValueError('Missing/ambiguous production CMake selection: ' + key)
        return found[0][1].replace('\\', '/')
    if value('CMAKE_GENERATOR_PLATFORM', 'INTERNAL') != 'Win32':
        raise ValueError('Production CMake configuration is not Win32')
    include = value('LibArchive_INCLUDE_DIR', 'PATH')
    library = value('LibArchive_LIBRARY', 'FILEPATH')
    def source_relative(text):
        path = Path(text)
        if not path.is_absolute():
            raise ValueError('Production CMake input must be absolute')
        try:
            name = path.relative_to(root).as_posix()
        except ValueError:
            raise ValueError('Production CMake input escaped source root') from None
        plain(root, name)
        return name
    if source_relative(include) != stage.CACHE + '/include':
        raise ValueError('Production archive include differs from stock cache')
    archive = source_relative(library)
    if archive not in {stage.CACHE + '/archive.lib', stage.CACHE + '/libarchive.lib'}:
        raise ValueError('Production archive import differs from stock cache')
    if (source_relative(value('wxWidgets_ROOT_DIR', 'PATH|UNINITIALIZED')) != WX or
            source_relative(value('wxWidgets_LIB_DIR', 'PATH|UNINITIALIZED')) != WX + '/lib/vc14x_dll' or
            value('wxWidgets_CONFIGURATION', 'STRING|UNINITIALIZED') != 'mswu'):
        raise ValueError('Production wx SDK/configuration differs')
    header_name = BUILD + '/include/OpenNavBuild.h'
    header = plain(root, header_name).read_text(encoding='utf-8-sig')
    if re.findall(r'^#define OPENNAV_BUILD_COMMIT "([0-9a-f]{40})"\s*$', header, re.M) != [commit]:
        raise ValueError('Production source commit differs')
    return archive, [cache_name, header_name]


def check_source_names(root, sources):
    # Include directory names, not only the projected inventory's file names.
    directories = {root}
    for source in sources:
        directories.update(p for p in (root / source).parents if p == root or root in p.parents)
    count = 0
    for directory in sorted(directories):
        children = list(directory.iterdir())
        count += len(children)
        if count > MAX_ENTRIES:
            raise ValueError('Source parent entry limit exceeded')
        if len({p.name.casefold() for p in children}) != len(children):
            raise ValueError('Case-aliased source parent entries')


def plan(root, commit, expected, inspect):
    archive, controls = production_selection(root, commit)
    sources = {}
    output_names = set()
    def add(source, target):
        receipt._relative(target)
        if target.casefold() in output_names:
            raise ValueError('Duplicate/case-aliased output input')
        if len(sources) >= MAX_FILES:
            raise ValueError('Input file limit exceeded')
        plain(root, source)
        output_names.add(target.casefold())
        sources[target] = source
    curl_prefix = stage.PREFIX['curl'] + '/include/curl/'
    curl_headers = tree(root, stage.PREFIX['curl'] + '/include/curl', SDK_SUFFIXES)
    if set(curl_headers) != {n for n in expected if n.startswith(curl_prefix)}:
        raise ValueError('Maintained curl header inventory differs')
    for source in curl_headers:
        add(source, 'sdk/curl/include/curl/' + source[len(curl_prefix):])
    add(stage.PREFIX['curl'] + '/lib/libcurl.lib', 'sdk/curl/lib/libcurl.lib')
    for name in ('archive.h', 'archive_entry.h'):
        add(stage.CACHE + '/include/' + name, 'sdk/archive/include/' + name)
    archive_target = 'sdk/archive/lib/' + Path(archive).name
    add(archive, archive_target)
    for source in tree(root, WX + '/include', SDK_SUFFIXES):
        add(source, 'sdk/wx/include/' + source[len(WX + '/include/'):])
    wx_lib = WX + '/lib/vc14x_dll'
    children = sorted(plain(root, wx_lib).iterdir())
    if len(children) > MAX_ENTRIES or len({p.name.casefold() for p in children}) != len(children):
        raise ValueError('Excessive/case-aliased wx import entries')
    libraries = [p for p in children if p.suffix.lower() == '.lib']
    if not libraries:
        raise ValueError('Missing wx import libraries')
    for path in libraries:
        add(wx_lib + '/' + path.name, 'sdk/wx/lib/vc14x_dll/' + path.name)
    for source in tree(root, wx_lib + '/mswu', {'.h', '.hpp', '.inl'}):
        add(source, 'sdk/wx/lib/vc14x_dll/mswu/' + source[len(wx_lib + '/mswu/'):])
    install = plain(root, INSTALL)
    children = list(install.iterdir())
    if len(children) > MAX_ENTRIES:
        raise ValueError('Production install entry limit exceeded')
    dlls = {}
    for path in children:
        if path.suffix.lower() == '.dll':
            name = path.name.lower()
            if name in dlls or not re.fullmatch(r'[a-z0-9_.-]+\.dll', name):
                raise ValueError('Invalid/case-aliased production DLL')
            dlls[name] = INSTALL + '/' + path.name
    runtime, imports, runtime_bytes = {}, {}, 0
    pending = set(runner.DLLS)
    system = native.SYSTEM_DLLS | EXTRA_SYSTEM
    while pending:
        name = min(pending)
        pending.remove(name)
        if name in runtime:
            continue
        if name not in dlls:
            raise ValueError('Required same-install runtime missing: ' + name)
        source = dlls[name]
        # Inspect only selected, bounded, plain files. Never load them.
        runtime_bytes += plain(root, source).stat().st_size
        if runtime_bytes > MAX_TOTAL_BYTES:
            raise ValueError('Runtime byte limit exceeded')
        record(root, source)
        dependencies = set(inspect(plain(root, source)))
        if not dependencies or any(not re.fullmatch(r'[a-z0-9_.-]+\.(dll|drv)', n) for n in dependencies):
            raise ValueError('Missing/invalid dumpbin dependencies: ' + name)
        imports[name] = sorted(dependencies)
        for dependency in dependencies:
            if dependency in system or dependency.startswith('api-ms-win-'):
                continue
            if dependency not in dlls or not dependency.endswith('.dll'):
                raise ValueError('Unresolved production runtime import: ' + dependency)
            pending.add(dependency)
        runtime[name] = 'runtime/' + name
        add(source, runtime[name])
        if len(runtime) > MAX_RUNTIME:
            raise ValueError('Runtime closure limit exceeded')
    inputs = {'curlInclude': 'sdk/curl/include', 'curlImport': 'sdk/curl/lib/libcurl.lib',
              'archiveInclude': 'sdk/archive/include', 'archiveImport': archive_target,
              'wxRoot': 'sdk/wx', 'wxLib': 'sdk/wx/lib/vc14x_dll'}
    check_source_names(root, list(sources.values()) + controls)
    return sources, inputs, runtime, imports, controls


def snapshot(root, sources):
    if len(sources) > MAX_FILES:
        raise ValueError('Input file limit exceeded')
    result, total = {}, 0
    for target, source in sorted(sources.items()):
        total += plain(root, source).stat().st_size
        if total > MAX_TOTAL_BYTES:
            raise ValueError('Input byte limit exceeded')
        result[target] = record(root, source)
    return result


def report_bytes(data):
    content = (json.dumps(data, sort_keys=True, indent=2) + '\n').encode('utf-8')
    if len(content) > receipt.MAX_RECEIPT_BYTES:
        raise ValueError('Capture report byte limit exceeded')
    return content


def capture(root, output, commit, expected, inspect, provenance, verify_producer=None):
    """Pure filesystem projection; host/producer authentication is the CLI's job."""
    root = receipt._workspace_root(root)
    output = output.absolute()
    receipt._workspace_root(output.parent)
    if output == root or root in output.parents:
        raise ValueError('Capture output must be outside producer source root')
    output.mkdir(mode=0o700, exist_ok=False)
    sources, inputs, runtime, imports, controls = plan(root, commit, expected, inspect)
    before = snapshot(root, sources)
    control_before = {name: record(root, name) for name in controls}
    data = {'schema': 1, 'candidateCommit': commit, 'inputs': inputs, 'runtime': runtime,
            'files': {name: item[0] for name, item in before.items()}}
    runner.bind_tls(data, expected)
    data_root = output / 'data'
    data_root.mkdir(mode=0o700)
    for target, source in sorted(sources.items()):
        destination = data_root / target
        destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        item, identity = before[target]
        path = plain(root, source)
        path_before = path.lstat()
        if stamp(path_before) != identity:
            raise ValueError('Source changed before copy: ' + source)
        digest, count = hashlib.sha256(), 0
        with path.open('rb') as source_stream, destination.open('xb') as target_stream:
            opened = os.fstat(source_stream.fileno())
            if common_stamp(opened) != common_stamp(path_before):
                raise ValueError('Source replaced before copy: ' + source)
            opened_identity = stamp(opened)
            while block := source_stream.read(1024 * 1024):
                count += len(block)
                if count > item['bytes']:
                    raise ValueError('Source grew during copy: ' + source)
                digest.update(block)
                target_stream.write(block)
            if stamp(os.fstat(source_stream.fileno())) != opened_identity:
                raise ValueError('Source changed during copy: ' + source)
        if stamp(plain(root, source).lstat()) != identity:
            raise ValueError('Source changed after copy: ' + source)
        if {'bytes': count, 'sha256': digest.hexdigest()} != item:
            raise ValueError('Copied bytes differ: ' + source)
        if record(data_root, target)[0] != item:
            raise ValueError('Retained bytes differ: ' + target)
    if (plan(root, commit, expected, inspect) != (sources, inputs, runtime, imports, controls) or
            snapshot(root, sources) != before or
            {name: record(root, name) for name in controls} != control_before):
        raise ValueError('Producer source changed during capture')
    if verify_producer is not None:
        verify_producer()
    pending = output / 'inventory.pending.json'
    with pending.open('xb') as stream:
        stream.write(report_bytes(data))
    pin = receipt._digest(pending)
    # Schema/copy consistency only; this computed hash is NOT an external pin.
    runner.retained_inputs(data_root, pending, pin, commit)
    metadata = dict(provenance, schema=1, status='captured-not-qualified',
                    candidateCommit=commit, inventorySha256=pin,
                    requiresIndependentInventoryPin=True, nativeAcceptance=False,
                    productAcceptance=False, boatAcceptance=False,
                    sources=sources, runtimeImports=imports,
                    productionControls={n: item[0] for n, item in control_before.items()})
    with (output / 'capture.json').open('xb') as stream:
        stream.write(report_bytes(metadata))
    pending.rename(output / 'inventory.json')
    return metadata


def host_guard(root, commit, run_id, attempt):
    runner.host_guard()
    if (not re.fullmatch('[0-9a-f]{40}', commit) or
            not re.fullmatch('[1-9][0-9]*', run_id) or not re.fullmatch('[1-9][0-9]*', attempt) or
            os.environ.get('GITHUB_SHA') != commit or os.environ.get('GITHUB_RUN_ID') != run_id or
            os.environ.get('GITHUB_RUN_ATTEMPT') != attempt or
            os.environ.get('GITHUB_JOB') != 'windows-integration' or
            root != ROOT or root != Path(os.environ.get('GITHUB_WORKSPACE', '')) / 'opennav-x'):
        raise ValueError('Capture requires exact native producer commit/run/attempt/source root')
    current = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if current != commit:
        raise ValueError('Checkout differs from producer commit')
    subprocess.run(['git', '-C', str(root), 'diff', '--exit-code', 'HEAD', '--', '.'],
                   check=True, stdout=subprocess.DEVNULL)


def dumpbin_inspector(root):
    files = list(plain(root, BUILD + '/CMakeFiles').glob('*/CMakeCXXCompiler.cmake'))
    if len(files) != 1:
        raise ValueError('Missing/ambiguous production compiler metadata')
    compilers = []
    for path in files:
        name = path.relative_to(root).as_posix()
        text = plain(root, name).read_text(encoding='utf-8')
        compilers += re.findall(r'set\(CMAKE_CXX_COMPILER "([^"]+)"\)', text)
    if len(compilers) != 1 or not re.search(r'/Host[^/]+/x86/cl\.exe$', compilers[0].replace('\\', '/'), re.I):
        raise ValueError('Requires exact production MSVC x86 compiler selection')
    tool = Path(compilers[0]).with_name('dumpbin.exe')
    tool_root = receipt._workspace_root(tool.parent)
    tool_before = record(tool_root, tool.name)
    selection_name = files[0].relative_to(root).as_posix()
    selection_before = record(root, selection_name)
    def inspect(path):
        native.win32(path)
        # dumpbin reads PE metadata; it does not execute the inspected DLL.
        result = subprocess.run([str(tool), '/DEPENDENTS', str(path)], check=True,
                                capture_output=True, timeout=30)
        if len(result.stdout) + len(result.stderr) > 1024 * 1024:
            raise ValueError('Excessive dumpbin output')
        if (record(tool_root, tool.name) != tool_before or
                record(root, selection_name) != selection_before):
            raise ValueError('Production dumpbin changed')
        return {n.lower() for n in re.findall(r'^\s*([\w.-]+\.(?:dll|drv))\s*$',
                                             result.stdout.decode('utf-8', errors='replace'), re.M | re.I)}
    return inspect, {'path': str(tool), **tool_before[0],
                     'compilerSelection': {'path': selection_name, **selection_before[0]}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--producer-commit', required=True)
    parser.add_argument('--producer-run-id', required=True)
    parser.add_argument('--producer-run-attempt', required=True)
    parser.add_argument('--architecture', choices=['Win32'], required=True)
    parser.add_argument('--dependency-bundle', type=Path, required=True)
    parser.add_argument('--dependency-bundle-provenance', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = receipt._workspace_root(args.root.absolute())
    host_guard(root, args.producer_commit, args.producer_run_id, args.producer_run_attempt)
    expected, authority = native.dependency_inputs(root, args.dependency_bundle, args.dependency_bundle_provenance)
    inspect, tool = dumpbin_inspector(root)
    provenance = {'producer': {'commit': args.producer_commit, 'runId': args.producer_run_id,
                              'runAttempt': args.producer_run_attempt, 'job': 'windows-integration',
                              'architecture': args.architecture, 'sourceRoot': str(root)},
                  'dependencyAuthority': authority, 'dumpbin': tool,
                  'helperSha256': receipt._digest(Path(__file__))}
    def verify_producer():
        host_guard(root, args.producer_commit, args.producer_run_id, args.producer_run_attempt)
        if native.dependency_inputs(root, args.dependency_bundle, args.dependency_bundle_provenance) != (expected, authority):
            raise ValueError('Dependency authority changed during capture')
    capture(root, args.output, args.producer_commit, expected, inspect, provenance, verify_producer)
    print('Captured inputs only; independent inventory review/pin and native qualification remain required.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print('Plugin input capture refused: ' + str(error), file=sys.stderr)
        sys.exit(1)
