#!/usr/bin/env python3
"""Native pinned stock provisioning, optionally followed by SDK/configure proof."""
import argparse
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import sys
import shutil

import windows_gettext

SPEC = importlib.util.spec_from_file_location('stock', Path(__file__).with_name('prepare-windows-stock-deps.py'))
stock = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stock)


def run(command, label, evidence, timeout=300):
    result, _ = windows_gettext.native(command, timeout, evidence / label)
    if result['exitCode']:
        raise ValueError('Native stock preflight failed: ' + label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--bundle', type=Path)
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--provision-only', action='store_true',
                        help='Cold unchanged batch proof without SDK restore or CMake')
    args = parser.parse_args()
    if args.provision_only:
        if args.bundle or args.provenance:
            parser.error('--provision-only does not consume a dependency SDK')
    elif not args.bundle or not args.provenance:
        parser.error('Full configure requires --bundle and --provenance')
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Stock configure preflight requires disposable native Windows CI')
    root = stock.receipt._workspace_root(args.root)
    evidence = root / ('evidence/local/windows-stock-provision' if args.provision_only
                       else 'evidence/local/windows-stock-configure')
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.mkdir(exist_ok=False)
    source = root / 'build/integration-source'
    cache = root / stock.stage.CACHE
    python = sys.executable
    original_path, original_directory = os.environ['PATH'], Path.cwd()
    report = {'schema': 1, 'passed': False, 'applicationBuilt': False, 'applicationInstalled': False,
              'scope': ('Cold pinned stock batch provisioning only; no SDK, CMake or application'
                        if args.provision_only else
                        'Native Win32 stock provisioning, authenticated TLS restage and full CMake configure; optional private chart adapter omitted'),
              'commit': os.environ.get('GITHUB_SHA'),
              'wxAcquisition': 'The same three hash-pinned curl archives as the application driver; upstream wx wget branches skipped'}
    before = None if args.provision_only else stock.prefix_inventory(root)
    try:
        batch = (root / stock.BATCH).read_bytes().replace(b'\r\n', b'\n')
        if hashlib.sha256(batch).hexdigest() != stock.BATCH_SHA256:
            raise ValueError('Pinned upstream dependency batch changed')
        report['batchSha256'] = stock.BATCH_SHA256
        run([python, root / 'tools/windows_gettext.py', 'ensure', '--receipt', evidence / 'gettext.json'],
            'gettext-verify', evidence)
        gettext = stock.receipt._read_json(evidence / 'gettext.json')['directory']
        os.environ['PATH'] = gettext + ';' + original_path
        # Prove CMD resolves actual native tools before any sentinel invalidation.
        # Batch shims do not return to this upstream batch (which omits CALL).
        os.chdir(source)
        cmd = Path(os.environ['SystemRoot']) / 'System32/cmd.exe'
        where = Path(os.environ['SystemRoot']) / 'System32/where.exe'
        report['tools'] = {}
        for name, arguments in (('msgmerge', ['--version']), ('git', ['--version']),
                                ('wget', ['--version']), ('7z', ['i'])):
            resolved = shutil.which(name)
            if not resolved or Path(resolved).suffix.lower() != '.exe':
                raise ValueError('Required native executable unavailable: ' + name)
            result, output = windows_gettext.native([where, name], 30, evidence / ('where-' + name))
            matches = output.strip().splitlines()
            if result['exitCode'] or not matches or Path(matches[0]).resolve() != Path(resolved).resolve():
                raise ValueError('CMD prerequisite resolution is ambiguous: ' + name)
            windows_gettext.plain_file(Path(resolved).resolve())
            run([cmd, '/d', '/c', name, *arguments], 'probe-' + name, evidence, timeout=30)
            report['tools'][name] = {'path': resolved, **windows_gettext.identity(Path(resolved))}
        nsis = Path(os.environ['ProgramFiles(x86)']) / 'NSIS/makensis.exe'
        if not Path('C:/ProgramData/chocolatey/lib/nsis').is_dir() or not nsis.is_file():
            raise ValueError('Native NSIS and its upstream package sentinel are required')
        run([nsis, '/VERSION'], 'probe-nsis', evidence, timeout=30)
        report['tools']['nsis'] = {'path': str(nsis), **windows_gettext.identity(nsis)}
        os.chdir(original_directory)
        if args.provision_only:
            if cache.exists() or (source / 'cache/QuickStartGuide.zip').exists():
                raise ValueError('Provision-only proof requires a cold stock cache')
        else:
            report['preparation'] = stock.prepare(root, args.bundle, args.provenance)
            if not report['preparation']['removed']:
                raise ValueError('Cold AIS regression did not reproduce the maintained-curl-only cache')
        # Same exact wx archives as the app driver, before unchanged win_deps.
        wx = source / 'cache/wxWidgets-3.2.8'
        downloads = source / 'cache/opennav-downloads'
        downloads.mkdir(parents=True, exist_ok=True)
        lock = stock.receipt._read_json(root / 'tools/windows-wx.lock.json')
        for item in lock['archives']:
            archive = downloads / item['file']
            run(['curl.exe', '--fail', '--location', '--silent', '--show-error', '--retry', '3',
                 '--retry-all-errors', '--connect-timeout', '20', '--max-time', '180',
                 '--output', archive, item['url']], 'fetch-' + item['file'], evidence)
            stock.curl_package.verify_file(archive, {'sha256': item['sha256'], 'bytes': item['bytes']})
            run(['7z', 'x', '-y', '-o' + str(wx), archive], 'extract-' + item['file'], evidence)
        if not (wx / 'include/wx/version.h').is_file():
            raise ValueError('Verified wx extraction missing headers')
        os.chdir(source)
        run([cmd, '/d', '/c', 'buildwin\\win_deps.bat'],
            'unchanged-win-deps', evidence, timeout=600)
        os.chdir(original_directory)
        if not stock.stock_present(root):
            raise ValueError('Unchanged stock batch did not populate required support closure')
        report['stockFiles'] = stock.receipt._inventory(root, [stock.stage.CACHE + '/' + name
            for name in (*stock.STOCK_FILES, 'archive.lib')])
        report['stockArchive'] = {'sha256': stock.receipt._digest(source / 'cache/OCPNWindowsCoreBuildSupport.zip')}
        for name in ('archive.dll', 'liblzma.dll', 'glew32.dll', 'crashrpt/CrashRpt1403.dll',
                     'crashrpt/CrashSender1403.exe', 'crashrpt/dbghelp.dll'):
            stock.curl_package._require_win32_pe(cache / name)
        guide = source / 'cache/QuickStartGuide.zip'
        guide_files = [item for item in (source / 'data/doc/local').rglob('*')
                       if item.is_file() and item.stat().st_size]
        if not guide.is_file() or not guide.stat().st_size or not guide_files:
            raise ValueError('Unchanged batch did not acquire and extract QuickStartGuide')
        report['quickStartGuide'] = {**windows_gettext.identity(guide), 'extractedFiles': len(guide_files)}
        if not args.provision_only:
            stock.bundle_api.stage_bundle(root, args.bundle, args.provenance)
            manifest = stock.receipt._read_json(root / stock.stage.PREFIX['curl'] / 'curl-build.json')
            stock.curl_package.verify_file(cache / 'libcurl.dll', manifest['outputs']['bin/libcurl.dll'])
            report['restagedCurl'] = manifest['outputs']['bin/libcurl.dll']
            os.environ['PATH'] += ';' + str(wx / 'lib/vc14x_dll') + ';' + str(cache)
            build = evidence / 'build'
            run(['cmake', '-S', source, '-B', build, '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                 '-DPython3_EXECUTABLE:FILEPATH=' + python, '-DCMAKE_POLICY_VERSION_MINIMUM=3.5',
                 '-DCMAKE_BUILD_TYPE=Release', '-DwxWidgets_ROOT_DIR=' + str(wx),
                 '-DwxWidgets_LIB_DIR=' + str(wx / 'lib/vc14x_dll'), '-DwxWidgets_CONFIGURATION=mswu',
                 '-DOCPN_CI_BUILD=ON', '-DGETTEXT_MSGFMT_EXECUTABLE=' + gettext + '/msgfmt.exe',
                 '-DGETTEXT_MSGMERGE_EXECUTABLE=' + gettext + '/msgmerge.exe', '-DOCPN_BUILD_TEST=ON',
                 '-DOCPN_BUNDLE_WXDLLS=ON', '-DOCPN_BUNDLE_DOCS=OFF', '-DOCPN_BUNDLE_GSHHS=ON',
                 '-DOCPN_BUNDLE_TCDATA=ON', '-DCMAKE_INSTALL_PREFIX=' + str(evidence / 'not-installed'),
                 '-DOPENNAV_ROOT=' + str(root), '-DOPENNAV_ENABLE_ROUTE_SCENARIO=ON',
                 '-DXNAV_ENABLE_TEST_FIXTURES=ON', '-DXNAV_ENABLE_PILOT_LOOPBACK_TESTS=ON',
                 '-DSKAGER_OCHARTS_PACKAGE:PATH='], 'native-full-configure', evidence, timeout=600)
            values = {}
            for line in (build / 'CMakeCache.txt').read_text(encoding='utf-8').splitlines():
                if ':' in line and '=' in line:
                    key, value = line.split('=', 1)
                    values[key.partition(':')[0]] = value
            for key in ('LibArchive_LIBRARY', 'LibArchive_INCLUDE_DIR'):
                selected = Path(values[key])
                if not selected.is_absolute() or not selected.is_relative_to(cache) or not selected.exists():
                    raise ValueError('LibArchive resolved outside the stock cache: ' + key)
            if values['CMAKE_GENERATOR_PLATFORM'] != 'Win32':
                raise ValueError('Configure did not retain native Win32 ABI')
            report['libArchive'] = {key: values[key] for key in ('LibArchive_LIBRARY', 'LibArchive_INCLUDE_DIR')}
        report['passed'] = True
    finally:
        os.environ['PATH'] = original_path
        os.chdir(original_directory)
        report['prefixesUnchanged'] = (None if args.provision_only
                                      else stock.prefix_inventory(root) == before)
        if report['prefixesUnchanged'] is False:
            report['passed'] = False
        (evidence / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if report['prefixesUnchanged'] is False:
        raise ValueError('Native preflight changed authenticated producer prefixes')


if __name__ == '__main__':
    main()
