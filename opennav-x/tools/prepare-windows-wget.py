#!/usr/bin/env python3
"""Supply a pinned native wget on disposable Windows CI, without Chocolatey.

This consumer-only tool does not change win_deps.bat or authenticated SDK inputs.
Vendor binaries and GPL documentation stay in RUNNER_TEMP, outside evidence and
product packages. Native stock provisioning is a separate required check.
"""
import argparse
import json
import os
from pathlib import Path
import re
import stat
import struct
import sys
import tempfile
import zipfile

import windows_gettext as native_tools

# Jernej Simoncic's Windows build, not an official GNU binary distribution.
# Exact Chocolatey package provenance, including vendor URL, checksum and GPL:
# https://github.com/chocolatey-community/chocolatey-packages/tree/ad69a3374a630f9c49ec2b803ce8128591674895/automatic/wget
# 1.25.0 fixes the vendor's older writable system-wgetrc location; see vendor page.
SOURCE_COMMIT = 'ad69a3374a630f9c49ec2b803ce8128591674895'
VERSION = '1.25.0'
URL = 'https://eternallybored.org/misc/wget/releases/wget-1.25.0-win64.zip'
ARCHIVE = {'bytes': 5900718,
           'sha256': '7567cbc08a1616931162b1c05f1731316853c32c79a21af1e850ccb673d1620c'}
EXECUTABLE = {'bytes': 8555792,
              'sha256': '28068376aab98ff7747ae1792fcb902aadbadc3ef25ca0422724387984cbcd5d'}
MEMBERS = {'COPYING', 'wget.exe', 'wget.exe.debug', 'wget.html', 'wget.man.html'}


def require_ci():
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_OS') != 'Windows' or os.environ.get('RUNNER_ARCH') != 'X64'):
        raise RuntimeError('Wget preparation requires disposable Windows x64 CI; never the boat')


def plain_directory(path):
    if not path.is_absolute() or '..' in path.parts:
        raise RuntimeError('Required tool directory is not absolute and plain')
    for component in (path, *path.parents):
        info = component.lstat()
        if (not stat.S_ISDIR(info.st_mode) or
                getattr(info, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT):
            raise RuntimeError('Required tool directory contains a link/reparse point')


def verify_file(path, expected):
    native_tools.plain_file(path)
    if path.stat().st_size != expected['bytes'] or native_tools.identity(path) != expected:
        raise RuntimeError('Pinned Wget size/checksum mismatch')


def extract(archive, directory):
    verify_file(archive, ARCHIVE)
    plain_directory(directory)
    with zipfile.ZipFile(archive) as bundle:
        entries = bundle.infolist()
        # Fixed leaf names exclude traversal, drive paths, case collisions and
        # unexpected executable/configuration additions before any extraction.
        if len(entries) != len(MEMBERS) or {item.filename for item in entries} != MEMBERS:
            raise RuntimeError('Pinned Wget archive member inventory mismatch')
        for item in entries:
            kind = stat.S_IFMT(item.external_attr >> 16)
            if kind not in (0, stat.S_IFREG) or item.is_dir() or item.flag_bits & 1:
                raise RuntimeError('Pinned Wget archive contains a nonregular/encrypted member')
        for item in entries:
            with (directory / item.filename).open('xb') as target:
                target.write(bundle.read(item))
    executable = directory / 'wget.exe'
    verify_file(executable, EXECUTABLE)
    data = executable.read_bytes()
    if len(data) < 64 or data[:2] != b'MZ':
        raise RuntimeError('Pinned Wget is not a native Windows x64 PE')
    offset = struct.unpack_from('<I', data, 0x3c)[0]
    if (offset + 6 > len(data) or data[offset:offset + 4] != b'PE\0\0' or
            struct.unpack_from('<H', data, offset + 4)[0] != 0x8664):
        raise RuntimeError('Pinned Wget is not a native Windows x64 PE')
    return {name: native_tools.identity(directory / name) for name in sorted(MEMBERS)}


def prepare(evidence):
    require_ci()
    evidence = evidence.absolute()
    evidence.mkdir(parents=True, exist_ok=False)
    state = {'schemaVersion': 1, 'status': 'failed', 'phase': 'preparation',
             'url': URL, 'version': VERSION, 'packageSourceCommit': SOURCE_COMMIT,
             'vendor': 'Jernej Simoncic Windows build',
             'vendorSource': 'https://eternallybored.org/misc/wget/src/',
             'license': 'GPL-3.0-or-later', 'expectedArchive': ARCHIVE,
             'expectedExecutable': EXECUTABLE, 'nativeProductAcceptance': False}
    try:
        runner_temp = Path(os.environ['RUNNER_TEMP'])
        plain_directory(runner_temp)
        github_path = Path(os.environ['GITHUB_PATH'])
        native_tools.plain_file(github_path)
        curl = Path(os.environ['SystemRoot']) / 'System32/curl.exe'
        native_tools.plain_file(curl)
        state['downloadTool'] = {'path': str(curl), **native_tools.identity(curl)}
        # New owned directory, never an inherited PATH tool or an old receipt.
        directory = Path(tempfile.mkdtemp(prefix='skager-wget-', dir=runner_temp))
        if any(character in str(directory) for character in '\r\n'):
            raise RuntimeError('Wget tool directory cannot be published as one PATH entry')
        state['directory'] = str(directory)
        state['phase'] = 'download'
        with tempfile.TemporaryDirectory(prefix='skager-wget-download-', dir=runner_temp) as temporary:
            archive = Path(temporary) / 'wget.zip'
            result, _ = native_tools.native([
                curl, '--fail', '--location', '--silent', '--show-error',
                '--proto', '=https', '--proto-redir', '=https',
                '--connect-timeout', '20', '--max-time', '120',
                '--max-filesize', str(ARCHIVE['bytes']),
                '--output', archive, URL], 150, evidence / 'download')
            state['download'] = result
            if result['exitCode'] != 0:
                raise RuntimeError('Pinned Wget download failed')
            state['phase'] = 'extraction'
            state['files'] = extract(archive, directory)
        state['phase'] = 'native-verification'
        executable = directory / 'wget.exe'
        result, output = native_tools.native([executable, '--version'], 30, evidence / 'wget-version')
        state['probe'] = result
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        line = lines[0] if lines else ''
        if result['exitCode'] != 0 or not re.fullmatch(
                r'GNU Wget ' + re.escape(VERSION) + r' built on mingw(?:32|64)\.', line):
            raise RuntimeError('Pinned Wget native version probe failed: ' + line)
        verify_file(executable, EXECUTABLE)
        state['versionLine'] = line
        state['phase'] = 'publish-path'
        native_tools.plain_file(github_path)
        # This command file affects subsequent steps only. No machine PATH edit
        # and no MSYS usr/bin or replacement shell/compiler is introduced.
        with github_path.open('a', encoding='utf-8', newline='\n') as output_file:
            output_file.write(str(directory) + '\n')
        state.update(status='passed', phase='complete', pathPublished=True)
        return state
    except Exception as error:
        state['error'] = str(error)
        raise
    finally:
        (evidence / 'receipt.json').write_text(json.dumps(state, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.evidence), indent=2))


if __name__ == '__main__':
    main()
