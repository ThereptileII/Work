#!/usr/bin/env python3
"""Supply a pinned native wget on disposable Windows CI, without Chocolatey.

This consumer-only tool does not change win_deps.bat or authenticated SDK inputs.
Vendor files and GPL documentation stay in RUNNER_TEMP; only the verified EXE
is copied exclusively into the existing CI Chocolatey bin directory. PATH and
SDK facts stay unchanged. Native stock provisioning is a separate required check.
"""
import argparse
import hashlib
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
CHOCOLATEY_BIN = Path('C:/ProgramData/chocolatey/bin')


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
    owned_target = False
    target = CHOCOLATEY_BIN / 'wget.exe'
    try:
        inherited_path = os.environ['PATH']
        state['inheritedPathSha256'] = hashlib.sha256(inherited_path.encode('utf-8')).hexdigest()
        plain_directory(CHOCOLATEY_BIN)
        native_tools.plain_file(CHOCOLATEY_BIN / 'choco.exe')
        if not any(Path(entry.strip('"')).resolve() == CHOCOLATEY_BIN.resolve()
                   for entry in inherited_path.split(';') if entry):
            raise RuntimeError('Existing CI Chocolatey bin is not on inherited PATH')
        if os.path.lexists(target):
            raise RuntimeError('Refusing to overwrite an existing Wget target')
        runner_temp = Path(os.environ['RUNNER_TEMP'])
        plain_directory(runner_temp)
        curl = Path(os.environ['SystemRoot']) / 'System32/curl.exe'
        where = Path(os.environ['SystemRoot']) / 'System32/where.exe'
        native_tools.plain_file(curl)
        native_tools.plain_file(where)
        state['downloadTool'] = {'path': str(curl), **native_tools.identity(curl)}
        # New owned directory, never an inherited PATH tool or an old receipt.
        directory = Path(tempfile.mkdtemp(prefix='skager-wget-', dir=runner_temp))
        state['directory'] = str(directory)
        state['phase'] = 'download'
        with tempfile.TemporaryDirectory(prefix='skager-wget-download-', dir=runner_temp) as temporary:
            archive = Path(temporary) / 'wget.zip'
            result, _ = native_tools.native([
                curl, '--fail', '--location', '--silent', '--show-error',
                '--proto', '=https', '--proto-redir', '=https',
                # The vendor host occasionally closes a runner connection with
                # HTTP 499.  Retry the complete download without weakening the
                # byte/hash gate; a successful attempt is still verified below.
                '--retry', '4', '--retry-all-errors', '--retry-delay', '3',
                '--retry-max-time', '300', '--connect-timeout', '20', '--max-time', '300',
                '--max-filesize', str(ARCHIVE['bytes']),
                '--output', archive, URL], 360, evidence / 'download')
            state['download'] = result
            if result['exitCode'] != 0:
                raise RuntimeError('Pinned Wget download failed')
            state['phase'] = 'extraction'
            state['files'] = extract(archive, directory)
        state['phase'] = 'exclusive-copy'
        # This CI-host directory already participates in the frozen producer's
        # PATH. Never create it, overwrite a tool, or publish a new PATH entry.
        with target.open('xb') as output_file:
            owned_target = True
            output_file.write((directory / 'wget.exe').read_bytes())
        verify_file(target, EXECUTABLE)
        state['publishedExecutable'] = {'path': str(target), **EXECUTABLE}
        state['phase'] = 'native-verification'
        result, output = native_tools.native([target, '--version'], 30, evidence / 'wget-version')
        state['probe'] = result
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        line = lines[0] if lines else ''
        if result['exitCode'] != 0 or not re.fullmatch(
                r'GNU Wget ' + re.escape(VERSION) + r' built on mingw(?:32|64)\.', line):
            raise RuntimeError('Pinned Wget native version probe failed: ' + line)
        verify_file(target, EXECUTABLE)
        state['versionLine'] = line
        state['phase'] = 'command-resolution'
        result, output = native_tools.native([where, 'wget'], 30, evidence / 'wget-resolution')
        state['resolution'] = result
        matches = [line.strip() for line in output.splitlines() if line.strip()]
        if result['exitCode'] != 0 or not matches or Path(matches[0]).resolve() != target.resolve():
            raise RuntimeError('Native Wget command resolution does not select the verified EXE')
        verify_file(target, EXECUTABLE)
        if os.environ['PATH'] != inherited_path:
            raise RuntimeError('Inherited PATH changed during Wget preparation')
        state['resolvedExecutable'] = matches[0]
        state.update(status='passed', phase='complete', pathUnchanged=True, executablePublished=True)
        return state
    except Exception as error:
        state['error'] = str(error)
        if owned_target:
            # Remove only our still-identical failed copy, never changed bytes
            # or an existing tool. The private vendor files remain diagnostic.
            try:
                verify_file(target, EXECUTABLE)
                target.unlink()
                state['failedCopyRemoved'] = True
            except (OSError, RuntimeError) as cleanup_error:
                state['failedCopyRemovalError'] = str(cleanup_error)
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
