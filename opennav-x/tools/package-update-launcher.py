#!/usr/bin/env python3
"""Native Windows launcher + exact corresponding source; never creates trust config."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import struct
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent.parent
GO_VERSION = 'go1.27.1'
MAX_FILES = 50000
MAX_SOURCE_BYTES = 512 << 20
MAX_FILE_BYTES = 32 << 20
MAX_ARCHIVE_BYTES = 256 << 20
MAX_MODULES = 256
MAX_COMMAND_BYTES = 8 << 20
MAIN_MODULE = 'example.com/opennav-update-verifier'


def is_native_windows():
    return os.name == 'nt'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def plain_path(path, boundary='Source/output path'):
    path = Path(os.path.abspath(path))
    for parent in (path, *path.parents):
        try:
            entry = parent.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(entry.st_mode) or getattr(entry, 'st_file_attributes', 0) & 0x400:
            # Filesystem components are diagnostic data, not command output or
            # environment dumps. repr keeps control characters out of the log.
            raise ValueError(f'{boundary} contains a link or reparse point: component={str(parent)!r}')
    return path


def provisioned_source_root(path, role, *, allow_missing=False):
    """Resolve only build-provisioned compiler/runtime/cache roots, then recheck.

    setup-go deliberately exposes its Windows tool cache through a junction
    (C:\\hostedtoolcache\\... -> D:\\hostedtoolcache\\...). This allowance never
    applies to checkout inputs, inventory children, package paths or outputs.
    """
    if role not in ('Go compiler', 'GOROOT', 'GOMODCACHE'):
        raise ValueError('Unrecognized provisioned source boundary')
    selected = Path(os.path.abspath(path))
    resolved = selected.resolve(strict=not allow_missing)
    resolved = plain_path(resolved, boundary=f'Canonical provisioned {role}')
    if role == 'Go compiler':
        if not resolved.is_file():
            raise ValueError('Provisioned Go compiler is not a regular file')
    elif resolved.exists() and not resolved.is_dir():
        raise ValueError(f'Provisioned {role} is not a directory')
    return resolved


def module_cache_source(path, cache):
    """Compiler-reported module inputs must stay below the pinned cache root."""
    selected = Path(path)
    if not selected.is_absolute() or '..' in selected.parts:
        raise ValueError('Verified module source requires an absolute path without traversal')
    selected = plain_path(selected, boundary='Verified module-cache source')
    try:
        relative = selected.relative_to(cache)
    except ValueError as error:
        raise ValueError('Verified module source is outside the provisioned GOMODCACHE') from error
    if not relative.parts:
        raise ValueError('Verified module source cannot be the whole module cache')
    return selected


def run(command, directory, environment=None):
    # Bound both runtime and retained output without an unbounded PIPE allocation.
    with tempfile.TemporaryFile() as output:
        process = subprocess.run(command, cwd=directory, env=environment, stdout=output,
                                 stderr=subprocess.STDOUT, timeout=300, check=False)
        size = output.tell()
        if size > MAX_COMMAND_BYTES:
            raise ValueError('Packaging command output exceeds limit')
        output.seek(0)
        data = output.read().decode('utf-8', errors='strict')
    if process.returncode:
        raise ValueError('Packaging command failed: ' + data[-4000:])
    return data


def json_stream(text):
    decoder = json.JSONDecoder()
    result = []
    while text.strip():
        value, end = decoder.raw_decode(text.lstrip())
        if not isinstance(value, dict) or len(result) >= MAX_MODULES:
            raise ValueError('Invalid or oversized Go module graph')
        result.append(value)
        text = text.lstrip()[end:]
    return result


def archive_name(name):
    portable = PurePosixPath(name)
    if (portable.as_posix() != name or portable.is_absolute() or '..' in portable.parts
            or '\\' in name or ':' in name or any(ord(c) < 32 for c in name)):
        raise ValueError('Unsafe corresponding-source archive path')
    return name


class Inventory:
    def __init__(self):
        self.files = {}
        self.total = 0

    def add(self, source, name):
        source = plain_path(source, boundary='Corresponding-source file')
        name = archive_name(name)
        info = source.stat()
        if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_FILE_BYTES:
            raise ValueError('Nonregular or oversized corresponding-source file')
        if name in self.files or len(self.files) >= MAX_FILES or self.total + info.st_size > MAX_SOURCE_BYTES:
            raise ValueError('Duplicate source name or corresponding-source budget exceeded')
        self.files[name] = {'source': source, 'bytes': info.st_size, 'sha256': digest(source)}
        self.total += info.st_size

    def tree(self, directory, prefix):
        directory = plain_path(directory, boundary='Corresponding-source tree')
        if not directory.is_dir():
            raise ValueError('Corresponding-source directory missing')
        for current, directories, files in os.walk(directory, followlinks=False):
            directories.sort()
            for child in directories:
                plain_path(Path(current) / child, boundary='Corresponding-source child directory')
            for child in sorted(files):
                source = Path(current) / child
                self.add(source, prefix + '/' + source.relative_to(directory).as_posix())

    def write(self, archive, reference):
        if archive.exists():
            raise ValueError('Refusing to overwrite corresponding source')
        reference = dict(reference)
        reference['files'] = {name: {'bytes': item['bytes'], 'sha256': item['sha256']}
                              for name, item in sorted(self.files.items())}
        with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=6) as target:
            for name, item in sorted(self.files.items()):
                # Detect source/cache mutation between inventory and packaging.
                source = plain_path(item['source'], boundary='Inventoried corresponding-source file')
                if source.stat().st_size != item['bytes'] or digest(source) != item['sha256']:
                    raise ValueError('Corresponding source changed before archiving')
                info = zipfile.ZipInfo(name)
                info.compress_type = zipfile.ZIP_DEFLATED
                info.create_system = 3
                info.external_attr = (stat.S_IFREG | 0o644) << 16
                with source.open('rb') as content, target.open(info, 'w') as out:
                    shutil.copyfileobj(content, out, 1 << 20)
                if archive.stat().st_size > MAX_ARCHIVE_BYTES:
                    raise ValueError('Compressed corresponding-source budget exceeded')
            target.writestr('SOURCE_REFERENCE.json', json.dumps(reference, sort_keys=True, indent=2) + '\n')
        if archive.stat().st_size > MAX_ARCHIVE_BYTES:
            raise ValueError('Compressed corresponding-source budget exceeded')
        with zipfile.ZipFile(archive) as check:
            if check.testzip() is not None:
                raise ValueError('Corresponding-source CRC verification failed')
            for name, item in self.files.items():
                with check.open(name) as source:
                    if hashlib.file_digest(source, 'sha256').hexdigest() != item['sha256']:
                        raise ValueError('Archived corresponding-source digest mismatch')
        return digest(archive)


def validate_pe32(path):
    with path.open('rb') as binary:
        header = binary.read(64)
        if len(header) != 64 or header[:2] != b'MZ':
            raise ValueError('Launcher is not a Windows PE binary')
        offset = struct.unpack_from('<I', header, 60)[0]
        if offset > path.stat().st_size - 26:
            raise ValueError('Invalid launcher PE header')
        binary.seek(offset)
        pe = binary.read(26)
    if pe[:4] != b'PE\0\0' or struct.unpack_from('<H', pe, 4)[0] != 0x14c or struct.unpack_from('<H', pe, 24)[0] != 0x10b:
        raise ValueError('Launcher must use native Windows x86 PE32')


def prepare_outputs(install):
    install = plain_path(install, boundary='Package install directory')
    if not install.is_dir():
        raise ValueError('Existing application install directory required')
    binary = plain_path(install / 'skager-start.exe', boundary='Package launcher output')
    sources = plain_path(install / 'opennav/third-party/updater', boundary='Package source-bundle output')
    if binary.exists() or sources.exists():
        raise ValueError('Refusing to overwrite existing updater outputs')
    return install, binary, sources


def package(install, commit, go):
    if not is_native_windows():
        raise ValueError('Launcher packaging requires native Windows')
    install, final_binary, final_sources = prepare_outputs(install)
    if not re.fullmatch(r'[a-f0-9]{40}', commit or ''):
        raise ValueError('Exact product commit is required (--commit or GITHUB_SHA)')
    if run(['git', 'rev-parse', 'HEAD'], ROOT).strip() != commit:
        raise ValueError('Launcher source is not the selected product commit')
    if run(['git', 'status', '--porcelain=v1', '--untracked-files=all', '--', '.'], ROOT).strip():
        raise ValueError('Packaging requires committed source and no untracked source inputs')
    module = ROOT / 'tools/update-verifier'
    compiler = provisioned_source_root(shutil.which(go) or go, 'Go compiler')
    go = str(compiler)
    environment = dict(os.environ, GOOS='windows', GOARCH='386', GO386='sse2', CGO_ENABLED='0',
                       GOTOOLCHAIN='local', GOWORK='off', GOFLAGS='-mod=readonly', GOEXPERIMENT='')
    tool = json.loads(run([go, 'env', '-json', 'GOVERSION', 'GOROOT', 'GOHOSTOS', 'GOMOD', 'GOMODCACHE'], module, environment))
    if tool['GOVERSION'] != GO_VERSION or tool['GOHOSTOS'] != 'windows' or Path(tool['GOMOD']).resolve() != (module / 'go.mod').resolve():
        raise ValueError('Pinned native Go 1.27.1 and exact launcher module are required')
    goroot = provisioned_source_root(tool['GOROOT'], 'GOROOT')
    if compiler != plain_path(goroot / 'bin/go.exe', boundary='Pinned GOROOT compiler'):
        raise ValueError('Selected compiler is outside the provisioned GOROOT/bin/go.exe')
    # A first build may populate an absent module cache. Resolve its existing
    # provisioned ancestors now, and require the canonical directory after the
    # bounded download. Pin both roots so later commands cannot reuse aliases.
    module_cache = provisioned_source_root(tool['GOMODCACHE'], 'GOMODCACHE', allow_missing=True)
    environment.update(GOROOT=str(goroot), GOMODCACHE=str(module_cache))
    if plain_path(goroot / 'VERSION', boundary='Provisioned Go VERSION').read_text(encoding='utf-8').splitlines()[0] != GO_VERSION:
        raise ValueError('Go source version and compiler disagree')
    tracked = run(['git', 'ls-files', '-z', '--', 'tools/update-verifier', 'tools/package-update-launcher.py', 'LICENSE'], ROOT).split('\0')
    tracked = [name for name in tracked if name]
    required = {'LICENSE', 'tools/package-update-launcher.py', 'tools/update-verifier/go.mod', 'tools/update-verifier/go.sum'}
    if not required.issubset(tracked):
        raise ValueError('Exact launcher source, dependency locks, build recipe and license must be committed')
    inventory = Inventory()
    for name in tracked:
        if Path(name).suffix.lower() in ('.exe', '.dll', '.pdb', '.test', '.log') or Path(name).name == 'verifier-tests.json':
            raise ValueError('Generated binaries/logs must not be tracked as launcher source')
        inventory.add(ROOT / name, 'project/' + name)
    with tempfile.TemporaryDirectory(prefix='.updater-build-', dir=install) as temporary:
        staging = Path(temporary)
        # Provision in a private copy: go mod download may add checksum entries,
        # but it must never rewrite the product checkout's committed go.sum.
        provision = staging / 'provisioning'
        provision.mkdir()
        for name in ('go.mod', 'go.sum'):
            shutil.copyfile(module / name, provision / name)
        downloads = json_stream(run([go, 'mod', 'download', '-json', 'all'], provision, environment))
        if provisioned_source_root(module_cache, 'GOMODCACHE') != module_cache:
            raise ValueError('Provisioned module-cache root changed during download')
        if (provision / 'go.mod').read_bytes() != (module / 'go.mod').read_bytes():
            raise ValueError('Source provisioning changed the pinned module graph')
        run([go, 'mod', 'verify'], provision, environment)
        run([go, 'mod', 'verify'], module, environment)
        modules = json_stream(run([go, 'list', '-mod=readonly', '-m', '-json', 'all'], module, environment))
        selected = {}
        for item in downloads:
            if item.get('Error') or not item.get('Sum') or not item.get('GoModSum'):
                raise ValueError('Dependency source checksum verification is incomplete')
            selected[(item['Path'], item['Version'])] = item
        references = []
        mains = 0
        for item in modules:
            if item.get('Replace') or item.get('Error'):
                raise ValueError('Replaced or unresolved module source refused')
            if item.get('Main'):
                mains += 1
                if item['Path'] != MAIN_MODULE or Path(item.get('Dir', '')).resolve() != module.resolve():
                    raise ValueError('Unexpected main module source')
                continue
            key = (item['Path'], item['Version'])
            source = selected.get(key)
            if source is None:
                raise ValueError('Every resolved module must have verified complete source')
            prefix = archive_name('modules/' + item['Path'] + '@' + item['Version'])
            inventory.tree(module_cache_source(source['Dir'], module_cache), prefix)
            licenses = [name for name in inventory.files if name.startswith(prefix + '/')
                        and PurePosixPath(name).name.upper().startswith(('LICENSE', 'COPYING'))]
            if not licenses:
                raise ValueError('Dependency source lacks its upstream license notice')
            # Module zip contents may omit go.mod for legacy repositories; the
            # authenticated module-file bytes are still supplied separately.
            inventory.add(module_cache_source(source['GoMod'], module_cache), 'module-locks/' + item['Path'] + '@' + item['Version'] + '.mod')
            references.append({'path': item['Path'], 'version': item['Version'], 'sum': source['Sum'],
                               'goModSum': source['GoModSum'], 'sourcePrefix': prefix})
        if mains != 1:
            raise ValueError('Exactly one pinned main module is required')
        inventory.add(provision / 'go.sum', 'provisioning/go.sum')
        inventory.tree(goroot / 'src', GO_VERSION + '/src')
        for name in ('LICENSE', 'VERSION', 'PATENTS', 'AUTHORS', 'CONTRIBUTORS'):
            if name in ('LICENSE', 'VERSION') or (goroot / name).is_file():
                inventory.add(goroot / name, GO_VERSION + '/' + name)
        source_directory = staging / 'updater'
        source_directory.mkdir()
        archive = source_directory / 'updater-source.zip'
        reference = {'schema': 1, 'productCommit': commit, 'goVersion': GO_VERSION,
                     'mainModule': 'tools/update-verifier', 'modules': sorted(references, key=lambda x: x['path']),
                     'build': 'CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build -mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start'}
        source_hash = inventory.write(archive, reference)
        # Full source and upstream notices are now available before compilation.
        binary = staging / 'skager-start.exe'
        run([go, 'build', '-mod=readonly', '-trimpath', '-buildvcs=true', '-ldflags=-H=windowsgui',
             '-o', str(binary), './cmd/skager-start'], module, environment)
        run([go, 'mod', 'verify'], module, environment)
        if run(['git', 'status', '--porcelain=v1', '--untracked-files=all', '--', '.'], ROOT).strip():
            raise ValueError('Product sources changed during launcher packaging')
        validate_pe32(binary)
        build_info = run([go, 'version', '-m', str(binary)], module, environment)
        for expected in (GO_VERSION, 'path\texample.com/opennav-update-verifier/cmd/skager-start',
                         'CGO_ENABLED=0', 'GOARCH=386', 'GOOS=windows', 'vcs.revision=' + commit, 'vcs.modified=false'):
            if expected not in build_info:
                raise ValueError('Compiled launcher does not match pinned source/build identity: ' + expected)
        build_info = 'skager-start.exe: ' + GO_VERSION + '\n' + '\n'.join(build_info.splitlines()[1:]) + '\n'
        record = {'schema': 1, 'productCommit': commit, 'goVersion': GO_VERSION,
                  'binary': {'path': 'skager-start.exe', 'sha256': digest(binary)},
                  'sourceBundle': {'archive': 'opennav/third-party/updater/updater-source.zip',
                                   'path': 'third-party-sources/skager-updater-source.zip',
                                   'sha256': source_hash, 'reference': reference},
                  'buildInfo': build_info}
        (source_directory / 'build.json').write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
        # Publish only verified outputs. A partial publication is preserved and
        # a subsequent run refuses it rather than overwriting package evidence.
        prepare_outputs(install)
        plain_path(final_sources.parent, boundary='Package source-bundle parent').mkdir(parents=True, exist_ok=True)
        source_directory.rename(final_sources)
        with binary.open('rb') as source, final_binary.open('xb') as target:
            shutil.copyfileobj(source, target, 1 << 20)
            target.flush()
            os.fsync(target.fileno())
        if digest(final_binary) != record['binary']['sha256']:
            raise ValueError('Published launcher digest changed')
        return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--commit', default=os.environ.get('GITHUB_SHA', ''))
    parser.add_argument('--go', default='go')
    args = parser.parse_args()
    try:
        result = package(args.install, args.commit, args.go)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
