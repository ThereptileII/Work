"""Verify the copied launcher/source pair before installed or recovery packaging."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import struct
import tempfile

GO_VERSION = 'go1.27.1'
MAIN_MODULE = 'example.com/opennav-update-verifier'
SOURCE_RELATIVE = 'opennav/third-party/updater/updater-source.zip'
BUNDLE_PATH = 'third-party-sources/skager-updater-source.zip'
MAX_RECORD_BYTES = 1 << 20
MAX_BINARY_BYTES = 128 << 20
MAX_ARCHIVE_BYTES = 256 << 20
SHA256 = re.compile(r'[a-f0-9]{64}\Z')
COMMIT = re.compile(r'[a-f0-9]{40}\Z')


def _plain(path, *, allow_missing=False):
    path = Path(path).absolute()
    for component in (path, *path.parents):
        try:
            entry = component.lstat()
        except FileNotFoundError:
            if allow_missing:
                continue
            raise
        if stat.S_ISLNK(entry.st_mode) or getattr(entry, 'st_file_attributes', 0) & 0x400:
            raise ValueError('Updater package contains a link or reparse point')
    return path


def _file(path, limit):
    path = _plain(path)
    entry = path.stat()
    if not stat.S_ISREG(entry.st_mode) or entry.st_size <= 0 or entry.st_size > limit:
        raise ValueError('Updater package file has invalid type or size')
    return path


def _keys(record, names):
    if not isinstance(record, dict) or set(record) != set(names):
        raise ValueError('Unexpected updater package schema')


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate updater package JSON field')
        result[key] = value
    return result


def _hash(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def _digest(value):
    if not isinstance(value, str) or not SHA256.fullmatch(value):
        raise ValueError('Invalid updater package SHA-256 declaration')
    return value


def _sum(value):
    if not isinstance(value, str) or not value.startswith('h1:'):
        raise ValueError('Missing verified Go module checksum')
    try:
        if len(base64.b64decode(value[3:], validate=True)) != 32:
            raise ValueError('Invalid Go module checksum size')
    except (ValueError, TypeError) as error:
        raise ValueError('Invalid Go module checksum') from error


def _reference(reference, commit):
    _keys(reference, ('schema', 'productCommit', 'goVersion', 'mainModule', 'modules', 'build'))
    if (type(reference['schema']) is not int or reference['schema'] != 1 or
            reference['productCommit'] != commit or reference['goVersion'] != GO_VERSION or
            reference['mainModule'] != 'tools/update-verifier'):
        raise ValueError('Updater source identity does not match the product')
    recipe = ('CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
              '-mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start')
    if reference['build'] != recipe:
        raise ValueError('Updater source build recipe changed')
    modules = reference['modules']
    if not isinstance(modules, list) or not 1 <= len(modules) <= 256:
        raise ValueError('Updater module source inventory is unavailable or oversized')
    seen = set()
    for module in modules:
        _keys(module, ('path', 'version', 'sum', 'goModSum', 'sourcePrefix'))
        path, version = module['path'], module['version']
        if (not isinstance(path, str) or len(path) > 512 or
                not re.fullmatch(r'[A-Za-z0-9._~!+-]+(?:/[A-Za-z0-9._~!+-]+)*', path) or
                any(part in ('.', '..') for part in PurePosixPath(path).parts) or path in seen or
                not isinstance(version, str) or len(version) > 128 or not re.fullmatch(r'v[0-9][A-Za-z0-9.+-]*', version) or
                module['sourcePrefix'] != 'modules/' + path + '@' + version):
            raise ValueError('Invalid or duplicate updater module source declaration')
        seen.add(path)
        _sum(module['sum'])
        _sum(module['goModSum'])


def _build_info(info, commit):
    if not isinstance(info, str) or len(info) > 65536:
        raise ValueError('Updater compiler build identity is unavailable')
    lines = info.splitlines()
    if not lines or lines[0] != 'skager-start.exe: ' + GO_VERSION:
        raise ValueError('Updater compiler build version mismatch')
    paths, settings = [], {}
    for line in lines[1:]:
        fields = line.strip().split('\t')
        if fields[0] == 'path':
            if len(fields) != 2:
                raise ValueError('Malformed Go build module path')
            paths.append(fields[1])
        if fields[0] == 'build':
            if len(fields) != 2 or '=' not in fields[1]:
                raise ValueError('Malformed Go build setting')
            key, value = fields[1].split('=', 1)
            if key in settings:
                raise ValueError('Duplicate Go build setting')
            settings[key] = value
    required = {'CGO_ENABLED': '0', 'GOARCH': '386', 'GOOS': 'windows', '-trimpath': 'true',
                'vcs.revision': commit, 'vcs.modified': 'false'}
    if paths != [MAIN_MODULE + '/cmd/skager-start'] or any(settings.get(key) != value for key, value in required.items()):
        raise ValueError('Updater compiler build settings do not match the product')


def _pe32(path):
    with path.open('rb') as source:
        header = source.read(64)
        if len(header) != 64 or header[:2] != b'MZ':
            raise ValueError('Updater launcher is not a PE binary')
        offset = struct.unpack_from('<I', header, 60)[0]
        if offset > path.stat().st_size - 26:
            raise ValueError('Updater PE header is outside the binary')
        source.seek(offset)
        pe = source.read(26)
    if (len(pe) != 26 or pe[:4] != b'PE\0\0' or struct.unpack_from('<H', pe, 4)[0] != 0x14c or
            struct.unpack_from('<H', pe, 24)[0] != 0x10b):
        raise ValueError('Updater launcher must use Win32 x86 PE32')


def verify_updater_package(app, commit):
    """Return source_package's descriptor for the exact copied package bytes.

    Trust provisioning is a separate authorized operation. The default product
    must not accidentally ship the development repository's update-trust.json.
    Source contents were already checked by the producer; here the complete ZIP
    hash binds those contents without expanding every dependency again.
    """
    if not isinstance(commit, str) or not COMMIT.fullmatch(commit):
        raise ValueError('Exact product commit required for updater verification')
    app = _plain(app)
    if not app.is_dir():
        raise ValueError('Updater application directory is missing')
    try:
        (app / 'update-trust.json').lstat()
    except FileNotFoundError:
        pass
    else:
        raise ValueError('Default product must not bundle update trust configuration')
    record_path = _file(app / 'opennav/third-party/updater/build.json', MAX_RECORD_BYTES)
    record = json.loads(record_path.read_text(encoding='utf-8'), object_pairs_hook=_unique)
    _keys(record, ('schema', 'productCommit', 'goVersion', 'binary', 'sourceBundle', 'buildInfo'))
    if (type(record['schema']) is not int or record['schema'] != 1 or
            record['productCommit'] != commit or record['goVersion'] != GO_VERSION):
        raise ValueError('Updater package commit or locked compiler identity mismatch')
    binary_record, bundle = record['binary'], record['sourceBundle']
    _keys(binary_record, ('path', 'sha256'))
    _keys(bundle, ('archive', 'path', 'sha256', 'reference'))
    if binary_record['path'] != 'skager-start.exe' or bundle['archive'] != SOURCE_RELATIVE or bundle['path'] != BUNDLE_PATH:
        raise ValueError('Updater package requires the fixed relative binary/source paths')
    _reference(bundle['reference'], commit)
    _build_info(record['buildInfo'], commit)
    binary = _file(app / 'skager-start.exe', MAX_BINARY_BYTES)
    archive = _file(app / SOURCE_RELATIVE, MAX_ARCHIVE_BYTES)
    if _hash(binary) != _digest(binary_record['sha256']) or _hash(archive) != _digest(bundle['sha256']):
        raise ValueError('Updater binary or corresponding source changed after production')
    _pe32(binary)
    return {'archive': archive, 'path': BUNDLE_PATH, 'sha256': bundle['sha256'], 'reference': bundle['reference']}


TRANSFER_FILES = {
    'skager-start.exe': MAX_BINARY_BYTES,
    SOURCE_RELATIVE: MAX_ARCHIVE_BYTES,
    'opennav/third-party/updater/build.json': MAX_RECORD_BYTES,
}


def copy_verified_updater_package(source_install, install, commit):
    """Transfer only a verified same-commit producer closure, without rebuilding.

    The caller must authenticate the artifact's successful same-run producer.
    These local checks bind its exact bytes/commit, not its remote provenance.
    Partial publication is preserved and cannot be silently overwritten.
    """
    source_install = _plain(source_install)
    verify_updater_package(source_install, commit)
    allowed = set(TRANSFER_FILES)
    directories = {str(parent).replace('\\', '/') for name in allowed
                   for parent in Path(name).parents if str(parent) != '.'}
    for current, children, files in os.walk(source_install, followlinks=False):
        for name in children + files:
            child = _plain(Path(current) / name)
            relative = child.relative_to(source_install).as_posix()
            if relative not in allowed | directories:
                raise ValueError('Unexpected updater producer install entry: ' + repr(relative))
    identities = {name: (_file(source_install / name, limit).stat().st_size,
                         _hash(source_install / name)) for name, limit in TRANSFER_FILES.items()}
    install = _plain(install)
    if not install.is_dir():
        raise ValueError('Existing product install directory required')
    for name in ('skager-start.exe', 'opennav/third-party/updater', 'update-trust.json'):
        destination = _plain(install / name, allow_missing=True)
        if destination.exists():
            raise ValueError('Refusing to overwrite updater outputs or bundle trust configuration')
    with tempfile.TemporaryDirectory(prefix='.updater-transfer-', dir=install) as temporary:
        staging = Path(temporary)
        for name, (size, checksum) in identities.items():
            destination = staging / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            remaining = size
            with _file(source_install / name, TRANSFER_FILES[name]).open('rb') as source, destination.open('xb') as target:
                while remaining:
                    data = source.read(min(1 << 20, remaining))
                    if not data:
                        raise ValueError('Updater producer file truncated during transfer')
                    target.write(data)
                    remaining -= len(data)
                if source.read(1):
                    raise ValueError('Updater producer file grew during transfer')
                target.flush()
                os.fsync(target.fileno())
            if _hash(destination) != checksum:
                raise ValueError('Updater producer bytes changed during transfer')
        verify_updater_package(staging, commit)
        # Publish the source closure first; a failure keeps evidence and blocks
        # retries, rather than accepting an incomplete launcher/source pair.
        parent = _plain(install / 'opennav/third-party', allow_missing=True)
        parent.mkdir(parents=True, exist_ok=True)
        (parent / 'updater').mkdir()
        # Hard links publish the verified bytes exclusively on this volume;
        # unlike POSIX rename they cannot replace racing existing files.
        for name in (SOURCE_RELATIVE, 'opennav/third-party/updater/build.json', 'skager-start.exe'):
            os.link(staging / name, install / name)
    return verify_updater_package(install, commit)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-install', type=Path, required=True)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--commit', required=True)
    args = parser.parse_args()
    try:
        result = copy_verified_updater_package(args.source_install, args.install, args.commit)
    except (ValueError, OSError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(result, default=str, indent=2))
