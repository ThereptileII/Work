"""Verify a same-revision broker prerequisite; never provision or activate it.

Successful authenticated remote producer provenance remains the caller's duty.
Hashes, PE metadata and corresponding-source records bind copied bytes; they do
not prove publisher authenticity, reproducibility or native service qualification.
"""
import hashlib
import json
from pathlib import PurePosixPath
import re
import stat
import struct
import zipfile

import updater_package as updater

BINARY = 'skager-update-broker.exe'
RECORD_RELATIVE = 'opennav/third-party/update-broker/build.json'
BUILD_RECIPE = ('CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
                '-mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-update-broker')
MAX_BUILD_INFO = 65536
MAX_SOURCE_RECORD = 16 << 20
MAX_SOURCE_FILES = 50000
MAX_SOURCE_MEMBER = 32 << 20
MAX_SOURCE_TOTAL = 512 << 20
PREFIX = 'project/tools/update-verifier/'
REQUIRED_SOURCES = {PREFIX + name for name in ('cmd/skager-update-broker/main.go',
    'brokerhost/host_windows.go', 'brokerhost/operation.go', 'brokerhost/registration.go',
    'brokerhost/stock_windows.go')} | {'project/tools/package-update-broker.py',
                                     'project/tools/broker_package.py'}
MAGIC = b'\xff Go buildinf:'
START = bytes.fromhex('3077af0c9274080241e1c107e6d618e6')
END = bytes.fromhex('f932433186182072008242104116d8f2')


def _regular(path, limit):
    path = updater._file(path, limit)
    if path.stat().st_nlink != 1:
        raise ValueError('Broker package requires a single-link regular file')
    return path


def _inline_string(data, offset, limit):
    length = 0
    for i in range(10):
        if offset + i >= len(data):
            break
        byte = data[offset + i]
        length |= (byte & 127) << (7 * i)
        if byte < 128:
            start = offset + i + 1
            if length == 0 or length > limit or start + length > len(data):
                break
            return data[start:start + length], start + length
    raise ValueError('Unsupported or oversized inline Go build string')


def _embedded_info(binary):
    """Pinned Go >=1.18 inline format, scoped to PE's mapped writable data.

    Section selection follows pinned debug/buildinfo.peExe.DataStart; no whole
    binary scan, decorative overlay marker, pointer format or ambiguity accepted.
    """
    with binary.open('rb') as source:
        dos = source.read(64)
        if len(dos) != 64 or dos[:2] != b'MZ':
            raise ValueError('Broker must be a Win32 PE32 executable')
        pe_offset = struct.unpack_from('<I', dos, 60)[0]
        source.seek(pe_offset)
        header = source.read(24)
        if len(header) != 24 or header[:4] != b'PE\0\0':
            raise ValueError('Invalid broker PE header')
        machine, count = struct.unpack_from('<HH', header, 4)
        optional_size = struct.unpack_from('<H', header, 20)[0]
        characteristics = struct.unpack_from('<H', header, 22)[0]
        optional = source.read(optional_size)
        if (machine != 0x14c or not 1 <= count <= 96 or optional_size < 70 or
                len(optional) != optional_size or struct.unpack_from('<H', optional)[0] != 0x10b or
                struct.unpack_from('<H', optional, 68)[0] != 2 or
                not characteristics & 2 or characteristics & 0x2000):
            raise ValueError('Broker must use Windows GUI Win32 x86 PE32 executable format')
        table = source.read(count * 40)
        if len(table) != count * 40:
            raise ValueError('Truncated broker PE section table')
        size = binary.stat().st_size
        selected = None
        for i in range(count):
            virtual_size, address, raw_size, offset = struct.unpack_from('<IIII', table, i * 40 + 8)
            flags = struct.unpack_from('<I', table, i * 40 + 36)[0]
            if offset + raw_size > size:
                raise ValueError('Broker PE section exceeds binary')
            if selected is None and address and raw_size and flags & ~0x600000 == 0xc0000040:
                selected = (offset, min(raw_size, virtual_size))
        if selected is None or not 32 <= selected[1] <= updater.MAX_BINARY_BYTES:
            raise ValueError('Broker mapped Go build-info section missing')
        source.seek(selected[0])
        data = source.read(selected[1])
    positions = [i for i in range(0, len(data) - 31, 16) if data[i:i + 14] == MAGIC]
    if len(positions) != 1:
        raise ValueError('Broker Go build-info missing or ambiguous')
    offset = positions[0]
    if data[offset + 14:offset + 16] != b'\x04\x02' or any(data[offset + 16:offset + 32]):
        raise ValueError('Unsupported broker Go build-info format')
    version, next_offset = _inline_string(data, offset + 32, 64)
    framed, _ = _inline_string(data, next_offset, MAX_BUILD_INFO)
    if not framed.startswith(START) or not framed.endswith(END):
        raise ValueError('Broker Go module framing invalid')
    return version.decode('utf-8'), framed[16:-16].decode('utf-8')


def _build_info(binary, declared, commit, reference):
    if not isinstance(declared, str) or len(declared.encode('utf-8')) > MAX_BUILD_INFO:
        raise ValueError('Broker buildInfo missing or oversized')
    version, module = _embedded_info(binary)
    if version != updater.GO_VERSION or not module.endswith('\n'):
        raise ValueError('Broker actual Go compiler identity mismatch')
    lines = module.splitlines()
    expected = BINARY + ': ' + version + '\n' + ''.join('\t' + line + '\n' for line in lines)
    # Windows subprocess text mode normalizes CRLF before writing the record.
    if declared != expected:
        raise ValueError('Broker buildInfo differs from actual embedded executable metadata')
    paths, mains, dependencies, settings = [], [], set(), {}
    locked = {m['path']: (m['version'], m['sum']) for m in reference['modules']}
    for line in lines:
        fields = line.split('\t')
        if fields[0] == 'path' and len(fields) == 2:
            paths.append(fields[1])
        elif fields[0] == 'mod' and len(fields) == 4:
            mains.append(fields[1:])
        elif fields[0] == 'dep' and len(fields) == 4:
            name, version, checksum = fields[1:]
            if name in dependencies or locked.get(name) != (version, checksum):
                raise ValueError('Broker dependency differs from locked corresponding source')
            dependencies.add(name)
        elif fields[0] == 'build' and len(fields) == 2 and '=' in fields[1]:
            key, value = fields[1].split('=', 1)
            if key in settings:
                raise ValueError('Duplicate broker Go build setting')
            settings[key] = value
        else:
            raise ValueError('Unsupported broker Go module metadata')
    required = {'-buildmode': 'exe', '-compiler': 'gc', '-trimpath': 'true',
                '-ldflags': '-H=windowsgui', 'CGO_ENABLED': '0', 'GOARCH': '386',
                'GOOS': 'windows', 'GO386': 'sse2', 'vcs': 'git',
                'vcs.revision': commit, 'vcs.modified': 'false'}
    if (paths != [updater.MAIN_MODULE + '/cmd/skager-update-broker'] or
            mains != [[updater.MAIN_MODULE, '(devel)', '']] or
            any(settings.get(k) != v for k, v in required.items()) or
            not re.fullmatch(r'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ', settings.get('vcs.time', ''))):
        raise ValueError('Broker actual Go build settings do not match pinned product recipe')


def _source_contents(archive, reference):
    with zipfile.ZipFile(archive) as source:
        entries = source.infolist()
        if not 1 <= len(entries) <= MAX_SOURCE_FILES + 1:
            raise ValueError('Broker source inventory exceeds file bound')
        names, folded, total = {}, set(), 0
        for entry in entries:
            name = entry.filename
            path = PurePosixPath(name)
            mode = entry.external_attr >> 16
            if (entry.orig_filename != name or path.as_posix() != name or path.is_absolute() or '..' in path.parts or
                    '\\' in name or ':' in name or any(ord(c) < 32 for c in name) or
                    name.casefold() in folded or entry.is_dir() or entry.flag_bits & 1 or
                    stat.S_IFMT(mode) not in (0, stat.S_IFREG) or
                    entry.file_size > MAX_SOURCE_MEMBER):
                raise ValueError('Unsafe, linked or duplicate broker source entry')
            names[name] = entry; folded.add(name.casefold()); total += entry.file_size
        if total > MAX_SOURCE_TOTAL or 'SOURCE_REFERENCE.json' not in names:
            raise ValueError('Broker source reference missing or source budget exceeded')
        if names['SOURCE_REFERENCE.json'].file_size > MAX_SOURCE_RECORD:
            raise ValueError('Broker source reference oversized')
        actual = json.loads(source.read('SOURCE_REFERENCE.json').decode('utf-8'), object_pairs_hook=updater._unique)
        updater._keys(actual, (*reference, 'files'))
        archived_reference = {k: actual[k] for k in reference}
        updater._reference(archived_reference, reference['productCommit'])
        if archived_reference != reference:
            raise ValueError('Archived broker source reference differs from updater descriptor')
        files = actual['files']
        if not isinstance(files, dict) or set(files) != set(names) - {'SOURCE_REFERENCE.json'}:
            raise ValueError('Broker source inventory does not match archive entries')
        for name, item in files.items():
            updater._keys(item, ('bytes', 'sha256'))
            if type(item['bytes']) is not int or item['bytes'] != names[name].file_size:
                raise ValueError('Broker source inventory size mismatch')
            updater._digest(item['sha256'])
        if not REQUIRED_SOURCES <= set(files):
            raise ValueError('Actual tracked broker main/host sources missing from archive')
        checked = REQUIRED_SOURCES | {n for n in files if n.startswith(PREFIX + 'brokerhost/') and n.endswith('.go')}
        for name in checked:
            data = source.read(name)  # Bounded above; ZipFile also verifies CRC.
            if not data or hashlib.sha256(data).hexdigest() != files[name]['sha256']:
                raise ValueError('Archived broker source bytes differ from inventory')


def verify_broker_package(app, commit):
    """Return the unchanged source-package descriptor for exact broker bytes.

    This is opt-in packaging validation, not installation/SCM/trust authority.
    The caller authenticates successful same-revision remote production first.
    """
    descriptor = updater.verify_updater_package(app, commit)
    app = updater._plain(app)
    record_path = _regular(app / RECORD_RELATIVE, updater.MAX_RECORD_BYTES)
    record = json.loads(record_path.read_text(encoding='utf-8'), object_pairs_hook=updater._unique)
    updater._keys(record, ('schema', 'productCommit', 'goVersion', 'binary', 'sourceBundle', 'buildInfo', 'build'))
    if (type(record['schema']) is not int or record['schema'] != 1 or record['productCommit'] != commit or
            record['goVersion'] != updater.GO_VERSION or record['build'] != BUILD_RECIPE):
        raise ValueError('Broker package commit/compiler/build recipe mismatch')
    updater._keys(record['binary'], ('path', 'sha256'))
    updater._keys(record['sourceBundle'], ('archive', 'path', 'sha256', 'reference'))
    updater._reference(record['sourceBundle']['reference'], commit)
    expected_bundle = {**descriptor, 'archive': updater.SOURCE_RELATIVE}
    if record['binary']['path'] != BINARY or record['sourceBundle'] != expected_bundle:
        raise ValueError('Broker requires fixed binary and exact updater source descriptor')
    binary = _regular(app / BINARY, updater.MAX_BINARY_BYTES)
    archive = _regular(descriptor['archive'], updater.MAX_ARCHIVE_BYTES)
    if updater._hash(binary) != updater._digest(record['binary']['sha256']):
        raise ValueError('Broker executable changed after production')
    _build_info(binary, record['buildInfo'], commit, descriptor['reference'])
    _source_contents(archive, descriptor['reference'])
    return descriptor
