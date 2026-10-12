"""Bind the unchanged NSIS package manifest into the final Recovery archive.

Data-only producer check, not signature/entitlement or native product acceptance.
Run after successful Setup creation in an otherwise exclusive packaging directory.
Fresh staged outputs are validated before publication; failed publication restores
prior outputs. Publication is not a cross-file crash-atomic filesystem transaction;
any failure leaves this build unqualified. No hostile concurrent namespace custody.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import tempfile
import zipfile

from hardware_output_policy import require_product_output_policy
from product_version import validate_product_version

# Keep output acceptance within tools/update-verifier/recovery.go: ASCII paths
# <=240, 12,000 members, 512 MiB/member, 2 GiB expansion, 4 MiB metadata;
# PRODUCT_BUILD <=32 KiB. Larger layouts require a coordinated reader change.
MAX_FILES = 12000
MAX_METADATA = 4 * 1024 * 1024
MAX_PRODUCT = 32 * 1024
MAX_FILE = 512 * 1024 * 1024
MAX_TOTAL = 2 * 1024 * 1024 * 1024
MAX_ARCHIVE = 4 * 1024 * 1024 * 1024
SIDECAR = 'installer/package.json'
INVENTORY = 'FILE_SHA256.json'
MARKER = 'app/OPENNAV_PORTABLE_PREVIEW'
PREFIX = 'SKAGER-Beta2-Portable-Recovery'
ENGINE_HELPERS = ('Lifecycle.ps1', 'UpdateTransaction.ps1', 'UpdateSupervisor.ps1')
MAX_ENGINE = 4 * 1024 * 1024
COMPAT_REQUIRED = {'version', 'arch', 'executableSha256', 'upstreamCommit'}
COMPAT_OPTIONAL = {'qualification', 'integrationPackage', 'qualificationCommit',
                   'qualificationRun', 'qualificationEvidence', 'stockSetupSha256'}


def _hex(value, length=64):
    if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{%d}' % length, value):
        raise ValueError('Canonical lowercase identity hash required')
    return value


def _path(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_ .()&@/+-]{1,240}', value):
        raise ValueError('Recovery reader requires an ASCII relative path of at most 240 characters')
    for part in value.split('/'):
        if (not part or len(part) > 255 or part in {'.', '..'} or part[-1] in ' .'
                or re.search(r'[\x00-\x1f\x7f<>:"\\|?*]', part)
                or re.match(r'^(?:CON|PRN|AUX|NUL|CONIN\$|CONOUT\$|COM[0-9¹²³]|LPT[0-9¹²³])(?: *\.|$)', part, re.I)):
            raise ValueError('Unsafe Windows package path: ' + value)
    return value


def _aliases(paths, directories=()):
    # Include ancestors: app/Foo/a and app/foo/b also collide on Windows.
    seen = {}
    files = set(paths) - set(directories)
    for name in paths:
        _path(name)
        parts = name.split('/')
        for n in range(1, len(parts) + 1):
            current = '/'.join(parts[:n])
            key = current.casefold()
            if key in seen and seen[key] != current:
                raise ValueError('Case-aliased package path')
            if n < len(parts) and current in files:
                raise ValueError('File/directory package collision')
            seen[key] = current


def _plain(path, directory=False):
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
        raise ValueError('Links/reparse points refused')
    if directory:
        if not stat.S_ISDIR(info.st_mode):
            raise ValueError('Plain directory required')
    elif not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise ValueError('Plain singly-linked file required')
    return info


def _local(path, directory=False):
    path = Path(os.path.abspath(path))
    for ancestor in reversed(path.parents):
        _plain(ancestor, True)
    _plain(path, directory)
    return path


def _signature(info):
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def _read(path, limit=MAX_FILE, collect=False, target=None):
    before = _plain(path)
    if before.st_size > limit:
        raise ValueError('Package input exceeds size bound')
    flags = os.O_RDONLY | getattr(os, 'O_BINARY', 0) | getattr(os, 'O_NOFOLLOW', 0)
    digest = hashlib.sha256()
    chunks = []
    with os.fdopen(os.open(path, flags), 'rb') as source:
        if _signature(os.fstat(source.fileno())) != _signature(before):
            raise ValueError('Package input changed before read')
        total = 0
        while chunk := source.read(1024 * 1024):
            total += len(chunk)
            if total > limit:
                raise ValueError('Package input grew beyond bound')
            digest.update(chunk)
            if collect:
                chunks.append(chunk)
            if target is not None:
                target.write(chunk)
        if total != before.st_size or _signature(os.fstat(source.fileno())) != _signature(before):
            raise ValueError('Package input changed during read')
    if _signature(_plain(path)) != _signature(before):
        raise ValueError('Package input replaced during read')
    return b''.join(chunks) if collect else digest.hexdigest()


def _json(raw):
    def pairs(items):
        result = {}
        seen = set()
        for key, value in items:
            if not key or len(key.encode('utf-8')) > 240 or key.casefold() in seen:
                raise ValueError('Duplicate/case-aliased JSON field')
            seen.add(key.casefold())
            result[key] = value
        return result
    if not raw or len(raw) > MAX_METADATA:
        raise ValueError('Bounded JSON required')
    value = json.loads(raw.decode('utf-8'), object_pairs_hook=pairs,
                       parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
    def depth(item, level=0):
        if level > 12:
            raise ValueError('Recovery reader JSON nesting bound')
        if isinstance(item, dict):
            for child in item.values():
                depth(child, level + 1)
        elif isinstance(item, list):
            for child in item:
                depth(child, level + 1)
    depth(value)
    return value


def _tree(root):
    result = {}
    names = []
    directories = []
    total = 0
    pending = [root]
    while pending:
        directory = pending.pop()
        _plain(directory, True)
        for child in sorted(directory.iterdir()):
            name = child.relative_to(root).as_posix()
            names.append(name)
            _path(name)
            info = child.lstat()
            if stat.S_ISDIR(info.st_mode):
                _plain(child, True)
                directories.append(name)
                pending.append(child)
            else:
                _plain(child)
                total += info.st_size
                if total > MAX_TOTAL:
                    raise ValueError('Recovery tree exceeds byte bound')
                result[name] = _read(child, MAX_METADATA if name == INVENTORY else MAX_FILE)
            if len(names) > MAX_FILES:
                raise ValueError('Recovery tree exceeds entry bound')
    _aliases(names, directories)
    return result


def _zip_hashes(path, prefix=''):
    _local(path)
    before = _plain(path)
    if before.st_size > MAX_ARCHIVE:
        raise ValueError('Archive exceeds size bound')
    result = {}
    total = 0
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        if not infos or len(infos) > MAX_FILES:
            raise ValueError('Bounded nonempty archive required')
        for entry in infos:
            mode = entry.external_attr >> 16
            if (entry.filename != entry.orig_filename or entry.is_dir()
                    or entry.flag_bits & 0x41 or entry.external_attr & 0x410 or entry.compress_type not in {zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED}
                    or stat.S_IFMT(mode) not in {0, stat.S_IFREG}):
                raise ValueError('Unsupported archive entry or link')
            if prefix and not entry.filename.startswith(prefix + '/'):
                raise ValueError('Unexpected Recovery archive root')
            name = entry.filename[len(prefix) + 1:] if prefix else entry.filename
            _path(name)
            if name in result:
                raise ValueError('Duplicate archive path')
            total += entry.file_size
            if entry.file_size > MAX_FILE or total > MAX_TOTAL:
                raise ValueError('Archive expansion exceeds bound')
            digest = hashlib.sha256()
            length = 0
            # Reading to EOF verifies each member CRC without extraction.
            with archive.open(entry) as member:
                while chunk := member.read(1024 * 1024):
                    length += len(chunk)
                    if length > entry.file_size:
                        raise ValueError('Archive member exceeded declared size')
                    digest.update(chunk)
            if length != entry.file_size:
                raise ValueError('Archive member length differs')
            result[name] = digest.hexdigest()
    if _signature(_plain(path)) != _signature(before):
        raise ValueError('Archive changed during verification')
    _aliases(result)
    return result


def _compatibility(supported):
    if not isinstance(supported, list) or not 1 <= len(supported) <= 16:
        raise ValueError('Bounded supported OpenCPN identities required')
    identities = set()
    for item in supported:
        if not isinstance(item, dict) or not COMPAT_REQUIRED <= item.keys() or item.keys() - COMPAT_REQUIRED - COMPAT_OPTIONAL:
            raise ValueError('Unknown OpenCPN identity shape')
        if item['version'] != '5.12.4' or item['arch'] != 'x86':
            raise ValueError('Unsupported OpenCPN version/ABI')
        identity = (item['version'], item['arch'], _hex(item['executableSha256']), _hex(item['upstreamCommit'], 40))
        if identity in identities:
            raise ValueError('Duplicate OpenCPN identity')
        identities.add(identity)
        for key in item.keys() - COMPAT_REQUIRED:
            if not isinstance(item[key], str) or not 1 <= len(item[key]) <= 2048 or re.search(r'[\x00-\x1f\x7f]', item[key]):
                raise ValueError('Unbounded OpenCPN qualification metadata')
    return identities


def _package(raw, commit, version):
    value = _json(raw)
    keys = {'schema', 'version', 'commit', 'payloadSha256', 'supportedOpenCpn', 'files', 'qualification'}
    if not isinstance(value, dict) or set(value) != keys or type(value['schema']) is not int or value['schema'] != 1:
        raise ValueError('Exact installer package schema required')
    if value['commit'] != _hex(commit, 40) or value['version'] != validate_product_version(version):
        raise ValueError('Installer package commit/version differs')
    _hex(value['payloadSha256'])
    if not isinstance(value['qualification'], str) or value['qualification'] not in {'candidate', 'accepted'}:
        raise ValueError('Unknown installer qualification')
    _compatibility(value['supportedOpenCpn'])
    files = value['files']
    if not isinstance(files, list) or not 1 <= len(files) <= MAX_FILES:
        raise ValueError('Bounded installer file list required')
    expected = {}
    for item in files:
        if not isinstance(item, dict) or set(item) != {'path', 'sha256'}:
            raise ValueError('Exact installer file entry required')
        name = _path(item['path'])
        if not name.startswith(('app/', 'docs/')) or name in expected or name == MARKER:
            raise ValueError('Unexpected or duplicate installer payload path')
        expected[name] = _hex(item['sha256'])
    _aliases(expected)
    return value, expected


def _product(recovery, expected, commit, version):
    build = _json(_read(recovery / 'docs/PRODUCT_BUILD.json', MAX_PRODUCT, collect=True))
    required = {'version', 'commit', 'test_fixtures', 'build_purpose', 'xnav_hardware_output_policy',
                'xnav_manual_control_contract', 'executable_sha256', 'restart_helper_sha256',
                'commissioning_restart_protocol', 'update_startup_health', 'startup_launcher_sha256'}
    if (not isinstance(build, dict) or not required <= build.keys() or build.keys() - required - {'update_trust'}
            or any(value is None for value in build.values())
            or type(build['xnav_manual_control_contract']) is not int
            or type(build['commissioning_restart_protocol']) is not int or build['commissioning_restart_protocol'] != 1
            or type(build['update_startup_health']) is not int or build['update_startup_health'] != 1):
        raise ValueError('Exact Recovery reader product record required')
    for path, key in [('app/opennav-restart.exe', 'restart_helper_sha256'),
                      ('app/skager-start.exe', 'startup_launcher_sha256')]:
        if expected.get(path) != _hex(build[key]):
            raise ValueError('Product helper executable identity differs')
    if (not isinstance(build, dict) or build.get('commit') != commit or build.get('version') != version
            or build.get('test_fixtures') is not False or build.get('build_purpose') != 'INSTALLED PRODUCT'):
        raise ValueError('Exact fixture-free product identity required')
    require_product_output_policy(build)
    executable = expected.get('app/opencpn.exe')
    if not executable or build.get('executable_sha256') != executable:
        raise ValueError('Product executable identity differs')
    # Only the data-only PE machine/header check; native version resources and
    # actual runtime self-test remain the original producer's separate gates.
    path = recovery / 'app/opencpn.exe'
    with path.open('rb') as stream:
        header = stream.read(64)
        if len(header) != 64 or header[:2] != b'MZ':
            raise ValueError('Product requires native x86 PE executable')
        offset = struct.unpack_from('<I', header, 60)[0]
        if offset < 64 or offset > path.stat().st_size - 26:
            raise ValueError('Invalid product PE offset')
        stream.seek(offset)
        pe = stream.read(26)
        if pe[:4] != b'PE\0\0' or struct.unpack_from('<H', pe, 4)[0] != 0x14c or struct.unpack_from('<H', pe, 24)[0] != 0x10b:
            raise ValueError('Product requires supported Win32 ABI')


def _write(path, raw):
    with path.open('xb') as output:
        output.write(raw)
        output.flush()
        os.fsync(output.fileno())


def validate_installer_helpers(installer, expected):
    """Check fixed producer snapshots, not extracted Setup or publisher trust."""
    installer = _local(installer, True)
    if not isinstance(expected, dict) or set(expected) != set(ENGINE_HELPERS):
        raise ValueError('Exact producer helper snapshot hashes required')
    for digest in expected.values():
        _hex(digest)
    engine = _local(installer / 'engine', True)
    if {child.name for child in engine.iterdir()} != set(ENGINE_HELPERS):
        raise ValueError('Exact fixed installer helper namespace required')
    result = {}
    for name in ENGINE_HELPERS:
        path = _local(engine / name)
        if not 1 <= _plain(path).st_size <= MAX_ENGINE:
            raise ValueError('Bounded nonempty installer helper required')
        result[name] = _read(path, MAX_ENGINE)
    if result != expected:
        raise ValueError('Installer helper snapshot changed')
    return result


def snapshot_installer_helpers(source, installer):
    """Snapshot exact source bytes before NSIS into a fresh fixed output subtree.

    The recipe must pass these snapshots to NSIS and revalidate after compilation.
    This binds producer inputs only, without extraction or first-execution proof.
    """
    source = _local(source, True)
    installer = _local(installer, True)
    names = {name.casefold(): name for name in ENGINE_HELPERS}
    if any(child.name.casefold() in names and child.name != names[child.name.casefold()]
           for child in source.iterdir()):
        raise ValueError('Case-aliased source helper refused')
    if any(child.name.casefold() == 'engine' for child in installer.iterdir()):
        raise ValueError('Fresh installer helper snapshot directory required')
    original = {}
    for name in ENGINE_HELPERS:
        raw = _read(_local(source / name), MAX_ENGINE, collect=True)
        if not raw:
            raise ValueError('Nonempty source helper required')
        original[name] = raw
    engine = installer / 'engine'
    engine.mkdir()
    hashes = {}
    for name, raw in original.items():
        _write(engine / name, raw)
        hashes[name] = hashlib.sha256(raw).hexdigest()
    validate_installer_helpers(installer, hashes)
    return hashes


def _publish(replacements, sidecar_directory, staged_sidecar, stage):
    installed = []
    backups = []
    try:
        # Fresh root sidecar directory; never merge into a preexisting namespace.
        staged_sidecar.rename(sidecar_directory)
        installed.append(sidecar_directory)
        for index, (old, new) in enumerate(replacements):
            backup = stage / ('original-' + str(index))
            old.rename(backup)
            backups.append((old, backup))
            new.rename(old)
            installed.append(old)
    except BaseException as original:
        # Attempt every rollback action. Retain originals if a cleanup also fails;
        # never mask the initial failure or overwrite an unexpected destination.
        failures = []
        for path in reversed(installed):
            try:
                if path.is_dir():
                    shutil.rmtree(path)
                else:
                    path.unlink()
            except BaseException as error:
                failures.append(error)
        for old, backup in reversed(backups):
            try:
                if os.path.lexists(old):
                    raise OSError('Rollback destination still occupied: ' + str(old))
                backup.rename(old)
            except BaseException as error:
                failures.append(error)
        if failures:
            raise RuntimeError('Receipt publication/rollback failed; retained originals: ' + str(stage)) from original
        raise


def retain_installer_package(recovery, installer, commit, version, supported_opencpn, engine_hashes):
    """Add exact package.json and the fixed actual NSIS helper snapshots.

    Requires successful Setup production using these snapshots, with their hashes
    retained by the producer before compilation. No signature/launch
    authority is created: the final signed outer Recovery hash must authenticate
    this sidecar at consumption. Returns only content-binding diagnostics.
    """
    recovery = _local(recovery, True)
    installer = _local(installer, True)
    if recovery.name != PREFIX or recovery == installer or recovery in installer.parents or installer in recovery.parents:
        raise ValueError('Separate exact Recovery and installer roots required')
    commit = _hex(commit, 40)
    version = validate_product_version(version)
    validate_installer_helpers(installer, engine_hashes)
    engines = {f'installer/engine/{name}': _read(installer / 'engine' / name, MAX_ENGINE, collect=True)
               for name in ENGINE_HELPERS}
    if {name.rsplit('/', 1)[1]: hashlib.sha256(raw).hexdigest() for name, raw in engines.items()} != engine_hashes:
        raise ValueError('Installer helpers changed while reading snapshots')
    record = _local(installer / 'package.json')
    payload = _local(installer / 'payload.zip')
    setup = _local(installer / 'SKAGER-Beta2-Setup.exe')
    setup_hash = _read(setup)
    setup_checksum = _local(setup.with_suffix('.exe.sha256'))
    setup_checksum_raw = _read(setup_checksum, 1024, collect=True)
    if setup_checksum_raw.decode('ascii').splitlines() != [setup_hash + '  ' + setup.name]:
        raise ValueError('Generated Setup checksum differs')
    raw = _read(record, MAX_METADATA, collect=True)
    package, expected = _package(raw, commit, version)
    if _compatibility(package['supportedOpenCpn']) != _compatibility(supported_opencpn):
        raise ValueError('Installer OpenCPN identities differ from selected source compatibility')
    if _read(payload, MAX_ARCHIVE) != package['payloadSha256'] or _zip_hashes(payload) != expected:
        raise ValueError('Exact installer payload hash/file list differs')
    actual = _tree(recovery)
    if SIDECAR in actual or any(name.split('/')[0].casefold() == 'installer' for name in actual):
        raise ValueError('Fresh Recovery installer receipt required')
    if any(child.name.casefold() == 'installer' for child in recovery.iterdir()):
        raise ValueError('Preexisting installer receipt directory refused')
    original_manifest = _json(_read(recovery / INVENTORY, MAX_METADATA, collect=True))
    if not isinstance(original_manifest, dict):
        raise ValueError('Recovery inventory object required')
    _aliases(original_manifest)
    for digest in original_manifest.values():
        _hex(digest)
    if original_manifest != {name: digest for name, digest in actual.items() if name != INVENTORY}:
        raise ValueError('Original Recovery inventory differs from actual files')
    appdocs = {name: digest for name, digest in actual.items() if name.startswith(('app/', 'docs/')) and name != MARKER}
    if appdocs != expected:
        raise ValueError('Recovery app/docs differ from exact installer payload')
    _product(recovery, expected, commit, version)
    archive = _local(recovery.parent / (PREFIX + '.zip'))
    checksum = _local(archive.with_suffix('.zip.sha256'))
    old_archive_hash = _read(archive, MAX_ARCHIVE)
    checksum_raw = _read(checksum, 1024, collect=True)
    if checksum_raw.decode('ascii').splitlines() != [old_archive_hash + '  ' + archive.name]:
        raise ValueError('Original Recovery archive checksum differs')
    if _zip_hashes(archive, PREFIX) != actual:
        raise ValueError('Original Recovery archive differs from actual files')
    new_manifest = dict(original_manifest)
    new_manifest[SIDECAR] = hashlib.sha256(raw).hexdigest()
    new_manifest.update({name: hashlib.sha256(body).hexdigest() for name, body in engines.items()})
    manifest_raw = (json.dumps(new_manifest, indent=2) + '\n').encode('utf-8')
    if len(manifest_raw) > MAX_METADATA:
        raise ValueError('Updated Recovery inventory exceeds bound')
    stage = Path(tempfile.mkdtemp(prefix='.installer-receipt-', dir=recovery.parent))
    completed = False
    try:
        sidecar = stage / 'installer'
        sidecar.mkdir()
        _write(sidecar / 'package.json', raw)
        (sidecar / 'engine').mkdir()
        for name, body in engines.items():
            _write(sidecar / 'engine' / name.rsplit('/', 1)[1], body)
        staged_manifest = stage / INVENTORY
        _write(staged_manifest, manifest_raw)
        staged_archive = stage / archive.name
        final_files = dict(actual, **{INVENTORY: hashlib.sha256(manifest_raw).hexdigest(), SIDECAR: hashlib.sha256(raw).hexdigest()})
        final_files.update({name: hashlib.sha256(body).hexdigest() for name, body in engines.items()})
        with zipfile.ZipFile(staged_archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=6, allowZip64=False) as output:
            for name, digest in sorted(final_files.items()):
                source = staged_manifest if name == INVENTORY else sidecar / Path(name).relative_to('installer') if name.startswith('installer/') else recovery / name
                with output.open(PREFIX + '/' + name, 'w') as member:
                    if _read(source, MAX_FILE, target=member) != digest:
                        raise ValueError('Recovery changed while staging archive')
        if _zip_hashes(staged_archive, PREFIX) != final_files:
            raise ValueError('Staged Recovery archive differs')
        new_archive_hash = _read(staged_archive, MAX_ARCHIVE)
        staged_checksum = stage / checksum.name
        _write(staged_checksum, (new_archive_hash + '  ' + archive.name + '\n').encode('ascii'))
        validate_installer_helpers(installer, engine_hashes)
        if (_tree(recovery) != actual or _read(record, MAX_METADATA, collect=True) != raw
                or _read(payload, MAX_ARCHIVE) != package['payloadSha256']
                or _read(setup) != setup_hash or _read(setup_checksum, 1024, collect=True) != setup_checksum_raw
                or _read(archive, MAX_ARCHIVE) != old_archive_hash
                or _read(checksum, 1024, collect=True) != checksum_raw):
            raise ValueError('Packaging inputs changed before publication')
        _publish([(recovery / INVENTORY, staged_manifest), (archive, staged_archive), (checksum, staged_checksum)], recovery / 'installer', sidecar, stage)
        completed = True
        return {'packageSha256': hashlib.sha256(raw).hexdigest(), 'payloadSha256': package['payloadSha256'],
                'recoverySha256': new_archive_hash, 'setupSha256': setup_hash, 'commit': commit, 'version': version,
                'engineSha256': dict(engine_hashes),
                'appDocsUnchanged': True, 'launchAuthority': False}
    finally:
        # On failed publication retain staged/original records for inspection;
        # the caller must treat any raised exception as a failed package.
        if completed:
            shutil.rmtree(stage)
