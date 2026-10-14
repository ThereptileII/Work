"""Verify a same-revision broker prerequisite; never provision or activate it.

Successful authenticated remote producer provenance remains the caller's duty.
Hashes, PE metadata and corresponding-source records bind copied bytes; they do
not prove publisher authenticity, reproducibility or native service qualification.
"""
import hashlib
import json
from pathlib import PurePosixPath
from typing import NamedTuple
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
# Both executables retain the same registration/enrollment source closure. The
# fixed lists detect omitted members even if an archive's own manifest omits
# them too; hashing only the members that remain cannot establish completeness.
REGISTRATION_SOURCES = frozenset(PREFIX + name for name in (
    'approved_baseline.go', 'artifact.go', 'broker_bootstrap.go', 'broker_download.go',
    'broker_code_upgrade.go',
    'download.go', 'download_portable.go', 'download_windows.go', 'identity.go',
    'prepare.go', 'recovery.go', 'release.go', 'stage_recovery.go', 'state.go',
    'state_unix.go', 'state_windows.go', 'verifier.go', 'baselineevidence/evidence.go',
    'brokerhost/host_other.go', 'brokerhost/host_windows.go',
    'brokerhost/initialization.go', 'brokerhost/operation.go', 'brokerhost/registration.go',
    'brokerhost/execution_child.go', 'brokerhost/execution_child_other.go',
    'brokerhost/execution_child_windows.go', 'brokerhost/execution_child_profile.go',
    'brokerhost/execution_child_profile_other.go', 'brokerhost/execution_child_profile_windows.go',
    'brokerhost/execution_selection.go', 'brokerhost/execution_selection_windows.go',
    'brokerhost/execution_capability.go',
    'brokerhost/execution_authorization.go', 'brokerhost/execution_commission.go',
    'brokerhost/execution_current_host.go', 'brokerhost/execution_current_host_windows.go',
    'brokerhost/execution_current_providers_windows.go',
    'brokerhost/execution_application_owner.go', 'brokerhost/execution_application_owner_windows.go',
    'brokerhost/execution_manual_rollback.go', 'brokerhost/execution_manual_rollback_windows.go',
    'brokerhost/execution_repair.go', 'brokerhost/execution_repair_windows.go',
    'brokerhost/execution_repair_custody.go', 'brokerhost/execution_repair_custody_windows.go',
    'brokerhost/maintenance_host_windows.go',
    'brokerhost/execution_host.go', 'brokerhost/execution_host_history.go',
    'brokerhost/execution_host_spawn_windows.go', 'brokerhost/execution_host_windows.go',
    'brokerhost/execution_profile_capture.go', 'brokerhost/execution_profile_capture_other.go',
    'brokerhost/execution_profile_capture_windows.go',
    'brokerhost/execution_profile_capture_restore.go', 'brokerhost/execution_profile_capture_restore_windows.go',
    'brokerhost/execution_profile_capture_reopen.go',
    'brokerhost/execution_profile_capture_reopen_windows.go',
    'brokerhost/execution_profile_capture_reopen_other.go',
    'brokerhost/execution_providers_windows.go', 'brokerhost/execution_relay_host_windows.go',
    'brokerhost/execution_restore_authority_windows.go', 'brokerhost/service_endpoints.go',
    'brokerhost/execution_startup_host_windows.go',
    'brokerhost/execution_offer_host_windows.go',
    'brokerhost/client_registration.go', 'brokerhost/client_registration_other.go',
    'brokerhost/client_registration_windows.go', 'brokerhost/client_baseline_windows.go',
    'brokerhost/registration_start.go',
    'brokerhost/stock_cleanup.go', 'brokerhost/stock_windows.go',
    'brokerhost/decommission.go', 'brokerhost/decommission_windows.go',
    'brokerhost/retirement_lineage.go',
    'brokerhost/selected_diagnostics.go', 'brokerhost/selected_diagnostics_windows.go',
    'brokerpipe/identity_windows.go', 'brokerpipe/io_windows.go',
    'brokerpipe/peer_capability.go', 'brokerpipe/peer_capability_windows.go',
    'brokerpipe/maintenance_peer_other.go', 'brokerpipe/maintenance_peer_windows.go',
    'brokerpipe/maintenance_protocol.go', 'brokerpipe/maintenance_native_windows.go',
    'brokerpipe/maintenance_other.go', 'brokerpipe/maintenance_location_windows.go',
    'brokerpipe/maintenance_location_other.go',
    'brokerpipe/maintenance_folder_token_windows.go',
    'brokerpipe/maintenance_diagnostics.go',
    'brokerpipe/repair.go',
    'brokerpipe/decommission.go', 'brokerpipe/decommission_windows.go',
    'brokerpipe/decommission_other.go',
    'brokerpipe/uninstall_fence.go', 'brokerpipe/uninstall_protocol.go',
    'brokerpipe/execution_protocol.go', 'brokerpipe/execution_session.go',
    'brokerpipe/execution_windows.go', 'brokerpipe/execution_other.go',
    'brokerpipe/native_other.go', 'brokerpipe/native_windows.go', 'brokerpipe/protocol.go',
    'brokerregistration/request.go',
    'brokerregistration/protected_record.go',
    'brokerstore/store.go', 'brokerstore/store_other.go', 'brokerstore/store_windows.go',
    'brokerstore/store_lock_windows.go', 'brokerstore/acquisition.go',
    'brokerstore/programdata_guard_windows.go',
    'brokerstore/decommission.go', 'brokerstore/decommission_windows.go',
    'brokerstore/decommission_inventory.go', 'brokerstore/decommission_inventory_windows.go',
    'brokerstore/retirement.go', 'brokerstore/retirement_windows.go',
    'brokerstore/acquisition_journal.go', 'brokerstore/acquisition_windows.go',
    'brokerstore/acquisition_recovery_windows.go', 'brokerstore/initialize.go',
    'brokerstore/candidate_evidence.go', 'brokerstore/candidate_evidence_windows.go',
    'brokerstore/initialize_other.go', 'brokerstore/initialize_windows.go',
    'brokerstore/registration.go', 'brokerstore/registration_windows.go',
    'brokerstore/installed_baseline.go', 'brokerstore/installed_baseline_windows.go',
    'brokerstore/installed_generation.go', 'brokerstore/installed_generation_windows.go',
    'brokerstore/generation_readonly.go', 'brokerstore/generation_readonly_windows.go',
    'brokerstore/generation_baseline.go', 'brokerstore/generation_baseline_windows.go',
    'brokerstore/generation_history.go', 'brokerstore/generation_history_windows.go',
    'brokerstore/generation_executor.go', 'brokerstore/generation_executor_windows.go',
    'brokerstore/generation_application.go', 'brokerstore/generation_application_windows.go',
    'brokerstore/generation_reopen.go', 'brokerstore/generation_reopen_windows.go',
    'brokerstore/generation_repair.go', 'brokerstore/generation_repair_windows.go',
    'brokerstore/generation_repair_native_source_windows.go',
    'brokerstore/generation_repair_recovery.go',
    'brokerstore/execution_journal.go', 'brokerstore/execution_journal_windows.go',
    'brokerstore/execution_journal_reopen.go', 'brokerstore/execution_journal_reopen_windows.go',
    'brokerstore/execution_profile_backup.go', 'brokerstore/execution_profile_backup_windows.go',
    'brokerstore/generation_stage_seal.go', 'brokerstore/generation_stage_seal_windows.go',
    'brokerenroll/enroll.go', 'brokerenroll/files.go', 'brokerenroll/files_other.go',
    'brokerenroll/code_upgrade.go', 'brokerenroll/code_upgrade_record.go',
    'brokerenroll/code_upgrade_other.go', 'brokerenroll/code_upgrade_windows.go',
    'brokerenroll/code_upgrade_finalize.go', 'brokerenroll/code_upgrade_finalize_other.go',
    'brokerenroll/code_upgrade_finalize_windows.go',
    'brokerenroll/files_windows.go', 'brokerenroll/applications_windows.go',
    'brokerenroll/programdata_guard_windows.go',
    'brokerenroll/service.go', 'brokerenroll/service_other.go',
    'brokerenroll/service_windows.go', 'brokerenroll/client_request.go',
    'brokerenroll/client_request_other.go', 'brokerenroll/client_request_windows.go',
    'brokerenroll/client_baseline.go', 'brokerenroll/client_baseline_other.go',
    'brokerenroll/client_baseline_windows.go',
    'brokerenroll/startup.go', 'brokerenroll/startup_other.go', 'brokerenroll/startup_shortcuts.go',
    'brokerenroll/startup_windows.go', 'brokerenroll/startup_link_windows.go',
    'brokerenroll/startup_publication_windows.go',
    'brokerenroll/startup_binding.go', 'brokerenroll/startup_binding_windows.go',
    'brokerenroll/startup_restoration.go', 'brokerenroll/startup_restoration_other.go',
    'brokerenroll/startup_restoration_windows.go',
    'brokerenroll/startup_retirement_anchor_windows.go',
    'brokerenroll/startup_installed_windows.go', 'brokerenroll/startup_link_read_windows.go',
    'brokerenroll/maintenance.go', 'brokerenroll/maintenance_other.go',
    'brokerenroll/maintenance_windows.go', 'brokerenroll/maintenance_process_windows.go',
    'brokerenroll/maintenance_decommission.go', 'brokerenroll/maintenance_decommission_windows.go',
    'brokerenroll/installed_inventory.go', 'brokerenroll/installed_registry_windows.go',
    'brokerenroll/installed_uninstall.go', 'brokerenroll/installed_uninstall_other.go',
    'brokerenroll/installed_uninstall_windows.go',
    'brokerenroll/installed_completion.go', 'brokerenroll/installed_completion_other.go',
    'brokerenroll/installed_completion_windows.go',
    'brokerenroll/installed_maintenance.go', 'brokerenroll/installed_maintenance_other.go',
    'brokerenroll/installed_maintenance_windows.go',
    'brokerenroll/maintenance_image.go', 'brokerenroll/maintenance_image_other.go',
    'brokerenroll/maintenance_image_windows.go',
    'launcher/broker_update.go', 'launcher/broker_update_other.go',
    'launcher/broker_update_windows.go',
    'launcher/maintenance_barrier_windows.go', 'launcher/maintenance_barrier_other.go',
    'launcher/maintenance_client.go', 'launcher/maintenance_client_windows.go',
    'launcher/maintenance_client_other.go', 'launcher/maintenance_folder_access.go',
    'launcher/maintenance_folder_access_windows.go',
    'launcher/decommission_client.go', 'launcher/decommission_client_windows.go',
    'launcher/decommission_client_other.go',
    'launcher/broker_query_access.go', 'launcher/broker_query_access_windows.go',
    'releasepolicy/broker_bootstrap.go', 'releasepolicy/broker_code_upgrade.go',
    'releasepolicy/evaluate.go',
    'releasepolicy/policy.go', 'releasepolicy/semver.go',
    'repository/broker_bootstrap.go', 'repository/broker_code_upgrade.go', 'repository/inputs.go',
    'repository/lock_linux.go', 'repository/lock_other.go', 'repository/repository.go'))
REQUIRED_SOURCES = REGISTRATION_SOURCES | {
    PREFIX + 'cmd/skager-update-broker/main.go', PREFIX + 'go.mod', PREFIX + 'go.sum',
    'project/LICENSE', 'project/tools/package-update-launcher.py',
    'project/tools/package-update-broker.py', 'project/tools/broker_package.py',
    'project/tools/updater_package.py'}


class Component(NamedTuple):
    """Fixed source-defined component identity; never a caller-selected helper."""
    name: str
    binary: str
    record_relative: str
    command: str
    recipe: str
    subsystem: int
    required_sources: frozenset
    source_prefix: str
    linker_args: tuple = ()


BROKER = Component('Broker', BINARY, RECORD_RELATIVE, 'cmd/skager-update-broker',
                   BUILD_RECIPE, 2, frozenset(REQUIRED_SOURCES), PREFIX,
                   ('-ldflags=-H=windowsgui',))


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


def _embedded_info(binary, component=BROKER):
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
                struct.unpack_from('<H', optional, 68)[0] != component.subsystem or
                not characteristics & 2 or characteristics & 0x2000):
            raise ValueError(component.name + ' must use Windows ' + ('GUI' if component.subsystem == 2 else 'console') + ' Win32 x86 PE32 executable format')
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


def _build_info(binary, declared, commit, reference, component=BROKER):
    if not isinstance(declared, str) or len(declared.encode('utf-8')) > MAX_BUILD_INFO:
        raise ValueError('Broker buildInfo missing or oversized')
    version, module = _embedded_info(binary, component)
    if version != updater.GO_VERSION or not module.endswith('\n'):
        raise ValueError('Broker actual Go compiler identity mismatch')
    lines = module.splitlines()
    expected = component.binary + ': ' + version + '\n' + ''.join('\t' + line + '\n' for line in lines)
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
                'CGO_ENABLED': '0', 'GOARCH': '386',
                'GOOS': 'windows', 'GO386': 'sse2', 'vcs': 'git',
                'vcs.revision': commit, 'vcs.modified': 'false'}
    if component != BROKER and set(settings) != set(required) | {'vcs.time'}:
        raise ValueError('Provisioner actual Go build settings differ from fixed recipe')
    # Pinned Go omits -ldflags with -trimpath (cmd/go/internal/load/pkg.go).
    # Component identity comes from the PE subsystem and fixed declared recipe.
    if ('-ldflags' in settings or
            paths != [updater.MAIN_MODULE + '/' + component.command] or
            mains != [[updater.MAIN_MODULE, '(devel)', '']] or
            any(settings.get(k) != v for k, v in required.items()) or
            not re.fullmatch(r'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ', settings.get('vcs.time', ''))):
        raise ValueError('Broker actual Go build settings do not match pinned product recipe')


def _source_contents(archive, reference, component=BROKER):
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
        if not component.required_sources <= set(files):
            raise ValueError('Actual tracked broker main/host sources missing from archive')
        checked = component.required_sources | {n for n in files if n.startswith(component.source_prefix) and n.endswith('.go')}
        for name in checked:
            data = source.read(name)  # Bounded above; ZipFile also verifies CRC.
            if not data or hashlib.sha256(data).hexdigest() != files[name]['sha256']:
                raise ValueError('Archived broker source bytes differ from inventory')


def _verify_component_package(app, commit, component, *, prerequisite=None):
    """Return the unchanged source-package descriptor for exact component bytes.

    This is opt-in packaging validation, not installation/SCM/trust authority.
    The caller authenticates successful same-revision remote production first.
    """
    descriptor = updater.verify_updater_package(app, commit)
    app = updater._plain(app)
    record_path = _regular(app / component.record_relative, updater.MAX_RECORD_BYTES)
    record = json.loads(record_path.read_text(encoding='utf-8'), object_pairs_hook=updater._unique)
    keys = ('schema', 'productCommit', 'goVersion', 'binary', 'sourceBundle', 'buildInfo', 'build')
    updater._keys(record, keys + (('brokerPrerequisite',) if prerequisite is not None else ()))
    if prerequisite is not None and record['brokerPrerequisite'] != prerequisite:
        raise ValueError('Provisioner broker prerequisite descriptor changed')
    if (type(record['schema']) is not int or record['schema'] != 1 or record['productCommit'] != commit or
            record['goVersion'] != updater.GO_VERSION or record['build'] != component.recipe):
        raise ValueError('Broker package commit/compiler/build recipe mismatch')
    updater._keys(record['binary'], ('path', 'sha256'))
    updater._keys(record['sourceBundle'], ('archive', 'path', 'sha256', 'reference'))
    updater._reference(record['sourceBundle']['reference'], commit)
    expected_bundle = {**descriptor, 'archive': updater.SOURCE_RELATIVE}
    if record['binary']['path'] != component.binary or record['sourceBundle'] != expected_bundle:
        raise ValueError('Broker requires fixed binary and exact updater source descriptor')
    binary = _regular(app / component.binary, updater.MAX_BINARY_BYTES)
    archive = _regular(descriptor['archive'], updater.MAX_ARCHIVE_BYTES)
    if updater._hash(binary) != updater._digest(record['binary']['sha256']):
        raise ValueError('Broker executable changed after production')
    _build_info(binary, record['buildInfo'], commit, descriptor['reference'], component)
    _source_contents(archive, descriptor['reference'], component)
    return descriptor


def verify_broker_package(app, commit):
    """Verify the fixed GUI SCM broker without installing or activating it."""
    return _verify_component_package(app, commit, BROKER)


def broker_prerequisite(app, commit):
    """Bind a verified native prerequisite; hashes do not authenticate execution."""
    app = updater._plain(app)

    def snapshot():
        return {'binary': {'path': BINARY,
                           'sha256': updater._hash(_regular(app / BINARY, updater.MAX_BINARY_BYTES))},
                'buildRecord': {'path': RECORD_RELATIVE,
                                'sha256': updater._hash(_regular(app / RECORD_RELATIVE, updater.MAX_RECORD_BYTES))}}
    before = snapshot()
    verify_broker_package(app, commit)
    if snapshot() != before:
        raise ValueError('Broker prerequisite changed during verification')
    return before
