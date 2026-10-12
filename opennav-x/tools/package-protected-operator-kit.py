#!/usr/bin/env python3
"""Package three inactive protected-broker operator tools on native Windows.

Consume an original authenticated same-commit launcher producer install; never
rebuild its launcher or change that install. The ZIP must be paired with the
independently authenticated SAME-COMMIT recovery/launcher prerequisite before
verification/use. Checksums bind bytes, not publisher trust or operator approval.
No output is executed/elevated, enrolled, signed or automatically approved.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import zipfile

import baseline_handoff_package as handoff
import broker_package as broker
import provisioner_package as provisioner
import updater_package as updater


def _producer(name):
    spec = importlib.util.spec_from_file_location('_operator_kit_' + name.replace('-', '_'),
                                                  Path(__file__).with_name(name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


broker_producer = _producer('package-update-broker')
provisioner_producer = _producer('package-update-provisioner')
handoff_producer = _producer('package-baseline-handoff')
COMPONENTS = (broker.BROKER, provisioner.PROVISIONER, handoff.BASELINE_HANDOFF)
LICENSE = 'opennav/third-party/updater/LICENSE'
REFERENCE = 'opennav/third-party/updater/SOURCE_REFERENCE.json'
MANIFEST = 'SHA256SUMS.json'
MAX_LICENSE_BYTES = 1 << 20
MAX_KIT_BYTES = 768 << 20
COMPONENT_LIMITS = {component.binary: updater.MAX_BINARY_BYTES for component in COMPONENTS}
COMPONENT_LIMITS.update({component.record_relative: updater.MAX_RECORD_BYTES for component in COMPONENTS})
COMPONENT_INSTALL_LIMITS = {**updater.TRANSFER_FILES, **COMPONENT_LIMITS}
PAYLOAD_LIMITS = dict(COMPONENT_LIMITS)
PAYLOAD_LIMITS.update({updater.SOURCE_RELATIVE: updater.MAX_ARCHIVE_BYTES,
                       LICENSE: MAX_LICENSE_BYTES, REFERENCE: broker.MAX_SOURCE_RECORD})


def _snapshot(root, limits):
    """Bind bounded single-link original files, including native replacement IDs."""
    result = {}
    for name, limit in limits.items():
        path = broker._regular(root / name, limit)
        before = path.stat()
        checksum = updater._hash(path)
        after = path.stat()
        identity = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns, s.st_nlink)
        if identity(before) != identity(after):
            raise ValueError('Operator kit input changed during read')
        result[name] = (identity(after), checksum)
    return result


def _source_snapshot(source, commit, limits=updater.TRANSFER_FILES):
    descriptor = updater.verify_updater_package(source, commit)
    # The existing transfer API also enforces this exact original producer tree.
    # Recheck its inventory after builds, so added state cannot go unnoticed.
    allowed = set(limits)
    allowed |= {parent.as_posix() for name in limits
                for parent in Path(name).parents if parent != Path('.')}
    for current, directories, files in os.walk(source, followlinks=False):
        for name in directories + files:
            child = updater._plain(Path(current) / name)
            if child.relative_to(source).as_posix() not in allowed:
                raise ValueError('Unexpected producer install entry')
    return descriptor, _snapshot(source, limits)


def _verify_components(install, commit, expected):
    for verify in (broker.verify_broker_package, provisioner.verify_provisioner_package,
                   handoff.verify_baseline_handoff_package):
        descriptor = verify(install, commit)
        if any(descriptor[key] != expected[key] for key in ('path', 'sha256', 'reference')):
            raise ValueError('Operator component corresponding source differs')


def _components_snapshot(source, commit, original):
    current = _source_snapshot(source, commit, COMPONENT_INSTALL_LIMITS)
    _verify_components(source, commit, original[0])
    for name in updater.TRANSFER_FILES:
        identity, checksum = current[1][name]
        original_identity, original_checksum = original[1][name]
        if identity[2] != original_identity[2] or checksum != original_checksum:
            raise ValueError('Components require the exact original launcher/source closure')
    return current


def _copy_component_files(source, install, snapshot):
    # The shared source archive was already copied from the original launcher;
    # components_snapshot binds it to the exact same bytes in this producer.
    for name, limit in COMPONENT_LIMITS.items():
        source_path = broker._regular(source / name, limit)
        target = updater._plain(install / name, allow_missing=True)
        target.parent.mkdir(parents=True, exist_ok=True)
        remaining = snapshot[1][name][0][2]
        with source_path.open('rb') as original, target.open('xb') as output:
            while remaining:
                data = original.read(min(1 << 20, remaining))
                if not data:
                    raise ValueError('Component producer truncated during transfer')
                output.write(data)
                remaining -= len(data)
            if original.read(1):
                raise ValueError('Component producer grew during transfer')
            output.flush()
            os.fsync(output.fileno())
        if updater._hash(target) != snapshot[1][name][1]:
            raise ValueError('Component producer changed during transfer')


def _source_documents(install, descriptor):
    archive = broker._regular(install / updater.SOURCE_RELATIVE, updater.MAX_ARCHIVE_BYTES)
    if updater._hash(archive) != descriptor['sha256']:
        raise ValueError('Operator corresponding source archive changed')
    with broker_producer.retained_source(descriptor) as (source, files):
        if 'project/LICENSE' not in files or not 0 < files['project/LICENSE']['bytes'] <= MAX_LICENSE_BYTES:
            raise ValueError('Bounded corresponding-source license required')
        license_bytes = source.read('project/LICENSE')
        if hashlib.sha256(license_bytes).hexdigest() != files['project/LICENSE']['sha256']:
            raise ValueError('Corresponding-source license hash differs')
        reference_bytes = source.read('SOURCE_REFERENCE.json')
    for name, data in ((LICENSE, license_bytes), (REFERENCE, reference_bytes)):
        path = updater._plain(install / name, allow_missing=True)
        with path.open('xb') as output:
            output.write(data)


def _zip_info(name):
    info = zipfile.ZipInfo(name, (1980, 1, 1, 0, 0, 0))
    info.create_system = 3
    info.external_attr = (stat.S_IFREG | 0o644) << 16
    info.compress_type = zipfile.ZIP_DEFLATED
    return info


def _write_archive(install, archive, manifest):
    with zipfile.ZipFile(archive, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as kit:
        for name in sorted(PAYLOAD_LIMITS):
            path = broker._regular(install / name, PAYLOAD_LIMITS[name])
            with path.open('rb') as source, kit.open(_zip_info(name), 'w') as target:
                remaining = manifest['files'][name]['bytes']
                while remaining:
                    data = source.read(min(1 << 20, remaining))
                    if not data:
                        raise ValueError('Operator kit payload truncated')
                    target.write(data)
                    remaining -= len(data)
                if source.read(1):
                    raise ValueError('Operator kit payload grew')
        kit.writestr(_zip_info(MANIFEST), json.dumps(manifest, sort_keys=True, indent=2) + '\n')
    broker._regular(archive, MAX_KIT_BYTES)
    # Read the actual ZIP back with exact inventory and streamed hashes. No
    # extraction, execution or external paths are involved in this check.
    with zipfile.ZipFile(archive) as kit:
        entries = kit.infolist()
        expected_names = set(PAYLOAD_LIMITS) | {MANIFEST}
        if len(entries) != len(expected_names) or {item.filename for item in entries} != expected_names:
            raise ValueError('Operator kit ZIP inventory differs')
        for item in entries:
            if item.is_dir() or item.flag_bits & 1 or item.external_attr >> 16 != stat.S_IFREG | 0o644:
                raise ValueError('Operator kit ZIP entry type differs')
            if item.filename == MANIFEST:
                if item.file_size > updater.MAX_RECORD_BYTES:
                    raise ValueError('Operator kit checksum manifest oversized')
                actual = json.loads(kit.read(item), object_pairs_hook=updater._unique)
                if actual != manifest:
                    raise ValueError('Operator kit checksum manifest differs')
                continue
            record = manifest['files'][item.filename]
            if item.file_size != record['bytes'] or item.file_size > PAYLOAD_LIMITS[item.filename]:
                raise ValueError('Operator kit ZIP size differs')
            with kit.open(item) as source:
                if hashlib.file_digest(source, 'sha256').hexdigest() != record['sha256']:
                    raise ValueError('Operator kit ZIP payload hash differs')


def package(source_install, output, commit, go='go', *, components_install=None):
    if not broker_producer.launcher.is_native_windows():
        raise ValueError('Protected operator kit production requires native Windows')
    if not isinstance(commit, str) or not updater.COMMIT.fullmatch(commit):
        raise ValueError('Explicit exact product commit required')
    source = updater._plain(source_install)
    output = updater._plain(output, allow_missing=True)
    if output.is_relative_to(source) or output.exists() or output.suffix.lower() != '.zip':
        raise ValueError('New separate operator kit ZIP output required; refusing overwrite')
    if not output.parent.is_dir():
        raise ValueError('Existing plain operator kit output parent required')
    original = _source_snapshot(source, commit)
    components = None
    if components_install is not None:
        components_install = updater._plain(components_install)
        if output.is_relative_to(components_install):
            raise ValueError('Operator kit output must be separate from component producer')
        components = _components_snapshot(components_install, commit, original)

    def check_originals():
        if _source_snapshot(source, commit) != original:
            raise ValueError('Operator kit original launcher producer changed')
        if components is not None and _components_snapshot(components_install, commit, original) != components:
            raise ValueError('Original component producer changed')

    with tempfile.TemporaryDirectory(prefix='.protected-operator-kit-', dir=output.parent) as temporary:
        stage = Path(temporary)
        install = stage / 'install'
        install.mkdir()
        descriptor = updater.copy_verified_updater_package(source, install, commit)
        if components is None:
            for producer in (broker_producer, provisioner_producer, handoff_producer):
                producer.package(install, commit, go)
        else:
            _copy_component_files(components_install, install, components)
            check_originals()
        _verify_components(install, commit, descriptor)
        _source_documents(install, descriptor)
        before = _snapshot(install, PAYLOAD_LIMITS)
        manifest = {'schema': 1, 'productCommit': commit,
                    'files': {name: {'bytes': value[0][2], 'sha256': value[1]}
                              for name, value in before.items()}}
        if sum(item['bytes'] for item in manifest['files'].values()) > MAX_KIT_BYTES:
            raise ValueError('Operator kit payload exceeds total bound')
        archive = stage / 'operator-kit.zip'
        _write_archive(install, archive, manifest)
        _verify_components(install, commit, descriptor)
        if _snapshot(install, PAYLOAD_LIMITS) != before:
            raise ValueError('Operator kit payload changed')
        check_originals()
        updater._plain(output, allow_missing=True)
        # Same-volume exclusive publication refuses a racing output; never
        # overwrite it or retry. The private staging link is then removed.
        os.link(archive, output)
        result = {'schema': 1, 'productCommit': commit, 'bytes': archive.stat().st_size,
                  'sha256': updater._hash(archive)}
        if updater._hash(output) != result['sha256']:
            raise ValueError('Published operator kit changed')
        check_originals()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-install', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--commit', required=True)
    parser.add_argument('--go', default='go')
    parser.add_argument('--components-install', type=Path,
                        help='Read-only exact same-commit three-component producer install; skips all builds')
    args = parser.parse_args()
    try:
        result = package(args.source_install, args.output, args.commit, args.go,
                         components_install=args.components_install)
    except (ValueError, OSError, KeyError, zipfile.BadZipFile, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
