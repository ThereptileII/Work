#!/usr/bin/env python3
"""Native-free copied broker/source checks; no builds, process or network."""
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import tempfile
import unittest
import warnings
from unittest import mock
import zipfile

import broker_package as broker
import updater_package as updater


def sha(data):
    return hashlib.sha256(data).hexdigest()


def varint(value):
    out = bytearray()
    while value >= 128:
        out.append((value & 127) | 128); value >>= 7
    return bytes(out + bytes([value]))


def pe(module, version=updater.GO_VERSION):
    framed = broker.START + module.encode() + broker.END
    data = broker.MAGIC + b'\x04\x02' + bytes(16)
    data += varint(len(version)) + version.encode() + varint(len(framed)) + framed
    result = bytearray(512 + len(data))
    result[:2] = b'MZ'; struct.pack_into('<I', result, 60, 64)
    result[64:68] = b'PE\0\0'
    struct.pack_into('<HH', result, 68, 0x14c, 1)
    struct.pack_into('<HH', result, 84, 224, 2)
    struct.pack_into('<H', result, 88, 0x10b)
    struct.pack_into('<H', result, 88 + 68, 2)
    result[312:320] = b'.data\0\0\0'
    struct.pack_into('<IIII', result, 320, len(data), 4096, len(data), 512)
    struct.pack_into('<I', result, 348, 0xc0000040)
    result[512:] = data
    return bytes(result)


class BrokerPackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='broker-package-')
        self.addCleanup(self.temp.cleanup)
        self.app = Path(self.temp.name)
        self.commit = 'a' * 40
        checksum = 'h1:' + base64.b64encode(bytes(32)).decode()
        self.reference = {'schema': 1, 'productCommit': self.commit, 'goVersion': updater.GO_VERSION,
            'mainModule': 'tools/update-verifier', 'modules': [{'path': 'public.example/module',
                'version': 'v1.0.0', 'sum': checksum, 'goModSum': checksum,
                'sourcePrefix': 'modules/public.example/module@v1.0.0'}],
            'build': broker.BUILD_RECIPE.replace('./cmd/skager-update-broker', './cmd/skager-start')}
        self.module = ('path\t' + updater.MAIN_MODULE + '/cmd/skager-update-broker\nmod\t' +
            updater.MAIN_MODULE + '\t(devel)\t\ndep\tpublic.example/module\tv1.0.0\t' + checksum + '\n' +
            ''.join('build\t' + key + '=' + value + '\n' for key, value in {
                '-buildmode': 'exe', '-compiler': 'gc', '-trimpath': 'true',
                'CGO_ENABLED': '0', 'GOARCH': '386', 'GOOS': 'windows', 'GO386': 'sse2',
                'vcs': 'git', 'vcs.revision': self.commit, 'vcs.time': '2026-10-12T00:00:00Z',
                'vcs.modified': 'false'}.items()))
        self.sources = {name: b'package main\n' if '/cmd/' in name else b'package brokerhost\n'
                        for name in broker.REQUIRED_SOURCES}
        self.archive = self.app / updater.SOURCE_RELATIVE
        self.archive.parent.mkdir(parents=True)
        self.updater_record = {'schema': 1, 'productCommit': self.commit, 'goVersion': updater.GO_VERSION,
            'binary': {'path': 'skager-start.exe', 'sha256': ''},
            'sourceBundle': {'archive': updater.SOURCE_RELATIVE, 'path': updater.BUNDLE_PATH, 'sha256': '',
                'reference': self.reference},
            'buildInfo': 'skager-start.exe: ' + updater.GO_VERSION + '\n\tpath\t' + updater.MAIN_MODULE +
                '/cmd/skager-start\n' + ''.join('\tbuild\t' + setting + '\n' for setting in (
                    '-trimpath=true', 'CGO_ENABLED=0', 'GOARCH=386', 'GOOS=windows',
                    'vcs.revision=' + self.commit, 'vcs.modified=false'))}
        launcher = self.app / 'skager-start.exe'; launcher.write_bytes(pe(self.module))
        self.updater_record['binary']['sha256'] = sha(launcher.read_bytes())
        self.record = {'schema': 1, 'productCommit': self.commit, 'goVersion': updater.GO_VERSION,
            'binary': {'path': broker.BINARY, 'sha256': ''}, 'sourceBundle': {},
            'buildInfo': '', 'build': broker.BUILD_RECIPE}
        self.record_path = self.app / broker.RECORD_RELATIVE
        self.record_path.parent.mkdir(parents=True)
        self.write_source()
        self.write_binary()

    def write_source(self, reference=None, extras=()):
        actual = copy.deepcopy(reference if reference is not None else self.reference)
        actual['files'] = {name: {'bytes': len(data), 'sha256': sha(data)} for name, data in self.sources.items()}
        with zipfile.ZipFile(self.archive, 'w', zipfile.ZIP_DEFLATED) as output:
            for name, data in self.sources.items():
                info = zipfile.ZipInfo(name); info.external_attr = (stat.S_IFREG | 0o644) << 16
                output.writestr(info, data)
            output.writestr('SOURCE_REFERENCE.json', json.dumps(actual))
            for name, data in extras:
                output.writestr(name, data)
        self.updater_record['sourceBundle']['sha256'] = sha(self.archive.read_bytes())
        self.record['sourceBundle'] = copy.deepcopy(self.updater_record['sourceBundle'])
        (self.archive.parent / 'build.json').write_text(json.dumps(self.updater_record))
        self.write_record()

    def write_binary(self, module=None, version=updater.GO_VERSION):
        module = self.module if module is None else module
        self.binary = self.app / broker.BINARY; self.binary.write_bytes(pe(module, version))
        self.record['binary']['sha256'] = sha(self.binary.read_bytes())
        self.record['buildInfo'] = broker.BINARY + ': ' + version + '\n' + ''.join('\t' + line + '\n' for line in module.splitlines())
        self.write_record()

    def write_record(self):
        self.record_path.write_text(json.dumps(self.record))

    def verify(self):
        return broker.verify_broker_package(self.app, self.commit)

    def test_exact_descriptor_and_no_mutation(self):
        before = {p: p.read_bytes() for p in self.app.rglob('*') if p.is_file()}
        self.assertEqual(self.verify(), updater.verify_updater_package(self.app, self.commit))
        self.assertEqual(before, {p: p.read_bytes() for p in before})

    def test_forged_record_metadata_cannot_replace_embedded_info(self):
        for changed in (self.record['buildInfo'].replace(self.commit, 'b' * 40),
                        self.record['buildInfo'] + '\tbuild\tGOARCH=386\n'):
            self.record['buildInfo'] = changed; self.write_record()
            with self.assertRaisesRegex(ValueError, 'embedded'): self.verify()

    def test_actual_bad_build_identity_even_with_matching_record_and_hash(self):
        for before, after in [('GOARCH=386', 'GOARCH=amd64'), ('CGO_ENABLED=0', 'CGO_ENABLED=1'),
            ('vcs.modified=false', 'vcs.modified=true'), (self.commit, 'b' * 40),
            ('/cmd/skager-update-broker', '/cmd/skager-start'), ('-trimpath=true', '-trimpath=false'),
            ('GO386=sse2', 'GO386=softfloat'), ('-compiler=gc', '-compiler=gccgo'),
            ('v1.0.0', 'v1.1.0')]:
            with self.subTest(after=after):
                self.write_binary(self.module.replace(before, after))
                with self.assertRaises(ValueError): self.verify()
        self.write_binary(version='go1.27.0')
        with self.assertRaisesRegex(ValueError, 'compiler'): self.verify()
        self.write_binary(self.module + 'build\tGOARCH=386\n')
        with self.assertRaisesRegex(ValueError, 'Duplicate'): self.verify()

    def test_trimpath_rejects_anomalous_ldflags_metadata(self):
        # Go 1.27.1 intentionally omits this setting under -trimpath, even for
        # the actual windowsgui recipe. Matching a forged record is insufficient.
        for flags in ('-H=windowsgui', '-H=console', ''):
            self.write_binary(self.module + 'build\t-ldflags=' + flags + '\n')
            with self.assertRaisesRegex(ValueError, 'build settings'):
                self.verify()

    def test_mixed_commit_schema_recipe_and_source_descriptor(self):
        original = copy.deepcopy(self.record)
        for mutate in [lambda r: r.update(productCommit='b' * 40), lambda r: r.update(schema=True),
            lambda r: r.update(build=broker.BUILD_RECIPE.replace('readonly', 'mod')),
            lambda r: r.update(extra=True), lambda r: r['sourceBundle'].update(sha256='b' * 64),
            lambda r: r['sourceBundle']['reference'].update(productCommit='b' * 40),
            lambda r: r['sourceBundle']['reference'].update(schema=True)]:
            self.record = copy.deepcopy(original); mutate(self.record); self.write_record()
            with self.assertRaises(ValueError): self.verify()

    def test_wrong_architecture_subsystem_and_binary_bytes(self):
        for offset, value in [(68, 0x8664), (88, 0x20b), (156, 3)]:
            self.write_binary(); data = bytearray(self.binary.read_bytes()); struct.pack_into('<H', data, offset, value)
            self.binary.write_bytes(data); self.record['binary']['sha256'] = sha(data); self.write_record()
            with self.assertRaisesRegex(ValueError, 'Win32'): self.verify()
        self.write_binary(); self.binary.write_bytes(self.binary.read_bytes() + b'changed')
        with self.assertRaisesRegex(ValueError, 'changed'): self.verify()

    def test_missing_renamed_linked_or_oversized_binary_and_record(self):
        self.record['binary']['path'] = 'renamed.exe'; self.write_record()
        with self.assertRaisesRegex(ValueError, 'fixed'): self.verify()
        self.record['binary']['path'] = broker.BINARY; self.write_record()
        outside = self.app / 'other.exe'; self.binary.rename(outside)
        with self.assertRaises(FileNotFoundError): self.verify()
        self.binary.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, 'reparse'): self.verify()
        self.binary.unlink(); os.link(outside, self.binary)
        with self.assertRaisesRegex(ValueError, 'single-link'): self.verify()
        outside.unlink()
        with mock.patch.object(updater, 'MAX_BINARY_BYTES', 8):
            with self.assertRaises(ValueError): self.verify()
        self.record_path.write_bytes(b' ' * (updater.MAX_RECORD_BYTES + 1))
        with self.assertRaises(ValueError): self.verify()

    def test_duplicate_json_and_bounded_build_info(self):
        text = self.record_path.read_text(); self.record_path.write_text('{"schema":1,' + text[1:])
        with self.assertRaisesRegex(ValueError, 'Duplicate'): self.verify()
        self.record['buildInfo'] = 'x' * (broker.MAX_BUILD_INFO + 1); self.write_record()
        with self.assertRaisesRegex(ValueError, 'oversized'): self.verify()

    def test_source_main_host_inventory_reference_and_member_bytes(self):
        for name in broker.REQUIRED_SOURCES:
            data = self.sources.pop(name); self.write_source()
            with self.assertRaisesRegex(ValueError, 'sources missing'): self.verify()
            self.sources[name] = data
        bad = copy.deepcopy(self.reference); bad['productCommit'] = 'b' * 40; self.write_source(bad)
        with self.assertRaisesRegex(ValueError, 'identity|reference differs'): self.verify()
        self.write_source()
        # Replace actual bytes, preserving the inventoried digest and outer hash binding.
        with zipfile.ZipFile(self.archive) as old:
            entries = [(i, old.read(i)) for i in old.infolist()]
        with zipfile.ZipFile(self.archive, 'w') as output:
            for entry, data in entries:
                output.writestr(entry, b'X' * len(data) if entry.filename in broker.REQUIRED_SOURCES else data)
        self.updater_record['sourceBundle']['sha256'] = sha(self.archive.read_bytes())
        self.record['sourceBundle'] = copy.deepcopy(self.updater_record['sourceBundle'])
        (self.archive.parent / 'build.json').write_text(json.dumps(self.updater_record)); self.write_record()
        with self.assertRaisesRegex(ValueError, 'bytes differ'): self.verify()

    def test_source_alias_links_duplicates_unsafe_paths_and_budgets(self):
        name = next(iter(broker.REQUIRED_SOURCES))
        link = zipfile.ZipInfo('linked.go'); link.external_attr = (stat.S_IFLNK | 0o777) << 16
        for entry in (name.upper(), name, '../escape', link):
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', UserWarning)
                self.write_source(extras=[(entry, b'ignored')])
            with self.assertRaisesRegex(ValueError, 'Unsafe'): self.verify()
        self.write_source()
        for constant in ('MAX_SOURCE_FILES', 'MAX_SOURCE_MEMBER', 'MAX_SOURCE_TOTAL', 'MAX_SOURCE_RECORD'):
            with mock.patch.object(broker, constant, 1):
                with self.assertRaises(ValueError): self.verify()

    def test_decorative_overlay_unsupported_inline_and_ambiguous_markers(self):
        original = self.binary.read_bytes()
        # A correctly framed marker in the unmapped overlay is insufficient.
        data = bytearray(original); data[512:526] = bytes(14); data.extend(original[512:])
        variants = [data]
        data = bytearray(original); data[527] = 0; variants.append(data)
        data = bytearray(original); data.extend(bytes((-len(data)) % 16)); data.extend(original[512:])
        struct.pack_into('<I', data, 320, len(data) - 512); struct.pack_into('<I', data, 328, len(data) - 512)
        variants.append(data)
        for data in variants:
            self.binary.write_bytes(data); self.record['binary']['sha256'] = sha(data); self.write_record()
            with self.assertRaises(ValueError): self.verify()


if __name__ == '__main__':
    unittest.main()
