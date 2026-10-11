#!/usr/bin/env python3
"""Disposable filesystem fixtures using the real C++ import reader."""
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

import win32_plugin_inventory as api


def pe(normal=(), delayed=()):
    data = bytearray(0x1200)
    def put(at, value, size=4):
        struct.pack_into('<H' if size == 2 else '<I', data, at, value)
    for at, value, size in ((0, 0x5a4d, 2), (60, 0x80, 4), (0x80, 0x4550, 4),
            (0x84, 0x14c, 2), (0x86, 1, 2), (0x94, 224, 2), (0x96, 0x2102, 2),
            (0x98, 0x10b, 2), (0xb4, 0x400000, 4), (0xd4, 0x200, 4),
            (0xf4, 16, 4), (0x180, 0x1000, 4), (0x184, 0x1000, 4),
            (0x188, 0x1000, 4), (0x18c, 0x200, 4)):
        put(at, value, size)
    for names, entry, table, raw, width, name_raw, name_rva in (
            (normal, 0x100, 0x1100, 0x300, 20, 0x700, 0x1500),
            (delayed, 0x160, 0x1200, 0x400, 32, 0xb00, 0x1900)):
        if not names:
            continue
        put(entry, table); put(entry + 4, width * (len(names) + 1))
        for i, name in enumerate(names):
            put(raw + width * i, 1 if width == 32 else 0x1e00)
            put(raw + width * i + (4 if width == 32 else 12), name_rva + i * 0x80)
            encoded = name.encode('ascii') + b'\0'
            data[name_raw + i * 0x80:name_raw + i * 0x80 + len(encoded)] = encoded
    return bytes(data)


class InventoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reader_context = api.compiled_reader()
        cls.reader = staticmethod(cls.reader_context.__enter__())

    @classmethod
    def tearDownClass(cls):
        cls.reader_context.__exit__(None, None, None)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='skager-inventory-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)

    def file(self, name, normal=(), delayed=(), data=None):
        path = self.base / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(pe(normal, delayed) if data is None else data)
        return path

    def run_inventory(self, roots, directories=(), **kwargs):
        before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.base.rglob('*') if p.is_file() and not p.is_symlink()}
        report = api.inventory(roots, directories, self.reader, **kwargs)
        after = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in self.base.rglob('*') if p.is_file() and not p.is_symlink()}
        self.assertEqual(before, after, 'Inventory wrote an inspected file')
        self.assertFalse(report['runtimeEnforcement'])
        self.assertFalse(report['loaderClosureAccepted'])
        self.assertTrue(report['limits'])
        self.assertEqual(json.loads(json.dumps(report)), report)
        return report

    def test_recursive_normal_delay_cycle_and_exact_identity(self):
        root = self.file('external Unicode å/plugin_pi.dll', ['LIBCURL.dll'], ['optional.drv'])
        curl = self.file('app/libcurl.dll', ['zlib1.dll'])
        zlib = self.file('app/zlib1.dll', ['libcurl.dll'])
        optional = self.file('external Unicode å/optional.drv')
        report = self.run_inventory([root], [curl.parent])
        self.assertEqual(report['status'], 'inventory-observed')
        self.assertEqual(len(report['nodes']), 4)
        self.assertEqual(len(report['edges']), 4)
        self.assertEqual({edge['kind'] for edge in report['edges']}, {'normal', 'delay'})
        for node in report['nodes']:
            path = Path(node['path'])
            self.assertIn(path, (root, curl, zlib, optional))
            self.assertEqual(node['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            self.assertEqual(node['bytes'], path.stat().st_size)
        self.assertIsNotNone(report['readerIdentity'])

    def test_ambiguity_does_not_select_or_hide_candidate_graphs(self):
        root = self.file('plugins/a_pi.dll', ['helper.dll'])
        a = self.file('first/helper.dll', ['missing-a.dll'])
        b = self.file('second/helper.dll', ['missing-b.dll'])
        report = self.run_inventory([root], [b.parent, a.parent])
        edge = report['edges'][0]
        self.assertEqual(edge['status'], 'ambiguous')
        self.assertEqual(edge['candidates'], sorted(map(str, (a, b))))
        self.assertEqual({node['path'] for node in report['nodes']}, set(map(str, (root, a, b))))
        self.assertEqual({e['name'] for e in report['edges'][1:]}, {'missing-a.dll', 'missing-b.dll'})
        self.assertEqual(report['status'], 'inventory-incomplete')

    def test_missing_system_api_set_and_dynamic_limits_explicit(self):
        root = self.file('a_pi.dll', ['kernel32.dll', 'api-ms-win-core-file-l1-1-0.dll'], ['libeay32.dll'])
        report = self.run_inventory([root])
        self.assertEqual([e['status'] for e in report['edges']], ['unresolved'] * 3)
        self.assertTrue(any('Dynamic' in note for note in report['limits']))

    def test_path_shaped_names_do_not_escape_or_read(self):
        root = self.file('a_pi.dll', ['../escape.dll', 'C:escape.dll'], ['bad.dll.'])
        self.file('escape.dll')
        report = self.run_inventory([root])
        self.assertEqual(len(report['nodes']), 1)
        self.assertEqual([e['status'] for e in report['edges']], ['unsupported-name'] * 3)

    def test_malformed_dependency_keeps_its_hash(self):
        root = self.file('a_pi.dll', ['broken.dll'])
        broken = self.file('broken.dll', data=b'MZ')
        report = self.run_inventory([root])
        node = next(n for n in report['nodes'] if n['path'] == str(broken))
        self.assertFalse(node['parsed'])
        self.assertEqual(node['sha256'], hashlib.sha256(b'MZ').hexdigest())
        self.assertEqual(report['status'], 'inventory-incomplete')

    def test_foreign_dependency_keeps_identity_but_is_not_parsed(self):
        root = self.file('a_pi.dll', ['foreign.dll'])
        data = bytearray(pe()); struct.pack_into('<H', data, 0x84, 0x8664)
        foreign = self.file('foreign.dll', data=data)
        report = self.run_inventory([root])
        node = next(n for n in report['nodes'] if n['path'] == str(foreign))
        self.assertFalse(node['parsed'])
        self.assertEqual(node['sha256'], hashlib.sha256(data).hexdigest())
        self.assertIn('Win32 x86 required', node['error'])

    def test_missing_root_and_network_paths_are_explicit(self):
        report = self.run_inventory([self.base / 'missing.dll'])
        self.assertEqual(report['status'], 'inventory-incomplete')
        self.assertTrue(report['nodes'][0]['error'])
        for value in ('\\\\server\\share\\plugin.dll', '//server/share/plugin.dll'):
            with self.assertRaisesRegex(ValueError, 'Network/device'):
                api.plain_path(value)

    def test_identical_candidate_bytes_still_ambiguous_paths(self):
        root = self.file('a_pi.dll', ['helper.dll'])
        a = self.file('one/helper.dll'); b = self.file('two/helper.dll')
        report = self.run_inventory([root], [a.parent, b.parent])
        self.assertEqual(report['edges'][0]['status'], 'ambiguous')

    def test_unreadable_search_cannot_be_called_resolved(self):
        root = self.file('a_pi.dll', ['helper.dll']); self.file('helper.dll')
        absent = self.base / 'missing-directory'
        report = self.run_inventory([root], [absent])
        self.assertEqual(report['edges'][0]['status'], 'unavailable-search')
        self.assertTrue(report['edges'][0]['errors'])
        self.assertEqual(len(report['nodes']), 2)

    def test_links_refused(self):
        target = self.file('real/helper.dll')
        link = self.base / 'helper.dll'
        try:
            link.symlink_to(target)
        except OSError as error:
            self.skipTest('Symlink creation unavailable: ' + str(error))
        root = self.file('a_pi.dll', ['helper.dll'])
        report = self.run_inventory([root])
        self.assertIn('Symlink/reparse', report['nodes'][1]['error'])

    def test_budgets_are_incomplete_not_silent_truncation(self):
        root = self.file('a_pi.dll', ['one.dll', 'two.dll']); self.file('one.dll'); self.file('two.dll')
        for limits in ({'max_nodes': 1}, {'max_edges': 1}, {'max_bytes': 4608}, {'max_entries': 1}):
            with self.subTest(limits=limits):
                report = self.run_inventory([root], **limits)
                self.assertEqual(report['status'], 'inventory-incomplete')
        with self.assertRaises(ValueError):
            self.run_inventory([root], max_nodes=0)

    def test_case_collisions_detected_on_case_sensitive_filesystem(self):
        root = self.file('a_pi.dll', ['HELPER.dll'])
        a = self.file('Helper.dll'); b = self.file('helper.dll')
        if a.samefile(b):
            self.skipTest('Filesystem is case insensitive')
        report = self.run_inventory([root])
        self.assertEqual(report['edges'][0]['status'], 'ambiguous')

    def test_changes_after_read_reported_with_original_captured_hash(self):
        root = self.file('a_pi.dll')
        original = root.read_bytes()
        def change_after_read(data):
            self.assertEqual(data, original)
            root.write_bytes(original + b'changed')
            return self.reader(data)
        report = api.inventory([root], [], change_after_read)
        self.assertEqual(report['nodes'][0]['sha256'], hashlib.sha256(original).hexdigest())
        self.assertEqual(report['status'], 'inventory-incomplete')
        self.assertTrue(any('changed after' in error for error in report['errors']))

    def test_directory_changes_after_lookup_are_reported(self):
        root = self.file('a_pi.dll', ['helper.dll']); helper = self.file('helper.dll')
        calls = 0
        def change_after_lookup(data):
            nonlocal calls
            calls += 1
            if calls == 2:
                self.file('new-sibling.dll')
            return self.reader(data)
        report = api.inventory([root], [], change_after_lookup)
        self.assertEqual(calls, 2)
        self.assertEqual(report['status'], 'inventory-incomplete')
        self.assertTrue(any('Directory changed after' in error for error in report['errors']))
        self.assertEqual({n['path'] for n in report['nodes']}, {str(root), str(helper)})

    def test_cli_json_and_exit_status(self):
        root = self.file('a_pi.dll', ['missing.dll'])
        done = subprocess.run([sys.executable, str(api.ROOT / 'tools/win32_plugin_inventory.py'), '--root', str(root)],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False, timeout=60)
        self.assertEqual(done.returncode, 1, done.stderr.decode())
        self.assertEqual(json.loads(done.stdout)['status'], 'inventory-incomplete')


if __name__ == '__main__':
    unittest.main()
