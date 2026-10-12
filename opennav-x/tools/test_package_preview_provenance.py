#!/usr/bin/env python3
"""Execute packaging's CLI and provenance handoffs without native packaging.

Only selected boundary statements run; no DLL loading, source downloads or
product self-test is invoked. Component byte validation has its own fixtures.
"""
import argparse
import ast
import base64
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from msvc_runtime_receipt import parse_visual_studio_catalog_base64

SOURCE = Path(__file__).with_name('package-preview.py')
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'))


def execute(nodes, namespace):
    program = ast.Module(body=nodes, type_ignores=[])
    exec(compile(program, str(SOURCE), 'exec'), namespace)


def assignment(name):
    return next(node for node in TREE.body if isinstance(node, ast.Assign) and
                any(isinstance(target, ast.Name) and target.id == name for target in node.targets))


def call(name):
    return next(node for node in TREE.body if isinstance(node, ast.Expr) and
                isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and
                node.value.func.id == name)


def arguments(extra=(), omit=()):
    nodes = [node for node in TREE.body if
             (isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and
               target.id in ('parser', 'args') for target in node.targets)) or
             (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and
              isinstance(node.value.func, ast.Attribute) and isinstance(node.value.func.value, ast.Name) and
              node.value.func.value.id == 'parser')]
    values = ['package-preview.py', '--install', 'install', '--build', 'build', '--runtime', 'runtime',
              '--runtime-vs-installation', 'vs', '--runtime-vs-version', '17.14.1',
              '--runtime-vs-product-id', 'Microsoft.VisualStudio.Product.BuildTools',
              '--output', 'output', '--openssl-source-cache', 'openssl.tar.gz',
              '--dependency-source-cache', 'dependencies', '--wx-runtime-archive', 'wx.7z', *extra]
    for option in omit:
        index = values.index(option)
        del values[index:index + 2]
    namespace = {'argparse': argparse, 'Path': Path}
    with patch.object(sys, 'argv', values):
        execute(nodes, namespace)
    return namespace['args']


class PackagingProvenanceTests(unittest.TestCase):
    def qualification_boundary(self, actual, directory):
        # Execute the actual producer boundary, without loading any application
        # or invoking packaging's Windows-only startup. This tests what a bad
        # executable self-test can cause the packager to publish.
        guard = next(node for node in TREE.body if isinstance(node, ast.If) and
                     any(isinstance(value, ast.Constant) and value.value == 'update_qualified_config'
                         for value in ast.walk(node.test)))
        writer = next(node for node in TREE.body if isinstance(node, ast.Expr) and
                      any(isinstance(value, ast.Constant) and value.value == 'update-qualification.json'
                          for value in ast.walk(node)))
        execute([guard, writer], {'actual': actual, 'app': directory, 'commit': 'a' * 40,
                                 'hashlib': hashlib, 'json': json})

    def test_qualification_evidence_binds_the_copied_executable(self):
        with tempfile.TemporaryDirectory() as temporary:
            app = Path(temporary)
            executable = b'owned copied executable bytes'
            (app / 'opencpn.exe').write_bytes(executable)
            self.qualification_boundary({key: 1 for key in (
                'update_qualified_config', 'update_qualified_core_output',
                'update_qualified_plugins_deferred')}, app)
            raw = (app / 'update-qualification.json').read_bytes()
            record = json.loads(raw)
            self.assertEqual(record['executableSha256'], hashlib.sha256(executable).hexdigest())
            self.assertEqual(record['commit'], 'a' * 40)
            self.assertEqual(record['pluginPolicy'], 'deferred-until-ordinary-start')
            self.assertEqual(raw, json.dumps(record, separators=(',', ':')).encode('utf-8'))

    def test_unqualified_executable_cannot_publish_evidence(self):
        keys = ('update_qualified_config', 'update_qualified_core_output',
                'update_qualified_plugins_deferred')
        with tempfile.TemporaryDirectory() as temporary:
            app = Path(temporary)
            (app / 'opencpn.exe').write_bytes(b'copied image')
            for key in keys:
                for invalid in (None, False, True, 0, 2, '1', 1.0):
                    with self.subTest(key=key, invalid=invalid):
                        actual = dict.fromkeys(keys, 1)
                        actual[key] = invalid
                        with self.assertRaises(SystemExit):
                            self.qualification_boundary(actual, app)
                        self.assertFalse((app / 'update-qualification.json').exists())
                actual = dict.fromkeys(keys, 1)
                del actual[key]
                with self.assertRaises(SystemExit):
                    self.qualification_boundary(actual, app)
                self.assertFalse((app / 'update-qualification.json').exists())

    def test_required_identity_and_archive_cli(self):
        args = arguments()
        self.assertEqual(parse_visual_studio_catalog_base64(args.runtime_vs_catalog_base64), {})
        for option in ('--runtime-vs-product-id', '--wx-runtime-archive'):
            with self.subTest(option=option), patch.object(sys, 'stderr'), self.assertRaises(SystemExit):
                arguments(omit=[option])

    def test_msvc_catalog_transport_reaches_copy_with_exact_identity(self):
        catalog = {'productName': 'Visual Studio Build Tools', 'productDisplayVersion': '17.14'}
        encoded = base64.b64encode(json.dumps(catalog).encode()).decode()
        args = arguments(['--runtime-vs-catalog-base64', encoded])
        copied = Mock(return_value={'copy': 'verified'})
        execute([assignment('runtime_receipt')], {
            'args': args, 'app': Path('copied-app'), 'copy_msvc_runtime': copied,
            'parse_visual_studio_catalog_base64': parse_visual_studio_catalog_base64})
        copied.assert_called_once_with(args.runtime, Path('copied-app'), args.runtime_vs_installation,
                                       args.runtime_vs_version, args.runtime_vs_product_id, catalog)

    def test_invalid_catalog_cannot_reach_copy(self):
        copied = Mock()
        with self.assertRaises(ValueError):
            execute([assignment('runtime_receipt')], {
                'args': arguments(['--runtime-vs-catalog-base64', 'invalid!']), 'app': Path('copied-app'),
                'copy_msvc_runtime': copied, 'parse_visual_studio_catalog_base64': parse_visual_studio_catalog_base64})
        copied.assert_not_called()

    def test_wx_verifier_receives_copied_app_and_explicit_archive(self):
        verifier = Mock(return_value={'receipt': 'verified'})
        args = arguments()
        execute([assignment('wx_receipt')], {'args': args, 'app': Path('copied-app'),
                'ROOT': Path('source'), 'verify_runtime_receipt': verifier})
        verifier.assert_called_once_with(Path('copied-app'), Path('source/tools/windows-wx.lock.json'), args.wx_runtime_archive)

    def test_archive_and_inventory_share_verified_runtime_descriptors(self):
        source_writer, inventory_writer = Mock(), Mock()
        updater, adapter, wx, msvc = ({'verified': name} for name in ('updater', 'adapter', 'wx', 'msvc'))
        namespace = {'ROOT': Path('source'), 'commit': 'a' * 40, 'args': arguments(),
                     'destination': Path('package'), 'openssl_source': {'sourceBundle': {'openssl': 1}, 'manifest': {'openssl': 2}},
                     'curl_sources': {'sourceBundles': [{'curl': 1}, {'zlib': 1}], 'manifests': {'curl': {}, 'zlib': {}}},
                     'updater_source': updater, 'adapter_sources': [adapter], 'wx_receipt': wx,
                     'runtime_receipt': msvc, 'create_source_archive': source_writer, 'write_inventory': inventory_writer}
        execute([call('create_source_archive'), call('write_inventory')], namespace)
        self.assertEqual(source_writer.call_args.args[3], [{'openssl': 1}, {'curl': 1}, {'zlib': 1}, adapter, updater])
        self.assertEqual(inventory_writer.call_args.kwargs, {
            'verified_runtime_sources': [updater, adapter], 'verified_wx': wx, 'verified_msvc': msvc})


if __name__ == '__main__':
    unittest.main()
