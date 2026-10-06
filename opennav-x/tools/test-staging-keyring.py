#!/usr/bin/env python3
"""Fake custody/process boundaries only: no real store, key creation or subprocess."""
import argparse
import base64
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('staging_keyring', Path(__file__).with_name('staging-keyring.py'))
keyring = importlib.util.module_from_spec(spec)
spec.loader.exec_module(keyring)


def encoded(n):
    return base64.b64encode(bytes([n]) * 32).decode('ascii')


class Store:
    def __init__(self, repo=None):
        self.records = [] if repo is None else [
            (str(i), keyring.attributes(role, repo), False) for i, role in enumerate(keyring.ROLES)]
        self.values = {str(i): encoded(i + 1) for i in range(4)} if repo else {}
        self.loads = []
        self.creates = []
        self.locked = False
        self.fail_create = None

    def inventory(self):
        keyring.require(not self.locked)
        return copy.deepcopy(self.records)

    def create(self, attrs, value):
        if len(self.creates) == self.fail_create:
            raise RuntimeError('backend error containing SECRET-MUST-NOT-PRINT')
        identity = str(len(self.records))
        self.creates.append((dict(attrs), value))
        self.records.append((identity, dict(attrs), False))
        self.values[identity] = value
        return identity

    def load(self, identity):
        self.loads.append(identity)
        return self.values[identity]


class Tool:
    def __init__(self):
        self.calls = []
        self.fail = False

    def run(self, args, keys):
        self.calls.append((list(args), dict(keys)))
        if self.fail:
            raise RuntimeError('SECRET-MUST-NOT-PRINT')

    def close(self):
        pass


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = str(Path(self.temp.name) / 'repository')
        self.init = argparse.Namespace(command='initialize', repository=self.repo, origin='https://selected.example.test')
        self.publish = argparse.Namespace(command='publish', repository=self.repo, assets=self.temp.name,
            policy=str(Path(self.temp.name) / 'policy.json'), selection=str(Path(self.temp.name) / 'selection.json'),
            selection_sha256='a' * 64)
        self.tool = Tool()

    def publish_store(self):
        Path(self.repo).mkdir(exist_ok=True)
        return Store(self.repo)

    def test_initialize_creates_distinct_fixed_scope_then_pipes_strict_roles(self):
        store = Store()
        counter = iter(range(1, 5))
        keyring.operate(self.init, store, self.tool, lambda size: bytes([next(counter)]) * size)
        self.assertEqual(len(store.creates), 4)
        self.assertEqual(store.loads, ['0', '1', '2', '3'])
        self.assertEqual(len(set(store.values.values())), 4)
        args, keys = self.tool.calls[0]
        self.assertEqual(set(keys), set(keyring.ROLES))
        self.assertEqual(args, ['initialize', '--repository', self.repo, '--metadata-url', self.init.origin,
                                '--artifact-origin', self.init.origin, '--keys-stdin'])
        self.assertNotIn('--keys-file', args)
        for attrs, value in store.creates:
            self.assertEqual(attrs, keyring.attributes(attrs['role'], self.repo))
            self.assertNotIn(value, repr(args))
        with self.assertRaises(keyring.Refused):
            keyring.operate(self.init, store, self.tool)
        self.assertEqual(len(store.creates), 4)

    def test_publish_does_not_load_root_or_create_keys(self):
        store = self.publish_store()
        store.values['0'] = 'root-secret-must-never-be-read'
        keyring.operate(self.publish, store, self.tool, lambda _: self.fail('Generated during publish'))
        self.assertEqual(store.loads, ['1', '2', '3'])
        self.assertEqual(store.creates, [])
        args, keys = self.tool.calls[0]
        self.assertEqual(set(keys), {'targets', 'snapshot', 'timestamp'})
        self.assertNotIn('root-secret', repr((args, keys)))
        self.assertEqual(args[-3:], ['--selection-sha256', 'a' * 64, '--keys-stdin'])

    def test_locked_duplicate_partial_misbound_and_unknown_state_rejected_before_load(self):
        store = self.publish_store()
        variants = []
        for count in [0, 1, 3]:
            broken = copy.deepcopy(store)
            broken.records = broken.records[:count]
            variants.append(broken)
        broken = copy.deepcopy(store); broken.records.append(broken.records[0]); variants.append(broken)
        broken = copy.deepcopy(store); broken.locked = True; variants.append(broken)
        for field, value in [('scope', 'production'), ('repository', 'f' * 64), ('role', 'unknown'), ('extra', 'unapproved')]:
            broken = copy.deepcopy(store); broken.records[1][1][field] = value; variants.append(broken)
        broken = copy.deepcopy(store); identity, attrs, _ = broken.records[0]
        broken.records[0] = (identity, attrs, True); variants.append(broken)
        broken = copy.deepcopy(store); broken.records[1] = copy.deepcopy(broken.records[0]); variants.append(broken)
        for broken in variants:
            with self.assertRaises(keyring.Refused):
                keyring.operate(self.publish, broken, self.tool)
            self.assertEqual(broken.loads, [])
        self.assertEqual(self.tool.calls, [])

    def test_bad_or_duplicate_seed_and_failed_creation_retain_state(self):
        for seed in ['not-base64', base64.b64encode(b'x' * 31).decode(), encoded(1) + '\n', encoded(1)[:-1] + '!']:
            store = self.publish_store(); store.values['1'] = seed
            with self.assertRaises(keyring.Refused):
                keyring.operate(self.publish, store, self.tool)
        store = self.publish_store(); store.values['1'] = store.values['2']
        with self.assertRaises(keyring.Refused):
            keyring.operate(self.publish, store, self.tool)
        Path(self.repo).rmdir()
        store = Store()
        with self.assertRaises(keyring.Refused):
            keyring.operate(self.init, store, self.tool, lambda size: b'x' * size)
        self.assertEqual(store.creates, [])
        store.fail_create = 2
        counter = iter(range(1, 5))
        with self.assertRaises(RuntimeError):
            keyring.operate(self.init, store, self.tool, lambda size: bytes([next(counter)]) * size)
        self.assertEqual(len(store.records), 2)
        with self.assertRaises(keyring.Refused):
            keyring.operate(self.init, store, self.tool)
        self.assertEqual(len(store.records), 2)
        self.assertEqual(self.tool.calls, [])

    def test_constrained_args_and_existing_repository_refuse_before_key_writes(self):
        for origin in ['http://host.test', 'https://user:pass@host.test', 'https://host.test?secret=foo',
                       'https://host.test/#fragment', 'https://host.test/path', 'https://host.test:443']:
            args = copy.copy(self.init); args.origin = origin
            store = Store()
            with self.assertRaises(keyring.Refused):
                keyring.operate(args, store, self.tool)
            self.assertEqual(store.creates, [])
        Path(self.repo).mkdir()
        store = Store()
        with self.assertRaises(keyring.Refused):
            keyring.operate(self.init, store, self.tool)
        self.assertEqual(store.creates, [])

    def test_pinned_inode_envelope_environment_and_redacted_child_failure(self):
        binary = Path(self.temp.name) / 'reviewed-tool'
        binary.write_bytes(b'\x7fELF' + b'inert fixture; never executed')
        binary.chmod(0o500)
        digest = hashlib.sha256(binary.read_bytes()).hexdigest()
        with contextlib.closing(keyring.PinnedTool(str(binary), digest)) as tool:
            with patch.object(keyring.subprocess, 'run', return_value=SimpleNamespace(returncode=0)) as run:
                online = {role: encoded(i + 1) for i, role in enumerate(keyring.ROLES[1:])}
                tool.run(['publish', '--repository', self.repo, '--keys-stdin'], online)
                argv = run.call_args.args[0]; kwargs = run.call_args.kwargs
                self.assertEqual(json.loads(kwargs['input']), {'schema': 1, 'keys': online})
                self.assertEqual(kwargs['executable'], '/proc/self/fd/' + str(tool.fd))
                self.assertEqual(kwargs['pass_fds'], (tool.fd,))
                self.assertEqual(kwargs['stdout'], subprocess.DEVNULL)
                self.assertEqual(kwargs['stderr'], subprocess.DEVNULL)
                self.assertNotIn('shell', kwargs)
                self.assertEqual(set(kwargs['env']), {'PATH', 'LANG', 'LC_ALL'})
                for seed in online.values():
                    self.assertNotIn(seed, repr(argv) + repr(kwargs['env']))
            for result in [SimpleNamespace(returncode=1), subprocess.TimeoutExpired('SECRET-MUST-NOT-PRINT', 120)]:
                with patch.object(keyring.subprocess, 'run', **({'side_effect': result} if isinstance(result, Exception) else {'return_value': result})):
                    with self.assertRaises(keyring.Refused) as caught:
                        tool.run(['publish'], online)
                    self.assertEqual(str(caught.exception), keyring.FAILURE)
        with self.assertRaises(keyring.Refused):
            keyring.PinnedTool(str(binary), '0' * 64)
        Path(self.repo).mkdir()
        argv = ['--tool', str(binary), '--tool-sha256', '0' * 64, 'publish', '--repository', self.repo,
                '--assets', self.temp.name, '--policy', self.publish.policy, '--selection', self.publish.selection,
                '--selection-sha256', self.publish.selection_sha256]
        with patch.object(keyring, 'SecretStore') as store, patch.object(keyring.resource, 'setrlimit'), \
             contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(keyring.main(argv), 1)
            store.assert_not_called()
            self.assertEqual(err.getvalue(), keyring.FAILURE + '\n')
        link = Path(self.temp.name) / 'linked'; link.symlink_to(binary)
        with self.assertRaises(OSError):
            keyring.PinnedTool(str(link), digest)
        binary.chmod(0o777)
        with self.assertRaises(keyring.Refused):
            keyring.PinnedTool(str(binary), digest)

    def test_secret_adapter_inventory_never_loads_and_creation_never_replaces_or_prompts(self):
        class Variant:
            def __init__(self, signature, value):
                self.signature, self.value = signature, value
            def unpack(self):
                return self.value
            def get_child_value(self, index):
                return self.value[index]
            def get_variant(self):
                return self
            def get_type_string(self):
                return self.signature
        adapter = object.__new__(keyring.SecretStore)
        flag = object()
        items = [SimpleNamespace(get_object_path=lambda i=i: '/items/' + str(i),
                    get_attributes=lambda role=role: dict(keyring.attributes(role, self.repo), **{'xdg:schema': keyring.SCHEMA}),
                    get_locked=lambda: False) for i, role in enumerate(keyring.ROLES)]
        searches = []
        def search(schema, attrs, flags, cancellable):
            searches.append((schema, attrs, flags, cancellable))
            return items
        adapter.schema = object()
        adapter.collection = SimpleNamespace(get_locked=lambda: False,
            load_items_sync=lambda _: self.fail('Cached collection load used'),
            get_items=lambda: self.fail('Cached collection membership used'),
            get_object_path=lambda: '/collection/default')
        adapter.Secret = SimpleNamespace(SearchFlags=SimpleNamespace(ALL=flag),
            Value=SimpleNamespace(new=lambda text, length, kind: (text, length, kind)))
        adapter.service = SimpleNamespace(search_sync=search,
            encode_dbus_secret=lambda value: Variant('(oayays)', ('/session/test', [], [1, 2, 3], 'text/plain')),
            get_name_owner=lambda: ':1.123')
        adapter.GLib = SimpleNamespace(Variant=Variant, VariantType=SimpleNamespace(new=lambda value: value))
        no_start = object()
        adapter.Gio = SimpleNamespace(DBusCallFlags=SimpleNamespace(NO_AUTO_START=no_start))
        metadata_calls = []
        def metadata(*args):
            metadata_calls.append(args)
            return Variant('(v)', (Variant('ao', [item.get_object_path() for item in items]),))
        adapter.bus = SimpleNamespace(call_sync=metadata)
        # Model direct creation followed by a stale/empty libsecret collection
        # cache: the four newly created paths exist only in the direct response.
        records = adapter.inventory()
        self.assertEqual(len(records), 4)
        self.assertEqual(searches, [(adapter.schema, {'scope': keyring.SCOPE}, flag, None)])
        self.assertEqual(metadata_calls[0][:4], (':1.123', '/collection/default',
                                               'org.freedesktop.DBus.Properties', 'Get'))
        self.assertEqual(metadata_calls[0][4].unpack(), ('org.freedesktop.Secret.Collection', 'Items'))
        self.assertEqual(metadata_calls[0][5:], ('(v)', no_start, 5000, None))
        for signature, paths in [('ao', []), ('as', ['/items/0']), ('ao', ['/items/0'] * 2),
                                 ('ao', ['/malformed-path']), ('ao', ['/items/0'] * 4097)]:
            adapter.bus.call_sync = lambda *args, signature=signature, paths=paths: Variant('(v)', (Variant(signature, paths),))
            with self.assertRaises(keyring.Refused):
                adapter.inventory()
        # A fresh metadata response recovers membership without recreating or
        # overwriting records, while duplicate role checks remain fail-closed.
        adapter.bus.call_sync = metadata
        self.assertEqual(len(keyring.checked_inventory(adapter, self.repo)), 4)
        items.append(items[0])
        with self.assertRaises(keyring.Refused):
            keyring.checked_inventory(adapter, self.repo)
        items.pop()
        calls = []
        def create(*args):
            calls.append(args)
            return Variant('(oo)', ('/items/new', '/'))
        adapter.bus = SimpleNamespace(call_sync=create)
        self.assertEqual(adapter.create(keyring.attributes('root', self.repo), encoded(1)), '/items/new')
        args = calls[0]
        self.assertEqual(args[:4], (':1.123', '/collection/default', 'org.freedesktop.Secret.Collection', 'CreateItem'))
        properties, _, replace = args[4].unpack()
        self.assertIs(replace, False)
        self.assertEqual(properties['org.freedesktop.Secret.Item.Attributes'].unpack()['xdg:schema'], keyring.SCHEMA)
        self.assertIs(args[6], no_start)
        adapter.bus.call_sync = lambda *args: Variant('(oo)', ('/', '/prompt/requires-unlock'))
        with self.assertRaises(keyring.Refused):
            adapter.create(keyring.attributes('root', self.repo), encoded(1))

    def test_main_never_prints_backend_errors_and_failed_child_preserves_keys(self):
        store = self.publish_store()
        self.tool.fail = True
        with self.assertRaises(RuntimeError):
            keyring.operate(self.publish, store, self.tool)
        self.assertEqual(len(store.records), 4)
        args = ['--tool', '/reviewed/tool', '--tool-sha256', 'f' * 64, 'publish', '--repository', self.repo,
                '--assets', self.temp.name, '--policy', self.publish.policy, '--selection', self.publish.selection,
                '--selection-sha256', self.publish.selection_sha256]
        for argv in [args, args + ['--keys-file', 'SECRET-MUST-NOT-PRINT']]:
            with patch.object(keyring, 'SecretStore', side_effect=RuntimeError('SECRET-MUST-NOT-PRINT')), \
                 patch.object(keyring, 'PinnedTool', return_value=self.tool), \
                 patch.object(keyring, 'custody_lock', return_value=contextlib.nullcontext()), \
                 patch.object(keyring.resource, 'setrlimit'), \
                 contextlib.redirect_stderr(io.StringIO()) as err, contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(keyring.main(argv), 1)
                self.assertEqual(err.getvalue(), keyring.FAILURE + '\n')
                self.assertEqual(out.getvalue(), '')


if __name__ == '__main__':
    unittest.main()
