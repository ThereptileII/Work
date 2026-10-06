#!/usr/bin/env python3
"""Explicit Linux private-Staging custody; never a key export or general command runner."""
import argparse
import base64
import contextlib
import fcntl
import hashlib
import json
import os
import re
import resource
import secrets
import stat
import subprocess
import sys

SCHEMA = 'app.skager.PrivateStagingSigning.v1'
SCOPE = 'private-staging'
ROLES = ('root', 'targets', 'snapshot', 'timestamp')
FAILURE = 'Private Staging custody refused; retain repository and keyring state for review. No secret material is logged.'
PIN = re.compile(r'[0-9a-f]{64}\Z')
ORIGIN = re.compile(r'https://(?=.{1,253}\Z)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\.)+'
                    r'[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?\Z')


class Refused(Exception):
    pass


def require(condition):
    if not condition:
        raise Refused(FAILURE)


def absolute(path):
    require(isinstance(path, str) and 0 < len(path) <= 4096 and path.startswith('/') and
            not any(ord(c) < 32 or ord(c) == 127 for c in path) and
            all(part not in ('', '.', '..') for part in path[1:].split('/')))
    return path


def open_plain(path, flags, mode=0o600):
    """Open the selected inode through no-follow directory descriptors."""
    parts = absolute(path)[1:].split('/')
    parent = os.open('/', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
            os.close(parent)
            parent = child
        return os.open(parts[-1], flags | os.O_NOFOLLOW | os.O_NONBLOCK, mode, dir_fd=parent)
    finally:
        os.close(parent)


@contextlib.contextmanager
def custody_lock():
    # A fixed, non-secret lock serializes this helper's inventory/create/use
    # sequence. It contains no keys, payload, repository path or keyring data.
    directory = '/run/user/' + str(os.getuid())
    fd = open_plain(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        info = os.fstat(fd)
        require(info.st_uid == os.getuid() and not info.st_mode & 0o077)
        lock = os.open('skager-private-staging-custody.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW,
                       0o600, dir_fd=fd)
    finally:
        os.close(fd)
    try:
        info = os.fstat(lock)
        require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid() and
                not info.st_mode & 0o077 and info.st_nlink == 1)
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        os.close(lock)


class SecretStore:
    """Only this adapter talks to Secret Service; tests replace it entirely."""
    def __init__(self):
        # Disable GLib/D-Bus diagnostic dumping before importing the binding.
        for name in ('G_DBUS_DEBUG', 'G_MESSAGES_DEBUG', 'LIBSECRET_DEBUG'):
            os.environ.pop(name, None)
        import gi
        gi.require_version('Secret', '1')
        from gi.repository import Gio, GLib, Secret
        self.Secret, self.Gio, self.GLib = Secret, Gio, GLib
        bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)
        self.bus = bus
        owned = bus.call_sync('org.freedesktop.DBus', '/org/freedesktop/DBus',
                              'org.freedesktop.DBus', 'NameHasOwner',
                              GLib.Variant('(s)', ('org.freedesktop.secrets',)),
                              GLib.VariantType.new('(b)'), Gio.DBusCallFlags.NONE, 5000, None)
        require(owned.unpack() == (True,))  # Never intentionally activate a missing store.
        self.schema = Secret.Schema.new(SCHEMA, Secret.SchemaFlags.NONE, {
            'scope': Secret.SchemaAttributeType.STRING,
            'role': Secret.SchemaAttributeType.STRING,
            'repository': Secret.SchemaAttributeType.STRING,
        })
        self.service = Secret.Service.get_sync(Secret.ServiceFlags.OPEN_SESSION, None)
        self.collection = Secret.Collection.for_alias_sync(self.service, 'default',
                                                            Secret.CollectionFlags.NONE, None)
        require(self.collection is not None and not self.collection.get_locked())
        self.items = {}

    def inventory(self):
        require(not self.collection.get_locked())
        # Direct CreateItem does not reliably refresh libsecret's cached Items
        # property in this instance. Read the selected collection's live metadata
        # from its current service owner; never load a secret to prove membership.
        response = self.bus.call_sync(self.service.get_name_owner(), self.collection.get_object_path(),
                                      'org.freedesktop.DBus.Properties', 'Get',
                                      self.GLib.Variant('(ss)', ('org.freedesktop.Secret.Collection', 'Items')),
                                      self.GLib.VariantType.new('(v)'), self.Gio.DBusCallFlags.NO_AUTO_START,
                                      5000, None)
        value = response.get_child_value(0).get_variant()
        require(value.get_type_string() == 'ao')
        paths = value.unpack()
        require(isinstance(paths, list) and len(paths) <= 4096 and
                all(isinstance(path, str) and len(path) <= 1024 and
                    re.fullmatch(r'(?:/[A-Za-z0-9_]+)+', path) for path in paths))
        members = set(paths)
        require(len(members) == len(paths))
        # ALL includes locked and duplicate records, but neither unlocks nor
        # loads secret values. Search all collections to detect misplaced copies.
        found = self.service.search_sync(self.schema, {'scope': SCOPE},
                                          self.Secret.SearchFlags.ALL, None)
        result = []
        self.items = {}
        for item in found:
            identity = item.get_object_path()
            require(identity in members)
            attrs = item.get_attributes()
            require(attrs.pop('xdg:schema', SCHEMA) == SCHEMA)
            result.append((identity, attrs, item.get_locked()))
            self.items[identity] = item
        return result

    def create(self, attrs, encoded):
        require(not self.collection.get_locked())
        value = self.Secret.Value.new(encoded, len(encoded), 'text/plain')
        properties = {
            'org.freedesktop.Secret.Item.Label': self.GLib.Variant('s', 'SKAGER private Staging ' + attrs['role']),
            'org.freedesktop.Secret.Item.Attributes': self.GLib.Variant('a{ss}', dict(attrs, **{'xdg:schema': SCHEMA})),
        }
        # libsecret's high-level create_sync may automatically run a prompt.
        # Use its negotiated secret encoding, but refuse a prompt
        # response instead of unlocking or authorizing anything implicitly.
        secret = self.service.encode_dbus_secret(value)
        require(secret is not None)
        parameters = self.GLib.Variant('(a{sv}(oayays)b)', (properties, secret.unpack(), False))
        response = self.bus.call_sync(self.service.get_name_owner(), self.collection.get_object_path(),
                                      'org.freedesktop.Secret.Collection', 'CreateItem', parameters,
                                      self.GLib.VariantType.new('(oo)'), self.Gio.DBusCallFlags.NO_AUTO_START,
                                      5000, None)
        identity, prompt = response.unpack()
        require(prompt == '/' and isinstance(identity, str) and identity.startswith('/') and identity != '/')
        return identity

    def load(self, identity):
        item = self.items[identity]
        require(not self.collection.get_locked() and not item.get_locked())
        require(item.load_secret_sync(None))
        value = item.get_secret()
        require(value is not None)
        return value.get_text()


class PinnedTool:
    """Hash and execute the same selected ELF inode, without a shell or inherited loader environment."""
    def __init__(self, path, pin):
        require(PIN.fullmatch(pin))
        self.fd = open_plain(path, os.O_RDONLY)
        try:
            before = os.fstat(self.fd)
            require(stat.S_ISREG(before.st_mode) and before.st_uid in (0, os.getuid()) and
                    not before.st_mode & 0o022 and before.st_mode & 0o111 and
                    before.st_nlink == 1 and 0 < before.st_size <= 256 << 20)
            require(os.read(self.fd, 4) == b'\x7fELF')
            os.lseek(self.fd, 0, os.SEEK_SET)
            digest = hashlib.sha256()
            while chunk := os.read(self.fd, 128 << 10):
                digest.update(chunk)
            after = os.fstat(self.fd)
            require((before.st_size, before.st_mtime_ns, before.st_ctime_ns) ==
                    (after.st_size, after.st_mtime_ns, after.st_ctime_ns) and digest.hexdigest() == pin)
        except BaseException:
            os.close(self.fd)
            raise

    def close(self):
        os.close(self.fd)

    def run(self, args, keys):
        payload = json.dumps({'schema': 1, 'keys': keys}, separators=(',', ':')).encode('ascii')
        require(len(payload) <= 16384)
        try:
            result = subprocess.run(['skager-repository'] + args, executable='/proc/self/fd/' + str(self.fd),
                                    pass_fds=(self.fd,), input=payload, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, timeout=120, check=False, close_fds=True,
                                    cwd='/', env={'PATH': '/usr/bin:/bin', 'LANG': 'C', 'LC_ALL': 'C'})
            require(result.returncode == 0)
        except Exception:
            raise Refused(FAILURE) from None
        finally:
            # Python/GI/Go may retain immutable copies; no complete zeroization claim.
            del payload


def attributes(role, repository):
    return {'scope': SCOPE, 'role': role,
            'repository': hashlib.sha256(repository.encode('utf-8')).hexdigest()}


def checked_inventory(store, repository):
    found = store.inventory()
    require(len(found) == 4)
    roles = {}
    identities = set()
    for identity, attrs, locked in found:
        require(isinstance(attrs, dict) and isinstance(identity, str) and identity and type(locked) is bool)
        role = attrs.get('role')
        require(not locked and role in ROLES and role not in roles and identity not in identities and
                attrs == attributes(role, repository))
        roles[role] = identity
        identities.add(identity)
    return roles


def seed_text(text):
    require(isinstance(text, str) and len(text) == 44)
    try:
        raw = base64.b64decode(text, validate=True)
    except Exception:
        raise Refused(FAILURE) from None
    require(len(raw) == 32 and base64.b64encode(raw).decode('ascii') == text)
    return text


def delegate(args):
    repository = absolute(args.repository)
    common = ['--repository', repository]
    if args.command == 'initialize':
        require(ORIGIN.fullmatch(args.origin))
        require(not os.path.lexists(repository) and os.path.isdir(os.path.dirname(repository)))
        parent = open_plain(os.path.dirname(repository), os.O_RDONLY | os.O_DIRECTORY)
        os.close(parent)
        return ['initialize'] + common + ['--metadata-url', args.origin, '--artifact-origin', args.origin, '--keys-stdin']
    require(args.command == 'publish' and os.path.isdir(repository) and PIN.fullmatch(args.selection_sha256))
    existing = open_plain(repository, os.O_RDONLY | os.O_DIRECTORY)
    os.close(existing)
    return ['publish'] + common + ['--assets', absolute(args.assets), '--policy', absolute(args.policy),
                                  '--selection', absolute(args.selection), '--selection-sha256',
                                  args.selection_sha256, '--keys-stdin']


def operate(args, store, tool, generate=secrets.token_bytes):
    command = delegate(args)  # Constrained args and fresh/existing repository before any write.
    created = None
    if args.command == 'initialize':
        require(not store.inventory())  # No overwrite, resume, partial-set repair or duplicate acceptance.
        seeds = [generate(32) for _ in ROLES]
        require(all(isinstance(seed, bytes) and len(seed) == 32 for seed in seeds) and len(set(seeds)) == 4)
        created = {}
        for role, seed in zip(ROLES, seeds):
            created[role] = store.create(attributes(role, args.repository), base64.b64encode(seed).decode('ascii'))
        del seeds
    roles = checked_inventory(store, args.repository)
    if created is not None:
        require(roles == created)
    wanted = ROLES if args.command == 'initialize' else ROLES[1:]
    keys = {role: seed_text(store.load(roles[role])) for role in wanted}
    require(len(set(keys.values())) == len(wanted))
    try:
        tool.run(command, keys)
    finally:
        keys.clear()


def main(argv=None):
    # Parser failures never echo a supplied value that might accidentally contain a secret.
    class Parser(argparse.ArgumentParser):
        def error(self, message):
            raise Refused(FAILURE)
    parser = Parser(description=__doc__)
    parser.add_argument('--tool', required=True)
    parser.add_argument('--tool-sha256', required=True)
    commands = parser.add_subparsers(dest='command', required=True, parser_class=Parser)
    initialize = commands.add_parser('initialize')
    initialize.add_argument('--repository', required=True)
    initialize.add_argument('--origin', required=True)
    publish = commands.add_parser('publish')
    for option in ('repository', 'assets', 'policy', 'selection', 'selection-sha256'):
        publish.add_argument('--' + option, required=True)
    try:
        require(sys.platform == 'linux')
        args = parser.parse_args(argv)
        delegate(args)
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        with contextlib.closing(PinnedTool(args.tool, args.tool_sha256)) as tool, custody_lock():
            operate(args, SecretStore(), tool)
        print('Private Staging repository ' + ('initialized' if args.command == 'initialize' else 'published') + '.')
        return 0
    except KeyboardInterrupt:
        print(FAILURE, file=sys.stderr)
        return 1
    except Exception:
        print(FAILURE, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
