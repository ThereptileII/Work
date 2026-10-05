#!/usr/bin/env python3
"""Disposable copied-artifact verification; no external process or full build."""
import base64
import copy
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest import mock
import zipfile

from updater_package import copy_verified_updater_package, verify_updater_package, GO_VERSION, MAIN_MODULE, SOURCE_RELATIVE, BUNDLE_PATH
from updater_package import (select_update_trust, copy_selected_update_trust, validate_public_update_trust,
                             TRUST_SOURCE, STOCK_SHA256, UPSTREAM_COMMIT)


def public_config():
    # Inert structural fixture. Crypto is tested by the real Go validator suite;
    # these tests exercise the trusted subprocess handoff, source and copy binding.
    roles = dict(zip(('root', 'targets', 'snapshot', 'timestamp'), ('1'*64, '2'*64, '3'*64, '4'*64)))
    return {'schema': 1, 'channel': 'beta', 'metadataUrl': 'https://updates.example.test/beta/metadata',
            'artifactOrigin': 'https://updates.example.test',
            'localCompatibilityAllowlist': [{'version': '5.12.4', 'arch': 'x86',
                'executableSha256': STOCK_SHA256, 'upstreamCommit': UPSTREAM_COMMIT}],
            'bootstrapRoot': {'signed': {'_type': 'root', 'spec_version': '1.0.31', 'version': 1,
                'expires': '2030-01-01T00:00:00Z', 'consistent_snapshot': True,
                'keys': {key: {'keytype': 'ed25519', 'scheme': 'ed25519', 'keyval': {'public': key}} for key in roles.values()},
                'roles': {name: {'keyids': [key], 'threshold': 1} for name, key in roles.items()}},
                'signatures': [{'keyid': roles['root'], 'sig': '0'*128}]}}


class UpdaterPackageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='skager-updater-package-')
        self.addCleanup(self.temp.cleanup)
        self.app = Path(self.temp.name)
        self.commit = 'a' * 40
        self.archive = self.app / SOURCE_RELATIVE
        self.archive.parent.mkdir(parents=True)
        with zipfile.ZipFile(self.archive, 'x') as source:
            source.writestr('LICENSE', b'Original upstream license\r\n')
        self.binary = self.app / 'skager-start.exe'
        pe = bytearray(128)
        pe[:2] = b'MZ'
        struct.pack_into('<I', pe, 60, 64)
        pe[64:68] = b'PE\0\0'
        struct.pack_into('<H', pe, 68, 0x14c)
        struct.pack_into('<H', pe, 88, 0x10b)
        self.binary.write_bytes(pe)
        checksum = 'h1:' + base64.b64encode(bytes(32)).decode()
        self.record = {
            'schema': 1, 'productCommit': self.commit, 'goVersion': GO_VERSION,
            'binary': {'path': 'skager-start.exe', 'sha256': self.sha(self.binary)},
            'sourceBundle': {'archive': SOURCE_RELATIVE, 'path': BUNDLE_PATH, 'sha256': self.sha(self.archive),
                             'reference': {'schema': 1, 'productCommit': self.commit, 'goVersion': GO_VERSION,
                                           'mainModule': 'tools/update-verifier',
                                           'modules': [{'path': 'public.example/module', 'version': 'v1.0.0',
                                                        'sum': checksum, 'goModSum': checksum,
                                                        'sourcePrefix': 'modules/public.example/module@v1.0.0'}],
                                           'build': 'CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build -mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-start'}},
            'buildInfo': f'skager-start.exe: {GO_VERSION}\n\tpath\t{MAIN_MODULE}/cmd/skager-start\n\tbuild\t-trimpath=true\n\tbuild\tCGO_ENABLED=0\n\tbuild\tGOARCH=386\n\tbuild\tGOOS=windows\n\tbuild\tvcs.revision={self.commit}\n\tbuild\tvcs.modified=false\n',
        }
        self.record_file = self.archive.parent / 'build.json'
        self.write()

    @staticmethod
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write(self, record=None):
        self.record_file.write_text(json.dumps(record or self.record), encoding='utf-8')

    def destination(self):
        directory = tempfile.TemporaryDirectory(prefix='skager-updater-destination-')
        self.addCleanup(directory.cleanup)
        return Path(directory.name)

    def test_verified_transfer_preserves_producer_and_existing_product(self):
        install = self.destination()
        stock = install / 'opencpn.exe'; stock.write_bytes(b'preserved product')
        originals = {str(p.relative_to(self.app)): p.read_bytes() for p in self.app.rglob('*') if p.is_file()}
        result = copy_verified_updater_package(self.app, install, self.commit)
        self.assertEqual(result, verify_updater_package(install, self.commit))
        self.assertEqual(stock.read_bytes(), b'preserved product')
        for name, data in originals.items():
            self.assertEqual((self.app / name).read_bytes(), data)
            self.assertEqual((install / name).read_bytes(), data)
        self.assertFalse(list(install.glob('.updater-transfer-*')))
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            copy_verified_updater_package(self.app, install, self.commit)

    def test_transfer_refuses_wrong_commit_unknown_closure_and_existing_outputs(self):
        install = self.destination()
        with self.assertRaisesRegex(ValueError, 'commit'):
            copy_verified_updater_package(self.app, install, 'b' * 40)
        for name in ('unknown.txt', 'opennav/third-party/updater/unknown.txt'):
            extra = self.app / name; extra.write_bytes(b'unknown')
            with self.assertRaisesRegex(ValueError, 'Unexpected updater producer'):
                copy_verified_updater_package(self.app, install, self.commit)
            extra.unlink()
        self.assertEqual(list(install.iterdir()), [])
        for name in ('skager-start.exe', 'update-trust.json'):
            existing = install / name; existing.write_bytes(b'preserve')
            with self.assertRaisesRegex(ValueError, 'overwrite'):
                copy_verified_updater_package(self.app, install, self.commit)
            self.assertEqual(existing.read_bytes(), b'preserve')
            existing.unlink()
        existing = install / 'opennav/third-party/updater'; existing.mkdir(parents=True)
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            copy_verified_updater_package(self.app, install, self.commit)

    def test_transfer_rejects_corruption_before_publication(self):
        install = self.destination()
        original_verify = verify_updater_package
        def mutate_staged(app, commit):
            if app != self.app:
                (app / 'skager-start.exe').write_bytes(b'changed in transit')
            return original_verify(app, commit)
        with mock.patch('updater_package.verify_updater_package', side_effect=mutate_staged):
            with self.assertRaises(ValueError):
                copy_verified_updater_package(self.app, install, self.commit)
        self.assertEqual(list(install.iterdir()), [])

    def test_transfer_refuses_linked_destination_and_racing_output(self):
        install = self.destination(); outside = self.destination()
        link = install / 'opennav'
        try:
            link.symlink_to(outside, target_is_directory=True)
        except OSError:
            self.skipTest('Host cannot create disposable symlink')
        with self.assertRaisesRegex(ValueError, 'reparse'):
            copy_verified_updater_package(self.app, install, self.commit)
        self.assertEqual(list(outside.iterdir()), [])
        link.unlink()
        original_link = os.link
        def race(source, destination):
            if destination.name == 'skager-start.exe':
                destination.write_bytes(b'racing output')
            return original_link(source, destination)
        with mock.patch('updater_package.os.link', side_effect=race):
            with self.assertRaises(FileExistsError):
                copy_verified_updater_package(self.app, install, self.commit)
        self.assertEqual((install / 'skager-start.exe').read_bytes(), b'racing output')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            copy_verified_updater_package(self.app, install, self.commit)

    def test_returns_actual_copied_source_descriptor_without_archive_expansion(self):
        with mock.patch.object(zipfile, 'ZipFile', side_effect=AssertionError('Must not expand dependency source again')):
            result = verify_updater_package(self.app, self.commit)
        self.assertEqual(result['archive'], self.archive)
        self.assertEqual(result['path'], BUNDLE_PATH)
        self.assertEqual(result['sha256'], self.sha(self.archive))
        self.assertEqual(set(result), {'archive', 'path', 'sha256', 'reference'})

    def test_changed_source_or_binary_rejected(self):
        for file in (self.archive, self.binary):
            with self.subTest(file=file.name):
                original = file.read_bytes()
                file.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
                with self.assertRaisesRegex(ValueError, 'changed'):
                    verify_updater_package(self.app, self.commit)
                file.write_bytes(original)

    def test_exact_commit_compiler_and_schema(self):
        mutations = [
            lambda x: x.update(productCommit='b' * 40),
            lambda x: x.update(goVersion='go1.27.0'),
            lambda x: x.update(schema=True),
            lambda x: x.update(extra='unreviewed'),
            lambda x: x['sourceBundle']['reference'].update(productCommit='b' * 40),
            lambda x: x['sourceBundle']['reference'].update(goVersion='go1.27.0'),
            lambda x: x['sourceBundle']['reference']['modules'][0].update(sum='unverified'),
            lambda x: x['sourceBundle']['reference']['modules'].append(x['sourceBundle']['reference']['modules'][0].copy()),
            lambda x: x.update(buildInfo=x['buildInfo'].replace('GOARCH=386', 'GOARCH=amd64')),
            lambda x: x.update(buildInfo=x['buildInfo'].replace('vcs.modified=false', 'vcs.modified=true')),
            lambda x: x.update(buildInfo=x['buildInfo'] + '\tbuild\tGOARCH=386\n'),
        ]
        for mutate in mutations:
            record = copy.deepcopy(self.record)
            mutate(record)
            self.write(record)
            with self.subTest(record=record), self.assertRaises(ValueError):
                verify_updater_package(self.app, self.commit)
        with self.assertRaisesRegex(ValueError, 'Exact product commit'):
            verify_updater_package(self.app, 'unverified')

    def test_ambiguous_json_rejected(self):
        text = self.record_file.read_text(encoding='utf-8')
        self.record_file.write_text('{"schema":1,' + text[1:], encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            verify_updater_package(self.app, self.commit)

    def test_fixed_relative_paths_only(self):
        for field, value in [('archive', '../outside.zip'), ('archive', str(self.archive)),
                             ('archive', 'opennav\\third-party\\updater\\updater-source.zip'),
                             ('path', 'third-party-sources/../../escape.zip')]:
            record = copy.deepcopy(self.record)
            record['sourceBundle'][field] = value
            self.write(record)
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'fixed relative'):
                verify_updater_package(self.app, self.commit)
        record = copy.deepcopy(self.record)
        record['binary']['path'] = '../skager-start.exe'
        self.write(record)
        with self.assertRaisesRegex(ValueError, 'fixed relative'):
            verify_updater_package(self.app, self.commit)

    def test_pe_architecture_verified_independently_of_hash(self):
        pe = bytearray(self.binary.read_bytes())
        struct.pack_into('<H', pe, 68, 0x8664)
        self.binary.write_bytes(pe)
        self.record['binary']['sha256'] = self.sha(self.binary)
        self.write()
        with self.assertRaisesRegex(ValueError, 'Win32'):
            verify_updater_package(self.app, self.commit)

    def test_redirected_source_refused(self):
        original = self.app / 'outside-source.zip'
        self.archive.rename(original)
        try:
            self.archive.symlink_to(original)
        except OSError:
            self.skipTest('Host cannot create disposable symlink')
        with self.assertRaisesRegex(ValueError, 'reparse'):
            verify_updater_package(self.app, self.commit)

    def test_default_trust_fixture_and_oversized_record_refused(self):
        trust = self.app / 'update-trust.json'
        trust.write_text('{"bootstrapRoot":"fixture"}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'must not bundle update trust'):
            verify_updater_package(self.app, self.commit)
        trust.unlink()
        self.record_file.write_bytes(b'x' * ((1 << 20) + 1))
        with self.assertRaisesRegex(ValueError, 'type or size'):
            verify_updater_package(self.app, self.commit)

    def trust_source(self, *, monorepo=False):
        repository = self.destination()
        root = repository/'opennav-x' if monorepo else repository
        source = root/TRUST_SOURCE; source.parent.mkdir(parents=True)
        source.write_text(json.dumps(public_config(), indent=2)+'\n', encoding='utf-8', newline='\n')
        def git(*args):
            return subprocess.check_output(['git', '-C', str(repository), *args], stderr=subprocess.PIPE).decode().strip()
        git('init', '--quiet'); git('config', 'user.name', 'Disposable Fixture')
        git('config', 'user.email', 'fixture@example.invalid'); git('config', 'core.autocrlf', 'false')
        git('add', '.'); git('commit', '--quiet', '-m', 'Public trust fixture')
        self.commit = git('rev-parse', 'HEAD')
        self.record['productCommit'] = self.commit
        self.record['sourceBundle']['reference']['productCommit'] = self.commit
        self.record['buildInfo'] = self.record['buildInfo'].replace('a'*40, self.commit); self.write()
        validator = self.destination()/'skager-repository.exe'; validator.write_bytes(b'inert validator placeholder')
        return root, source, validator, git

    @staticmethod
    def validator_result(command, **kwargs):
        if 'validate-trust' not in command:
            return subprocess.run(command, **kwargs)
        data = Path(command[-1]).read_bytes()
        return subprocess.CompletedProcess(command, 0, json.dumps({'schema': 1, 'status': 'valid',
            'configSha256': hashlib.sha256(data).hexdigest(), 'channel': 'beta'}).encode(), b'')

    def select(self, root, source, validator):
        original_run = subprocess.run
        def result(command, **kwargs):
            if 'validate-trust' not in command:
                return original_run(command, **kwargs)
            self.assertEqual(command, [str(validator), 'validate-trust', '--config', str(source)])
            self.assertEqual(kwargs['timeout'], 30)
            return self.validator_result(command, **kwargs)
        with mock.patch('updater_package.subprocess.run', side_effect=result):
            return select_update_trust(root, self.commit, source, validator)

    def test_explicit_trust_binds_committed_source_and_exact_copied_bytes(self):
        root, source, validator, _ = self.trust_source(monorepo=True)
        selection = self.select(root, source, validator)
        self.assertEqual(selection.git_path, 'opennav-x/'+TRUST_SOURCE)
        self.assertEqual(selection.provenance()['configSha256'], self.sha(source))
        with self.assertRaisesRegex(ValueError, 'missing'):
            verify_updater_package(self.app, self.commit, expected_trust=selection)
        copy_selected_update_trust(self.app, selection)
        self.assertEqual((self.app/'update-trust.json').read_bytes(), source.read_bytes())
        verify_updater_package(self.app, self.commit, expected_trust=selection)
        with self.assertRaisesRegex(ValueError, 'must not bundle update trust'):
            verify_updater_package(self.app, self.commit)
        with self.assertRaises(FileExistsError):
            copy_selected_update_trust(self.app, selection)
        (self.app/'update-trust.json').write_bytes(selection.data+b' ')
        with self.assertRaisesRegex(ValueError, 'selected committed bytes'):
            verify_updater_package(self.app, self.commit, expected_trust=selection)

    def test_trust_source_crlf_is_explicit_but_other_edits_and_untracked_refuse(self):
        root, source, validator, git = self.trust_source()
        original = source.read_bytes(); source.write_bytes(original.replace(b'\n', b'\r\n'))
        selection = self.select(root, source, validator)
        self.assertEqual(selection.git_blob_sha256, hashlib.sha256(original).hexdigest())
        self.assertNotEqual(selection.git_blob_sha256, selection.provenance()['configSha256'])
        self.assertEqual(selection.provenance()['sourceSha256'], selection.provenance()['configSha256'])
        source.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError, 'committed source'):
            self.select(root, source, validator)
        source.write_bytes(original)
        git('rm', '--cached', TRUST_SOURCE); git('commit', '--quiet', '-m', 'Remove selection')
        self.commit = git('rev-parse', 'HEAD')
        with self.assertRaisesRegex(ValueError, 'tracked'):
            self.select(root, source, validator)

    def test_bad_validator_changed_file_and_wrong_receipt_refuse(self):
        root, source, validator, _ = self.trust_source()
        original = source.read_bytes(); run = subprocess.run
        for scenario in ('exit', 'stderr', 'hash', 'changed', 'timeout'):
            source.write_bytes(original)
            def result(command, **kwargs):
                if 'validate-trust' not in command:
                    return run(command, **kwargs)
                checked = self.validator_result(command, **kwargs)
                if scenario == 'exit': checked.returncode = 1
                if scenario == 'stderr': checked.stderr = b'private diagnostic must not leak'
                if scenario == 'hash': checked.stdout = checked.stdout.replace(self.sha(source).encode(), b'f'*64)
                if scenario == 'changed': source.write_bytes(original+b' ')
                if scenario == 'timeout': raise subprocess.TimeoutExpired(command, 30)
                return checked
            with self.subTest(scenario=scenario), mock.patch('updater_package.subprocess.run', side_effect=result):
                with self.assertRaises(ValueError) as error:
                    select_update_trust(root, self.commit, source, validator)
                self.assertNotIn('private diagnostic', str(error.exception))

    def test_secret_unknown_schema_and_endpoint_or_allowlist_changes_refuse(self):
        original = public_config()
        changes = [lambda c:c.update(token='secret'), lambda c:c.update(schema=True),
            lambda c:c.update(channel='stable'), lambda c:c.update(metadataUrl='http://updates.example.test'),
            lambda c:c.update(metadataUrl='https://user:secret@updates.example.test'),
            lambda c:c.update(metadataUrl='https://updates.example.test?token=secret'),
            lambda c:c.update(artifactOrigin='https://updates.example.test/artifacts'),
            lambda c:c['localCompatibilityAllowlist'][0].update(executableSha256='f'*64),
            lambda c:c['localCompatibilityAllowlist'][0].update(unknown='secret'),
            lambda c:c['bootstrapRoot']['signed'].update(private='secret'),
            lambda c:c['bootstrapRoot']['signed']['keys']['1'*64]['keyval'].update(private='secret'),
            lambda c:c['bootstrapRoot']['signed']['roles']['targets'].update(keyids=['1'*64]),
            lambda c:c['bootstrapRoot'].update(extra='secret')]
        validate_public_update_trust(json.dumps(original).encode())
        for change in changes:
            config = copy.deepcopy(original); change(config)
            with self.subTest(config=config), self.assertRaises(ValueError):
                validate_public_update_trust(json.dumps(config).encode())
        duplicate = b'{"schema":1,'+json.dumps(original).encode()[1:]
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            validate_public_update_trust(duplicate)


if __name__ == '__main__':
    unittest.main()
