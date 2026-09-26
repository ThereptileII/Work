#!/usr/bin/env python3
"""Source-distribution tests using disposable committed repositories."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import source_package


class SourceDistributionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='opennav-source-test-')
        self.repository = Path(self.temporary.name)
        self.root = self.repository / 'opennav-x'
        self.root.mkdir()
        self.workflow = self.repository / '.github/workflows/opennav-baseline.yml'
        self.workflow.parent.mkdir(parents=True)
        self.workflow.write_text('name: exact committed workflow\n')
        (self.root / '.gitignore').write_text('build/\n')
        (self.root / 'source.cpp').write_text('int main() {}\n')
        (self.root / 'patches').mkdir()
        for name in ('opencpn-5.12.4-xnav.patch', 'opencpn-5.12.4-regression-tests.patch'):
            (self.root / 'patches' / name).write_text('fixture patch\n')
        self.init(self.repository)
        # A Git symlink need not be creatable on a restricted Windows runner.
        # Populate its index blob directly; the archiver must preserve link text.
        blob = subprocess.check_output(['git', '-C', str(self.repository), 'hash-object', '-w', '--stdin'], input=b'source.cpp').decode().strip()
        self.git(self.repository, 'update-index', '--add', '--cacheinfo', '120000,' + blob + ',opennav-x/source-link')
        self.git(self.repository, 'commit', '-qm', 'source distribution fixture')
        self.commit = self.git(self.repository, 'rev-parse', 'HEAD').strip()
        # Materialize the indexed link as a plain checkout-compatible file. With
        # core.symlinks=false Git recognizes its text as an unchanged symlink.
        self.git(self.repository, 'config', 'core.symlinks', 'false')
        (self.root / 'source-link').write_text('source.cpp')
        self.upstream = self.root / 'build/integration-source'
        self.upstream.mkdir(parents=True)
        (self.upstream / 'navigation.cpp').write_text('pinned baseline\n')
        self.init(self.upstream)
        self.git(self.upstream, 'commit', '-qm', 'upstream fixture')
        pin = self.git(self.upstream, 'rev-parse', 'HEAD').strip()
        self.pin = patch.object(source_package, 'PINNED_UPSTREAM', pin)
        self.pin.start()
        self.addCleanup(self.pin.stop)
        self.addCleanup(self.temporary.cleanup)

    def git(self, path, *args):
        return subprocess.check_output(['git', '-C', str(path), *args], text=True)

    def init(self, path):
        self.git(path, 'init', '-q')
        self.git(path, 'config', 'user.name', 'Source Archive Test')
        self.git(path, 'config', 'user.email', 'fixture@example.invalid')
        self.git(path, 'add', '.')

    def test_monorepo_recipe_patch_hashes_and_symlink(self):
        # The application build deliberately modifies the pinned upstream tree.
        (self.upstream / 'navigation.cpp').write_text('reviewed integration\n')
        archive = self.repository / 'source.zip'
        references = source_package.create_source_archive(self.root, self.commit, archive)
        with zipfile.ZipFile(archive) as source:
            self.assertIsNone(source.testzip())
            self.assertEqual(source.read('.github/workflows/opennav-baseline.yml'), self.workflow.read_bytes())
            self.assertEqual(source.read('opennav-x/source-link'), b'source.cpp')
            self.assertEqual(source.getinfo('opennav-x/source-link').external_attr >> 16 & 0o170000, 0o120000)
            self.assertEqual(source.read('OpenCPN-5.12.4-integrated/navigation.cpp'), b'reviewed integration\n')
            self.assertEqual(json.loads(source.read('SOURCE_REFERENCE.json')), references)
            for name, record in references['files'].items():
                self.assertEqual(hashlib.sha256(source.read(name)).hexdigest(), record['sha256'])
        with self.assertRaises(ValueError):
            source_package.create_source_archive(self.root, self.commit, archive)

    def test_uncommitted_product_refused(self):
        (self.root / 'source.cpp').write_text('uncommitted change\n')
        with self.assertRaises(subprocess.CalledProcessError):
            source_package.source_inventory(self.root, self.commit)

    def test_staged_product_refused(self):
        (self.root / 'source.cpp').write_text('staged uncommitted change\n')
        self.git(self.repository, 'add', '.')
        with self.assertRaises(subprocess.CalledProcessError):
            source_package.source_inventory(self.root, self.commit)

    def test_wrong_commit_refused(self):
        with self.assertRaises(ValueError):
            source_package.source_inventory(self.root, '0' * 40)

    def test_wrong_upstream_refused(self):
        with patch.object(source_package, 'PINNED_UPSTREAM', '0' * 40):
            with self.assertRaises(ValueError):
                source_package.source_inventory(self.root, self.commit)

    def test_changed_top_level_workflow_refused(self):
        self.workflow.write_text('uncommitted workflow\n')
        with self.assertRaises(ValueError):
            source_package.source_inventory(self.root, self.commit)

    def test_windows_crlf_checkout_is_exact_committed_recipe(self):
        self.git(self.repository, 'config', 'core.autocrlf', 'true')
        self.workflow.write_bytes(b'name: exact committed workflow\r\n')
        entries, _ = source_package.source_inventory(self.root, self.commit)
        self.assertEqual(entries['.github/workflows/opennav-baseline.yml'][0], self.workflow.read_bytes())


if __name__ == '__main__':
    unittest.main()
