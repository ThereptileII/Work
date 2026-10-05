#!/usr/bin/env python3
"""Disposable real-Git and inert compiled-tree recovery tests; never builds/runs app."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock
import zipfile

import compiled_recovery as recovery
import staging_build_inputs as sealed


class Recovery(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name).resolve()
        self.repository = self.base / 'repository'; self.repository.mkdir()
        self.root = self.repository / 'opennav-x'; self.root.mkdir()
        self.output = self.base / 'retained'
        self.git(self.repository, 'init', '-q')
        for name, content in {'opennav-x/app.cpp': 'original tracked application\n',
                              'opennav-x/.gitignore': 'build/\n.local/\n',
                              '.github/workflows/opennav-baseline.yml': 'name: inert producer\n'}.items():
            path = self.repository / name; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        self.commit = self.commit_all(self.repository)
        self.upstream = self.root / 'build/integration-source'; self.upstream.mkdir(parents=True)
        self.git(self.upstream, 'init', '-q')
        (self.upstream / 'core.cpp').write_text('pinned baseline\n')
        self.upstream_commit = self.commit_all(self.upstream)
        # Real prepared integration tree legitimately differs from pinned HEAD.
        (self.upstream / 'core.cpp').write_text('patched compiled integration\n')
        for variant in sealed.VARIANTS:
            for name, data in {
                f'build/{variant}-install/opencpn.exe': b'MZ inert compiled application; never execute',
                f'build/{variant}-install/opennav-restart.exe': b'MZ inert restart fixture',
                f'build/{variant}-install/plugin/runtime.dll': b'MZ inert plugin bytes',
                f'build/{variant}-windows/include/config.h': b'#define INERT 1\n',
                f'build/{variant}-windows/include/OpenNavBuild.h': ('#define OPENNAV_BUILD_COMMIT "'+self.commit+'"\n').encode(),
                f'build/{variant}-windows/Release/tests.exe': b'MZ inert tests',
            }.items():
                path=self.root / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(data)
        (self.root / 'untracked-private.txt').write_text('MUST NOT RETAIN')
        self.expected = sealed.producer(self.commit, '123', '2')
        self.pin = mock.patch.object(recovery, 'PINNED_UPSTREAM', self.upstream_commit); self.pin.start()

    def tearDown(self):
        self.pin.stop(); self.temp.cleanup()

    @staticmethod
    def git(root, *args):
        return subprocess.check_output(['git', '-C', str(root), *args], stderr=subprocess.STDOUT).decode().strip()

    def commit_all(self, root):
        self.git(root, 'add', '.')
        self.git(root, '-c', 'user.name=Inert Fixture', '-c', 'user.email=inert@example.invalid', 'commit', '-qm', 'fixture')
        return self.git(root, 'rev-parse', 'HEAD')

    def retain(self):
        return recovery.retain(self.root, self.output, self.expected)

    def test_built_dirty_tree_retained_before_any_package_exists(self):
        # This is precisely the packaging failure context: actual outputs exist,
        # but a tracked edit and arbitrary untracked checkout data also exist.
        (self.root / 'app.cpp').write_text('dirty actual source\n')
        receipt = self.retain()
        self.assertEqual(receipt['qualification'], 'not-run')
        self.assertEqual(receipt['packaging'], 'not-run')
        self.assertTrue(receipt['trackedSourceDirty'])
        with zipfile.ZipFile(self.output / recovery.ARCHIVE) as archive:
            names = archive.namelist()
            self.assertFalse(any('untracked-private' in name for name in names))
            self.assertEqual(archive.read('source/product/app.cpp'), (self.root / 'app.cpp').read_bytes())
            self.assertEqual(archive.read('source/integrated/core.cpp'), (self.upstream / 'core.cpp').read_bytes())
            self.assertEqual(archive.read('source/recipe/.github/workflows/opennav-baseline.yml'), (self.repository / '.github/workflows/opennav-baseline.yml').read_bytes())
            manifest = json.loads(archive.read(recovery.MANIFEST))
            self.assertEqual(manifest['producer'], self.expected)
            for item in manifest['files']:
                import hashlib
                self.assertEqual(hashlib.sha256(archive.read(item['path'])).hexdigest(), item['sha256'])
            self.assertNotIn(sealed.MANIFEST, names)
            self.assertFalse(any('Setup.exe' in name for name in names))
        self.assertEqual(receipt['archiveSha256'], sealed.sha(self.output / recovery.ARCHIVE))
        # The qualified input path still refuses an incomplete package.
        with self.assertRaisesRegex(ValueError, 'Missing required'):
            sealed.validate_inventory(sealed.inventory(self.root))

    def test_wrong_compiled_commit_refuses(self):
        (self.root / 'build/production-windows/include/OpenNavBuild.h').write_text('#define OPENNAV_BUILD_COMMIT "'+'f'*40+'"')
        with self.assertRaisesRegex(ValueError, 'header differs'): self.retain()
        self.assertFalse(self.output.exists())

    def test_wrong_source_and_upstream_commits_refuse(self):
        with mock.patch.object(recovery, 'PINNED_UPSTREAM', 'f'*40):
            with self.assertRaisesRegex(ValueError, 'baseline differs'): self.retain()
        expected = dict(self.expected, commit='f'*40)
        with self.assertRaisesRegex(ValueError, 'checkout differs'):
            recovery.source_inputs(self.root, expected['commit'])

    def test_missing_app_and_existing_output_refuse(self):
        path=self.root / 'build/production-install/opencpn.exe'; data=path.read_bytes(); path.unlink()
        with self.assertRaisesRegex(ValueError, 'completed native'): self.retain()
        path.write_bytes(data); self.output.mkdir(); (self.output / 'sentinel').write_text('unchanged')
        with self.assertRaisesRegex(ValueError, 'fresh'): self.retain()
        self.assertEqual((self.output / 'sentinel').read_text(), 'unchanged')

    def test_bounds_remove_partial_output(self):
        with mock.patch.object(recovery, 'MAX_SOURCE', 1):
            with self.assertRaisesRegex(ValueError, 'budget'): self.retain()
        self.assertFalse(self.output.exists())

    def test_source_mutation_after_capture_refuses(self):
        (self.root / 'app.cpp').write_text('dirty before retention')
        original = recovery.source_inputs; calls = 0
        def changing(*args):
            nonlocal calls
            calls += 1
            if calls == 2: (self.root / 'app.cpp').write_text('changed while retaining')
            return original(*args)
        with mock.patch.object(recovery, 'source_inputs', side_effect=changing):
            with self.assertRaisesRegex(ValueError, 'changed during retention'): self.retain()
        self.assertFalse(self.output.exists())

    def test_linked_binary_refuses(self):
        path=self.root / 'build/production-install/opencpn.exe'; path.unlink()
        try: path.symlink_to(self.root / 'app.cpp')
        except OSError: self.skipTest('Host does not permit disposable symlink fixture')
        with self.assertRaisesRegex(ValueError, 'Linked|Redirected'): self.retain()


if __name__ == '__main__':
    unittest.main()
