#!/usr/bin/env python3
"""Exercise the real preparation script with Windows-style Git mode handling."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('xnav', 'regression-tests', 'ais-transport', 'chart-presentation',
         'maintained-curl', 'download-trust', 'wxcurl-trust',
         'peer-response-buffer', 'peer-unavailable')


def git(path, *args, **kwargs):
    return subprocess.run(['git', '-C', str(path), *args], check=True,
                          capture_output=True, **kwargs)


class PreparationTests(unittest.TestCase):
    def fixture(self, base, autocrlf=False, filemode=False):
        root = base / 'product'
        source = root / 'upstream/OpenCPN'
        source.mkdir(parents=True)
        git(source, 'init')
        git(source, 'config', 'user.name', 'Fixture')
        git(source, 'config', 'user.email', 'fixture@example.invalid')
        git(source, 'config', 'core.filemode', str(filemode).lower())
        git(source, 'config', 'core.autocrlf', str(autocrlf).lower())
        for index in range(9):
            (source / f'unit{index}.cpp').write_text('one\ntwo\nthree\n')
        git(source, 'add', '.')
        git(source, 'commit', '-m', 'Pinned fixture')
        commit = git(source, 'rev-parse', 'HEAD').stdout.decode().strip()
        (root / 'upstream.lock.json').write_text(json.dumps({'commit': commit}))
        (root / 'tools').mkdir()
        for name in ('prepare-integration.py', 'verify-upstream.py'):
            shutil.copyfile(ROOT / 'tools' / name, root / 'tools' / name)
        (root / 'patches').mkdir()
        for index, name in enumerate(NAMES):
            filename = f'unit{index}.cpp'
            patch = (f'diff --git a/{filename} b/{filename}\n'
                     f'--- a/{filename}\n+++ b/{filename}\n'
                     '@@ -1,3 +1,3 @@\n-one\n+ONE\n two\n three\n')
            if name == 'chart-presentation':
                patch += (f'diff --git a/{filename} b/{filename}\n'
                          'index 1111111..2222222 100644\n'
                          f'--- a/{filename}\n+++ b/{filename}\n'
                          '@@ -1,3 +1,3 @@\n ONE\n-two\n+TWO\n three\n')
            (root / 'patches' / f'opencpn-5.12.4-{name}.patch').write_text(patch)
        return root, source

    def prepare(self, root):
        return subprocess.run([sys.executable, str(root / 'tools/prepare-integration.py')],
                              capture_output=True, text=True)

    def test_original_failure_then_exact_preparation_and_no_real_index_change(self):
        for filemode, autocrlf in ((False, False), (False, True), (True, False)):
            with self.subTest(filemode=filemode, autocrlf=autocrlf), tempfile.TemporaryDirectory() as temp:
                root, source = self.fixture(Path(temp), autocrlf, filemode)
                patch = (root / 'patches/opencpn-5.12.4-chart-presentation.patch').read_bytes()
                original = subprocess.run(['git', '-C', str(source), 'apply', '--check', '-'],
                                          input=patch, capture_output=True)
                if filemode:
                    self.assertEqual(original.returncode, 0, original.stderr)
                else:
                    self.assertNotEqual(original.returncode, 0)
                    self.assertIn(b'wrong type', original.stderr)
                result = self.prepare(root)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                target = root / 'build/integration-source'
                self.assertEqual((target / 'unit3.cpp').read_text(), 'ONE\nTWO\nthree\n')
                self.assertEqual(git(target, 'diff', '--cached', '--name-only').stdout, b'')
                real_index = Path(git(target, 'rev-parse', '--git-path', 'index').stdout.decode().strip())
                if not real_index.is_absolute():
                    real_index = target / real_index
                before = hashlib.sha256(real_index.read_bytes()).hexdigest()
                repeated = self.prepare(root)
                self.assertEqual(repeated.returncode, 0, repeated.stdout + repeated.stderr)
                self.assertEqual(before, hashlib.sha256(real_index.read_bytes()).hexdigest())
                # Same line count: refuse tampering instead of reapplying over it.
                (target / 'unit3.cpp').write_text('BAD\nTWO\nthree\n')
                changed = self.prepare(root)
                self.assertNotEqual(changed.returncode, 0)
                self.assertIn('differs from reviewed patches', changed.stderr)
                self.assertEqual((target / 'unit3.cpp').read_text(), 'BAD\nTWO\nthree\n')
                self.assertEqual(git(source, 'status', '--porcelain').stdout, b'')

    def test_wrong_pin_and_dirty_pristine_source_remain_refused(self):
        with tempfile.TemporaryDirectory() as temp:
            root, source = self.fixture(Path(temp))
            (source / 'unit0.cpp').write_text('changed\n')
            result = self.prepare(root)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('tracked OpenCPN source is modified', result.stderr)
            git(source, 'checkout', '--', 'unit0.cpp')
            (root / 'upstream.lock.json').write_text(json.dumps({'commit': '0' * 40}))
            result = self.prepare(root)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('commit differs', result.stderr)


if __name__ == '__main__':
    unittest.main()
