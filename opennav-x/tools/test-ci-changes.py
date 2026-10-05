#!/usr/bin/env python3
"""Focused selection contracts, including real Git rename/delete/history cases."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import ci_changes as ci


class ClassificationTests(unittest.TestCase):
    def expect(self, paths, flags, layout='local'):
        result = ci.classify(paths, layout)
        self.assertEqual({key for key in ci.FLAGS if result[key]}, set(flags), result)
        return result

    def test_runtime_and_packaged_content_rebuild(self):
        for path in ('src/application/Version.h', 'resources/chart-style/icon.svg',
                     'installer/windows/Lifecycle.ps1', 'docs/beta2/Release.md',
                     'docs/third-party/curl/COPYING', 'LICENSE', 'CMakeLists.txt',
                     'tools/source_package.py', 'tools/package-preview.py',
                     'tools/chart_building_point.py', 'tests/support/VesselFixture.cpp',
                     'tests/vessel_state_tests.cpp', 'tests/installer_welcome_tests.py'):
            with self.subTest(path=path):
                self.expect([path], ['product'])

    def test_ordinary_docs_do_not_request_new_candidate(self):
        self.expect(['README.md', 'docs/delivery-workflow.md',
                     'docs/evidence/test.json', 'docs/design/prototype/index.html'], ['docs'])

    def test_helpers_do_not_build_application(self):
        for path in ('tools/smoke-navigation.py',
                     'tools/test-ci-changes.py', 'tools/ci_changes.py',
                     'tools/github_release_delivery.py', 'tools/staging_build_inputs.py',
                     'tools/fetch_ci_inputs.py', 'tools/qualify-staging-windows.ps1',
                     'tools/retest-staging-windows.py',
                     'tools/test-staging-build-inputs.py',
                     'tools/test-windows-dependency-bundle.py',
                     '.github/workflows/opennav-baseline.yml',
                     'tools/boat/RestartWindowReview.ps1', 'tools/boat/build.ps1',
                     'tools/prototype/derive-example.py'):
            with self.subTest(path=path):
                self.expect([path], ['helpers'])

    def test_producer_workflow_invalidates_reuse_without_application_build(self):
        self.expect(['.github/workflows/skager-windows-dependencies.yml'],
                    ['helpers', 'dependencies'], 'monorepo')

    def test_dependency_producer_and_test_named_producer_inputs(self):
        for path in ('tools/build-curl-windows.ps1', 'tools/windows-curl.lock.json',
                     'tools/patch-curl-test-openssl.py', 'tools/windows_dependency_bundle.py',
                     'tools/build-windows-dependency-bundle.ps1',
                     'tools/test-curl-source-preflight.ps1',
                     'tools/test-zlib-source-verification.ps1',
                     'patches/opencpn-5.12.4-maintained-curl.patch', 'upstream.lock.json'):
            with self.subTest(path=path):
                self.expect([path], ['product', 'dependencies'])

    def test_unknown_is_conservative_not_extension_guessing(self):
        for path in ('future-manifest.json', 'new-runtime/readme.md',
                     'tools/new-transform.py', '../docs/readme.md', '/docs/readme.md'):
            with self.subTest(path=path):
                self.expect([path], ['product', 'dependencies', 'helpers'])

    def test_other_project_and_monorepo_mapping(self):
        self.expect(['web/app.py'], [])
        self.expect(['opennav-x/src/application/Version.h'], ['product'], 'monorepo')
        self.expect(['opennav-x/docs/status.md'], ['docs'], 'monorepo')
        self.expect(['opennav-x/web/app.py'], [], 'monorepo')
        self.expect(['new-project/unknown.c'], ['product', 'dependencies', 'helpers'], 'monorepo')
        self.expect(['.github/workflows/skager-production.yml'], ['helpers'], 'monorepo')

    def test_mixed_changes_and_force_are_additive(self):
        self.expect(['docs/status.md', 'tools/test-ci-changes.py', 'src/main.cpp'],
                    ['docs', 'helpers', 'product'])
        result = ci.classify(['docs/status.md'], force_product=True)
        self.assertTrue(result['product'])
        self.assertTrue(result['dependencies'])
        self.assertTrue(result['docs'])
        self.assertNotIn('production', result)
        self.assertNotIn('design', result)

    def test_invalid_git_records_refuse_partial_selection(self):
        for data in (b'M\0docs/a', b'R100\0old\0', b'U\0docs/a\0', b'M\0\xff\0'):
            with self.subTest(data=data), self.assertRaises(ValueError):
                ci.parse_changes(data)


class GitRangeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.repo = Path(self.temporary.name)
        self.run_git('init', '-q')
        self.run_git('config', 'user.name', 'CI selection test')
        self.run_git('config', 'user.email', 'ci-selection@example.invalid')
        self.write('README.md', 'initial\n')
        self.initial = self.commit()

    def run_git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.repo), *args], stderr=subprocess.PIPE).decode().strip()

    def write(self, name, content):
        path = self.repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)

    def commit(self):
        self.run_git('add', '-A')
        self.run_git('commit', '-qm', 'fixture')
        return self.run_git('rev-parse', 'HEAD')

    def commit_unusual_path(self, name, content):
        # Windows cannot create newline filenames. Keep this real Git history
        # fixture in the object database, without checking it out or relying on
        # filesystem/index path acceptance. NUL records preserve the exact name.
        def object_command(*args, data):
            return subprocess.check_output(['git', '-C', str(self.repo), *args],
                                           input=data, stderr=subprocess.PIPE).decode().strip()
        object_id = object_command('hash-object', '-w', '--stdin', data=content)
        mode, kind = '100644', 'blob'
        parts = name.split('/')
        for position in range(len(parts) - 1, -1, -1):
            record = f'{mode} {kind} {object_id}\t{parts[position]}\0'.encode()
            if position == 0:
                readme = self.run_git('rev-parse', self.initial + ':README.md')
                record += f'100644 blob {readme}\tREADME.md\0'.encode()
            object_id = object_command('mktree', '-z', data=record)
            mode, kind = '040000', 'tree'
        return self.run_git('commit-tree', object_id, '-p', self.initial, '-m', 'unusual-path fixture')

    def test_docs_range_and_empty_range(self):
        self.write('docs/notes.md', 'ordinary docs\n')
        head = self.commit()
        result = ci.select(self.repo, self.initial, head)
        self.assertFalse(result['product'])
        self.assertTrue(result['docs'])
        self.assertEqual(result['changed_paths'], ['docs/notes.md'])
        self.assertFalse(any(ci.select(self.repo, head, head)[flag] for flag in ci.FLAGS))

    def test_rename_product_to_docs_keeps_old_input(self):
        self.write('src/old.cpp', 'runtime source\n')
        base = self.commit()
        self.run_git('mv', 'src/old.cpp', 'README-renamed.md')
        head = self.commit()
        result = ci.select(self.repo, base, head)
        self.assertTrue(result['product'])
        self.assertIn('src/old.cpp', result['changed_paths'])
        self.assertIn('README-renamed.md', result['changed_paths'])

    def test_delete_packaged_notice_and_unusual_filename(self):
        base = self.commit_unusual_path('docs/beta2/line\nbreak.md', b'packaged\n')
        original_tree = self.run_git('rev-parse', self.initial + '^{tree}')
        head = self.run_git('commit-tree', original_tree, '-p', base, '-m', 'delete packaged notice')
        result = ci.select(self.repo, base, head)
        self.assertTrue(result['product'])
        self.assertEqual(result['changed_paths'], ['docs/beta2/line\nbreak.md'])

    def test_missing_zero_and_option_like_revisions_build(self):
        for base in ('', '0' * 40, 'unknown-revision', '--help'):
            with self.subTest(base=base):
                result = ci.select(self.repo, base, self.initial)
                self.assertTrue(result['product'])
                self.assertTrue(result['dependencies'])
                self.assertEqual(result['reasons'][-1]['category'], 'fallback')

    def test_diverged_history_requires_explicit_merge_base(self):
        self.write('docs/base.md', 'base branch\n')
        base = self.commit()
        self.run_git('checkout', '-q', '--detach', self.initial)
        self.write('docs/head.md', 'head branch\n')
        head = self.commit()
        self.assertTrue(ci.select(self.repo, base, head)['product'])
        pr = ci.select(self.repo, base, head, merge_base=True)
        self.assertFalse(pr['product'])
        self.assertEqual(pr['changed_paths'], ['docs/head.md'])

    def test_unrelated_history_is_conservative_even_for_pr(self):
        self.run_git('checkout', '-q', '--orphan', 'unrelated')
        self.write('docs/unrelated.md', 'unrelated root\n')
        head = self.commit()
        result = ci.select(self.repo, self.initial, head, merge_base=True)
        self.assertTrue(result['product'])
        self.assertTrue(result['dependencies'])

    def test_shallow_clone_missing_base_is_conservative(self):
        self.write('docs/later.md', 'later\n')
        head = self.commit()
        with tempfile.TemporaryDirectory() as target:
            subprocess.run(['git', 'clone', '-q', '--depth=1', self.repo.as_uri(), target], check=True)
            result = ci.select(Path(target), self.initial, head)
            self.assertTrue(result['product'])
            self.assertTrue(result['dependencies'])

    def test_cli_boolean_outputs_are_stable_and_json_paths_are_escaped(self):
        head = self.commit_unusual_path('docs/a\nb.md', b'docs\n')
        output = self.repo / 'github-output'
        command = [sys.executable, str(Path(ci.__file__).resolve()), '--repo', str(self.repo),
                   '--base', self.initial, '--head', head, '--github-output', str(output)]
        result = json.loads(subprocess.check_output(command))
        self.assertEqual(result['changed_paths'], ['docs/a\nb.md'])
        self.assertEqual(output.read_text(), 'product=false\nhelpers=false\ndependencies=false\ndocs=true\n')


if __name__ == '__main__':
    unittest.main()
