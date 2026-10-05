#!/usr/bin/env python3
"""Deterministic inert transport tests; these never qualify native producers."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import unittest
from unittest import mock

import windows_dependency_bundle as bundle
import windows_dependency_receipt as receipt

spec = importlib.util.spec_from_file_location('evidence_fixtures', Path(__file__).with_name('test-windows-dependency-evidence.py'))
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)


class BundleTests(fixtures.WindowsDependencyEvidenceTests):
    def setUp(self):
        super().setUp()
        source = Path(__file__).resolve().parent.parent
        for name in bundle.INPUTS:
            target = self.root / name
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source / name, target)
        for kind, name in bundle.reuse.PRODUCER_FACTS.items():
            self._write_json(self.root / name, {'schemaVersion': 1, 'kind': kind})
        for name in bundle.LOGS:
            target = self.root / name
            if not target.exists():
                if name.endswith('source-preflight'):
                    target.mkdir(parents=True)
                    (target / 'source-analysis.json').write_text('{}')
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_text('{}')
        for build in bundle.CMAKE_BUILDS:
            target = self.root / build
            target.mkdir(parents=True)
            for name in ('CMakeCache.txt', 'xnav-native-cmake-tools.txt',
                         'CMakeFiles/3.30/CMakeCCompiler.cmake',
                         'zlib.vcxproj' if 'zlib' in build else 'lib/libcurl_shared.vcxproj'):
                file = target / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_text('owned native metadata fixture')
        # The inherited fixture already has complete package/build/test records;
        # preserve them while proving all producer outputs exist independently.
        workflow = self.root / bundle.WORKFLOW
        workflow.parent.mkdir(parents=True)
        workflow.write_text('owned producer workflow fixture')
        self.identity = {'repository': 'owned/fixture', 'workflowPath': bundle.WORKFLOW,
                         'runId': '23', 'runAttempt': '2', 'job': bundle.JOB,
                         'headSha': 'a' * 40}
        self.stack.enter_context(mock.patch.object(bundle, '_identity', return_value=self.identity))
        self.stack.enter_context(mock.patch.dict(os.environ, {'GITHUB_REPOSITORY': 'owned/fixture', 'RUNNER_OS': 'Windows',
                                                    'RUNNER_ARCH': 'X64', 'ImageOS': 'win22',
                                                    'ImageVersion': '20261001.1.0'}))
        self.output = self.root / 'sealed'
        self.authority = self.root / 'authenticated.json'

    def seal(self):
        bundle.seal(self.root, self.output, producer_success=True)
        document = receipt._read_json(self.output / 'bundle.json')
        authority = {**self.identity, 'schemaVersion': 1, 'conclusion': 'success',
                     'artifactId': '345', 'artifactDigest': 'sha256:' + 'b' * 64,
                     'bundleSha256': receipt._digest(self.output / 'bundle.json'),
                     'artifactName': f"windows-dependencies-{document['fingerprint']['sha256']}-run23-attempt2"}
        self._write_json(self.authority, authority)
        return document

    def test_roundtrip_full_prefix_and_stage(self):
        # Undeclared additional public header/license is retained by complete
        # prefix inventory, rather than reduced to manifest DLL outputs.
        prefix = self.root / bundle.evidence.PREFIXES['openssl']
        (prefix / 'include/openssl/extra.h').write_text('header')
        (prefix / 'LICENSE.txt').write_text('license')
        document = self.seal()
        shutil.rmtree(prefix)
        bundle.restore(self.root, self.output, self.authority)
        self.assertEqual((prefix / 'include/openssl/extra.h').read_text(), 'header')
        cache = self.root / bundle.stage.CACHE
        cache.mkdir(parents=True)
        (cache / 'ssleay32.dll').write_text('obsolete')
        bundle.stage_bundle(self.root, self.output, self.authority)
        self.assertFalse((cache / 'ssleay32.dll').exists())
        self.assertEqual((cache / 'include/openssl/extra.h').read_text(), 'header')
        self.assertIn('build/windows-curl-8.22.0/install/lib/libcurl.lib', document['files'])

    def test_failed_or_zero_test_producer_cannot_seal(self):
        path = self.root / 'evidence/local/windows-openssl-native-output.log'
        path.write_text('All tests successful.\nFiles=1, Tests=0, 1 sec\nResult: PASS\n')
        with self.assertRaisesRegex(ValueError, 'nonzero'):
            self.seal()
        self.assertFalse(self.output.exists())

    def test_nested_test_compiler_metadata_is_not_producer_toolchain(self):
        for build in bundle.CMAKE_BUILDS:
            unrelated = self.root / build / 'tests/subbuild/CMakeFiles/3.30/CMakeCCompiler.cmake'
            unrelated.parent.mkdir(parents=True)
            unrelated.write_text('nested standalone test project metadata')
        document = self.seal()
        self.assertFalse(any('tests/subbuild' in name for name in document['files']))
        bundle.verify_bundle(self.root, self.output, self.authority)

    def test_ambiguous_selected_compiler_metadata_still_rejected(self):
        duplicate = self.root / bundle.CMAKE_BUILDS[0] / 'CMakeFiles/other/CMakeCCompiler.cmake'
        duplicate.parent.mkdir(parents=True)
        duplicate.write_text('ambiguous actual producer toolchain')
        with self.assertRaisesRegex(ValueError, 'ambiguous native compiler'):
            self.seal()

    def test_missing_selected_compiler_metadata_is_rejected(self):
        selected = self.root / bundle.CMAKE_BUILDS[0] / 'CMakeFiles/3.30/CMakeCCompiler.cmake'
        selected.unlink()
        with self.assertRaisesRegex(ValueError, 'missing or ambiguous native compiler'):
            self.seal()

    def test_no_success_assertion(self):
        with self.assertRaises(ValueError):
            bundle.seal(self.root, self.output, producer_success=False)

    def test_seal_is_immutable(self):
        self.seal()
        with self.assertRaisesRegex(ValueError, 'already exists'):
            self.seal()

    def test_changed_recipe_rejects_before_restore(self):
        self.seal()
        path = self.root / 'tools/windows-curl-import-layout.cmake'
        path.write_text('changed configuration')
        with self.assertRaisesRegex(ValueError, 'inputs changed'):
            bundle.restore(self.root, self.output, self.authority)

    def test_producer_workflow_change_invalidates_fingerprint(self):
        self.seal()
        (self.root / bundle.WORKFLOW).write_text('changed workflow environment')
        with self.assertRaisesRegex(ValueError, 'inputs changed'):
            bundle.verify_bundle(self.root, self.output, self.authority)

    def test_published_parent_workflow_cannot_be_shadowed_by_snapshot(self):
        original = bundle.fingerprint(self.root)
        nested = self.root / 'opennav-x'
        for name in bundle.INPUTS:
            destination = nested / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(self.root / name, destination)
        shadow = nested / bundle.WORKFLOW
        shadow.parent.mkdir(parents=True)
        shadow.write_text('untrusted obsolete nested snapshot')
        self.assertEqual(bundle.fingerprint(nested), original)
        (self.root / bundle.WORKFLOW).write_text('changed authoritative parent')
        self.assertNotEqual(bundle.fingerprint(nested), original)

    def test_ui_only_change_keeps_fingerprint(self):
        original = bundle.fingerprint(self.root)
        (self.root / 'src').mkdir()
        (self.root / 'src/ui.cpp').write_text('new UI')
        self.assertEqual(original, bundle.fingerprint(self.root))

    def test_missing_changed_extra_and_case_alias_payload(self):
        document = self.seal()
        payload = self.output / 'payload'
        name = next(name for name in document['files'] if name.endswith('libcurl.lib'))
        file = payload / name
        original = file.read_bytes()
        for mutation in ('missing', 'changed', 'extra', 'alias'):
            with self.subTest(mutation=mutation):
                extra = payload / ('UNLISTED.txt' if mutation == 'extra' else name.upper())
                if mutation == 'missing':
                    file.unlink()
                elif mutation == 'changed':
                    file.write_bytes(b'tampered')
                else:
                    if mutation == 'alias' and os.name == 'nt':
                        continue  # Windows itself prevents separate case aliases.
                    extra.parent.mkdir(parents=True, exist_ok=True)
                    extra.write_bytes(b'extra')
                with self.assertRaises((OSError, ValueError)):
                    bundle.verify_bundle(self.root, self.output, self.authority)
                if mutation in ('missing', 'changed'):
                    file.write_bytes(original)
                else:
                    extra.unlink()

    def test_manifest_tamper_is_rejected(self):
        self.seal()
        path = self.output / 'bundle.json'
        path.write_bytes(path.read_bytes() + b' ')
        with self.assertRaisesRegex(ValueError, 'authenticated artifact'):
            bundle.verify_bundle(self.root, self.output, self.authority)

    def test_wrong_producer_run_attempt_repository_job_and_status(self):
        self.seal()
        original = receipt._read_json(self.authority)
        for key, value in (('runId', '24'), ('runAttempt', '3'), ('headSha', 'c' * 40),
                           ('repository', 'other/repo'), ('job', 'unapproved'),
                           ('workflowPath', '.github/workflows/other.yml'), ('conclusion', 'failure'),
                           ('artifactName', 'broad-cache-latest')):
            with self.subTest(key=key):
                self._write_json(self.authority, {**original, key: value})
                with self.assertRaises(ValueError):
                    bundle.verify_bundle(self.root, self.output, self.authority)

    def test_embedded_provenance_is_not_authority(self):
        self.seal()
        embedded = self.output / 'claimed-authority.json'
        shutil.copyfile(self.authority, embedded)
        with self.assertRaisesRegex(ValueError, 'outside'):
            bundle.verify_bundle(self.root, self.output, embedded)

    def test_relocation_is_explicitly_refused(self):
        self.seal()
        relocated = self.root / 'relocated'
        relocated.mkdir()
        with self.assertRaisesRegex(ValueError, 'relocation'):
            bundle.verify_bundle(relocated, self.output, self.authority)

    def test_conflicting_destination_is_not_overwritten(self):
        self.seal()
        target = self.root / bundle.evidence.PREFIXES['curl'] / 'bin/libcurl.dll'
        target.write_bytes(b'current changed file')
        with self.assertRaises(ValueError):
            bundle.restore(self.root, self.output, self.authority)
        self.assertEqual(target.read_bytes(), b'current changed file')

    def test_changed_native_image_or_python_toolchain_rejects(self):
        self.seal()
        with mock.patch.dict(os.environ, {'ImageVersion': '20261002.1.0'}):
            with self.assertRaisesRegex(ValueError, 'runner image differs'):
                bundle.verify_bundle(self.root, self.output, self.authority)
        changed = {**bundle._python_identity(), 'sha256': 'c' * 64}
        with mock.patch.object(bundle, '_python_identity', return_value=changed):
            with self.assertRaisesRegex(ValueError, 'Python'):
                bundle.verify_bundle(self.root, self.output, self.authority)

    def consumer_upgrade(self):
        self.identity['headSha'] = bundle.COMPATIBLE_PRODUCER
        document = self.seal()
        current = {}
        records = {}
        for name in bundle.CONSUMER_FILES:
            path = self.root / name
            current[name] = path.read_bytes() + b'\n# owned consumer correction fixture\n'
            path.write_bytes(current[name])
            records[name] = {'originalSha256': document['files'][name]['sha256'],
                             'currentSha256': receipt._digest(path)}
        policy = {'schemaVersion': 1, 'entries': [{
            'producerCommit': bundle.COMPATIBLE_PRODUCER, 'files': records,
            'reason': 'owned consumer-only runtime validation correction'}]}
        self._write_json(self.root / bundle.CONSUMER_COMPATIBILITY, policy)
        return document, policy, current

    def test_exact_consumer_correction_preserves_original_sdk_and_current_helpers(self):
        document, policy, current = self.consumer_upgrade()
        manifest_bytes = (self.output / 'bundle.json').read_bytes()
        originals = {name: (self.output / 'payload' / name).read_bytes() for name in bundle.CONSUMER_FILES}
        shutil.rmtree(self.root / bundle.evidence.PREFIXES['openssl'])
        bundle.restore(self.root, self.output, self.authority)
        self.assertEqual(bundle.verify_restored(self.root, self.output, self.authority), document)
        self.assertEqual((self.output / 'bundle.json').read_bytes(), manifest_bytes)
        for name in bundle.CONSUMER_FILES:
            self.assertEqual((self.root / name).read_bytes(), current[name])
            self.assertEqual((self.output / 'payload' / name).read_bytes(), originals[name])
        (self.root / bundle.stage.CACHE).mkdir(parents=True)
        bundle.stage_bundle(self.root, self.output, self.authority)

    def test_consumer_correction_rejects_other_commit_hash_or_input(self):
        document, policy, current = self.consumer_upgrade()
        wrong_commit = {**document, 'producer': {**document['producer'], 'headSha': 'b' * 40}}
        with self.assertRaisesRegex(ValueError, 'producer commit'):
            bundle._consumer_compatibility(self.root, wrong_commit)
        for name in sorted(bundle.CONSUMER_FILES):
            for field in ('originalSha256', 'currentSha256'):
                changed = json.loads(json.dumps(policy))
                changed['entries'][0]['files'][name][field] = 'c' * 64
                self._write_json(self.root / bundle.CONSUMER_COMPATIBILITY, changed)
                with self.assertRaisesRegex(ValueError, 'exact reviewed compatibility'):
                    bundle.verify_bundle(self.root, self.output, self.authority)
        self._write_json(self.root / bundle.CONSUMER_COMPATIBILITY, policy)
        helper = self.root / 'tools/windows_dependency_bundle.py'
        helper.write_bytes(current['tools/windows_dependency_bundle.py'] + b'# unapproved\n')
        with self.assertRaisesRegex(ValueError, 'exact reviewed compatibility'):
            bundle.verify_bundle(self.root, self.output, self.authority)
        helper.write_bytes(current['tools/windows_dependency_bundle.py'])
        producer = self.root / 'tools/build-curl-windows.ps1'
        producer.write_bytes(producer.read_bytes() + b'# actual producer changed\n')
        with self.assertRaisesRegex(ValueError, 'inputs changed'):
            bundle.verify_bundle(self.root, self.output, self.authority)

    def test_consumer_correction_cannot_expand_to_producer_scripts(self):
        document, policy, current = self.consumer_upgrade()
        policy['entries'][0]['files']['tools/build-curl-windows.ps1'] = {
            'originalSha256': 'c' * 64, 'currentSha256': 'd' * 64}
        self._write_json(self.root / bundle.CONSUMER_COMPATIBILITY, policy)
        with self.assertRaisesRegex(ValueError, 'unreviewed consumer compatibility scope'):
            bundle.verify_bundle(self.root, self.output, self.authority)

    def test_repository_policy_pins_current_consumer_bytes(self):
        source = Path(__file__).resolve().parent.parent
        policy = json.loads((source / bundle.CONSUMER_COMPATIBILITY).read_text())
        actual = {name: receipt._digest(source / name) for name in bundle.CONSUMER_FILES}
        matches = [entry for entry in policy['entries'] if all(
            entry['files'][name]['currentSha256'] == actual[name] for name in bundle.CONSUMER_FILES)]
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0]['producerCommit'], bundle.COMPATIBLE_PRODUCER)

    def test_live_tool_reprobe_is_required_in_orchestrator(self):
        text = Path(__file__).with_name('build-pristine-windows.ps1').read_text()
        branch = text.split('if ($BundleRequested) {\n            $BundleArguments', 1)[1].split('} elseif ($ReuseVerifiedDependencies)', 1)[0]
        self.assertLess(branch.index("'restore'"), branch.index('-VerifyToolFactsOnly'))
        self.assertEqual(branch.count('-VerifyToolFactsOnly'), 2)
        self.assertLess(branch.rindex('-VerifyToolFactsOnly'), branch.index('verify-curl-bundle.ps1'))
        self.assertLess(branch.index('verify-curl-bundle.ps1'), branch.index("'stage'"))
        probe = Path(__file__).with_name('verify-curl-bundle.ps1').read_text()
        self.assertEqual(probe.count('Checked-Python $VerificationArguments'), 2)
        self.assertIn("'build-curl-windows.ps1'", probe)
        self.assertIn('-Mode Verify -Kind curl-parent -Output $Facts -ProducerScript $ProducerScript', probe)
        self.assertNotIn('-Mode Capture', probe)



def load_tests(loader, tests, pattern):
    return unittest.TestSuite(BundleTests(name) for name in sorted(BundleTests.__dict__) if name.startswith('test_'))


if __name__ == '__main__':
    unittest.main()
