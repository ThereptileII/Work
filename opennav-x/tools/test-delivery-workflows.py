#!/usr/bin/env python3
"""Offline delivery gate and workflow-policy regressions; inert package fixtures only."""
import copy
import fnmatch
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest

import yaml
import release_manifest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools'


def workflow_directory(root):
    # Local checkouts keep workflows beside tools; published checkouts nest
    # this project under opennav-x/ while workflows remain at repository root.
    for directory in (root / '.github/workflows', root.parent / '.github/workflows'):
        if directory.is_dir():
            return directory
    raise FileNotFoundError('Delivery workflow directory missing from project and repository roots')


WORKFLOWS = workflow_directory(ROOT)
COMMIT = '1' * 40
HARNESS = '2' * 40


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('-', '_'), TOOLS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fixture_module = load('test-release-manifest')
staging = load('staging-qualification')
production = load('production-qualification')
retention = load('retain-release-inputs')
preparation = load('prepare-release-retest')


class StrictLoader(yaml.SafeLoader):
    """Reject shadowed security keys and retain GitHub's YAML 1.2 `on` spelling."""
    yaml_implicit_resolvers = {
        key: [(tag, pattern) for tag, pattern in values if tag != 'tag:yaml.org,2002:bool']
        for key, values in yaml.SafeLoader.yaml_implicit_resolvers.items()
    }

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in result:
                raise ValueError('Duplicate YAML key: ' + str(key))
            result[key] = self.construct_object(value_node, deep=deep)
        return result


StrictLoader.add_implicit_resolver('tag:yaml.org,2002:bool', re.compile(r'^(?:true|false|True|False|TRUE|FALSE)$'), list('tTfF'))


def workflow(name):
    return yaml.load((WORKFLOWS / name).read_text(), Loader=StrictLoader)


def snapshot(directory):
    return {path.name: path.read_bytes() for path in directory.iterdir()}


class PackageGates(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture_module.ManifestTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.directory = self.fixture.root
        self.extra = tempfile.TemporaryDirectory()
        self.addCleanup(self.extra.cleanup)
        self.work = Path(self.extra.name)
        self.evidence = self.work / 'evidence'
        self.evidence.mkdir()
        self.env = dict(GITHUB_SHA=COMMIT, GITHUB_RUN_ID='42', GITHUB_RUN_ATTEMPT='1')
        self.results = {name: {'result': 'success'} for name in staging.REQUIRED_JOBS}

    def stage(self):
        return staging.qualify(self.directory, self.results, self.env)

    def production_receipts(self):
        self.stage()
        self.env.update(GITHUB_SHA=HARNESS, GITHUB_RUN_ID='99', GITHUB_RUN_ATTEMPT='2')
        values = {
            'installer-lifecycle.json': dict(status='passed', mode='production', product_commit=COMMIT,
                harness_commit=HARNESS, setup_sha256=production.digest(self.directory / 'SKAGER-Beta2-Setup.exe')),
            'production-recovery-results.json': dict(status='passed', build={'commit': COMMIT}, harness_commit=HARNESS,
                package_sha256=production.digest(self.directory / 'SKAGER-Beta2-Portable-Recovery.zip')),
            'production-functional.json': dict(status='passed', commit=COMMIT, harnessCommit=HARNESS,
                                               designReview='not-requested'),
        }
        self.write_receipts(values)
        return values

    def write_receipts(self, values):
        for name, report in values.items():
            (self.evidence / name).write_text(json.dumps(report))

    def recreate_manifest(self):
        (self.directory / 'RELEASE.json').unlink()
        self.fixture.create()

    def test_staging_missing_failed_skipped_or_cancelled_required_jobs_write_nothing(self):
        original = snapshot(self.directory)
        for name in staging.REQUIRED_JOBS:
            for state in (None, 'failure', 'skipped', 'cancelled'):
                with self.subTest(job=name, state=state):
                    results = copy.deepcopy(self.results)
                    if state is None:
                        results.pop(name)
                    else:
                        results[name]['result'] = state
                    with self.assertRaises(ValueError):
                        staging.qualify(self.directory, results, self.env)
                    self.assertEqual(snapshot(self.directory), original)

    def test_staging_success_binds_identity_and_preserves_product_bytes(self):
        original = snapshot(self.directory)
        record = self.stage()
        self.assertEqual(record['commit'], COMMIT)
        self.assertEqual(record['runId'], '42')
        self.assertEqual(record['runAttempt'], '1')
        self.assertEqual(record['channel'], 'staging')
        qualification = json.loads((self.directory / 'QUALIFICATION.json').read_text())
        self.assertIs(qualification['publicAccess'], False)
        self.assertEqual(qualification['endurance'], 'skipped')
        self.assertEqual(qualification['designReview'], 'not-requested')
        self.assertEqual({name: (self.directory / name).read_bytes() for name in original}, original)
        self.assertEqual(release_manifest.verify(self.directory), record)

    def test_staging_requested_design_is_recorded_without_claiming_a_pass(self):
        self.env['SKAGER_DESIGN_VALIDATION'] = 'true'
        record = self.stage()
        qualification = json.loads((self.directory / 'QUALIFICATION.json').read_text())
        self.assertEqual(record['designReview'], 'requested')
        self.assertEqual(qualification['designReview'], 'requested')
        self.assertNotIn('design', qualification['gates'])
        release_manifest.verify(self.directory)

    def test_production_success_binds_package_and_separate_harness_without_mutation(self):
        self.production_receipts()
        original = snapshot(self.directory)
        report = production.qualify(self.directory, self.evidence, self.env)
        self.assertEqual(report['status'], 'passed')
        self.assertEqual(report['commit'], COMMIT)
        self.assertEqual(report['harnessCommit'], HARNESS)
        self.assertEqual(report['manifestSha256'], production.digest(self.directory / 'RELEASE.json'))
        self.assertEqual(report['setupSha256'], production.digest(self.directory / 'SKAGER-Beta2-Setup.exe'))
        self.assertEqual(report['qualificationRunId'], '99')
        self.assertEqual(report['runAttempt'], '2')
        self.assertEqual(snapshot(self.directory), original)

    def test_production_rejects_failed_missing_or_incomplete_receipts(self):
        reports = self.production_receipts()
        for name in reports:
            for state in ('failed', 'running', None):
                with self.subTest(receipt=name, state=state):
                    changed = copy.deepcopy(reports)
                    if state is None:
                        changed[name].pop('status')
                    else:
                        changed[name]['status'] = state
                    self.write_receipts(changed)
                    with self.assertRaises(ValueError):
                        production.qualify(self.directory, self.evidence, self.env)
            self.write_receipts(reports)
            (self.evidence / name).unlink()
            with self.assertRaises(FileNotFoundError):
                production.qualify(self.directory, self.evidence, self.env)
            self.write_receipts(reports)

    def test_production_rejects_wrong_bytes_mode_product_or_harness(self):
        reports = self.production_receipts()
        cases = [
            ('installer-lifecycle.json', 'setup_sha256', 'f' * 64),
            ('installer-lifecycle.json', 'mode', 'alpha'),
            ('installer-lifecycle.json', 'product_commit', HARNESS),
            ('installer-lifecycle.json', 'harness_commit', COMMIT),
            ('production-recovery-results.json', 'package_sha256', 'f' * 64),
            ('production-recovery-results.json', 'build', {'commit': HARNESS}),
            ('production-recovery-results.json', 'harness_commit', COMMIT),
            ('production-functional.json', 'commit', HARNESS),
            ('production-functional.json', 'harnessCommit', COMMIT),
        ]
        for name, key, value in cases:
            with self.subTest(receipt=name, field=key):
                changed = copy.deepcopy(reports)
                changed[name][key] = value
                self.write_receipts(changed)
                with self.assertRaises(ValueError):
                    production.qualify(self.directory, self.evidence, self.env)
        self.write_receipts(reports)

    def test_production_rejects_missing_inherited_linux_even_with_valid_manifest(self):
        self.production_receipts()
        path = self.directory / 'QUALIFICATION.json'
        report = json.loads(path.read_text())
        report['gates']['linux'] = 'failed'
        path.write_text(json.dumps(report))
        self.recreate_manifest()
        release_manifest.verify(self.directory)
        with self.assertRaises(ValueError):
            production.qualify(self.directory, self.evidence, self.env)

    def retain_support(self, support_attempt='1'):
        root = self.work / 'source'
        for name in ('build/beta-installer/package.json', 'build/beta-installer/payload.zip',
                     'build/production-windows/include/config.h', 'build/xnav-windows/include/config.h',
                     'build/xnav-install/opencpn.exe'):
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'Inert retained input: ' + name.encode())
        (root / 'build/beta-artifacts').mkdir(parents=True)
        receipt = retention.retain(root, COMMIT, '42', support_attempt)
        (self.directory / 'RETEST_SUPPORT.json').write_text(json.dumps(receipt))
        self.stage()
        return root / 'build/release-retest', receipt

    def test_retained_support_preparation_preserves_original_release(self):
        support, receipt = self.retain_support()
        original = snapshot(self.directory)
        output = self.work / 'retest'
        result = preparation.prepare(self.directory, support, output)
        self.assertEqual(result['commit'], COMMIT)
        self.assertEqual(snapshot(self.directory), original)
        self.assertEqual((output / receipt['archiveName']).read_bytes(), (support / receipt['archiveName']).read_bytes())
        self.assertEqual((output / 'RELEASE.json').read_bytes(), original['RELEASE.json'])
        self.assertEqual((output / 'SKAGER-Beta2-Setup.exe').read_bytes(), original['SKAGER-Beta2-Setup.exe'])
        self.assertIn(receipt['sha256'] + '  ' + receipt['archiveName'], (output / 'SHA256SUMS.txt').read_text())
        release_manifest.verify(self.directory)

    def test_retained_support_prior_qualification_attempt_survives_publish_retry(self):
        self.env['GITHUB_RUN_ATTEMPT'] = '3'
        support, receipt = self.retain_support(support_attempt='2')
        original = snapshot(self.directory)
        output = self.work / 'prior-attempt-retest'
        result = preparation.prepare(self.directory, support, output)
        self.assertEqual(result['runAttempt'], '3')
        self.assertEqual(snapshot(self.directory), original)
        retained = json.loads((output / 'RETEST_SUPPORT.json').read_text())
        self.assertEqual(retained['runAttempt'], '2')
        self.assertEqual(retained['artifactName'], 'staging-retest-' + COMMIT + '-attempt2')
        self.assertEqual(retained['sha256'], receipt['sha256'])
        self.assertEqual((output / receipt['archiveName']).read_bytes(),
                         (support / receipt['archiveName']).read_bytes())
        for name in release_manifest.PRODUCT_FILES | {'RELEASE.json', 'RETEST_SUPPORT.json'}:
            self.assertEqual((output / name).read_bytes(), original[name])

    def test_retained_support_same_size_tamper_refused_before_copy(self):
        support, receipt = self.retain_support()
        original = snapshot(self.directory)
        archive = support / receipt['archiveName']
        data = archive.read_bytes()
        archive.write_bytes(bytes([data[0] ^ 1]) + data[1:])
        output = self.work / 'retest'
        with self.assertRaises(ValueError):
            preparation.prepare(self.directory, support, output)
        self.assertFalse(output.exists())
        self.assertEqual(snapshot(self.directory), original)

    def test_retained_support_wrong_run_receipt_refused_even_when_manifest_valid(self):
        support, receipt = self.retain_support()
        receipt['runId'] = '43'
        (self.directory / 'RETEST_SUPPORT.json').write_text(json.dumps(receipt))
        self.recreate_manifest()
        release_manifest.verify(self.directory)
        with self.assertRaises(ValueError):
            preparation.prepare(self.directory, support, self.work / 'retest')
        self.assertFalse((self.work / 'retest').exists())


    def test_retained_support_wrong_attempt_refused_even_when_manifest_valid(self):
        support, receipt = self.retain_support()
        receipt['runAttempt'] = '2'
        receipt['artifactName'] = 'staging-retest-' + COMMIT + '-attempt2'
        (self.directory / 'RETEST_SUPPORT.json').write_text(json.dumps(receipt))
        self.recreate_manifest()
        release_manifest.verify(self.directory)
        with self.assertRaises(ValueError):
            preparation.prepare(self.directory, support, self.work / 'retest')
        self.assertFalse((self.work / 'retest').exists())


class WorkflowPolicy(unittest.TestCase):
    def test_workflow_directory_supports_local_and_published_layouts_and_refuses_missing(self):
        with tempfile.TemporaryDirectory() as temporary:
            repository = Path(temporary)
            project = repository / 'opennav-x'
            project.mkdir()
            with self.assertRaises(FileNotFoundError):
                workflow_directory(project)
            published = repository / '.github/workflows'
            published.mkdir(parents=True)
            self.assertEqual(workflow_directory(project), published)
            local = project / '.github/workflows'
            local.mkdir(parents=True)
            self.assertEqual(workflow_directory(project), local)

    def test_strict_yaml_rejects_duplicate_security_keys(self):
        for sample in ('permissions:\n  contents: read\n  contents: write\n',
                       'on:\n  push:\n  push:\n', 'jobs:\n  promote:\n    if: false\n    if: true\n'):
            with self.assertRaises(ValueError):
                yaml.load(sample, Loader=StrictLoader)
        parsed = yaml.load('on:\n  workflow_dispatch:\n    default: false\n', Loader=StrictLoader)
        self.assertIn('on', parsed)
        self.assertIs(parsed['on']['workflow_dispatch']['default'], False)
        paths = list(WORKFLOWS.glob('*.yml'))
        self.assertTrue(paths, 'Delivery workflow directory contains no YAML workflows')
        for path in paths:
            with self.subTest(workflow=path.name):
                yaml.load(path.read_text(), Loader=StrictLoader)

    def test_production_is_manual_and_promotes_only_after_qualification(self):
        data = workflow('skager-production.yml')
        self.assertEqual(set(data['on']), {'workflow_dispatch'})
        for name in ('staging_release', 'confirmation', 'instruction'):
            self.assertIs(data['on']['workflow_dispatch']['inputs'][name]['required'], True)
        self.assertEqual(data['env']['SKAGER_DESIGN_VALIDATION'], 'false')
        self.assertNotIn('design_validation', data['on']['workflow_dispatch']['inputs'])
        self.assertIs(data['concurrency']['cancel-in-progress'], False)
        self.assertEqual(set(data['jobs']['promote']['needs']), {'inspect', 'qualify'})
        self.assertEqual(data['jobs']['qualify']['needs'], 'inspect')
        self.assertEqual(data['jobs']['qualify']['name'], 'Qualify retained Windows package')
        self.assertEqual(data['jobs']['promote']['environment'], 'production')
        for job in data['jobs'].values():
            self.assertNotIn('continue-on-error', job)
            for step in job['steps']:
                run = step.get('run', '')
                self.assertNotRegex(run, r'(?i)(build-pristine|cmake\s+(?:--build|-S)|msbuild|package-alpha-installer|package-preview|tools/prototype/|capture-dpi)')
                self.assertNotIn('continue-on-error', step)
                self.assertNotIn('gh release', run)

    def test_automatic_delivery_has_one_branch_and_historical_builds_remain_manual(self):
        baseline = workflow('opennav-baseline.yml')
        helpers = workflow('skager-delivery-checks.yml')
        self.assertEqual(baseline['on']['push']['branches'], ['staging'])
        self.assertEqual(set(helpers['on']['push']['branches']),
                         {'staging', 'skager-delivery-workflow'})
        for data in (baseline, helpers):
            self.assertIn('workflow_dispatch', data['on'])
            self.assertNotIn('branches-ignore', data['on']['push'])
        # Manual selection on a historical source branch still forces the same
        # Staging product gates; the branch restriction applies only to pushes.
        selector = baseline['jobs']['changes']
        step = next(item for item in selector['steps'] if item.get('id') == 'select')
        self.assertEqual(step['env']['MANUAL'],
                         "${{ github.event_name == 'workflow_dispatch' && 'true' || 'false' }}")
        self.assertIn("if os.environ['MANUAL'] == 'true': command.append('--force-product')", step['run'])

    def test_staging_defaults_and_required_dependencies_keep_design_opt_in(self):
        data = workflow('opennav-baseline.yml')
        inputs = data['on']['workflow_dispatch']['inputs']
        self.assertIs(inputs['design_validation']['default'], False)
        self.assertIs(inputs['extended_tests']['default'], False)
        self.assertIn("github.event_name == 'workflow_dispatch'", data['env']['SKAGER_DESIGN_VALIDATION'])
        self.assertIn("|| 'false'", data['env']['SKAGER_DESIGN_VALIDATION'])
        job = data['jobs']['publish-staging']
        self.assertTrue(set(staging.REQUIRED_JOBS) <= set(job['needs']))
        for required in staging.REQUIRED_JOBS:
            self.assertNotIn('extended_tests', str(data['jobs'][required].get('if', '')))
            self.assertNotIn('design_validation', str(data['jobs'][required].get('if', '')))
        runs = [step.get('run', '') for step in job['steps']]
        qualify_index = next(i for i, run in enumerate(runs) if 'staging-qualification.py' in run)
        publish_index = next(i for i, run in enumerate(runs) if 'publish-staging --directory' in run)
        self.assertLess(qualify_index, publish_index)
        self.assertNotIn('always()', str(job.get('if', '')))

    def test_changed_inputs_gate_product_jobs_and_manual_runs_force_selection(self):
        jobs = workflow('opennav-baseline.yml')['jobs']
        selector = jobs['changes']
        self.assertEqual(selector['outputs']['product'], '${{ steps.select.outputs.product }}')
        checkout = next(step for step in selector['steps'] if step.get('uses','').startswith('actions/checkout@'))
        self.assertEqual(checkout['with']['fetch-depth'], 0)
        command = '\n'.join(step.get('run','') for step in selector['steps'])
        self.assertIn('ci_changes.py', command)
        self.assertIn('--force-product', command)
        for name in set(staging.REQUIRED_JOBS) - {'publish-staging'}:
            job = jobs[name]
            self.assertIn("needs.changes.outputs.product == 'true'", job.get('if',''), name)
            needed = job.get('needs',[])
            self.assertIn('changes', [needed] if isinstance(needed,str) else needed, name)
            self.assertNotIn('always()', job.get('if',''), name)

    def test_helper_changes_have_focused_checks_when_native_jobs_skip(self):
        data = workflow('skager-delivery-checks.yml')
        patterns = data['on']['push']['paths']
        sources = ('ci_changes.py','staging_build_inputs.py','fetch_ci_inputs.py',
                   'qualify-staging-windows.ps1','retest-staging-windows.py')
        for source in sources:
            self.assertTrue(any(fnmatch.fnmatchcase('opennav-x/tools/'+source, pattern)
                                for pattern in patterns), source)
        commands = '\n'.join(step.get('run','') for job in data['jobs'].values() for step in job['steps'])
        for suite in ('test-ci-changes.py','test-staging-build-inputs.py','test-fetch-ci-inputs.py'):
            self.assertIn(suite, commands)
        self.assertNotRegex(commands, r'(?im)^\s*(?:python\s+)?[^\n]*[ /]build-pristine-windows\.ps1(?:\s|$)')

    def test_compiled_inputs_are_retained_before_runtime_qualification(self):
        jobs = workflow('opennav-baseline.yml')['jobs']
        producer = jobs['windows-integration']
        self.assertEqual(producer['name'], 'windows-integration')
        steps = producer['steps']
        commands = [step.get('run','') for step in steps]
        seal = next(i for i, text in enumerate(commands) if 'staging_build_inputs.py seal ' in text)
        upload = next(i for i, step in enumerate(steps) if step.get('uses','').startswith('actions/upload-artifact@')
                      and step.get('with',{}).get('path') == 'staging-build')
        self.assertLess(seal, upload)
        compilers = [text for text in commands if 'build-pristine-windows.ps1' in text]
        self.assertEqual(len(compilers), 2)
        for command in compilers:
            self.assertIn('-DeferRuntimeQualification', command)
            self.assertIn('-DependencyBundle ', command)
            self.assertIn('-DependencyBundleProvenance ', command)
        recovery = next(i for i, text in enumerate(commands) if 'package-preview-windows.ps1' in text)
        installer = next(i for i, text in enumerate(commands) if 'package-alpha-installer.py' in text)
        self.assertLess(recovery, seal); self.assertLess(installer, seal)
        self.assertIn('-DeferRuntimeQualification', commands[recovery])
        for command in commands:
            self.assertNotRegex(command, r'smoke-(?:navigation|modes|pilot|portable|installer-windows|charts|preview|user-flows)')
            self.assertNotIn('--fixture-success', command)
            self.assertNotIn('test-boat-feedback-widgets.py', command)
        downstream = jobs['windows-qualification']
        self.assertIn('windows-integration', downstream['needs'])
        restore = next(i for i, step in enumerate(downstream['steps']) if 'staging_build_inputs.py restore ' in step.get('run',''))
        qualify = next(i for i, step in enumerate(downstream['steps']) if 'qualify-staging-windows.ps1' in step.get('run',''))
        self.assertLess(restore, qualify)

    def test_qualification_cannot_rebuild_repackage_or_downgrade_required_results(self):
        jobs = workflow('opennav-baseline.yml')['jobs']
        qualifier = jobs['windows-qualification']
        self.assertIn('windows-qualification', staging.REQUIRED_JOBS)
        self.assertIn('windows-qualification', jobs['publish-staging']['needs'])
        self.assertNotIn('continue-on-error', qualifier)
        commands = []
        for step in qualifier['steps']:
            self.assertNotIn('continue-on-error', step)
            commands.append(step.get('run',''))
        commands.append((TOOLS/'qualify-staging-windows.ps1').read_text())
        for command in commands:
            self.assertNotRegex(command, r'(?i)(build-pristine|cmake\s+(?:--build|-S)|msbuild|package-alpha-installer|package-preview|makensis)')
        runner = commands[-1]
        for script in ('test-boat-feedback-widgets.py','smoke-installer-selftest.py','smoke-modes-windows.py','smoke-navigation.py',
                       'smoke-signalk.py','smoke-recording.py','smoke-pilot.py','smoke-recovery.py',
                       'smoke-user-flows.py','smoke-portable-production.py','smoke-installer-windows.py','smoke-charts.py'):
            self.assertIn("Check '"+script+"'", runner)
        self.assertIn("$env:SKAGER_DESIGN_VALIDATION -ceq 'true'", runner)
        self.assertIn("@('--mode', 'staging')", runner)

    def test_restore_uses_actual_producer_attempt_and_authenticated_digest(self):
        jobs = workflow('opennav-baseline.yml')['jobs']
        producer = jobs['windows-integration']
        for name in ('archive_sha256','artifact_name','producer_attempt'):
            self.assertIn(name, producer['outputs'])
        qualifier = jobs['windows-qualification']
        download = next(step for step in qualifier['steps'] if step.get('uses','').startswith('actions/download-artifact@'))
        self.assertEqual(download['with']['name'], '${{ needs.windows-integration.outputs.artifact_name }}')
        restore = next(step for step in qualifier['steps'] if 'staging_build_inputs.py restore ' in step.get('run',''))
        self.assertEqual(restore['env']['INPUT_SHA256'], '${{ needs.windows-integration.outputs.archive_sha256 }}')
        self.assertEqual(restore['env']['PRODUCER_ATTEMPT'], '${{ needs.windows-integration.outputs.producer_attempt }}')
        self.assertIn('--archive-sha256 $env:INPUT_SHA256', restore['run'])
        self.assertIn('--producer-run-attempt $env:PRODUCER_ATTEMPT', restore['run'])
        self.assertNotIn('--producer-run-attempt $env:GITHUB_RUN_ATTEMPT', restore['run'])

    def test_publish_retry_downloads_actual_qualification_artifact(self):
        jobs = workflow('opennav-baseline.yml')['jobs']
        downloads = [step['with']['name'] for step in jobs['publish-staging']['steps']
                     if step.get('uses','').startswith('actions/download-artifact@')]
        self.assertIn('${{ needs.windows-qualification.outputs.candidate_artifact }}', downloads)
        self.assertIn('candidate_artifact', jobs['windows-qualification'].get('outputs',{}))
        for name in downloads:
            self.assertNotIn('github.run_attempt', name)

    def test_prototype_is_manual_and_all_jobs_require_explicit_design(self):
        data = workflow('opennav-prototype.yml')
        self.assertEqual(set(data['on']), {'workflow_dispatch'})
        self.assertIs(data['on']['workflow_dispatch']['inputs']['design_validation']['default'], False)
        for name, job in data['jobs'].items():
            with self.subTest(job=name):
                self.assertIn('inputs.design_validation == true', job.get('if', ''))


if __name__ == '__main__':
    unittest.main()
