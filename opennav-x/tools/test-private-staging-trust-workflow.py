#!/usr/bin/env python3
"""Offline workflow dataflow checks; execute only an inert packaging substitute."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = ROOT / '.github/workflows/opennav-baseline.yml'
CONFIG = 'opennav-x/installer/windows/staging-update-trust.json'
VALIDATOR = 'opennav-x/build/updater-qualified/skager-repository.exe'
RECORD = 'opennav-x/build/updater-qualified/skager-repository.json'
SELECTION = "${{ github.event_name == 'workflow_dispatch' && inputs.private_staging_trust && 'true' || 'false' }}"


class StrictLoader(yaml.SafeLoader):
    # Preserve GitHub's YAML 1.2 `on`, while decoding true/false as booleans.
    yaml_implicit_resolvers = {
        key: [(tag, pattern) for tag, pattern in values
              if tag != 'tag:yaml.org,2002:bool']
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


StrictLoader.add_implicit_resolver(
    'tag:yaml.org,2002:bool', re.compile(r'^(?:true|false|True|False|TRUE|FALSE)$'), list('tTfF'))


def workflow():
    return yaml.load(WORKFLOW.read_text(encoding='utf-8'), Loader=StrictLoader)


def package_step():
    steps = workflow()['jobs']['windows-integration']['steps']
    matches = [step for step in steps
               if step.get('name') == 'Package fixture-free recovery before desktop qualification']
    if len(matches) != 1:
        raise AssertionError('Expected one actual recovery packaging step')
    return matches[0]


class WorkflowContracts(unittest.TestCase):
    def test_manual_boolean_is_off_by_default_and_push_is_never_selected(self):
        document = workflow()
        selected = document['on']['workflow_dispatch']['inputs']['private_staging_trust']
        self.assertEqual(selected['type'], 'boolean')
        self.assertIs(selected['default'], False)
        self.assertEqual(document['on']['push']['branches'], ['staging'])
        self.assertEqual(package_step()['env'], {'SKAGER_PRIVATE_STAGING_TRUST': SELECTION})

    def test_duplicate_keys_are_refused_including_nested_inputs(self):
        for sample in ('on: {}\non: {}\n',
                       'on:\n  workflow_dispatch:\n    inputs:\n      enabled: true\n      enabled: false\n'):
            with self.assertRaisesRegex(ValueError, 'Duplicate YAML key'):
                yaml.load(sample, Loader=StrictLoader)

    def test_validator_is_the_same_run_download_before_package(self):
        job = workflow()['jobs']['windows-integration']
        self.assertIn('updater-contracts', job['needs'])
        steps = job['steps']
        packages = [i for i, step in enumerate(steps) if step == package_step()]
        downloads = [(i, step) for i, step in enumerate(steps)
                     if step.get('with', {}).get('path') == 'opennav-x/build/updater-qualified']
        self.assertEqual(len(downloads), 1)
        index, download = downloads[0]
        self.assertLess(index, packages[0])
        self.assertTrue(download['uses'].startswith('actions/download-artifact@'))
        self.assertEqual(download['with'], {
            'name': 'update-verifier-windows-2022-386-${{ github.sha }}',
            'path': 'opennav-x/build/updater-qualified'})
        self.assertNotIn('if', download)
        # No alternate run, repository, token or caller-selected path is supplied.
        self.assertNotIn('secrets.', json.dumps(package_step()))

    def test_focused_test_is_registered(self):
        steps = workflow()['jobs']['contracts']['steps']
        self.assertEqual(sum('test-private-staging-trust-workflow.py' in step.get('run', '')
                             for step in steps), 1)


@unittest.skipUnless(shutil.which('pwsh'), 'pwsh required for inert workflow-script execution')
class PackageDataflow(unittest.TestCase):
    def invoke(self, selection, missing=None, directory=False, exit_code=0):
        with tempfile.TemporaryDirectory(prefix='staging-trust-workflow-') as temporary:
            root = Path(temporary)
            for name in (CONFIG, VALIDATOR, RECORD):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                if name == missing:
                    if directory:
                        path.mkdir()
                else:
                    path.write_text('inert input; never executed', encoding='utf-8')
            stub = root / 'opennav-x/tools/package-preview-windows.ps1'
            stub.parent.mkdir(parents=True)
            stub.write_text('''param([switch]$DeferRuntimeQualification,[string]$UpdateTrustConfig,[string]$UpdateTrustValidator)
[IO.File]::WriteAllText((Join-Path (Get-Location) 'called.json'),
    (ConvertTo-Json @{defer=[bool]$DeferRuntimeQualification;config=$UpdateTrustConfig;validator=$UpdateTrustValidator}))
exit ([int]$env:FIXTURE_EXIT)
''', encoding='utf-8')
            script = root / 'workflow-step.ps1'
            script.write_text(package_step()['run'], encoding='utf-8')
            environment = dict(os.environ, FIXTURE_EXIT=str(exit_code))
            environment.pop('SKAGER_PRIVATE_STAGING_TRUST', None)
            if selection is not None:
                environment['SKAGER_PRIVATE_STAGING_TRUST'] = selection
            result = subprocess.run([shutil.which('pwsh'), '-NoProfile', '-NonInteractive',
                                     '-File', str(script)], cwd=root, env=environment,
                                    capture_output=True, text=True, timeout=30)
            called = root / 'called.json'
            return result, json.loads(called.read_text()) if called.exists() else None

    def test_selected_fixed_inputs_reach_actual_package_arguments(self):
        result, called = self.invoke('true')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(called, {'defer': True, 'config': CONFIG, 'validator': VALIDATOR})

    def test_unselected_keeps_no_trust_arguments_even_if_file_is_missing(self):
        for missing in (None, CONFIG, VALIDATOR, RECORD):
            with self.subTest(missing=missing):
                result, called = self.invoke('false', missing)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(called, {'defer': True, 'config': '', 'validator': ''})

    def test_missing_or_directory_input_denies_before_any_packaging(self):
        for missing in (CONFIG, VALIDATOR, RECORD):
            for directory in (False, True):
                with self.subTest(missing=missing, directory=directory):
                    result, called = self.invoke('true', missing, directory)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('Required private-Staging trust input missing', result.stderr)
                    self.assertIsNone(called)

    def test_ambiguous_selection_denies_before_packaging(self):
        for selection in (None, '', 'True', '1'):
            with self.subTest(selection=selection):
                result, called = self.invoke(selection)
                self.assertNotEqual(result.returncode, 0)
                self.assertIsNone(called)

    def test_package_failure_propagates(self):
        result, called = self.invoke('true', exit_code=7)
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertIsNotNone(called)


if __name__ == '__main__':
    unittest.main()
