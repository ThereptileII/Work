#!/usr/bin/env python3
"""Exercise the actual dependency workflow's early image-selection program."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


class DependencyImageSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        repo = Path(__file__).resolve().parents[2]
        workflow = yaml.safe_load((repo / '.github/workflows/skager-windows-dependencies.yml').read_text())
        cls.steps = workflow['jobs']['windows-dependencies']['steps']
        cls.guard = next(step for step in cls.steps if step.get('name') ==
                         'Check the explicitly selected Windows image before setup')

    def exercise(self, expected, actual='20261012.333.1', **override):
        env = {**os.environ, 'SKAGER_EXPECTED_WINDOWS_IMAGE': expected,
               'ImageVersion': actual, 'ImageOS': 'win22',
               'RUNNER_OS': 'Windows', 'RUNNER_ARCH': 'X64', **override}
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, '-c', self.guard['run']],
                                    cwd=directory, env=env, capture_output=True,
                                    text=True, timeout=10)
            report = json.loads((Path(directory) /
                'opennav-x/evidence/local/dependency-image-selection.json').read_text())
            self.assertEqual(report['expectedImage'], expected)
            self.assertEqual(report['actualImage'], actual)
            return result, report

    def test_exact_selected_image_passes_without_changing_observation(self):
        result, report = self.exercise('20261012.333.1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(report['accepted'])

    def test_different_image_stops_before_dependency_build(self):
        result, report = self.exercise('20261004.326.1')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(report['accepted'])
        self.assertIn('no dependency build started', result.stderr)

    def test_unpinned_first_producer_still_requires_valid_native_identity(self):
        result, report = self.exercise('')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(report['accepted'])
        for override in ({'RUNNER_ARCH': 'ARM64'}, {'RUNNER_OS': 'Linux'},
                         {'ImageOS': 'ubuntu24'}, {'ImageVersion': ''}):
            with self.subTest(override=override):
                actual = override.pop('ImageVersion', '20261012.333.1')
                result, report = self.exercise('', actual, **override)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(report['accepted'])

    def test_malformed_or_pattern_selection_cannot_widen_match(self):
        for expected in ('*', '20261012.*', '20261012.333.1\n', ' 20261012.333.1'):
            with self.subTest(expected=expected):
                result, report = self.exercise(expected)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(report['accepted'])

    def test_guard_is_before_all_setup_and_build_steps(self):
        self.assertEqual(self.steps.index(self.guard), 1)
        self.assertNotIn('if', self.guard)
        self.assertEqual(self.guard['env']['SKAGER_EXPECTED_WINDOWS_IMAGE'],
                         '${{ inputs.expected_image }}')
        self.assertEqual(self.steps[0]['uses'].split('@')[0], 'actions/checkout')


if __name__ == '__main__':
    unittest.main()
