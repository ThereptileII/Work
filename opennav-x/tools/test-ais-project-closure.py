#!/usr/bin/env python3
"""SCRUM-285 retained native MSBuild project regression; no compile or AIS execution."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'tests/fixtures/ais-projects-4dd'
SPEC = importlib.util.spec_from_file_location('ais_runtime', ROOT / 'tools/test-ais-runtime-windows.py')
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)
NATIVE_SPEC = importlib.util.spec_from_file_location('ais_project_native', ROOT / 'tools/test-ais-projects-native.py')
NATIVE = importlib.util.module_from_spec(NATIVE_SPEC)
NATIVE_SPEC.loader.exec_module(NATIVE)
MANIFEST = json.loads((FIXTURE / 'manifest.json').read_text())
PROJECT_DIRECTORY = None


def verified_projects(destination):
    if PROJECT_DIRECTORY is None:
        archive = FIXTURE / 'projects.zip'
        if hashlib.sha256(archive.read_bytes()).hexdigest() != MANIFEST['fixtureZipSha256']:
            raise ValueError('Fixture ZIP identity changed')
        with zipfile.ZipFile(archive) as z:
            if set(z.namelist()) != set(MANIFEST['members']) or len(z.namelist()) != len(MANIFEST['members']):
                raise ValueError('Fixture member set changed')
            payloads = {name: z.read(name) for name in MANIFEST['members']}
    else:
        expected = {Path(name).name for name in MANIFEST['members']}
        if {p.name for p in PROJECT_DIRECTORY.iterdir()} != expected:
            raise ValueError('Native retained project set changed')
        payloads = {name: (PROJECT_DIRECTORY / Path(name).name).read_bytes() for name in MANIFEST['members']}
    result = {}
    for name, data in payloads.items():
        if {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()} != MANIFEST['members'][name]:
            raise ValueError('Retained native project bytes changed: ' + name)
        path = destination / Path(name).name
        path.write_bytes(data)
        result[path.stem] = path
    return result


class ProjectTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.projects = verified_projects(Path(self.temp.name))
        self.path = self.projects['ais_session_native']

    def test_actual_eight_project_closure(self):
        self.assertEqual(set(GATE.project_closure(self.projects)), set(MANIFEST['expectedClosure']))

    def test_original_selector_reproduces_real_failure(self):
        nodes = ET.parse(self.path).findall('.//m:ProjectReference', GATE.NS)
        self.assertEqual(sum('Include' not in n.attrib for n in nodes), 4)
        self.assertEqual(sum('Include' in n.attrib for n in nodes), 4)
        with self.assertRaisesRegex(KeyError, 'Include'):
            _ = nodes[0].attrib['Include']

    def test_configuration_metadata_is_never_an_edge(self):
        tree = ET.parse(self.path)
        nodes = tree.findall('.//m:ItemDefinitionGroup/m:ProjectReference', GATE.NS)
        self.assertEqual(len(nodes), 4)
        for node in nodes:
            node.set('Include', r'C:\not-a-dependency\metadata-only.vcxproj')
        tree.write(self.path)
        self.assertEqual(set(GATE.project_closure(self.projects)), set(MANIFEST['expectedClosure']))

    def test_missing_real_include_refuses(self):
        tree = ET.parse(self.path)
        edge = tree.findall('.//m:ItemGroup/m:ProjectReference', GATE.NS)[-1]
        del edge.attrib['Include']
        tree.write(self.path)
        with self.assertRaisesRegex(KeyError, 'Include'):
            GATE.project_closure(self.projects)

    def test_missing_dependency_project_refuses(self):
        del self.projects['opennav_vessel']
        with self.assertRaisesRegex(KeyError, 'opennav_vessel'):
            GATE.project_closure(self.projects)

    def test_unknown_real_dependency_refuses(self):
        tree = ET.parse(self.path)
        tree.findall('.//m:ItemGroup/m:ProjectReference', GATE.NS)[-1].set('Include', r'C:\foreign\unknown.vcxproj')
        tree.write(self.path)
        with self.assertRaisesRegex(KeyError, 'unknown'):
            GATE.project_closure(self.projects)

    def test_downstream_runtime_and_foreign_source_guards_unchanged(self):
        source = (ROOT / 'tools/test-ais-runtime-windows.py').read_text()
        start = source.index("        report['projects']")
        end = source.index('        retained = sources', start)
        self.assertEqual(hashlib.sha256(source[start:end].encode()).hexdigest(),
                         MANIFEST['unchangedDownstreamSourceGuardSha256'])


class IdentityTests(unittest.TestCase):
    def test_terminal_native_with_active_sibling_and_identity_refusals(self):
        # Fake API payloads isolate identity/status policy; no HTTP or CI action.
        run = {'id': MANIFEST['originalRunId'], 'head_sha': MANIFEST['originalCommit'],
               'run_attempt': 1, 'status': 'in_progress', 'conclusion': None}
        job = {'id': MANIFEST['originalNativeJobId'], 'run_id': run['id'],
               'head_sha': run['head_sha'], 'run_attempt': 1,
               'status': 'completed', 'conclusion': 'failure', 'steps': [
                   {'number': 17, 'name': 'Capture successful same-job Windows dependency closure',
                    'status': 'completed', 'conclusion': 'success'},
                   {'number': 18, 'name': 'Native AIS observation and transport with same-job maintained TLS',
                    'status': 'completed', 'conclusion': 'failure'}]}
        NATIVE.validate_original_job(run, job, MANIFEST)
        without_optional_attempt = copy.deepcopy(job)
        del without_optional_attempt['run_attempt']
        NATIVE.validate_original_job(run, without_optional_attempt, MANIFEST)
        for field, value in [('id', 123), ('run_id', 123), ('head_sha', '0' * 40),
                             ('run_attempt', 2), ('status', 'in_progress'), ('conclusion', 'success')]:
            with self.subTest(job_field=field):
                bad = copy.deepcopy(job); bad[field] = value
                with self.assertRaises(ValueError): NATIVE.validate_original_job(run, bad, MANIFEST)
        for field, value in [('id', 123), ('head_sha', '0' * 40), ('run_attempt', 2)]:
            with self.subTest(run_field=field):
                bad = dict(run); bad[field] = value
                with self.assertRaises(ValueError): NATIVE.validate_original_job(bad, job, MANIFEST)
        for position in (0, 1):
            with self.subTest(step=position + 17):
                bad = copy.deepcopy(job); bad['steps'][position]['conclusion'] = 'skipped'
                with self.assertRaises(ValueError): NATIVE.validate_original_job(run, bad, MANIFEST)
        bad = copy.deepcopy(job); bad['steps'].append(copy.deepcopy(job['steps'][1]))
        with self.assertRaises(ValueError): NATIVE.validate_original_job(run, bad, MANIFEST)


def main():
    global PROJECT_DIRECTORY
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--projects', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    PROJECT_DIRECTORY = args.projects
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(test)
                               for test in (ProjectTests, IdentityTests))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if args.output:
        args.output.write_text(json.dumps({'passed': result.wasSuccessful(), 'tests': result.testsRun,
            'skipped': len(result.skipped), 'originalRun': MANIFEST['originalRunId'],
            'originalCommit': MANIFEST['originalCommit'], 'expectedClosure': MANIFEST['expectedClosure'],
            'scope': 'Original generated project parsing only; no AIS/native binary execution'}, indent=2) + '\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
