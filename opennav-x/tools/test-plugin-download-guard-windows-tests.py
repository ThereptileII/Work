#!/usr/bin/env python3
"""Portable contract tests. Does not claim MSVC, native TLS or certificate-store execution."""
import base64
import copy
import importlib.util
import json
import os
from pathlib import Path, PureWindowsPath
import subprocess
import sys
import tempfile
import textwrap
import unittest
import zipfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))

def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result

api = load('native_plugin_guard', 'test-plugin-download-guard-windows.py')
guard = load('plugin_guard', 'test-plugin-download-guard.py')
tls = load('trust_certificates', 'test-downloader-trust.py')


class RetainedInputs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'retained'; self.root.mkdir()
        self.manifest = self.base / 'inputs.json'
        self.inputs = {'curlInclude': 'curl/include', 'curlImport': 'curl/lib/libcurl.lib',
            'archiveInclude': 'archive/include', 'archiveImport': 'archive/lib/archive.lib',
            'wxRoot': 'wx', 'wxLib': 'wx/lib/vc14x_dll'}
        names = ['curl/include/curl/curl.h', 'archive/include/archive.h', 'wx/include/wx/wx.h',
                 'wx/lib/vc14x_dll/wxbase32u.lib', self.inputs['curlImport'], self.inputs['archiveImport']]
        self.runtime = {name: 'runtime/' + name for name in api.DLLS}
        names.extend(self.runtime.values())
        for name in names:
            path = self.root / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b'fixture ' + name.encode())
        self.data = {'schema': 1, 'candidateCommit': 'a' * 40, 'inputs': self.inputs,
                     'runtime': self.runtime, 'files': api.receipt._inventory(self.root, ['curl', 'archive', 'wx', 'runtime'])}
        self.write()

    def write(self):
        self.manifest.write_text(json.dumps(self.data))
        self.pin = api.receipt._digest(self.manifest)

    def verify(self):
        return api.retained_inputs(self.root, self.manifest, self.pin, 'a' * 40)

    def test_exact_inventory(self):
        self.assertEqual(self.verify(), self.data)

    def test_external_pin_required(self):
        self.manifest.write_text(self.manifest.read_text() + ' ')
        with self.assertRaisesRegex(ValueError, 'external reviewed hash'): self.verify()

    def test_candidate_mismatch(self):
        self.data['candidateCommit'] = 'b' * 40; self.write()
        with self.assertRaisesRegex(ValueError, 'schema'): self.verify()

    def test_file_tampering(self):
        (self.root / self.inputs['curlImport']).write_bytes(b'tampered')
        with self.assertRaisesRegex(ValueError, 'missing, changed, extra'): self.verify()

    def test_extra_sdk_header(self):
        (self.root / 'curl/include/extra.h').write_bytes(b'new')
        with self.assertRaisesRegex(ValueError, 'missing, changed, extra'): self.verify()

    def test_missing_dll(self):
        del self.runtime['archive.dll']; self.write()
        with self.assertRaisesRegex(ValueError, 'schema'): self.verify()

    def test_path_traversal(self):
        self.data['inputs']['curlImport'] = '../outside'; self.write()
        with self.assertRaisesRegex(ValueError, 'unsafe relative'): self.verify()

    def test_case_alias(self):
        self.data['files']['CURL/include/curl/curl.h'] = self.data['files']['curl/include/curl/curl.h']; self.write()
        with self.assertRaisesRegex(ValueError, 'Case-aliased'): self.verify()

    def test_boolean_size_rejected(self):
        self.data['files']['curl/include/curl/curl.h']['bytes'] = True; self.write()
        with self.assertRaisesRegex(ValueError, 'Invalid retained'): self.verify()

    def test_symlink_rejected(self):
        path = self.root / 'curl/include/curl/curl.h'; path.unlink(); path.symlink_to(self.manifest)
        with self.assertRaises(ValueError): self.verify()

    def test_duplicate_json_keys(self):
        self.manifest.write_text('{"schema":1,"schema":1}')
        self.pin = api.receipt._digest(self.manifest)
        with self.assertRaisesRegex(ValueError, 'duplicate JSON'): self.verify()

    def tls_expected(self):
        bindings = [(self.inputs['curlImport'], api.stage.PREFIX['curl'] + '/lib/libcurl.lib'),
                    ('curl/include/curl/curl.h', api.stage.PREFIX['curl'] + '/include/curl/curl.h')]
        for kind, name in [('curl', 'libcurl.dll'), ('openssl', 'libssl-3.dll'),
                           ('openssl', 'libcrypto-3.dll'), ('zlib', 'zlib1.dll')]:
            bindings.append((self.runtime[name], api.stage.PREFIX[kind] + '/bin/' + name))
        return {producer: copy.deepcopy(self.data['files'][local]) for local, producer in bindings}

    def test_authenticated_tls_match(self):
        api.bind_tls(self.data, self.tls_expected())

    def test_authenticated_tls_mismatch(self):
        for key in self.tls_expected():
            with self.subTest(key=key):
                expected = self.tls_expected(); expected[key]['sha256'] = '0' * 64
                with self.assertRaises(ValueError): api.bind_tls(self.data, expected)

    def test_extra_curl_header_not_authorized(self):
        expected = self.tls_expected()
        self.data['files']['curl/include/curl/extra.h'] = self.data['files']['curl/include/curl/curl.h']
        with self.assertRaisesRegex(ValueError, 'curl headers'): api.bind_tls(self.data, expected)


class ProjectAndHost(unittest.TestCase):
    def test_host_refuses_local_machine(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, 'disposable'): api.host_guard()

    def test_host_compiler_override(self):
        env = {'GITHUB_ACTIONS': 'true', 'RUNNER_ENVIRONMENT': 'github-hosted',
               'RUNNER_OS': 'Windows', 'GITHUB_REPOSITORY': 'ThereptileII/Work'}
        with patch.object(sys, 'platform', 'win32'), patch.dict(os.environ, env, clear=True):
            api.host_guard()
            for name in ('CL', '_CL_', 'CMAKE_TOOLCHAIN_FILE'):
                with patch.dict(os.environ, {name: 'injected'}):
                    with self.assertRaisesRegex(ValueError, 'override'): api.host_guard()

    def test_generated_project_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); source = base / 'source'; sdk = base / 'sdk'; tools = base / 'tools'
            inputs = {'curlImport': 'curl/lib/libcurl.lib', 'archiveImport': 'archive/lib/archive.lib', 'wxLib': 'wx/lib'}
            project = base / 'probe.vcxproj'
            content = f'''<Project xmlns="{api.NS['m']}"><ItemGroup>
<ClCompile Include="{source}/model/src/downloader.cpp"/><ClCompile Include="{tools}/plugin-download-guard-probe.cpp"/>
</ItemGroup><ItemDefinitionGroup Condition="Release|Win32"><ClCompile><RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary><AdditionalIncludeDirectories>{source};{source}/model/include;{sdk}/curl/include;{sdk}/archive/include;{sdk}/wx/include;%(AdditionalIncludeDirectories)</AdditionalIncludeDirectories></ClCompile>
<Link><AdditionalDependencies>{sdk}/curl/lib/libcurl.lib;{sdk}/archive/lib/archive.lib;{sdk}/wx/lib/wxbase32u.lib</AdditionalDependencies></Link>
</ItemDefinitionGroup></Project>'''
            project.write_text(content); api.verify_project(project, source, tools, sdk, inputs)
            for old, new in [('MultiThreadedDLL', 'MultiThreaded'), ('Release|Win32', 'Release|x64'),
                (str(sdk) + '/curl/lib', '/outside'), (str(sdk) + '/wx/include', '/outside'), (str(sdk) + '/wx/lib', '/outside'),
                ('</Project>', '<Flag>OPENNAV_DOWNLOADER_TLS_TEST</Flag></Project>'),
                ('model/src/downloader.cpp', 'model/src/fake.cpp')]:
                with self.subTest(new=new):
                    project.write_text(content.replace(old, new))
                    with self.assertRaises(ValueError): api.verify_project(project, source, tools, sdk, inputs)

    def test_windows_openssl_config_path(self):
        self.assertEqual(tls.config_path(PureWindowsPath('C:/owned test/$ca/cert.pem')), '"C:/owned test/\\$ca/cert.pem"')

    def test_case_name_and_negative_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = subprocess.CompletedProcess([], 1, 'install_ok=false\narchive_opens=0\narchive_entries=0\nerror=Cannot download plugin: Fixture [TLS refused]\n', '')
            with patch.object(guard.subprocess, 'run', return_value=result):
                observed = guard.probe(root / 'probe', root, 'wrong_host', 'https://localhost', root / 'ca.pem', False)
            self.assertEqual(observed['case'], 'wrong_host')
            self.assertTrue((root / 'wrong_host.stdout').is_file())
            self.assertTrue(observed['installedFilesAndRecordsUnchanged'])


class RetainedWorkflow(unittest.TestCase):
    """Execute the workflow's actual authentication code with inert API/ZIP inputs."""
    @classmethod
    def setUpClass(cls):
        cls.workflow = (Path(__file__).resolve().parents[2] /
                        '.github/workflows/skager-plugin-download-retained.yml').read_text()
        code = cls.workflow.split('          # BEGIN RETAINED_PLUGIN_AUTH\n', 1)[1].split(
            '          # END RETAINED_PLUGIN_AUTH', 1)[0]
        cls.namespace = {'__name__': 'retained_workflow_contract'}
        exec(compile(textwrap.dedent(code), '<retained-workflow>', 'exec'), cls.namespace)

    def setUp(self):
        self.commit = 'a' * 40
        self.run = {'id': 123, 'run_attempt': 2, 'head_sha': self.commit,
                    'path': '.github/workflows/opennav-baseline.yml', 'event': 'workflow_dispatch',
                    'status': 'completed', 'conclusion': 'failure',
                    'head_repository': {'full_name': 'ThereptileII/Work'}}
        self.job = {'name': 'windows-integration', 'run_id': 123, 'run_attempt': 2,
                    'head_sha': self.commit, 'status': 'completed', 'conclusion': 'failure',
                    'steps': [{'name': self.namespace[key], 'status': 'completed', 'conclusion': 'success'}
                              for key in ('PRODUCTION', 'CAPTURE')]}
        self.artifact = {'id': 456, 'name': f'plugin-guard-inputs-{self.commit}-run123-attempt2',
                         'expired': False, 'digest': 'sha256:' + 'b' * 64, 'size_in_bytes': 100,
                         'workflow_run': {'id': 123, 'head_sha': self.commit}}

    def authenticate(self):
        owner = self
        class GitHub:
            base = 'repos/ThereptileII/Work'
            repo = 'ThereptileII/Work'
            def api(self, endpoint):
                if endpoint.endswith('/attempts/2'): return owner.run
                if '/artifacts?' in endpoint: return {'artifacts': [owner.artifact]}
                if endpoint.endswith('/artifacts/456'): return owner.artifact
                raise AssertionError('Unexpected API endpoint: ' + endpoint)
            def pages(self, endpoint):
                self_endpoint = self.base + '/actions/runs/123/attempts/2/jobs'
                if endpoint != self_endpoint: raise AssertionError(endpoint)
                return [owner.job]
        return self.namespace['authenticate'](GitHub(), '123', '2', self.commit)

    def test_later_failure_does_not_relabel_production_or_capture(self):
        result = self.authenticate()
        self.assertEqual(result['producerJobConclusion'], 'failure')
        self.assertEqual(result['producerRunConclusion'], 'failure')
        self.assertEqual(result['artifactDigest'], 'sha256:' + 'b' * 64)

    def test_attempt_endpoint_binds_documented_job_without_optional_attempt(self):
        del self.job['run_attempt']
        self.assertEqual(self.authenticate()['producerAttempt'], '2')

    def test_actual_dispatch_workflow_bytes_required(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'workflow.yml'; path.write_bytes(b'exact recipe')
            class GitHub:
                base = 'repos/ThereptileII/Work'
                def api(self, endpoint):
                    if not endpoint.endswith('?ref=' + 'e' * 40): raise AssertionError(endpoint)
                    return {'type': 'file', 'encoding': 'base64', 'size': 12,
                            'content': base64.b64encode(b'exact recipe').decode()}
            with patch.object(self.namespace['subprocess'], 'check_output', return_value=b'exact recipe'):
                result = self.namespace['authenticate_workflow'](GitHub(), 'e' * 40, path)
                self.assertEqual(set(result), {'gitSha256', 'checkoutSha256'})
                path.write_bytes(b'exact recipe\r\n')
                result = self.namespace['authenticate_workflow'](GitHub(), 'e' * 40, path)
                self.assertNotEqual(result['gitSha256'], result['checkoutSha256'])
            with patch.object(self.namespace['subprocess'], 'check_output', return_value=b'older recipe'):
                with self.assertRaisesRegex(ValueError, 'exact dispatch workflow'):
                    self.namespace['authenticate_workflow'](GitHub(), 'e' * 40, path)

    def test_auth_failure_before_extraction_retains_failed_receipt(self):
        class RefusingGitHub:
            base = 'repos/ThereptileII/Work'
            def __init__(self, repository): pass
            def api(self, endpoint): raise ValueError('inert API refusal')
        env = {'PRODUCER_RUN': '123', 'PRODUCER_ATTEMPT': '2', 'PRODUCER_COMMIT': self.commit,
               'INVENTORY_SHA256': 'd' * 64, 'GITHUB_WORKFLOW_SHA': 'e' * 40,
               'GITHUB_WORKFLOW_REF': 'ThereptileII/Work/.github/workflows/skager-plugin-download-retained.yml@ref',
               'GITHUB_RUN_ID': '789', 'GITHUB_RUN_ATTEMPT': '1'}
        source = str(Path(__file__).resolve())
        with tempfile.TemporaryDirectory() as directory:
            previous = Path.cwd()
            try:
                os.chdir(directory)
                with patch.dict(os.environ, env, clear=True), patch.dict(self.namespace,
                    {'GitHub': RefusingGitHub, '__file__': source}), \
                    patch.object(subprocess, 'check_output', return_value=self.commit), \
                    patch.object(subprocess, 'run'):
                    with self.assertRaisesRegex(ValueError, 'inert API refusal'): self.namespace['main']()
                evidence = Path('opennav-x/evidence/local/retained-plugin-auth')
                result = json.loads((evidence / 'authority.json').read_text())
                self.assertEqual(result['status'], 'failed')
                self.assertEqual(result['phase'], 'source-identity')
                self.assertEqual(result['independentlyApprovedInventorySha256'], 'd' * 64)
                self.assertFalse(result['productAcceptance'])
                self.assertTrue((evidence / 'authenticate-inputs.py').is_file())
                self.assertFalse(Path('retained-plugin-inputs').exists())
            finally: os.chdir(previous)

    def test_exact_producer_identity(self):
        for target, key, value in [('run', 'id', 124), ('run', 'run_attempt', 1),
            ('run', 'head_sha', 'c' * 40), ('run', 'path', '.github/workflows/foreign.yml'),
            ('run', 'event', 'pull_request'), ('run', 'status', 'in_progress'),
            ('run', 'head_repository', {'full_name': 'foreign/Work'}),
            ('job', 'run_id', 124), ('job', 'run_attempt', 1), ('job', 'head_sha', 'c' * 40),
            ('job', 'status', 'in_progress'), ('job', 'name', 'foreign')]:
            with self.subTest(target=target, key=key):
                item = getattr(self, target); previous = copy.deepcopy(item[key]); item[key] = value
                try:
                    with self.assertRaises(ValueError): self.authenticate()
                finally: item[key] = previous

    def test_prerequisite_steps_must_be_unique_and_successful(self):
        original = copy.deepcopy(self.job['steps'])
        for index in range(2):
            for mode in ('failure', 'skipped', 'cancelled', 'running', 'missing', 'ambiguous'):
                with self.subTest(index=index, mode=mode):
                    self.job['steps'] = copy.deepcopy(original)
                    if mode == 'missing': del self.job['steps'][index]
                    elif mode == 'ambiguous': self.job['steps'].append(copy.deepcopy(original[index]))
                    elif mode == 'running': self.job['steps'][index]['status'] = 'in_progress'
                    else: self.job['steps'][index]['conclusion'] = mode
                    with self.assertRaises(ValueError): self.authenticate()

    def test_artifact_identity_and_integrity_authority(self):
        for key, value in [('name', 'plugin-guard-inputs-foreign'), ('expired', True),
            ('digest', ''), ('digest', 'sha256:' + 'B' * 64), ('size_in_bytes', True),
            ('size_in_bytes', 0), ('size_in_bytes', self.namespace['MAX_BYTES'] + 1),
            ('workflow_run', {'id': 124, 'head_sha': self.commit}),
            ('workflow_run', {'id': 123, 'head_sha': 'c' * 40})]:
            with self.subTest(key=key, value=value):
                previous = copy.deepcopy(self.artifact[key]); self.artifact[key] = value
                try:
                    with self.assertRaises(ValueError): self.authenticate()
                finally: self.artifact[key] = previous

    def capture(self):
        return {'schema': 1, 'status': 'captured-not-qualified', 'candidateCommit': self.commit,
                'producer': {'commit': self.commit, 'runId': '123', 'runAttempt': '2',
                             'job': 'windows-integration', 'architecture': 'Win32'},
                'helperSha256': 'c' * 64, 'inventorySha256': 'd' * 64,
                'dependencyAuthority': {'mode': 'cross-run-bundle'},
                'requiresIndependentInventoryPin': True, 'nativeAcceptance': False,
                'productAcceptance': False, 'boatAcceptance': False}

    def verify_capture(self, capture):
        self.namespace['verify_capture'](capture, self.authenticate(), 'd' * 64, 'c' * 64,
                                         {'mode': 'cross-run-bundle'})

    def test_capture_binds_producer_source_pin_and_dependency(self):
        self.verify_capture(self.capture())
        cases = [('schema', True), ('candidateCommit', 'b' * 40), ('status', 'qualified'),
                 ('helperSha256', 'b' * 64), ('inventorySha256', 'b' * 64),
                 ('dependencyAuthority', {'mode': 'same-job-receipt'}),
                 ('requiresIndependentInventoryPin', False), ('nativeAcceptance', True),
                 ('productAcceptance', True), ('boatAcceptance', True)]
        for key, value in cases:
            with self.subTest(key=key):
                capture = self.capture(); capture[key] = value
                with self.assertRaises(ValueError): self.verify_capture(capture)
        for key, value in [('runId', '124'), ('runAttempt', '1'), ('commit', 'b' * 40),
                           ('job', 'foreign'), ('architecture', 'x64')]:
            with self.subTest(producer=key):
                capture = self.capture(); capture['producer'][key] = value
                with self.assertRaises(ValueError): self.verify_capture(capture)

    def zip_members(self):
        return ['plugin-guard-inputs/inventory.json', 'plugin-guard-inputs/capture.json',
                'opennav-x/evidence/local/plugin-guard-capture.log', 'plugin-guard-inputs/data/sdk/header.h']

    def unpack(self, members, *, linked=False):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory); archive = base / 'artifact.zip'; destination = base / 'out'
            with zipfile.ZipFile(archive, 'w') as output:
                for name in members:
                    item = zipfile.ZipInfo(name)
                    if linked: item.external_attr = 0o120777 << 16
                    output.writestr(item, b'inert fixture')
            self.namespace['unpack'](archive, destination)
            return sorted(path.relative_to(destination).as_posix() for path in destination.rglob('*') if path.is_file())

    def test_closed_retained_zip_projection(self):
        self.assertEqual(self.unpack(self.zip_members()), sorted(self.zip_members()))

    def test_unsafe_extra_alias_and_collision_members_refused(self):
        for name in ['../escape', 'plugin-guard-inputs/data/../escape', '/absolute',
            'plugin-guard-inputs/data/CON.h', 'plugin-guard-inputs/data/file:stream',
            'plugin-guard-inputs/data/trailing.', 'plugin-guard-inputs/data/back\\slash',
            'plugin-guard-inputs/data/directory/', 'foreign.txt',
            'plugin-guard-inputs/data/sdk/HEADER.h', 'plugin-guard-inputs/data/sdk']:
            with self.subTest(name=name):
                with self.assertRaises(ValueError): self.unpack(self.zip_members() + [name])
        with self.assertRaises(ValueError): self.unpack(self.zip_members(), linked=True)
        for index in range(3):
            with self.subTest(missing=index):
                members = self.zip_members(); del members[index]
                with self.assertRaises(ValueError): self.unpack(members)

    def test_native_invocation_keeps_external_pin_and_no_application_build(self):
        self.assertEqual(self.workflow.count('workflow_dispatch:'), 1)
        self.assertNotIn('\n  push:', self.workflow)
        self.assertIn('ref: ${{ inputs.producer_commit }}', self.workflow)
        self.assertIn('persist-credentials: false', self.workflow)
        self.assertIn('test-ocharts-loader-windows.py --evidence', self.workflow)
        self.assertIn('--retained-inputs-sha256 "$env:INVENTORY_SHA256"', self.workflow)
        self.assertIn('--candidate-commit "$env:PRODUCER_COMMIT"', self.workflow)
        self.assertIn('--patch-program "$env:SKAGER_PLUGIN_PATCH"', self.workflow)
        self.assertIn('install: perl patch', self.workflow)
        self.assertNotIn('build-pristine-windows.ps1', self.workflow)
        self.assertNotIn('inventorySha256=pin', self.workflow)


if __name__ == '__main__':
    unittest.main()
