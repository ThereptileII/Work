#!/usr/bin/env python3
"""Portable contract tests. Does not claim MSVC, native TLS or certificate-store execution."""
import base64
import copy
import importlib.util
import json
import os
from pathlib import Path, PureWindowsPath
import shutil
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
                self.assertEqual(set(result), {'helperCommit', 'gitSha256', 'checkoutSha256'})
                self.assertEqual(result['helperCommit'], 'e' * 40)
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
               'GITHUB_SHA': 'e' * 40,
               'GITHUB_WORKFLOW_REF': 'ThereptileII/Work/.github/workflows/skager-plugin-download-retained.yml@ref',
               'GITHUB_RUN_ID': '789', 'GITHUB_RUN_ATTEMPT': '1'}
        source = str(Path(__file__).resolve())
        with tempfile.TemporaryDirectory() as directory:
            previous = Path.cwd()
            try:
                os.chdir(directory)
                with patch.dict(os.environ, env, clear=True), patch.dict(self.namespace,
                    {'GitHub': RefusingGitHub, '__file__': source}), \
                    patch.object(subprocess, 'check_output', return_value='e' * 40), \
                    patch.dict(self.namespace, {'authenticate_sources': lambda candidate, helper: {'producerCommit': candidate, 'helperCommit': helper}}), \
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

    def test_helper_checkout_and_dispatch_identity_must_agree(self):
        base = {'PRODUCER_RUN': '123', 'PRODUCER_ATTEMPT': '2', 'PRODUCER_COMMIT': self.commit,
                'INVENTORY_SHA256': 'd' * 64, 'GITHUB_WORKFLOW_SHA': 'e' * 40,
                'GITHUB_SHA': 'e' * 40, 'GITHUB_WORKFLOW_REF': 'ThereptileII/Work/.github/workflows/skager-plugin-download-retained.yml@ref',
                'GITHUB_RUN_ID': '789', 'GITHUB_RUN_ATTEMPT': '1'}
        for head, dispatch in ((self.commit, 'e' * 40), ('e' * 40, self.commit)):
            with self.subTest(head=head, dispatch=dispatch), tempfile.TemporaryDirectory() as directory:
                previous = Path.cwd()
                try:
                    os.chdir(directory)
                    env = dict(base, GITHUB_SHA=dispatch)
                    with patch.dict(os.environ, env, clear=True), patch.dict(self.namespace, {'__file__': __file__}), \
                            patch.object(subprocess, 'check_output', return_value=head):
                        with self.assertRaisesRegex(ValueError, 'Exact dispatched helper checkout'):
                            self.namespace['main']()
                    result = json.loads(Path('opennav-x/evidence/local/retained-plugin-auth/authority.json').read_text())
                    self.assertEqual(result['status'], 'failed')
                    self.assertEqual(result['helperCommit'], 'e' * 40)
                    self.assertEqual(result['producerCommit'], self.commit)
                    self.assertFalse(Path('retained-plugin-inputs').exists())
                finally:
                    os.chdir(previous)

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

    def test_post_restore_binding_uses_preserved_recipe_and_retains_failures(self):
        auth_code = textwrap.dedent(self.workflow.split('          # BEGIN RETAINED_PLUGIN_AUTH\n', 1)[1].split(
            '          # END RETAINED_PLUGIN_AUTH', 1)[0])
        binding_code = textwrap.dedent(self.workflow.split('          # BEGIN RETAINED_PLUGIN_BIND\n', 1)[1].split(
            '          # END RETAINED_PLUGIN_BIND', 1)[0])
        source_binding = {'producerCommit': self.commit, 'helperCommit': 'e' * 40}
        # SDK reads/native imports are inert adapters. The actual workflow's
        # binding sequence, prior identity checks, capture validator and durable
        # failure receipt execute, including loading its preserved source script.
        runner = '''from types import SimpleNamespace
def retained_inputs(root, manifest, pin, candidate):
    assert pin == 'd' * 64 and candidate == 'a' * 40
    return {'retained': True}
def module(name, filename):
    assert filename == 'plugin_subject_sdk.py'
    def verify_subject(*args):
        if args[-1] != {'mode': 'cross-run-bundle'}: raise ValueError('Subject capture differs')
        return {'files': {'authenticated': True}}
    return SimpleNamespace(verify_subject=verify_subject, dependency_authority=lambda doc: {'mode': 'cross-run-bundle'})
def bind_tls(data, expected):
    assert data == {'retained': True} and expected == {'authenticated': True}
'''
        for failure in (None, 'helper', 'source', 'capture'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                previous, search_path = Path.cwd(), list(sys.path)
                try:
                    os.chdir(directory)
                    evidence = Path('opennav-x/evidence/local/retained-plugin-auth'); evidence.mkdir(parents=True)
                    script = evidence / 'authenticate-inputs.py'
                    script.write_text(auth_code + '\nauthenticate_sources = lambda *args: ' + repr(source_binding) + '\n')
                    tools = Path('opennav-x/tools'); tools.mkdir(parents=True)
                    (tools / 'test-plugin-download-guard-windows.py').write_text(runner)
                    capture_helper = tools / 'retain-plugin-guard-inputs.py'; capture_helper.write_bytes(b'inert original capture helper')
                    authority = dict(self.authenticate(), status='source-and-artifact-authenticated-not-qualified',
                                     helperCommit='e' * 40, independentlyApprovedInventorySha256='d' * 64,
                                     sourceBinding=source_binding)
                    capture = self.capture(); capture['helperSha256'] = self.namespace['file_hash'](capture_helper)
                    if failure == 'helper': authority['helperCommit'] = 'b' * 40
                    if failure == 'source': authority['sourceBinding'] = {'changed': True}
                    if failure == 'capture': capture['dependencyAuthority'] = {'mode': 'same-job-receipt'}
                    receipt = evidence / 'authority.json'; receipt.write_text(json.dumps(authority))
                    captured = Path('retained-plugin-inputs/plugin-guard-inputs'); captured.mkdir(parents=True)
                    (captured / 'capture.json').write_text(json.dumps(capture))
                    env = {'PRODUCER_COMMIT': self.commit, 'GITHUB_WORKFLOW_SHA': 'e' * 40, 'INVENTORY_SHA256': 'd' * 64}
                    with patch.dict(os.environ, env, clear=True):
                        if failure:
                            with self.assertRaises(ValueError): exec(compile(binding_code, '<retained-binding>', 'exec'), {})
                        else:
                            exec(compile(binding_code, '<retained-binding>', 'exec'), {})
                    result = json.loads(receipt.read_text())
                    self.assertEqual(result['status'], 'failed' if failure else 'inputs-authenticated-not-qualified')
                    self.assertEqual(result['producerCommit'], self.commit)
                finally:
                    os.chdir(previous); sys.path[:] = search_path
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
        self.assertIn('ref: ${{ github.workflow_sha }}', self.workflow)
        self.assertIn('git fetch --no-tags --depth=1 origin "$env:PRODUCER_COMMIT"', self.workflow)
        self.assertIn('persist-credentials: false', self.workflow)
        self.assertLess(self.workflow.index('Authenticate exact retained producer artifact'),
                        self.workflow.index('Authenticate selected immutable dependency SDK'))
        self.assertLess(self.workflow.index('Bind retained capture to the authenticated dependency SDK'),
                        self.workflow.index('Run actual native loader'))
        self.assertIn('test-ocharts-loader-windows.py --evidence', self.workflow)
        self.assertIn('--retained-inputs-sha256 "$env:INVENTORY_SHA256"', self.workflow)
        self.assertIn('--candidate-commit "$env:PRODUCER_COMMIT"', self.workflow)
        self.assertIn('--patch-program "$env:SKAGER_PLUGIN_PATCH"', self.workflow)
        self.assertIn('install: perl patch', self.workflow)
        self.assertNotIn('build-pristine-windows.ps1', self.workflow)
        self.assertNotIn('inventorySha256=pin', self.workflow)


class SourceBindings(unittest.TestCase):
    """Exercise the actual workflow gate against disposable producer/helper Git commits."""
    @classmethod
    def setUpClass(cls):
        RetainedWorkflow.setUpClass()
        cls.namespace = RetainedWorkflow.namespace
        cls.repository = Path(__file__).resolve().parents[2]

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        roots = set(self.namespace['DECLARATIONS']) | self.namespace['DIRECT_INPUTS'] | self.namespace['MUTABLE_PROBES'] | set(self.namespace['HELPER_ONLY'])
        for path in self.namespace['DECLARATIONS']:
            roots.update(self.namespace['input_declaration'](str(self.repository / path)))
        for name in sorted(roots):
            source, target = self.repository / name, self.root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_dir():
                shutil.copytree(source, target, dirs_exist_ok=True)
            else:
                shutil.copyfile(source, target)
        self.git('init', '--quiet')
        self.git('config', 'core.autocrlf', 'false')
        self.git('config', 'core.hooksPath', str(self.root / 'no-hooks'))
        self.producer = self.commit()

    def git(self, *args):
        env = dict(os.environ, GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull)
        return subprocess.check_output(['git', '-C', str(self.root), *args], env=env, text=True).strip()

    def commit(self):
        self.git('add', '.')
        self.git('-c', 'user.name=Inert Fixture', '-c', 'user.email=fixture@example.invalid',
                 '-c', 'commit.gpgsign=false', 'commit', '--quiet', '-m', 'Inert source binding fixture')
        return self.git('rev-parse', 'HEAD')

    def bind(self, helper):
        previous = Path.cwd()
        try:
            os.chdir(self.root)
            return self.namespace['authenticate_sources'](self.producer, helper)
        finally:
            os.chdir(previous)

    def test_reviewed_consumer_requires_exact_candidate_and_both_blobs(self):
        name = 'opennav-x/tools/test-plugin-download-guard-windows.py'
        old = self.git('rev-parse', self.producer + ':' + name)
        with (self.root / name).open('ab') as stream: stream.write(b'\n# reviewed inert consumer change\n')
        helper = self.commit()
        pair = {'producer': old, 'helper': self.git('rev-parse', helper + ':' + name)}
        with patch.dict(self.namespace, {'REVIEWED_CANDIDATE': self.producer,
                                         'REVIEWED_CONSUMERS': {name: pair}}):
            result = self.bind(helper)
            self.assertEqual(result['allowedConsumerChanges'], [name])
            self.assertEqual(result['allowedProbeChanges'], [])
            for key in ('producer', 'helper'):
                with patch.dict(pair, {key: 'f' * 40}):
                    with self.assertRaisesRegex(ValueError, 'Immutable consumed input drift'): self.bind(helper)
            with patch.dict(self.namespace, {'REVIEWED_CANDIDATE': 'f' * 40}):
                with self.assertRaisesRegex(ValueError, 'Immutable consumed input drift'): self.bind(helper)

    def test_new_helper_exact_git_and_checkout_bytes_before_import(self):
        name = next(iter(self.namespace['HELPER_ONLY']))
        path = self.root / name
        original = path.read_bytes()
        path.write_bytes(original + b'\n# unreviewed helper change\n')
        with self.assertRaisesRegex(ValueError, 'Helper-only checkout differs'): self.bind(self.producer)
        helper = self.commit()
        with self.assertRaisesRegex(ValueError, 'Reviewed helper-only source differs'): self.bind(helper)

    def test_only_console_probe_changes_are_recorded_with_both_hashes(self):
        for name in self.namespace['MUTABLE_PROBES']:
            with (self.root / name).open('ab') as stream:
                stream.write(b'\n// inert helper-only header correction\n')
        helper = self.commit()
        result = self.bind(helper)
        self.assertEqual(result['producerCommit'], self.producer)
        self.assertEqual(result['helperCommit'], helper)
        self.assertEqual(result['allowedProbeChanges'], sorted(self.namespace['MUTABLE_PROBES']))
        for name in self.namespace['MUTABLE_PROBES']:
            record = result['consumedInputs'][name]
            self.assertNotEqual(record['producer']['sha256'], record['helper']['sha256'])
            self.assertEqual(record['helper']['sha256'], record['checkoutSha256'])

    def test_production_patch_dependency_and_native_assertion_drift_refuse(self):
        for name in ('opennav-x/src/integration/OriginalPluginTlsGuard.h',
                     'opennav-x/patches/opencpn-5.12.4-download-trust.patch',
                     'opennav-x/tools/windows-curl.lock.json',
                     'opennav-x/tools/windows-dependency-bundle.lock.json',
                     'opennav-x/tools/windows-dependency-consumer-compatibility.json',
                     'opennav-x/tools/verify-curl-bundle.ps1',
                     'opennav-x/tools/test-plugin-download-guard-windows.py',
                     'opennav-x/tools/release_manifest.py',
                     'opennav-x/tools/hardware_output_policy.py',
                     'opennav-x/tools/packaged_notice_gate.py',
                     'opennav-x/tools/freefont_payload.py',
                     'opennav-x/tests/windows_ocharts_loader/loader_test.cpp'):
            with self.subTest(name=name):
                path = self.root / name; original = path.read_bytes()
                path.write_bytes(original + b'\nimmutable source changed\n')
                with self.assertRaisesRegex(ValueError, 'Immutable consumed input drift'):
                    self.bind(self.commit())
                path.write_bytes(original)

    def test_new_consumed_file_and_changed_declaration_refuse(self):
        path = self.root / 'opennav-x/patches/unreviewed.patch'
        path.write_text('Inert added production patch')
        with self.assertRaisesRegex(ValueError, 'inventory differs'):
            self.bind(self.commit())
        path.unlink()
        declaration = self.root / self.namespace['DECLARATIONS'][0]
        declaration.write_text('INPUTS = ("tools/plugin-download-guard-probe.cpp",)\n')
        with self.assertRaisesRegex(ValueError, 'declarations drifted'):
            self.bind(self.commit())

    def test_uncommitted_allowed_probe_or_declaration_drift_refuses(self):
        for name in (*self.namespace['MUTABLE_PROBES'], *self.namespace['DECLARATIONS']):
            with self.subTest(name=name):
                path = self.root / name; original = path.read_bytes()
                path.write_bytes(original + b'\nlocal uncommitted change\n')
                try:
                    with self.assertRaisesRegex(ValueError, 'checkout.*(differs|drift)'):
                        self.bind(self.producer)
                finally:
                    path.write_bytes(original)


if __name__ == '__main__':
    unittest.main()
