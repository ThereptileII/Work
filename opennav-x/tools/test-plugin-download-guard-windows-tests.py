#!/usr/bin/env python3
"""Portable contract tests. Does not claim MSVC, native TLS or certificate-store execution."""
import copy
import importlib.util
import json
import os
from pathlib import Path, PureWindowsPath
import subprocess
import sys
import tempfile
import unittest
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


if __name__ == '__main__':
    unittest.main()
