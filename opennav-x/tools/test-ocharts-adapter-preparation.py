#!/usr/bin/env python3
"""Focused package/PE/source guards; synthetic PE is never native evidence."""
import importlib.util
from pathlib import Path
import struct
import shutil
import json
import hashlib
import os
import re
import zipfile
from unittest.mock import patch
import tempfile
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('adapter_validator', ROOT / 'tools/verify-ocharts-adapter-package.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)


def pe(import_name=b'opencpn.exe', exported=None):
    data = bytearray(4096)
    def put(fmt, off, *args): struct.pack_into(fmt, data, off, *args)
    data[:2] = b'MZ'; put('<I', 60, 128); data[128:132] = b'PE\0\0'
    put('<HHIIIHH', 132, 0x14c, 1, 0, 0, 0, 224, 0x2000)
    put('<H', 152, 0x10b); put('<I', 244, 16)
    put('<II', 248, 0x1100, 100); put('<II', 256, 0x1200, 40)
    put('<IIII', 384, 0xc00, 0x1000, 0xc00, 0x400)
    # section RVA1000 maps file offset400.
    put('<IIIII', 0x600, 0x1500, 0, 0, 0x1300, 0x1500)
    data[0x700:0x700 + len(import_name) + 1] = import_name + b'\0'
    names = sorted(v.EXPORTS) if exported is None else exported
    put('<IIIII', 0x514, len(names), len(names), 0x1400, 0x1420, 0x1440)
    cursor = 0x900
    for n, name in enumerate(names):
        put('<I', 0x800 + n * 4, 0x1700 + n)
        put('<I', 0x820 + n * 4, cursor + 0xc00)
        put('<H', 0x840 + n * 2, n)
        encoded = name.encode() + b'\0'; data[cursor:cursor + len(encoded)] = encoded
        cursor += len(encoded)
    return data


class Guards(unittest.TestCase):
    def test_prepared_owned_include_closure(self):
        # Exercise the production copier in an isolated directory. Resolve every
        # quoted owned include from the copied source, not the full checkout's
        # include path which hid this omission from the host's native proof.
        def check(directory):
            files = {p.relative_to(directory / 'local').as_posix(): p
                     for p in (directory / 'local').rglob('*') if p.is_file()}
            self.assertEqual(set(files), set(v.prep.LOCAL))
            for name, target in files.items():
                self.assertIn(name, v.prep.INPUTS)
                self.assertEqual(target.read_bytes(),
                                 (ROOT / name).read_bytes().replace(b'\r\n', b'\n'))
                for include in re.findall(r'^\s*#include\s*"([^"]+)"', target.read_text(), re.M):
                    owned = Path('src') / include
                    if (ROOT / owned).is_file():
                        self.assertIn(owned.as_posix(), files,
                                      name + ' cannot resolve ' + include)
        with tempfile.TemporaryDirectory() as raw:
            prepared = Path(raw)
            v.prep.copy_local_inputs(prepared)
            check(prepared)
        # Reproduce both pre-fix omissions through the same actual copy path.
        for missing in ('src/integration/ChartNameSpacing.h', 'src/ui/Theme.h'):
            with self.subTest(missing=missing), tempfile.TemporaryDirectory() as raw:
                with patch.object(v.prep, 'LOCAL', tuple(p for p in v.prep.LOCAL if p != missing)):
                    prepared = Path(raw)
                    v.prep.copy_local_inputs(prepared)
                    with self.assertRaisesRegex(AssertionError, 'cannot resolve'):
                        check(prepared)

    def test_exact_pe(self):
        imports, exports = v.pe_contract(pe())
        self.assertEqual(imports, ['opencpn.exe'])
        self.assertEqual(set(exports), v.EXPORTS)

    def test_wrong_arch(self):
        data = pe(); struct.pack_into('<H', data, 132, 0x8664)
        with self.assertRaisesRegex(ValueError, 'I386'): v.pe_contract(data)

    def test_legacy_extra_dependency(self):
        for name in (b'libeay32.dll', b'ssleay32.dll', b'oexserverd.dll', b'unknown.dll'):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'Unexpected'):
                v.pe_contract(pe(name))

    def test_not_dll(self):
        data = pe(); struct.pack_into('<H', data, 150, 0)
        with self.assertRaises(ValueError): v.pe_contract(data)

    def test_delay_import(self):
        data = pe(); struct.pack_into('<II', data, 248 + 13 * 8, 0x1800, 32)
        with self.assertRaisesRegex(ValueError, 'Delay'): v.pe_contract(data)

    def test_forwarded_export(self):
        data = pe(); struct.pack_into('<I', data, 0x800, 0x1100)
        with self.assertRaisesRegex(ValueError, 'Forwarded'): v.pe_contract(data)

    def test_wrong_export(self):
        with self.assertRaisesRegex(ValueError, 'contract'):
            v.pe_contract(pe(exported=sorted(v.EXPORTS - {'skager_chart_point_style_v1'}) + ['wrong']))

    def test_missing_point_style_export(self):
        with self.assertRaisesRegex(ValueError, 'inventory'):
            v.pe_contract(pe(exported=sorted(v.EXPORTS - {'skager_chart_point_style_v1'})))

    def test_truncated(self):
        for length in (0, 62, 140, 400, 2048):
            with self.subTest(length=length), self.assertRaises(ValueError): v.pe_contract(pe()[:length])

    def test_path_and_blob_negative(self):
        for path in ('../escape', '/absolute', 'x\\y', 'c:/test', 'x/../a'):
            with self.subTest(path=path), self.assertRaises(ValueError): v.prep.safe_path(path)
        self.assertFalse(v.prep.blob_ok(b'changed', {'bytes': 7, 'gitBlob': '0' * 40}))

    def test_no_extra_package(self):
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw); (directory / 'oexserverd.exe').write_bytes(b'forbidden')
            with self.assertRaisesRegex(ValueError, 'payload'): v.verify(directory, directory)

    def test_package_roundtrip_and_tampering(self):
        from curl_package import SOURCES, SOURCE_KEYS, CURL_OUTPUTS, ZLIB_OUTPUTS
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw); product = root / 'product'; package = root / 'package'; resources = root / 'resources'
            (product / 'tools').mkdir(parents=True); package.mkdir(); resources.mkdir()
            (product / 'input.txt').write_bytes(b'reviewed patch\n')
            blob = b'open source license\n'
            lock = {'source': {'repository': 'bdbcat/o-charts_pi',
                    'commit': 'c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8',
                    'files': {'COPYING': {'bytes': len(blob), 'gitBlob': hashlib.sha1(b'blob ' + str(len(blob)).encode() + b'\0' + blob).hexdigest()}}},
                    'gitlink': {'commit': '38762f4e7b8faf39133cc40efc99b12b5200f04b', 'files': {}}}
            (product / v.prep.LOCK).write_text(json.dumps(lock))
            (resources / 'manifest.json').write_text('{"files":{}}')
            (resources / 'XNavChartResources.h').write_text('resource identity')
            inputs = ('input.txt', v.prep.LOCK)
            with patch.object(v, 'ROOT', product), patch.object(v.prep, 'INPUTS', inputs):
                (package / v.DLL).write_bytes(pe())
                archive = package / 'corresponding-source.zip'
                with zipfile.ZipFile(archive, 'w') as out:
                    for name in inputs: out.writestr('product/' + name, (product / name).read_bytes())
                    out.writestr('source/COPYING', blob)
                imports, exports = v.pe_contract(pe())
                rec = v.prep.record
                manifest = {'schema':1, 'kind':'skager-ocharts-adapter', 'bindingVersion':1,
                    'source': {'repository': lock['source']['repository'], 'commit': lock['source']['commit'],
                               'gitlinkCommit':lock['gitlink']['commit'], 'manifestSha256':rec(product / v.prep.LOCK)['sha256']},
                    'inputs': {p:rec(product / p) for p in inputs},
                    'chartResourceManifest':rec(resources / 'manifest.json'),
                    'chartResourceHeader':rec(resources / 'XNavChartResources.h'),
                    'dll':dict(rec(package / v.DLL), name=v.DLL, machine='I386'),
                    'imports':imports, 'exports':exports,
                    'correspondingSource':dict(rec(archive), path=archive.name),
                    'dependencies':{lib:{'version':SOURCES[lib]['version'],
                        'source':{k:SOURCES[lib][k] for k in SOURCE_KEYS},
                        'outputs':{name:{'bytes':1,'sha256':'a'*64} for name in names}}
                        for lib,names in [('curl',CURL_OUTPUTS),('zlib',ZLIB_OUTPUTS)]}}
                def save(): (package / 'manifest.json').write_text(json.dumps(manifest))
                save(); self.assertEqual(v.verify(package, resources), rec(package / v.DLL))
                # Exercise actual package verification through its installed
                # layout, including compiled identity, runtime and source ZIP.
                template = product / 'src/integration/SkagerOChartsPackage.h.in'
                template.parent.mkdir(parents=True)
                template.write_bytes((ROOT / 'src/integration/SkagerOChartsPackage.h.in').read_bytes())
                app, build = Path(raw) / 'app', Path(raw) / 'build'
                notices = app / 'opennav/third-party/ocharts'; notices.mkdir(parents=True)
                (build / 'include').mkdir(parents=True)
                for name in ('chartsymbols.xml', 'S52RAZDS.RLE', 'rastersymbols-day.png',
                             'rastersymbols-dusk.png', 'rastersymbols-dark.png'):
                    (resources / name).write_bytes(name.encode())
                for lib, name in [('curl', 'libcurl.dll'), ('zlib', 'zlib1.dll')]:
                    (app / name).write_bytes(name.encode())
                    manifest['dependencies'][lib]['outputs']['bin/' + name] = rec(app / name)
                save()
                shutil.copy2(package / v.DLL, app / v.DLL)
                for name in ('manifest.json', 'corresponding-source.zip'):
                    shutil.copy2(package / name, notices / name)
                shutil.copytree(resources, build / 'opennav-chart-style/v1')
                shutil.copytree(resources, app / 'opennav/chart-style/v1')
                header = build / 'include/SkagerOChartsPackage.h'
                header.write_text(v.trust_header(rec(package / v.DLL)))
                bundle = v.installed_source_bundle(app, build)
                self.assertEqual(bundle[0]['archive'], notices / 'corresponding-source.zip')
                for target in (app / v.DLL, header, app / 'libcurl.dll',
                               notices / 'corresponding-source.zip',
                               app / 'opennav/chart-style/v1/chartsymbols.xml'):
                    before = target.read_bytes(); target.write_bytes(before + b'changed')
                    with self.subTest(installed=target.name), self.assertRaises(ValueError):
                        v.installed_source_bundle(app, build)
                    target.write_bytes(before)
                absent = template.read_text().replace('@skager_ocharts_available@','false').replace('@skager_ocharts_sha256@','').replace('@skager_ocharts_bytes@','0')
                header.write_text(absent)
                with self.assertRaisesRegex(ValueError,'adapter-free'):
                    v.installed_source_bundle(app, build)
                (app / v.DLL).unlink(); shutil.rmtree(notices)
                self.assertEqual(v.installed_source_bundle(app, build), [])
                for target, message in [(package / v.DLL,'DLL'), (product / 'input.txt','inputs'),
                                        (resources / 'XNavChartResources.h','header'), (archive,'source package')]:
                    original = target.read_bytes(); target.write_bytes(original + b'changed')
                    with self.subTest(path=target.name), self.assertRaisesRegex(ValueError,message): v.verify(package,resources)
                    target.write_bytes(original)
                manifest['dependencies']['curl']['version']='legacy';save()
                with self.assertRaisesRegex(ValueError,'Legacy'):v.verify(package,resources)

    def test_patch_isolated_from_parent_git_and_crlf(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            subprocess.run(['git','init','--quiet',str(root)],check=True)
            source = root / 'nested/source'; source.mkdir(parents=True)
            patch_file = root / 'change.patch'
            patch_file.write_text('--- a/input.txt\n+++ b/input.txt\n@@ -1 +1 @@\n-before\n+after\n')
            for autocrlf in ('true', 'input', 'false'):
                with self.subTest(autocrlf=autocrlf):
                    (source / 'input.txt').write_bytes(b'before\r\n')
                    policy = {'GIT_CONFIG_COUNT': '2', 'GIT_CONFIG_KEY_0': 'core.autocrlf',
                              'GIT_CONFIG_VALUE_0': autocrlf, 'GIT_CONFIG_KEY_1': 'core.eol',
                              'GIT_CONFIG_VALUE_1': 'crlf'}
                    with patch.dict(os.environ, policy), patch.object(v.prep,'PATCHES',('change.patch',)):
                        v.prep.apply_patches(source,root)
                    self.assertEqual((source / 'input.txt').read_bytes(),b'after\n')
                    self.assertFalse((source / '.git').exists())

    def test_native_private_tls_source_selection_and_alpha_closure(self):
        cmake = ((ROOT / 'tests/downloader_trust/CMakeLists.txt').read_text() +
                 (ROOT / 'tests/downloader_trust/Targets.cmake').read_text())
        native = (ROOT / 'tools/test-downloader-trust-windows.ps1').read_text()
        recipe = (ROOT / 'cmake/ocharts-adapter/Targets.cmake').read_text()
        for helper in ('InputPaths.cmake', 'Targets.cmake'):
            self.assertIn('tests/downloader_trust/' + helper, v.prep.INPUTS)
        self.assertIn('--verify-prepared', cmake)
        self.assertIn('"${SKAGER_OCHARTS_PREPARED}/source/libs/wxcurl/src"', cmake)
        self.assertIn('"${trust_wxcurl_source}/base.cpp"', cmake)
        self.assertIn('"${trust_wxcurl_source}/http.cpp"', cmake)
        self.assertIn('"${trust_wxcurl_include}"', cmake)
        self.assertIn('model/src/downloader.cpp', cmake)
        self.assertNotIn('-DOPENNAV_WXCURL_TLS_TEST', native)
        self.assertIn("'evidence/local/ocharts-private-wxcurl-trust-windows'", native)
        self.assertIn('Private wxCurl probe dependency differs', native)
        self.assertIn('Private wxCurl runtime differs from locked wxWidgets', native)
        self.assertIn('src/integration/ChartNameAlphaWindows.cpp', v.prep.LOCAL)
        self.assertIn('"${SKAGER_PREPARED}/local/src/integration/ChartNameAlphaWindows.cpp"', recipe)
        self.assertIn('OpenGL::GL OpenGL::GLU gdiplus)', recipe)
        # Preserve every existing case and exact owned-CA cleanup boundary.
        for case in ('valid', 'https_redirect', 'http_downgrade', 'local_file_redirect',
                     'wrong_host', 'expired', 'untrusted', 'valid_unrelated_cwd', 'trust_removed'):
            self.assertIn("'" + case + "'", native)
        self.assertIn('Remove-OwnedTrust', native)
        self.assertIn("$Fields.bad_option_blocked -cne 'true'", native)
        self.assertIn("$GetOk -ne $Expected -or $HeadOk -ne $Expected", native)

    def test_native_recipe_include_source_closure(self):
        entry = ROOT / v.prep.RECIPE
        pending = [entry]
        visited = set()
        while pending:
            path = pending.pop()
            if path in visited:
                continue
            visited.add(path)
            self.assertIn(path.relative_to(ROOT).as_posix(), v.prep.INPUTS)
            for name in re.findall(r'include\("\$\{CMAKE_CURRENT_LIST_DIR\}/([^"\n]+)"\)', path.read_text()):
                pending.append(path.parent / name)
        self.assertEqual(len(visited), 3)
        entry_text = entry.read_text()
        self.assertLess(entry_text.index('PreparedPath.cmake'), entry_text.index('--verify-prepared'))
        self.assertLess(entry_text.index('--verify-prepared'), entry_text.index('Targets.cmake'))

    def test_trust_patch_policy(self):
        patch = (ROOT / 'patches/ocharts-wxcurl-trust.patch').read_text()
        added = '\n'.join(line[1:] for line in patch.splitlines() if line.startswith('+') and not line.startswith('+++'))
        for required in ('CURLOPT_SSL_VERIFYPEER, 1L', 'CURLOPT_SSL_VERIFYHOST, 2L',
                         'CURLSSLOPT_NATIVE_CA', 'if (!m_pCURL || !m_bHandleReady)',
                         'm_bHandleReady = false', 'CURLPROTO_HTTPS', 'CURLOPT_MAXREDIRS, 5L'):
            self.assertIn(required, added)
        self.assertNotIn('CAINFO', added)
        self.assertNotIn('OPENNAV_WXCURL_TLS_TEST', added)
        self.assertIn('-        SetOpt(CURLOPT_CAINFO, "curl-ca-bundle.crt")', patch)


if __name__ == '__main__': unittest.main()
