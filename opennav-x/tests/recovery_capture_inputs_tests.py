#!/usr/bin/env python3
"""Offline refusal and real retained-image probes; no executable is launched."""
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/prototype')]
import recovery_capture_inputs as inputs
from PIL import Image


class PackageBoundary(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.package = self.base/'package'
        self.package.mkdir()
        for directory in ('app', 'docs', 'profile'):
            (self.package/directory).mkdir()
        (self.package/'app/opencpn.exe').write_bytes(b'Test identity bytes only; never executed')
        (self.package/'app/OPENNAV_PORTABLE_PREVIEW').write_text('SKAGER portable Beta 2 recovery\n')
        self.commit = 'a'*40
        self.exe = inputs.sha(self.package/'app/opencpn.exe')
        self.product = dict(commit=self.commit, executable_sha256=self.exe, test_fixtures=False,
                            build_purpose='INSTALLED PRODUCT', xnav_hardware_output_policy='status-only')
        self.write_product()
        (self.package/'profile/opencpn.conf').write_text(
            '[Settings]\nConfigVersionString=Version 5.12.4 Build 2026-10-03\n'
            'DangerousConnection=must never copy\n[Plugins]\nUnrelated=must never copy\n')
        self.archive = self.base/'original.zip'
        self.seal()

    def tearDown(self):
        self.temp.cleanup()

    def write_product(self):
        (self.package/'docs/PRODUCT_BUILD.json').write_text(json.dumps(self.product))

    def seal(self):
        manifest = {p.relative_to(self.package).as_posix(): inputs.sha(p)
                    for p in self.package.rglob('*') if p.is_file() and p.name != 'FILE_SHA256.json'}
        (self.package/'FILE_SHA256.json').write_text(json.dumps(manifest))
        self.manifest_hash = inputs.sha(self.package/'FILE_SHA256.json')
        with zipfile.ZipFile(self.archive, 'w', zipfile.ZIP_DEFLATED) as output:
            for p in self.package.rglob('*'):
                if p.is_file():
                    output.write(p, 'SKAGER-Beta2-Portable-Recovery/'+p.relative_to(self.package).as_posix())
        self.archive_hash = inputs.sha(self.archive)

    def verify(self, **changes):
        kwargs = dict(root=self.package, expected_commit=self.commit, expected_exe=self.exe,
                      expected_manifest=self.manifest_hash, archive=self.archive, expected_archive=self.archive_hash)
        kwargs.update(changes)
        return inputs.verify_package(**kwargs)

    def test_valid_identity_and_version_only_profile(self):
        receipt = self.verify()
        profile = self.base/'new-profile'
        inputs.new_profile(profile, receipt['config_version_string'])
        text = (profile/'opencpn.conf').read_text()
        self.assertIn('ConfigVersionString=Version 5.12.4 Build 2026-10-03', text)
        self.assertNotIn('DangerousConnection', text)
        self.assertNotIn('Plugins', text)
        with self.assertRaises(FileExistsError):
            inputs.new_profile(profile, receipt['config_version_string'])

    def test_external_identity_refusals(self):
        for field, value in [('expected_commit','b'*40), ('expected_exe','b'*64),
                             ('expected_manifest','b'*64), ('expected_archive','b'*64),
                             ('expected_commit',None), ('expected_manifest',None)]:
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.verify(**{field:value})

    def test_semantic_capability_refusals_even_when_resealed(self):
        original = copy.deepcopy(self.product)
        for key, value in [('test_fixtures',True), ('build_purpose','TEST FIXTURES'),
                           ('xnav_hardware_output_policy','enabled'), ('commit','b'*40),
                           ('executable_sha256','b'*64)]:
            self.product = dict(original, **{key:value})
            self.write_product(); self.seal()
            with self.subTest(key=key), self.assertRaises(ValueError): self.verify()

    def test_changed_missing_extra_and_duplicate_inputs(self):
        path = self.package/'app/opencpn.exe'
        original = path.read_bytes()
        path.write_bytes(original+b'changed')
        with self.assertRaises(ValueError): self.verify()
        path.unlink()
        with self.assertRaises(FileNotFoundError): self.verify()
        path.write_bytes(original)
        (self.package/'unlisted.dll').write_bytes(b'extra')
        with self.assertRaises(ValueError): self.verify()
        (self.package/'unlisted.dll').unlink()
        manifest = self.package/'FILE_SHA256.json'
        manifest.write_text('{"x":"'+'a'*64+'","x":"'+'a'*64+'"}')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON'):
            self.verify(expected_manifest=inputs.sha(manifest))

    def test_zip_payload_must_match_bound_file_manifest(self):
        with zipfile.ZipFile(self.archive) as original:
            files = {item.filename:original.read(item) for item in original.infolist()}
        files['SKAGER-Beta2-Portable-Recovery/app/opencpn.exe'] += b'changed archive only'
        with zipfile.ZipFile(self.archive, 'w', zipfile.ZIP_DEFLATED) as replaced:
            for name, content in files.items(): replaced.writestr(name, content)
        with self.assertRaisesRegex(ValueError, 'ZIP payload differs'):
            self.verify(expected_archive=inputs.sha(self.archive))

    def test_zip_nonregular_metadata_refused_even_with_audited_hash(self):
        with zipfile.ZipFile(self.archive) as original:
            files = {item.filename: original.read(item) for item in original.infolist()}
        name = 'SKAGER-Beta2-Portable-Recovery/app/opencpn.exe'
        for attrs in ((inputs.stat.S_IFLNK | 0o777) << 16,
                      (inputs.stat.S_IFIFO | 0o600) << 16,
                      (inputs.stat.S_IFCHR | 0o600) << 16,
                      ((inputs.stat.S_IFREG | 0o600) << 16) | 0x400,
                      ((inputs.stat.S_IFREG | 0o600) << 16) | 0x10):
            with zipfile.ZipFile(self.archive, 'w') as replaced:
                for key, content in files.items():
                    item = zipfile.ZipInfo(key)
                    if key == name: item.external_attr = attrs
                    replaced.writestr(item, content)
            with self.subTest(attrs=attrs), self.assertRaisesRegex(ValueError, 'Non-regular/link/reparse'):
                self.verify(expected_archive=inputs.sha(self.archive))

    def test_marker_and_product_json_type_refusals(self):
        marker = self.package/'app/OPENNAV_PORTABLE_PREVIEW'
        marker.write_text('wrong marker\n'); self.seal()
        with self.assertRaisesRegex(ValueError, 'portable marker content'): self.verify()
        marker.write_bytes(b'SKAGER portable Beta 2 recovery\r\n'); self.seal()
        self.verify()  # Native Windows line ending from package-preview.py.
        for product in ([], 'status-only', None, False):
            self.product = product; self.write_product(); self.seal()
            with self.subTest(product=product), self.assertRaisesRegex(ValueError, 'status-only'):
                self.verify()

    def test_path_escape_and_link_refused(self):
        manifest = self.package/'FILE_SHA256.json'
        original = manifest.read_bytes()
        for name in ('../escape','/absolute','app\\name','app/name:stream','app/../escape','app/name.'):
            manifest.write_text(json.dumps({name:'a'*64}))
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.verify(expected_manifest=inputs.sha(manifest))
        manifest.write_bytes(original)
        link = self.package/'unlisted-link'
        try: link.symlink_to(self.base/'outside')
        except OSError: self.skipTest('Symlink creation is not available to this account')
        with self.assertRaises(ValueError): self.verify()

    def test_missing_duplicate_profile_version(self):
        path = self.package/'profile/opencpn.conf'
        for text in ('[Settings]\nOther=1\n', '[Settings]\nConfigVersionString=wrong\n',
                     '[Settings]\nConfigVersionString=Version a Build b\nConfigVersionString=Version c Build d\n'):
            path.write_text(text); self.seal()
            with self.subTest(text=text), self.assertRaises((ValueError, inputs.configparser.Error)):
                self.verify()

    def test_iho_wrong_source_refused_without_copy(self):
        source = self.base/'GB4X0000.000';source.write_bytes(b'not an IHO cell')
        output = self.base/'scene'
        with self.assertRaises(ValueError): inputs.stage_iho(source, output)
        self.assertFalse(output.exists())


class ActualRetainedPixels(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.output = Path(self.temp.name)
        self.base = ROOT/'docs/evidence/scrum264-yellow-e1d-linux'

    def tearDown(self): self.temp.cleanup()

    def source(self, renderer='software', style='SKAGER', theme='Day'):
        stem = f's64-yellow-{style}-{theme}'
        image = self.output/(renderer+'-'+stem+'.png')
        shutil.copyfile(self.base/renderer/(stem+'.png'), image)
        snapshot = json.loads((self.base/renderer/(stem+'.json')).read_text())
        return image, snapshot

    def test_real_recorded_pair_probes(self):
        for renderer in ('software','opengl'):
            for style in ('SKAGER','Standard'):
                for theme in ('Day','Dusk','Night'):
                    image, snapshot = self.source(renderer,style,theme)
                    code = 'XNav' if style=='SKAGER' else 'Standard'
                    inputs.presentation(snapshot,code,True)
                    inputs.iho_pixels(image,snapshot,(0,0),code,renderer,ROOT)

    def test_missing_head_or_body_cannot_pass(self):
        for bounds in ((-4,-11,4,-4),(-1,3,2,10)):
            image,snapshot = self.source()
            r=snapshot['runtime']['display']['chart_region'];x=r['x']+507;y=r['y']+283
            with Image.open(image) as im:
                im=im.convert('RGB');a,b,c,d=bounds
                im.paste(im.getpixel((x+30,y+30)),(x+a,y+b,x+c,y+d));im.save(image)
            with self.subTest(bounds=bounds),self.assertRaises(ValueError):
                inputs.iho_pixels(image,snapshot,(0,0),'XNav','software',ROOT)

    def test_viewport_private_state_or_application_identity_refused(self):
        _,original=self.source()
        for section,field,value in [('chart','latitude',0),('chart','scale_ppm',.6),
                                    ('chart','database_entries',2),('chart','follow',True)]:
            snapshot=copy.deepcopy(original);snapshot['runtime'][section][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):inputs.presentation(snapshot,'XNav',True)
        snapshot=copy.deepcopy(original);snapshot['runtime']['chart_presentation']['private_ocharts']={'available':True}
        with self.assertRaises(ValueError): inputs.presentation(snapshot,'XNav',True)
        with self.assertRaises(ValueError): inputs.runtime_identity(original,'b'*40,True)

    def test_whole_chart_return_keeps_known_standard_failure(self):
        first,snapshot=self.source('opengl','Standard')
        returned=self.output/'return.png'
        shutil.copyfile(self.base/'opengl/s64-yellow-Standard-Day-return.png',returned)
        with self.assertRaisesRegex(ValueError,'Whole chart Day-return'):
            inputs.day_return(returned,first,snapshot,(0,0))
        inputs.day_return(first,first,snapshot,(0,0))


if __name__ == '__main__': unittest.main()
