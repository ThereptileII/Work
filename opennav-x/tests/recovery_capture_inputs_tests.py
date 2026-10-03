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


class DisposablePackage(unittest.TestCase):
    # Reuse the input builder without inheriting/rerunning the archive suite.
    write_product = PackageBoundary.write_product
    seal = PackageBoundary.seal
    verify = PackageBoundary.verify
    tearDown = PackageBoundary.tearDown

    def setUp(self):
        PackageBoundary.setUp(self)
        (self.package/'app/plugins').mkdir()
        (self.package/'app/plugins/dashboard_pi.dll').write_bytes(b'bundled identity, never executed')
        (self.package/'app/empty-resource-directory').mkdir()
        (self.package/'logs').mkdir()
        (self.package/'logs/old.log').write_text('must never copy old logs')
        (self.package/'profile/routes.gpx').write_text('must never copy routes')
        (self.package/'profile/plugins').mkdir()
        (self.package/'profile/plugins/unrelated.dll').write_bytes(b'must never copy profile plugins')
        self.seal()
        self.identity = self.verify()
        self.output = self.base/'output'
        self.output.mkdir()

    def stage(self):
        return inputs.stage_disposable_package(self.package, self.output, self.identity)

    def test_portable_paths_exact_payload_fresh_profile_and_mutable_logs(self):
        original = inputs._capture_tree(self.package)
        record = self.stage()
        copied = Path(record['root'])
        self.assertEqual(Path(record['executable']), copied/'app/opencpn.exe')
        self.assertEqual(Path(record['profile']), copied/'profile')
        self.assertEqual(Path(record['logs']), copied/'logs')
        self.assertEqual(Path(record['profile']).parent, Path(record['executable']).parent.parent)
        self.assertEqual((copied/'FILE_SHA256.json').read_bytes(), (self.package/'FILE_SHA256.json').read_bytes())
        for name in ('app/OPENNAV_PORTABLE_PREVIEW','app/opencpn.exe','app/plugins/dashboard_pi.dll','docs/PRODUCT_BUILD.json'):
            self.assertEqual((copied/name).read_bytes(), (self.package/name).read_bytes())
        config_text = (copied/'profile/opencpn.conf').read_text()
        config = inputs.configparser.ConfigParser(interpolation=None, strict=True)
        config.read_string(config_text)
        self.assertEqual(dict(config['Settings']), {
            'configversionstring': 'Version 5.12.4 Build 2026-10-03',
            'navmessageshown': '1', 'showstatusbar': '1', 'showmenubar': '1'})
        self.assertEqual(dict(config['Settings/GlobalState']), {
            'framewinx': '1280', 'framewiny': '800', 'framewinposx': '0',
            'framewinposy': '0', 'framemax': '0'})
        self.assertNotIn('DangerousConnection', config_text)
        self.assertNotIn('Plugins', config_text)
        self.assertEqual(sorted(p.name for p in (copied/'profile').iterdir()), ['OPENNAV_TEST_PROFILE','opencpn.conf'])
        self.assertEqual(list((copied/'logs').iterdir()), [])
        (copied/'profile/opencpn.conf').write_text('new disconnected test settings')
        (copied/'profile/runtime-cache').mkdir()
        (copied/'profile/runtime-cache/chart.bin').write_bytes(b'new generated cache')
        (copied/'logs/opennav-diagnostics.json').write_text('{}')
        self.assertEqual(inputs.verify_disposable_package(record), record)
        self.assertEqual(inputs._capture_tree(self.package), original)
        self.assertEqual(self.verify(), self.identity)

    def test_immutable_file_directory_marker_and_plugin_mutations_refused(self):
        record = self.stage(); copied = Path(record['root'])
        for name in ('app/opencpn.exe','app/OPENNAV_PORTABLE_PREVIEW','app/plugins/dashboard_pi.dll','FILE_SHA256.json'):
            path = copied/name; original = path.read_bytes()
            path.write_bytes(original+b'changed')
            with self.subTest(name=name), self.assertRaisesRegex(ValueError,'immutable package'):
                inputs.verify_disposable_package(record)
            path.unlink()
            with self.subTest(missing=name), self.assertRaises(ValueError):
                inputs.verify_disposable_package(record)
            path.write_bytes(original)
        for name, directory in [('app/unlisted.dll',False),('extra-empty',True)]:
            path = copied/name
            if directory: path.mkdir()
            else: path.write_bytes(b'extra')
            with self.subTest(extra=name), self.assertRaises(ValueError): inputs.verify_disposable_package(record)
            if directory: path.rmdir()
            else: path.unlink()
        for name in ('profile','logs'):
            path, alias = copied/name, copied/name.title()
            path.rename(alias)
            with self.subTest(alias=name), self.assertRaises((ValueError, FileNotFoundError)):
                inputs.verify_disposable_package(record)
            alias.rename(path)
        inputs.verify_disposable_package(record)

    def test_existing_overlap_and_output_alias_refused_without_original_mutation(self):
        original = inputs._capture_tree(self.package)
        for output in (self.package, self.package/'app', self.base):
            with self.subTest(output=output), self.assertRaisesRegex(ValueError,'overlap'):
                inputs.stage_disposable_package(self.package, output, self.identity)
        alias = self.output/'Disposable-Package'; alias.mkdir()
        with self.assertRaisesRegex(ValueError,'case alias'): self.stage()
        alias.rmdir()
        self.stage()
        with self.assertRaisesRegex(ValueError,'already exists'): self.stage()
        self.assertEqual(inputs._capture_tree(self.package), original)

    def test_changed_original_and_mutable_case_alias_refused_before_copy(self):
        path = self.package/'app/plugins/dashboard_pi.dll'; original = path.read_bytes()
        path.write_bytes(original+b'changed')
        with self.assertRaisesRegex(ValueError,'Original payload changed'): self.stage()
        self.assertFalse((self.output/'disposable-package').exists())
        path.write_bytes(original)
        for name in ('profile','logs'):
            path, alias = self.package/name, self.package/name.title()
            path.rename(alias)
            with self.subTest(name=name), self.assertRaisesRegex(ValueError,'Case-ambiguous|Case alias'): self.stage()
            alias.rename(path)
        self.assertFalse((self.output/'disposable-package').exists())

    def test_audited_equivalent_path_and_different_root_boundary(self):
        other = self.base/'other-package'; other.mkdir()
        wrong = dict(self.identity, root=str(other))
        with self.assertRaisesRegex(ValueError,'Verified original package path differs'):
            inputs.stage_disposable_package(self.package, self.output, wrong)
        # This real noncanonical spelling reproduces asymmetric normalization
        # without mocking Path.resolve or requiring an enabled Windows 8.3 volume.
        equivalent = self.package/'app'/'..'
        audited = self.verify(root=equivalent)
        self.assertNotEqual(Path(audited['root']), self.package.resolve())
        copied = inputs.stage_disposable_package(self.package, self.output, audited)
        self.assertEqual(Path(copied['original_root']), self.package.resolve())
        self.assertEqual(inputs.verify_disposable_package(copied), copied)
        self.assertEqual(self.verify(root=equivalent), audited)

    def test_links_in_original_output_and_mutable_copy_refused(self):
        probe = self.base/'link-probe'
        try: probe.symlink_to(self.package, target_is_directory=True)
        except OSError: self.skipTest('Symlink creation unavailable')
        with self.assertRaisesRegex(ValueError,'Linked/reparse'):
            inputs.stage_disposable_package(probe,self.output,self.identity)
        with self.assertRaisesRegex(ValueError,'Linked/reparse'):
            inputs.stage_disposable_package(self.package,self.output,dict(self.identity,root=str(probe)))
        probe.unlink(); probe.symlink_to(self.output,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'Linked/reparse'):
            inputs.stage_disposable_package(self.package,probe,self.identity)
        probe.unlink()
        original_link = self.package/'profile/linked'
        original_link.symlink_to(self.base/'missing')
        with self.assertRaisesRegex(ValueError,'Linked/reparse'): self.stage()
        original_link.unlink()
        record = self.stage(); copied = Path(record['root'])
        for name in ('profile/outside','logs/outside','app/outside'):
            link = copied/name; link.symlink_to(self.package, target_is_directory=True)
            with self.subTest(name=name), self.assertRaisesRegex(ValueError,'Linked/reparse'):
                inputs.verify_disposable_package(record)
            link.unlink()
        # The mutable roots themselves cannot become links either.
        shutil.rmtree(copied/'logs'); (copied/'logs').symlink_to(self.package/'logs',target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'Linked/reparse'): inputs.verify_disposable_package(record)

    def test_hardlink_special_file_and_missing_mutable_root_refused(self):
        import os
        record = self.stage(); copied = Path(record['root'])
        os.link(self.package/'app/opencpn.exe',copied/'profile/hardlink')
        with self.assertRaisesRegex(ValueError,'hard-linked'): inputs.verify_disposable_package(record)
        (copied/'profile/hardlink').unlink()
        if hasattr(os,'mkfifo'):
            os.mkfifo(copied/'logs/fifo')
            with self.assertRaisesRegex(ValueError,'Non-regular'): inputs.verify_disposable_package(record)
            (copied/'logs/fifo').unlink()
        (copied/'logs').rmdir()
        with self.assertRaises(FileNotFoundError): inputs.verify_disposable_package(record)


class NamedIhoScenes(unittest.TestCase):
    def test_named_source_views_and_wrong_scene_refusal(self):
        # Reuse actual historical canvas facts, combined only with the current
        # observer schema. This is a guard fixture, not a new runtime receipt.
        current = json.loads((ROOT/'docs/evidence/scrum264-yellow-e1d-linux/software/s64-yellow-SKAGER-Day.json').read_text())
        for name, center, scale, ids in (
                ('lateral', [-32.5186315, 61.0216421], .3, [219,224]),
                ('cardinals', [-32.37658945, 61.0300087], .12, [82,4,72,10])):
            with self.subTest(scene=name):
                scene = inputs.iho_scene(name)
                self.assertEqual(scene['center'], center)
                self.assertEqual(scene['requested_scale_ppm'], scale)
                self.assertEqual([f['attributes']['RCID'] for f in scene['source_features']], ids)
                self.assertIsNone(scene['pixel_proof'])
                self.assertIn('review-required', scene['visual_acceptance'])
                old = json.loads((ROOT/f'docs/evidence/scrum264-s64-9632421-linux/software/s64-{name}-SKAGER-Day.json').read_text())
                snapshot = copy.deepcopy(current)
                snapshot['runtime']['chart'] = old['runtime']['chart']
                inputs.presentation(snapshot, 'XNav', True, name)
                with self.assertRaisesRegex(ValueError, 'viewport'):
                    inputs.presentation(snapshot, 'XNav', True, 'yellow')
                for field, value in [('scale_ppm', scale+.01), ('quilt', False), ('database_entries', 2),
                                     ('quilt_members', []), ('follow', True)]:
                    wrong = copy.deepcopy(snapshot); wrong['runtime']['chart'][field] = value
                    with self.subTest(field=field), self.assertRaises(ValueError):
                        inputs.presentation(wrong, 'XNav', True, name)
        with self.assertRaisesRegex(ValueError, 'Unknown named'):
            inputs.iho_scene('arbitrary')

    def test_source_lock_allows_only_checkout_eol_and_rejects_changes(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            original = inputs.iho_scene('lateral')
            for item in (original['source_inventory'], original['observed_viewport_receipt']):
                target = root/item['path']; target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((ROOT/item['path']).read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
            copied = inputs.iho_scene('lateral',root)
            self.assertEqual(copied['center'],original['center'])
            self.assertEqual(copied['observed_scale_ppm'],original['observed_scale_ppm'])
            target = root/original['source_inventory']['path']
            target.write_bytes(target.read_bytes().replace(b'61.0216421',b'61.0216422'))
            with self.assertRaisesRegex(ValueError,'scene source changed'):
                inputs.iho_scene('lateral',root)

    def test_yellow_default_and_collector_dispatch_remain_bounded(self):
        import ast
        yellow = inputs.iho_scene()
        self.assertEqual(yellow,inputs.iho_scene('yellow'))
        self.assertEqual(yellow['center'],[-32.3471615,61.169588])
        self.assertEqual(yellow['observed_scale_ppm'],.5826126536)
        self.assertEqual(yellow['pixel_proof'],'exact-yellow-pair')
        tree = ast.parse((ROOT/'tools/prototype/capture-native.py').read_text())
        gates = [n for n in ast.walk(tree) if isinstance(n,ast.If) and
                 any(isinstance(c,ast.Call) and isinstance(c.func,ast.Name) and c.func.id=='iho_pixels'
                     for statement in n.body for c in ast.walk(statement))]
        exact = [n for n in gates if ast.unparse(n.test)=="args.iho_s64 and args.iho_scene == 'yellow'"]
        self.assertEqual(len(exact),1)
        self.assertEqual(sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='day_return'
                             for n in ast.walk(tree)),1)


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
