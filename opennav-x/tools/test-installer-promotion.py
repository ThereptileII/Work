#!/usr/bin/env python3
"""Focused retained-input and mode policy tests; no native acceptance claim."""
import argparse
import ast
import hashlib
import io
import importlib.util
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import unittest
import zipfile

SOURCE=Path(__file__).with_name('smoke-installer-windows.py')
TREE=ast.parse(SOURCE.read_text())
FUNCTIONS=ast.Module(body=[node for node in TREE.body if isinstance(node,ast.FunctionDef)
    and node.name in ('arguments','archive_members','prepare_retained','sha')],type_ignores=[])
ENV=dict(argparse=argparse,Path=Path,PurePosixPath=PurePosixPath,re=re,hashlib=hashlib,
         json=json,shutil=shutil,zipfile=zipfile)
exec(compile(FUNCTIONS,str(SOURCE),'exec'),ENV)
COMMIT='a'*40

def digest(data):return hashlib.sha256(data).hexdigest()
def zipped(entries):
    output=io.BytesIO()
    with zipfile.ZipFile(output,'w') as archive:
        for name,data in entries.items():
            # ZipInfo's constructor normalizes Windows backslashes. Assign
            # afterwards so malformed-member tests retain their actual bytes.
            entry=zipfile.ZipInfo()
            entry.filename=name
            archive.writestr(entry,data)
    return output.getvalue()

class Promotion(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)/'checkout'
        self.root.mkdir();self.inputs=Path(self.temp.name)/'retained';self.inputs.mkdir()
        (self.root/'tools').mkdir()
        (self.root/'tools/test-installer-missing-dll-selftest.ps1').write_text('current test helper')
        self.product={'commit':COMMIT,'test_fixtures':False,'build_purpose':'INSTALLED PRODUCT',
            'xnav_hardware_output_policy':'status-only','executable_sha256':digest(b'product')}
        self.entries={'app/opencpn.exe':b'product','docs/PRODUCT_BUILD.json':json.dumps(self.product).encode()}
        payload=zipped(self.entries)
        manifest={'commit':COMMIT,'payloadSha256':digest(payload),
            'files':[{'path':name,'sha256':digest(data)} for name,data in self.entries.items()]}
        self.support={'build/beta-installer/package.json':json.dumps(manifest).encode(),
            'build/beta-installer/payload.zip':payload,
            'build/production-windows/include/config.h':b'original profile version',
            'build/xnav-install/opencpn.exe':b'fixture',
            'build/xnav-windows/include/config.h':b'fixture profile version',
            'build/xnav-install/wx.dll':b'original fixture dependency'}
        self.engine='opennav-x/installer/windows/Lifecycle.ps1'
        self.reference={'productCommit':COMMIT,'files':{self.engine:{'sha256':digest(b'candidate engine')}}}
        self.build_inputs()
    def tearDown(self):self.temp.cleanup()
    def build_inputs(self,source_commit=COMMIT):
        self.reference['productCommit']=source_commit
        files={'SKAGER-Beta2-Setup.exe':b'original setup',
            'SKAGER-Beta2-Portable-Recovery.zip':zipped({'SKAGER-Beta2-Portable-Recovery/'+name:data for name,data in self.entries.items()}),
            'SKAGER-Beta2-Retest-Support.zip':zipped(self.support),
            'SKAGER-Beta2-source.zip':zipped({self.engine:b'candidate engine','SOURCE_REFERENCE.json':json.dumps(self.reference).encode()})}
        for name,data in files.items():(self.inputs/name).write_bytes(data)
        (self.inputs/'SHA256SUMS.txt').write_text(''.join(digest(data)+'  '+name+'\n' for name,data in files.items()))
    def restore(self):return ENV['prepare_retained'](self.inputs,self.root,COMMIT)
    def test_staging_is_default_and_production_is_explicit(self):
        self.assertEqual(ENV['arguments']([]).mode,'staging')
        self.assertEqual(ENV['arguments'](['--mode','production']).mode,'production')
    def test_retained_inputs_require_complete_paired_identity(self):
        from contextlib import redirect_stderr
        for args in (['--prepare-only'],['--retained-package','x'],['--expected-commit',COMMIT],
                     ['--retained-package','x','--expected-commit','abc']):
            with redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ENV['arguments'](args)
    def test_restores_same_bytes_and_separate_current_helper(self):
        report=self.restore()
        self.assertEqual(report['product_commit'],COMMIT)
        self.assertEqual((self.root/'build/production-install/opencpn.exe').read_bytes(),b'product')
        self.assertEqual((self.root/'build/xnav-install/wx.dll').read_bytes(),b'original fixture dependency')
        self.assertEqual((self.root/'build/retained-source/installer/windows/Lifecycle.ps1').read_bytes(),b'candidate engine')
        self.assertEqual((self.root/'build/retained-source/tools/test-installer-missing-dll-selftest.ps1').read_text(),'current test helper')
    def test_changed_setup_refused_before_restore(self):
        (self.inputs/'SKAGER-Beta2-Setup.exe').write_bytes(b'substituted')
        with self.assertRaisesRegex(ValueError,'checksum'):self.restore()
        self.assertFalse((self.root/'build').exists())
    def test_existing_build_is_never_overwritten(self):
        (self.root/'build/production-install').mkdir(parents=True)
        with self.assertRaisesRegex(ValueError,'fresh'):self.restore()
    def test_wrong_source_commit_is_refused(self):
        self.build_inputs(source_commit='b'*40)
        with self.assertRaisesRegex(ValueError,'source revision'):self.restore()
    def test_fixture_output_enabled_product_refused(self):
        self.product['xnav_hardware_output_policy']='enabled'
        self.entries['docs/PRODUCT_BUILD.json']=json.dumps(self.product).encode();self.build_inputs()
        with self.assertRaisesRegex(ValueError,'status-only'):self.restore()
    def test_unexpected_support_path_is_refused(self):
        self.support['tools/replace.py']=b'bad';self.build_inputs()
        with self.assertRaisesRegex(ValueError,'fixture installation only'):self.restore()
    def test_installer_and_recovery_must_identify_same_executable(self):
        self.entries['app/opencpn.exe']=b'substituted';self.build_inputs()
        with self.assertRaisesRegex(ValueError,'executable differs'):self.restore()
    def test_windows_unsafe_archive_names_are_refused(self):
        for name in ('../outside','app\\evil','C:/escape','app/trailing.','/absolute','app/./ambiguous','app/NUL.txt'):
            with self.subTest(name=name),zipfile.ZipFile(io.BytesIO(zipped({name:b'bad'}))) as archive:
                self.assertEqual(archive.infolist()[0].orig_filename,name)
                with self.assertRaisesRegex(ValueError,'Unsafe'):ENV['archive_members'](archive)
    def test_case_collisions_and_symlinks_are_refused(self):
        with zipfile.ZipFile(io.BytesIO(zipped({'app/A':b'a','app/a':b'b'}))) as archive:
            with self.assertRaisesRegex(ValueError,'duplicate'):ENV['archive_members'](archive)
        output=io.BytesIO()
        with zipfile.ZipFile(output,'w') as archive:
            entry=zipfile.ZipInfo('app/link');entry.external_attr=(0o120777<<16)
            archive.writestr(entry,'../../target')
        with zipfile.ZipFile(io.BytesIO(output.getvalue())) as archive:
            with self.assertRaisesRegex(ValueError,'Unsafe'):ENV['archive_members'](archive)

class FunctionalChart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('chart_render_check',SOURCE.with_name('chart-render-check.py'))
        cls.charts=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.charts)
    def frame(self,land,water):
        # Synthetic unit input only: exercise coverage/contrast rejection, never
        # substitute this frame for a native chart result or design reference.
        return (bytes(land)*640+bytes(water)*640)*800
    def test_arbitrary_distinct_chart_inks_do_not_require_prototype_palette(self):
        result=self.charts.functional(self.frame((60,70,80),(120,130,140)),'XNav','Day','unit')
        self.assertTrue(result['coastline_visible'])
    def test_functional_layout_accepts_other_geometry_and_rejects_obstruction(self):
        frame=dict(x=0,y=0,width=1280,height=800)
        controls=[dict(x=130+i*50,y=100,width=40,height=40,visible=True,enabled=True,label=label)
                  for i,label in enumerate(('Measure','Waypoint','+','−','Follow boat','Layers','North'))]
        controls.extend([dict(x=10,y=y,width=60,height=40,visible=True,enabled=True,label=label)
                         for y,label in ((100,'Chart'),(200,'Settings'))])
        controls.append(dict(x=900,y=720,width=80,height=40,visible=True,enabled=True,label='Source health'))
        display=dict(chart_region=dict(x=100,y=80,width=900,height=570),
            rail_regions=[dict(x=1040,y=80+i*90,width=100,height=60,visible=True,label=str(i)) for i in range(4)],
            interaction_controls=controls,footer_region=dict(x=0,y=700,width=1280,height=80),footer_middle_visible=True)
        self.charts.functional_layout(display,frame,frame)
        display['rail_regions'][0]['x']=150
        with self.assertRaisesRegex(AssertionError,'overlap'):
            self.charts.functional_layout(display,frame,frame)
    def test_route_visibility_requires_actual_stroke_at_projected_samples(self):
        rgb=bytearray(bytes((20,30,40))*1280*800)
        points=[dict(x=200,y=400),dict(x=1000,y=400)]
        with self.assertRaises(AssertionError):self.charts.route_visibility(rgb,points)
        for y in range(399,402):
            for x in range(200,1001):rgb[(y*1280+x)*3:(y*1280+x)*3+3]=bytes((80,180,100))
        self.assertEqual(len(self.charts.route_visibility(rgb,points)['samples']),5)
        for y in range(396,405):
            for x in range(596,605):rgb[(y*1280+x)*3:(y*1280+x)*3+3]=bytes((20,30,40))
        with self.assertRaises(AssertionError):self.charts.route_visibility(rgb,points)

    def test_blank_or_indistinguishable_chart_is_refused(self):
        for land,water in (((20,30,40),(20,30,40)),((20,30,40),(21,31,41))):
            with self.assertRaises(AssertionError):
                self.charts.functional(self.frame(land,water),'XNav','Day','unit')


if __name__=='__main__':unittest.main()
