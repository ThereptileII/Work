#!/usr/bin/env python3
"""Compile only exact pinned loader + core/private pattern-cell methods under their own guards."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('loader',ROOT/'tools/verify-anchor-loader.py')
loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)
p=argparse.ArgumentParser()
for key in ('source','core','private','private-original','generated','output','wx-config','wx-prefix'):
    p.add_argument('--'+key,type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
text=(a.source/'libs/s52plib/src/chartsymbols.cpp').read_text()
private_path=a.private_original/'libs/s52plib/src/chartsymbols.cpp'
raw=private_path.read_bytes();lock=json.loads((ROOT/'tools/ocharts-adapter-source.lock.json').read_text())
identity=lock['source']['files']['libs/s52plib/src/chartsymbols.cpp']
assert len(raw)==identity['bytes'] and hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()==identity['gitBlob']
private_text=raw.decode().replace('\r\n','\n')
names=['ChartSymbols','~ChartSymbols','InitializeTables','DeleteGlobals','ProcessColorTables',
       'ProcessPatterns','BuildPattern','ProcessVectorTag','ProcessSymbols','BuildSymbol',
       'LoadRasterFileForColorTable','FindColorTable','HashKey','GetImage','GetGLTextureRect','GetColor']
for name in ('ProcessPatterns','BuildPattern','ProcessVectorTag','GetImage','GetGLTextureRect'):
    assert loader.method(text,name)==loader.method(private_text,name)
(a.output/'fishing-loader.inc').write_text(''.join(loader.method(text,n) for n in names))
signature='render_canvas_parms *s52plib::CreatePatternBufferSpec('
core=loader.function(a.core.read_text(),signature);private=loader.function(a.private.read_text(),signature)
assert core.count('#ifdef OPENNAV_X') == 1
assert private.count('#ifdef SKAGER_OCHARTS_ADAPTER') == 1
assert core==private.replace('#ifdef SKAGER_OCHARTS_ADAPTER', '#ifdef OPENNAV_X'), 'Actual core/private cell methods differ beyond their owner guard'
config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)]
command=['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy','-DocpnUSE_GL']
command+=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
for part in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml'):command+=['-I'+str(a.source/part)]
command+=['-I'+str(ROOT/'src'),'-I'+str(a.output),str(ROOT/'tests/chart_fishing_pattern_cell_test.cpp'),
          str(a.source/'libs/pugixml/pugixml.cpp'),str(a.source/'libs/geoprim/src/bbox.cpp')]
command+=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))+['-lGL','-lGLEW','-o',str(a.output/'fishing-test')]
env=dict(os.environ);env['LD_LIBRARY_PATH']=str(a.wx_prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
for label, body, guard in (('core',core,'OPENNAV_X'),('private',private,'SKAGER_OCHARTS_ADAPTER')):
    (a.output/'fishing-cell.inc').write_text(body)
    invocation=command+['-D'+guard]
    subprocess.run(invocation,check=True)
    r=subprocess.run([str((a.output/'fishing-test').resolve()),str(a.generated.resolve()),str(a.output.resolve())],text=True,capture_output=True,env=env)
    (a.output/(label+'-cell.log')).write_text(r.stdout+r.stderr)
    (a.output/(label+'-cell.json')).write_text(json.dumps({'source':str(a.source),'executedLoaderMethods':names,
        'privateByteIdenticalLoaderMethods':['ProcessPatterns','BuildPattern','ProcessVectorTag','GetImage','GetGLTextureRect'],
        'loaderSha256':hashlib.sha256((a.output/'fishing-loader.inc').read_bytes()).hexdigest(),
        'cellSha256':hashlib.sha256(body.encode()).hexdigest(),
        'command':invocation,'exitCode':r.returncode,'scope':'Actual loader/cell conversion under owner-specific guard; owner/HPGL fixtures; no polygon/GL/private DLL/native Windows/boat'},indent=2)+'\n')
    print(label+': '+r.stdout+r.stderr,end='');r.check_returncode()
