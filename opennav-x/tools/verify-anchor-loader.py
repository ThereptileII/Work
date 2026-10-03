#!/usr/bin/env python3
"""Focused Linux wx fixture: execute pinned symbol loader methods verbatim.

Requires wx-config, a wx-capable DISPLAY, g++, rsvg-convert and the pinned source.
No navigation/chart rendering is mocked as accepted. There is no GL context.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def method(source, name):
    match = re.search(r'^[^\n]*ChartSymbols::'+name+r'\(', source, re.M)
    assert match
    start = source.index('{', match.end())
    depth = 1
    end = start+1
    # These pinned functions have balanced braces in comments and strings too;
    # compilation and the source digest provide a separate extraction check.
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]+'\n'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--generated', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wx-config', type=Path, required=True)
    parser.add_argument('--wx-prefix', type=Path, required=True)
    parser.add_argument('--services', action='store_true', help='SCRUM-254 pilot/radar glyphs')
    parser.add_argument('--cardinals', action='store_true', help='SCRUM-256 classified Simplified cardinal glyphs')
    parser.add_argument('--seamarks', action='store_true', help='SCRUM-264 exact marine aliases and LIGHTS13')
    parser.add_argument('--private-source', type=Path, help='Optional pinned o-charts source for exact shared loader-body comparison')
    args = parser.parse_args()
    assert sum((args.services,args.cardinals,args.seamarks)) <= 1
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    source = args.source.resolve()
    original = source/'libs/s52plib/src/chartsymbols.cpp'
    methods = ['ChartSymbols','~ChartSymbols','InitializeTables','DeleteGlobals','ProcessColorTables',
               'ProcessSymbols','BuildSymbol','LoadRasterFileForColorTable',
               'FindColorTable','HashKey','GetImage','GetGLTextureRect']
    text = original.read_text()
    excerpts = ''.join(method(text,name) for name in methods)
    private_proof = None
    if args.private_source:
        assert args.seamarks
        path = args.private_source/'libs/s52plib/src/chartsymbols.cpp'
        content = path.read_bytes()
        identity = json.loads((ROOT/'tools/ocharts-adapter-source.lock.json').read_text())['source']['files']['libs/s52plib/src/chartsymbols.cpp']
        blob = hashlib.sha1(b'blob '+str(len(content)).encode()+b'\0'+content).hexdigest()
        assert len(content)==identity['bytes'] and blob==identity['gitBlob']
        shared = {}
        for name in ('ProcessSymbols','BuildSymbol','GetImage','GetGLTextureRect'):
            body=method(text,name)
            assert body==method(content.decode(),name), 'Private S52 loader boundary differs: '+name
            shared[name]=hashlib.sha256(body.encode()).hexdigest()
        private_proof={'source':str(path),'gitBlob':blob,'sha256':hashlib.sha256(content).hexdigest(),
                       'byteIdenticalExecutedMethods':shared,
                       'scope':'Actual owned-resource validation executes; shared method bodies match. Private DLL renderer and GL atlas upload remain unexecuted.'}
    (output/'anchor-loader-methods.inc').write_text(excerpts)
    if args.seamarks:
        conditional = (source/'libs/s52plib/src/s52cnsy.cpp').read_text()
        start = conditional.index('wxString _selSYcol(')
        end = conditional.index('\nstatic double _DEPVAL01',start)
        (output/'light-selector-method.inc').write_text(conditional[start:end])

    svg = ROOT/'resources/chart-style/v1/anchorage/ACHARE51.svg'
    png = output/'prototype-anchor.png'
    subprocess.run(['rsvg-convert',str(svg),'-o',str(png)],check=True)
    if args.services:
        for name in ('PILBOP02','RTPBCN02'):
            subprocess.run(['rsvg-convert',str(ROOT/'resources/chart-style/v1/services'/(name+'.svg')),'-o',str(output/(name+'.png'))],check=True)
        png = output
    if args.cardinals:
        for name in ('BOYCAR01','BOYCAR02','BOYCAR03','BOYCAR04'):
            for theme in ('DAY_BRIGHT','DUSK','NIGHT'):
                file=name+'-'+theme
                subprocess.run(['rsvg-convert',str(ROOT/'resources/chart-style/v1/cardinals'/(file+'.svg')),'-o',str(output/(file+'.png'))],check=True)
        png = output
    if args.seamarks:
        names=('XNLAT013','XNLAT014','XNLAT023','XNLAT024','XNCAN072','XNCAN073','XNCON066','XNCON067','BOYISD12','BOYSAW12','LIGHTS13')
        for name in names:
            for theme in ('DAY_BRIGHT','DUSK','NIGHT'):
                file=name+'-'+theme
                subprocess.run(['rsvg-convert',str(ROOT/'resources/chart-style/v1/seamarks'/(file+'.svg')),'-o',str(output/(file+'.png'))],check=True)
        png = output
    config = [str(args.wx_config),'--prefix='+str(args.wx_prefix)]
    cflags = shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
    libs = shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))
    command = ['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy',
               '-DocpnUSE_GL',*cflags]
    for directory in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml'):
        command += ['-I'+str(source/directory)]
    command += ['-I'+str(ROOT/'src'),'-I'+str(output),str(ROOT/('tests/chart_seamark_loader_test.cpp' if args.seamarks else 'tests/chart_cardinal_loader_test.cpp' if args.cardinals else 'tests/chart_service_loader_test.cpp' if args.services else 'tests/chart_anchor_loader_test.cpp')),
                str(source/'libs/pugixml/pugixml.cpp'),*libs,'-lGL','-lGLEW',
                '-o',str(output/'anchor-loader-test')]
    subprocess.run(command,check=True)
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = str(args.wx_prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    result = subprocess.run([str(output/'anchor-loader-test'),str(args.generated.resolve()),
                             str(png),str(output)],env=env,text=True,capture_output=True)
    (output/'loader.log').write_text(result.stdout+result.stderr)
    evidence = {'productionSource':str(original),
        'productionSourceSha256':hashlib.sha256(original.read_bytes()).hexdigest(),
        'unchangedMethods':methods,'excerptsSha256':hashlib.sha256(excerpts.encode()).hexdigest(),
        'fixtureContainers':'Only S52 owner containers are substituted; real pinned ProcessSymbols, BuildSymbol, PNG loader, GetImage, GetGLTextureRect execute.',
        'privateLoaderSourceProof':private_proof,
        'compileCommand':command,'exitCode':result.returncode,'output':result.stdout+result.stderr,
        'limitations':['No GL context or texture draw','No full chart canvas','Native Windows and boat display acceptance remain open']}
    (output/'loader.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(result.stdout+result.stderr,end='')
    result.check_returncode()


if __name__ == '__main__':
    main()
