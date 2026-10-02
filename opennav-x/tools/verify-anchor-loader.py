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
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    source = args.source.resolve()
    original = source/'libs/s52plib/src/chartsymbols.cpp'
    methods = ['ChartSymbols','~ChartSymbols','InitializeTables','DeleteGlobals','ProcessColorTables',
               'ProcessSymbols','BuildSymbol','LoadRasterFileForColorTable',
               'FindColorTable','HashKey','GetImage','GetGLTextureRect']
    text = original.read_text()
    excerpts = ''.join(method(text,name) for name in methods)
    (output/'anchor-loader-methods.inc').write_text(excerpts)
    svg = ROOT/'resources/chart-style/v1/anchorage/ACHARE51.svg'
    png = output/'prototype-anchor.png'
    subprocess.run(['rsvg-convert',str(svg),'-o',str(png)],check=True)
    config = [str(args.wx_config),'--prefix='+str(args.wx_prefix)]
    cflags = shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
    libs = shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))
    command = ['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy',
               '-DocpnUSE_GL',*cflags]
    for directory in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml'):
        command += ['-I'+str(source/directory)]
    command += ['-I'+str(output),str(ROOT/'tests/chart_anchor_loader_test.cpp'),
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
        'compileCommand':command,'exitCode':result.returncode,'output':result.stdout+result.stderr,
        'limitations':['No GL context or texture draw','No full chart canvas','Native Windows and boat display acceptance remain open']}
    (output/'loader.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(result.stdout+result.stderr,end='')
    result.check_returncode()


if __name__ == '__main__':
    main()
