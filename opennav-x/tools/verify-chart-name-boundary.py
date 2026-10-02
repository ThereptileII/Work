#!/usr/bin/env python3
"""Run actual patched S52 text methods in a bounded native wx/recorded-GL fixture."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def block(text, token):
    start=text.index(token)
    body=text.index('{',start)
    depth=1;end=body+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]+'\n'

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wx-config',type=Path,required=True)
    p.add_argument('--wx-prefix',type=Path,required=True)
    a=p.parse_args();source=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    file=source/'libs/s52plib/src/s52plib.cpp';text=file.read_text()
    methods=['S52_TextC::S52_TextC()', 'S52_TextC::~S52_TextC()',
        'static void rotate(wxRect *r,', 'bool s52plib::RenderText(',
        'bool s52plib::CheckTextRectList(']
    extracted=''.join(block(text,m) for m in methods)
    caller=text[text.index('//      If this text was actually drawn, add a pointer'):]
    registration=block(caller,'    if (m_bDeClutterText)')
    extracted+='void s52plib::RegisterText(bool bwas_drawn,S52_TextC *text) {\nconst bool b_dupok=false;\n'+registration+'}\n'
    (out/'chart-name-render-methods.inc').write_text(extracted)
    config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)]
    cflags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
    libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))
    command=['g++','-std=c++17','-Wall','-Werror','-Wno-unused-variable',
        '-DOPENNAV_X','-DocpnUSE_GL',*cflags,'-I'+str(ROOT/'src'),'-I'+str(out)]
    for d in ('libs/s52plib/src','libs/geoprim/src'):
        command+=['-I'+str(source/d)]
    command+=[str(ROOT/'tests/chart_name_render_boundary_test.cpp'),*libs,'-o',str(out/'chart-name-boundary-test')]
    compile_result=subprocess.run(command,text=True,capture_output=True)
    (out/'compile.log').write_text(compile_result.stdout+compile_result.stderr)
    if compile_result.returncode:
        print(compile_result.stderr);compile_result.check_returncode()
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(a.wx_prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    result=subprocess.run([str(out/'chart-name-boundary-test')],env=env,text=True,capture_output=True)
    evidence={'actualSourceSha256':hashlib.sha256(file.read_bytes()).hexdigest(),
        'actualMethods':methods,'callerRegistrationVerbatim':True,
        'extractedSha256':hashlib.sha256(extracted.encode()).hexdigest(),
        'compileCommand':command,'exitCode':result.returncode,'output':result.stdout+result.stderr,
        'limitations':['GL calls record uploads; no actual GL context or driver draw',
                      'Viewport and text-owner containers are fixtures',
                      'Linux font raster; native Windows and boat acceptance remain open']}
    (out/'boundary.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(result.stdout+result.stderr,end='');result.check_returncode()

if __name__=='__main__':main()
