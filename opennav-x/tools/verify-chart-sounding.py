#!/usr/bin/env python3
"""Run actual patched S52 sounding and digit-atlas methods in a bounded native wx/recorded-GL fixture."""
import argparse
import hashlib
import json
import os
import re
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
    p.add_argument('--negative-legacy-rounding',action='store_true',help='Negative control: restore stock atlas point rounding')
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wx-config',type=Path,required=True)
    p.add_argument('--wx-prefix',type=Path,required=True)
    a=p.parse_args();source=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    file=source/'libs/s52plib/src/s52plib.cpp';text=file.read_text()
    methods=['bool s52plib::RenderSoundingSymbol(']
    extracted=block(text,methods[0])
    stock_text=(ROOT/'upstream/OpenCPN/libs/s52plib/src/s52plib.cpp').read_text()
    stock_method=block(stock_text,methods[0])
    # All coordinates, digit/pivot parsing, GL color/rotation and software draw
    # below font selection must remain the exact pinned implementation.
    assert extracted[extracted.index('  int pivot_x;'):]==stock_method[stock_method.index('  int pivot_x;'):]
    stock_font=stock_method[stock_method.index('  wxFontWeight fontWeight'):stock_method.index('\n  int pivot_x;')]
    assert stock_font.replace('  int charWidth, charHeight, charDescent;\n','') in extracted
    assert block(text,'int s52plib::RenderMPS(')==block(stock_text,'int s52plib::RenderMPS(')
    bridge=(ROOT/'src/integration/ChartPresentation.cpp').read_text()
    assert bridge.count('SetSoundingFontResolver(')==1
    success=bridge[bridge.index('      if (library->m_bOK) {'):bridge.index('      delete library;')]
    assert 'SetSoundingFontResolver(ChartSoundingFont)' in success
    css=(ROOT/'docs/design/prototype/src/style.css').read_text()
    sizes=re.findall(r'\.chart-depth\{[^}]*font-size:(\d+)px[^}]*\}',css)
    prototype_px=int(sizes[-1]);assert prototype_px==10
    prototype_stack=re.search(r'font-family:([^;}]+)',css).group(1)
    helper=(ROOT/'src/integration/ChartSoundingFont.h').read_text()
    for face in prototype_stack.split(',')[:-1]:assert face in helper
    depth=source/'libs/s52plib/src/DepthFont.cpp'
    atlas_methods=['DepthFont::DepthFont()', 'DepthFont::~DepthFont()',
        'void DepthFont::Build(', 'void DepthFont::Delete()',
        'bool DepthFont::GetGLTextureRect(']
    extracted=''.join(block(depth.read_text(),m) for m in atlas_methods)+extracted
    if a.negative_legacy_rounding:
        extracted=extracted.replace('exact_font ? font :','false ? font :')
    (out/'sounding-methods.inc').write_text(extracted)
    config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)]
    cflags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
    libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))
    command=['g++','-std=c++17','-Wall','-Werror','-Wno-unused-variable',
        '-DOPENNAV_X',*cflags,'-I'+str(ROOT/'src'),'-I'+str(out)]
    for d in ('libs/s52plib/src','libs/geoprim/src'):
        command+=['-I'+str(source/d)]
    command+=[str(ROOT/'tests/chart_sounding_render_test.cpp'),str(source/'libs/geoprim/src/bbox.cpp'),*libs,'-o',str(out/'chart-sounding-test')]
    compile_result=subprocess.run(command,text=True,capture_output=True)
    (out/'compile.log').write_text(compile_result.stdout+compile_result.stderr)
    if compile_result.returncode:
        print(compile_result.stderr);compile_result.check_returncode()
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(a.wx_prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    result=subprocess.run([str(out/'chart-sounding-test'),str(prototype_px),str(out/'digits.png')],env=env,text=True,capture_output=True)
    evidence={'actualSourceSha256':hashlib.sha256(file.read_bytes()).hexdigest(),
        'negativeLegacyRounding':a.negative_legacy_rounding,
        'actualMethods':methods,'atlasSourceSha256':hashlib.sha256(depth.read_bytes()).hexdigest(),
        'atlasMethods':atlas_methods,
        'prototype':{'sizePx':prototype_px,'stack':prototype_stack},
        'pinnedPainterTailAndStockFontBranchUnchanged':True,
        'pinnedRenderMPSUnchanged':True,'verifiedLibraryInstallationOnly':True,
        'extractedSha256':hashlib.sha256(extracted.encode()).hexdigest(),
        'compileCommand':command,'exitCode':result.returncode,'output':result.stdout+result.stderr,
        'limitations':['Actual DepthFont raster/upload, GL calls record textures; no driver draw',
                      'Viewport and text-owner containers are fixtures',
                      'Linux font raster; native Windows and boat acceptance remain open']}
    (out/'sounding.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(result.stdout+result.stderr,end='');result.check_returncode()

if __name__=='__main__':main()
