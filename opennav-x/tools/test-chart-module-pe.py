#!/usr/bin/env python3
"""Focused portable PE refusal tests; no DLL is executed."""
import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('prep_tests',ROOT/'tools/test-ocharts-adapter-preparation.py')
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)

# Independent private adapter ABI: do not derive expectations from either parser.
EXPORTS = (
    'create_pi', 'destroy_pi', 'skager_bind_chart_presentation_v1',
    'skager_chart_point_style_v1', 'skager_chart_presentation_status_v1',
    'skager_configure_chart_display_v1',
)

def valid(exported=None):
    data=fixtures.pe(exported=sorted(EXPORTS) if exported is None else exported)
    struct.pack_into('<II',data,256,0x1200,60)
    struct.pack_into('<IIIII',data,0x614,0x1540,0,0,0x1340,0x1540)
    name=b'wxmsw32u_core_vc14x.dll\0';data[0x740:0x740+len(name)]=name
    return data

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,default=ROOT,
                        help='Product source root containing src/integration/ChartModulePe.h')
    parser.add_argument('--adapter',type=Path,
                        help='Optionally inspect an actual adapter DLL as bytes; never load it')
    args=parser.parse_args()
    source_root=args.source_root.resolve()
    with tempfile.TemporaryDirectory(prefix='skager-module-pe-') as raw:
        work=Path(raw);source=work/'check.cpp'
        source.write_text('''#include "integration/ChartModulePe.h"
#include <fstream>
#include <iterator>
int main(int argc,char**argv){if(argc!=2)return 2;
std::ifstream f(argv[1],std::ios::binary);if(!f)return 2;
std::vector<unsigned char>b((std::istreambuf_iterator<char>(f)),{});
std::vector<std::string>i;return opennav::integration::ChartModulePe(b,i)?0:1;}
''')
        if os.name=='nt':
            # CMake selects the native x86 MSVC environment; cl/g++ need not be
            # on PATH in the early Windows contracts job.
            include=(source_root/'src').as_posix().replace('"','\\"')
            (work/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.24)
project(SkagerChartModulePe LANGUAGES CXX)
add_executable(check check.cpp)
target_compile_features(check PRIVATE cxx_std_17)
target_compile_options(check PRIVATE /W4 /utf-8)
target_include_directories(check PRIVATE "'''+include+'''")
''')
            subprocess.run(['cmake','-S',str(work),'-B',str(work/'build'),
                            '-G','Visual Studio 17 2022','-A','Win32'],check=True,timeout=120)
            subprocess.run(['cmake','--build',str(work/'build'),'--config','Release'],
                           check=True,timeout=120)
            exe=work/'build/Release/check.exe'
        else:
            exe=work/'check'
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-I'+str(source_root/'src'),
                            str(source),'-o',str(exe)],check=True,timeout=120)
        cases=[('valid',valid(),True),
               ('missing-observation-export',valid([n for n in EXPORTS if n!='skager_chart_point_style_v1']),False),
               ('missing-display-export-old-five',valid([n for n in EXPORTS if n!='skager_configure_chart_display_v1']),False),
               ('extra-export',valid([*EXPORTS,'unexpected_export']),False),
               ('wrong-export',valid([*EXPORTS[:-1],'wrong_export']),False)]
        for name,offset,fmt,value in [('machine',132,'<H',0x8664),('not-dll',150,'<H',0),
             ('optional',152,'<H',0x20b),('delay',248+13*8,'<I',0x1800),
             ('forwarder',0x800,'<I',0x1100),('ordinal',0x840,'<H',len(EXPORTS)),
             ('duplicate-ordinal',0x842,'<H',0),('name-rva',0x820,'<I',0xffffffff),
             ('export-count',0x514,'<I',3),('unterminated-import',260,'<I',40)]:
            data=valid();struct.pack_into(fmt,data,offset,value);cases.append((name,data,False))
        for name in (b'unknown.dll',b'libeay32.dll',b'oexserverd.exe'):
            data=valid();data[0x700:0x720]=b'\0'*32;data[0x700:0x700+len(name)]=name
            cases.append(('forbidden-'+name.decode(),data,False))
        data=valid();data[0x740]=ord('z');cases.append(('wrong-wx',data,False))
        for size in (0,62,140,400,2048):cases.append(('truncated-'+str(size),valid()[:size],False))
        for name,data,expected in cases:
            path=work/'candidate.bin';path.write_bytes(data)
            result=subprocess.run([str(exe),str(path)],timeout=5)
            if result.returncode != (0 if expected else 1):
                raise AssertionError(f'{name}: expected {0 if expected else 1}, got {result.returncode}')
        print(f'{len(cases)} actual portable PE parser cases passed; no native module executed')
        if args.adapter:
            adapter=args.adapter.resolve()
            if not adapter.is_file() or not 0<adapter.stat().st_size<=128*1024*1024:
                raise ValueError('Expected a bounded regular adapter file')
            subprocess.run([str(exe),str(adapter)],check=True,timeout=5)
            print(f'Actual adapter PE accepted as bytes only: {adapter}; '
                  f'sha256={hashlib.sha256(adapter.read_bytes()).hexdigest()}; no DLL loaded')

if __name__=='__main__':main()
