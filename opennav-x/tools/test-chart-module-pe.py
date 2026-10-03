#!/usr/bin/env python3
"""Focused portable PE refusal tests; no DLL is executed."""
import importlib.util
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('prep_tests',ROOT/'tools/test-ocharts-adapter-preparation.py')
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)

def valid():
    data=fixtures.pe()
    struct.pack_into('<II',data,256,0x1200,60)
    struct.pack_into('<IIIII',data,0x614,0x1540,0,0,0x1340,0x1540)
    name=b'wxmsw32u_core_vc14x.dll\0';data[0x740:0x740+len(name)]=name
    return data

def main():
    with tempfile.TemporaryDirectory(prefix='skager-module-pe-') as raw:
        work=Path(raw);source=work/'check.cpp';exe=work/'check'
        source.write_text('''#include "integration/ChartModulePe.h"
#include <fstream>
#include <iterator>
int main(int,char**argv){std::ifstream f(argv[1],std::ios::binary);
std::vector<unsigned char>b((std::istreambuf_iterator<char>(f)),{});
std::vector<std::string>i;return opennav::integration::ChartModulePe(b,i)?0:1;}
''')
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-I'+str(ROOT/'src'),str(source),'-o',str(exe)],check=True)
        cases=[('valid',valid(),True)]
        for name,offset,fmt,value in [('machine',132,'<H',0x8664),('not-dll',150,'<H',0),
             ('optional',152,'<H',0x20b),('delay',248+13*8,'<I',0x1800),
             ('forwarder',0x800,'<I',0x1100),('ordinal',0x840,'<H',4),
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
            if (result.returncode==0)!=expected:raise AssertionError(name)
        print(f'{len(cases)} actual portable PE parser cases passed; no native module executed')

if __name__=='__main__':main()
