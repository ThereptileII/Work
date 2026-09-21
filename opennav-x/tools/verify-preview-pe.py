#!/usr/bin/env python3
"""Check PE architecture and normal/delay import closure without build PATH.

Only actual Windows OS DLLs may be external. MSVC and wx/runtime dependencies
must be app-local even when a CI image happens to have global copies installed.
"""
import argparse
import json
import os
from pathlib import Path
import struct

def imports(path):
    data = path.read_bytes()
    u16 = lambda p: struct.unpack_from('<H', data, p)[0]
    u32 = lambda p: struct.unpack_from('<I', data, p)[0]
    pe = u32(0x3c)
    if data[:2] != b'MZ' or data[pe:pe+4] != b'PE\0\0':
        raise ValueError(f'Not a PE binary: {path}')
    if u16(pe+4) != 0x14c or u16(pe+24) != 0x10b:
        raise ValueError(f'Not the approved x86 application ABI: {path}')
    optional = pe+24
    sections = pe+24+u16(pe+20)
    ranges = [(u32(sections+i*40+12), max(u32(sections+i*40+8), u32(sections+i*40+16)),
               u32(sections+i*40+20)) for i in range(u16(pe+6))]
    def offset(rva):
        for start, size, raw in ranges:
            if start <= rva < start+size: return raw+rva-start
        raise ValueError(f'Invalid PE RVA {rva:x}: {path}')
    def name(rva):
        start=offset(rva)
        return data[start:data.index(b'\0', start)].decode('ascii').lower()
    found=set()
    table=u32(optional+96+8)
    if table:
        p=offset(table)
        while any(data[p:p+20]):
            found.add(name(u32(p+12)));p+=20
    delay=u32(optional+96+13*8)
    if delay:
        p=offset(delay)
        while any(data[p:p+32]):
            rva=u32(p+4)
            if not (u32(p)&1):rva-=u32(optional+28)
            found.add(name(rva));p+=32
    return sorted(found)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('app',type=Path);parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args();app=args.app.resolve()
    binaries=sorted(p for p in app.rglob('*') if p.suffix.lower() in ('.dll','.exe'))
    system=Path(os.environ['SystemRoot'])/'SysWOW64'
    report={'architecture':'x86 on Windows x64','dependencies':{},'missing':[]}
    for binary in binaries:
        dependencies=[]
        for dependency in imports(binary):
            local=next((p for folder in (binary.parent,app) for p in folder.iterdir() if p.name.lower()==dependency),None)
            mandatory=dependency != 'msvcrt.dll' and dependency.startswith(('msvcp','msvcr','vcruntime','vcomp','concrt','wx','lib','archive','zlib','glew'))
            if local:origin=str(local.relative_to(app))
            elif not mandatory and (dependency.startswith(('api-ms-win-','ext-ms-win-')) or (system/dependency).is_file()):origin='Windows OS'
            else:origin='MISSING';report['missing'].append({'binary':str(binary.relative_to(app)),'dll':dependency})
            dependencies.append({'dll':dependency,'origin':origin})
        report['dependencies'][str(binary.relative_to(app))]=dependencies
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    if report['missing']:raise SystemExit('Unbundled dependencies: '+json.dumps(report['missing']))
    print(f'Validated {len(binaries)} x86 binaries and their native import closure')

if __name__=='__main__':main()
