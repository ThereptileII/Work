#!/usr/bin/env python3
"""Execute real pinned core/private lookup parsers and the shared yellow guard."""
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

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','private-source','resources','output','wx-config','wx-prefix'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)]
    flags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True))
    libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True))
    env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib')+':'+os.environ.get('LD_LIBRARY_PATH','')}
    results={}
    header=(ROOT/'src/integration/ChartYellowBuoySymbol.h').read_text()
    for label,path in [('core',a.source),('private',a.private_source)]:
        out=a.output/label;out.mkdir(exist_ok=True)
        file=path/'libs/s52plib/src/chartsymbols.cpp';raw=file.read_bytes()
        if label=='private':
            lock=json.loads((ROOT/'tools/ocharts-adapter-source.lock.json').read_text())['source']['files']['libs/s52plib/src/chartsymbols.cpp']
            assert len(raw)==lock['bytes'] and hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()==lock['gitBlob']
        methods=''.join(loader.method(raw.decode(),n) for n in ('ProcessLookups','BuildLookup'))
        (out/'yellow-lookup-methods.inc').write_text(methods)
        lookup=loader.function((path/'libs/s52plib/src/chartsymbols.h').read_text(),'class Lookup {')+';\n'
        (out/'yellow-lookup-type.inc').write_text(lookup)
        override=out/'override/integration';override.mkdir(parents=True,exist_ok=True)
        target=override/'ChartYellowBuoySymbol.h';target.write_text(header)
        command=['c++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-deprecated-copy',*flags,
                 *(['-DPRIVATE_LOOKUP'] if label=='private' else []),
                 '-I'+str(out/'override'),'-I'+str(ROOT/'src'),'-I'+str(out)]
        for directory in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml'):
            command+=['-I'+str(path/directory)]
        command += [str(ROOT/'tests/chart_yellow_lookup_parser_test.cpp'),str(a.source/'libs/pugixml/pugixml.cpp'),*libs,'-o',str(out/'fixture')]
        def execute():
            build=subprocess.run(command,env=env,capture_output=True,text=True)
            (out/'compile.log').write_text(build.stdout+build.stderr)
            if build.returncode: print(build.stdout+build.stderr)
            build.check_returncode()
            return subprocess.run([str(out/'fixture'),str(a.resources/'chartsymbols.xml')],capture_output=True,text=True,env=env)
        try:
            result=execute();(out/'run.log').write_text(result.stdout+result.stderr);result.check_returncode()
            negatives={}
            for name,changed in {
                'old-empty-only':header.replace("value.empty() || (value.length() == 1 && value[0] == '\\037')",'value.empty()'),
                'prefix-only':header.replace("value.length() == 1 && value[0] == '\\037'","!value.empty() && value[0] == '\\037'"),
                'ignore-rule-list':header.replace(' || rz->LUP->ruleList ||',' || false ||'),
            }.items():
                assert changed!=header;target.write_text(changed);negative=execute()
                (out/(name+'.log')).write_text(negative.stdout+negative.stderr)
                assert negative.returncode==1 and 'Yellow parser check' in negative.stderr
                negatives[name]=negative.returncode
        finally:
            target.write_text(header)
        subprocess.run(command,check=True,env=env,capture_output=True)
        results[label]={'sourceSha256':hashlib.sha256(raw).hexdigest(),'methodsSha256':hashlib.sha256(methods.encode()).hexdigest(),'lookupTypeSha256':hashlib.sha256(lookup.encode()).hexdigest(),'compileCommand':command,'result':result.stdout,'negativeControls':negatives}
        print(label,result.stdout.strip())
    (a.output/'receipt.json').write_text(json.dumps({'results':results,'headerSha256':hashlib.sha256(header.encode()).hexdigest(),'limits':'Real ProcessLookups/BuildLookup with actual core value/private pointer types and product guard; fixture containers/objects. No actual canvas, native Windows, GL, plugin or boat execution.'},indent=2)+'\n')

if __name__=='__main__':main()
