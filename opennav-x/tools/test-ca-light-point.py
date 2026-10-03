#!/usr/bin/env python3
"""Bounded SCRUM-275 classifier/lifetime and actual CA dispatch-method proof.

Terminal arc/raster painters and projection are recorded stubs. This is not
actual software/GL canvas acceptance; use the exact candidate chart captures.
"""
import argparse,hashlib,importlib.util,json,os,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('loader',ROOT/'tools/verify-anchor-loader.py');loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--private-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-config',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True));receipts=[]
 for name,source,guard,gl in [('core',a.source,'OPENNAV_X',True),('private',a.private_source,'SKAGER_OCHARTS_ADAPTER',True),('core-no-gl',a.source,'OPENNAV_X',False)]:
  out=a.output/name;out.mkdir(exist_ok=True);cpp=source/'libs/s52plib/src/s52plib.cpp';header=source/'libs/s52plib/src/s52plib.h'
  signatures=['void s52plib::RenderPresentationCaLightPoint(','int s52plib::RenderCARC(']
  methods={sig:loader.function(cpp.read_text(),sig) for sig in signatures};(out/'ca-methods.inc').write_text(''.join(methods.values()))
  (out/'scope-methods.inc').write_text(loader.function(header.read_text(),'bool PresentationCaLightsEnabled(')+loader.function(header.read_text(),'opennav::integration::CaLightPointInventory* SetPresentationCaLights('))
  conditional=(source/'libs/s52plib/src/s52cnsy.cpp').read_text()
  conditional_methods={sig:loader.function(conditional,sig) for sig in ('bool GetDoubleAttr(S57Obj *obj,','bool GetStringAttr(S57Obj *obj,','static int _parseList(','wxString _selSYcol(','static void *LIGHTS06(void *param)\n')}
  index=loader.function((a.source/'gui/src/s57obj.cpp').read_text(),'int S57Obj::GetAttributeIndex(')
  (out/'ca-conditional.inc').write_text(index+''.join(conditional_methods.values()))
  cmd=['g++','-std=c++17','-O3','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy','-D'+guard,*(['-DocpnUSE_GL'] if gl else []),*flags]
  cmd+=['-I'+str(source/x) for x in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml')]+['-I'+str(ROOT/'src'),'-I'+str(out),str(ROOT/'tests/chart_ca_light_point_test.cpp'),*libs,'-lGL','-lGLEW','-o',str(out/'test')]
  env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib')};built=subprocess.run(cmd,capture_output=True,text=True,env=env);(out/'compile.log').write_text(built.stdout+built.stderr);built.check_returncode();ran=subprocess.run([str(out/'test')],capture_output=True,text=True,env=env);(out/'run.log').write_text(ran.stdout+ran.stderr);print(name,ran.stdout+ran.stderr,flush=True);ran.check_returncode()
  receipts.append({'name':name,'command':cmd,'sourceSha256':sha(cpp),'headerSha256':sha(header),'methodSha256':{key:hashlib.sha256(value.encode()).hexdigest() for key,value in {**methods,**conditional_methods}.items()},'stdout':ran.stdout,'exit':ran.returncode,'executableSha256':sha(out/'test')})
 (a.output/'receipt.json').write_text(json.dumps({'helperSha256':sha(ROOT/'src/integration/ChartCaLightPoint.h'),'fixtureSha256':sha(ROOT/'tests/chart_ca_light_point_test.cpp'),'runs':receipts,'limits':'Real production methods; terminal painters/projection recorded. No native canvas, private DLL, boat or full application execution.'},indent=2)+'\n')
if __name__=='__main__':main()
