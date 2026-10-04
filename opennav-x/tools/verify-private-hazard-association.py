#!/usr/bin/env python3
"""Focused actual-source SCRUM-283 conditional/bridge proof; no app or DLL build."""
import argparse,hashlib,importlib.util,json,os,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--core',type=Path,required=True);p.add_argument('--private-original',type=Path,required=True);p.add_argument('--private-patched',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 spec=importlib.util.spec_from_file_location('loader',ROOT/'tools/verify-anchor-loader.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);extract=module.function
 # Refuse unpinned original private inputs before extracting any test bodies.
 lock=json.loads((ROOT/'tools/ocharts-adapter-source.lock.json').read_text())['source']['files']
 for name in ('libs/s52plib/src/s52cnsy.cpp','libs/s52plib/src/s52s57.h','src/eSENCChart.cpp'):
  data=(a.private_original/name).read_bytes();expected=lock[name]
  assert len(data)==expected['bytes'] and hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==expected['gitBlob'],name
 signatures=('bool GetIntAttr(S57Obj *obj,','bool GetDoubleAttr(S57Obj *obj,','wxString *GetStringAttrWXS(','static int _parseList(','static double _DEPVAL01(','static wxString *_UDWHAZ03(','wxString *CSQUAPNT01(S57Obj *obj)\n','wxString SNDFRM02(S57Obj *obj, double depth_value_in)','static void *OBSTRN04(void *param)\n','static void *WRECKS02(void *param)\n')
 wx=[str(a.wx_prefix/'bin/wx-config'),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(wx+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(wx+['--libs','core,base'],text=True));env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib')};receipt=[];logs={}
 for kind,source,defines in [('core',a.core,[]),('original-private',a.private_original,['-DPRIVATE_SOURCE','-DORIGINAL_PRIVATE']),('repaired-private',a.private_patched,['-DPRIVATE_SOURCE','-DPATCHED_PRIVATE','-DSKAGER_OCHARTS_ADAPTER']),('patched-stock',a.private_patched,['-DPRIVATE_SOURCE','-DORIGINAL_PRIVATE'])]:
  source=source.resolve();folder=out/kind;folder.mkdir(exist_ok=True);cnsy=source/'libs/s52plib/src/s52cnsy.cpp';text=cnsy.read_text();idxsource=source/('gui/src/s57obj.cpp' if kind=='core' else 'src/eSENCChart.cpp');idxtext=idxsource.read_text();index=extract(idxtext,'int S57Obj::GetAttributeIndex(' if kind=='core' else 'int S57Obj::GetAttributeIndex( ');start=text.index('#ifndef chk_snprintf');macro=text[start:text.index('#endif',start)+6]+'\n';bodies={s:extract(text,s) for s in signatures};(folder/'conditional.inc').write_text(macro+index+(extract(idxtext,'void S57Obj::Init()') if kind!='core' else '')+''.join(bodies.values()))
  if kind=='repaired-private':(folder/'bridge.inc').write_text(extract(idxtext,'static std::list<S57Obj *> *SkagerAssociatedObjects('))
  inc=[ROOT/'src',ROOT/'tests',folder,source/'libs/s52plib/src',source/'libs/geoprim/src'];cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy',*defines,*flags,*['-I'+str(i) for i in inc],str(ROOT/'tests/private_hazard_association_test.cpp'),*([str(ROOT/'tests/private_hazard_allocation_test.cpp')] if kind=='repaired-private' else []),*libs,'-o',str(folder/'test')]
  r=subprocess.run(cmd,env=env,text=True,capture_output=True);(folder/'compile.log').write_text(r.stdout+r.stderr);entry={'kind':kind,'source':str(cnsy),'sha256':hashlib.sha256(cnsy.read_bytes()).hexdigest(),'bodies':{s:hashlib.sha256(v.encode()).hexdigest() for s,v in bodies.items()},'command':cmd,'compileExit':r.returncode};receipt.append(entry);(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');r.check_returncode()
  r=subprocess.run([str(folder/'test')],env=env,text=True,capture_output=True);(folder/'run.log').write_text(r.stdout+r.stderr);entry['runExit']=r.returncode;entry['binarySha256']=hashlib.sha256((folder/'test').read_bytes()).hexdigest();entry['headerSha256']=hashlib.sha256((source/'libs/s52plib/src/s52s57.h').read_bytes()).hexdigest();logs[kind]=r.stdout;print(kind,r.returncode,r.stdout.splitlines()[-1:] or r.stderr,flush=True);(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');r.check_returncode()
  if kind=='original-private':
   n=subprocess.run([str(folder/'test'),'require-repair'],env=env,text=True,capture_output=True);(folder/'required-repair-negative.log').write_text(n.stdout+n.stderr);assert n.returncode==1 and 'association check' in n.stderr;entry['requiredRepairNegativeExit']=n.returncode
 # Preserve the original private Wk placement while comparing every other byte.
 def cases(s):return [line for line in s.splitlines() if line.startswith('case ')]
 core=cases(logs['core']);fixed=cases(logs['repaired-private']);assert len(core)==len(fixed)==26
 assert "TX('Wk',2,1,2,'15110',1,0,CHBLK,21)" in fixed[24]
 fixed[24]=fixed[24].replace("TX('Wk',2,1,2,'15110',1,0,CHBLK,21)","TX('Wk',3,1,2,'15110',2,0,CHBLK,21)")
 assert core==fixed
 assert cases(logs['original-private'])==cases(logs['patched-stock'])
 layouts={k:next(line for line in v.splitlines() if line.startswith('context layout ')) for k,v in logs.items()}
 assert layouts['original-private']==layouts['patched-stock']
 assert layouts['original-private'].split()[-1]==layouts['repaired-private'].split()[-1]
 (out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');(out/'parity.json').write_text(json.dumps({'cases':26,'result':'Exact conditional output and display-category parity except explicitly retained original private Wk alignment; cases18/19 additionally assert DisplayBase','originalPrivateNegative':'original-private/required-repair-negative.log','scope':'Actual conditional bodies and bridge with deterministic associated-list fixture; original geographic query not simulated or accepted'},indent=2)+'\n')
 print('26 core/repaired-private conditional outputs agree except retained original Wk alignment')
if __name__=='__main__':main()
