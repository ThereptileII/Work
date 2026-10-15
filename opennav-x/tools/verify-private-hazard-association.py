#!/usr/bin/env python3
"""Focused actual-source SCRUM-283 conditional/bridge proof; no app or DLL build."""
import argparse,hashlib,importlib.util,json,os,shlex,shutil,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def record(path):
 data=path.read_bytes()
 return {'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}

def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 return module

def bind_sources(core,original,patched):
 """Bind focused inputs to pins and the existing production patch function."""
 core_lock=json.loads((ROOT/'upstream.lock.json').read_text())
 core_names=('libs/s52plib/src/s52cnsy.cpp','libs/s52plib/src/s52s57.h',
             'libs/s52plib/src/s52utils.h','libs/s52plib/src/color_types.h',
             'libs/geoprim/src/bbox.h','gui/src/s57obj.cpp',
             'data/s57data/s57expectedinput.csv')
 for name in core_names:
  expected=subprocess.check_output(['git','-C',str(core),'show',core_lock['commit']+':'+name])
  if (core/name).read_bytes().replace(b'\r\n',b'\n')!=expected.replace(b'\r\n',b'\n'):
   raise ValueError('Core fixture source differs from pinned OpenCPN: '+name)
 prep=load('hazard_preparation',ROOT/'tools/prepare-ocharts-adapter.py')
 lock=json.loads((ROOT/prep.LOCK).read_text())
 locked={('opencpn-libs/' if key=='gitlink' else '')+name:identity
         for key in ('source','gitlink') for name,identity in lock[key]['files'].items()}
 available={p.relative_to(original).as_posix():p for p in original.rglob('*') if p.is_file()}
 for name,path in available.items():
  if name not in locked or path.is_symlink() or not prep.blob_ok(path.read_bytes(),locked[name]):
   raise ValueError('Original private source differs from lock: '+name)
 required=set(core_names[:5])|{'src/eSENCChart.cpp'}
 if not required.issubset(available):raise ValueError('Required private fixture sources missing')
 with tempfile.TemporaryDirectory(prefix='skager-hazard-source-') as raw:
  derived=Path(raw)/'source';shutil.copytree(original,derived)
  prep.apply_patches(derived,ROOT)
  expected={p.relative_to(derived).as_posix():record(p) for p in derived.rglob('*') if p.is_file()}
  for name,identity in expected.items():
   if (patched/name).is_symlink() or record(patched/name)!=identity:
    raise ValueError('Patched source differs from production patch composition: '+name)
 return {'coreCommit':core_lock['commit'],'core':{n:record(core/n) for n in core_names},
         'privateCommit':lock['source']['commit'],
         'original':{n:record(p) for n,p in available.items()},'patched':expected,
         'patches':{n:record(ROOT/n) for n in prep.PATCHES},
         'scope':'Focused source subset; not complete distributable source or application qualification'}

def main():
 if not __debug__:raise SystemExit('Run without -O: fixture parity assertions are required')
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--core',type=Path,required=True);p.add_argument('--private-original',type=Path,required=True);p.add_argument('--private-patched',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True)
 p.add_argument('--native-sdk-receipt',type=Path,help='Native only: existing workflow SDK verification receipt; caller must authenticate SDK first')
 a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 a.wx_prefix=a.wx_prefix.resolve()
 source_identity=bind_sources(a.core.resolve(),a.private_original.resolve(),a.private_patched.resolve())
 (out/'source-inputs.json').write_text(json.dumps(source_identity,indent=2)+'\n')
 extract=load('hazard_loader',ROOT/'tools/verify-anchor-loader.py').function
 signatures=('bool GetIntAttr(S57Obj *obj,','bool GetDoubleAttr(S57Obj *obj,','wxString *GetStringAttrWXS(','static int _parseList(','static double _DEPVAL01(','static wxString *_UDWHAZ03(','wxString *CSQUAPNT01(S57Obj *obj)\n','wxString SNDFRM02(S57Obj *obj, double depth_value_in)','static void *OBSTRN04(void *param)\n','static void *WRECKS02(void *param)\n')
 native=sys.platform=='win32';env=dict(os.environ);sdk_identity=None
 if native:
  # Compile only this existing fixture in an initialized MSVC x86 environment.
  # The caller supplies the authenticated SDK; no dependency fetch/build here.
  if os.environ.get('VSCMD_ARG_TGT_ARCH')!='x86':raise ValueError('Initialize the native MSVC x86 toolchain first')
  for name in ('CL','_CL_','CXXFLAGS','CFLAGS'):
   if os.environ.get(name):raise ValueError('Inherited compiler override refused: '+name)
  if not a.native_sdk_receipt:raise ValueError('Existing workflow SDK verification receipt required; authenticate SDK before invoking')
  json.loads(a.native_sdk_receipt.read_text(encoding='utf-8-sig'))
  sdk_identity={'receipt':str(a.native_sdk_receipt.resolve()),**record(a.native_sdk_receipt),
                'authentication':'Caller prerequisite; this fixture does not authenticate or qualify an SDK'}
  wxlib=a.wx_prefix/'lib/vc14x_dll'
  for name in ('wxbase32u.lib','wxmsw32u_core.lib','wxbase32u_vc14x.dll','wxmsw32u_core_vc14x.dll'):
   if not (wxlib/name).is_file():raise ValueError('Required Win32 wx SDK file missing: '+name)
  sdk_identity['consumedFiles']={str(path.relative_to(a.wx_prefix)):record(path) for path in
      [*(wxlib/name for name in ('wxbase32u.lib','wxmsw32u_core.lib','wxbase32u_vc14x.dll','wxmsw32u_core_vc14x.dll')),
       wxlib/'mswu/wx/setup.h',a.wx_prefix/'include/wx/version.h']}
  flags=['/DWXUSINGDLL','/D_UNICODE','/DUNICODE','/D__WXMSW__','/I'+str(a.wx_prefix/'include'),'/I'+str(wxlib/'mswu')]
  libs=['/link','/MACHINE:X86','/LIBPATH:'+str(wxlib),'wxbase32u.lib','wxmsw32u_core.lib']
  env['PATH']=str(wxlib)+os.pathsep+env.get('PATH','')
 else:
  wx=[str(a.wx_prefix/'bin/wx-config'),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(wx+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(wx+['--libs','core,base'],text=True));env['LD_LIBRARY_PATH']=str(a.wx_prefix/'lib')
 receipt=[];logs={}
 harness_files=('tools/verify-private-hazard-association.py','tests/private_hazard_association_test.cpp','tests/private_hazard_cases.inc','tests/private_hazard_allocation_test.cpp','tools/verify-anchor-loader.py','tools/prepare-ocharts-adapter.py','tools/ocharts-adapter-source.lock.json','upstream.lock.json','tools/windows-wx.lock.json')
 harness_identity={name:record(ROOT/name) for name in harness_files}
 (out/'harness.json').write_text(json.dumps({'commit':subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),'files':harness_identity,'platform':sys.platform,'compiler':'MSVC x86' if native else 'g++','nativeSdk':sdk_identity,'applicationBuilt':False,'runtimeProductQualification':False},indent=2)+'\n')
 for kind,source,defines in [('core',a.core,[]),('original-private',a.private_original,['-DPRIVATE_SOURCE','-DORIGINAL_PRIVATE']),('repaired-private',a.private_patched,['-DPRIVATE_SOURCE','-DPATCHED_PRIVATE','-DSKAGER_OCHARTS_ADAPTER']),('patched-stock',a.private_patched,['-DPRIVATE_SOURCE','-DORIGINAL_PRIVATE'])]:
  source=source.resolve();folder=out/kind;folder.mkdir(exist_ok=True);cnsy=source/'libs/s52plib/src/s52cnsy.cpp';text=cnsy.read_text(encoding='utf-8');idxsource=source/('gui/src/s57obj.cpp' if kind=='core' else 'src/eSENCChart.cpp');idxtext=idxsource.read_text(encoding='utf-8');index=extract(idxtext,'int S57Obj::GetAttributeIndex(' if kind=='core' else 'int S57Obj::GetAttributeIndex( ');start=text.index('#ifndef chk_snprintf');macro=text[start:text.index('#endif',start)+6]+'\n';bodies={s:extract(text,s) for s in signatures};(folder/'conditional.inc').write_text(macro+index+(extract(idxtext,'void S57Obj::Init()') if kind!='core' else '')+''.join(bodies.values()),encoding='utf-8')
  if kind=='repaired-private':(folder/'bridge.inc').write_text(extract(idxtext,'static std::list<S57Obj *> *SkagerAssociatedObjects('),encoding='utf-8')
  inc=[ROOT/'src',ROOT/'tests',folder,source/'libs/s52plib/src',source/'libs/geoprim/src'];binary=folder/('test.exe' if native else 'test');units=[str(ROOT/'tests/private_hazard_association_test.cpp'),*([str(ROOT/'tests/private_hazard_allocation_test.cpp')] if kind=='repaired-private' else [])]
  if native:
   cmd=['cl','/nologo','/std:c++17','/utf-8','/O2','/EHsc','/MD','/D_CRT_SECURE_NO_WARNINGS',*['/D'+d[2:] for d in defines],*flags,*['/I'+str(i) for i in inc],*units,'/Fo'+str(folder)+os.sep,'/Fe'+str(binary),*libs]
  else:
   cmd=['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-deprecated-copy',*defines,*flags,*['-I'+str(i) for i in inc],*units,*libs,'-o',str(binary)]
  r=subprocess.run(cmd,env=env,text=True,capture_output=True,timeout=120);(folder/'compile.log').write_text(r.stdout+r.stderr);entry={'kind':kind,'source':str(cnsy),'sha256':hashlib.sha256(cnsy.read_bytes()).hexdigest(),'bodies':{s:hashlib.sha256(v.encode()).hexdigest() for s,v in bodies.items()},'extracted':{p.name:record(p) for p in folder.glob('*.inc')},'command':cmd,'compileExit':r.returncode};receipt.append(entry);(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');r.check_returncode()
  if native:
   pe=binary.read_bytes();offset=int.from_bytes(pe[60:64],'little')
   assert pe[:2]==b'MZ' and pe[offset:offset+4]==b'PE\0\0' and int.from_bytes(pe[offset+4:offset+6],'little')==0x14c,'Native fixture must be Win32 x86'
   entry['machine']='I386'
  r=subprocess.run([str(binary)],env=env,text=True,capture_output=True,timeout=30);(folder/'run.log').write_text(r.stdout+r.stderr);entry['runExit']=r.returncode;entry['binarySha256']=hashlib.sha256(binary.read_bytes()).hexdigest();entry['headerSha256']=hashlib.sha256((source/'libs/s52plib/src/s52s57.h').read_bytes()).hexdigest();logs[kind]=r.stdout;print(kind,r.returncode,r.stdout.splitlines()[-1:] or r.stderr,flush=True);(out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');r.check_returncode()
  if kind=='original-private':
   n=subprocess.run([str(binary),'require-repair'],env=env,text=True,capture_output=True,timeout=30);(folder/'required-repair-negative.log').write_text(n.stdout+n.stderr);assert n.returncode==1 and 'association check' in n.stderr;entry['requiredRepairNegativeExit']=n.returncode
 # Preserve the original private Wk placement while comparing every other byte.
 def cases(s):return [line for line in s.splitlines() if line.startswith('case ')]
 core=cases(logs['core']);fixed=cases(logs['repaired-private']);assert len(core)==len(fixed)==26
 assert "TX('Wk',2,1,2,'15110',1,0,CHBLK,21)" in fixed[24]
 fixed[24]=fixed[24].replace("TX('Wk',2,1,2,'15110',1,0,CHBLK,21)","TX('Wk',3,1,2,'15110',2,0,CHBLK,21)")
 assert core==fixed
 assert cases(logs['original-private'])==cases(logs['patched-stock'])
 def exposures(s):return [line for line in s.splitlines() if line.startswith('exposure ')]
 core_exposure=exposures(logs['core']);fixed_exposure=exposures(logs['repaired-private'])
 assert len(core_exposure)==len(fixed_exposure)==288
 original_exposure=exposures(logs['original-private'])
 assert len(original_exposure)==288
 original_differences=sum(before!=after for before,after in zip(original_exposure,fixed_exposure))
 assert original_differences>0,'Original missing associated-area query must remain distinguishable'
 # Same documented private Wk placement exception as the original 26 cases.
 fixed_exposure=[line.replace("TX('Wk',2,1,2,'15110',1,0,CHBLK,21)","TX('Wk',3,1,2,'15110',2,0,CHBLK,21)") for line in fixed_exposure]
 assert core_exposure==fixed_exposure
 assert exposures(logs['original-private'])==exposures(logs['patched-stock'])
 layouts={k:next(line for line in v.splitlines() if line.startswith('context layout ')) for k,v in logs.items()}
 assert layouts['original-private']==layouts['patched-stock']
 assert layouts['original-private'].split()[-1]==layouts['repaired-private'].split()[-1]
 for root,key in ((a.core,'core'),(a.private_original,'original'),(a.private_patched,'patched'),(ROOT,'patches')):
  if any(record(root/name)!=identity for name,identity in source_identity[key].items()):raise ValueError('Source changed during fixture: '+key)
 if any(record(ROOT/name)!=identity for name,identity in harness_identity.items()):raise ValueError('Harness changed during fixture')
 if sdk_identity and record(a.native_sdk_receipt)!={k:sdk_identity[k] for k in ('bytes','sha256')}:raise ValueError('SDK receipt changed during fixture')
 if sdk_identity and any(record(a.wx_prefix/name)!=identity for name,identity in sdk_identity['consumedFiles'].items()):raise ValueError('SDK inputs changed during fixture')
 (out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');(out/'parity.json').write_text(json.dumps({'cases':26,'exposureCases':288,'exposureValues':['absent',1,2,3],'exposureProcedures':['UDWHAZ03','OBSTRN04','WRECKS02'],'originalPrivateExposureDifferences':original_differences,'result':'Exact conditional output and display-category parity except explicitly retained original private Wk alignment; cases18/19 additionally assert DisplayBase','originalPrivateNegative':'original-private/required-repair-negative.log','scope':'Actual conditional bodies and bridge with deterministic associated-list fixture; original geographic query not simulated or accepted'},indent=2)+'\n')
 print('26 core/repaired-private conditional outputs agree except retained original Wk alignment')
 print('288 exposure/depth/water-level outputs and display categories agree with pinned core; direct UDWHAZ03 promotion flags agree')
if __name__=='__main__':main()
