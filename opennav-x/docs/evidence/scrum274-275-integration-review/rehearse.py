from pathlib import Path
import hashlib,importlib.util,json,re,shutil,subprocess,tempfile,sys
root=Path.cwd();out=root/'.local/integration-review';core=out/'core';core.mkdir()
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
def rec(p):
 d=p.read_bytes();return {'bytes':len(d),'sha256':hashlib.sha256(d).hexdigest()}
report={'scope':'isolated composition/static source closure only','base':'214b2d057a05d14a0c30dcc112fe9e6676c0ef63','combined':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()}
source=Path('/home/standard/Projects/X-nav/upstream/OpenCPN');pin=json.loads((root/'upstream.lock.json').read_text())['commit'];report['corePin']=pin
p=subprocess.Popen(['git','archive',pin],cwd=source,stdout=subprocess.PIPE)
subprocess.run(['tar','-x','-C',str(core)],stdin=p.stdout,check=True);p.stdout.close();assert p.wait()==0
core_before=(core/'libs/s52plib/src/s52plib.cpp').read_text()
patches=re.findall(r"root / '(patches/[^']+)'",(root/'tools/prepare-integration.py').read_text());assert len(patches)==9
with tempfile.TemporaryDirectory(dir=out) as tmp:
 subprocess.run(['git','init','--bare','-q',tmp],check=True)
 command=['git','-c','core.bare=false','-c','core.autocrlf=false','--git-dir='+tmp,'--work-tree='+str(core),'apply']
 for name in patches:
  data=(root/name).read_bytes().replace(b'\r\n',b'\n')
  subprocess.run(command+['--check','-'],cwd=core,input=data,check=True)
  subprocess.run(command+['-'],cwd=core,input=data,check=True)
report['orderedCorePatches']={name:rec(root/name) for name in patches}
prep=load('adapter_prep',root/'tools/prepare-ocharts-adapter.py');lock=json.loads((root/prep.LOCK).read_text());original=out/'private-original';original.mkdir();blob_cache=Path('/home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/blobs');proof={}
for kind in ('source','gitlink'):
 for name,expected in lock[kind]['files'].items():
  path=Path('opencpn-libs' if kind=='gitlink' else '')/name
  data=(blob_cache/expected['gitBlob']).read_bytes();assert prep.blob_ok(data,expected),str(path)
  target=original/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
  proof[path.as_posix()]=rec(target)
private=out/'private';shutil.copytree(original,private);private_before=(private/'libs/s52plib/src/s52plib.cpp').read_text();prep.apply_patches(private,root);prep.copy_local_inputs(out/'private-prepared')
report['privateOriginalInputs']=proof;report['orderedPrivatePatches']={name:rec(root/name) for name in prep.PATCHES};report['privateLocalInputs']={name:rec(out/'private-prepared/local'/name) for name in prep.LOCAL}
helper=load('extract',root/'tools/verify-anchor-loader.py');report['unchangedPainters']={};report['scopes']={}
for kind,before,directory in [('core',core_before,core),('private',private_before,private)]:
 after=(directory/'libs/s52plib/src/s52plib.cpp').read_text();report['unchangedPainters'][kind]={}
 for sig in ('int s52plib::RenderCARC_GLSL(','int s52plib::RenderCARC_VBO(','bool s52plib::RenderRasterSymbol('):
  body=helper.function(after,sig);assert body==helper.function(before,sig);report['unchangedPainters'][kind][sig]=hashlib.sha256(body.encode()).hexdigest()
 for filename,cases in ([('gui/src/s57chart.cpp',{'bool s57chart::DoRenderOnGL(':1,'bool s57chart::DCRenderLPB(':1})] if kind=='core' else [('src/eSENCChart.cpp',{'bool eSENCChart::DoRender2RectOnGL(':2,'bool eSENCChart::DCRenderLPB(':1})]):
  for sig,count in cases.items():
   body=helper.function((directory/filename).read_text(),sig);assert body.count('CaLightPointScope<s52plib>')==count;report['scopes'][kind+':'+sig]=count
ix=core/'libs/IXWebSocket';expected_ix=Path('/home/standard/Projects/X-nav-worktrees/scrum274-ais-connection-observation/.local/verify-final/libs/IXWebSocket');report['ixInputs']={p.relative_to(ix).as_posix():rec(p) for p in sorted(ix.rglob('*')) if p.is_file()}
for name,expected in report['ixInputs'].items():assert rec(expected_ix/name)==expected,name
report['combinedSourceOutputs']={str(p.relative_to(out)):rec(p) for p in [core/'libs/s52plib/src/s52plib.cpp',core/'libs/s52plib/src/s52plib.h',core/'gui/src/s57chart.cpp',private/'libs/s52plib/src/s52plib.cpp',private/'libs/s52plib/src/s52plib.h',private/'src/eSENCChart.cpp']}
(out/'source-proof.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS nine core + two private ordered patches; '+str(len(proof))+' locked private blobs; '+str(len(prep.LOCAL))+' copied owned inputs; five scopes; six unchanged painter bodies; '+str(len(report['ixInputs']))+' IX inputs equal verified20ee source')
