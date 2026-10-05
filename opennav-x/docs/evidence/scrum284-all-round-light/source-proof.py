from pathlib import Path
import tempfile,os,subprocess,json,hashlib,importlib.util,shutil,re
root=Path.cwd();source=root/'.local/core';lock=json.loads((root/'upstream.lock.json').read_text());names=['xnav','regression-tests','ais-transport','chart-presentation','maintained-curl','download-trust','wxcurl-trust','peer-response-buffer','peer-unavailable'];report={}
# Own metadata/objects: never write through the copied donor worktree pointer.
gitdir=root/'.local/core-git';subprocess.run(['git','init','--bare','--quiet',str(gitdir)],check=True)
objects=subprocess.check_output(['git','-C',str(root.parent/'skager-product-fidelity/upstream/OpenCPN'),'rev-parse','--path-format=absolute','--git-path','objects'],text=True).strip();(gitdir/'objects/info/alternates').write_text(objects+'\n');(source/'.git').write_text('gitdir: '+str(gitdir)+'\n')
with tempfile.TemporaryDirectory() as d:
 env={**os.environ,'GIT_INDEX_FILE':d+'/index'};git=['git','-c','core.bare=false','--git-dir='+str(gitdir),'--work-tree='+str(source)]
 subprocess.run(git+['read-tree',lock['commit']],env=env,check=True)
 for name in names:subprocess.run(git+['apply','--cached','-'],env=env,input=(root/f'patches/opencpn-5.12.4-{name}.patch').read_bytes(),check=True)
 subprocess.run(git+['diff','--quiet'],env=env,check=True);report['core9PatchTree']=subprocess.check_output(git+['write-tree'],env=env,text=True).strip()
spec=importlib.util.spec_from_file_location('prep',root/'tools/prepare-ocharts-adapter.py');prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)
cache=root.parent/'scrum259-adapter-preparation/.local/pinned-source';dest=root/'.local/private-reproduced';shutil.copytree(cache,dest);prep.apply_patches(dest,root)
paths=set()
for patch in prep.PATCHES:paths.update(re.findall(r'^\+\+\+ b/(.+)$',(root/patch).read_text(),re.M))
for path in paths:assert (dest/path).read_bytes()==(root/'.local/private'/path).read_bytes(),path
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report['private3PatchFiles']={p:sha(dest/p) for p in sorted(paths)}
local=root/'.local/private-local';
for path in prep.LOCAL:
 target=local/path;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/path,target);assert target.read_bytes()==(root/path).read_bytes()
assert 'src/integration/ChartCaAllRound.h' in prep.LOCAL and 'src/integration/ChartCaAllRound.h' in prep.INPUTS
report['privateOwnedHeaderSha256']=sha(local/'src/integration/ChartCaAllRound.h')
# Only two renderer methods and their include changed in prepared S52 source.
spec=importlib.util.spec_from_file_location('extract',root/'tools/verify-anchor-loader.py');ex=importlib.util.module_from_spec(spec);spec.loader.exec_module(ex)
report['unchangedMethods']={}
for kind in ('core','private'):
 before=(root/f'.local/{kind}-before.cpp').read_text();after=(root/f'.local/{kind}/libs/s52plib/src/s52plib.cpp').read_text()
 for sig in ('void s52plib::draw_lc_poly(','render_canvas_parms *s52plib::CreatePatternBufferSpec(','int s52plib::RenderCARC('):
  b=ex.function(before,sig);a=ex.function(after,sig);assert a==b;report['unchangedMethods'][kind+':'+sig]=hashlib.sha256(a.encode()).hexdigest()
 for sig in ('int s52plib::RenderCARC_GLSL(','int s52plib::RenderCARC_VBO('):after=after.replace(ex.function(after,sig),ex.function(before,sig))
 after=after.replace('#include "integration/ChartCaAllRound.h"\n','');assert after==before
 # Conditional outputs, private283 callback/header and all other files retained.
 for path in ('libs/s52plib/src/s52cnsy.cpp','libs/s52plib/src/s52s57.h'):
  donor=root.parent/'scrum281-combined-review'/('build/integration-source' if kind=='core' else '.local/private')/path
  assert (root/f'.local/{kind}'/path).read_bytes()==donor.read_bytes()
report['scope']='Ordered source/owned-header closure; no application or private DLL build'
(root/'.local/source-proof.json').write_text(json.dumps(report,indent=2)+'\n');print('9 core + 3 private patches compose exactly; finite/cable/fishing dispatch and conditional sources unchanged')
