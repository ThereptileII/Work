from pathlib import Path
import subprocess,tempfile,re,hashlib,json,importlib.util,shutil
root=Path.cwd();out=root/'.local/patch-proof';out.mkdir(exist_ok=True)
core=out/'core';core.mkdir(exist_ok=True)
p=subprocess.Popen(['git','archive','HEAD'],cwd='/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/upstream/OpenCPN',stdout=subprocess.PIPE);subprocess.run(['tar','-x','-C',str(core)],stdin=p.stdout,check=True);assert p.wait()==0
original=(core/'libs/s52plib/src/s52plib.cpp').read_text()
patches=re.findall(r"root / '(patches/[^']+)'",(root/'tools/prepare-integration.py').read_text())
with tempfile.TemporaryDirectory() as tmp:
 subprocess.run(['git','init','--bare','-q',tmp],check=True)
 cmd=['git','-c','core.bare=false','--git-dir='+tmp,'--work-tree='+str(core),'apply']
 for name in patches:subprocess.run(cmd+['--check',str(root/name)],cwd=core,check=True);subprocess.run(cmd+[str(root/name)],cwd=core,check=True)
for name in ['libs/s52plib/src/s52plib.h','libs/s52plib/src/s52plib.cpp','gui/src/s57chart.cpp']:assert(core/name).read_bytes()==(root/'.local/core'/name).read_bytes()
spec=importlib.util.spec_from_file_location('fixture',root/'tools/verify-anchor-loader.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
report={'base':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'corePatches':{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in patches},'unchangedPainterMethods':{},'callers':{}}
for kind,before,after in [('core',original,(core/'libs/s52plib/src/s52plib.cpp').read_text()),('private',(root.parent/'scrum259-adapter-preparation/.local/pinned-source/libs/s52plib/src/s52plib.cpp').read_text(),(root/'.local/private/libs/s52plib/src/s52plib.cpp').read_text())]:
 report['unchangedPainterMethods'][kind]={}
 for signature in ['int s52plib::RenderCARC_GLSL(','int s52plib::RenderCARC_VBO(','bool s52plib::RenderRasterSymbol(']:
  old=m.function(before,signature);new=m.function(after,signature);assert old==new,(kind,signature);report['unchangedPainterMethods'][kind][signature]=hashlib.sha256(new.encode()).hexdigest()
for kind,filename,callers in [('core','gui/src/s57chart.cpp',{'bool s57chart::DoRenderOnGL(':1,'bool s57chart::DCRenderLPB(':1}),('private','src/eSENCChart.cpp',{'bool eSENCChart::DoRender2RectOnGL(':2,'bool eSENCChart::DCRenderLPB(':1})]:
 text=(root/'.local'/kind/filename).read_text();report['callers'][kind]={}
 for signature,count in callers.items():
  body=m.function(text,signature);assert body.count('CaLightPointScope<s52plib>')==count;report['callers'][kind][signature]={'scopes':count,'sha256':hashlib.sha256(body.encode()).hexdigest()}
# Private exact preparer applies both append-only patches against locked blobs.
private=out/'private';shutil.copytree(root.parent/'scrum259-adapter-preparation/.local/pinned-source',private)
spec=importlib.util.spec_from_file_location('prep',root/'tools/prepare-ocharts-adapter.py');prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep);prep.apply_patches(private,root)
for name in ['libs/s52plib/src/s52plib.h','libs/s52plib/src/s52plib.cpp','src/eSENCChart.cpp']:assert(private/name).read_bytes()==(root/'.local/private'/name).read_bytes()
report['privatePatches']={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in prep.PATCHES}
(root/'docs/evidence/scrum275-ca-light-point/source-proof.json').write_text(json.dumps(report,indent=2)+'\n')
print('nine core + two private patches; unchanged CA painters/raster painter; exactly five active scopes')
