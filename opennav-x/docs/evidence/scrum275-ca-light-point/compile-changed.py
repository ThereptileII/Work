from pathlib import Path
import subprocess,re,shlex,json,os,hashlib
root=Path.cwd();out=root/'.local/objects';out.mkdir(exist_ok=True)
builder=root.parent/'scrum259-linux-6dafd29';ninja=(builder/'build/xnav-linux/build.ninja').read_text();start=ninja.index('build CMakeFiles/opencpn.dir/'+str(builder).lstrip('/')+'/src/integration/ChartPresentation.cpp.o:');block=ninja[start:ninja.index('\n\nbuild ',start)]
flags=[]
for key in ['DEFINES','INCLUDES']:flags+=shlex.split(re.search('^  '+key+' = (.+)$',block,re.M)[1])
core=root/'.local/core';private=root/'.local/private';wx=Path('/home/standard/Projects/X-nav/.local/sysroot/usr');env={**os.environ,'LD_LIBRARY_PATH':str(wx/'lib')};receipts=[]
resources=builder/'build/xnav-linux/opennav-chart-style/v1'
def run(cmd,name):
 r=subprocess.run(cmd,capture_output=True,text=True,env=env);(out/(name+'.log')).write_text(r.stdout+r.stderr);receipts.append({'name':name,'command':cmd,'exit':r.returncode});(out/'commands.json').write_text(json.dumps(receipts,indent=2)+'\n');print(name,r.returncode,flush=True)
 if r.returncode:print((r.stdout+r.stderr)[-2800:]);raise SystemExit(r.returncode)
for name,path in [('core-s52plib',core/'libs/s52plib/src/s52plib.cpp'),('core-s57chart',core/'gui/src/s57chart.cpp')]:
 base=['c++','-std=gnu++17','-O1','-fPIC','-I'+str(root/'src'),'-I'+str(resources),'-I'+str(core/'libs/s52plib/src'),'-I'+str(core/'gui/include'),*flags]
 run(base+['-c',str(path),'-o',str(out/(name+'.o'))],name)
wxflags=shlex.split(subprocess.check_output([str(wx/'bin/wx-config'),'--prefix='+str(wx),'--cxxflags'],text=True))
config=(private/'config.h.in').read_text().replace('#cmakedefine OPENGL_FOUND','#define OPENGL_FOUND 1')
for k,v in {'API_VERSION':17,'PROJECT_VERSION_MAJOR':2,'PROJECT_VERSION_MINOR':0,'PROJECT_VERSION_PATCH':29,'PROJECT_VERSION_TWEAK':0}.items():config=config.replace('@'+k+'@',str(v))
(out/'config.h').write_text(config)
api=root.parent/'scrum259-ocharts-port/.local/plugin-source/opencpn-libs/api-17'
incs=[root/'src',resources,private/'src',api,out,wx/'include']+list((private/'libs').glob('*'))+list((private/'libs').glob('*/src'))+list((private/'libs').glob('*/include'))
base=['c++','-std=c++17','-O1','-fPIC','-DSKAGER_OCHARTS_ADAPTER','-DTIXML_USE_STL','-D__OCPN_USE_GLEW__','-DocpnUSE_GL=1','-DocpnUSE_GLSL=1',*wxflags,*['-I'+str(p) for p in incs if p.is_dir()]]
for name,path in [('private-s52plib',private/'libs/s52plib/src/s52plib.cpp'),('private-eSENCChart',private/'src/eSENCChart.cpp')]:run(base+['-c',str(path),'-o',str(out/(name+'.o'))],name)
run([x for x in base if x!='-DSKAGER_OCHARTS_ADAPTER']+['-c',str(private/'libs/s52plib/src/s52plib.cpp'),'-o',str(out/'private-disabled.o')],'private-disabled')
(out/'objects.json').write_text(json.dumps({p.name:{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in out.glob('*.o')},indent=2)+'\n')
