from pathlib import Path
import subprocess,shlex,json,os,hashlib
root=Path.cwd();out=root/'.local/objects';out.mkdir(exist_ok=True);private=root/'.local/source';wx=Path('/home/standard/Projects/X-nav/.local/sysroot/usr');env={**os.environ,'LD_LIBRARY_PATH':str(wx/'lib')};receipts=[]
resources=root.parent/'scrum259-linux-6dafd29/build/xnav-linux/opennav-chart-style/v1'
wxflags=shlex.split(subprocess.check_output([str(wx/'bin/wx-config'),'--prefix='+str(wx),'--cxxflags'],text=True))
config=(private/'config.h.in').read_text().replace('#cmakedefine OPENGL_FOUND','#define OPENGL_FOUND 1')
for k,v in {'API_VERSION':17,'PROJECT_VERSION_MAJOR':2,'PROJECT_VERSION_MINOR':0,'PROJECT_VERSION_PATCH':29,'PROJECT_VERSION_TWEAK':0}.items():config=config.replace('@'+k+'@',str(v))
(out/'config.h').write_text(config)
api=root.parent/'scrum259-ocharts-port/.local/plugin-source/opencpn-libs/api-17'
incs=[root/'src',resources,private/'src',api,out,wx/'include']+list((private/'libs').glob('*'))+list((private/'libs').glob('*/src'))+list((private/'libs').glob('*/include'))
base=['c++','-std=c++17','-O3','-Werror=dangling-pointer','-fPIC','-DSKAGER_OCHARTS_ADAPTER','-DTIXML_USE_STL','-D__OCPN_USE_GLEW__','-DocpnUSE_GL=1','-DocpnUSE_GLSL=1',*wxflags,*['-I'+str(p) for p in incs if p.is_dir()]]
for name,path in [('private-s52cnsy',private/'libs/s52plib/src/s52cnsy.cpp'),('private-eSENCChart',private/'src/eSENCChart.cpp')]:
 cmd=base+['-c',str(path),'-o',str(out/(name+'.o'))];r=subprocess.run(cmd,capture_output=True,text=True,env=env);(out/(name+'.log')).write_text(r.stdout+r.stderr);receipts.append({'name':name,'command':cmd,'exit':r.returncode});(out/'commands.json').write_text(json.dumps(receipts,indent=2)+'\n');print(name,r.returncode,flush=True)
 if r.returncode:print((r.stdout+r.stderr)[-3500:]);raise SystemExit(r.returncode)
(out/'objects.json').write_text(json.dumps({p.name:{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in out.glob('*.o')},indent=2)+'\n')
