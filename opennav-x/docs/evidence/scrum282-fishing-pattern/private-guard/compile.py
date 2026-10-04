from pathlib import Path
import shlex,subprocess,json,re,os,hashlib
root=Path.cwd();out=root/'.local/fishing/followup';p=out/'source';sys=Path('/home/standard/Projects/X-nav/.local/sysroot/usr')
recipe=(root/'cmake/ocharts-adapter/Targets.cmake').read_text()
defs=re.search(r'add_compile_definitions\((UNICODE.*?)\)',recipe,re.S).group(1).split()+['DECL_EXP=']
assert 'SKAGER_OCHARTS_ADAPTER' in defs and 'OPENNAV_X' not in defs
flags=shlex.split(subprocess.check_output([str(sys/'bin/wx-config'),'--prefix='+str(sys),'--cxxflags'],text=True))
incs=[p/'opencpn-libs/api-17',out,p/'src',p/'libs/gdal/src',root/'src',root/'.local/fishing/current',sys/'include',p/'libs/s52plib/src',p/'libs/geoprim/src',p/'libs/pugixml',p/'include',p/'libs/api-17']
config=(p/'config.h.in').read_text().replace('#cmakedefine OPENGL_FOUND','#define OPENGL_FOUND 1')
for k,v in {'API_VERSION':'1.17','PROJECT_VERSION_MAJOR':'2','PROJECT_VERSION_MINOR':'0','PROJECT_VERSION_PATCH':'29','PROJECT_VERSION_TWEAK':'0'}.items(): config=config.replace('@'+k+'@',v)
(out/'config.h').write_text(config)
command=['g++','-std=c++17','-fPIC',*['-D'+d for d in defs],*flags,*['-I'+str(i) for i in incs],'-c',str(p/'libs/s52plib/src/s52plib.cpp'),'-o',str(out/'private-s52plib.o')]
r=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(out/'private-s52plib.log').write_text(r.stdout)
(out/'private-s52plib.json').write_text(json.dumps({'command':command,'exitCode':r.returncode,'definitionsFromActualTargets':defs,'sourceSha256':hashlib.sha256((p/'libs/s52plib/src/s52plib.cpp').read_bytes()).hexdigest(),'scope':'Full private translation unit, actual adapter definitions and target include closure; Linux wx platform and system GL headers. No Win32 compile/link or DLL execution.'},indent=2)+'\n')
print(r.returncode,r.stdout[-3000:]);raise SystemExit(r.returncode)
