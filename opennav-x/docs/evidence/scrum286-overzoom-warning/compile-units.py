from pathlib import Path
import shlex,re,subprocess,os,json,hashlib
r=Path.cwd();donor=r.parent/'scrum275-linux-e6ef384';virtual=r.parent/'scrum259-linux-6dafd29';out=r/'.local/objects';out.mkdir(exist_ok=True)
s=(donor/'build/xnav-linux/build.ninja').read_text();env={**os.environ,'LD_LIBRARY_PATH':'/home/standard/Projects/X-nav/.local/sysroot/usr/lib'};records=[]
for name in ['ChartPresentation.cpp','chcanv.cpp','glChartCanvas.cpp']:
 matches=list(re.finditer(r'^build ([^\n]*'+re.escape(name)+r'\.o):[^\n]*\n(?:  [^\n]*\n)+',s,re.M));assert len(matches)==1,(name,len(matches));block=matches[0].group();args=[]
 for key in ['DEFINES','INCLUDES','FLAGS']:
  m=re.search('^  '+key+' = (.*)$',block,re.M);assert m,key;args+=shlex.split(m[1])
 args=[x.replace(str(virtual/'src'),str(r/'src')).replace(str(virtual),str(donor)) for x in args]
 source=r/'src/integration'/name if name=='ChartPresentation.cpp' else r/'.local/source/gui/src'/name
 cmd=['/usr/bin/c++',*args,'-c',str(source),'-o',str(out/(name+'.o'))]
 p=subprocess.run(cmd,cwd=donor/'build/xnav-linux',env=env,text=True,capture_output=True);(out/(name+'.log')).write_text(p.stdout+p.stderr);records.append({'command':cmd,'sourceSha256':hashlib.sha256(source.read_bytes()).hexdigest(),'exit':p.returncode});(out/'commands.json').write_text(json.dumps(records,indent=2)+'\n');print(name,p.returncode,flush=True)
 if p.returncode:print((p.stdout+p.stderr)[-2500:]);raise SystemExit(p.returncode)
 records[-1]['objectSha256']=hashlib.sha256((out/(name+'.o')).read_bytes()).hexdigest();(out/'commands.json').write_text(json.dumps(records,indent=2)+'\n')
