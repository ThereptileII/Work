from pathlib import Path
import tempfile,subprocess,json,hashlib,re
r=Path.cwd();up=r.parent/'skager-product-fidelity/upstream/OpenCPN';pin=json.loads((r/'upstream.lock.json').read_text())['commit'];paths=['gui/src/chcanv.cpp','gui/src/glChartCanvas.cpp'];report={}
with tempfile.TemporaryDirectory() as temp:
 for path in paths:
  t=Path(temp)/path;t.parent.mkdir(parents=True,exist_ok=True);t.write_bytes(subprocess.check_output(['git','-C',str(up),'show',pin+':'+path]))
 for name in ['xnav','regression-tests','ais-transport','chart-presentation','maintained-curl','download-trust','wxcurl-trust','peer-response-buffer','peer-unavailable']:
  subprocess.run(['git','apply','--unsafe-paths',*['--include='+p for p in paths],str(r/f'patches/opencpn-5.12.4-{name}.patch')],cwd=temp,check=True)
 for path in paths:
  got=(Path(temp)/path).read_bytes();assert got==(r/'.local/source'/path).read_bytes();after=got.decode();before=(r/'.local/before'/path).read_text();start=after.index('#ifdef OPENNAV_X\n  auto *overzoom = ');end=after.index('\n#endif',start)+len('\n#endif');branch=after[start:end];old=branch.split('\n#else\n')[1].split('\n#endif')[0];assert after[:start]+old+after[end:]==before
  report[path]={'sha256':hashlib.sha256(got).hexdigest(),'onlyDelta':'one indicator call, guarded custom paint, same stock pointer fallback; macro-off original'}
report['pin']=pin;report['orderedPatchCount']=9
(r/'.local/source-proof.json').write_text(json.dumps(report,indent=2)+'\n')
print('Both changed canvas sources exactly reproduced through 9 ordered patches; only caller blocks changed')
