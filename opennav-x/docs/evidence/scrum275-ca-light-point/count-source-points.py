from pathlib import Path
import hashlib,json
import pyogrio
from pyogrio import raw
paths=[Path('/home/standard/Projects/X-nav/.local/noaa-chart/ENC_ROOT/US5SEAFL/US5SEAFL.000'),Path('/home/standard/Projects/X-nav-worktrees/scrum264-final-light-captures/.local/colored-light-capture/fixture/GB4X0000.000')]
report={'decoder':{'pyogrio':pyogrio.__version__,'gdal':list(pyogrio.__gdal_version__)},'limit':32768,'scope':'Source point/multipoint feature counts, an upper bound before lookup selection, not runtime razRules instrumentation. Official S64 is test geography.','cells':[]}
for path in paths:
 def receipts():return {p.name:{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(path.parent.glob(path.stem+'.*')) if p.suffix[1:].isdigit()}
 before=receipts();counts={}
 for layer,geometry in pyogrio.list_layers(path):
  meta,fids,geoms,fields=raw.read(path,layer=layer,return_fids=True,read_geometry=False,UPDATES='APPLY')
  if 'PRIM' not in meta['fields']:continue
  prim=fields[list(meta['fields']).index('PRIM')]; count=sum(int(v)==1 for v in prim if v is not None)
  if count:counts[str(layer)]={'geometry':str(geometry),'features':count}
 assert before==receipts();report['cells'].append({'cell':path.stem,'files':before,'pointFeatureCount':sum(v['features'] for v in counts.values()),'layers':counts})
 print(path.stem,report['cells'][-1]['pointFeatureCount'])
Path('docs/evidence/scrum275-ca-light-point/source-point-counts.json').write_text(json.dumps(report,indent=2)+'\n')
