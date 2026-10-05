"""Read retained public cells without modifying any chart; apply adjacent updates."""
import argparse,hashlib,json,math
from pathlib import Path
import pyogrio
from pyogrio import raw
from shapely import from_wkb

parser=argparse.ArgumentParser()
parser.add_argument('--noaa',type=Path,required=True)
parser.add_argument('--s64',type=Path,required=True)
parser.add_argument('--mapping',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
classes=['BOYLAT','BOYISD','BOYSAW','BOYSPP','BOYCAR','BCNCAR','LIGHTS','TOPMAR']
def clean(x):
 if hasattr(x,'tolist'):x=x.tolist()
 if isinstance(x,list):return [clean(y) for y in x]
 if isinstance(x,float) and not math.isfinite(x):return None
 return x
def rows(path,layer,updates):
 meta,fids,geoms,fields=raw.read(path,layer=layer,return_fids=True,UPDATES=updates)
 result=[]
 for i,fid in enumerate(fids):
  row={str(k):clean(v[i]) for k,v in zip(meta['fields'],fields)}
  row={k:v for k,v in row.items() if v is not None and v!=[] and v!=''}
  row['ogrFid']=int(fid)
  if geoms is not None and geoms[i] is not None:
   point=from_wkb(geoms[i]);row['longitude']=point.x;row['latitude']=point.y
  result.append(row)
 return result
report={'decoder':{'pyogrio':pyogrio.__version__,'gdal':list(pyogrio.__gdal_version__)},
 'mappingSha256':hashlib.sha256(a.mapping.read_bytes()).hexdigest(),'updates':'APPLY','cells':[]}
inputs=[('NOAA-real-world',a.noaa/'ENC_ROOT/US5SEAFL/US5SEAFL.000'),
 ('NOAA-real-world',a.noaa/'adjacent/ENC_ROOT/US5SEAFK/US5SEAFK.000'),
 ('IHO-S64-official-test-data-not-real-world',a.s64/'2.1.1 Power Up/ENC_ROOT/GB4X0000.000')]
for kind,p in inputs:
 files=[x for x in sorted(p.parent.glob(p.stem+'.*')) if x.suffix[1:].isdigit()]
 before={x.name:{'bytes':x.stat().st_size,'sha256':hashlib.sha256(x.read_bytes()).hexdigest()} for x in files}
 layers={x[0] for x in pyogrio.list_layers(p)}
 cell={'name':p.stem,'kind':kind,'sourceFiles':before,'dsidWithoutUpdates':rows(p,'DSID','IGNORE'),
       'dsidWithUpdates':rows(p,'DSID','APPLY'),'features':{},'counts':{}}
 for cls in classes:
  cell['features'][cls]=rows(p,cls,'APPLY') if cls in layers else []
  cell['counts'][cls]=len(cell['features'][cls])
 after={x.name:{'bytes':x.stat().st_size,'sha256':hashlib.sha256(x.read_bytes()).hexdigest()} for x in files}
 if before!=after:raise RuntimeError('Read-only source identity changed')
 report['cells'].append(cell)
a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
for cell in report['cells']:
 print(cell['name'],cell['counts'],'UPDN',cell['dsidWithoutUpdates'][0]['DSID_UPDN'],'->',cell['dsidWithUpdates'][0]['DSID_UPDN'])
