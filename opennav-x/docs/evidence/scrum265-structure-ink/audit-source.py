import hashlib,json,collections
from pathlib import Path
import pyogrio,shapely,numpy
src=Path('/home/standard/Projects/X-nav/build/chart-fixtures/ENC_ROOT/US5SEAFL/US5SEAFL.000')
bbox=[-122.39,47.58,-122.33,47.62]; result={'source':src.name,'sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'bbox':bbox,'reader':'pyogrio '+pyogrio.__version__,'features':{}}
result['inputFiles']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(src.parent.glob('US5SEAFL.*')) if p.suffix[1:].isdigit()}
layers=dict(pyogrio.list_layers(src))
for name in ['BUISGL','FLODOC','MORFAC','PONTON','CRANES','LNDMRK','SILTNK']:
 if name not in layers:result['features'][name]=[];continue
 columns=['RCID','PRIM','CATMOR','FUNCTN','CONVIS','WATLEV','CONDTN','STATUS','SCAMIN','OBJNAM']
 meta,fids,geometry,fields=pyogrio.raw.read(src,layer=name,columns=columns,bbox=tuple(bbox))
 rows=[]
 for i,g in enumerate(geometry):
  attrs={}
  for k,values in zip(meta['fields'],fields):
   v=values[i]
   if isinstance(v,numpy.ndarray):v=v.tolist()
   if isinstance(v,numpy.generic):v=v.item()
   if v is not None and not (isinstance(v,float) and numpy.isnan(v)):attrs[k]=v
  attrs['geometry']=shapely.from_wkb(g).geom_type;rows.append(attrs)
 result['features'][name]=rows
print(json.dumps({k: {'count':len(v),'geometry':dict(collections.Counter(x['geometry'] for x in v)),'CATMOR':dict(collections.Counter(str(x.get('CATMOR')) for x in v))}for k,v in result['features'].items()},indent=2))
Path('docs/evidence/scrum265-structure-ink/actual-enc.json').write_text(json.dumps(result,indent=2)+'\n')
