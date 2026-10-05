from pathlib import Path
from pyogrio import raw
import pyogrio,json,math,hashlib
from shapely import from_wkb
root=Path('/home/standard/Projects/X-nav-worktrees/scrum275-linux-e6ef384');p=root/'evidence/local/scrum265-27e93e4/iho/fixture/GB4X0000.000';rows=[]
lat0,lon0=-32.3760351,61.0307025;scale=.5826126536;z=6378137*.9996
for layer,geometry in pyogrio.list_layers(p):
 if layer != 'BUISGL':continue
 meta,fids,geoms,fields=raw.read(p,layer=layer,return_fids=True,UPDATES='APPLY')
 if geoms is None:continue
 for i,g in enumerate(geoms):
  if g is None:continue
  shape=from_wkb(g)
  if shape.geom_type!='Point':continue
  x=507+(shape.x-lon0)*math.pi/180*z*scale;y=283-(math.atanh(math.sin(math.radians(shape.y)))-math.atanh(math.sin(math.radians(lat0))))*z*scale
  if not (0 <= x < 1014 and 0 <= y < 566):continue
  row={str(k):v[i].tolist() if hasattr(v[i],'tolist') else v[i] for k,v in zip(meta['fields'],fields)}
  row={k:v for k,v in row.items() if v is not None and not (isinstance(v,float) and not math.isfinite(v)) and v!=[]}
  rows.append({'class':str(layer),'ogrFid':int(fids[i]),'position':[shape.y,shape.x],'attributes':row,'projectedChart':[x,y],'projectedScreen':[x+80,y+68]})
report={'source':str(p),'sourceSha256':hashlib.sha256(p.read_bytes()).hexdigest(),'decoder':{'pyogrio':pyogrio.__version__,'gdal':list(pyogrio.__gdal_version__)},'scope':'Direct original source decode and projection only; no new runtime probe; official IHO test geography','targetScreen':[611,448],'selection':'All source BUISGL points projected inside this viewport; projection is not runtime visibility','sceneCenter':[lat0,lon0],'actualScalePpm':scale,'canvas':[1014,566],'screenOrigin':[80,68],'nearby':rows}
(root/'evidence/local/scrum265-27e93e4/viewport-buildings.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(rows,indent=2))
