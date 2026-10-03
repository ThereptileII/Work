"""Read-only audit of the retained public ENC feature; no chart mutation."""
import hashlib,json,math
from pathlib import Path
from pyogrio import raw
from shapely import from_wkb
chart=Path('/home/standard/Projects/X-nav/build/chart-fixtures/ENC_ROOT/US5SEAFL/US5SEAFL.000')
files={p.name:{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(chart.parent.glob('US5SEAFL.0*'))}
meta,fids,geoms,columns=raw.read(chart,layer='SLCONS',return_fids=True,UPDATES='APPLY')
for i,fid in enumerate(fids):
 values={str(k):v[i].tolist() if hasattr(v[i],'tolist') else v[i] for k,v in zip(meta['fields'],columns)}
 if values['RCID']!=90:continue
 values={k:v for k,v in values.items() if v is not None and not (isinstance(v,float) and math.isnan(v))}
 assert values['LNAM']=='02261EC35F311923' and values['CATSLC']==4 and values['CONDTN']==2 and values['WATLEV']==3
 geom=from_wkb(geoms[i]); coords=list(geom.exterior.coords)
 report={'sourceFiles':files,'updateMode':'APPLY','attributes':values,'geometryWkt':geom.wkt,'geographicBounds':geom.bounds}
 meta,_,_,cols=raw.read(chart,layer='DSID',return_fids=True,UPDATES='APPLY')
 report['updateNumber']=str(dict(zip(meta['fields'],cols))['DSID_UPDN'][0]);assert report['updateNumber']=='1'
 diagnostic=Path('docs/evidence/skager-chart-9632421-linux/pier57/software/SKAGER-Day.json');d=json.loads(diagnostic.read_text());v=d['runtime']['chart'];r=d['runtime']['display']['chart_region'];z=6378137*.9996
 def merc(lat):s=math.sin(math.radians(lat));return .5*math.log((1+s)/(1-s))*z
 pixels=[(r['x']+r['width']/2+(lon-v['longitude'])*math.pi/180*z*v['scale_ppm'],r['y']+r['height']/2-(merc(lat)-merc(v['latitude']))*v['scale_ppm']) for lon,lat in coords]
 report['diagnosticSha256']=hashlib.sha256(diagnostic.read_bytes()).hexdigest();report['sourceCommit']=d['build_commit'];report['projectedScreenshotBounds']=[min(p[0] for p in pixels),min(p[1] for p in pixels),max(p[0] for p in pixels),max(p[1] for p in pixels)];report['projection']='Pinned spherical Mercator z=6378137*0.9996; actual diagnostic center/ppm/region; north-up fixed capture'
 assert files=={p.name:{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(chart.parent.glob('US5SEAFL.0*'))}
 Path('docs/evidence/scrum265-pier57-ruin-hatch/feature.json').write_text(json.dumps(report,indent=2)+'\n');print(report['projectedScreenshotBounds'])
 break
else:raise RuntimeError('Required exact public feature absent')
