from pathlib import Path
from pyogrio import raw
import pyogrio,json,math,hashlib
from shapely import from_wkb
p=Path('/home/standard/Projects/X-nav-worktrees/scrum264-final-light-captures/.local/colored-light-capture/fixture/GB4X0000.000')
rows=[]
for layer,geometry in pyogrio.list_layers(p):
 meta,fids,geoms,fields=raw.read(p,layer=layer,return_fids=True,UPDATES='APPLY')
 if geoms is None:continue
 for i,g in enumerate(geoms):
  if g is None:continue
  shape=from_wkb(g)
  if shape.geom_type!='Point':continue
  row={str(k):v[i].tolist() if hasattr(v[i],'tolist') else v[i] for k,v in zip(meta['fields'],fields)}
  row={k:v for k,v in row.items() if v is not None and not (isinstance(v,float) and not math.isfinite(v)) and v!=[]}
  rows.append({'class':str(layer),'RCID':int(row.get('RCID',0)),'position':[shape.y,shape.x],'attributes':{k:row.get(k) for k in ['COLOUR','VALNMR','SECTR1','SECTR2','ORIENT','CATLIT','LITVIS','STATUS','QUAPOS','CATLMK','FUNCTN'] if row.get(k) is not None}})
result=[]
for obj in rows:
 if obj['class']=='LIGHTS' and obj['RCID'] in (32,992,993,994):
  colocated=[other for other in rows if other['position']==obj['position']];result.append({'target':obj,'coLocatedPoints':colocated})
candidates=[]
for obj in rows:
 a=obj['attributes']
 if obj['class']!='LIGHTS' or a.get('COLOUR') not in (['1'],['3'],['4'],['6']):continue
 if any(k in a for k in ('ORIENT','CATLIT','LITVIS','STATUS','QUAPOS')):continue
 if not ((a.get('VALNMR',9)>=10 and 'SECTR1' not in a and 'SECTR2' not in a) or ('SECTR1' in a and 'SECTR2' in a)):continue
 group=[other for other in rows if other['position']==obj['position']]
 if len(group)==1:candidates.append(obj)
report={'sourceSha256':hashlib.sha256(p.read_bytes()).hexdigest(),'geometry':'Official IHO presentation test geography, not operational ENC','cases':result,'standaloneSourceCandidates':candidates[:4],'limits':'Decoder inventory only; no native lookup/visibility/canvas claim. NaN decoder nulls and empty lists omitted as absent fields.'}
print([(r['RCID'],r['position']) for r in candidates[:4]])
Path('docs/evidence/scrum275-ca-light-point/source-cases.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
