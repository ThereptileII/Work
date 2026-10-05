import json,xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter
import pyogrio,shapely
src=Path('/home/standard/Projects/X-nav/build/chart-fixtures/ENC_ROOT/US5SEAFL/US5SEAFL.000')
xml=Path('/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29/upstream/OpenCPN/data/s57data/chartsymbols.xml')
tree=ET.parse(xml); lookup={}
for node in tree.findall('lookups/lookup'):
 if 'LANDF' in node.findtext('instruction'):
  lookup.setdefault(node.get('name'),[]).append({'id':node.get('id'),'RCID':node.get('RCID'),'type':node.findtext('type'),'table':node.findtext('table-name'),'selectors':[n.text for n in node.findall('attrib-code')],'instruction':node.findtext('instruction')})
report=[]
for name,_ in pyogrio.list_layers(src):
 if name not in lookup:continue
 meta,_,geometry,fields=pyogrio.raw.read(src,layer=name,columns=['RCID','PRIM','CATRIV','CONVIS','FUNCTN'],bbox=(-122.39,47.58,-122.33,47.62))
 if not len(geometry):continue
 types=Counter(shapely.from_wkb(g).geom_type for g in geometry)
 report.append({'class':name,'observedGeometryCounts':dict(types),'possiblePinnedLANDFRules':lookup[name]})
print(json.dumps(report,indent=2))
Path('docs/evidence/scrum265-structure-ink/remaining-landf-audit.json').write_text(json.dumps(report,indent=2)+'\n')
