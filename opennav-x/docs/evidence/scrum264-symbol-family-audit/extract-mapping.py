"""Read-only exact prototype/source mapping; no generation or portrayal edits."""
from pathlib import Path
import argparse,collections,hashlib,json,re,xml.etree.ElementTree as E
p=argparse.ArgumentParser();p.add_argument('--stock',type=Path,required=True);a=p.parse_args();root=Path(__file__).resolve().parents[3];proto=root/'docs/design/prototype';out=Path(__file__).resolve().parent
original=json.loads((root/'docs/design/prototype-original.json').read_text());manifest={r['path']:r for r in original['files']}
files=['SYMBOLS.md','symbol-library.mjs','src/chart-marker-art.js','src/chart-symbols.js','src/chart-symbols.json','src/chart-symbols.css','src/seamarks.json','src/symbol-catalogue.json','vendor/opencpn/chartsymbols.xml','vendor/opencpn/rastersymbols-day.png','vendor/opencpn/rastersymbols-dusk.png','vendor/opencpn/rastersymbols-dark.png','vendor/opencpn/SOURCE.json','vendor/opencpn/COPYING','index.html']
identities={}
for name in files:
 raw=(proto/name).read_bytes();h=hashlib.sha256(raw).hexdigest();assert h==manifest[name]['sha256'];identities[name]={'bytes':len(raw),'sha256':h}
stock=E.parse(a.stock).getroot();vendor=E.parse(proto/'vendor/opencpn/chartsymbols.xml').getroot();catalog=json.loads((proto/'src/symbol-catalogue.json').read_text());guide=json.loads((proto/'src/seamarks.json').read_text());byid={r['id']:r for r in catalog['items']}
names=sorted({g[k] for g in guide for k in ('code','codeA','codeB') if k in g}|{'BCNGEN01','LIGHTS13'})
classes={'BOYLAT','BOYCAR','BOYISD','BOYSAW','BOYSPP','BCNLAT','BCNCAR','BCNISD','BCNSAW','BCNSPP','LIGHTS','LITFLT','LITVES','TOPMAR','LNDMRK','_bcngn','_slgto'}
def record(x):return {'id':x.get('id'),'RCID':x.get('RCID'),'class':x.get('name'),'type':x.findtext('type'),'table':x.findtext('table-name'),'attributes':[v.text for v in x.findall('attrib-code')],'instruction':x.findtext('instruction'),'displayPriority':x.findtext('disp-prio'),'radarPriority':x.findtext('radar-prio'),'displayCategory':x.findtext('display-cat')}
lookups=[record(x) for x in stock.findall('lookups/lookup') if x.get('name').upper() in classes or x.get('name') in classes]
symbols=[]
for name in names:
 xs=stock.findall("symbols/symbol[name='"+name+"']");vs=vendor.findall("symbols/symbol[name='"+name+"']");x=xs[-1];assert E.tostring(x)==E.tostring(vs[-1]);c=byid['point:'+name];b=x.find('bitmap');loc=b.find('graphics-location');pivot=b.find('pivot');geometry={'x':int(loc.get('x')),'y':int(loc.get('y')),'width':int(b.get('width')),'height':int(b.get('height')),'pivot':{k:int(pivot.get(k)) for k in ('x','y')}};assert c['bitmap']==geometry
 symbols.append({'name':name,'effectiveRCID':x.get('RCID'),'sourceDefinitions':len(xs),'effectiveNodeSha256':hashlib.sha256(E.tostring(x)).hexdigest(),'vendorEffectiveNodeEqualsPinned':True,'description':x.findtext('description'),'colorRef':x.findtext('color-ref'),'bitmap':geometry,'allDirectLookupUsers':[record(l) for l in stock.findall('lookups/lookup') if re.search(r'SY\('+name+r'[,)]',l.findtext('instruction') or '')]})
summary={}
for name in sorted(classes):
 rows=[r for r in lookups if r['class']==name];tables={}
 for table in sorted({r['table'] for r in rows}):
  matches=[r for r in rows if r['table']==table];syms=sorted({s for r in matches for s in re.findall(r'SY\(([^,)]+)',r['instruction'] or '')});tables[table]={'lookups':len(matches),'symbols':syms,'explicitModernPackageSymbols':[s for s in syms if s in names]}
 summary[name]=tables
result={'sourceCommit':'a3e84771652c920479517f0d16a1dd6133c440d2','applicationUpstream':'37fd0cddb7334fe489e9f18aa163977a9c5c84f7','stockXmlSha256':hashlib.sha256(a.stock.read_bytes()).hexdigest(),'prototypeSource':catalog['source'],'immutableFilesVerified':identities,'designedFamilySymbols':symbols,'familySummary':summary,'allFamilyLookups':lookups,'scope':'Mapping/provenance only. No source resource or original prototype was changed.'}
(out/'mapping.json').write_text(json.dumps(result,indent=2)+'\n');print(len(files),'immutable inputs verified;',len(symbols),'exact designed family nodes match pinned effective nodes;',len(lookups),'related lookup rows recorded')
