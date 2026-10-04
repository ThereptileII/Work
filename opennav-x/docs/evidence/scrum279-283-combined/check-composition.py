from pathlib import Path
import hashlib, json, importlib.util, subprocess, sys, time
import xml.etree.ElementTree as ET
root=Path.cwd();sys.path.insert(0,str(root/'tools'))
from chart_raster_ink import decode
spec=importlib.util.spec_from_file_location('generator',root/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
source=Path('/home/standard/Projects/X-nav/upstream/OpenCPN/data/s57data')
before=Path('/home/standard/Projects/X-nav-worktrees/scrum280-hazard-glyphs/.local/before')
out=root/'.local/combined-symbols/generated';proof=root/'.local/combined-symbols'
started=time.monotonic();head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
identity=json.loads((before/'manifest.json').read_text())
assert hashlib.sha256((before/'manifest.json').read_bytes()).hexdigest()=='15a6bb831392088990737d688bc7716efa42f1d950f051c7a0dd909ee3065e2d'
for name,item in identity['files'].items():assert hashlib.sha256((before/name).read_bytes()).hexdigest()==item['sha256'],name
now=g.generate(source,out) # includes unchanged strict whole-source identity/semantic guard
print('Combined resource generation and strict semantic guard passed',flush=True)
a,b=ET.parse(before/'chartsymbols.xml').getroot(),ET.parse(out/'chartsymbols.xml').getroot()
for name in ['SMCFAC02','UWTROC03','UWTROC04','WRECKS05']:
 old=a.find("symbols/symbol[name='"+name+"']/bitmap");new=b.find("symbols/symbol[name='"+name+"']/bitmap")
 assert new.attrib=={'width':'24','height':'24'} and new.find('pivot').attrib=={'x':'12','y':'12'}
 new.attrib=dict(old.attrib)
 for tag in ['pivot','graphics-location']:new.find(tag).attrib=dict(old.find(tag).attrib)
lookup=b.find("lookups/lookup[@id='65']")
assert lookup.get('RCID')=='32101' and lookup.findtext('instruction')=='AP(XNFISH03);LS(DASH,1,CHGRD)'
lookup.find('instruction').text=a.find("lookups/lookup[@id='65']/instruction").text
patterns=b.findall("patterns/pattern[name='XNFISH03']");assert len(patterns)==1;b.find('patterns').remove(patterns[0])
old=a.find("line-styles/line-style[name='CBLSUB06']");new=b.find("line-styles/line-style[name='CBLSUB06']")
assert new.find('vector').attrib=={'width':'635','height':'168'}
new.find('HPGL').text=old.findtext('HPGL');new.find('vector').attrib=dict(old.find('vector').attrib)
for tag in ['pivot','origin']:new.find('vector/'+tag).attrib=dict(old.find('vector/'+tag).attrib)
for tree in [a,b]:
 for el in tree.iter():
  if el.text is not None and not el.text.strip():el.text=None
  if el.tail is not None and not el.tail.strip():el.tail=None
assert ET.tostring(a)==ET.tostring(b),'Combined XML changed beyond the reviewed five glyphs/pattern and cable metadata'
# Exact independent slot/coverage expectations from reviewed individual proofs.
slots=[('SMCFAC02',820,152,'services'),('UWTROC03',852,56,'hazards'),('UWTROC04',884,88,'hazards'),('WRECKS05',916,161,'hazards'),('XNFISH03',948,48,'fishing-pattern')]
records=[]
for file in ['rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png']:
 ca,pa=decode((before/file).read_bytes());cb,pb=decode((out/file).read_bytes());count=0
 assert [(k,v) for k,v in ca if k!=b'IDAT']==[(k,v) for k,v in cb if k!=b'IDAT']
 for name,x,expected,folder in slots:
  mask=json.loads((root/'resources/chart-style/v1'/folder/(name+'-alpha.json')).read_text())
  alpha=bytes.fromhex(''.join(mask['rows']));assert len(alpha)==576
  changed=0
  for y in range(24):
   for dx in range(24):
    i=((1160+y)*1500+x+dx)*4;j=y*24+dx
    assert pa[i+3]==0 and pb[i+3]==alpha[j],(file,name,j)
    changed += pa[i:i+4]!=pb[i:i+4]
   i=((1160+y)*1500+x)*4;pb[i:i+96]=pa[i:i+96]
  assert changed==expected,(file,name,changed);count+=changed
 assert pb==pa,'Pixels outside approved combined tiles changed'
 records.append({'file':file,'changedPixels':count,'allOtherPixelsAndPngMetadataIdentical':True})
 print(file, count,'approved pixels, all other RGBA identical',flush=True)
assert (before/'S52RAZDS.RLE').read_bytes()==(out/'S52RAZDS.RLE').read_bytes()
report={'headAtStart':head,'elapsedSeconds':time.monotonic()-started,'originalPrototypeSha256':hashlib.sha256((root/'docs/design/prototype/index.html').read_bytes()).hexdigest(),'priorManifestSha256':hashlib.sha256((before/'manifest.json').read_bytes()).hexdigest(),'combinedManifestSha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'atlases':records,'xmlInverse':'exact after only four symbol bitmaps, one owned AP alias/selector, and cable physical/HPGL metadata restored','scope':'combined resource composition only; prior individual negative/loader/method checks retained, no app/native/boat acceptance'}
(proof/'composition.json').write_text(json.dumps(report,indent=2)+'\n');print('PASS combined resource composition',flush=True)
