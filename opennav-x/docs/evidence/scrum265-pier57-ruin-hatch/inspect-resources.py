"""Read-only pinned rule/pattern and actual installed tile comparison."""
from pathlib import Path
import csv,hashlib,json,xml.etree.ElementTree as E
from PIL import Image
root=Path('/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29')
stock=root/'build/xnav-install/share/opencpn/s57data';styled=root/'build/xnav-install/share/opencpn/opennav/chart-style/v1'
a=E.parse(stock/'chartsymbols.xml').getroot();b=E.parse(styled/'chartsymbols.xml').getroot()
lookup=next(n for n in a.findall('lookups/lookup') if n.get('id')=='181') if a.find('lookups') is not None else next(n for n in a.findall('lookup') if n.get('id')=='181')
pattern=next(n for n in a.findall('patterns/pattern') if n.findtext('name')=='CROSSX01')
new_pattern=next(n for n in b.findall('patterns/pattern') if n.findtext('name')=='CROSSX01')
assert E.tostring(pattern)==E.tostring(new_pattern)
lookup2=next(n for n in b.findall('lookups/lookup') if n.get('id')=='181') if b.find('lookups') is not None else next(n for n in b.findall('lookup') if n.get('id')=='181')
assert E.tostring(lookup)==E.tostring(lookup2)
assert lookup.findtext('instruction')=='CS(SLCONS03)' and pattern.findtext('color-ref')=='ACHBRN'
report={'lookup':E.tostring(lookup,encoding='unicode'),'pattern':E.tostring(pattern,encoding='unicode'),'stockAndStyledLookupIdentical':True,'stockAndStyledPatternIdentical':True,'tiles':{}}
for name in ['rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png']:
 old=Image.open(stock/name).convert('RGBA').crop((400,1040,416,1056)).tobytes();new=Image.open(styled/name).convert('RGBA').crop((400,1040,416,1056)).tobytes();assert old==new
 report['tiles'][name]={'bounds':[400,1040,416,1056],'decodedRgbaSha256':hashlib.sha256(old).hexdigest(),'stockAndStyledIdentical':True}
report['attributeMeanings']={}
for row in csv.DictReader((stock/'s57expectedinput.csv').open()):
 if (row['Code'],row['ID']) in [('60','4'),('81','2'),('187','3')]:report['attributeMeanings'][row['Code']+':'+row['ID']]=row['Meaning']
inputs=[root/'build/integration-source/libs/s52plib/src/s52cnsy.cpp',root/'build/integration-source/gui/src/viewport.cpp',root/'build/integration-source/model/src/georef.cpp',stock/'chartsymbols.xml',stock/'s57attributes.csv',stock/'s57expectedinput.csv',styled/'chartsymbols.xml',styled/'manifest.json']
report['inputSha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
Path('docs/evidence/scrum265-pier57-ruin-hatch/resources.json').write_text(json.dumps(report,indent=2)+'\n');print('Exact lookup and all three16x16 CROSSX01 tiles remain byte-identical; meanings:',report['attributeMeanings'])
