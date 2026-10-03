#!/usr/bin/env python3
"""Offline SCRUM-264 classified white/orange pillar derivative; no prototype edits."""
import argparse,hashlib,json,re,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/special-buoy'
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
ASSETS.mkdir(exist_ok=True);a.evidence.mkdir(parents=True,exist_ok=True)
lock=json.loads((ROOT/'resources/chart-style/v1/source-lock.json').read_text())['files']['chartsymbols.xml']
raw=(a.source/'chartsymbols.xml').read_bytes();assert hashlib.sha256(raw).hexdigest()==lock['sha256']
tree=ET.fromstring(raw);node=tree.find("symbols/symbol[name='BOYPIL81']")
assert node.get('RCID')=='43' and node.findtext('color-ref')=='ADEPMDBCHBLKCCHWHTDCHCOR'
lookup=tree.find("lookups/lookup[@id='1947']");assert lookup.attrib=={'id':'1947','RCID':'30221','name':'BOYSPP'}
assert [n.text for n in lookup.findall('attrib-code')]==['BOYSHP4','COLOUR1,11','COLPAT1']
assert lookup.findtext('instruction').startswith('SY(BOYPIL81)')
art=(ROOT/'docs/design/prototype/src/chart-marker-art.js').read_text()
assert 'M0-7V8' in art and 'M-2.5 6H2.5' in art and '<circle cy="9" r="1.7" fill="var(--water)"/>' in art
assert 'M0 ${-3+i*11/c.length}v${11/c.length}' in art
colors={n.get('name'):{c.get('name'):tuple(int(c.get(k)) for k in ('r','g','b')) for c in n.findall('color')} for n in tree.findall('color-tables/color-table')}
tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes'];proof={'issue':'SCRUM-264','sourceXmlSha256':lock['sha256'],'orangeSource':'BOYPIL81/43 CHCOR, exact Paper1947 attributes. Day unchanged; fixed Night HLS-lightness lift meets >=3:1 against styled depth fills without a global palette edit. Dusk retains source ink in the unused alias tile and is ineligible at RenderSY; its tested lift lost orange recognition.','sourceNodeSha256':hashlib.sha256(ET.tostring(node)).hexdigest(),'tile':[660,1160,24,28],'pivot':[12,14],'prototypeScale':27/32,'derivation':'Prototype stem/base circle only; no head/topmark. Neutral silhouette, white/orange horizontal bands. No physical stripe measurements asserted.','nightLegibilityException':{'reason':'Prototype has no orange role; stock Night CHCOR fails visible band recognition. Dusk lift is rejected as cream; stock Rule remains active there.','hlsLightnessStep':32982,'hlsDenominator':65535,'preserveSourceHueAndSaturation':True,'precedingRgb':[207,112,49],'requiredSolidContrast':3,'waterRoles':['DEPDW','DEPMD','DEPMS','DEPVS','DEPIT']},'themes':{},'sources':{}}
for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
 def role(name):
  x=tokens[theme][name];rgb=[int(x[i:i+2],16) for i in (1,3,5)]
  return tuple(round(c*.78) for c in rgb) if theme=='night' else tuple(rgb)
 rgb={'neutral':role('--mark-black'),'white':role('--mark-white'),'water':role('--water'),'orange':{'NIGHT':(208,113,49)}.get(table,colors[table]['CHCOR'])}
 def color(k):return '#'+''.join(f'{v:02x}' for v in rgb[k])
 svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="24" height="28" viewBox="0 0 24 28"><g transform="translate(12 14)"><g transform="scale(0.84375)" fill="none" stroke="{color('neutral')}" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round"><path d="M0-7V8"/><path d="M0 -3v5.5" stroke="{color('white')}" stroke-width="1.6"/><path d="M0 2.5v5.5" stroke="{color('orange')}" stroke-width="1.6"/><path d="M-2.5 6H2.5"/><circle cy="9" r="1.7" fill="{color('water')}"/></g></g></svg>\n'''
 path=ASSETS/('XNSPPW01-'+table+'.svg');path.write_text(svg);png=a.evidence/('XNSPPW01-'+table+'.png');subprocess.run(['rsvg-convert',str(path),'-o',str(png)],check=True)
 rgba=Image.open(png).convert('RGBA').tobytes();(ASSETS/('XNSPPW01-'+table+'-rgba.json')).write_text(json.dumps({'width':24,'height':28,'rows':[rgba[y*96:(y+1)*96].hex() for y in range(28)]},indent=2)+'\n')
 proof['themes'][table]={'rgbaSha256':hashlib.sha256(rgba).hexdigest(),'changedPixels':sum(bool(x) for x in rgba[3::4]),'colors':rgb,'sourceOrange':colors[table]['CHCOR'],'orangeLiftException':table=='NIGHT','eligibleForAlias':table!='DUSK'}
for path in ['docs/design/prototype/src/chart-marker-art.js','docs/design/prototype/src/chart-symbols.css','docs/design/prototype-tokens.json','tools/derive-special-buoy-art.py']+[str(p.relative_to(ROOT)) for p in sorted(ASSETS.iterdir()) if p.name!='provenance.json']:
 proof['sources'][path]=hashlib.sha256((ROOT/path).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
(ASSETS/'provenance.json').write_text(json.dumps(proof,indent=2)+'\n')
