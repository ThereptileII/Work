#!/usr/bin/env python3
"""Source-locked two-pen transfer weights, preserving baked filtering and alpha.

This is not a hue classifier. The complete pinned node proves exactly two HPGL
pens; only its exact9x9 Day tile determines81 rational coordinate weights. Small
negative/above-one weights are retained filter ringing. The other theme tiles
share the exact alpha geometry, but their baked RGB is not the XML palette.
"""
from pathlib import Path
import argparse,json,hashlib,sys,xml.etree.ElementTree as E
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'tools'))
parser=argparse.ArgumentParser(description='Author the pinned9x9 generic-building two-ink recipe; no chart/application execution.')
parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
from chart_raster_ink import decode
from chart_seamark_art import canonical
source=args.source
lock=json.loads((root/'resources/chart-style/v1/source-lock.json').read_text())
for name in ('chartsymbols.xml','rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png'):
 data=(source/name).read_bytes();data=data.replace(b'\r\n',b'\n') if name.endswith('.xml') else data
 assert hashlib.sha256(data).hexdigest()==lock['files'][name]['sha256']
tree=E.parse(source/'chartsymbols.xml').getroot();node=tree.find("symbols/symbol[name='BUISGL01']");lup=tree.find("lookups/lookup[@id='1091']");tiles={}
for table,suffix in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','dark')]:
 _,p=decode((source/('rastersymbols-'+suffix+'.png')).read_bytes());raw=b''.join(p[((78+y)*1500+459)*4:((78+y)*1500+468)*4] for y in range(9));tiles[table]=raw
assert len({v[3::4] for v in tiles.values()})==1
outline=(139,102,31);fill=(177,145,57);delta=tuple(f-o for f,o in zip(fill,outline));den=sum(d*d for d in delta);raw=tiles['DAY_BRIGHT'];weights=[sum((raw[i+c]-outline[c])*delta[c] for c in range(3)) for i in range(0,len(raw),4)]
reconstruct=bytes((outline[c]*den+n*delta[c]+den//2)//den for n in weights for c in range(3));source_rgb=bytes(raw[i+c] for i in range(0,len(raw),4) for c in range(3));errors=[abs(a-b) for a,b in zip(reconstruct,source_rgb)];assert max(errors)==1
recipe={'schema':1,'sourceSymbol':'BUISGL01','sourceRcid':1307,'sourceNodeCanonicalSha256':hashlib.sha256(canonical(node)).hexdigest(),'lookupCanonicalSha256':hashlib.sha256(canonical(lup)).hexdigest(),'sourceTile':[459,78,9,9],'pivot':[4,4],'sourceTileRgbaSha256':{k:hashlib.sha256(v).hexdigest() for k,v in tiles.items()},'sourceXmlSha256':hashlib.sha256((source/'chartsymbols.xml').read_bytes().replace(b'\r\n',b'\n')).hexdigest(),'prototypeSha256':hashlib.sha256((root/'docs/design/prototype/index.html').read_bytes()).hexdigest(),'model':'Per-coordinate signed rational transfer on the exact known Day LANDF-to-CHBRN two-ink line; no hue selection or threshold. Retains pinned filtered overshoot and alpha. Other palettes reuse the same shape weights, not their differently baked RGB.','sourceOutlineRgb':outline,'sourceFillRgb':fill,'denominator':den,'fillNumerators':[weights[y*9:y*9+9] for y in range(9)],'alphaRows':[raw[3::4][y*9:y*9+9].hex() for y in range(9)],'sourceDayReconstruction':{'maximumRoundedChannelError':max(errors),'channelsDifferentByOne':sum(bool(e) for e in errors),'minimumNumerator':min(weights),'maximumNumerator':max(weights)},'alias':'XNBLDG01','aliasRcid':60016,'tile':[788,1160,9,9],'roles':{'fill':'XNBLF','outline':'XNBLO'},'tokens':{'fill':'--mark-service','outline':'--mark-black'},'nightBrightness':.78}
p=args.output;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(recipe,indent=2)+'\n');print(hashlib.sha256(p.read_bytes()).hexdigest());print(recipe['sourceDayReconstruction'])
