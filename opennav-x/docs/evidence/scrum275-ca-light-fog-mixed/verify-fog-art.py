"""Read-only source/resource proof; does not regenerate or modify any artwork."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,xml.etree.ElementTree as ET
from PIL import Image
ROOT=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser();p.add_argument('--upstream',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--core-source',type=Path,required=True);p.add_argument('--private-source',type=Path,required=True);a=p.parse_args()
def record(path):
 data=path.read_bytes();return {'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
lock=json.loads((ROOT/'resources/chart-style/v1/source-lock.json').read_text())
for name,expected in lock['files'].items():assert record(a.upstream/name)==expected,name
manifest=json.loads((a.generated/'manifest.json').read_text())
for name,expected in manifest['files'].items():assert record(a.generated/name)==expected,name
stock=ET.parse(a.upstream/'chartsymbols.xml').getroot();generated=ET.parse(a.generated/'chartsymbols.xml').getroot()
lookup=stock.findall(".//lookup[@RCID='31164']");assert len(lookup)==1;lookup=lookup[0]
expected={'type':'Point','disp-prio':'Area Symbol','radar-prio':'On Top','table-name':'Simplified','instruction':'SY(FOGSIG01)','display-cat':'Standard','comment':'27080'}
assert lookup.attrib=={'id':'1112','RCID':'31164','name':'FOGSIG'}
assert {node.tag:node.text for node in lookup}==expected
assert ET.tostring(lookup)==ET.tostring(generated.find(".//lookup[@RCID='31164']"))
def symbol(root,name):
 result=[node for node in root.findall('.//symbol') if node.findtext('name')==name];assert result;return result[-1]
def geometry(node):
 bitmap=node.find('bitmap');return tuple(int(bitmap.attrib[k]) for k in ('width','height')),tuple(int(bitmap.find('pivot').attrib[k]) for k in ('x','y')),tuple(int(bitmap.find('graphics-location').attrib[k]) for k in ('x','y'))
def crop(path,node):
 (w,h),(px,py),(x,y)=geometry(node);return Image.open(path).convert('RGBA').crop((x,y,x+w,y+h))
def ink(image,pivot):return {(x-pivot[0],y-pivot[1]) for y in range(image.height) for x in range(image.width) if image.getpixel((x,y))[3]}
fog=symbol(stock,'FOGSIG01');assert fog.attrib['RCID']=='1338';assert geometry(fog)==((12,13),(15,-3),(791,121));assert ET.tostring(fog)==ET.tostring(symbol(generated,'FOGSIG01'))
light=symbol(generated,'XNLIT013');assert geometry(light)[:2]==((24,28),(12,14))
report={'sourceLockSha256':record(ROOT/'resources/chart-style/v1/source-lock.json')['sha256'],'generatedManifest':record(a.generated/'manifest.json'),'lookup':expected,'fogGeometry':geometry(fog),'resources':{},'prototypeSources':{}}
# Bind the bitmap guard to both real pinned loaders, not a fixture Rule.
loader_path=ROOT/'tools/verify-anchor-loader.py'
spec=importlib.util.spec_from_file_location('anchor_loader',loader_path)
loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)
core_path=a.core_source/'libs/s52plib/src/chartsymbols.cpp'
private_path=a.private_source/'libs/s52plib/src/chartsymbols.cpp'
core_text=core_path.read_text();private_bytes=private_path.read_bytes()
private_lock=json.loads((ROOT/'tools/ocharts-adapter-source.lock.json').read_text())['source']['files']['libs/s52plib/src/chartsymbols.cpp']
private_blob=hashlib.sha1(b'blob '+str(len(private_bytes)).encode()+b'\0'+private_bytes).hexdigest()
assert len(private_bytes)==private_lock['bytes'] and private_blob==private_lock['gitBlob']
shared={}
for name in ('ProcessSymbols','BuildSymbol'):
 body=loader.method(core_text,name)
 assert body==loader.method(private_bytes.decode(),name),name
 shared[name]={'sha256':hashlib.sha256(body.encode()).hexdigest(),'bytes':len(body.encode())}
process=loader.method(core_text,'ProcessSymbols');build=loader.method(core_text,'BuildSymbol')
assert 'symbol.preferBitmap = true;' in process and 'symbol.hasBitmap = true;' in process
assert 'symbol.hasVector && !(symbol.preferBitmap && symbol.hasBitmap)' in build
assert "symb->definition.SYDF = 'R';\n    symbolSize = symbol.bitmapSize;" in build
assert fog.findtext('definition')=='V' and fog.find('vector') is not None
assert fog.find('prefer-bitmap') is None and fog.find('bitmap') is not None
report['loaderSelection']={'core':record(core_path),'private':record(private_path),'privateGitBlob':private_blob,'privateSourceLock':record(ROOT/'tools/ocharts-adapter-source.lock.json'),'extractor':record(loader_path),'byteIdenticalMethods':shared,'fogHasVector':True,'fogHasBitmap':True,'fogExplicitPreferBitmap':None,'defaultPreferBitmap':True,'selectedDefinition':'R','selectedGeometry':'bitmap','scope':'Source proof through actual identical core/private ProcessSymbols and BuildSymbol bodies. No native loader or canvas execution in this increment.'}
point_masks={}
for theme,filename in [('DAY_BRIGHT','rastersymbols-day.png'),('DUSK','rastersymbols-dusk.png'),('NIGHT','rastersymbols-dark.png')]:
 before=crop(a.upstream/filename,fog);after=crop(a.generated/filename,symbol(generated,'FOGSIG01'));assert before.tobytes()==after.tobytes()
 rgba=ROOT/'resources/chart-style/v1/seamarks'/('XNLIT013-'+theme+'-rgba.json');data=json.loads(rgba.read_text());committed=b''.join(bytes.fromhex(row) for row in data['rows']);point=crop(a.generated/filename,light)
 # The committed painter replaces alpha>0 pixels only; transparent pixels
 # retain exact stock atlas RGBA (white RGB with zero alpha in this region).
 expected=bytearray(crop(a.upstream/filename,light).tobytes())
 for i in range(0,len(committed),4):
  if committed[i+3]:expected[i:i+4]=committed[i:i+4]
 assert point.tobytes()==bytes(expected)
 fogInk=ink(before,(15,-3));pointInk=ink(point,(12,14));point_masks[theme]=pointInk;overlap=fogInk&pointInk
 # Native one-to-one source pixels only. Renderer filtering/scaling may differ.
 assert not overlap
 report['resources'][theme]={'stockAtlas':record(a.upstream/filename),'generatedAtlas':record(a.generated/filename),'fogTileSha256':hashlib.sha256(before.tobytes()).hexdigest(),'aliasCommitted':record(rgba),'aliasCommittedRgbaSha256':hashlib.sha256(committed).hexdigest(),'aliasTileSha256':hashlib.sha256(point.tobytes()).hexdigest(),'fogInkPixels':len(fogInk),'pointInkPixels':len(pointInk),'sourcePixelOverlap':len(overlap)}
for name in ['src/chart-marker-art.js','src/chart-symbols.css','src/chart-symbols.json','src/chart-symbols.js','src/light-sectors.js']:
 report['prototypeSources'][name]=record(ROOT/'docs/design/prototype'/name)
report['structuralConflicts']={}
for name in ['TOWERS01','PILPNT02']:
 node=symbol(stock,name);image=crop(a.upstream/'rastersymbols-day.png',node);overlap=ink(image,geometry(node)[1])&point_masks['DAY_BRIGHT'];assert overlap
 report['structuralConflicts'][name]={'geometry':geometry(node),'overlapDay1xPixels':len(overlap)}
report['limits']='Only native 1x source-alpha overlap, not scaled/DPI/rotated/clipped software or GL canvas proof. Current fog lookup, node and all three tiles remain byte-identical. Tower/pile conflicts remain refused.'
a.output.write_text(json.dumps(report,indent=2)+'\n');print({theme:data['sourcePixelOverlap'] for theme,data in report['resources'].items()},report['structuralConflicts'])
