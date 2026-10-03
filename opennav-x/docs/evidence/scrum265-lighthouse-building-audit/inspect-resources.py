from pathlib import Path
from PIL import Image,ImageChops
from collections import Counter
import hashlib,json,re,xml.etree.ElementTree as E
root=Path(__file__).resolve().parents[3];out=Path(__file__).resolve().parent
source=root/'build/integration-source/data/s57data';installed=root/'build/xnav-install/share/opencpn/opennav/chart-style/v1';tree=E.parse(source/'chartsymbols.xml').getroot();node=next(n for n in tree.find('symbols') if n.findtext('name')=='BUISGL01');b=node.find('bitmap');xy=b.find('graphics-location');x,y=int(xy.get('x')),int(xy.get('y'));w,h=int(b.get('width')),int(b.get('height'));box=(x,y,x+w,y+h)
lookup=[n for n in tree.find('lookups') if n.get('name')=='BUISGL' and n.findtext('table-name')=='Simplified'];uses=[E.tostring(n,encoding='unicode') for n in tree.find('lookups') if 'BUISGL01' in (n.findtext('instruction') or '')]
owners=[]
for group in ('symbols','patterns','line-styles'):
 for n in tree.find(group):
  bmp=n.find('bitmap')
  if bmp is None or bmp.find('graphics-location') is None:continue
  pos=bmp.find('graphics-location');a,c=int(pos.get('x')),int(pos.get('y'));d,e=a+int(bmp.get('width')),c+int(bmp.get('height'))
  if max(x,a)<min(x+w,d) and max(y,c)<min(y+h,e):owners.append({'group':group,'name':n.findtext('name'),'RCID':n.get('RCID'),'rectangle':[a,c,d,e]})
colors={t.get('name'):{c.get('name'):[int(c.get(k)) for k in ('r','g','b')] for c in t.findall('color') if c.get('name') in ('CHBRN','LANDF','CHBLK')} for t in tree.find('color-tables') if t.get('name') in ('DAY_BRIGHT','DUSK','NIGHT')}
sheets={}
for theme,suffix in [('Day','day'),('Dusk','dusk'),('Night','dark')]:
 a=Image.open(source/('rastersymbols-'+suffix+'.png')).convert('RGBA').crop(box);b=Image.open(installed/('rastersymbols-'+suffix+'.png')).convert('RGBA').crop(box);assert a.tobytes()==b.tobytes()
 sheets[theme]={'unchangedSourceTile':True,'rgbaSha256':hashlib.sha256(a.tobytes()).hexdigest(),'rgbaColors':len(Counter(a.getdata())),'alphaValues':sorted(set(a.getchannel('A').getdata()))}
 a.save(out/('BUISGL01-pinned-'+theme+'.png'))
shotpath=root/'docs/evidence/scrum268-5bb-linux-canvas/iho/output/capture-5bb7e05-s64-light-fog-opengl/s64-light-fog-SKAGER-Day.png';shot=Image.open(shotpath).convert('RGB');background=shot.getpixel((620,447));tile=Image.open(out/'BUISGL01-pinned-Day.png').convert('RGBA');expected=Image.alpha_composite(Image.new('RGBA',(w,h),background+(255,)),tile).convert('RGB');actual=shot.crop((607,443,616,452));diff=ImageChops.difference(expected,actual)
# GL UNORM blending can differ from Pillow rounding by one; retain exact comparison rather than assert equality.
match={'screenRectangle':[607,443,616,452],'background':background,'sourceTileRectangle':box,'pillowCompositeExact':diff.getbbox() is None,'maximumChannelDifference':max(v for px in diff.getdata() for v in px),'changedPixels':sum(px!=(0,0,0) for px in diff.getdata()),'opaqueSourcePixelsExact':all(a[:3]==b for a,b in zip(tile.getdata(),actual.getdata()) if a[3]==255),'opaqueSourcePixels':sum(a[3]==255 for a in tile.getdata())}
actual.save(out/'actual-Day-square-exact9.png');shot.crop((601,437,622,458)).save(out/'actual-Day-square-context.png')
prototype=root/'docs/design/prototype/index.html';html=prototype.read_text();catalog=json.JSONDecoder().raw_decode(html[html.index('{"id":"point:BUISGL01"'):])[0]
fragments={}
for selector in ('#app{--mark-red:', '#app[data-theme=dusk]{--mark-red:', '#app[data-theme=night]{--mark-red:', '.chart-marker-art.marker-service{'):
 start=html.index(selector);fragments[selector]=html[start:html.index('}',start)+1]
result={'scope':'Read-only resource/attribute/projection/pixel evidence; actual runtime lookup not newly probed','sourceCommit':'5bb7e0584029c72ce21f9230e246dad5771dc321','sourceXmlSha256':hashlib.sha256((source/'chartsymbols.xml').read_bytes()).hexdigest(),'prototypeSha256':hashlib.sha256(prototype.read_bytes()).hexdigest(),'captureSha256':hashlib.sha256(shotpath.read_bytes()).hexdigest(),'simplifiedLookups':[{**n.attrib,'attributes':[a.text for a in n.findall('attrib-code')],'instruction':n.findtext('instruction'),'priority':n.findtext('disp-prio'),'category':n.findtext('display-cat')} for n in lookup],'allDirectBUISGL01Lookups':uses,'symbol':E.tostring(node,encoding='unicode'),'conspicuousSymbol':E.tostring(next(n for n in tree.find('symbols') if n.findtext('name')=='BUISGL11'),encoding='unicode'),'bitmapOwners':owners,'sourceColors':colors,'tiles':sheets,'retainedCaptureMatch':match,'prototypeCatalogEntry':catalog,'prototypeExplicitBUISGLArtwork':False,'prototypeFragments':fragments}
(out/'resource-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'owners':owners,'match':match,'tiles':sheets},indent=2))
