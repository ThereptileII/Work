"""Independent yellow/X source geometry, narrow tile restoration and XML guards."""
import copy,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
from chart_raster_ink import decode
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/yellow-buoy'
NAMES={'XNSPPY01':(60013,692),'XNSPPT01':(60015,756)}
TABLES=(('DAY_BRIGHT','day','rastersymbols-day.png'),('DUSK','dusk','rastersymbols-dusk.png'),('NIGHT','night','rastersymbols-dark.png'))
def pixels(name,table):
    data=json.loads((ASSETS/(name+'-'+table+'-rgba.json')).read_text())
    assert data['width']==24 and data['height']==28
    return bytes.fromhex(''.join(data['rows']))
def restore_yellow(before,after):
    for name,(_,x) in NAMES.items():
        for table,_,_ in TABLES:
            rgba=pixels(name,table)
            if all(before[((1160+y)*1500+x+dx)*4+3]==0 and
                   after[((1160+y)*1500+x+dx)*4:((1160+y)*1500+x+dx)*4+4]==
                   (rgba[(y*24+dx)*4:(y*24+dx)*4+4] if rgba[(y*24+dx)*4+3] else before[((1160+y)*1500+x+dx)*4:((1160+y)*1500+x+dx)*4+4])
                   for y in range(28) for dx in range(24)):
                for y in range(28):
                    for dx in range(24):
                        if rgba[(y*24+dx)*4+3]:
                            i=((1160+y)*1500+x+dx)*4;after[i:i+4]=before[i:i+4]
                break
        else:raise AssertionError('Yellow special buoy tile differs outside approved pixels')
def verify_yellow_buoy(source,output,check):
    original=ET.parse(source/'chartsymbols.xml').getroot();current=ET.parse(output/'chartsymbols.xml').getroot()
    stock=original.find("symbols/symbol[name='BOYSPP11']")
    check(ET.tostring(stock)==ET.tostring(current.find("symbols/symbol[name='BOYSPP11']")))
    for a,b in zip(original.findall('lookups/lookup'),current.findall('lookups/lookup')):
        if a.get('name') in ('BOYSPP','TOPMAR'):check(ET.tostring(a)==ET.tostring(b))
    for name,(rcid,x) in NAMES.items():
        nodes=current.findall("symbols/symbol[name='"+name+"']");check(len(nodes)==1);n=nodes[0]
        check(n.get('RCID')==str(rcid) and n.find('bitmap').attrib=={'width':'24','height':'28'})
        check(n.find('bitmap/pivot').attrib=={'x':'12','y':'14'})
        check(n.find('bitmap/graphics-location').attrib=={'x':str(x),'y':'1160'})
        restored=copy.deepcopy(n);restored.set('RCID',stock.get('RCID'));restored.find('name').text='BOYSPP11'
        restored.find('bitmap').attrib=stock.find('bitmap').attrib.copy()
        for tag in ('pivot','graphics-location'):restored.find('bitmap/'+tag).attrib=stock.find('bitmap/'+tag).attrib.copy()
        check(ET.tostring(restored)==ET.tostring(stock))
        for table,theme,filename in TABLES:
            svg=ET.parse(ASSETS/(name+'-'+table+'.svg')).getroot()
            for el in svg.iter():el.tag=el.tag.rsplit('}',1)[-1]
            marker=svg.find('g/g');check(marker.get('transform')=='scale(0.84375)')
            shape=marker.find('g');tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes'][theme]
            def ink(role):
                value=tokens[role];rgb=[int(value[i:i+2],16) for i in (1,3,5)]
                if theme=='night':rgb=[round(c*.78) for c in rgb]
                return '#'+''.join(f'{c:02x}' for c in rgb)
            check(shape.get('stroke')==ink('--mark-yellow'))
            if name=='XNSPPY01':
                check([n.tag for n in shape]==['path','path','path','circle'])
                check([n.get('d') for n in shape[:3]]==['M0-7V8','M0 -3v11','M-2.5 6H2.5'])
                check(shape[1].get('stroke-width')=='1.6' and shape[-1].get('fill')==ink('--water'))
            else:
                check(len(shape)==1 and shape[0].tag=='g' and shape[0].get('fill-opacity')=='.13')
                check(len(shape[0])==1 and shape[0][0].attrib=={'d':'M-3-12 3-6M3-12-3-6','fill':'none'})
            _,atlas=decode((output/filename).read_bytes());rgba=pixels(name,table)
            for y in range(28):
                for dx in range(24):
                    off=(y*24+dx)*4
                    if rgba[off+3]:check(atlas[((1160+y)*1500+x+dx)*4:((1160+y)*1500+x+dx)*4+4]==rgba[off:off+4])
