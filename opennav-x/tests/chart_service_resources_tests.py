"""Independent SCRUM-254 effective-symbol, exact-art and isolation checks."""
import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, derive
from chart_day_neutral_ink import derive_day


def verify_services(source, output, metadata, check):
    root = Path(__file__).resolve().parents[1]
    art = (root/'docs/design/prototype/src/chart-marker-art.js').read_text()
    fixtures = [('PILBOP02',1,2,52,148,'a5939fe16a9ceab9bdd44e5b0182c76407b6c3b0800d5429ce39f510e7f9bc86'),
                ('RTPBCN02',2259,1,84,156,'4e8e9899f67b94972331aac087f52726aaf53ca4644e6da4d316104e91472ba6')]
    stock = ET.parse(source/'chartsymbols.xml').getroot()
    current = ET.parse(output/'chartsymbols.xml').getroot()
    for name,rcid,count,x,pixels,digest in fixtures:
        paths = re.search("'point:"+name+"':'([^']+)'",art)[1]
        svg = ET.parse(root/'resources/chart-style/v1/services'/(name+'.svg')).getroot()
        group = list(svg)[0]
        expected = ET.fromstring('<g xmlns="http://www.w3.org/2000/svg">'+paths+'</g>')
        check([ET.tostring(n) for n in group] == [ET.tostring(n) for n in expected])
        check(group.attrib == {'transform':'translate(12 12) scale(0.84375)',
            'fill':'none','stroke':'#ffffff','stroke-width':'1.3','stroke-linecap':'round',
            'stroke-linejoin':'round','shape-rendering':'geometricPrecision'})
        nodes = current.findall("symbols/symbol[name='"+name+"']")
        check(len(nodes)==count and nodes[-1].get('RCID')==str(rcid))
        check(nodes[-1].findtext('prefer-bitmap') not in ('no','false'))
        check(nodes[-1].find('bitmap').attrib=={'width':'24','height':'24'})
        check(nodes[-1].find('bitmap/pivot').attrib=={'x':'12','y':'12'})
        check(nodes[-1].find('bitmap/graphics-location').attrib=={'x':str(x),'y':'1160'})
        for original,styled in zip(stock.findall("symbols/symbol[name='"+name+"']")[:-1],nodes[:-1]):
            check(ET.tostring(original)==ET.tostring(styled))
        for bitmap in current.findall('.//bitmap'):
            pos=bitmap.find('graphics-location')
            if pos is None or bitmap is nodes[-1].find('bitmap'):continue
            bx,by,bw,bh=map(int,(pos.get('x'),pos.get('y'),bitmap.get('width'),bitmap.get('height')))
            check(not (x-2<bx+bw and x+26>bx and 1158<by+bh and 1186>by))
    check('function chartSymbolGraphic(s,size=27)' in art and 'size/32' in art)
    css=(root/'docs/design/prototype/src/chart-symbols.css').read_text()
    check(re.findall(r'--mark-service:(#[a-f0-9]+)',css)==['#7c858a','#a8bbb7','#7e948a'])
    check('#app[data-theme=night] .chart-canvas{filter:brightness(.78)}' in (root/'docs/design/prototype/src/style.css').read_text())
    _,day=decode((source/'rastersymbols-day.png').read_bytes())
    for name,table,neutral,color in [('rastersymbols-day.png','DAY_BRIGHT',None,(124,133,138)),
                                  ('rastersymbols-dusk.png','DUSK',(54,54,54),(168,187,183)),
                                  ('rastersymbols-dark.png','NIGHT',(27,27,27),(98,115,108))]:
        content=(source/name).read_bytes()
        if table=='DAY_BRIGHT':content,_=derive_day((source/'chartsymbols.xml').read_bytes(),content,metadata['palette'][table]['CHBLK'])
        if neutral:content,_=derive(day,content,neutral,metadata['palette'][table]['CHBLK'])
        cb,before=decode(content);ca,after=decode((output/name).read_bytes())
        check([(k,v) for k,v in cb if k!=b'IDAT']==[(k,v) for k,v in ca if k!=b'IDAT'])
        # ACHARE51 has its own independent all-pixel proof. Remove exactly that
        # approved rectangle for this service-only delta comparison.
        for y in range(1160,1180):
            i=(y*1500+20)*4;after[i:i+80]=before[i:i+80]
        # SCRUM-256 cardinal tiles have a separate full-pixel/semantic proof.
        for y in range(1160,1188):
            for x in (116,148,180,212):
                start=(y*1500+x)*4
                after[start:start+96]=before[start:start+96]
        from chart_seamark_resources_tests import restore_tiles
        restore_tiles(before,after)
        changed=[i for i in range(0,len(before),4) if before[i:i+4]!=after[i:i+4]]
        check(len(changed)==304)
        for symbol,rcid,count,x,pixels,digest in fixtures:
            changes=[i for i in changed if x<=i//4%1500<x+24 and 1160<=i//4//1500<1184]
            check(len(changes)==pixels)
            check(all(before[i+3]==0 and after[i+3]>0 and after[i:i+3]==bytes(color) for i in changes))
            alpha=bytes(after[((1160+y)*1500+x+dx)*4+3] for y in range(24) for dx in range(24))
            check(hashlib.sha256(alpha).hexdigest()==digest)
            for y in range(1158,1186):check(not any(before[(y*1500+x-2)*4+3:(y*1500+x+26)*4:4]))
        for i in changed:after[i:i+4]=before[i:i+4]
        check(after==before)  # Includes all old tiles, neighbors and invisible RGB.
    for scale in (.5,1,1.25,1.5,2,3):
        for angle in (0,.37,math.pi/2,math.pi):
            c,s=math.cos(angle),math.sin(angle)
            for px,py in ((0,0),(0,-11),(9,0),(-8,-8),(8,8)):
                dx=(12+px*27/32)*scale-int(12*scale)
                dy=(12+py*27/32)*scale-int(12*scale)
                eps=12*scale-int(12*scale)
                check(abs(c*dx+s*dy-(c*px+s*py)*27/32*scale-eps*(c+s))<1e-12)
                check(abs(-s*dx+c*dy-(-s*px+c*py)*27/32*scale-eps*(c-s))<1e-12)
