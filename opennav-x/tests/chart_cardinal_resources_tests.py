"""Independent category, prototype geometry/color and whole-atlas isolation proof."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from chart_raster_ink import decode,derive
from chart_day_neutral_ink import derive_day


def verify_cardinals(source,output,metadata,check):
    root=Path(__file__).resolve().parents[1]
    guide=json.loads((root/'docs/design/prototype/src/seamarks.json').read_text())
    stock=ET.parse(source/'chartsymbols.xml').getroot()
    current=ET.parse(output/'chartsymbols.xml').getroot()
    expected=[('BOYCAR01','north',['black','yellow'],True,True,102),
              ('BOYCAR02','east',['black','yellow','black'],True,False,102),
              ('BOYCAR03','south',['yellow','black'],False,False,100),
              ('BOYCAR04','west',['yellow','black','yellow'],False,True,100)]
    inks={'DAY_BRIGHT':((83,100,95),(172,141,76),(213,229,229)),
          'DUSK':((195,206,194),(203,177,122),(52,79,89)),
          'NIGHT':((107,119,109),(131,112,76),(14,23,28))}
    # No selector, Paper body, physical topmark, label, priority or category edits.
    for a,b in zip(stock.findall('lookups/lookup'),current.findall('lookups/lookup')):
        if a.get('name') in ('BOYCAR','TOPMAR'):check(ET.tostring(a)==ET.tostring(b))
    check(current.find("lookups/lookup[@id='1262']/instruction").text is None)
    for i,(name,direction,bands,upper,lower,count) in enumerate(expected):
        g=next(x for x in guide if x.get('code')==name)
        check((g['top'],g['colors'])==(direction,bands))
        rows=[x for x in current.findall('lookups/lookup') if 'SY('+name+')' in (x.findtext('instruction') or '')]
        check(len(rows)==1 and rows[0].findtext('table-name')=='Simplified')
        check(rows[0].findtext('attrib-code')=='CATCAM'+str(i+1))
        definitions=current.findall("symbols/symbol[name='"+name+"']")
        check(len(definitions)==1 and definitions[0].get('RCID')==str(1270+i))
        bitmap=definitions[0].find('bitmap')
        check(bitmap.attrib=={'width':'24','height':'28'})
        check(bitmap.find('pivot').attrib=={'x':'12','y':'14'})
        check(bitmap.find('graphics-location').attrib=={'x':str(116+32*i),'y':'1160'})
        for table,(black,yellow,water) in inks.items():
            svg=ET.parse(root/'resources/chart-style/v1/cardinals'/(name+'-'+table+'.svg')).getroot()
            group=list(svg)[0];shape=list(group)[0]
            check(group.attrib=={'transform':'translate(12 14) scale(0.84375)','fill':'none','stroke-width':'1.3','stroke-linecap':'round','stroke-linejoin':'round','shape-rendering':'geometricPrecision'})
            nodes=list(shape);heads=nodes[-3]
            color=lambda rgb:'#'+''.join(f'{c:02x}' for c in rgb)
            check(heads.attrib=={'fill':color(black),'fill-opacity':'.13','stroke':color(black)})
            check([n.get('d') for n in heads]==[
                'M0 -14l-3 4.5h6Z' if upper else 'M-3 -14h6L0 -9.5Z',
                'M0 -7l-3 4.5h6Z' if lower else 'M-3 -7h6L0 -2.5Z'])
            check([n.get('stroke') for n in nodes[1:1+len(bands)]]==[color(black if b=='black' else yellow) for b in bands])
            check(nodes[-1].attrib=={'cy':'9','r':'1.7','fill':color(water)})
    _,day=decode((source/'rastersymbols-day.png').read_bytes())
    for table,file,neutral in [('DAY_BRIGHT','rastersymbols-day.png',None),('DUSK','rastersymbols-dusk.png',(54,54,54)),('NIGHT','rastersymbols-dark.png',(27,27,27))]:
        raw=(source/file).read_bytes()
        if table=='DAY_BRIGHT':raw,_=derive_day((source/'chartsymbols.xml').read_bytes(),raw,metadata['palette'][table]['CHBLK'])
        if neutral:raw,_=derive(day,raw,neutral,metadata['palette'][table]['CHBLK'])
        ca,before=decode(raw);cb,after=decode((output/file).read_bytes())
        check([(k,v) for k,v in ca if k!=b'IDAT']==[(k,v) for k,v in cb if k!=b'IDAT'])
        # Existing anchor/service deltas have independent proofs; remove only their tiles.
        for x,w,h in [(20,20,20),(52,24,24),(84,24,24),(820,24,24)]:
            for y in range(1160,1160+h):
                start=(y*1500+x)*4;after[start:start+w*4]=before[start:start+w*4]
        from chart_seamark_resources_tests import restore_tiles
        restore_tiles(before,after)
        changed=[j for j in range(0,len(before),4) if before[j:j+4]!=after[j:j+4]]
        check(len(changed)==404)
        for i,(name,_,_,_,_,count) in enumerate(expected):
            x=116+32*i
            data=json.loads((root/'resources/chart-style/v1/cardinals'/(name+'-'+table+'-rgba.json')).read_text())
            rgba=bytes.fromhex(''.join(data['rows']))
            actual=bytearray()
            touched=0
            for y in range(28):
                for dx in range(24):
                    offset=((1160+y)*1500+x+dx)*4;p=rgba[(y*24+dx)*4:(y*24+dx+1)*4]
                    if p[3]:
                        check(after[offset:offset+4]==p and before[offset+3]==0);touched+=1
                    else:check(after[offset:offset+4]==before[offset:offset+4])
                    actual.extend(after[offset:offset+4])
            check(touched==count)
            # Recognizable chromatic stem ink remains in every native-size tile.
            for index,target in enumerate(inks[table][:2]):
                other=inks[table][1-index]
                # The 1.6-unit band overlaps the 1.3-unit first-color stem.
                # Test resulting category separation, not a false pure-RGB claim.
                check(any(actual[j+3]>=190 and
                          sum((actual[j+c]-target[c])**2 for c in range(3)) <
                          sum((actual[j+c]-other[c])**2 for c in range(3))
                          for j in range(0,len(actual),4)))
            for y in range(1158,1190):check(not any(before[(y*1500+x-2)*4+3:(y*1500+x+26)*4:4]))
        for j in changed:after[j:j+4]=before[j:j+4]
        check(after==before)  # Includes old tiles, other atlas content and invisible RGB.
