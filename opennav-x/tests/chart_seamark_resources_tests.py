"""Independent approved selectors, unchanged semantic branches and atlas proof."""
import copy
import hashlib
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET
from chart_raster_ink import decode,derive
from chart_day_neutral_ink import derive_day

# Independent oracle, not imported from the production mapping/helper.
ALIASES=('XNLAT013','XNLAT014','XNLAT023','XNLAT024','XNCAN072','XNCAN073','XNCON066','XNCON067','XNLIT011','XNLIT012','XNLIT013')
ORIGINALS=('BOYLAT13','BOYLAT14','BOYLAT23','BOYLAT24','BOYCAN72','BOYCAN73','BOYCON66','BOYCON67','LIGHTS11','LIGHTS12','LIGHTS13')
NAMES=ALIASES[:8]+('BOYISD12','BOYSAW12')+ALIASES[8:]
SELECTED=('XNCON066','XNCON067','XNCAN072','XNCAN073','XNCAN072','XNCAN073','XNCON066','XNCON067',
          'XNLAT014','XNLAT013','XNLAT024','XNLAT023','XNLAT024','XNLAT023','XNLAT014','XNLAT013')

def restore_tiles(before,after):
    from chart_construction_hatch_tests import restore_hatch
    restore_hatch(before,after)
    # Exactly thirteen transparent 24x28 slots. No broad atlas row exclusions.
    for x in (244,276,308,340,372,404,436,468,500,532,564,596,628):
        for y in range(1160,1188):
            i=(y*1500+x)*4;after[i:i+96]=before[i:i+96]

def verify_seamarks(source,output,metadata,check):
    root=Path(__file__).resolve().parents[1]
    stock=ET.parse(source/'chartsymbols.xml').getroot();current=ET.parse(output/'chartsymbols.xml').getroot()
    expected_attrs=[['BOYSHP1','COLOUR3,4,3'],['BOYSHP1','COLOUR4,3,4'],['BOYSHP2','COLOUR3,4,3'],['BOYSHP2','COLOUR4,3,4'],
                    ['CATLAM3','COLOUR3,4,3'],['CATLAM3','COLOUR4,3,4'],['CATLAM4','COLOUR3,4,3'],['CATLAM4','COLOUR4,3,4'],
                    ['BOYSHP1','COLOUR3'],['BOYSHP1','COLOUR4'],['BOYSHP2','COLOUR3'],['BOYSHP2','COLOUR4'],
                    ['CATLAM1','COLOUR3'],['CATLAM1','COLOUR4'],['CATLAM2','COLOUR3'],['CATLAM2','COLOUR4']]
    changes=[]
    check(len(stock.find('lookups'))==len(current.find('lookups')))
    for a,b in zip(stock.findall('lookups/lookup'),current.findall('lookups/lookup')):
        if a.get('name') in ('BOYLAT','boylat','BOYISD','BOYSAW','BOYSPP','TOPMAR','LIGHTS'):
            restored=copy.deepcopy(b)
            i=int(a.get('id'))-1029
            if 0<=i<16:
                check(a.attrib=={'id':str(1029+i),'RCID':str(31081+i),'name':'BOYLAT'})
                check(a.findtext('table-name')=='Simplified' and a.findtext('type')=='Point')
                check([x.text for x in a.findall('attrib-code')]==expected_attrs[i])
                check(b.findtext('instruction')==re.sub(r'^SY\(BOYLAT\d{2}\)','SY('+SELECTED[i]+')',a.findtext('instruction')))
                restored.find('instruction').text=a.findtext('instruction');changes.append(a.get('id'))
            check(ET.tostring(a)==ET.tostring(restored))
    check(changes==list(map(str,range(1029,1045))))
    original_rcids={x.get('RCID') for x in stock.iter() if x.get('RCID')}
    for i,name in enumerate(NAMES):
        nodes=current.findall("symbols/symbol[name='"+name+"']");check(len(nodes)==1)
        n=nodes[0];check(len(name)==8 and name.isascii())
        check(n.find('bitmap').attrib=={'width':'24','height':'28'})
        check(n.find('bitmap/pivot').attrib=={'x':'12','y':'14'})
        check(n.find('bitmap/graphics-location').attrib=={'x':str(244+32*i),'y':'1160'})
        source_name=ORIGINALS[i if i<8 else i-2] if name in ALIASES else name
        original=stock.find("symbols/symbol[name='"+source_name+"']")
        restored=copy.deepcopy(n)
        if name in ALIASES:
            alias_index=ALIASES.index(name)
            check(n.get('RCID')==str(60001+alias_index) and n.get('RCID') not in original_rcids)
            check(ET.tostring(current.find("symbols/symbol[name='"+source_name+"']"))==ET.tostring(original))
            restored.set('RCID',original.get('RCID'));restored.find('name').text=source_name
        if name.startswith('XNLIT'):
            check(n.findtext('prefer-bitmap')=='yes' and original.findtext('prefer-bitmap')=='no')
            restored.find('prefer-bitmap').text='no'
        restored.find('bitmap').attrib=original.find('bitmap').attrib.copy()
        for tag in ('pivot','graphics-location'):restored.find('bitmap/'+tag).attrib=original.find('bitmap/'+tag).attrib.copy()
        # Includes untouched vector HPGL, origin, distances and descriptions.
        check(ET.tostring(restored)==ET.tostring(original))
    tokens=json.loads((root/'docs/design/prototype-tokens.json').read_text())['themes']
    bands=[['green'],['red'],['green'],['red'],['red','green','red'],['green','red','green'],
           ['red','green','red'],['green','red','green'],['black','red','black'],['red','white']]
    for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
        def ink(role):
            value=tokens[theme]['--'+role];rgb=[int(value[j:j+2],16) for j in (1,3,5)]
            if theme=='night':rgb=[round(c*.78) for c in rgb]
            return '#'+''.join(f'{c:02x}' for c in rgb)
        for i,name in enumerate(NAMES):
            svg=ET.parse(root/'resources/chart-style/v1/seamarks'/(name+'-'+table+'.svg')).getroot()
            for n in svg.iter():n.tag=n.tag.rsplit('}',1)[-1]
            check(svg.attrib=={'width':'24','height':'28','viewBox':'0 0 24 28'})
            outer=svg.find('g');check(outer.get('transform')=='translate(12 14)')
            marker=outer.find('g');check(marker.get('transform')==('scale(0.78125)' if name.startswith('XNLIT') else 'scale(0.84375)'))
            if name.startswith('XNLIT'):
                check(marker.find('circle').attrib=={'class':'lighthouse-point','r':'3.5'})
                check(marker.find('path').attrib=={'class':'lighthouse-rays','d':'M0-7V-10M7 0H10M0 7V10M-7 0H-10'})
                check('.lighthouse-point{fill:'+ink('floating')+';stroke:'+ink('chart-text') in svg.findtext('style'))
                check('.lighthouse-rays{stroke:'+ink('mark-'+{'XNLIT011':'red','XNLIT012':'green','XNLIT013':'yellow'}[name])+';stroke-width:1.3}' in svg.findtext('style'))
            else:
                shape=marker.find('g');nodes=list(shape);head=shape.find('g')
                check([n.get('stroke') for n in nodes[1:1+len(bands[i])]]==[ink('mark-'+b) for b in bands[i]])
                check(head.get('fill-opacity')=='.13')
                if i in (0,1,6,7):check(head.find('path').get('d')=='M0 -12l-3 4.5h6Z')
                elif i in (2,3,4,5):check(head.find('rect').attrib=={'x':'-2.6','y':'-11','width':'5.2','height':'4.5','rx':'.4'})
                elif i==8:check([n.attrib for n in head]==[{'cy':'-12','r':'2.2'},{'cy':'-6','r':'2.2'}])
                elif i==9:check(head.find('circle').attrib=={'cy':'-9','r':'3'})
    check(len(current.find('symbols'))==len(stock.find('symbols'))+11)
    # No physical topmark, special-purpose (including actual white/orange),
    # inland beacon, Paper Chart body, other light or cardinal node changes here.
    for name in ('BOYSPP11','BCNGEN01','LIGHTS11','LIGHTS12','LIGHTS13','LITDEF11','LIGHTS81','LIGHTS82','QUESMRK1'):
        check([ET.tostring(n) for n in current.findall("symbols/symbol[name='"+name+"']")]==[ET.tostring(n) for n in stock.findall("symbols/symbol[name='"+name+"']")])
    _,day=decode((source/'rastersymbols-day.png').read_bytes())
    for table,file,neutral in [('DAY_BRIGHT','rastersymbols-day.png',None),('DUSK','rastersymbols-dusk.png',(54,54,54)),('NIGHT','rastersymbols-dark.png',(27,27,27))]:
        raw=(source/file).read_bytes()
        if table=='DAY_BRIGHT':raw,_=derive_day((source/'chartsymbols.xml').read_bytes(),raw,metadata['palette'][table]['CHBLK'])
        if neutral:raw,_=derive(day,raw,neutral,metadata['palette'][table]['CHBLK'])
        ca,before=decode(raw);cb,after=decode((output/file).read_bytes())
        check([(k,v) for k,v in ca if k!=b'IDAT']==[(k,v) for k,v in cb if k!=b'IDAT'])
        for x,w,h in ((20,20,20),(52,24,24),(84,24,24),(116,24,28),(148,24,28),(180,24,28),(212,24,28)):
            for y in range(1160,1160+h):
                start=(y*1500+x)*4;after[start:start+w*4]=before[start:start+w*4]
        for i,name in enumerate(NAMES):
            data=json.loads((root/'resources/chart-style/v1/seamarks'/(name+'-'+table+'-rgba.json')).read_text())
            pixels=bytes.fromhex(''.join(data['rows']));x=244+32*i
            check(hashlib.sha256(pixels).hexdigest()==metadata['seamarkArtwork'][file][name]['themes'][table]['rgbaSha256'])
            for y in range(28):
                for dx in range(24):
                    offset=((1160+y)*1500+x+dx)*4;p=pixels[(y*24+dx)*4:(y*24+dx+1)*4]
                    check(after[offset:offset+4]==(p if p[3] else before[offset:offset+4]))
                    check(before[offset+3]==0)
            check(sum(a>=190 for a in pixels[3::4])>=12) # Native-size strokes are not erased.
        restore_tiles(before,after)
        check(after==before) # All old tiles, neighboring pixels, alpha and hidden RGB.

    import importlib.util
    loader=importlib.util.spec_from_file_location('seamark_generator',root/'tools/generate-xnav-chart-style.py')
    generator=importlib.util.module_from_spec(loader);loader.loader.exec_module(generator)
    original=(source/'chartsymbols.xml').read_bytes();styled=(output/'chartsymbols.xml').read_bytes()
    mutations=[
        lambda t:t.find("symbols/symbol[name='XNLAT013']").set('RCID','1270'),
        lambda t:setattr(t.find("symbols/symbol[name='LIGHTS13']/vector/HPGL"),'text','PU0,0;'),
        lambda t:t.find("symbols/symbol[name='XNLIT013']/bitmap/pivot").set('x','0'),
        lambda t:setattr(t.find("symbols/symbol[name='XNLIT013']/prefer-bitmap"),'text','no'),
        lambda t:setattr(t.find("lookups/lookup[@id='1030']/attrib-code"),'text','BOYSHP2'),
        lambda t:setattr(t.find("lookups/lookup[@id='1058']/instruction"),'text','SY(XNCAN072)'),
        lambda t:t.find('symbols').append(copy.deepcopy(t.find("symbols/symbol[name='XNLAT013']"))),
        lambda t:t.find('lookups').append(copy.deepcopy(t.find("lookups/lookup[@id='1029']")))]
    for mutate in mutations:
        changed=ET.fromstring(styled);mutate(changed)
        try:generator.validate_resource_changes(original,ET.tostring(changed),metadata['palette'])
        except AssertionError:check(True)
        else:raise AssertionError('Unapproved seamark mutation accepted')
