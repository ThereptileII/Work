"""Deterministic palette/provenance tests; not ENC visual/safety acceptance."""
import importlib.util
import hashlib
import json
import re
from pathlib import Path
import shutil
import tempfile
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
spec=importlib.util.spec_from_file_location('generate',ROOT/'tools/generate-xnav-chart-style.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
source=ROOT/'upstream/OpenCPN/data/s57data'
checks=0
def check(value):
    global checks
    checks+=1
    assert value, f'Chart presentation resource check {checks}'
def luminance(rgb):
    channels=[v/255 for v in rgb]
    linear=[v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4 for v in channels]
    return sum(v*w for v,w in zip(linear,[.2126,.7152,.0722]))
def contrast(a,b):
    bright,dark=sorted([luminance(a),luminance(b)],reverse=True)
    return (bright+.05)/(dark+.05)
with tempfile.TemporaryDirectory(prefix='xnav-chart-test-') as d:
    folder=Path(d)
    output=folder/'generated'
    original={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}
    data=g.generate(source,output)
    check(data['upstreamCommit']=='37fd0cddb7334fe489e9f18aa163977a9c5c84f7')
    check(len(data['palette'])==3)
    first={p.name:p.read_bytes() for p in output.iterdir()}
    check(g.generate(source,output)==data)
    check(first=={p.name:p.read_bytes() for p in output.iterdir()})
    for name,identity in data['files'].items():
        check(hashlib.sha256((output/name).read_bytes()).hexdigest()==identity['sha256'])
        if name == 'S52RAZDS.RLE':
            check((output/name).read_bytes()==g.pinned_bytes(source/name,identity))
    from chart_anchor_resources_tests import verify_anchor
    verify_anchor(source,output,data,check)
    _,day=g.decode((source/'rastersymbols-day.png').read_bytes())
    for name,ink in data['neutralRasterInk'].items():
        before_chunks,before=g.decode((source/name).read_bytes())
        after_chunks,after=g.decode((output/name).read_bytes())
        check([(k,v) for k,v in before_chunks if k!=b'IDAT']==[(k,v) for k,v in after_chunks if k!=b'IDAT'])
        # The separate anchor proof checks every new alpha byte and pixel.
        # Restore only that verified tile before the original neutral-ink proof.
        for y in range(1160,1180):
            start=(y*1500+20)*4
            after[start:start+80]=before[start:start+80]
        check(before[3::4]==after[3::4])
        changed=[i for i in range(0,len(before),4) if before[i:i+4]!=after[i:i+4]]
        check(len(changed)==ink['changedPixels']==42100)
        check(all(day[i:i+3]==b'\x07\x07\x07' and day[i+3]==before[i+3] and before[i+3]>0 for i in changed))
        check(all(before[i:i+3]==bytes(ink['sourceRgb']) and after[i:i+3]==bytes(ink['targetRgb']) for i in changed))
        check(hashlib.sha256(after).hexdigest()=={
            'rastersymbols-dusk.png':'2513060ab2decd060ae8cd80919d1922f85f0a4282b99dee51b826d860ba9a5e',
            'rastersymbols-dark.png':'36a37bf3fe9257893adfb82e3d737ae62e98f631b4ed4bffe645e42d1697e8c9'}[name])
    a,b=ET.parse(source/'chartsymbols.xml').getroot(),ET.parse(output/'chartsymbols.xml').getroot()
    # Independent exceptions: two area fills and geographic OBJNAM ink only.
    expected_ids={'16':('32052','Plain'),'356':('32391','Symbolized')}
    old="AC(CHBRN);TX(OBJNAM,1,2,3,'16120',0,0,CHBLK,26);LS(SOLD,1,LANDF)"
    changed=[];geographic=[]
    for stock,styled in zip(a.find('lookups'),b.find('lookups')):
        expected=stock.findtext('instruction')
        if stock.get('id') in expected_ids:
            rcid,table=expected_ids[stock.get('id')]
            check(stock.attrib=={'id':stock.get('id'),'RCID':rcid,'name':'BUAARE'})
            check(stock.findtext('type')=='Area' and stock.findtext('table-name')==table)
            check(expected==old)
            expected=expected.replace('AC(CHBRN)','AC(XNBUA)')
            changed.append(stock.get('id'))
        if stock.get('name') in {'BUAARE','LNDARE','LNDRGN','SEAARE'}:
            new=re.sub(r"(TX\(OBJNAM,[^;()]+,)(CHBLK|CHGRD)(,26\))",r"\g<1>XNGEO\3",expected)
            if new!=expected:geographic.append((stock.get('id'),stock.get('RCID')))
            expected=new
        check(styled.findtext('instruction')==expected)
        styled.find('instruction').text=stock.findtext('instruction')
        check(ET.tostring(stock)==ET.tostring(styled))
    check(set(changed)==set(expected_ids) and len(changed)==2)
    check(len(geographic)==18 and data['geographicNameLookups']==18)
    check({i for i,_ in geographic}=={'16','84','91','178','356','424','431','519','1066','1134','1174','1240','1996','2209','2290','2358'})
    # Independently restore the sole authorized bitmap metadata change.
    stock=a.findall("symbols/symbol[name='ACHARE51']")[-1].find('bitmap')
    styled=b.findall("symbols/symbol[name='ACHARE51']")[-1].find('bitmap')
    styled.attrib=stock.attrib.copy()
    for tag in ('pivot','graphics-location'):styled.find(tag).attrib=stock.find(tag).attrib.copy()
    for section in ['lookups','line-styles','patterns','symbols']:
        check(ET.tostring(a.find(section))==ET.tostring(b.find(section)))
    for stock,styled in zip(a.find('color-tables'),b.find('color-tables')):
        check(stock.attrib==styled.attrib)
        for name in ('XNBUA','XNGEO'):
            added=styled.findall("color[@name='"+name+"']")
            check(len(added)==(1 if stock.get('name') in data['palette'] else 0))
            for entry in added:styled.remove(entry)
        check(len(stock)==len(styled))
        for before,after in zip(stock,styled):
            if before.tag=='color' and stock.attrib['name'] in data['palette'] and before.attrib['name'] in g.ALLOWED:
                check(before.attrib['name']==after.attrib['name'])
            else:check(ET.tostring(before)==ET.tostring(after))
    for table,colors in data['palette'].items():
        check(len({tuple(colors[n]) for n in ['DEPDW','DEPMD','DEPMS','DEPVS','DEPIT']})==5)
        check(colors['DEPSC']!=colors['DEPCN'])
        check(colors['SNDG1']!=colors['SNDG2'])
        check(colors['LANDA']!=colors['DEPDW'])
        check(colors['XNBUA']==colors['CSTLN'])
        check(colors['XNBUA'] not in [colors[n] for n in ['LANDA','DEPDW','DEPMD','DEPMS','DEPVS','DEPIT']])
    for color in g.ALLOWED:
        check(luminance(data['palette']['NIGHT'][color])<luminance(data['palette']['DUSK'][color]))
    # Guard the observed invisible dark ink on the new Night water. These
    # numerical checks do not replace actual symbol/hazard review.
    check(data['palette']['DAY_BRIGHT']['CHBLK']==(7,7,7))
    for table in ('DUSK','NIGHT'):
        colors=data['palette'][table]
        check(contrast(colors['CHBLK'],colors['DEPDW'])>=4)
        check(contrast(colors['CHBLK'],colors['DEPVS'])>=2)
        check(contrast(colors['CHBLK'],colors['LANDA'])>=3)
    # Fail closed on accidental broad recoloring, altered hazards, labels,
    # classifications, point symbols, or absent/duplicate dedicated paint roles.
    def reject(mutator):
        tree=ET.parse(output/'chartsymbols.xml').getroot();mutator(tree)
        try:g.validate_resource_changes((source/'chartsymbols.xml').read_bytes(),ET.tostring(tree),data['palette'])
        except AssertionError:check(True)
        else:raise AssertionError('Unexpected resource mutation accepted')
    reject(lambda t:t.find("color-tables/color-table/color[@name='CHBRN']").set('r','1'))
    reject(lambda t:setattr(t.find("lookups/lookup[@name='OBSTRN']/instruction"),'text','AC(XNBUA)'))
    reject(lambda t:setattr(t.find("lookups/lookup[@id='16']/instruction"),'text',old.replace('AC(CHBRN)','AC(XNBUA)').replace('16120','15110')))
    reject(lambda t:setattr(t.find("lookups/lookup[@id='356']/display-cat"),'text','Displaybase'))
    reject(lambda t:setattr(t.find("lookups/lookup[@id='1066']/instruction"),'text','AC(XNBUA)'))
    reject(lambda t:t.find('color-tables/color-table').remove(t.find("color-tables/color-table/color[@name='XNBUA']")))
    reject(lambda t:t.find('color-tables/color-table').append(ET.fromstring('<color name="XNBUA" r="175" g="191" b="174"/>')))
    for name in ('LANDA','XNBUA','XNGEO'):
        path="color-tables/color-table/color[@name='"+name+"']"
        reject(lambda t:t.find(path).set('r','1'))
        reject(lambda t:t.find(path).set('a','0'))
        reject(lambda t:t.find(path).set('unexpected','true'))
    reject(lambda t:t.find("color-tables/color-table/color[@name='XNBUA']").append(ET.fromstring('<color name="CHBRN" r="1" g="1" b="1"/>')))
    reject(lambda t:setattr(t.find("lookups/lookup[@name='LIGHTS']/instruction"),'text','TX(OBJNAM,1,2,3,15112,0,0,XNGEO,26)'))
    reject(lambda t:t.find('color-tables/color-table').remove(t.find("color-tables/color-table/color[@name='XNGEO']")))
    reject(lambda t:t.find('color-tables/color-table').append(ET.fromstring('<color name="XNGEO" r="1" g="1" b="1"/>')))
    reject(lambda t:t.findall("symbols/symbol[name='ACHARE51']")[-1].find('bitmap/pivot').set('x','11'))
    reject(lambda t:t.findall("symbols/symbol[name='ACHARE51']")[0].find('bitmap/pivot').set('x','11'))
    reject(lambda t:t.findall("symbols/symbol[name='ACHARE51']")[-1].find('bitmap/origin').set('x','1'))
    reject(lambda t:t.findall("symbols/symbol[name='ACHARE51']")[-1].find('bitmap').set('width','21'))
    check(original=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()})
    damaged=folder/'damaged';damaged.mkdir()
    for name in data['files']:shutil.copyfile(source/name,damaged/name)
    for name in ['chartsymbols.xml','S52RAZDS.RLE']:
        (damaged/name).write_bytes((source/name).read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
    check(g.generate(damaged,folder/'windows-checkout')==data)
    check(first=={p.name:p.read_bytes() for p in (folder/'windows-checkout').iterdir()})
    (damaged/'chartsymbols.xml').write_bytes((damaged/'chartsymbols.xml').read_bytes()+b'changed')
    try:g.generate(damaged,folder/'refused')
    except AssertionError:check(not (folder/'refused').exists())
    else:raise AssertionError('Unknown presentation source was accepted')
print(f'{checks} chart-resource checks passed; real ENC, native palette review and GL/software gates remain required')
