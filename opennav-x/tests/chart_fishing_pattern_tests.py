"""Independent SCRUM-282 AP namespace, source geometry and inverse proof."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from chart_construction_hatch import canonical
from chart_raster_ink import decode,encode


def verify_fishing(source,output,check):
    original=ET.parse(source/'chartsymbols.xml').getroot()
    current=ET.parse(output/'chartsymbols.xml').getroot()
    old=original.find("patterns/pattern[name='FSHFAC03']")
    check(canonical(old)==canonical(current.find("patterns/pattern[name='FSHFAC03']")))
    check(canonical(original.find("symbols/symbol[name='FSHFAC03']"))==
          canonical(current.find("symbols/symbol[name='FSHFAC03']")))
    targets=[n for n in current.findall('lookups/lookup') if 'XNFISH03' in n.findtext('instruction','')]
    check(len(targets)==1 and targets[0].attrib=={'id':'65','RCID':'32101','name':'FSHFAC'})
    check(targets[0].findtext('instruction')=='AP(XNFISH03);LS(DASH,1,CHGRD)')
    check(targets[0].findtext('attrib-code')=='CATFIF1' and targets[0].findtext('table-name')=='Plain')
    for ident in ('405','1115','2182'):
        check(canonical(original.find("lookups/lookup[@id='"+ident+"']"))==
              canonical(current.find("lookups/lookup[@id='"+ident+"']")))
    aliases=current.findall("patterns/pattern[name='XNFISH03']")
    check(len(aliases)==1 and aliases[0].get('RCID')=='60017')
    alias=copy.deepcopy(aliases[0])
    check(alias.findtext('prefer-bitmap')=='no')
    check(alias.find('bitmap').attrib=={'width':'24','height':'24'})
    check(alias.find('bitmap/graphics-location').attrib=={'x':'948','y':'1160'})
    check(alias.find('bitmap/pivot').attrib=={'x':'12','y':'12'})
    alias.remove(alias.find('bitmap'));alias.remove(alias.find('prefer-bitmap'))
    alias.set('RCID','2001');alias.find('name').text='FSHFAC03'
    check(canonical(alias)==canonical(old))
    check(not current.findall("symbols/symbol[name='XNFISH03']"))
    art=(ROOT/'docs/design/prototype/src/chart-marker-art.js').read_text()
    path=re.search("'pattern:FSHFAC03':'([^']+)'",art)[1]
    svg=ET.parse(ROOT/'resources/chart-style/v1/fishing-pattern/XNFISH03.svg').getroot()
    group=list(svg)[0]
    expected=ET.fromstring('<g xmlns="http://www.w3.org/2000/svg">'+path+'</g>')
    check([ET.tostring(n) for n in group]==[ET.tostring(n) for n in expected])
    check(group.attrib=={'transform':'translate(12 12) scale(0.53125)','fill':'none','stroke':'#ffffff',
                        'stroke-width':'1.3','stroke-linecap':'round','stroke-linejoin':'round','shape-rendering':'geometricPrecision'})
    html=(ROOT/'docs/design/prototype/index.html').read_text()
    check('chartSymbolGraphic(s,line?32:17)' in html and path in html)
    check(re.findall(r'--mark-area:(#[a-f0-9]+)',(ROOT/'docs/design/prototype/src/chart-symbols.css').read_text())==['#9c8696','#b8a0b1','#917f8d'])
    for file,rgb in [('rastersymbols-day.png',(156,134,150)),('rastersymbols-dusk.png',(184,160,177)),('rastersymbols-dark.png',(113,99,110))]:
        _,pixels=decode((output/file).read_bytes())
        alpha=[];count=0
        for y in range(1160,1184):
            for x in range(948,972):
                i=(y*1500+x)*4;alpha.append(pixels[i+3])
                if pixels[i+3]:count+=1;check(pixels[i:i+3]==bytes(rgb))
        check(count==48)
        check(hashlib.sha256(bytes(alpha)).hexdigest()=='6fd32cd07301655501afc6e16ebb9e40f072a771bb5c604f4959425e1cc5c6fb')


if __name__=='__main__':
    import argparse
    from unittest.mock import patch
    import chart_fishing_pattern as art
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--prior',type=Path,required=True);p.add_argument('--current',type=Path,required=True)
    a=p.parse_args();checks=[0]
    def check(b):
        checks[0]+=1
        assert b, f'Fishing check {checks[0]}'
    verify_fishing(a.source,a.current,check)
    prior=(a.prior/'chartsymbols.xml').read_bytes();current=(a.current/'chartsymbols.xml').read_bytes()
    before,after=ET.fromstring(prior),ET.fromstring(current)
    after.find('patterns').remove(after.find("patterns/pattern[name='XNFISH03']"))
    after.find("lookups/lookup[@id='65']/instruction").text='AP(FSHFAC03);LS(DASH,1,CHGRD)'
    check(canonical(before)==canonical(after))
    for file in ('rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png'):
        cb,b=decode((a.prior/file).read_bytes());ca,c=decode((a.current/file).read_bytes())
        check([(k,v) for k,v in cb if k!=b'IDAT']==[(k,v) for k,v in ca if k!=b'IDAT'])
        changes=[i for i in range(0,len(b),4) if b[i:i+4]!=c[i:i+4]]
        check(len(changes)==48)
        check(all(948<=i//4%1500<972 and 1160<=i//4//1500<1184 for i in changes))
        for i in changes:check(b[i+3]==0 and c[i+3]>0);c[i:i+4]=b[i:i+4]
        check(c==b)
    check((a.prior/'S52RAZDS.RLE').read_bytes()==(a.current/'S52RAZDS.RLE').read_bytes())
    def reject(call):
        try:call()
        except AssertionError:check(True)
        else:raise AssertionError('Invalid pattern mutation accepted')
    spec=importlib.util.spec_from_file_location('generator',ROOT/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    metadata=json.loads((a.current/'manifest.json').read_text());source=(a.source/'chartsymbols.xml').read_bytes()
    def bad(mutator):
        t=ET.fromstring(current);mutator(t)
        reject(lambda:g.validate_resource_changes(source,ET.tostring(t),metadata['palette']))
    for target,tag,value in [('65','attrib-code','CATFIF2'),('65','table-name','Symbolized'),('405','instruction','AP(XNFISH03)')]:
        bad(lambda t:setattr(t.find("lookups/lookup[@id='"+target+"']/"+tag),'text',value))
    for path,attr,value in [('bitmap/pivot','x','13'),('vector/distance','min','1999'),('vector','height','604'),('bitmap/graphics-location','x','940')]:
        bad(lambda t:t.find("patterns/pattern[name='XNFISH03']/"+path).set(attr,value))
    bad(lambda t:setattr(t.find("patterns/pattern[name='FSHFAC03']/HPGL"),'text','PU0,0;'))
    bad(lambda t:t.find("symbols/symbol[name='FSHFAC03']/bitmap").set('width','24'))
    t=ET.fromstring(prior);t.find('.//bitmap/graphics-location').attrib={'x':'946','y':'1158'}
    reject(lambda:art.relocate(ET.tostring(t,encoding='unicode')))
    chunks,pixels=decode((a.prior/'rastersymbols-day.png').read_bytes())
    for x,y in ((948,1160),(946,1158)):
        damaged=bytearray(pixels);damaged[(y*1500+x)*4+3]=1
        reject(lambda:art.paint(encode(chunks,damaged),'DAY_BRIGHT'))
    read=Path.read_bytes
    with patch.object(Path,'read_bytes',lambda p:read(p)+(b'x' if p.name=='XNFISH03.svg' else b'')):
        reject(art.coverage)
    print(f'{checks[0]} focused fishing-pattern source, namespace, complete inverse and negative checks passed')
