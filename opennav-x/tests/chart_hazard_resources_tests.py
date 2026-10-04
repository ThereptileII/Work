"""Independent SCRUM-280 geometry, semantic inverse and occupied-slot refusals."""
import copy, hashlib, importlib.util, json, re
from pathlib import Path
import xml.etree.ElementTree as ET
from unittest.mock import patch
from chart_raster_ink import decode, encode
ROOT=Path(__file__).resolve().parents[1]
# Review oracle independent of the production helper/mapping/provenance.
SPEC={
 'UWTROC03':(2249,852,'eed289d33362016f13383ed1c9bdb484b7c4fe4b1643badbf923f078df50dc0a'),
 'UWTROC04':(3164,884,'d2c1ad63c199582835453e4ddce89be2d1b400cb99b4760432a11901a171bf0b'),
 'WRECKS05':(2038,916,'207b813777a9f78693fe1da56a51df8053c5e7289d2438949d25fef4a2b6842c')}
COLORS=((104,123,122),(173,187,177),(91,104,94))
def restore_hazards(before,after):
    for _,x,digest in SPEC.values():
        alpha=bytes(after[((1160+y)*1500+x+dx)*4+3] for y in range(24) for dx in range(24))
        assert hashlib.sha256(alpha).hexdigest()==digest, 'Hazard alpha changed'
        ink=set()
        for y in range(24):
            for dx in range(24):
                i=((1160+y)*1500+x+dx)*4
                assert before[i+3]==0, 'Hazard slot was occupied'
                if alpha[y*24+dx]:ink.add(tuple(after[i:i+3]))
                else:assert after[i:i+4]==before[i:i+4], 'Invisible hazard pixel changed'
            i=((1160+y)*1500+x)*4;after[i:i+96]=before[i:i+96]
        assert len(ink)==1 and next(iter(ink)) in COLORS, 'Unexpected hazard ink'

def verify_hazards(source,output,check):
    stock=ET.parse(source/'chartsymbols.xml').getroot();current=ET.parse(output/'chartsymbols.xml').getroot()
    html=(ROOT/'docs/design/prototype/index.html').read_text()
    for name,(rcid,x,digest) in SPEC.items():
        ns=current.findall("symbols/symbol[name='"+name+"']");check(len(ns)==1 and ns[0].get('RCID')==str(rcid))
        n=ns[0];check(n.find('bitmap').attrib=={'width':'24','height':'24'})
        check(n.find('bitmap/pivot').attrib=={'x':'12','y':'12'})
        check(n.find('bitmap/graphics-location').attrib=={'x':str(x),'y':'1160'})
        old=stock.find("symbols/symbol[name='"+name+"']");restored=copy.deepcopy(n)
        restored.find('bitmap').attrib=old.find('bitmap').attrib.copy()
        for tag in ('pivot','graphics-location'):restored.find('bitmap/'+tag).attrib=old.find('bitmap/'+tag).attrib.copy()
        check(ET.tostring(restored)==ET.tostring(old)) # description, HPGL, colors, prefer-bitmap, distances.
        svg=ET.parse(ROOT/'resources/chart-style/v1/hazards'/(name+'.svg')).getroot()
        for e in svg.iter():e.tag=e.tag.rsplit('}',1)[-1]
        check(svg.attrib=={'width':'24','height':'24','viewBox':'0 0 24 24'})
        g=svg.find('g');check(g.attrib=={'transform':'translate(12 12)'})
        g=g.find('g');check(g.attrib=={'class':'chart-marker-art marker-hazard','transform':'scale(0.84375)'})
        expected=ET.fromstring('<g>'+re.search("'point:"+name+"':'([^']+)'",html)[1]+'</g>')
        check([ET.tostring(e) for e in g]==[ET.tostring(e) for e in expected])
        check(svg.findtext('style')=='.chart-marker-art{fill:none;stroke:#ffffff;stroke-width:1.3;stroke-linecap:round;stroke-linejoin:round;pointer-events:none;shape-rendering:geometricPrecision}.chart-marker-art.marker-hazard{opacity:.8;stroke-width:1.2}.marker-dot{fill:#ffffff;stroke:none}')
    # Every rock/wreck/obstruction lookup and every direct consumer is unchanged,
    # including Paper, $CSYMB, foul ground, conditional call and display priority.
    for a,b in zip(stock.findall('lookups/lookup'),current.findall('lookups/lookup')):
        if a.get('name') in ('UWTROC','WRECKS','OBSTRN') or any(n in (a.findtext('instruction') or '') for n in SPEC):
            check(ET.tostring(a)==ET.tostring(b))
    for name in ('ISODGR51','WRECKS01','WRECKS04','WRECKS07','QUAPOS01','QUAPOS02','QUAPOS03','LOWACC03','DANGER51','DANGER52','LNDARE01'):
        check([ET.tostring(n) for n in stock.findall("symbols/symbol[name='"+name+"']")]==[ET.tostring(n) for n in current.findall("symbols/symbol[name='"+name+"']")])
    meta=json.loads((output/'manifest.json').read_text())
    for table,file,color in zip(('DAY_BRIGHT','DUSK','NIGHT'),('rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png'),COLORS):
        _,before=decode((source/file).read_bytes());_,after=decode((output/file).read_bytes())
        for name,(_,x,digest) in SPEC.items():
            tile=bytes(v for y in range(24) for v in after[((1160+y)*1500+x)*4:((1160+y)*1500+x+24)*4])
            check(hashlib.sha256(tile[3::4]).hexdigest()==digest)
            check(all(tuple(tile[i:i+3])==color for i in range(0,len(tile),4) if tile[i+3]))
            check(all(not any(before[(y*1500+x-2)*4+3:(y*1500+x+26)*4:4]) for y in range(1158,1186)))
            check(meta['hazardArtwork'][file][name]['pivot']==[12,12])
    import chart_hazard_art as art
    def refuses(action):
        try:action()
        except AssertionError:check(True)
        else:raise AssertionError('Invalid hazard resource accepted')
    original=(source/'chartsymbols.xml').read_bytes();styled=(output/'chartsymbols.xml').read_bytes()
    module=importlib.util.spec_from_file_location('hazard_generator',ROOT/'tools/generate-xnav-chart-style.py');gen=importlib.util.module_from_spec(module);module.loader.exec_module(gen)
    for expr,tag,attribute,value in [("symbols/symbol[name='UWTROC03']",'bitmap/pivot','x','0'),("symbols/symbol[name='WRECKS05']",'vector/HPGL',None,'PU0,0;'),("lookups/lookup[@id='1276']",'instruction',None,'SY(UWTROC04)'),("lookups/lookup[@id='1298']",'display-cat',None,'Displaybase'),("symbols/symbol[name='QUAPOS01']",'description',None,'changed')]:
        tree=ET.fromstring(styled);target=tree.find(expr+'/'+tag)
        if attribute:target.set(attribute,value)
        else:target.text=value
        refuses(lambda:gen.validate_resource_changes(original,ET.tostring(tree),meta['palette']))
    tree=ET.fromstring(original);tree.find('.//bitmap/graphics-location').attrib={'x':'850','y':'1158'}
    refuses(lambda:art.relocate(ET.tostring(tree,encoding='unicode')))
    read=Path.read_bytes
    with patch.object(Path,'read_bytes',lambda p:read(p)+(b'changed' if p.name=='WRECKS05.svg' else b'')):refuses(art.coverage)
    chunks,pixels=decode((source/'rastersymbols-day.png').read_bytes());pixels[(1158*1500+882)*4+3]=1
    refuses(lambda:art.paint(encode(chunks,pixels),'DAY_BRIGHT'))
