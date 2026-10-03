"""Independent classified-body geometry, semantic, palette and pixel oracles."""
import colorsys,copy,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from chart_raster_ink import decode
ASSETS=ROOT/'resources/chart-style/v1/special-buoy'
TABLES=(('DAY_BRIGHT','day','rastersymbols-day.png'),('DUSK','dusk','rastersymbols-dusk.png'),('NIGHT','night','rastersymbols-dark.png'))

def rgba(table):
    data=json.loads((ASSETS/('XNSPPW01-'+table+'-rgba.json')).read_text())
    assert data['width']==24 and data['height']==28
    return bytes.fromhex(''.join(data['rows']))

def restore_buoy(before,after):
    # Validate the whole owned tile, including untouched transparent RGB,
    # before undoing only its proven painted pixels. No broad row exclusion.
    for table,_,_ in TABLES:
        pixels=rgba(table);valid=True
        for dy in range(28):
            for dx in range(24):
                i=((1160+dy)*1500+660+dx)*4;p=pixels[(dy*24+dx)*4:(dy*24+dx+1)*4]
                valid &= before[i+3]==0 and after[i:i+4]==(p if p[3] else before[i:i+4])
        if valid:
            for dy in range(28):
                for dx in range(24):
                    i=((1160+dy)*1500+660+dx)*4
                    if pixels[(dy*24+dx)*4+3]:after[i:i+4]=before[i:i+4]
            return
    raise AssertionError('White/orange pillar tile differs from approved complete coverage')

def verify_special_buoy(source,output,check):
    stock=ET.parse(source/'chartsymbols.xml').getroot();current=ET.parse(output/'chartsymbols.xml').getroot()
    old=stock.find("symbols/symbol[name='BOYSPP11']");new=current.find("symbols/symbol[name='XNSPPW01']")
    check(len(current.findall("symbols/symbol[name='XNSPPW01']"))==1)
    check(new.get('RCID')=='60012' and new.find('bitmap').attrib=={'width':'24','height':'28'})
    check(new.find('bitmap/pivot').attrib=={'x':'12','y':'14'})
    check(new.find('bitmap/graphics-location').attrib=={'x':'660','y':'1160'})
    check(ET.tostring(current.find("symbols/symbol[name='BOYSPP11']"))==ET.tostring(old))
    restored=copy.deepcopy(new);restored.set('RCID','1297');restored.find('name').text='BOYSPP11'
    restored.find('bitmap').attrib=old.find('bitmap').attrib.copy()
    for tag in ['pivot','graphics-location']:restored.find('bitmap/'+tag).attrib=old.find('bitmap/'+tag).attrib.copy()
    check(ET.tostring(restored)==ET.tostring(old))
    for a,b in zip(stock.findall('lookups/lookup'),current.findall('lookups/lookup')):
        if a.get('name') in ('BOYSPP','TOPMAR','LIGHTS'):check(ET.tostring(a)==ET.tostring(b))
    for table,theme,filename in TABLES:
        tree=ET.parse(ASSETS/('XNSPPW01-'+table+'.svg')).getroot()
        for n in tree.iter():n.tag=n.tag.rsplit('}',1)[-1]
        outer=tree.find('g');shape=outer.find('g')
        check(outer.attrib=={'transform':'translate(12 14)'})
        check(shape.get('transform')=='scale(0.84375)' and shape.get('stroke-width')=='1.3')
        check([n.tag for n in shape]==['path','path','path','path','circle'])
        check([n.get('d') for n in shape.findall('path')]==['M0-7V8','M0 -3v5.5','M0 2.5v5.5','M-2.5 6H2.5'])
        check(shape.find('circle').get('cy')=='9' and shape.find('circle').get('r')=='1.7')
        tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes'][theme]
        def color(key):
            value=tokens[key];rgb=[int(value[i:i+2],16) for i in (1,3,5)]
            if theme=='night':rgb=[round(x*.78) for x in rgb]
            return '#'+''.join(f'{x:02x}' for x in rgb)
        orange=stock.find("color-tables/color-table[@name='"+table+"']/color[@name='CHCOR']")
        orange='#'+''.join(f'{int(orange.get(k)):02x}' for k in ('r','g','b'))
        orange={'NIGHT':'#d07131'}.get(table,orange)
        check(shape.get('stroke')==color('--mark-black') and shape[1].get('stroke')==color('--mark-white'))
        check(shape[2].get('stroke')==orange and shape[-1].get('fill')==color('--water'))
        check(all(n.get('stroke-width')=='1.6' for n in [shape[1],shape[2]]))
        if table=='NIGHT':
            # Independent source-hue/lightness and rendered-water contrast gate.
            def contrast(a,b):
                def lum(rgb):
                    linear=[v/255/12.92 if v/255<=.04045 else ((v/255+.055)/1.055)**2.4 for v in rgb]
                    return sum(v*w for v,w in zip(linear,(.2126,.7152,.0722)))
                lo,hi=sorted((lum(a),lum(b)));return (hi+.05)/(lo+.05)
            h,_,sat=colorsys.rgb_to_hls(52/255,28/255,12/255)
            chosen=tuple(round(v*255) for v in colorsys.hls_to_rgb(h,32982/65535,sat))
            previous=tuple(round(v*255) for v in colorsys.hls_to_rgb(h,32981/65535,sat))
            check(chosen==(208,113,49) and previous==(207,112,49))
            fills=[current.find("color-tables/color-table[@name='NIGHT']/color[@name='"+name+"']") for name in ('DEPDW','DEPMD','DEPMS','DEPVS','DEPIT')]
            backgrounds=[tuple(int(n.get(k)) for k in ('r','g','b')) for n in fills]
            check(min(contrast(chosen,b) for b in backgrounds)>=3)
            check(min(contrast(previous,b) for b in backgrounds)<3)
            check(min(contrast((52,28,12),b) for b in backgrounds)<3)
        old_palette=stock.find("color-tables/color-table[@name='"+table+"']/color[@name='CHCOR']")
        new_palette=current.find("color-tables/color-table[@name='"+table+"']/color[@name='CHCOR']")
        check(ET.tostring(old_palette)==ET.tostring(new_palette))
        _,before=decode((source/filename).read_bytes());_,after=decode((output/filename).read_bytes());pixels=rgba(table)
        check(sum(bool(x) for x in pixels[3::4])==50)
        for dy in range(28):
            for dx in range(24):
                i=((1160+dy)*1500+660+dx)*4;p=pixels[(dy*24+dx)*4:(dy*24+dx+1)*4]
                check(before[i+3]==0 and after[i:i+4]==(p if p[3] else before[i:i+4]))

if __name__=='__main__':
    import argparse,sys
    sys.path.insert(0,str(ROOT/'tools'))
    import chart_special_buoy_art
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--report',type=Path,required=True);args=p.parse_args()
    checks=0
    def check(value):
        global checks
        checks+=1
        assert value,checks
    verify_special_buoy(args.source,args.generated,check)
    baseline=ET.parse(args.baseline/'chartsymbols.xml').getroot()
    final=ET.parse(args.generated/'chartsymbols.xml').getroot()
    def canonical(tree):
        tree=copy.deepcopy(tree)
        for n in tree.iter():
            if n.text is not None and not n.text.strip():n.text=None
            if n.tail is not None and not n.tail.strip():n.tail=None
        return ET.tostring(tree)
    def restored(tree):
        tree=copy.deepcopy(tree)
        chart_special_buoy_art.restore_for_validation(baseline,tree)
        assert canonical(tree)==canonical(baseline)
    restored(final);check(True)
    for change in ('pivot','rcid','hpgl','lookup-attribute','lookup-instruction','duplicate'):
        tree=copy.deepcopy(final);alias=tree.find("symbols/symbol[name='XNSPPW01']")
        if change=='pivot':alias.find('bitmap/pivot').set('x','13')
        elif change=='rcid':alias.set('RCID','60013')
        elif change=='hpgl':tree.find("symbols/symbol[name='BOYSPP11']/vector/HPGL").text+='PU0,0;'
        elif change=='lookup-attribute':tree.find("lookups/lookup[@id='1947']/attrib-code").text='BOYSHP1'
        elif change=='lookup-instruction':tree.find("lookups/lookup[@id='1947']/instruction").text='SY(BOYSPP11);'
        else:tree.find('symbols').append(copy.deepcopy(alias))
        try:restored(tree)
        except AssertionError:check(True)
        else:raise AssertionError('Mutation escaped: '+change)
    for _,_,filename in TABLES:
        _,before=decode((args.baseline/filename).read_bytes());_,after=decode((args.generated/filename).read_bytes())
        after=bytearray(after);check(sum(before[i:i+4]!=after[i:i+4] for i in range(0,len(before),4))==50)
        restore_buoy(before,after);check(before==after)
    check((args.baseline/'S52RAZDS.RLE').read_bytes()==(args.generated/'S52RAZDS.RLE').read_bytes())
    report={'checks':checks,'baselineManifestSha256':hashlib.sha256((args.baseline/'manifest.json').read_bytes()).hexdigest(),'manifestSha256':hashlib.sha256((args.generated/'manifest.json').read_bytes()).hexdigest(),'changedPixelsPerTheme':50,'wholeXmlInverse':True,'wholeRgbaInverse':True,'negativeXmlCases':6,'nightSolidContrastAtLeast':3,'duskDispatch':'stock Rule; lifted cream rejected'}
    args.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
