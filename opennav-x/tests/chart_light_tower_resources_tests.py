"""SCRUM-308 source-locked tower role/geometry and negative resource proof."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from chart_raster_ink import decode
from chart_seamark_art import canonical
import chart_light_tower as tower
# Independent resource contract, not the production recipe.
SPECS=(('XNLTWR01','TOWERS01',343,980,60018,'XNGEO'),('XNLTWR03','TOWERS03',367,1004,60019,'XNBLO'))

def tile(pixels,x,y=1160):
    return b''.join(pixels[((y+j)*1500+x)*4:((y+j)*1500+x+14)*4] for j in range(26))

def expected(day,colors,index):
    outline=(34,31,31); delta=(196,205,214); denominator=sum(d*d for d in delta)
    result=bytearray()
    for i in range(0,len(day),4):
        a=day[i+3]
        n=sum((day[i+c]-outline[c])*delta[c] for c in range(3)) if a and not index else 0
        rgb=[(o*denominator+n*(f-o)+denominator//2)//denominator for o,f in zip(colors[index],colors[2])] if a else [0,0,0]
        result.extend((*rgb,a))
    return bytes(result)

def restore_towers(before,after):
    # Exact isolated slots only. The focused verify below proves every pixel.
    for _,_,_,x,_,_ in SPECS:
        for y in range(1160,1186):
            i=(y*1500+x)*4
            assert not any(before[i+3:i+56:4]), 'Original tower slot occupied'
            after[i:i+56]=before[i:i+56]

def verify(source,generated,check,baseline=None):
    stock=ET.parse(source/'chartsymbols.xml').getroot()
    current=ET.parse(generated/'chartsymbols.xml').getroot()
    metadata=json.loads((generated/'manifest.json').read_text())
    _,day=decode((source/'rastersymbols-day.png').read_bytes())
    # Every LIGHTS/LNDMRK lookup remains unchanged, including priorities,
    # OBJNAM, Paper and all unknown/special semantic branches.
    for lookup in stock.findall('lookups/lookup'):
        if lookup.get('name') in ('LIGHTS','LNDMRK'):
            check(canonical(lookup)==canonical(current.find("lookups/lookup[@id='"+lookup.get('id')+"']")))
    for name,original,x,dest,rcid,role in SPECS:
        old=stock.findall("symbols/symbol[name='"+original+"']")
        actual=current.findall("symbols/symbol[name='"+original+"']")
        check(len(old)==len(actual)==2)
        check([canonical(n) for n in old]==[canonical(n) for n in actual])
        aliases=current.findall("symbols/symbol[name='"+name+"']")
        check(len(aliases)==1)
        alias=copy.deepcopy(aliases[0]);check(alias.get('RCID')==str(rcid))
        check(alias.findtext('definition')=='R' and alias.find('vector') is None)
        check(alias.find('bitmap').attrib=={'width':'14','height':'26'})
        check(alias.find('bitmap/pivot').attrib=={'x':'6','y':'22'})
        check(alias.find('bitmap/graphics-location').attrib=={'x':str(dest),'y':'1160'})
        check(alias.findtext('color-ref')=='ADEPMDB'+role)
        alias.set('RCID',old[-1].get('RCID'));alias.find('name').text=original
        alias.find('color-ref').text=old[-1].findtext('color-ref')
        alias.find('bitmap/graphics-location').attrib=old[-1].find('bitmap/graphics-location').attrib
        check(canonical(alias)==canonical(old[-1]))
    for table,suffix in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','dark')]:
        colors=metadata['palette'][table]
        # Direct prototype role oracle with one Night chart normalization.
        tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes'][{'DAY_BRIGHT':'day','DUSK':'dusk','NIGHT':'night'}[table]]
        for role,token in [('XNGEO','--chart-text'),('XNBLO','--mark-black')]:
            value=tokens[token];rgb=[int(value[i:i+2],16) for i in (1,3,5)]
            if table=='NIGHT':rgb=[(c*78+50)//100 for c in rgb]
            check(colors[role]==rgb)
        inks=[colors['XNGEO'],colors['XNBLO'],colors['DEPMD']]
        _,old=decode((source/f'rastersymbols-{suffix}.png').read_bytes())
        _,new=decode((generated/f'rastersymbols-{suffix}.png').read_bytes())
        for index,(name,original,x,dest,rcid,role) in enumerate(SPECS):
            shape=tile(day,x,742);actual=tile(new,dest)
            check(actual==expected(shape,inks,index))
            check(actual[3::4]==tile(old,x,742)[3::4]==shape[3::4])
            check(actual!=tile(old,x,742))
            check(tile(new,x,742)==tile(old,x,742))
            check(hashlib.sha256(actual).hexdigest()==metadata['lightTowerArtwork'][f'rastersymbols-{suffix}.png'][name]['rgbaSha256'])
            spec=tower.recipe()[name]
            for byte in (3,151,400):
                corrupted=bytearray(tile(old,x,742));corrupted[byte]^=1
                try:tower.tile_rgba(corrupted,table,spec,colors)
                except AssertionError:check(True)
                else:raise AssertionError('Accepted source pixel drift')
        if baseline:
            _,base=decode((baseline/f'rastersymbols-{suffix}.png').read_bytes())
            restored=bytearray(new);restore_towers(base,restored);check(restored==base)
    for name,_,_,_,_,_ in SPECS:
        query="symbols/symbol[name='"+name+"']"
        for field,value in [('bitmap/pivot','7'),('bitmap','15'),('name','OTHER001')]:
            bad=copy.deepcopy(current);node=bad.find(query+'/'+field)
            if field=='name':node.text=value
            elif field=='bitmap':node.set('width',value)
            else:node.set('x',value)
            try:tower.restore_for_validation(stock,bad)
            except AssertionError:check(True)
            else:raise AssertionError('Accepted alias drift')
    for mutation in ('selector','effective','slot'):
        bad=copy.deepcopy(stock)
        if mutation=='selector':bad.find("lookups/lookup[@id='1147']/instruction").text='SY(TOWERS03)'
        elif mutation=='effective':bad.findall("symbols/symbol[name='TOWERS01']")[-1].find('bitmap/pivot').set('x','5')
        else:bad.find("symbols/symbol/bitmap/graphics-location").attrib={'x':'980','y':'1160'}
        try:tower.relocate(ET.tostring(bad,encoding='unicode'))
        except AssertionError:check(True)
        else:raise AssertionError('Accepted source/slot drift')
    if baseline:
        base=ET.parse(baseline/'chartsymbols.xml').getroot();restored=copy.deepcopy(current)
        tower.restore_for_validation(stock,restored);check(canonical(restored)==canonical(base))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);p.add_argument('--baseline',type=Path)
    a=p.parse_args();checks=[]
    def check(v):assert v, f'Tower check {len(checks)+1}';checks.append(True)
    verify(a.source,a.generated,check,a.baseline)
    print(len(checks),'light-support tower resource checks passed; native/boat acceptance open')
