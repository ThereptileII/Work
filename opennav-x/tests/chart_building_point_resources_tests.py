"""Exact generic-building alias inverse/negative proof; not canvas acceptance."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from chart_raster_ink import decode
from chart_seamark_art import canonical
import chart_building_point as building
TABLES=(('DAY_BRIGHT','day',(124,133,138),(83,100,95)),('DUSK','dusk',(168,187,183),(195,206,194)),('NIGHT','dark',(98,115,108),(107,119,109)))


def tile(pixels,x,y):return b''.join(pixels[((y+j)*1500+x)*4:((y+j)*1500+x+9)*4] for j in range(9))


def expected_tile(before,table):
    # Independent literal target inks/formula. verify() separately derives every
    # recipe weight directly from the pinned Day bytes, without the helper.
    data=json.loads((ROOT/'resources/chart-style/v1/building-point/recipe.json').read_text())
    weights=sum(data['fillNumerators'],[]);alpha=bytes.fromhex(''.join(data['alphaRows']))
    _,_,fill,outline=next(t for t in TABLES if t[0]==table)
    return bytes(v for n,a in zip(weights,alpha) for v in (*[(o*3969+n*(f-o)+1984)//3969 for f,o in zip(fill,outline)],a))


def restore_building(before,after):
    actual=tile(after,788,1160)
    matches=[t[0] for t in TABLES if actual==expected_tile(before,t[0])]
    assert len(matches)==1, 'Building alias pixels/alpha differ'
    for y in range(1160,1169):
        i=(y*1500+788)*4
        assert not any(before[i+3:i+36:4]), 'Original alias slot was not empty'
        after[i:i+36]=before[i:i+36]


def verify(source,generated,check,baseline=None):
    before=ET.parse(source/'chartsymbols.xml').getroot();after=ET.parse(generated/'chartsymbols.xml').getroot()
    colors=json.loads((generated/'manifest.json').read_text())['palette']
    original=before.find("lookups/lookup[@id='1091']");selected=after.find("lookups/lookup[@id='1091']")
    check(original.attrib=={'id':'1091','RCID':'31143','name':'BUISGL'})
    check(original.findtext('instruction')=='SY(BUISGL01)' and selected.findtext('instruction')=='SY(XNBLDG01)')
    restored=copy.deepcopy(selected);restored.find('instruction').text='SY(BUISGL01)';check(canonical(restored)==canonical(original))
    for old in before.findall("lookups/lookup[@name='BUISGL']"):
        if old.get('id')!='1091' and old.findtext('type')=='Point':
            check(canonical(old)==canonical(after.find("lookups/lookup[@id='"+old.get('id')+"']")))
    stock=before.find("symbols/symbol[name='BUISGL01']");alias=after.find("symbols/symbol[name='XNBLDG01']")
    check(alias is not None and alias.get('RCID')=='60016')
    check(alias.find('bitmap').attrib=={'width':'9','height':'9'})
    check(alias.find('bitmap/pivot').attrib=={'x':'4','y':'4'})
    check(alias.find('bitmap/graphics-location').attrib=={'x':'788','y':'1160'})
    check(alias.findtext('color-ref')=='WXNBLOKXNBLF')
    # Every HPGL/vector attribute and pen letter remains exact; only color names
    # change, so vector fallback uses the same two palette colors as the raster.
    check(canonical(alias.find('vector'))==canonical(stock.find('vector')))
    check(alias.findtext('definition')==stock.findtext('definition')=='V')
    for name in ('BUISGL01','BUISGL11'):
        check(canonical(before.find("symbols/symbol[name='"+name+"']"))==canonical(after.find("symbols/symbol[name='"+name+"']")))
    recipe=json.loads((ROOT/'resources/chart-style/v1/building-point/recipe.json').read_text())
    _,day=decode((source/'rastersymbols-day.png').read_bytes());day_tile=tile(day,459,78)
    weights=sum(recipe['fillNumerators'],[])
    projected=[sum((day_tile[i+c]-o)*d for c,o,d in zip(range(3),(139,102,31),(38,43,26))) for i in range(0,324,4)]
    check(weights==projected and min(weights)==-376 and max(weights)==4521)
    for table,suffix,fill,outline in TABLES:
        check(tuple(colors[table]['XNBLF'])==fill and tuple(colors[table]['XNBLO'])==outline)
        for role in ('CHBRN','LANDF'):
            q="color-tables/color-table[@name='"+table+"']/color[@name='"+role+"']"
            check(canonical(before.find(q))==canonical(after.find(q)))
        _,old=decode((source/('rastersymbols-'+suffix+'.png')).read_bytes());_,new=decode((generated/('rastersymbols-'+suffix+'.png')).read_bytes())
        old_tile=tile(old,459,78);new_tile=tile(new,788,1160)
        check(tile(new,459,78)==old_tile)
        check(new_tile==expected_tile(old,table))
        check(new_tile[3::4]==old_tile[3::4]==day_tile[3::4] and len(new_tile)==324)
        if baseline:
            _,base=decode((baseline/('rastersymbols-'+suffix+'.png')).read_bytes());reverted=bytearray(new);restore_building(base,reverted);check(reverted==base)
        # Source RGB drift and single alpha changes must be refused, including
        # an opaque center and baked antialias edge: hashes cover all324 bytes.
        for offset in (0,3,160,163,323):
            damaged=bytearray(old_tile);damaged[offset]^=1
            try:building.tile_rgba(bytes(damaged),table,colors[table])
            except AssertionError:check(True)
            else:raise AssertionError('Accepted changed source tile')
        # Exercise the real paint guard at the tile and both moat corners;
        # replace only PNG decoding with already decoded pinned pixels.
        for x,y in ((788,1160),(786,1158),(798,1170)):
            occupied=bytearray(old);occupied[(y*1500+x)*4+3]=1
            with patch.object(building,'decode',return_value=([],occupied)):
                try:building.paint(b'',table,colors[table])
                except AssertionError as error:
                    check('not transparent' in str(error))
                else:raise AssertionError('Accepted occupied building tile/moat')
        bad=bytearray(new);bad[((1160+4)*1500+788+4)*4+3]^=1
        try:restore_building(old,bad)
        except AssertionError:check(True)
        else:raise AssertionError('Accepted changed alias alpha')
    spec=importlib.util.spec_from_file_location('generator',ROOT/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    xml=(source/'chartsymbols.xml').read_bytes();styled=(generated/'chartsymbols.xml').read_bytes();g.validate_resource_changes(xml,styled,colors);check(True)
    def reject(mutator):
        tree=ET.fromstring(styled);mutator(tree)
        try:g.validate_resource_changes(xml,ET.tostring(tree),colors)
        except AssertionError:check(True)
        else:raise AssertionError('Accepted building scope violation')
    q="lookups/lookup[@id='1091']"
    for name,value in [('type','Area'),('table-name','Paper'),('disp-prio','Area Symbol'),('display-cat','Standard'),('instruction','SY(BUISGL01)')]:
        reject(lambda t,n=name,v=value:setattr(t.find(q+'/'+n),'text',v))
    reject(lambda t:t.find(q).append(ET.fromstring('<attrib-code>CONVIS1</attrib-code>')))
    for identity in ('1090','2021','1080'):
        reject(lambda t,i=identity:setattr(t.find("lookups/lookup[@id='"+i+"']/instruction"),'text','SY(XNBLDG01)'))
    symbol="symbols/symbol[name='XNBLDG01']"
    reject(lambda t:t.find(symbol+'/bitmap/pivot').set('x','5'))
    reject(lambda t:t.find(symbol+'/bitmap').set('width','10'))
    reject(lambda t:setattr(t.find(symbol+'/vector/HPGL'),'text','SPK;CI200;'))
    reject(lambda t:setattr(t.find(symbol+'/color-ref'),'text','WXNBLFKXNBLO'))
    reject(lambda t:t.find('symbols').append(copy.deepcopy(t.find(symbol))))
    # Fail before generation if source selector or symbol changes, or an occupied
    # declaration is introduced into the proposed alias/moat.
    for mutation in ('class','selector','symbol','slot'):
        tree=ET.fromstring(xml)
        if mutation=='class':tree.find(q).set('name','LNDMRK')
        elif mutation=='selector':tree.find(q).append(ET.fromstring('<attrib-code>CONVIS1</attrib-code>'))
        elif mutation=='symbol':tree.find("symbols/symbol[name='BUISGL01']/bitmap/pivot").set('x','3')
        else:tree.find("symbols/symbol[name='BUISGL11']/bitmap/graphics-location").attrib={'x':'786','y':'1158'}
        try:building.relocate(ET.tostring(tree,encoding='unicode'))
        except AssertionError:check(True)
        else:raise AssertionError('Accepted source/slot drift')
    if baseline:
        base=ET.parse(baseline/'chartsymbols.xml').getroot();tree=copy.deepcopy(after)
        tree.find('symbols').remove(tree.find(symbol));tree.find(q+'/instruction').text='SY(BUISGL01)'
        for table,_,_,_ in TABLES:
            t=tree.find("color-tables/color-table[@name='"+table+"']")
            for role in ('XNBLF','XNBLO'):t.remove(t.find("color[@name='"+role+"']"))
        check(canonical(tree)==canonical(base))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);p.add_argument('--baseline',type=Path);a=p.parse_args();checks=[]
    def check(value):assert value;checks.append(True)
    verify(a.source,a.generated,check,a.baseline)
    print(len(checks),'focused building alias checks passed; native/canvas acceptance remains open')
