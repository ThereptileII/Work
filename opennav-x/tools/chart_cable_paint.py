"""Pinned cable waveform and ferry/area paint; only the cable motif owns new physical metadata."""
import re
import hashlib
from fractions import Fraction
from pathlib import Path
from chart_seamark_art import canonical
import xml.etree.ElementTree as ET

COLOR='XNCBL'
NAME='CBLSUB06'
RCID='2012'
AREA_COLOR='XNARE'
AREA_ORIGINAL='SY(CBLARE51);LS(DASH,2,CHMGD);CS(RESTRN01)'
AREA_STYLED='SY(CBLARE51);LS(DASH,2,XNARE);CS(RESTRN01)'

# Source-locked SCRUM-281 centerline increment. Integer HPGL strokes are NOT
# equivalent to the prototype stroke. The verified renderer hook paints the
# exact curve; this approximation is its bounded resource fallback.
WAVE_PATH = 'M-12 0q3-5 6 0t6 0t6 0t6 0'
ART_SHA256 = 'be2c7f44817c29d3714412c7974d3cf07e238f65c7131958b0cb8ff6fbd7816d'
NODE_SHA256 = '60b0c817180a355b16eb37b037258aadd7a760de34fdb28ba578825c265b1db9'
LOOKUP_SHA256 = {'710':'f66bceaa1e6193dfb007061f0104b457fe17d7998a36554b60d3061ba12fd6a4',
                 '709':'4a99d275fc242fc36b7afacfa24d01785968aede8423974f6c91a738812f99a9'}

def waveform_points():
    # One24CSS-unit motif is6.35mm at nominal96DPI:635 HPGL units.
    # Original navigation geometry is separate. Owned origin/pivot x0 yields
    # the same635 repeat in modern and legacy formulas; density is explicit.
    scale=Fraction(635,24)
    points=[]
    for segment in range(4):
        for step in range(17):
            if segment and step==0: continue
            t=Fraction(step,16)
            x=scale*(6*segment+6*t)
            y=scale*((-10 if segment%2==0 else 10)*t*(1-t))
            points.append((round(x),round(y)))
    return points

def waveform_hpgl():
    points=waveform_points()
    return 'SPA;SW1;PU%d,%d;'%points[0]+''.join('PD%d,%d;'%p for p in points[1:])

def waveform_source(tree):
    art=(Path(__file__).resolve().parents[1]/'docs/design/prototype/src/chart-marker-art.js').read_bytes().replace(b'\r\n',b'\n')
    assert hashlib.sha256(art).hexdigest()==ART_SHA256, 'Supplied cable artwork changed'
    assert ("'line:CBLSUB06':'<path d=\""+WAVE_PATH+"\"/>'").encode() in art
    result=node(tree)
    assert hashlib.sha256(canonical(result)).hexdigest()==NODE_SHA256, 'Pinned cable definition changed'
    for key,expected in LOOKUP_SHA256.items():
        matches=tree.findall("lookups/lookup[@id='"+key+"']")
        assert len(matches)==1 and hashlib.sha256(canonical(matches[0])).hexdigest()==expected, 'Pinned cable lookup changed'
    return result


def area_instruction(lookup):
    instruction=lookup.findtext('instruction')
    if lookup.get('id')=='25':
        assert lookup.attrib=={'id':'25','RCID':'32061','name':'CBLARE'}
        assert lookup.findtext('type')=='Area' and lookup.findtext('table-name')=='Plain'
        assert instruction==AREA_ORIGINAL, 'Pinned cable-area instruction changed'
        return AREA_STYLED
    return instruction

def ferry_node(tree):
    matches=tree.findall("line-styles/line-style[name='FERYRT01']")
    assert len(matches)==1 and matches[0].attrib=={'RCID':'2019'}, 'Pinned ferry node changed'
    assert len(matches[0].findall('color-ref'))==1
    return matches[0]


def theme_ink(table, rgb):
    # Immutable index.html:15 dims the complete Night chart canvas. Like the
    # isolated ACHARE51 ink, account for that only in this owned symbol role.
    return tuple(round(channel*.78) for channel in rgb) if table=='NIGHT' else rgb


def node(tree):
    matches=tree.findall("line-styles/line-style[name='CBLSUB06']")
    assert len(matches)==1, 'Missing or duplicated submarine-cable line style'
    result=matches[0]
    assert result.attrib=={'RCID':RCID}, 'Unexpected cable style identity'
    assert len(result.findall('color-ref'))==1, 'Unexpected cable color references'
    return result

def recolor(xml):
    original=waveform_source(ET.fromstring(xml))
    assert original.findtext('color-ref')=='ACHMGD', 'Pinned cable paint changed'
    pattern=r'(<line-style RCID="2012">)(.*?)(</line-style>)'
    matches=list(re.finditer(pattern,xml,re.S))
    assert len(matches)==1 and '<name>CBLSUB06</name>' in matches[0][2]
    assert matches[0][2].count('<color-ref>ACHMGD</color-ref>')==1
    xml=re.sub(pattern,lambda m:m[1]+m[2].replace(
        '<color-ref>ACHMGD</color-ref>','<color-ref>AXNCBL</color-ref>').replace(
        '<HPGL>'+original.findtext('HPGL')+'</HPGL>', '<HPGL>'+waveform_hpgl()+'</HPGL>').replace(
        '<vector width="2293" height="500">', '<vector width="635" height="168">').replace(
        '<pivot x="448" y="1274" />', '<pivot x="0" y="0" />').replace(
        '<origin x="692" y="1050" />', '<origin x="0" y="-84" />')+m[3],xml,flags=re.S)
    tree=ET.fromstring(xml)
    assert ferry_node(tree).findtext('color-ref')=='ACHMGD'
    lookups=tree.findall("lookups/lookup[@id='25']")
    assert len(lookups)==1 and area_instruction(lookups[0])==AREA_STYLED
    changes=[(r'(<line-style RCID="2019">)(.*?)(</line-style>)',
              '<color-ref>ACHMGD</color-ref>','<color-ref>AXNARE</color-ref>'),
             (r'(<lookup id="25" RCID="32061" name="CBLARE">)(.*?)(</lookup>)',
              '<instruction>'+AREA_ORIGINAL+'</instruction>',
              '<instruction>'+AREA_STYLED+'</instruction>')]
    for pattern,before,after in changes:
        matches=list(re.finditer(pattern,xml,re.S))
        assert len(matches)==1 and matches[0][2].count(before)==1
        xml=re.sub(pattern,lambda m:m[1]+m[2].replace(before,after)+m[3],xml,flags=re.S)
    return xml

def restore_for_validation(before, after):
    stock,styled=waveform_source(before),node(after)
    hpgl=styled.find('HPGL')
    assert hpgl is not None and hpgl.text==waveform_hpgl() and not hpgl.attrib and len(hpgl)==0, 'Unexpected cable waveform mutation'
    hpgl.text=stock.findtext('HPGL')
    vector=styled.find('vector')
    assert vector.attrib=={'width':'635','height':'168'}
    assert vector.find('pivot').attrib=={'x':'0','y':'0'}
    assert vector.find('origin').attrib=={'x':'0','y':'-84'}
    vector.attrib=dict(stock.find('vector').attrib)
    for tag in ('pivot','origin'):vector.find(tag).attrib=dict(stock.find('vector/'+tag).attrib)
    assert stock.findtext('color-ref')=='ACHMGD'
    paint=styled.find('color-ref')
    assert paint.text=='AXNCBL' and not paint.attrib and len(paint)==0, 'Unexpected cable paint mutation'
    paint.text=stock.findtext('color-ref')
    stock,styled=ferry_node(before),ferry_node(after)
    paint=styled.find('color-ref')
    assert stock.findtext('color-ref')=='ACHMGD'
    assert paint.text=='AXNARE' and not paint.attrib and len(paint)==0, 'Unexpected ferry paint mutation'
    paint.text=stock.findtext('color-ref')
    # The caller then compares the entire restored resource tree, including
    # restored exact HPGL, origin/pivot, lookup order/category, and all other nodes.
