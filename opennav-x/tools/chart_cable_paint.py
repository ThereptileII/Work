"""Pinned cable/ferry/area paint roles; no HPGL/layout changes."""
import re
import xml.etree.ElementTree as ET

COLOR='XNCBL'
NAME='CBLSUB06'
RCID='2012'
AREA_COLOR='XNARE'
AREA_ORIGINAL='SY(CBLARE51);LS(DASH,2,CHMGD);CS(RESTRN01)'
AREA_STYLED='SY(CBLARE51);LS(DASH,2,XNARE);CS(RESTRN01)'

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
    original=node(ET.fromstring(xml))
    assert original.findtext('color-ref')=='ACHMGD', 'Pinned cable paint changed'
    pattern=r'(<line-style RCID="2012">)(.*?)(</line-style>)'
    matches=list(re.finditer(pattern,xml,re.S))
    assert len(matches)==1 and '<name>CBLSUB06</name>' in matches[0][2]
    assert matches[0][2].count('<color-ref>ACHMGD</color-ref>')==1
    xml=re.sub(pattern,lambda m:m[1]+m[2].replace(
        '<color-ref>ACHMGD</color-ref>','<color-ref>AXNCBL</color-ref>')+m[3],xml,flags=re.S)
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
    stock,styled=node(before),node(after)
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
    # HPGL, SW widths, origin/pivot, lookup order/category, and all other nodes.
