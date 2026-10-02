"""One pinned submarine-cable symbol paint role; no HPGL/layout changes."""
import re
import xml.etree.ElementTree as ET

COLOR='XNCBL'
NAME='CBLSUB06'
RCID='2012'

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
    return re.sub(pattern,lambda m:m[1]+m[2].replace(
        '<color-ref>ACHMGD</color-ref>','<color-ref>AXNCBL</color-ref>')+m[3],xml,flags=re.S)

def restore_for_validation(before, after):
    stock,styled=node(before),node(after)
    assert stock.findtext('color-ref')=='ACHMGD'
    paint=styled.find('color-ref')
    assert paint.text=='AXNCBL' and not paint.attrib and len(paint)==0, 'Unexpected cable paint mutation'
    paint.text=stock.findtext('color-ref')
    # The caller then compares the entire restored resource tree, including
    # HPGL, SW widths, origin/pivot, lookup order/category, and all other nodes.
