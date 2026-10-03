"""Exact SCRUM-260 node/paint proof; no chart or renderer state substitution."""
import copy
import importlib.util
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


def verify_area_ink(source, output, data, check):
    root=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'tools'))
    spec=importlib.util.spec_from_file_location('area_generator',root/'tools/generate-xnav-chart-style.py')
    g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    original=(source/'chartsymbols.xml').read_bytes()
    styled=(output/'chartsymbols.xml').read_bytes()
    before,after=ET.fromstring(original),ET.fromstring(styled)
    g.validate_resource_changes(original,styled,data['palette'])
    ferry="line-styles/line-style[name='FERYRT01']"
    boundary="lookups/lookup[@id='25']"
    old='SY(CBLARE51);LS(DASH,2,CHMGD);CS(RESTRN01)'
    new='SY(CBLARE51);LS(DASH,2,XNARE);CS(RESTRN01)'
    check(before.find(ferry).attrib==after.find(ferry).attrib=={'RCID':'2019'})
    check(after.find(ferry+'/color-ref').text=='AXNARE')
    restored=copy.deepcopy(after.find(ferry));restored.find('color-ref').text='ACHMGD'
    check(ET.tostring(restored)==ET.tostring(before.find(ferry)))
    check(before.find(boundary).attrib==after.find(boundary).attrib=={'id':'25','RCID':'32061','name':'CBLARE'})
    check(before.find(boundary+'/instruction').text==old and after.find(boundary+'/instruction').text==new)
    restored=copy.deepcopy(after.find(boundary));restored.find('instruction').text=old
    check(ET.tostring(restored)==ET.tostring(before.find(boundary)))
    # Literal expected roles come independently from the immutable final CSS.
    html=(root/'docs/design/prototype/index.html').read_text()
    for table,selector in [('DAY_BRIGHT','#app'),('DUSK','#app[data-theme=dusk]'),('NIGHT','#app[data-theme=night]')]:
        css=re.search(re.escape(selector)+r'\{--mark-red:[^}]+',html)[0]
        hue=re.search(r'--mark-area:#([0-9a-f]{6})',css)[1]
        rgb=tuple(int(hue[i:i+2],16) for i in (0,2,4))
        if table=='NIGHT':
            factor=float(re.search(r'\#app\[data-theme=night\] \.chart-canvas\{filter:brightness\(([^)]+)\)',html)[1])
            rgb=tuple(round(c*factor) for c in rgb)
        check(tuple(data['palette'][table]['XNARE'])==rgb)
        path="color-tables/color-table[@name='"+table+"']/color[@name='CHMGD']"
        check(ET.tostring(before.find(path))==ET.tostring(after.find(path)))
    def reject(change):
        tree=ET.fromstring(styled);change(tree)
        try:g.validate_resource_changes(original,ET.tostring(tree),data['palette'])
        except AssertionError:check(True)
        else:raise AssertionError('Area paint exception accepted unrelated change')
    for instruction in (new.replace('DASH,2','DASH,1'),new.replace('DASH','SOLD'),
                        new.replace(';CS(RESTRN01)',''),new.replace('CBLARE51','CHINFO07')):
        reject(lambda t,value=instruction:setattr(t.find(boundary+'/instruction'),'text',value))
    reject(lambda t:setattr(t.find(boundary+'/table-name'),'text','Symbolized'))
    reject(lambda t:t.find('lookups').append(copy.deepcopy(t.find(boundary))))
    reject(lambda t:setattr(t.find("lookups/lookup[@id='365']/instruction"),'text',new))
    reject(lambda t:setattr(t.find("lookups/lookup[@id='41']/instruction"),'text','LS(DASH,1,XNARE);CS(RESTRN01)'))
    reject(lambda t:setattr(t.find(ferry+'/HPGL'),'text',t.find(ferry+'/HPGL').text.replace('SW1','SW2')))
    reject(lambda t:t.find(ferry+'/vector/pivot').set('x','0'))
    reject(lambda t:t.find(ferry).set('RCID','2020'))
    reject(lambda t:t.find('line-styles').append(copy.deepcopy(t.find(ferry))))
    reject(lambda t:setattr(t.find("line-styles/line-style[name='FERYRT02']/color-ref"),'text','AXNARE'))
    reject(lambda t:setattr(t.find("line-styles/line-style[name='CBLARE51']/color-ref"),'text','AXNARE'))
    reject(lambda t:t.find("color-tables/color-table/color[@name='CHMGD']").set('r','156'))
    color="color-tables/color-table/color[@name='XNARE']"
    reject(lambda t:t.find(color).set('a','.16'))
    reject(lambda t:t.find(color).set('r','0'))
    reject(lambda t:t.find('color-tables/color-table').remove(t.find(color)))
    reject(lambda t:t.find('color-tables/color-table').append(copy.deepcopy(t.find(color))))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--generated',type=Path,required=True)
    args=parser.parse_args()
    count=0
    def check(value):
        global count
        assert value
        count+=1
    verify_area_ink(args.source,args.generated,json.loads((args.generated/'manifest.json').read_text()),check)
    print(f'{count} exact area/ferry paint checks passed; real SW/GL and native review remain required')
