"""Independent curve/metadata/inverse checks; no native canvas qualification."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))

def verify_waveform_node(before, after, check):
    path="line-styles/line-style[name='CBLSUB06']"
    original=before.find(path);styled=after.find(path)
    hpgl=styled.findtext('HPGL')
    header=(ROOT/'src/integration/ChartCableWave.h').read_text()
    check(re.search(r'kCableWaveHpgl\[\] = "([^"]+)";',header)[1]==hpgl)
    check(hpgl.startswith('SPA;SW1;PU0,0;'))
    commands=hpgl.split(';')[:-1]
    check(len(commands)==67 and all(re.fullmatch(r'PD-?\d+,-?\d+',v) for v in commands[3:]))
    points=[tuple(map(int,re.findall(r'-?\d+',v))) for v in commands[2:]]
    check(len(points)==65 and points[-1]==(635,0))
    # Independent analytic evaluation at every vertex and dense interpolation:
    # each quadratic half-wave has linear x and alternating y=+/-10t(1-t).
    max_error=0
    for i,(x,y) in enumerate(points):
        phase=i/16;segment=min(3,int(phase));t=phase-segment
        true_x=635*phase/4
        true_y=635/24*((-1)**(segment+1))*10*t*(1-t)
        check(abs(x-true_x)<=.5 and abs(y-true_y)<=.5)
        check(0<=x<=635 and -84<=y<=84)
    for i in range(64):
        for j in range(33):
            u=j/32;phase=(i+u)/16;seg=min(3,int(phase));t=phase-seg
            exact=(635*phase/4,635/24*((-1)**(seg+1))*10*t*(1-t))
            actual=tuple(points[i][c]*(1-u)+points[i+1][c]*u for c in range(2))
            max_error=max(max_error,math.dist(exact,actual))
    # Quadratic chord bound5/(2*16^2)*635/24 + integer rounding sqrt(.5).
    bound=5/(2*16**2)*635/24+math.sqrt(.5)
    check(max_error<=bound<.966)
    restored=copy.deepcopy(styled);restored.find('HPGL').text=original.findtext('HPGL');restored.find('color-ref').text=original.findtext('color-ref')
    check(styled.find('vector').attrib=={'width':'635','height':'168'})
    check(styled.find('vector/pivot').attrib=={'x':'0','y':'0'})
    check(styled.find('vector/origin').attrib=={'x':'0','y':'-84'})
    for tag in ('vector','vector/pivot','vector/origin'):restored.find(tag).attrib=dict(original.find(tag).attrib)
    check(ET.tostring(restored)==ET.tostring(original))
    for key in ('709','710'):
        check(ET.tostring(before.find("lookups/lookup[@id='"+key+"']"))==ET.tostring(after.find("lookups/lookup[@id='"+key+"']")))
    return {'maximumMeasuredHpglError':max_error,'analyticBoundHpgl':bound,'points':points}

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('generator',ROOT/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    data=g.generate(args.source,args.output/'resources')
    raw=(args.source/'chartsymbols.xml').read_bytes();generated=(args.output/'resources/chartsymbols.xml').read_bytes()
    before,after=ET.fromstring(raw),ET.fromstring(generated)
    count=0
    def check(value):
        nonlocal count
        assert value
        count+=1
    proof=verify_waveform_node(before,after,check)
    g.validate_resource_changes(raw,generated,data['palette']);check(True)
    path="line-styles/line-style[name='CBLSUB06']"
    changes=[lambda t:setattr(t.find(path+'/HPGL'),'text',t.findtext(path+'/HPGL').replace('PU0,0','PU1,0',1)),
       lambda t:setattr(t.find(path+'/HPGL'),'text',t.findtext(path+'/HPGL').replace('SW1','SW2')),
       lambda t:t.find(path+'/vector').set('width','2294'),lambda t:t.find(path+'/vector/pivot').set('x','449'),
       lambda t:t.find(path+'/vector/origin').set('y','1051'),lambda t:t.find(path+'/HPGL').set('alpha','.16'),
       lambda t:setattr(t.find("lookups/lookup[@id='709']/instruction"),'text','LC(CBLSUB06)'),
       lambda t:setattr(t.find("lookups/lookup[@id='710']/table-name"),'text','Plain'),
       lambda t:setattr(t.find("line-styles/line-style[name='CBLARE51']/HPGL"),'text','SPA;SW1;'),
       lambda t:t.find('line-styles').append(copy.deepcopy(t.find(path)))]
    for change in changes:
        tree=ET.fromstring(generated);change(tree)
        try:g.validate_resource_changes(raw,ET.tostring(tree),data['palette'])
        except AssertionError:check(True)
        else:raise AssertionError('Unowned change accepted')
    # Generator refuses source drift before any geometry replacement.
    import chart_cable_paint as cable
    bad=ET.fromstring(raw);bad.find(path+'/vector').set('height','501')
    try:cable.recolor(ET.tostring(bad,encoding='unicode'))
    except AssertionError:check(True)
    else:raise AssertionError('Changed source accepted')
    ppmm=96/25.4
    proof.update({'checks':count,'palette':{k:v['XNCBL'] for k,v in data['palette'].items()},'representativePpmm':ppmm,
      'physicalWidthMm':6.35,'widthAt96dpi':6.35*ppmm,'modernRepeatAt96dpi':6.35*ppmm,'legacyRepeatAt96dpi':6.35*ppmm,
      'prototypeWidthCss':24,'strokeGap':'Resource fallback SW1; verified renderer exact quadratics/1.3 round stroke tested separately',
      'scope':'Resource/analytic proof only; no actual chart, GL or Windows acceptance'})
    (args.output/'proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    # Independent comparative SVG shows actual fixed scales, not fit-to-box.
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="840" height="420"><rect width="840" height="420" fill="white"/><g font-family="sans-serif" font-size="13" fill="#202020">',
         '<text x="20" y="25">Geometry proof only — 96 DPI / 3.7795 px/mm; native canvas still required</text>',
         '<text x="20" y="50">Prototype 24 CSS px</text><text x="225" y="50">Stock at native scale</text><text x="465" y="50">Derived at native scale (24 px)</text>']
    for row,theme in enumerate(('DAY_BRIGHT','DUSK','NIGHT')):
        y=100+row*100;rgb=data['palette'][theme]['XNCBL'];color='#'+''.join(f'{c:02x}' for c in rgb)
        svg += [f'<text x="20" y="{y+40}">{theme}</text>',f'<path d="M-12 0q3-5 6 0t6 0t6 0t6 0" transform="translate(70,{y})" fill="none" stroke="{color}" stroke-width="1.3" stroke-linecap="round"/>']
        stock=before.findtext(path+'/HPGL');parts=[]
        for command in stock.split(';'):
            if command.startswith(('PU','PD')):
                x,z=map(int,command[2:].split(','));parts.append(('M' if command[:2]=='PU' else 'L')+f'{225+(x-448)*ppmm/100:.5f},{y+(z-1274)*ppmm/100:.5f}')
        svg.append(f'<path d="{" ".join(parts)}" fill="none" stroke="{color}" stroke-width="1"/>')
        points=' '.join(f'{465+x*ppmm/100:.5f},{y+z*ppmm/100:.5f}' for x,z in proof['points'])
        svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1"/>')
    svg += ['<text x="20" y="410">Owned635-unit repeat; this fallback centerline SVG is not the actual fractional-stroke painter.</text></g></svg>']
    (args.output/'comparison.svg').write_text('\n'.join(svg))
    print(f'{count} focused cable checks passed; max centerline error {proof["maximumMeasuredHpglError"]:.6f} HPGL units')
if __name__=='__main__':main()
