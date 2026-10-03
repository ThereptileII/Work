"""Extra read-only paint evidence inside the existing actual route scenario."""
import collections,hashlib,json,os,subprocess,time,math
from pathlib import Path
from PIL import Image
from route_paint_probe import route_paint_probe
from diagnostic_snapshot import read_json_snapshot

def capture_cycle(profile,evidence,prefix,env,args,report,capture,result,app,phase):
    assert phase[0]=='rmc', 'Keep the real fixture input live during hot theme changes'
    entry=next(x for x in result['checks'] if x['check']=='restored route')
    assert entry['state']=='Valid' and entry['active_waypoint_index']==1,entry
    assert entry['active_waypoint_id']=='OPENNAV-TEST-point-2',entry
    points=entry['route_pixels'];assert len(points)==3
    # Scenario has reversed its real route: first=SIM 3, active=SIM 2, last=SIM 1.
    names=['SIM 3','SIM 2','SIM 1']
    records=[]
    def fresh(light):
        d=read_json_snapshot(profile/'opennav-diagnostics.json')
        values={v['name']:v for v in d['data']}
        valid=all(values[k]['quality']=='LIVE' and values[k]['validity']=='Measured' and
                  values[k]['age_ms']<2000 for k in ('Latitude','Longitude','Speed over ground','Course over ground'))
        scenario=read_json_snapshot(profile/'route-fixture-results.json')
        assert scenario['result']=='running' and scenario['phase']=='stop-input',scenario
        assert d['build_commit']==os.environ['SKAGER_ROUTE252_COMMIT']
        assert d['runtime']['chart']['opengl_enabled']==(args.renderer=='opengl')
        assert d['runtime']['chart_presentation']['requested']=='XNav'
        # Existing route scenario deliberately has no ENC directory; it exercises
        # verified SKAGER route paint over upstream GSHHS, not real ENC symbols.
        assert d['runtime']['chart_presentation']['status']=='SKAGER presentation v1 / verified palette; ENC not loaded'
        return d if valid and d['runtime']['display']['light']==light else None
    for index,light in enumerate(('Day','Dusk','Night','Day')):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            assert app.poll() is None,'Application exited during hot label capture'
            d=fresh(light)
            if d:break
            time.sleep(.1)
        else:raise AssertionError('Fresh theme publication did not arrive: '+light)
        time.sleep(.6);d=fresh(light);assert d,'Fresh input/theme lost before capture'
        name='labels-'+light.lower()+('-return' if index==3 else '')
        capture(name)
        path=evidence/f'{prefix}-{name}.png';image=Image.open(path).convert('RGB')
        assert image.size==(1280,800)
        fill={'Day':(247,248,240),'Dusk':(36,58,64),'Night':(16,26,32)}[light]
        probes=[]
        chart=d['runtime']['display']['chart_region']
        for i,p in enumerate(points):
            # Actual upstream projected screen pixels, never guessed route geometry.
            x,y=p['x'],p['y'];width=50;left=-64 if i==2 else -18
            expected=(x+left,y+18,x+left+width,y+43)
            assert all(0<=v<lim for v,lim in zip(expected,(1280,800,1280,800))),expected
            inside=(expected[0]>=chart['x'] and expected[1]>=chart['y'] and
                    expected[2]<=chart['x']+chart['width'] and expected[3]<=chart['y']+chart['height'])
            if not inside:
                probes.append({'point_index':i,'actual_name_from_pinned_fixture':names[i],
                    'actual_projected_pixels':p,'visible_label_in_chart':False,
                    'reason':'Original scenario arrival/reversal moved this point outside the unchanged chart viewport'})
                continue
            crop=image.crop((x-75,y-20,x+95,y+65))
            crop.save(evidence/f'{prefix}-{name}-point-{i+1}.png')
            body=image.crop((x+left+6,y+25,x+left+width-6,y+40))
            colors=collections.Counter(body.getdata());hits=colors[fill]
            assert hits>=180,('Actual default route label background missing',light,names[i],hits,fill)
            active_icon=None
            if i==1:
                active_icon=active_marker_probe(image,x,y,light)
            probes.append({'point_index':i,'actual_name_from_pinned_fixture':names[i],
                           'actual_projected_pixels':p,'active_stock_icon':i==1,
                           'floating_fill':fill,'exact_fill_pixels':hits,
                           'active_icon_probe':active_icon})
        assert sum('exact_fill_pixels' in p for p in probes)==2, 'Both actual visible default names require cards'
        assert sum(p.get('active_stock_icon',False) for p in probes)==1, 'Actual active icon distinction required'
        paint_probes=route_paint_probe(image,points,light)
        (evidence/f'{prefix}-{name}.json').write_text(json.dumps(d,indent=2)+'\n')
        records.append({'name':name,'png_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                        'route_record':entry,'labels':probes,'paint_probes':paint_probes,'light':light})
        if index==0:
            # Labels already stay visible during upstream icon blinking. Prove
            # both icon phases while retaining the same actual name card.
            phases={};deadline=time.monotonic()+3.5;sample=0
            while time.monotonic()<deadline and len(phases)<2:
                d=fresh(light);assert d,'Fresh Day input lost during icon-phase capture'
                phase_name='active-icon-phase-'+str(sample);capture(phase_name)
                phase_image=Image.open(evidence/f'{prefix}-{phase_name}.png').convert('RGB')
                x,y=points[1]['x'],points[1]['y']
                icon=active_marker_probe(phase_image,x,y,light)
                card=collections.Counter(phase_image.crop((x-12,y+25,x+26,y+40)).getdata())[fill]
                assert card>=180,('Active card disappeared during icon blink',card)
                phases[icon['red_pixels']>=5]={'image':phase_name,'icon':icon,'card_fill_pixels':card}
                sample+=1;time.sleep(.4)
            assert len(phases)==2,('Upstream active icon on/off phases not observed',phases)
            report['active_icon_blink']={'on':phases[True],'off':phases[False]}
        if index<3:
            button=next(b for b in d['runtime']['display']['interaction_controls']
                        if b['label']==light and b['visible'] and b['enabled'])
            subprocess.run(['xdotool','mousemove',str(button['x']+button['width']//2),
                            str(button['y']+button['height']//2),'click','1'],env=env,check=True)
    report['route_label_cycle']=records
    report['route_label_scope']='Real scenario-owned SIM names; default active SIM 2 card with unchanged stock blinking icon; no custom editing or pointer/model injection'

def capture_stale(profile,evidence,prefix,report):
    d=read_json_snapshot(profile/'opennav-diagnostics.json')
    path=evidence/f'{prefix}-02-stale-position.png'
    image=Image.open(path).convert('RGB')
    controls=[b for b in d['runtime']['display']['interaction_controls'] if b.get('visible')]
    report['route_label_stale']={'diagnostics':f'{prefix}-labels-stale.json',
        'visible_controls':controls,'png_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'review_required':'Inspect repaired floating chart controls over stale chart; source health/rail assertions stay in original collector'}
    (evidence/f'{prefix}-labels-stale.json').write_text(json.dumps(d,indent=2)+'\n')
    # Full canvas plus a broad lower-right floating-controls crop, using the
    # owned chart rectangle rather than hard-coded fake waypoint positions.
    r=d['runtime']['display']['chart_region'];x,y,w,h=[r[k] for k in ('x','y','width','height')]
    image.crop((max(x,x+w-450),max(y,y+h-220),x+w,y+h)).save(evidence/f'{prefix}-stale-floating-controls.png')


def active_marker_probe(image,x,y,light):
    ink={'Day':(38,124,118),'Dusk':(176,223,200),'Night':(113,147,126)}[light]
    red=ring=0;sectors=[0]*8
    for py in range(y-12,y+13):
        for px in range(x-12,x+13):
            r,g,b=image.getpixel((px,py))
            red+=r>1.4*g and r>1.4*b and r-max(g,b)>20
            if 9<=math.hypot(px-x,py-y)<=11:
                matched=all(abs(a-c)<=1 for a,c in zip((r,g,b),ink))
                ring+=matched
                if matched:sectors[int((math.atan2(py-y,px-x)+math.pi)*8/(2*math.pi))%8]+=1
    # A numbered circle covers the annulus around the point. Upstream arrows
    # can add many ink pixels in a few sectors, so count angular coverage.
    covered=sum(n>=3 for n in sectors)
    assert covered<6,('Active icon replaced by numbered route circle',light,sectors)
    return {'red_pixels':red,'numbered_ring_ink_pixels':ring,'ring_sectors':sectors}
