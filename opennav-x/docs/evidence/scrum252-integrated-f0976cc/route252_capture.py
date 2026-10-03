"""Extra read-only paint evidence inside the existing actual route scenario."""
import collections,hashlib,json,os,subprocess,time
from pathlib import Path
from PIL import Image
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
            if i!=1:
                assert hits>=180,('Eligible actual route label background missing',light,names[i],hits,fill)
            else:
                assert hits<30,('Active special waypoint incorrectly gained floating label',light,hits)
                text=image.crop((x-10,y+8,x+65,y+27))
                black=collections.Counter(text.getdata())[(0,0,0)]
                assert black>=5,('Stock named active waypoint ink missing',light,black)
            probes.append({'point_index':i,'actual_name_from_pinned_fixture':names[i],
                           'actual_projected_pixels':p,'active_stock_fallback':i==1,
                           'floating_fill':fill,'exact_fill_pixels':hits,
                           'stock_black_pixels':black if i==1 else None})
        assert sum(not p.get('active_stock_fallback',True) for p in probes)==1, 'One eligible visible label required'
        assert sum(p.get('active_stock_fallback',False) for p in probes)==1, 'Named active stock control required'
        (evidence/f'{prefix}-{name}.json').write_text(json.dumps(d,indent=2)+'\n')
        records.append({'name':name,'png_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                        'route_record':entry,'labels':probes,'light':light})
        if index<3:
            button=next(b for b in d['runtime']['display']['interaction_controls']
                        if b['label']==light and b['visible'] and b['enabled'])
            subprocess.run(['xdotool','mousemove',str(button['x']+button['width']//2),
                            str(button['y']+button['height']//2),'click','1'],env=env,check=True)
    report['route_label_cycle']=records
    report['route_label_scope']='Real scenario-owned SIM names; active SIM 2 stock control; no custom editing or pointer/model injection'

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
