#!/usr/bin/env python3
"""Official IHO S-64 presentation-test scenes; no operational-chart acceptance."""
import argparse,collections,datetime,hashlib,importlib.util,json,os,pathlib,re,shutil,socket,subprocess,sys,tempfile,threading,time
import xml.etree.ElementTree as ET
from PIL import Image, ImageChops
collector=pathlib.Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('phase')
parser.add_argument('--scene',required=True)
parser.add_argument('--display',default=':197')
parser.add_argument('--renderer',choices=('software','opengl'),required=True)
parser.add_argument('--expected-commit',required=True)
parser.add_argument('--expected-exe-sha256',required=True)
parser.add_argument('--expected-manifest-sha256',required=True)
args=parser.parse_args()
root=collector/'inputs'
spec=importlib.util.spec_from_file_location('cache_inputs',collector/'cache-inputs-readonly.py')
i=importlib.util.module_from_spec(spec);spec.loader.exec_module(i)
frozen=i.verify()
i.require(re.fullmatch('[a-f0-9]{40}',args.expected_commit) and frozen['commit']==args.expected_commit,'Explicit final source identity differs')
for digest in (args.expected_exe_sha256,args.expected_manifest_sha256):
    i.require(re.fullmatch('[a-f0-9]{64}',digest),'Explicit final SHA256 required')
i.require(re.fullmatch('[a-zA-Z0-9_-]+',args.phase) is not None,'Use a simple new evidence directory name')
i.require(re.fullmatch(':[0-9]+',args.display) is not None,'Use an isolated X display number')
evidence=collector/'output'/('capture-'+args.phase);evidence.mkdir(exist_ok=False)
profile_root=pathlib.Path(tempfile.mkdtemp(prefix='skager-capture-'))
profile_labels=[]
env=dict(os.environ,PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/bin:'+os.environ['PATH'],GDK_BACKEND='x11',LIBGL_ALWAYS_SOFTWARE='1',DISPLAY=args.display,LD_LIBRARY_PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/lib')
staged=json.loads((root/'staged-inputs.json').read_text());exe=root/'install/bin/opencpn'
i.require(staged['commit']==frozen['commit']==args.expected_commit and hashlib.sha256(exe.read_bytes()).hexdigest()==staged['binary_sha256']==args.expected_exe_sha256,'Staged executable identity differs')
resource_root=root/'install/share/opencpn/opennav/chart-style/v1'
i.require(hashlib.sha256((resource_root/'manifest.json').read_bytes()).hexdigest()==staged['manifest_sha256']==args.expected_manifest_sha256,'Staged manifest differs')
for name,identity in staged['resource_files'].items():
    raw=(resource_root/name).read_bytes();i.require(len(raw)==identity['bytes'] and hashlib.sha256(raw).hexdigest()==identity['sha256'],'Staged resource differs: '+name)
manifest=json.loads((resource_root/'manifest.json').read_text())
stock_lock=json.loads((root/'app/resources/chart-style/v1/source-lock.json').read_text())
for filename,identity in stock_lock['files'].items():
    content=(root/'install/share/opencpn/s57data'/filename).read_bytes()
    i.require(len(content)==identity['bytes'] and hashlib.sha256(content).hexdigest()==identity['sha256'],'Standard resource differs from exact pinned source: '+filename)
stock=ET.parse(root/'install/share/opencpn/s57data/chartsymbols.xml').getroot()
standard={table.get('name'):{c.get('name'):tuple(int(c.get(k)) for k in ('r','g','b')) for c in table.findall('color')} for table in stock.find('color-tables')}
chart_root=collector/('seattle-fixture' if args.scene=='seattle' else 'fixture')
scene_data=json.loads((collector/'scenes.json').read_text())
scene_data['scenes']=[s for s in scene_data['scenes'] if s['id']==args.scene]
i.require(args.scene=='s64-light-fog','Standard control is limited to original light/fog scene')
for s in scene_data['scenes']:s['features']=[f for f in s['features'] if f['class']!='BUISGL']
i.require(len(scene_data['scenes'])==1,'One explicit source scene required')
chart_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in chart_root.iterdir() if p.is_file()}
expected_files=scene_data['source_files']['US5SEAFL' if args.scene=='seattle' else 'GB4X0000']
i.require(chart_files=={k:v['sha256'] for k,v in expected_files.items()},'Exact public chart bytes differ')
report={**staged,'authority':'Linux Xvfb '+args.renderer+' development visuals only; not native Windows, production or boat acceptance', 'renderer_requested':args.renderer,
        'scope':('NOAA real-world ENC' if args.scene=='seattle' else 'Official IHO S-64 presentation-test geometry, not operational nautical ENC')+'; explicit source scene; controlled simulated loopback RMC; no Demo or injected chart objects',
        'scene_source':scene_data,'chart_source_files':chart_files,'captures':[],'status':'running','clean_exit':False,
        'font_evidence':{'policy_source_sha256':staged['source_evidence']['src/integration/ChartPresentation.cpp'],
                         'note':'Source policy and Linux fontconfig inventory recorded; this does not establish native Windows glyph metrics'}}
report['additional_source_evidence']={name:hashlib.sha256((root/'app'/name).read_bytes()).hexdigest() for name in ('src/ui/SkagerWordmark.h','src/ui/Theme.h','src/ui/Shell.cpp','src/integration/ChartCanvasInk.h','src/integration/ChartRouteLabel.cpp','src/integration/ChartRouteLabelRaster.h','tools/chart_cardinal_art.py','resources/chart-style/v1/cardinals/provenance.json')}
for family in ('Segoe UI Variable Display','Segoe UI','Arial'):
    probe=subprocess.run(['fc-match','-f','%{family}\n%{file}\n',family],env=env,text=True,capture_output=True)
    report['font_evidence'][family]={'exit_code':probe.returncode,'linux_fontconfig_match':probe.stdout.strip()}
xserver=None;app=None;log=None
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1);server.settimeout(.2)
stop=threading.Event();errors=[];sent=[];connections=[]
def sentence(body):
    checksum=0
    for c in body.encode():checksum^=c
    return f'${body}*{checksum:02X}\r\n'.encode()
def transmit():
    while not stop.is_set():
        try:peer,address=server.accept()
        except TimeoutError:continue
        except OSError:return
        try:
            i.require(address[0]=='127.0.0.1','Unexpected input client');peer.settimeout(2)
            connections.append({'peer':address[0],'connected_monotonic':time.monotonic()})
            while not stop.wait(.3):
                now=datetime.datetime.now(datetime.timezone.utc)
                raw=sentence(f'GPRMC,{now:%H%M%S},A,4736.000,N,12221.600,W,3.0,90.0,{now:%d%m%y},,,A')
                peer.sendall(raw);sent.append({'sent_monotonic':time.monotonic(),'sentence':raw.decode().strip()})
        except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
        except Exception as e:
            if not stop.is_set():errors.append(str(e))
        finally:peer.close()
thread=threading.Thread(target=transmit,daemon=True);thread.start()
def wait(fn,seconds=45):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if app and app.poll() is not None:raise AssertionError(('app exited',app.returncode))
        if xserver and xserver.poll() is not None:raise AssertionError(('X server exited',xserver.returncode))
        try:
            result=fn()
            if result:return result
        except (FileNotFoundError,json.JSONDecodeError,KeyError):pass
        time.sleep(.15)
    raise AssertionError('Bounded chart wait expired')
def xdo(*a):return subprocess.check_output(['xdotool',*map(str,a)],env=env,text=True).strip()
def edge_is_wrapped(top,bottom):
    i.require(len(top)==len(bottom) and len(top)>0,'Edge probe sizes differ')
    return sum(a==b for a,b in zip(top,bottom))==len(top)

def trace_records(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []

def trace_done(path):
    records=trace_records(path)
    failures=[r for r in records if r['kind']=='trace_error']
    if failures:raise RuntimeError('Read-only renderer trace: '+json.dumps(failures))
    return len([r for r in records if r['kind']=='trace_disabled'])==3

def check_trace(records,scene,style):
    i.require(not any(r['kind']=='trace_error' for r in records),'Read-only trace failed')
    done=[r for r in records if r['kind']=='trace_disabled']
    i.require(len(done)==3 and all(r['reason']=='all_requested_geometry_observed' for r in done),'Bounded renderer trace did not observe requested geometry')
    draws=[r for r in records if r['kind']=='RenderSY'];arcs=[r for r in records if r['kind']=='RenderCARC'];raster=[r for r in records if r['kind']=='RenderRasterSymbol']
    if not scene['expectedCA']:
        proof=[]
        for f in scene['features']:
            a=f['attributes'];selected=[r for r in draws if r['class']==f['class'] and abs(r['latitude']-a['latitude'])<1e-6 and abs(r['longitude']-a['longitude'])<1e-6]
            painted=[r for r in raster if r['class']==f['class'] and abs(r['latitude']-a['latitude'])<1e-6 and abs(r['longitude']-a['longitude'])<1e-6]
            i.require(selected and painted,'Actual conditional point draw missing')
            i.require(all(r['effective_symbol_table']==76 and r['lookup_table']==76 for r in selected+painted),'Actual hazard table differs')
            proof.append({'source_rcid':a['RCID'],'class':f['class'],'candidate':f['candidateRaster'],'selected_symbols':sorted({r['symbol'] for r in selected}),'painted_symbols':sorted({r['symbol'] for r in painted}),'candidate_reached':any(r['symbol']==f['candidateRaster'] for r in painted)})
        return {'conditional_selection':proof,'read_only_trace_complete':True,'draws':draws,'arc_draws':arcs,'raster_draws':raster}
    i.require(arcs and all(r['class']=='LIGHTS' and r['lookup_rcid']==31183 and r['lookup_table']==76 and r['effective_symbol_table']==76 for r in arcs),'Actual LIGHTS CA/table differs')
    if any(f['class']=='FOGSIG' for f in scene['features']):
        i.require(draws and all(r['class']=='FOGSIG' and r['symbol']=='FOGSIG01' and r['lookup_rcid']==31164 for r in draws),'Original fog lookup missing')
    actual_ca={(r['instruction'].split(',')[2].strip(),float(r['instruction'].split(',')[4]),float(r['instruction'].split(',')[5])) for r in arcs}
    i.require(actual_ca=={tuple(v) for v in scene['expectedCA']},'Actual individual source sectors differ')
    for feature in scene['features']:
        if style=='Standard' and feature['class']=='LIGHTS':continue
        attr=feature['attributes'];actual=[r for r in raster if r['class']==feature['class'] and abs(r['latitude']-attr['latitude'])<1e-6 and abs(r['longitude']-attr['longitude'])<1e-6]
        i.require(actual and all(r['symbol']==feature['expectedRaster'] and r['definition']==82 and r['effective_symbol_table']==76 and r['lookup_table']==76 for r in actual),'Actual light/fog raster/table differs for '+feature['class'])
        if feature['class']=='BUISGL':
            i.require(all(r['lookup_rcid']==31143 for r in actual),'Actual generic-building lookup differs')
    fan_proof=[r for r in records if r['kind'] in ('fan_build','fan_gl_draw')]
    if style=='XNav' and len(scene['expectedCA'])==3:
        for kind in ['fan_build']+(['fan_gl_draw'] if args.renderer=='opengl' else []):
            results=[r for r in fan_proof if r['kind']==kind]
            i.require(len(results)==3 and all(r['result']==1 for r in results),'Actual compact fan path failed: '+kind)
    return {'fan_proof':fan_proof,'read_only_trace_complete':True,'debugger_timing_note':'Entry probes disabled before settled snapshots; no inferior calls or model writes. CA and fog follow normal renderer entry; added light point reached actual raster painter.','draws':draws,'arc_draws':arcs,'raster_draws':raster}

try:
    i.require(not pathlib.Path('/tmp/.X'+args.display[1:]+'-lock').exists(),'Display lock already exists')
    xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    wait(lambda:subprocess.run(['xdotool','getdisplaygeometry'],env=env,capture_output=True).returncode==0,10)
    if args.renderer=='opengl':
        info=subprocess.run(['glxinfo','-B'],env=env,text=True,capture_output=True,timeout=20)
        (evidence/'mesa-renderer.txt').write_text(info.stdout+info.stderr)
        i.require(info.returncode==0 and 'llvmpipe' in info.stdout,'Expected isolated Mesa llvmpipe renderer')
        report['mesa_renderer']=info.stdout
    selftest=evidence/'loader-selftest.json'
    subprocess.run([str(exe),'--opennav-self-test',str(selftest)],env=env,check=True,capture_output=True,timeout=20)
    loader=json.loads(selftest.read_text());report['loader_selftest']=loader
    fixture=staged['build_flags']['XNAV_ENABLE_TEST_FIXTURES']=='ON'
    purpose='DEVELOPER TEST BUILD' if fixture else 'INSTALLED PRODUCT'
    output_policy='test-loopback-only' if staged['build_flags']['XNAV_ENABLE_PILOT_LOOPBACK_TESTS']=='ON' else 'status-only'
    i.require(loader['passed'] is True and loader['commit']==frozen['commit'] and loader['test_fixtures'] is fixture and loader['build_purpose']==purpose and loader['profile_initialized'] is False and loader['plugins_loaded'] is False and loader['xnav_hardware_output_policy']==output_policy,'Runtime loader identity/capability differs')
    for scene,label,style in [(scene,label,style) for scene in scene_data['scenes'] for label,style in [('Standard','Standard')]]:
        label=scene['id']+'-'+label
        lat,lon=scene['center'];scale=scene['scalePpm']
        profile=profile_root/label;profile_labels.append(label)
        i.require(len(str(profile/'opencpn-ipc').encode())<100,'Keep the Unix IPC path below platform socket-address capacity')
        subprocess.run([sys.executable,str(root/'app/tools/prepare-test-profile.py'),'--build',str(root/'build'),'--profile',str(profile)],check=True)
        with (profile/'opencpn.conf').open('a') as f:
            f.write('\n[Settings]\nOpenGL='+str(int(args.renderer=='opengl'))+'\nChartQuilting=1\n[ChartDirectories]\nChartDir1='+str(chart_root)+'\n[Settings/GlobalState]\nVPLatLon='+str(lat)+','+str(lon)+'\nVPScale='+str(scale)+'\nnSymbolStyle=76\n[OpenNav]\nChartPresentationV1='+style+'\n')
            # Exact existing smoke-chart input-only TCP serialization; loopback only.
            f.write('\n[Settings/NMEADataSource]\nDataConnections='+f'1;0;127.0.0.1;{server.getsockname()[1]};0;;4800;1;0;0;;0;;0;0;0;0;1;SIMULATED chart loopback;0;;0;1;\n')
        log=(evidence/(label+'-launch.log')).open('w');started=time.monotonic()
        trace=evidence/(label+'-trace.jsonl');pid_file=evidence/(label+'-inferior.pid')
        launch_env=dict(env,SKAGER_SEAMARK_LAYOUT=str(collector/'layout.json'),SKAGER_SEAMARK_TRACE=str(trace),SKAGER_SEAMARK_PID=str(pid_file),SKAGER_SEAMARK_SCENE=json.dumps(scene),SKAGER_SEAMARK_STYLE=style)
        app=subprocess.Popen(['gdb','-nx','-batch','-x',str(collector/'render-probe.gdb'),'--args',str(exe),'--configdir',str(profile),'--xnav','--rebuild_chart_db']+(['--no_opengl'] if args.renderer=='software' else []),env=launch_env,stdout=log,stderr=log)
        app_pid=int(wait(lambda:pid_file.read_text() if pid_file.exists() else None))
        def win():
            r=subprocess.run(['xdotool','search','--all','--onlyvisible','--pid',str(app_pid),'--name','^SKAGER / OpenCPN$'],env=env,text=True,capture_output=True)
            return r.stdout.strip().splitlines()[0] if r.returncode==0 and r.stdout.strip() else None
        handle=wait(win);xdo('windowsize',handle,1280,800);xdo('windowmove',handle,0,0);xdo('windowfocus',handle)
        wait(lambda:'OnInitTimer...Finalize Canvases' in (profile/'opencpn.log').read_text(errors='replace'))
        def data():return json.loads((profile/'opennav-diagnostics.json').read_text())
        def ready(light):
            d=data();c=d['runtime']['chart'];p=d['runtime']['chart_presentation'];owned={v['name']:v for v in d['data']}
            fresh=all(abs(owned[k].get('value',999999)-v)<1e-6 and owned[k].get('age_ms',999999)<2000 and owned[k]['validity']=='Measured' and owned[k]['quality']=='LIVE' for k,v in [('Latitude',47.6),('Longitude',-122.36),('Speed over ground',3.0),('Course over ground',90.0)])
            return d if fresh and sent and sent[-1]['sent_monotonic']>started and d['runtime']['display']['light']==light and any(x['file']==scene['cell'] and x['type']==5 for x in c.get('quilt_members',[])) and p['requested']==style else None
        for theme_index, light in enumerate(scene['themes']):
            wait(lambda:trace_done(trace))
            trace_proof=check_trace(trace_records(trace),scene,style)
            wait(lambda:ready(light));xdo('mousemove',5,5);time.sleep(1.2);d=ready(light);i.require(bool(d),'Fresh input lost before capture')
            i.require(d['build_commit']==frozen['commit'] and d['data_mode']=='OPENCPN selected navigation' and d['test_fixtures'] is fixture and d['build_purpose']==purpose and d['xnav_hardware_output_policy']==output_policy,'Demo, replay, different executable or unexpected capability')
            pilot=d['runtime']['pilot']
            i.require(pilot['enabled'] is False and pilot['simulated'] is False and pilot['control_capability'] is False and pilot['command_state']=='None','Pilot/control activation is outside this capture')
            i.require(d['runtime']['chart']['opengl_enabled'] is (args.renderer=='opengl'),'Requested renderer not active')
            presentation=d['runtime']['chart_presentation']
            i.require(presentation['core']=={'available':True,'saved_point_style':76,'effective_point_style':76},'Core saved/effective point-style differs')
            i.require(presentation['private_ocharts']=={'available':False},'Absent private adapter must remain explicitly unavailable')
            c=d['runtime']['chart']
            i.require(abs(c['latitude']-lat)<1e-7 and abs(c['longitude']-lon)<1e-7 and abs(c['scale_ppm']-scene['actualScale'])<1e-7 and c['follow'] is False and c['quilt'] is True,'Actual official test viewport differs')
            i.require(c['canvas_pixels']=={'width':1014,'height':566} and c['database_entries']==1 and c['quilt_members']==[{'type':5,'native_scale':scene['nativeScale'],'file':scene['cell'],'index':c['quilt_reference']}],'Actual single official test ENC quilt differs')
            i.require(d['runtime']['chart_presentation']['status']==(('SKAGER presentation v1 / pinned symbols' if style=='XNav' else 'Standard OpenCPN presentation')+'; o-charts: no SKAGER adapter loaded'),'Requested style or exact absent-adapter state differs')
            name=label+'-'+light+('-return' if theme_index==3 else '');picture=evidence/(name+'.png')
            subprocess.run(['import','-window','root',str(picture)],env=env,check=True)
            region=d['runtime']['display']['chart_region'];x,y,w,h=[region[k] for k in ('x','y','width','height')]
            image=Image.open(picture).convert('RGB');i.require(image.size==(1280,800),'Capture geometry differs')
            pixels=collections.Counter(image.crop((x,y,x+w,y+h)).getdata())
            table={'Day':'DAY_BRIGHT','Dusk':'DUSK','Night':'NIGHT'}[light]
            palette=manifest['palette'][table] if style=='XNav' else standard[table]
            water_colors={tuple(round(c*255/256) if args.renderer=='opengl' else c for c in palette[role]) for role in ('DEPDW','DEPMD','DEPMS','DEPVS')}
            water_pixels=sum(pixels[color] for color in water_colors)
            i.require(water_pixels>1000,'Exact ENC water ink absent from official fixture')
            chart_bounds=(x,y,x+w,y+h)
            if theme_index==0:
                day_chart=image.crop(chart_bounds).copy()
            elif theme_index==3:
                i.require(ImageChops.difference(day_chart,image.crop(chart_bounds)).getbbox() is None,'Whole chart Day-return differs')
                report.setdefault('day_return_checks',[]).append({'image':name,'bounds':chart_bounds,'equal':True,'mask':None})
            crop_receipts=[]
            for feature in scene['features']:
                if style=='Standard' and feature['class']=='LIGHTS':continue
                attr=feature['attributes']
                candidates=[r for r in trace_proof['raster_draws'] if r['class']==feature['class'] and abs(r['latitude']-attr['latitude'])<1e-6 and abs(r['longitude']-attr['longitude'])<1e-6]
                draw=candidates[-1];px=x+draw['pixel_x'];py=y+draw['pixel_y']
                bounds=(px-36,py-36,px+36,py+36)
                i.require(x<=bounds[0] and y<=bounds[1] and bounds[2]<=x+w and bounds[3]<=y+h,'Source glyph crop outside actual chart')
                crop=evidence/(name+'-'+feature['class']+'-'+str(attr['RCID'])+'.png')
                image.crop(bounds).save(crop)
                crop_receipts.append({'class':feature['class'],'source_rcid':attr['RCID'],'bounds':bounds,'draw':draw,'sha256':hashlib.sha256(crop.read_bytes()).hexdigest()})
            owned={v['name']:v for v in d['data']}
            (evidence/(name+'.json')).write_text(json.dumps(d,indent=2)+'\n')
            report['captures'].append({'name':name,'image_sha256':hashlib.sha256(picture.read_bytes()).hexdigest(),'scene':scene['id'],'chart':c,'presentation':d['runtime']['chart_presentation'],'source_feature_crops':crop_receipts,'trace':trace_proof,'water_pixels':water_pixels,'owned_values':{k:owned[k] for k in ('Latitude','Longitude','Speed over ground','Course over ground')}})
            if theme_index+1<len(scene['themes']):
                target=scene['themes'][theme_index+1]
                for _ in range(3):
                    current=data()['runtime']['display']['light']
                    if current==target:break
                    button=next(b for b in data()['runtime']['display']['interaction_controls'] if b['label']==current and b['visible'] and b['enabled'])
                    xdo('mousemove',button['x']+button['width']//2,button['y']+button['height']//2);xdo('click',1);xdo('mousemove',5,5)
                    wait(lambda:data()['runtime']['display']['light']!=current)
                i.require(data()['runtime']['display']['light']==target,'Actual theme controls did not reach requested light')
        subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,capture_output=True,timeout=15)
        i.require(app.wait(timeout=15)==0,'Application did not exit cleanly');app=None;log.close();log=None
        exits=[r for r in trace_records(trace) if r['kind']=='inferior_exit']
        i.require(exits==[{'kind':'inferior_exit','exit_code':0}],'Actual application inferior did not exit normally')
        persisted=(profile/'opencpn.conf').read_text()
        i.require(re.findall(r'^nSymbolStyle=(\d+)\s*$',persisted,re.M)==['76'],'Disposable Simplified symbol setting did not persist')
        report.setdefault('symbol_table_receipts',[]).append({'style':style,'persisted_nSymbolStyle':76,'scope':'Persisted preference plus explicitly scoped core diagnostics; private adapter unavailable; actual object table independently traced'})
    i.require(not errors and len(report['captures'])==4,'Input error or incomplete theme-cycle capture')
    i.require(chart_files=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in chart_root.iterdir() if p.is_file()},'Read-only chart fixture changed')
    report['status']='passed';report['clean_exit']=True
except Exception as error:
    report['status']='failed';report['failure']=str(error)
    subprocess.run(['import','-window','root',str(evidence/'failure-canvas.png')],env=env,timeout=15)
    raise
finally:
    stop.set();server.close();thread.join(timeout=3)
    (evidence/'controlled-input.json').write_text(json.dumps({'scope':'Owned simulated loopback RMC into actual OpenCPN input path; not live navigation','connections':connections,'sent':sent,'errors':errors},indent=2)+'\n')
    if app and app.poll() is None:app.terminate();app.wait(timeout=10)
    if log:log.close()
    if xserver and xserver.poll() is None:xserver.terminate();xserver.wait(timeout=5)
    for label in profile_labels:
        source=profile_root/label
        if source.exists():
            shutil.copytree(source,evidence/(label+'-profile'),ignore=shutil.ignore_patterns('opencpn-ipc','_OpenCPN_SILock'))
    shutil.rmtree(profile_root)
    (evidence/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print('Four exact-build SKAGER source-scene captures passed:',evidence)
