#!/usr/bin/env python3
"""Prepared final exact-build real-ENC renderer capture; explicit identities required."""
import argparse,collections,datetime,hashlib,importlib.util,json,os,pathlib,re,shutil,socket,subprocess,sys,tempfile,threading,time
import xml.etree.ElementTree as ET
from PIL import Image
collector=pathlib.Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('phase')
parser.add_argument('--display',default=':197')
parser.add_argument('--renderer',choices=('software','opengl'),required=True)
parser.add_argument('--expected-commit',required=True)
parser.add_argument('--expected-exe-sha256',required=True)
parser.add_argument('--expected-manifest-sha256',required=True)
parser.add_argument('--baseline-report',type=pathlib.Path,required=True)
args=parser.parse_args()
root=pathlib.Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity')
spec=importlib.util.spec_from_file_location('cache_inputs',collector/'cache-inputs-readonly.py')
i=importlib.util.module_from_spec(spec);spec.loader.exec_module(i)
frozen=i.verify()
i.require(re.fullmatch('[a-f0-9]{40}',args.expected_commit) and frozen['commit']==args.expected_commit,'Explicit final source identity differs')
for digest in (args.expected_exe_sha256,args.expected_manifest_sha256):
    i.require(re.fullmatch('[a-f0-9]{64}',digest),'Explicit final SHA256 required')
baseline=json.loads(args.baseline_report.read_text())
i.require(baseline['commit']=='1356fd1603aacbea04d7081d16331e9a181180bb' and baseline['status']=='passed' and baseline['clean_exit'] is True and baseline['renderer_requested']==args.renderer,'Baseline is not the exact successful frozen1356 renderer')
i.require(re.fullmatch('[a-zA-Z0-9_-]+',args.phase) is not None,'Use a simple new evidence directory name')
i.require(re.fullmatch(':[0-9]+',args.display) is not None,'Use an isolated X display number')
evidence=collector/'output'/('capture-'+args.phase);evidence.mkdir(exist_ok=False)
profile_root=pathlib.Path(tempfile.mkdtemp(prefix='skager-capture-'))
profile_labels=[]
env=dict(os.environ,PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/bin:'+os.environ['PATH'],GDK_BACKEND='x11',DISPLAY=args.display,LD_LIBRARY_PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/lib')
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
chart_root=pathlib.Path('/home/standard/Projects/X-nav/.local/noaa-chart/ENC_ROOT')
chart_files={str(p.relative_to(chart_root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(chart_root.rglob('*')) if p.is_file()}
i.require('US5SEAFL/US5SEAFL.000' in chart_files,'Required public real NOAA ENC missing')
report={**staged,'authority':'Linux Xvfb '+args.renderer+' development visuals only; not native Windows, production or boat acceptance', 'renderer_requested':args.renderer,
        'scope':'Public real ENC with controlled simulated loopback RMC; no private boat profile, hardware, Demo or scenario activation',
        'baseline_report_sha256':hashlib.sha256(args.baseline_report.read_bytes()).hexdigest(),'baseline_commit':baseline['commit'],'chart_source_files':chart_files,'captures':[],'status':'running','clean_exit':False,
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

def check_surface_roles(image,pixels,style,light,renderer,palette,baseline,name):
    def rendered(rgb):return tuple(round(c*255/256) for c in rgb) if renderer=='opengl' else tuple(rgb)
    expected={
        'Day':{'LANDA':(238,238,226),'DEPDW':(213,229,229)},
        'Dusk':{'LANDA':(78,97,93),'DEPDW':(52,79,89)},
        'Night':{'LANDA':(29,41,37),'DEPDW':(14,23,28)}}
    if style=='XNav':
        for role,rgb in expected[light].items():
            i.require(tuple(palette[role])==rgb,'Manifest is not independently expected effective '+light+' '+role)
        i.require(tuple(palette['XNBUA'])==expected[light]['LANDA'],'Built-up surface must use the approved land role: '+light)
        land=rendered(expected[light]['LANDA']);water=rendered(expected[light]['DEPDW'])
    else:
        land=rendered(palette['LANDA']);water=rendered(palette['DEPDW'])
    i.require(pixels[land]>1000,'Actual ENC land role missing: '+name)
    control=next(c for c in baseline['captures'] if c['name']==name)
    if style=='Standard' or light!='Night':
        previous={key for key,count in control['water_color_counts'].items() if count>0}
        current={str(c) for c in pixels if c==water and pixels[c]>0}
        # The public ENC can display a different Standard depth fill, so its
        # original independently pinned palette remains the full strict check.
        if style=='XNav':i.require(str(water) in previous and str(water) in current,'Day/Dusk water differs from exact frozen baseline')
    return {'land_rgb':land,'land_pixels':pixels[land],'water_rgb':water,
            'baseline_capture':control['name'],'land_built_up_share_color':style=='XNav','comparison_scope':'exact eligible surface roles; shared land/BUA color does not independently identify built-up geometry; historical comparison excludes changed BUA ink'}

def check_wordmark(image,light):
    background={'Day':(21,35,38),'Dusk':(29,40,46),'Night':(12,17,21)}[light]
    # Header is 68px high at this fixed100% capture; the original cropped logo
    # occupies x16..163/y11..56. These are proven empty source margins/band.
    probes=[image.getpixel((x,y)) for y in (11,40,41,42,56) for x in range(20,160)]
    i.require(all(rgb==background for rgb in probes),'Wordmark matte or tooltip covers transparent header pixels')
    counts=[]
    for top,bottom in ((13,37),(46,56)):
        occupied=[]
        for x in range(16,164):
            occupied.append(any(max(image.getpixel((x,y))[c]-background[c] for c in range(3))>20 for y in range(top,bottom)))
        runs=sum(v and (x==0 or not occupied[x-1]) for x,v in enumerate(occupied))
        counts.append(runs)
    i.require(counts==[6,3],'Integrated wordmark glyph rows are missing or obscured')
    return {'empty_background_probes':len(probes),'background_rgb':background,'separate_letter_columns':counts,'pointer_parked_at':[5,5]}

try:
    i.require(not pathlib.Path('/tmp/.X'+args.display[1:]+'-lock').exists(),'Display lock already exists')
    xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    wait(lambda:subprocess.run(['xdotool','getdisplaygeometry'],env=env,capture_output=True).returncode==0,10)
    selftest=evidence/'loader-selftest.json'
    subprocess.run([str(exe),'--opennav-self-test',str(selftest)],env=env,check=True,capture_output=True,timeout=20)
    loader=json.loads(selftest.read_text());report['loader_selftest']=loader
    fixture=staged['build_flags']['XNAV_ENABLE_TEST_FIXTURES']=='ON'
    purpose='DEVELOPER TEST BUILD' if fixture else 'INSTALLED PRODUCT'
    output_policy='test-loopback-only' if staged['build_flags']['XNAV_ENABLE_PILOT_LOOPBACK_TESTS']=='ON' else 'status-only'
    i.require(loader['passed'] is True and loader['commit']==frozen['commit'] and loader['test_fixtures'] is fixture and loader['build_purpose']==purpose and loader['profile_initialized'] is False and loader['plugins_loaded'] is False and loader['xnav_hardware_output_policy']==output_policy,'Runtime loader identity/capability differs')
    for label,style in [('SKAGER','XNav'),('Standard','Standard')]:
        profile=profile_root/label;profile_labels.append(label)
        i.require(len(str(profile/'opencpn-ipc').encode())<100,'Keep the Unix IPC path below platform socket-address capacity')
        subprocess.run([sys.executable,str(root/'app/tools/prepare-test-profile.py'),'--build',str(root/'build'),'--profile',str(profile)],check=True)
        with (profile/'opencpn.conf').open('a') as f:
            f.write('\n[Settings]\nOpenGL='+str(int(args.renderer=='opengl'))+'\nChartQuilting=1\n[ChartDirectories]\nChartDir1='+str(chart_root)+'\n[Settings/GlobalState]\nVPLatLon=47.6000,-122.3600\nVPScale=0.15\n[OpenNav]\nChartPresentationV1='+style+'\n')
            # Exact existing smoke-chart input-only TCP serialization; loopback only.
            f.write('\n[Settings/NMEADataSource]\nDataConnections='+f'1;0;127.0.0.1;{server.getsockname()[1]};0;;4800;1;0;0;;0;;0;0;0;0;1;SIMULATED chart loopback;0;;0;1;\n')
        log=(evidence/(label+'-launch.log')).open('w');started=time.monotonic()
        app=subprocess.Popen([str(exe),'--configdir',str(profile),'--xnav','--rebuild_chart_db']+(['--no_opengl'] if args.renderer=='software' else []),env=env,stdout=log,stderr=log)
        def win():
            r=subprocess.run(['xdotool','search','--all','--onlyvisible','--pid',str(app.pid),'--name','^SKAGER / OpenCPN$'],env=env,text=True,capture_output=True)
            return r.stdout.strip().splitlines()[0] if r.returncode==0 and r.stdout.strip() else None
        handle=wait(win);xdo('windowsize',handle,1280,800);xdo('windowmove',handle,0,0);xdo('windowfocus',handle)
        wait(lambda:'OnInitTimer...Finalize Canvases' in (profile/'opencpn.log').read_text(errors='replace'))
        def data():return json.loads((profile/'opennav-diagnostics.json').read_text())
        def ready(light):
            d=data();c=d['runtime']['chart'];p=d['runtime']['chart_presentation'];owned={v['name']:v for v in d['data']}
            fresh=all(abs(owned[k].get('value',999999)-v)<1e-6 and owned[k].get('age_ms',999999)<2000 and owned[k]['validity']=='Measured' and owned[k]['quality']=='LIVE' for k,v in [('Latitude',47.6),('Longitude',-122.36),('Speed over ground',3.0),('Course over ground',90.0)])
            return d if fresh and sent and sent[-1]['sent_monotonic']>started and d['runtime']['display']['light']==light and any(x['file']=='US5SEAFL.000' and x['type']==5 for x in c.get('quilt_members',[])) and p['requested']==style else None
        for theme_index, light in enumerate(['Day','Dusk','Night','Day']):
            wait(lambda:ready(light));xdo('mousemove',5,5);time.sleep(1.2);d=ready(light);i.require(bool(d),'Fresh input lost before capture')
            i.require(d['build_commit']==frozen['commit'] and d['data_mode']=='OPENCPN selected navigation' and d['test_fixtures'] is fixture and d['build_purpose']==purpose and d['xnav_hardware_output_policy']==output_policy,'Demo, replay, different executable or unexpected capability')
            pilot=d['runtime']['pilot']
            i.require(pilot['enabled'] is False and pilot['simulated'] is False and pilot['control_capability'] is False and pilot['command_state']=='None','Pilot/control activation is outside this capture')
            i.require(d['runtime']['chart']['opengl_enabled'] is (args.renderer=='opengl'),'Requested renderer not active')
            i.require(d['runtime']['chart_presentation']['status']==('SKAGER presentation v1 / pinned symbols' if style=='XNav' else 'Standard OpenCPN presentation'),'Requested style fell back')
            name=label+'-'+light+('-return' if theme_index==3 else '');picture=evidence/(name+'.png')
            subprocess.run(['import','-window','root',str(picture)],env=env,check=True)
            region=d['runtime']['display']['chart_region'];x,y,w,h=[region[k] for k in ('x','y','width','height')]
            image=Image.open(picture).convert('RGB');i.require(image.size==(1280,800),'Capture geometry differs')
            edge_top = list(image.crop((690,68,1094,76)).getdata())
            edge_bottom = list(image.crop((690,626,1094,634)).getdata())
            # Exact known regression: undersized FBO repeats these full eight
            # nonuniform chart rows. Keep a strict negative-control baseline.
            i.require(len(set(edge_bottom))>8,'Edge fixture lost chart detail')
            repeated_edge_pixels=sum(a==b for a,b in zip(edge_top,edge_bottom))
            i.require(not edge_is_wrapped(edge_top,edge_bottom), 'Repeated OpenGL chart edge strip: '+name)
            # Derived negative probe uses sampled bytes only. No capture image
            # is rewritten: repeating the actual top strip must fail exactly.
            negative_repeated=sum(a==b for a,b in zip(edge_top,list(edge_top)))
            i.require(edge_is_wrapped(edge_top,list(edge_top)),'Edge negative control did not reproduce the full repeated strip')
            pixels=collections.Counter(image.crop((x,y,x+w,y+h)).getdata())
            table={'Day':'DAY_BRIGHT','Dusk':'DUSK','Night':'NIGHT'}[light]
            palette=manifest['palette'][table] if style=='XNav' else standard[table]
            water=tuple(palette['DEPDW']);built=tuple(palette['XNBUA' if style=='XNav' else 'CHBRN'])
            # Pinned RenderToGLAC at s52plib.cpp:8319 uses RGB/256.
            # Match that precise conversion for ENC area fills, not a tolerance.
            if args.renderer=='opengl':
                water=tuple(round(c*255/256) for c in water)
                built=tuple(round(c*255/256) for c in built)
            water_colors={water}
            if style=='Standard':
                water_colors.update(tuple(round(c*255/256) if args.renderer=='opengl' else c for c in palette[k]) for k in ('DEPMD','DEPMS','DEPVS'))
                # Pinned upstream GSHHS fallback coast uses this separate dimmed
                # palette, while chart BUAARE fill and quilt identity prove ENC.
                dim={'Day':1.0,'Dusk':0.5,'Night':0.25}[light]
                water_colors.add(tuple(int(c*dim) for c in (170,195,240)))
            water_pixels=sum(pixels[c] for c in water_colors)
            i.require(water_pixels>200000 and pixels[built]>50000, f'Real ENC water/land-or-built surface evidence missing: {name}, water={water_pixels}, surface={pixels[built]}')
            surface_roles=check_surface_roles(image,pixels,style,light,args.renderer,palette,baseline,name)
            wordmark=check_wordmark(image,light)
            owned={v['name']:v for v in d['data']}
            (evidence/(name+'.json')).write_text(json.dumps(d,indent=2)+'\n')
            report['captures'].append({'name':name,'image_sha256':hashlib.sha256(picture.read_bytes()).hexdigest(),'surface_roles':surface_roles,'wordmark':wordmark,'edge_negative_rejected':negative_repeated==len(edge_top),'edge_probe_pixels':len(edge_top),'repeated_edge_pixels':repeated_edge_pixels,'build_commit':d['build_commit'],'chart':d['runtime']['chart'],'presentation':d['runtime']['chart_presentation'],'water_pixels':water_pixels,'water_color_counts':{str(c):pixels[c] for c in sorted(water_colors)},'land_or_built_area_pixels':pixels[built],'built_area_color_shared_with_land':style=='XNav','owned_values':{k:owned[k] for k in ('Latitude','Longitude','Speed over ground','Course over ground')},'latest_owned_input_monotonic':sent[-1]['sent_monotonic']})
            if theme_index<3:
                button=next(b for b in d['runtime']['display']['interaction_controls'] if b['label']==light and b['visible'] and b['enabled'])
                xdo('mousemove',button['x']+button['width']//2,button['y']+button['height']//2);xdo('click',1);xdo('mousemove',5,5)
        subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,capture_output=True,timeout=15)
        i.require(app.wait(timeout=15)==0,'Application did not exit cleanly');app=None;log.close();log=None
    i.require(not errors and len(report['captures'])==8,'Input error or incomplete theme-cycle capture')
    report['status']='passed';report['clean_exit']=True
except Exception as error:
    report['status']='failed';report['failure']=str(error);raise
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
print('Eight exact-build real-ENC theme-cycle captures passed:',evidence)
