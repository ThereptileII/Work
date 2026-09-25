#!/usr/bin/env python3
"""Disposable public ENC, upstream renderer, plugin and chart interaction gate."""
import collections
from contextlib import closing
import sqlite3
import ctypes
import datetime
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import zipfile
root=Path(__file__).resolve().parents[1];windows=sys.platform=='win32'
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
url='https://www.charts.noaa.gov/ENCs/US5SEAFL.zip'
digest='b027e029dc7b76595381d89e3718145eb5069e5a03917e78bada4f8ebe8c84e4'
report={'chart_origin':url,'chart_sha256':digest,'agreement':'https://charts.noaa.gov/ENCs/ENC_Agreement.shtml','scope':'Disposable rendering fixture; not redistributed as a navigation product','authority':'native Windows' if windows else 'Linux development','phases':[],'screenshots':[]}
def module(name):
 s=importlib.util.spec_from_file_location(name,root/'tools'/f'{name}.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
ui=module('windows-ui') if windows else None;fixtures=module('profile-fixtures')
cache=root/'build/chart-fixtures';cache.mkdir(parents=True,exist_ok=True)
archive=cache/'US5SEAFL.zip'
if not archive.exists():
 with urllib.request.urlopen(url,timeout=60) as response:content=response.read(64*1024*1024+1)
 assert len(content)<=64*1024*1024
 archive.write_bytes(content)
content=archive.read_bytes();assert hashlib.sha256(content).hexdigest()==digest,'NOAA fixture changed; inspect and approve a new fixture hash explicitly'
with zipfile.ZipFile(io.BytesIO(content)) as z:
 assert sum(f.file_size for f in z.infolist())<256*1024*1024
 for f in z.infolist():
  p=Path(f.filename);assert not p.is_absolute() and '..' not in p.parts
 z.extractall(cache)
chartdir=cache/'ENC_ROOT';assert (chartdir/'US5SEAFL/US5SEAFL.000').is_file()
# Adjacent cell's inspected exchange catalog bounds: 47.55..47.625 N,
# -122.475..-122.4 E. Keep its catalog/notices separate from the first exchange.
adjacent_url='https://www.charts.noaa.gov/ENCs/US5SEAFK.zip'
adjacent_digest='99c9b55d49828503666668e956f91444ee79b434db4bdf65f3104c3e42c32d51'
adjacent_archive=cache/'US5SEAFK.zip'
if not adjacent_archive.exists():
 with urllib.request.urlopen(adjacent_url,timeout=60) as response:content=response.read(64*1024*1024+1)
 assert len(content)<=64*1024*1024
 adjacent_archive.write_bytes(content)
content=adjacent_archive.read_bytes();assert hashlib.sha256(content).hexdigest()==adjacent_digest,'Adjacent NOAA fixture changed; inspect before repinning'
with zipfile.ZipFile(io.BytesIO(content)) as z:
 assert sum(f.file_size for f in z.infolist())<256*1024*1024
 for f in z.infolist():
  p=Path(f.filename);assert not p.is_absolute() and '..' not in p.parts
 z.extractall(cache/'adjacent')
adjacent_dir=cache/'adjacent/ENC_ROOT'
assert (adjacent_dir/'US5SEAFK/US5SEAFK.000').is_file()
report['adjacent_chart']={'origin':adjacent_url,'sha256':adjacent_digest,'switch_position':[47.59,-122.447]}
tmp=tempfile.TemporaryDirectory(prefix='opennav charts ',dir=None if windows else '/tmp');profile=Path(tmp.name)/'profile'
variant='xnav-windows' if windows else 'xnav-linux'
subprocess.run([sys.executable,str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build'/variant),'--profile',str(profile)],check=True)
fixtures.seed(profile);expected=fixtures.snapshot(profile)
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1);server.settimeout(.2)
stop=threading.Event();errors=[]
position=(47.6,-122.36)
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
   assert address[0]=='127.0.0.1';peer.settimeout(2)
   while not stop.wait(.3):
    now=datetime.datetime.now(datetime.timezone.utc)
    lat,lon=position
    lat_min=(lat-int(lat))*60;lon_min=(abs(lon)-int(abs(lon)))*60
    peer.sendall(sentence(f'GPRMC,{now:%H%M%S},A,{int(lat):02}{lat_min:06.3f},N,{int(abs(lon)):03}{lon_min:06.3f},W,3.0,90.0,{now:%d%m%y},,,A'))
  except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
  except Exception as e:
   if not stop.is_set():errors.append(str(e))
  finally:peer.close()
thread=threading.Thread(target=transmit,daemon=True);thread.start()
with (profile/'opencpn.conf').open('a') as f:
 f.write('\n[Settings]\nOpenGL=0\nChartQuilting=1\n[ChartDirectories]\nChartDir1='+chartdir.as_posix()+'\n[Settings/GlobalState]\nVPLatLon=47.6000,-122.3600\nVPScale=0.15\n')
 f.write('\n[ChartDirectories]\nChartDir2='+adjacent_dir.as_posix()+'\n')
 f.write('\n[Settings/NMEADataSource]\nDataConnections='+fixtures.CONNECTION+'|'+f'1;0;127.0.0.1;{server.getsockname()[1]};0;;4800;1;0;0;;0;;0;0;0;0;1;SIMULATED chart loopback;0;;0;1;\n')
 for name in ['wmm','grib']:
  plugin=name+'_pi.dll' if windows else 'lib'+name+'_pi.so'
  f.write(f'\n[PlugIns/{plugin}]\nbEnabled=1\n')
env=dict(os.environ);xserver=None;app=None;handle=None;pid=None;owned=set();count=0
if windows:report['display']=ui.ensure_desktop()
else:
 assert ctypes.CDLL(None).prctl(36,1,0,0,0)==0
 n=141
 while Path(f'/tmp/.X{n}-lock').exists():n+=1
 env['DISPLAY']=f':{n}'
 xserver=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);time.sleep(1)
exe=root/('build/xnav-install/opencpn.exe' if windows else 'build/xnav-install/bin/opencpn')
def xdo(*args):return subprocess.check_output(['xdotool',*map(str,args)],env=env,text=True).strip()
def window(title):
 if windows:return ui.wait_window(title)
 end=time.monotonic()+60
 while time.monotonic()<end:
  r=subprocess.run(['xdotool','search','--onlyvisible','--name','^'+title+'$'],env=env,capture_output=True,text=True)
  if r.returncode==0 and r.stdout.strip():
   h=r.stdout.splitlines()[0];return h,int(xdo('getwindowpid',h))
  time.sleep(.2)
 raise RuntimeError('Missing chart window '+title)
def ready():
 end=time.monotonic()+90
 while time.monotonic()<end:
  log=profile/'opencpn.log'
  if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>=count:
   time.sleep(1);return
  time.sleep(.2)
 raise RuntimeError('Chart initialization timed out')
def data(predicate=lambda d:True):
 end=time.monotonic()+30
 while time.monotonic()<end:
  try:
   d=json.loads((profile/'opennav-diagnostics.json').read_text())
   if predicate(d):return d
  except (OSError,json.JSONDecodeError,KeyError):pass
  time.sleep(.15)
 if 'd' in locals():
  (evidence/'charts-failed-diagnostic.json').write_text(json.dumps(d,indent=2)+'\n')
 raise RuntimeError('Chart diagnostic assertion timed out')
def chart(c):return c['runtime']['chart']
def enc(d):return any(c['file']=='US5SEAFL.000' for c in chart(d).get('quilt_members',[]))
def reference_cell(d,name):return any(c['file']==name and c['index']==chart(d)['quilt_reference'] for c in chart(d).get('quilt_members',[]))
def capture(name,check=True):
 p=evidence/(name+('.png' if windows else '-linux.png'))
 if windows:ui.SetForegroundWindow(handle);rgb=ui.capture(handle,p,screen_pixels=True)
 else:
  xdo('windowsize',handle,1280,800,'windowmove',handle,0,0);time.sleep(.5)
  subprocess.run(['import','-window','root',str(p)],env=env,check=True)
  rgb=subprocess.check_output(['convert',str(p),'-depth','8','rgb:-'],env=env)
 report['screenshots'].append(p.name)
 # Interior ENC symbols/contours use many colors and short edges, unlike a
 # uniform canvas. Chart identity comes from the upstream quilt, not pixels.
 colors=collections.Counter(bytes(rgb[(y*1280+x)*3:(y*1280+x)*3+3]) for y in range(180,600,2) for x in range(150,950,2))
 detail=sum(v for _,v in colors.most_common()[3:])/sum(colors.values())
 if check:assert len(colors)>20 and detail>.005,(name,len(colors),detail,'ENC detail absent')
 return {'screenshot':p.name,'interior_colors':len(colors),'non_background_fraction':detail}
def command(label,key):
 if windows:ui.click_text(pid,label)
 else:xdo('windowfocus',handle,'key',key);time.sleep(.5)
def close():
 global app
 if windows:
  p=ui.monitor_process(pid);ui.close(handle);ui.wait_clean_exit(p)
 else:
  subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],env=env,check=True,timeout=15)
  if app and app.pid==pid:assert app.wait(timeout=30)==0
  else:
   child,status=os.waitpid(pid,0);assert os.waitstatus_to_exitcode(status)==0
 owned.discard(pid);app=None
try:
 for rendering in ['software','opengl']:
  if rendering=='opengl':
   with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings]\nOpenGL=1\n')
  start=time.monotonic()
  with (evidence/'charts-launch.log').open('a') as out:
   app=subprocess.Popen([str(exe),'--configdir',str(profile),'--xnav','--rebuild_chart_db']+(['--no_opengl'] if rendering=='software' else []),env=env,stdout=out,stderr=out)
  owned.add(app.pid);count+=1;handle,pid=window('OpenNav X / OpenCPN');ready()
  if windows:ui.size_window(handle)
  else:xdo('windowsize',handle,1280,800,'windowmove',handle,0,0)
  d=data(enc);entry={'requested_rendering':rendering,'runtime':d['runtime'],'startup_to_enc_seconds':round(time.monotonic()-start,3),'captures':[]}
  if rendering=='software':assert not chart(d)['opengl_enabled']
  else:entry['gl_status']='enabled; inspect renderer log' if chart(d)['opengl_enabled'] else 'host rejected OpenGL; verified upstream software fallback; hardware GL gate remains open'
  for name in ['Dashboard','WMM','GRIB']:
   assert any(p['name'].lower()==name.lower() and p['enabled'] and p['initialized'] for p in d['runtime']['plugins']),(name,d['runtime']['plugins'])
  entry['captures'].append(capture('chart-'+rendering+'-01-loaded'))
  if windows:
   # Native process accounting; no psutil dependency or host-wide samples.
   k=ctypes.WinDLL('kernel32',use_last_error=True);ps=ctypes.WinDLL('psapi',use_last_error=True)
   class FT(ctypes.Structure):_fields_=[('low',ctypes.c_uint32),('high',ctypes.c_uint32)]
   class MEM(ctypes.Structure):_fields_=[('size',ctypes.c_uint32),('faults',ctypes.c_uint32)]+[(n,ctypes.c_size_t) for n in ['peak','working','paged_peak','paged','nonpaged_peak','nonpaged','pagefile','peak_pagefile']]
   k.OpenProcess.restype=ctypes.c_void_p;k.OpenProcess.argtypes=[ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32]
   k.GetProcessTimes.argtypes=[ctypes.c_void_p]+[ctypes.POINTER(FT)]*4
   k.CloseHandle.argtypes=[ctypes.c_void_p]
   ps.GetProcessMemoryInfo.argtypes=[ctypes.c_void_p,ctypes.POINTER(MEM),ctypes.c_uint32]
   def usage():
    h=k.OpenProcess(0x410,0,pid);assert h
    try:
     a,b,c,e=FT(),FT(),FT(),FT();assert k.GetProcessTimes(h,ctypes.byref(a),ctypes.byref(b),ctypes.byref(c),ctypes.byref(e))
     m=MEM();m.size=ctypes.sizeof(m);assert ps.GetProcessMemoryInfo(h,ctypes.byref(m),m.size)
     return ((c.high<<32)+c.low+(e.high<<32)+e.low)/1e7,m.working
    finally:k.CloseHandle(h)
  else:
   def usage():
    fields=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
    return (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),int(fields[21])*os.sysconf('SC_PAGE_SIZE')
  cpu0,_=usage();t0=time.monotonic();time.sleep(10);cpu1,rss=usage();elapsed=time.monotonic()-t0
  entry['performance']={'idle_live_input_seconds':elapsed,'cpu_percent_one_core':100*(cpu1-cpu0)/elapsed,'resident_bytes':rss,'shell_update':data()['runtime']['ui_update'],'scope':'CI desktop / two loaded NOAA ENCs / 3 plugins / 3.3 Hz RMC; not target navigation PC'}
  scale=chart(d)['scale_ppm'];command('+','plus');data(lambda d:enc(d) and chart(d)['scale_ppm']>scale*1.2)
  entry['captures'].append(capture('chart-'+rendering+'-02-zoom'))
  command('−','minus');data(lambda d:enc(d) and chart(d)['scale_ppm']<scale*1.2)
  before=chart(data(enc))
  if windows:
   ui.SetForegroundWindow(handle);ui.user.SetCursorPos(600,400);ui.user.mouse_event(2,0,0,0,0)
   for x in range(600,681,10):ui.user.SetCursorPos(x,420);time.sleep(.05)
   ui.user.mouse_event(4,0,0,0,0)
  else:xdo('mousemove',600,400,'mousedown',1,'mousemove',680,420,'sleep',.2,'mouseup',1)
  data(lambda d:abs(chart(d)['longitude']-before['longitude'])>.0001)
  command('GPS','F2');d=data(lambda d:chart(d)['follow'] and abs(chart(d)['latitude']-47.6)<.0001 and abs(chart(d)['longitude']+122.36)<.0001 and enc(d))
  entry['captures'].append(capture('chart-'+rendering+'-03-follow'))
  position=(47.59,-122.447)
  switched=data(lambda d:chart(d)['follow'] and abs(chart(d)['longitude']-position[1])<.0001 and reference_cell(d,'US5SEAFK.000'))
  entry['captures'].append(capture('chart-'+rendering+'-03a-adjacent-cell'))
  entry['chart_switch']={'synthetic_position_jump':True,'adjacent':chart(switched)}
  position=(47.6,-122.36)
  restored=data(lambda d:abs(chart(d)['longitude']-position[1])<.0001 and reference_cell(d,'US5SEAFL.000'))
  entry['chart_switch']['returned']=chart(restored)
  entry['captures'].append(capture('chart-'+rendering+'-03b-returned-cell'))
  command('Menu','ctrl+shift+m');data(lambda d:d['ui_page']=='Menu');capture('chart-'+rendering+'-overlay',False)
  command('Navigation','ctrl+shift+n');data(lambda d:d['ui_page']=='Navigation' and enc(d))
  entry['captures'].append(capture('chart-'+rendering+'-04-restored'))
  if rendering=='software':
   command('Menu','ctrl+shift+g')
   if windows:
    ui.click_text(pid,'Settings');ui.click_text(pid,'OpenCPN plugins')
    options,_=ui.wait_window('Options',pid)
    end=time.monotonic()+10
    while time.monotonic()<end:
     captions=[c for _,c in ui.children(options)]
     if any('Dashboard' in c for c in captions) and any('WMM' in c for c in captions):break
     time.sleep(.2)
    else:raise RuntimeError('Upstream plugin manager not visible: '+repr(captions))
    # Child captions can exist before wx has thawed and painted the dialog.
    # Require real interior pixels and painted action buttons. A partial first
    # paint can show the plugin list while the footer is still blank.
    ui.SetForegroundWindow(options);end=time.monotonic()+10
    while time.monotonic()<end:
     rgb=ui.capture(options,evidence/'chart-plugin-manager.png',resize=False,screen_pixels=True)
     rect=ui.W.RECT();assert ui.GetWindowRect(options,ctypes.byref(rect))
     width,height=rect.right-rect.left,rect.bottom-rect.top
     pixels=collections.Counter(bytes(rgb[(y*width+x)*3:(y*width+x)*3+3]) for y in range(100,height-70,2) for x in range(15,width-15,2))
     painted=[]
     for child,caption in ui.children(options):
      if caption.replace('&','') not in ('OK','Cancel','Apply'):continue
      bounds=ui.W.RECT();assert ui.GetWindowRect(child,ctypes.byref(bounds))
      left,right=bounds.left-rect.left+6,bounds.right-rect.left-6
      top,bottom=bounds.top-rect.top+4,bounds.bottom-rect.top-4
      if not (0<=left<right<width and 0<=top<bottom<height):continue
      luminance=[sum(rgb[(y*width+x)*3:(y*width+x)*3+3]) for y in range(top,bottom) for x in range(left,right)]
      if max(luminance)-min(luminance)>90:painted.append(caption.replace('&',''))
     if len(pixels)>=32 and set(painted)=={'OK','Cancel','Apply'}:
      entry['plugin_manager_interior_colors']=len(pixels)
      entry['plugin_manager_painted_buttons']=painted
      break
     time.sleep(.2)
    else:raise RuntimeError('Plugin manager controls exist but its contents did not paint')
    report['screenshots'].append('chart-plugin-manager.png');ui.dismiss_native_dialog(options,'Cancel')
   else:
    # Public settings keyboard focus is not guessed; Linux validates loading.
    # Native gate above verifies the actual upstream manager interaction.
    pass
   command('Navigation','ctrl+shift+n')
   if windows:
    ui.click_text(pid,'Menu');ui.click_text(pid,'Routes');ui.click_text(pid,'Create route on chart');ui.click_text(pid,'Create route')
    data(lambda d:d['ui_page']=='Navigation')
    ui.SetForegroundWindow(handle)
    for x,y in [(430,300),(580,300),(720,360)]:
     ui.user.SetCursorPos(x,y);ui.user.mouse_event(2,0,0,0,0);ui.user.mouse_event(4,0,0,0,0);time.sleep(.4)
    ui.click_text(pid,'Done')
    def created_points():
     with closing(sqlite3.connect((profile/'navobj.db').resolve().as_uri()+'?mode=ro',uri=True)) as db:
      return db.execute("SELECT r.guid,p.guid,p.lat,p.lon FROM routes r JOIN routepoints_link l ON r.guid=l.route_guid JOIN routepoints p ON p.guid=l.point_guid WHERE r.name NOT LIKE 'SIMULATED persistence%' OR r.name IS NULL ORDER BY l.point_order").fetchall()
    points=created_points();assert len(points)==3,points
    entry['captures'].append(capture('chart-created-route-before-edit'))
    # In upstream desktop mode LeftDown selects the point and starts editing.
    # A separate selection click then another down within 300 ms is a Windows
    # double-click and opens Mark Properties, preventing the intended drag.
    ui.user.SetCursorPos(430,300);time.sleep(.6);ui.user.mouse_event(2,0,0,0,0);time.sleep(.15)
    for x in range(430,471,5):ui.user.SetCursorPos(x,320);time.sleep(.08)
    ui.user.mouse_event(4,0,0,0,0);time.sleep(.7)
    changed=created_points()
    if not (len(changed)==3 and changed[0][:2]==points[0][:2] and changed[0][2:]!=points[0][2:]):
     capture('chart-route-edit-failed',False)
     raise AssertionError((points,changed,ui.windows(pid)))
    entry['route_geometry']='Native chart gestures created a three-point route and persisted a point move with unchanged identities'
    entry['captures'].append(capture('chart-created-and-edited-route'))
   entry['plugin_manager']='native manager opened and listed Dashboard/WMM' if windows else 'native Windows interaction is authoritative; loader records checked here'
  if windows:ui.click_text(pid,'System');ui.click_text(pid,'Open Legacy OpenCPN')
  else:xdo('key','ctrl+shift+l')
  assert app.wait(timeout=40)==0;owned.discard(pid);count+=1
  handle,pid=window('OpenCPN / Legacy');owned.add(pid);ready();entry['captures'].append(capture('chart-'+rendering+'-05-legacy'))
  if windows:
   p=ui.monitor_process(pid);ui.click_menu(handle,'Switch to XNav');ui.wait_clean_exit(p)
  else:
   xdo('windowfocus',handle,'mousemove',600,400,'click',3);time.sleep(.3);xdo('key','End','Return')
   child,status=os.waitpid(pid,0);assert os.waitstatus_to_exitcode(status)==0
  owned.discard(pid);count+=1;handle,pid=window('OpenNav X / OpenCPN');owned.add(pid);ready();data(enc)
  entry['captures'].append(capture('chart-'+rendering+'-06-returned'))
  close();assert fixtures.snapshot(profile)==expected;assert not errors,errors
  report['phases'].append(entry)
 report['result']='passed; native screenshot review and physical GPU gate remain separate'
finally:
 stop.set();thread.join(timeout=5);server.close()
 for p in owned:
  if windows:subprocess.run(['taskkill','/PID',str(p),'/F'],capture_output=True)
  else:
   try:os.kill(p,9);os.waitpid(p,0)
   except (ProcessLookupError,ChildProcessError):pass
 shutil.copytree(profile,evidence/'charts-profile',dirs_exist_ok=True,ignore=shutil.ignore_patterns('opencpn-ipc','*.pem','*.S57'))
 log=(profile/'opencpn.log').read_text(errors='replace') if (profile/'opencpn.log').exists() else ''
 report['renderer_log']=[s for s in log.splitlines() if 'OpenGL' in s or 'renderer' in s.lower()]
 (evidence/'charts-results.json').write_text(json.dumps(report,indent=2))
 if xserver:xserver.terminate();xserver.wait(timeout=10)
 tmp.cleanup()
print(report['result'])
