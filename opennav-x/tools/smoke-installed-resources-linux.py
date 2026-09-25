#!/usr/bin/env python3
"""Actual harmonic-loader regression under xvfb-run on isolated Linux profiles.

The stock executable is a labelled resource-locator fixture, never executed.
This is not native Windows installer acceptance. Actual bundled harmonic data
and the real application/configuration loader exercise all three modes.
"""
import configparser,json,os,pathlib,shutil,subprocess,tempfile,time
import sys
if sys.platform != 'linux' or not os.environ.get('DISPLAY'):
 raise SystemExit('Run this isolated Linux regression under xvfb-run.')
root=pathlib.Path(__file__).resolve().parents[1]; ev=root/'evidence/local/installed-resource-runtime';ev.mkdir(exist_ok=True)
report={'authority':'Linux development; actual application/harmonic loader, synthetic stock-locator identity','phases':[]}
with tempfile.TemporaryDirectory(prefix='onx-res-') as t:
 t=pathlib.Path(t); profile=t/'profile'; stock=t/'Stock Åland'; stock.mkdir()
 resources=root/'build/xnav-install/share/opencpn'
 for name in ['tcdata','gshhs','basemap_shp','sounds']:(stock/name).symlink_to(resources/name,target_is_directory=True)
 (stock/'opencpn.exe').write_text('Explicit resource locator fixture; never executed; not a stock binary acceptance.')
 subprocess.run(['python3',str(root/'tools/prepare-test-profile.py'),'--build',str(root/'build/xnav-linux'),'--profile',str(profile)],check=True)
 for index,mode in enumerate(['--xnav','--legacy','--safe-mode']):
  app=t/f'generation{index}';app.mkdir();exe=app/'opencpn';shutil.copy2(root/'build/xnav-install/bin/opencpn',exe)
  (app/'OPENNAV_INSTALLED_STOCK').write_text(str(stock/'opencpn.exe'))
  if index:
   shutil.rmtree(t/f'generation{index-1}')
  if index==2:
   cfg=configparser.RawConfigParser(strict=False);cfg.optionxform=str;cfg.read(profile/'opencpn.conf');cfg['TideCurrentDataSources']={'tcds0':str(stock/'tcdata/harmonics-dwf-20210110-free.tcd')}
   with (profile/'opencpn.conf').open('w') as f:cfg.write(f)
  output=(ev/f'launch-{index}.log').open('w')
  p=subprocess.Popen([str(exe),'--configdir',str(profile),'--no_opengl',mode],stdout=output,stderr=output)
  try:
   deadline=time.monotonic()+45
   while time.monotonic()<deadline:
    log=profile/'opencpn.log'
    if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>=index+1:break
    assert p.poll() is None
    time.sleep(.2)
   else:
    shutil.copy2(profile/'opencpn.log',ev/'failed-application.log')
    subprocess.run(['import','-window','root',str(ev/'failed.png')])
    raise RuntimeError('Initialization did not complete')
   assert 'OpenNav installed resource defaults: original supported OpenCPN' in log.read_text()
   time.sleep(.5)
   subprocess.run([str(exe),'--configdir',str(profile),'--remote','--quit'],check=True,timeout=20,stdout=output,stderr=output)
   assert p.wait(timeout=30)==0
   cfg=configparser.RawConfigParser(strict=False);cfg.optionxform=str;cfg.read(profile/'opencpn.conf')
   tides=list(cfg['TideCurrentDataSources'].values())
   assert len(tides)==(1 if index==2 else 2),tides
   assert all(pathlib.Path(v).is_file() and str(stock) in v for v in tides)
   for section,key in [('Directories','BasemapDir'),('Directories','BaseShapefileDir'),('Settings/AIS','AISAlertAudioFile')]:assert str(stock) in cfg.get(section,key)
   report['phases'].append({'mode':mode,'tides':len(tides),'removedPriorGeneration':bool(index),'result':'passed'})
  finally:
   if p.poll() is None:p.kill();p.wait()
   output.close()
   if (profile/'opencpn.log').exists():shutil.copy2(profile/'opencpn.log',ev/'application.log')
   if sys.exc_info()[1]:report['result']='failed';report['error']=repr(sys.exc_info()[1])
   (ev/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 shutil.copy2(profile/'opencpn.log',ev/'application.log')
report['result']='passed';(ev/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
