#!/usr/bin/env python3
"""Disposable native Windows installer lifecycle, shared profile and chart gate."""
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true':
    raise SystemExit('This destructive fixture is restricted to disposable Windows CI')
ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'evidence/local';EVIDENCE.mkdir(parents=True,exist_ok=True)
PACKAGE=ROOT/'build/alpha-installer';SETUP=PACKAGE/'OpenNavX-Alpha1-Setup.exe'
INSTALL=Path(os.environ['LOCALAPPDATA'])/'OpenNavXAlpha1'
STOCK_HASH='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
SETUP_HASH='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
PS=Path(os.environ['WINDIR'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
owned=set();report={'status':'running','checks':[],'screenshots':[],'authority':'native disposable Windows / PowerShell 5.1 / NSIS'}
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
ui=module('windows-ui');fixtures=module('profile-fixtures');charts=module('chart-render-check')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(p):return {f.relative_to(p).as_posix():sha(f) for f in p.rglob('*') if f.is_file()}
def check(name):report['checks'].append(name);print(name,flush=True)
def state():return json.loads((INSTALL/'state.json').read_text(encoding='utf-8-sig'))
def generation():return INSTALL/'generations'/state()['current']
def setup(action,stock,expected=0,failure='',executable=SETUP):
    out=EVIDENCE/f'installer-{len(report["checks"]):02}-{action}.json'
    assert all('"' not in str(value) for value in (stock,out,action,failure))
    # GetOptions recognizes /NAME="value with spaces", not a quoted entire
    # "/NAME=value with spaces" argument. This goes directly to CreateProcess.
    cmd=subprocess.list2cmdline([str(executable)])+' /S /ACTION='+action+' /OPENCPN="'+str(stock)+'" /REPORT="'+str(out)+'"'
    if failure:cmd+=' /FAILURE='+failure
    result=subprocess.run(cmd,timeout=180)
    assert result.returncode==expected,(action,result.returncode,out.read_text() if out.exists() else 'No report')
    assert out.exists(),out
    return json.loads(out.read_text(encoding='utf-8-sig'))
def engine(action,expected=0):
    script=generation()/'Lifecycle.ps1'
    out=EVIDENCE/f'installer-{len(report["checks"]):02}-{action}.json'
    r=subprocess.run([str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',str(script),'-Action',action,'-Report',str(out)],timeout=120,capture_output=True)
    assert r.returncode==expected,(action,r.returncode,r.stdout.decode(errors='replace'),r.stderr.decode(errors='replace'))
    return json.loads(out.read_text(encoding='utf-8-sig'))
def wait_ready(profile,before):
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        log=profile/'opencpn.log'
        if log.exists() and log.read_text(errors='replace').count('OnInitTimer...Finalize Canvases')>before:
            time.sleep(.6);return
        time.sleep(.1)
    raise RuntimeError('Installed app did not complete startup in shared profile')
def count_starts(profile):
    p=profile/'opencpn.log'
    return p.read_text(errors='replace').count('OnInitTimer...Finalize Canvases') if p.exists() else 0
def fixture_snapshot(profile):
    shutil.copy2(profile/'opencpn.ini',profile/'opencpn.conf')
    return fixtures.snapshot(profile)
def launch(exe,mode,title,profile,name):
    before=count_starts(profile)
    p=subprocess.Popen([str(exe),'--no_opengl',*mode]);owned.add(p.pid)
    h,pid=ui.wait_window(title,p.pid,timeout=45);wait_ready(profile,before)
    image=EVIDENCE/(name+'.png');rgb=ui.capture(h,image)
    report['screenshots'].append(image.name)
    return p,h,rgb
def close(p,h):
    ui.close(h);assert p.wait(timeout=30)==0;owned.discard(p.pid)
def wizard(stock):
    p=subprocess.Popen([str(SETUP)]);owned.add(p.pid)
    title='OpenNav X Alpha 1 Setup'
    h,_=ui.wait_window(title,p.pid,timeout=45)
    ui.capture(h,EVIDENCE/'installer-wizard-welcome.png',resize=False,screen_pixels=True)
    report['screenshots'].append('installer-wizard-welcome.png')
    get_item=ui.declare(ui.user,'GetDlgItem',ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int)
    ui.PostMessageW(get_item(h,1),0x00F5,0,0)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        if any('Select the original installed' in label for _,label in ui.children(h)):break
        time.sleep(.1)
    else:raise RuntimeError('Installer selection page did not open')
    assert any(ui.control_text(child)=='Install' for child,_ in ui.children(h)), 'Missing normal-launch Install default'
    ui.set_dialog_fields(p.pid,title,[str(stock)])
    ui.capture(h,EVIDENCE/'installer-wizard-selection.png',resize=False,screen_pixels=True)
    report['screenshots'].append('installer-wizard-selection.png')
    ui.PostMessageW(get_item(h,2),0x00F5,0,0)
    p.wait(timeout=30);owned.discard(p.pid)
    assert not INSTALL.exists()
    check('Conventional wizard opens without command-line options; default Install and path input work; Cancel changes no installation')
try:
    assert not INSTALL.exists(),'Runner must not contain a previous/user Alpha installation'
    report['display']=ui.ensure_desktop()
    subprocess.run([sys.executable,str(ROOT/'tools/build-installer-prior-fixture.py')],check=True)
    with tempfile.TemporaryDirectory(prefix='OpenNav installer ') as temp:
        temporary=Path(temp);stock=temporary/'stock OpenCPN';stock.mkdir()
        official=temporary/'official-setup.exe'
        urllib.request.urlretrieve('https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe',official)
        assert sha(official)==SETUP_HASH
        stock_report=EVIDENCE/'installer-official-prerequisite.json'
        r=subprocess.run([sys.executable,str(ROOT/'tools/install-official-opencpn-windows.py'),
                          str(official),str(stock),str(stock_report)],timeout=240)
        assert r.returncode==0,(r.returncode,stock_report.read_text() if stock_report.exists() else 'No prerequisite report')
        original=stock/'opencpn.exe';assert sha(original)==STOCK_HASH
        stock_before=inventory(stock)
        check('Official supported OpenCPN installed and exact PE executable SHA-256 verified')
        wizard(original)
        bad=temporary/'unknown';bad.mkdir();shutil.copy2(original,bad/'opencpn.exe')
        with (bad/'opencpn.exe').open('ab') as f:f.write(b'unsupported build')
        setup('Install',bad/'opencpn.exe',expected=1)
        assert not INSTALL.exists();assert inventory(stock)==stock_before
        check('Unknown executable hash refused before creating install root or changing stock')
        INSTALL.mkdir();(INSTALL/'owner.json').write_text('{"owner":"foreign fixture"}')
        (INSTALL/'keep.txt').write_text('Do not claim or change this directory')
        unowned=inventory(INSTALL)
        setup('Install',original,expected=1)
        assert inventory(INSTALL)==unowned and inventory(stock)==stock_before
        shutil.rmtree(INSTALL) # Only the explicitly created disposable fixture.
        check('Unknown installation ownership refused without adding logs or changing files')
        # Obtain wx standard profile path without initializing it; never guess it.
        loader=temporary/'locations.json'
        r=subprocess.run([str(ROOT/'build/xnav-install/opencpn.exe'),'--opennav-self-test',str(loader)],timeout=30)
        assert r.returncode==0
        profile=Path(json.loads(loader.read_text())['normal_config_directory'])
        assert str(profile).lower().startswith(os.environ['PROGRAMDATA'].lower()),profile
        if profile.exists():shutil.move(profile,temporary/'stock-created-profile')
        subprocess.run([sys.executable,str(ROOT/'tools/prepare-test-profile.py'),'--build',str(ROOT/'build/xnav-windows'),'--profile',str(profile)],check=True)
        fixtures.seed(profile)
        with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
        shutil.copy2(profile/'opencpn.conf',profile/'opencpn.ini')
        expected=fixtures.snapshot(profile);before=inventory(profile)
        prior=ROOT/'build/prior-alpha-fixture/setup/OpenNavX-Alpha1-Setup.exe'
        setup('Install',original,executable=prior)
        assert inventory(profile)==before and inventory(stock)==stock_before
        assert json.loads((generation()/'ownership.json').read_text())['version']=='0.2.0-alpha0-ci'
        old_exe=generation()/'app/opencpn.exe'
        assert sha(old_exe)!=sha(ROOT/'build/xnav-install/opencpn.exe')
        p,h,rgb=launch(old_exe,['--xnav'],'OpenNav X / OpenCPN',profile,'installer-00-prior-test-version')
        charts.reference(rgb);close(p,h);assert fixture_snapshot(profile)==expected
        prior_generation=state()['current'];before=inventory(profile)
        check('Distinct compiled prior Alpha test version installs and opens real coastline with shared fixtures')
        setup('Update',original)
        assert state()['previous']==prior_generation
        assert json.loads((generation()/'ownership.json').read_text())['version']=='0.2.0-alpha1'
        assert sha(generation()/'app/opencpn.exe')==sha(ROOT/'build/xnav-install/opencpn.exe')
        assert inventory(profile)==before and inventory(stock)==stock_before
        check('Prior test version updates to the exact Alpha candidate executable; stock/profile unchanged')
        first=state()['current'];exe=generation()/'app/opencpn.exe'
        assert not (exe.parent/'OPENNAV_PORTABLE_PREVIEW').exists()
        p,h,rgb=launch(exe,['--xnav'],'OpenNav X / OpenCPN',profile,'installer-01-xnav')
        colors=charts.reference(rgb);close(p,h);assert fixture_snapshot(profile)==expected
        check('Installed XNav starts against normal wx profile; coastline and navigation fixtures preserved')
        for mode,title,name in [('--legacy','OpenCPN / Legacy','legacy'),('--safe-mode','OpenNav Safe Mode / OpenCPN','safe')]:
            p,h,rgb=launch(exe,[mode],title,profile,'installer-02-'+name)
            charts.check(rgb,colors,'Installed '+name);close(p,h);assert fixture_snapshot(profile)==expected
        check('Installed Legacy and Safe start with charts and shared navigation/config/plugin preferences')
        # Controlled return through both product interfaces, not only separate launches.
        p,h,rgb=launch(exe,['--xnav'],'OpenNav X / OpenCPN',profile,'installer-03-before-switch')
        before=count_starts(profile);ui.click_text(p.pid,'System');ui.click_text(p.pid,'Open Legacy OpenCPN')
        assert p.wait(timeout=35)==0;owned.discard(p.pid)
        h,pid=ui.wait_window('OpenCPN / Legacy');owned.add(pid);wait_ready(profile,before)
        before=count_starts(profile);monitor=ui.monitor_process(pid);ui.click_menu(h,'Switch to XNav');ui.wait_clean_exit(monitor);owned.discard(pid)
        h,pid=ui.wait_window('OpenNav X / OpenCPN');owned.add(pid);wait_ready(profile,before)
        rgb=ui.capture(h,EVIDENCE/'installer-04-returned-xnav.png');report['screenshots'].append('installer-04-returned-xnav.png')
        charts.check(rgb,colors,'Installed XNav Legacy XNav')
        monitor=ui.monitor_process(pid);ui.close(h);ui.wait_clean_exit(monitor);owned.discard(pid)
        assert fixture_snapshot(profile)==expected
        check('Installed XNav to Legacy to XNav controlled restart retains real coastline')
        before=inventory(profile)
        damaged=generation()/'app/uidata/styles.xml';damaged.write_bytes(b'corrupt owned resource')
        custom=generation()/'app/plugins/alpha-user-preserved.txt';custom.write_text('user extension must persist')
        engine('Repair');assert state()['current']!=first and damaged.read_bytes()==b'corrupt owned resource'
        assert (generation()/'app/uidata/styles.xml').read_bytes()!=b'corrupt owned resource'
        assert (generation()/'app/plugins/alpha-user-preserved.txt').read_text()==custom.read_text()
        assert inventory(profile)==before
        check('Repair replaces corrupt owned resources in a new generation; original damaged backup and custom additions retained')
        repaired=state()['current'];setup('Update',original)
        assert state()['previous']==repaired
        assert (generation()/'app/plugins/alpha-user-preserved.txt').exists()
        check('Update preserves shared profile and user plugin additions; prior generation backed up')
        engine('Rollback');assert state()['current']==repaired and inventory(profile)==before
        check('Rollback restores exact prior generation without restoring older navigation data')
        setup('Update',original,expected=1,failure='before-commit')
        assert state()['current']==repaired and (INSTALL/'transaction.json').exists()
        setup('Update',original);assert not (INSTALL/'transaction.json').exists()
        check('Interrupted update before atomic commit retains active app and recovers on rerun')
        setup('Update',original,expected=1,failure='after-commit')
        committed=state()['current'];assert (INSTALL/'transaction.json').exists()
        setup('Repair',original);assert state()['previous']==committed and not (INSTALL/'transaction.json').exists()
        check('Interrupted post-commit shortcut publication recovers from durable journal')
        diagnostics=engine('Diagnostics');assert diagnostics['stockVerified']
        assert all(f['expected']==f['actual'] for f in diagnostics['files'])
        check('Diagnostics verifies installed hashes without collecting navigation or raw sensor data')
        # Exercise the conventional uninstall executable as well as the engine.
        maintain=generation()/'Maintain.exe';out=EVIDENCE/'installer-uninstall.json'
        result=subprocess.run([str(maintain),'/S','/ACTION=Uninstall','/REPORT='+str(out)],timeout=120)
        assert result.returncode==0
        # A normal NSIS uninstaller copies itself to a temporary process. The
        # durable engine report, not the initial wrapper exit, is completion.
        deadline=time.monotonic()+120
        while not out.exists() and time.monotonic()<deadline:time.sleep(.2)
        assert out.exists() and json.loads(out.read_text(encoding='utf-8-sig'))['status']=='passed'
        assert not (INSTALL/'state.json').exists()
        assert inventory(profile)==before and inventory(stock)==stock_before
        assert not list((INSTALL/'generations').glob('*/app/opencpn.exe')), 'Unmodified OpenNav application binaries remain'
        assert list((INSTALL/'generations').glob('*/app/plugins/alpha-user-preserved.txt')), 'Custom additions were removed'
        check('Conventional uninstaller removes verified owned app files; exact stock/profile unchanged; custom additions retained')
        p,h,rgb=launch(original,[],'OpenCPN 5.12.4',profile,'installer-05-restored-stock')
        charts.check(rgb,colors,'Untouched stock after uninstall');close(p,h)
        assert fixture_snapshot(profile)==expected
        check('Original official OpenCPN still loads charts and shared navigation data after uninstall')
        report['stock_sha256']=sha(original);report['setup_sha256']=sha(SETUP)
        report['status']='passed'
except Exception as e:
    report['status']='failed';report['error']=repr(e);raise
finally:
    for pid in owned:
        subprocess.run(['taskkill','/PID',str(pid),'/T','/F'],capture_output=True)
    (EVIDENCE/'installer-lifecycle.json').write_text(json.dumps(report,indent=2)+'\n')
