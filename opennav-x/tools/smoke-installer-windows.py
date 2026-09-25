#!/usr/bin/env python3
"""Disposable native Windows installer lifecycle, shared profile and chart gate."""
import ctypes
from contextlib import contextmanager
import configparser
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
import zipfile

if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true':
    raise SystemExit('This destructive fixture is restricted to disposable Windows CI')
ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'evidence/local';EVIDENCE.mkdir(parents=True,exist_ok=True)
PACKAGE=ROOT/'build/beta-installer';SETUP=PACKAGE/'OpenNavX-Beta1-Setup.exe'
INSTALL=Path(os.environ['LOCALAPPDATA'])/'OpenNavXAlpha1'
STOCK_HASH='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
SETUP_HASH='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
PS=Path(os.environ['WINDIR'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
owned=set();report={'status':'running','checks':[],'operations':[],'screenshots':[],'authority':'native disposable Windows / PowerShell 5.1 / NSIS'}
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
ui=module('windows-ui');fixtures=module('profile-fixtures');charts=module('chart-render-check')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(p):return {f.relative_to(p).as_posix():sha(f) for f in p.rglob('*') if f.is_file()}
def check(name):report['checks'].append(name);print(name,flush=True)
def state():return json.loads((INSTALL/'state.json').read_text(encoding='utf-8-sig'))
def generation():return INSTALL/'generations'/state()['current']
def operation_report(action):
    out=EVIDENCE/f'installer-{len(report["operations"]):02}-{action}.json'
    assert not out.exists(),'Each operation must retain its own report, including injected failures'
    report['operations'].append({'action':action,'report':out.name})
    return out
def setup(action,stock,expected=0,failure='',executable=SETUP):
    out=operation_report(action)
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
    out=operation_report(action)
    r=subprocess.run([str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',str(script),'-Action',action,'-Report',str(out)],timeout=120,capture_output=True)
    assert r.returncode==expected,(action,r.returncode,r.stdout.decode(errors='replace'),r.stderr.decode(errors='replace'))
    return json.loads(out.read_text(encoding='utf-8-sig'))
def package_engine(directory, stock, expected=1):
    out=operation_report('damaged-package')
    result=subprocess.run([str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
        '-File',str(ROOT/'installer/windows/Lifecycle.ps1'),'-Action','Update','-OpenCpn',str(stock),
        '-PackageDirectory',str(directory),'-ManifestSha256',sha(directory/'package.json'),
        '-Report',str(out)],timeout=180,capture_output=True)
    assert result.returncode==expected,(result.returncode,result.stderr.decode(errors='replace'))
    return json.loads(out.read_text(encoding='utf-8-sig'))
@contextmanager
def file_lock(path, share=1):
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    create=ui.declare(kernel,'CreateFileW',ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_ulong,
        ctypes.c_ulong,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_void_p)
    close_handle=ui.declare(kernel,'CloseHandle',ctypes.c_int,ctypes.c_void_p)
    handle=create(str(path),0x80000000,share,None,3,0,None)
    assert handle not in (None,ctypes.c_void_p(-1).value),ctypes.get_last_error()
    try:yield
    finally:assert close_handle(handle)
@contextmanager
def deny_generation_creation():
    # Deny only CreateDirectories on this disposable generations directory.
    # Existing application files remain readable; restore the exact ACL.
    directory=INSTALL/'generations'; acl_file=EVIDENCE/'installer-original-acl.txt'
    script=ROOT/'tools/installer-deny-directory.ps1'
    command=[str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
             '-File',str(script),'-Directory',str(directory),'-Saved',str(acl_file)]
    def run_acl(restore=False):
        result=subprocess.run(command+(['-Restore'] if restore else []),capture_output=True)
        if result.returncode:
            detail=(result.stdout+result.stderr).decode(errors='replace')
            (EVIDENCE/'installer-acl-error.txt').write_text(detail,encoding='utf-8')
            raise RuntimeError('Native ACL fixture failed: '+detail)
    try:
        run_acl()
        yield
    finally:
        if acl_file.exists():
            run_acl(restore=True)
            acl_file.unlink()
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
def stable_resources(profile,stock,tides=None):
    config=configparser.RawConfigParser(strict=False)
    config.read(profile/'opencpn.ini',encoding='utf-8-sig')
    expected=tides or [stock/'tcdata/harmonics-dwf-20210110-free.tcd',stock/'tcdata/HARMONICS_NO_US.IDX']
    actual=[Path(v) for _,v in config.items('TideCurrentDataSources')]
    # wx/Windows may expand the fixture's RUNNER~1 short-name alias. Require
    # identical filesystem objects, not identical spellings of the same files.
    assert len(actual)==len(expected) and all(a.samefile(e) for a,e in zip(actual,expected)),(actual,expected)
    assert all(p.is_file() for p in actual),'Retained tide sources must survive generation removal'
    for section,key,path in [('Directories','BasemapDir',stock/'gshhs'),('Directories','BaseShapefileDir',stock/'basemap_shp'),('Settings/AIS','AISAlertAudioFile',stock/'sounds/2bells.wav')]:
        assert Path(config.get(section,key)).samefile(path),(section,key,config.get(section,key))
        assert path.exists()
def launch(exe,mode,title,profile,name,stock_welcome=False):
    before=count_starts(profile)
    p=subprocess.Popen([str(exe),'--no_opengl',*mode]);owned.add(p.pid)
    if stock_welcome:
        # The official release has a different ConfigVersionString/build date.
        # Pinned MyApp::OnInit therefore presents its normal safety warning.
        # Acknowledge the visible dialog; do not bypass it by editing the profile.
        dialog,_=ui.wait_window('Welcome to OpenCPN',p.pid,timeout=45)
        image=EVIDENCE/(name+'-welcome.png')
        ui.capture(dialog,image,resize=False,screen_pixels=True)
        report['screenshots'].append(image.name)
        ui.dismiss_native_dialog(dialog,'Agree')
        check('Restored official OpenCPN safety notice acknowledged through its visible Agree button')
    h,pid=ui.wait_window(title,p.pid,timeout=45);wait_ready(profile,before)
    assert ui.IsWindowEnabled(h),'Application startup is still blocked by a modal dialog'
    image=EVIDENCE/(name+'.png');rgb=ui.capture(h,image)
    report['screenshots'].append(image.name)
    return p,h,rgb
def close(p,h):
    ui.close(h);assert p.wait(timeout=30)==0;owned.discard(p.pid)
def wizard(stock,install=False):
    p=subprocess.Popen([str(SETUP)]);owned.add(p.pid)
    title='OpenNav X Beta 1 Setup'
    h,_=ui.wait_window(title,p.pid,timeout=45)
    ui.capture(h,EVIDENCE/'installer-wizard-welcome.png',resize=False,screen_pixels=True)
    report['screenshots'].append('installer-wizard-welcome.png')
    get_item=ui.declare(ui.user,'GetDlgItem',ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int)
    def press(identifier):
        ui.SetForegroundWindow(h)
        button=get_item(h,identifier);assert button and ui.IsWindowEnabled(button)
        rect=ui.W.RECT();assert ui.GetWindowRect(button,ctypes.byref(rect))
        point=ui.W.POINT((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
        assert ui.WindowFromPoint(point)==button,'Wizard button is obscured'
        assert ui.SetCursorPos(point.x,point.y)
        ui.MouseEvent(2,0,0,0,0);time.sleep(.05);ui.MouseEvent(4,0,0,0,0)
    press(1)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        if any('Select the original installed' in label for _,label in ui.children(h)):break
        time.sleep(.1)
    else:raise RuntimeError('Installer selection page did not open')
    assert any(ui.control_text(child)=='Install' for child,_ in ui.children(h)), 'Missing normal-launch Install default'
    ui.set_dialog_fields(p.pid,title,[str(stock)])
    ui.capture(h,EVIDENCE/'installer-wizard-selection.png',resize=False,screen_pixels=True)
    report['screenshots'].append('installer-wizard-selection.png')
    if install:
        press(1)
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            assert p.poll() is None,'Installer exited before its completion page'
            for notice,_,_ in ui.windows(p.pid):
                captions=[ui.control_text(child) for child,_ in ui.children(notice)]
                if any('OpenNav setup did not complete' in caption for caption in captions):
                    raise RuntimeError('Alpha wizard reported installation failure; retained engine logs contain the cause')
            if ui.control_text(get_item(h,1)).replace('&','')=='Finish' and ui.IsWindowEnabled(get_item(h,1)):
                break
            time.sleep(.2)
        else:raise RuntimeError('Alpha wizard did not reach Finish')
        for child,caption in ui.children(h):
            if caption.replace('&','')=='Launch OpenNav X Beta 1':
                ui.SendMessageW(child,0x00F1,0,0)
        time.sleep(.5)
        ui.capture(h,EVIDENCE/'installer-wizard-installed.png',resize=False,screen_pixels=True)
        report['screenshots'].append('installer-wizard-installed.png')
        press(1)
        assert p.wait(timeout=30)==0
        check('Actual Alpha wizard Install preflight, staging and Finish complete successfully without command-line options')
    else:
        press(2)
        p.wait(timeout=30)
        assert not INSTALL.exists()
        check('Conventional wizard opens without command-line options; default Install and path input work; Cancel changes no installation')
    owned.discard(p.pid)
try:
    assert not INSTALL.exists(),'Runner must not contain a previous/user Alpha installation'
    report['display']=ui.ensure_desktop()
    try:
        subprocess.run([sys.executable,str(ROOT/'tools/build-installer-prior-fixture.py')],check=True)
    finally:
        os.environ.pop('OPENNAV_ARTIFACT_TOKEN',None)  # Do not pass it to any tested application.
    # On failure a live executable can still lock the disposable stock tree.
    # Preserve the original test exception; owned processes are stopped below.
    with tempfile.TemporaryDirectory(prefix='OpenNav installer ',ignore_cleanup_errors=True) as temp:
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
        setup('Preflight','')
        assert not INSTALL.exists() and inventory(stock)==stock_before
        check('Registry discovery handles the official build-suffixed key and verifies its exact hash without modification')
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
        wizard(original,install=True)
        assert inventory(profile)==before and inventory(stock)==stock_before
        assert not state()['previous']
        assert sha(generation()/'app/opencpn.exe')==sha(ROOT/'build/xnav-install/opencpn.exe')
        p,h,rgb=launch(generation()/'app/opencpn.exe',['--xnav'],'OpenNav X / OpenCPN',profile,'installer-00-clean-candidate')
        charts.reference(rgb);close(p,h);assert fixture_snapshot(profile)==expected
        stable_resources(profile,stock)
        before=inventory(profile)
        engine('Rollback')
        assert not (INSTALL/'state.json').exists()
        assert inventory(profile)==before and inventory(stock)==stock_before
        check('Exact candidate clean install and first-install rollback preserve stock/profile; real coastline and candidate hash verified')
        stable_resources(profile,stock)
        # A real user-selected harmonic source must remain selected; defaults
        # must never replace or append to this list during any installed mode.
        custom_tide=profile/'custom Åland harmonic fixture.tcd'
        shutil.copy2(stock/'tcdata/harmonics-dwf-20210110-free.tcd',custom_tide)
        config=configparser.RawConfigParser(strict=False)
        config.optionxform=str
        config.read(profile/'opencpn.ini',encoding='utf-8-sig')
        config['TideCurrentDataSources']={'tcds0':custom_tide.as_posix()}
        with (profile/'opencpn.ini').open('w',encoding='utf-8') as f:config.write(f)
        before=inventory(profile)
        prior=ROOT/'build/prior-alpha-fixture/setup/OpenNavX-Alpha1-Setup.exe'
        setup('Install',original,executable=prior)
        assert inventory(profile)==before and inventory(stock)==stock_before
        assert json.loads((generation()/'ownership.json').read_text())['version']=='0.2.0-alpha1'
        old_exe=generation()/'app/opencpn.exe'
        assert sha(old_exe)!=sha(ROOT/'build/xnav-install/opencpn.exe')
        p,h,rgb=launch(old_exe,['--xnav'],'OpenNav X / OpenCPN',profile,'installer-00-prior-test-version')
        charts.reference(rgb);close(p,h);assert fixture_snapshot(profile)==expected
        stable_resources(profile,stock,[custom_tide])
        prior_generation=state()['current'];before=inventory(profile)
        check('Accepted Alpha 1 release installs and opens real coastline with shared fixtures')
        setup('Update',original)
        assert state()['previous']==prior_generation
        assert json.loads((generation()/'ownership.json').read_text())['version']=='0.3.0-beta1'
        assert sha(generation()/'app/opencpn.exe')==sha(ROOT/'build/xnav-install/opencpn.exe')
        assert inventory(profile)==before and inventory(stock)==stock_before
        check('Accepted Alpha 1 updates to the exact Beta candidate executable; stock/profile unchanged')
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
        stable_resources(profile,stock,[custom_tide])
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
        active_hash=sha(generation()/'app/opencpn.exe'); state_hash=sha(INSTALL/'state.json')
        def unchanged():
            assert state()['current']==repaired and sha(INSTALL/'state.json')==state_hash
            assert sha(generation()/'app/opencpn.exe')==active_hash
            assert inventory(profile)==before and inventory(stock)==stock_before
        with file_lock(INSTALL/'transaction.lock',0):
            failure=setup('Update',original,expected=1)
            assert 'being used' in failure['error'].lower() or 'another process' in failure['error'].lower(),failure
        unchanged();check('Concurrent transaction lock refuses update; active executable/state/profile/stock remain exact')
        with deny_generation_creation():
            failure=setup('Update',original,expected=1)
            assert 'denied' in failure['error'].lower(),failure
        unchanged();check('Actual NTFS permission denial during staging preserves active installation; ACL restored')
        damaged_package=temporary/'damaged integration';damaged_package.mkdir()
        for name in ('package.json','payload.zip'):
            shutil.copy2(PACKAGE/name,damaged_package/name)
        shutil.copy2(generation()/'Maintain.exe',damaged_package/'Maintain.exe')
        with (damaged_package/'payload.zip').open('ab') as stream:stream.write(b'corrupt payload fixture')
        failure=package_engine(damaged_package,original)
        assert 'Payload ZIP integrity' in failure['error'],failure
        unchanged();check('Corrupt payload fails SHA-256 preflight without changing active installation')
        setup('Update',original,expected=1,failure='during-extraction')
        unchanged();assert not (INSTALL/'transaction.json').exists()
        check('Interrupted extraction never publishes incomplete generation or changes active state')
        # Deliberately trusted CI package with one dependency absent: valid ZIP
        # and manifest hashes are insufficient; the real staged loader must fail.
        manifest=json.loads((PACKAGE/'package.json').read_text())
        dependency=next(f['path'] for f in manifest['files'] if f['path'].lower().startswith('app/wxbase') and f['path'].endswith('.dll'))
        with zipfile.ZipFile(PACKAGE/'payload.zip') as source, zipfile.ZipFile(damaged_package/'payload.zip','w',zipfile.ZIP_DEFLATED) as target:
            for entry in source.infolist():
                if entry.filename!=dependency:target.writestr(entry,source.read(entry.filename))
        manifest['files']=[f for f in manifest['files'] if f['path']!=dependency]
        manifest['payloadSha256']=sha(damaged_package/'payload.zip')
        (damaged_package/'package.json').write_text(json.dumps(manifest))
        set_error_mode=ui.declare(ctypes.WinDLL('kernel32'),'SetErrorMode',ctypes.c_uint,ctypes.c_uint)
        old_error_mode=set_error_mode(0x8003)
        try:failure=package_engine(damaged_package,original)
        finally:set_error_mode(old_error_mode)
        assert any(word in failure['error'].lower() for word in ('self-test','loader','report')),failure
        unchanged();check('Missing required wx DLL rejected by actual staged executable loader before commit')
        with file_lock(INSTALL/'state.json'):
            setup('Update',original,expected=1)
        unchanged();assert (INSTALL/'transaction.json').exists()
        assert not list(INSTALL.glob('state.json.*.tmp'))
        check('Locked atomic state file preserves previous generation and durable recovery journal without temporary residue')
        setup('Repair',original);assert not (INSTALL/'transaction.json').exists()
        repaired=state()['current']
        check('Rerun after file-lock failure repairs and recovers normally')
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
        maintain=generation()/'Maintain.exe';out=operation_report('Uninstall')
        uninstall_command=subprocess.list2cmdline([str(maintain)])+' /S /ACTION=Uninstall /REPORT="'+str(out)+'"'
        result=subprocess.run(uninstall_command,timeout=120)
        assert result.returncode==0
        # A normal NSIS uninstaller copies itself to a temporary process. The
        # durable engine report, not the initial wrapper exit, is completion.
        deadline=time.monotonic()+120
        while not out.exists() and time.monotonic()<deadline:time.sleep(.2)
        assert out.exists() and json.loads(out.read_text(encoding='utf-8-sig'))['status']=='passed'
        assert not (INSTALL/'state.json').exists()
        assert inventory(profile)==before and inventory(stock)==stock_before
        for record in (INSTALL/'generations').glob('*/ownership.json'):
            assert not (record.parent/'app/opencpn.exe').exists(), 'Unmodified committed OpenNav binary remains'
        # Failed, unpublished stages deliberately remain diagnostic evidence;
        # the engine never recursively deletes a tree without ownership.json.
        report['unpublished_stages_retained']=sum(1 for d in (INSTALL/'generations').iterdir() if d.is_dir() and not (d/'ownership.json').exists())
        assert list((INSTALL/'generations').glob('*/app/plugins/alpha-user-preserved.txt')), 'Custom additions were removed'
        check('Conventional uninstaller removes verified owned app files; exact stock/profile unchanged; custom additions retained')
        p,h,rgb=launch(original,[],'OpenCPN 5.12.4-0',profile,'installer-05-restored-stock',stock_welcome=True)
        charts.check(rgb,colors,'Untouched stock after uninstall');close(p,h)
        assert fixture_snapshot(profile)==expected
        stable_resources(profile,stock,[custom_tide])
        check('Stock resource defaults survive every generation and uninstall; explicit custom tide selection is preserved')
        check('Original official OpenCPN still loads charts and shared navigation data after uninstall')
        report['stock_sha256']=sha(original);report['setup_sha256']=sha(SETUP)
        report['status']='passed'
except Exception as e:
    report['status']='failed';report['error']=repr(e)
    # Preserve the actual transaction failure before the disposable runner is
    # destroyed. Only bounded installation metadata, never the shared profile.
    details=EVIDENCE/'installer-engine';details.mkdir(exist_ok=True)
    for record in list((INSTALL/'logs').glob('*.log'))+[INSTALL/name for name in ('owner.json','state.json','transaction.json')]:
        if record.is_file() and not record.is_symlink() and record.stat().st_size<=4194304:
            shutil.copy2(record,details/record.name)
            if record.suffix=='.log':print(record.read_text(encoding='utf-8-sig',errors='replace'),flush=True)
    report['visible_windows']=[]
    for owner in owned:
        for handle,pid,title in ui.windows(owner):
            report['visible_windows'].append({'pid':pid,'title':title})
            try:
                name='installer-failed-'+str(len(report['visible_windows']))+'.png'
                ui.capture(handle,EVIDENCE/name,resize=False,screen_pixels=True)
                report['screenshots'].append(name)
            except Exception as capture_error:
                report.setdefault('capture_errors',[]).append(repr(capture_error))
    raise
finally:
    for pid in owned:
        subprocess.run(['taskkill','/PID',str(pid),'/T','/F'],capture_output=True)
    (EVIDENCE/'installer-lifecycle.json').write_text(json.dumps(report,indent=2)+'\n')
