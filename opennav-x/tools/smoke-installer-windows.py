#!/usr/bin/env python3
"""Disposable native Windows installer lifecycle, shared profile and chart gate."""
import argparse
import ctypes
from contextlib import contextmanager
import configparser
import hashlib
import importlib.util
import json
import os
import re
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
import zipfile

def arguments(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('staging','production'),default='staging',
                        help='Staging checks clean installation/startup/profile/rollback; production retains the full lifecycle matrix')
    parser.add_argument('--retained-package',type=Path,
                        help='Exact downloaded release set plus separately retained Retest-Support ZIP and checksums; never rebuilt')
    parser.add_argument('--prepare-only',action='store_true',
                        help='Verify and restore retained inputs, emit a preparation receipt, and run no lifecycle tests')
    parser.add_argument('--expected-commit',help='Frozen product revision, required with --retained-package')
    args=parser.parse_args(argv)
    if bool(args.retained_package)!=bool(args.expected_commit):
        parser.error('--retained-package and --expected-commit are required together')
    if args.prepare_only and not args.retained_package:
        parser.error('--prepare-only requires --retained-package')
    if args.expected_commit and not re.fullmatch('[0-9a-f]{40}',args.expected_commit):
        parser.error('--expected-commit must be a complete lowercase commit SHA')
    return args


def archive_members(archive):
    """Reject ambiguous Windows names, links and traversal before extracting."""
    entries={}; folded=set()
    for entry in archive.infolist():
        name=entry.filename; path=PurePosixPath(name)
        if (not name or '\\' in name or ':' in name or path.is_absolute() or
                '..' in path.parts or any(part.endswith((' ','.')) for part in path.parts) or
                any(re.fullmatch(r'(?i)(con|prn|aux|nul|com[1-9]|lpt[1-9])(?:\..*)?',part) for part in path.parts) or
                path.as_posix()!=name.rstrip('/') or
                (entry.external_attr>>16)&0o170000==0o120000 or name.casefold() in folded):
            raise ValueError('Unsafe or duplicate retained archive member')
        folded.add(name.casefold());entries[name]=entry
    return entries


def prepare_retained(directory,root,expected_commit):
    """Restore exact retained inputs into fresh test paths, without a compiler."""
    directory=directory.resolve()
    names=('SKAGER-Beta2-Setup.exe','SKAGER-Beta2-Portable-Recovery.zip',
           'SKAGER-Beta2-source.zip','SKAGER-Beta2-Retest-Support.zip')
    sums={}
    for line in (directory/'SHA256SUMS.txt').read_text().splitlines():
        digest,separator,name=line.partition('  ')
        if not separator or not re.fullmatch('[0-9a-f]{64}',digest) or name in sums:
            raise ValueError('Malformed or duplicate retained checksum')
        sums[name]=digest
    for name in names:
        path=directory/name
        if path.is_symlink() or not path.is_file() or sums.get(name)!=sha(path):
            raise ValueError('Retained package checksum missing or changed: '+name)
    support_paths=('build/beta-installer/package.json','build/beta-installer/payload.zip',
                   'build/production-windows/include/config.h','build/xnav-install/opencpn.exe',
                   'build/xnav-windows/include/config.h')
    fresh=('build/beta-installer','build/production-install','build/production-windows',
           'build/xnav-install','build/xnav-windows','build/developer-preview','build/retained-source')
    if any((root/name).exists() for name in fresh):
        raise ValueError('Retained retest requires fresh build input directories')
    with zipfile.ZipFile(directory/names[3]) as support:
        members=archive_members(support)
        if (not set(support_paths).issubset(members) or
                any(name not in support_paths and not name.startswith('build/xnav-install/') for name in members) or
                any(entry.is_dir() for entry in members.values())):
            raise ValueError('Retest support must contain original installer/config inputs and fixture installation only')
        manifest=json.loads(support.read(support_paths[0]))
        payload=support.read(support_paths[1])
        if manifest['commit']!=expected_commit or hashlib.sha256(payload).hexdigest()!=manifest['payloadSha256']:
            raise ValueError('Retained installer manifest identity mismatch')
        for name in members:
            target=root/name;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(support.read(name))
    with zipfile.ZipFile(directory/names[1]) as recovery:
        members=archive_members(recovery)
        prefix='SKAGER-Beta2-Portable-Recovery/'
        if any(not name.startswith(prefix) for name in members):
            raise ValueError('Unexpected recovery archive root')
        product=json.loads(recovery.read(prefix+'docs/PRODUCT_BUILD.json'))
        if (product['commit']!=expected_commit or product.get('test_fixtures') is not False or
                product.get('build_purpose')!='INSTALLED PRODUCT' or
                product.get('xnav_hardware_output_policy')!='status-only'):
            raise ValueError('Retained package must be the exact fixture-free, status-only product')
        for name,entry in members.items():
            if entry.is_dir():continue
            target=root/'build/developer-preview'/name
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(recovery.read(entry))
    preview=root/'build/developer-preview/SKAGER-Beta2-Portable-Recovery'
    if sha(preview/'app/opencpn.exe')!=product['executable_sha256']:
        raise ValueError('Retained executable differs from product identity')
    # The same installed bytes must appear in both payload forms. The recovery
    # marker is deliberately excluded from the installed integration payload.
    with zipfile.ZipFile(root/'build/beta-installer/payload.zip') as payload_zip:
        members=archive_members(payload_zip)
        records={entry['path']:entry['sha256'] for entry in manifest['files']}
        if len(records)!=len(manifest['files']) or set(records)!=set(members):
            raise ValueError('Retained payload manifest is incomplete or ambiguous')
        for name,digest in records.items():
            if hashlib.sha256(payload_zip.read(name)).hexdigest()!=digest or sha(preview/name)!=digest:
                raise ValueError('Installer payload differs from retained recovery bytes')
        if records.get('app/opencpn.exe')!=product['executable_sha256']:
            raise ValueError('Installer payload does not identify the retained executable')
    shutil.copytree(preview/'app',root/'build/production-install')
    (root/'build/production-install/OPENNAV_PORTABLE_PREVIEW').unlink(missing_ok=True)
    shutil.copy2(directory/names[0],root/'build/beta-installer'/names[0])
    shutil.copy2(directory/names[1],root/'build/developer-preview'/names[1])
    shutil.copy2(directory/names[2],root/'build/developer-preview'/names[2])
    # Source-owned installer engine remains pinned to the candidate. Current
    # test helpers may be repaired independently and their revision is recorded.
    engine_name='opennav-x/installer/windows/Lifecycle.ps1'
    with zipfile.ZipFile(directory/names[2]) as source:
        # Corresponding source legitimately retains repository symlinks. Read
        # only these two regular files; do not extract or follow source links.
        for name in ('SOURCE_REFERENCE.json',engine_name):
            selected=[entry for entry in source.infolist() if entry.filename==name]
            if len(selected)!=1 or (selected[0].external_attr>>16)&0o170000==0o120000:
                raise ValueError('Ambiguous or linked corresponding-source identity')
        reference=json.loads(source.read('SOURCE_REFERENCE.json'))
        if reference['productCommit']!=expected_commit:
            raise ValueError('Corresponding source revision differs from frozen product')
        engine_bytes=source.read(engine_name)
        if hashlib.sha256(engine_bytes).hexdigest()!=reference['files'][engine_name]['sha256']:
            raise ValueError('Corresponding installer source checksum differs')
    engine=root/'build/retained-source/installer/windows/Lifecycle.ps1'
    engine.parent.mkdir(parents=True);engine.write_bytes(engine_bytes)
    helper=root/'build/retained-source/tools/test-installer-missing-dll-selftest.ps1'
    helper.parent.mkdir();shutil.copy2(root/'tools'/helper.name,helper)
    return {'product_commit':expected_commit,'retained_sha256':{name:sums[name] for name in names}}


args=arguments()
if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true':
    raise SystemExit('This destructive fixture is restricted to disposable Windows CI')
ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'evidence/local';EVIDENCE.mkdir(parents=True,exist_ok=True)
PACKAGE=ROOT/'build/beta-installer';SETUP=PACKAGE/'SKAGER-Beta2-Setup.exe'
PRODUCT_SOURCE=ROOT/'build/retained-source' if args.retained_package else ROOT
INSTALL=Path(os.environ['LOCALAPPDATA'])/'OpenNavXAlpha1'
STOCK_HASH='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
SETUP_HASH='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
PS=Path(os.environ['WINDIR'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
PROGRAMS=Path(os.environ['APPDATA'])/'Microsoft/Windows/Start Menu/Programs'
SHORTCUTS=PROGRAMS/'SKAGER'
NEUTRAL_SHORTCUTS=PROGRAMS/'OpenNav X'
OLD_SHORTCUTS=PROGRAMS/'OpenNav X Alpha 1'
owned=set();report={'status':'running','mode':args.mode,'design_validation':'requested' if os.environ.get('SKAGER_DESIGN_VALIDATION')=='true' else 'not requested','checks':[],'operations':[],'screenshots':[],'authority':'native disposable Windows / PowerShell 5.1 / NSIS'}
def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/f'{name}.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
ui=module('windows-ui');fixtures=module('profile-fixtures');charts=module('chart-render-check')
welcome=module('installer-welcome');startup=module('startup-log')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(p):return {f.relative_to(p).as_posix():sha(f) for f in p.rglob('*') if f.is_file()}
def check(name):report['checks'].append(name);print(name,flush=True)
def chart_check(rgb,style,phase):
    # Default qualification checks chart content, not prototype conformity.
    verifier=charts.presentation if os.environ.get('SKAGER_DESIGN_VALIDATION')=='true' else charts.functional
    report.setdefault('chart_rendering',[]).append(verifier(rgb,style,'Day',phase))
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
def engine(action,expected=0,shortcut_modes='',failure=''):
    script=generation()/'Lifecycle.ps1'
    out=operation_report(action)
    command=[str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',str(script),'-Action',action,'-Report',str(out)]
    if shortcut_modes:command+=['-ShortcutModes',shortcut_modes]
    if failure:command+=['-FailurePoint',failure]
    started=time.monotonic()
    # Both direct and relocated uninstall scan every retained generation and
    # verify owned files before removal. Keep their completion bounds aligned.
    r=subprocess.run(command,timeout=600 if action=='Uninstall' else 120,capture_output=True)
    assert r.returncode==expected,(action,r.returncode,r.stdout.decode(errors='replace'),r.stderr.decode(errors='replace'))
    result=json.loads(out.read_text(encoding='utf-8-sig'))
    if action=='Uninstall' and expected==0:assert result['status']=='passed',result
    report['operations'][-1]['completion_seconds']=round(time.monotonic()-started,3)
    return result
def maintenance(action):
    maintain=generation()/'Maintain.exe';out=operation_report('Maintain-'+action)
    command=subprocess.list2cmdline([str(maintain)])+' /S /ACTION='+action+' /REPORT="'+str(out)+'"'
    result=subprocess.run(command,timeout=120)
    assert result.returncode==0
    # The bootstrap exits before the relocated uninstaller. The full matrix
    # retains many genuine generations; verified cleanup took 152 seconds on
    # native CI. Keep a bounded wait for its actual durable result, not the
    # bootstrap's exit. A timeout must never be reported as successful removal.
    completion_started=time.monotonic()
    completion_limit=600 if action=='Uninstall' else 120
    deadline=completion_started+completion_limit
    while not out.exists() and time.monotonic()<deadline:time.sleep(.2)
    assert out.exists(), 'Native maintenance did not publish its report within '+str(completion_limit)+' seconds'
    result=json.loads(out.read_text(encoding='utf-8-sig'))
    if action!='Diagnostics':assert result['status']=='passed',result
    report['operations'][-1]['completion_seconds']=round(time.monotonic()-completion_started,3)
    return result

@contextmanager
def installer_fixture():
    directory=tempfile.mkdtemp(prefix='OpenNav installer ')
    try:
        yield directory
    except BaseException:
        # A relocated maintenance child may still be running after a failed
        # wait. Preserve its stock/profile inputs for evidence and completion;
        # deleting them here manufactured a secondary missing-stock failure.
        report['retained_failed_fixture']=directory
        raise
    else:
        shutil.rmtree(directory,ignore_errors=True)

def maintenance_wizard():
    maintain=generation()/'Maintain.exe';expected_hash=sha(maintain)
    before=inventory(INSTALL)
    wrapper=subprocess.Popen([str(maintain)])
    frame,pid=ui.wait_window('SKAGER Maintenance',timeout=45);owned.add(pid)
    monitor=ui.monitor_process(pid)
    query=ui.declare(ui.kernel,'QueryFullProcessImageNameW',ctypes.c_int,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong))
    buffer=ctypes.create_unicode_buffer(32768);length=ctypes.c_ulong(len(buffer))
    assert query(monitor,0,buffer,ctypes.byref(length)) and sha(Path(buffer.value))==expected_hash,'Maintenance window is not the owned executable or its exact NSIS temporary copy'
    labels=[ui.control_text(child) for child,_ in ui.children(frame)]
    assert 'Maintain SKAGER' in labels and 'Repair' in labels,labels
    image=EVIDENCE/'installer-maintenance-title.png'
    ui.capture(frame,image,resize=False,screen_pixels=True);report['screenshots'].append(image.name)
    get_item=ui.declare(ui.user,'GetDlgItem',ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int)
    cancel=get_item(frame,2);assert cancel and ui.IsWindowEnabled(cancel)
    ui.SendMessageW(cancel,0x00F5,0,0)
    # NSIS documents 1 for the explicit Cancel button before execution. This
    # applies only to this identified maintenance page, never a completed
    # installation/repair, application close or unobserved process exit.
    # https://nsis.sourceforge.io/Docs/AppendixD.html#D.1
    assert ui.wait_exit_code(monitor)==1, 'The reviewed maintenance Cancel must report user cancellation'
    owned.discard(pid)
    assert pid!=wrapper.pid and wrapper.wait(timeout=30)==0, 'NSIS relocation wrapper failed'
    assert inventory(INSTALL)==before, 'Cancelling maintenance changed the installation'
    check('Owned maintenance wizard has version-neutral title, Repair default and non-mutating Cancel')

def package_engine(directory, stock, expected=1):
    out=operation_report('damaged-package')
    result=subprocess.run([str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
        '-File',str(PRODUCT_SOURCE/'installer/windows/Lifecycle.ps1'),'-Action','Update','-OpenCpn',str(stock),
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
        if log.exists() and startup.initialized_since(before,log.read_bytes()):
            time.sleep(.6);return
        time.sleep(.1)
    raise RuntimeError('Installed app did not complete startup in shared profile')
def startup_baseline(profile):
    p=profile/'opencpn.log'
    return p.read_bytes() if p.exists() else b''
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
def launch(exe,mode,title,profile,name,welcome_transition=None):
    proof=None
    if welcome_transition:
        ownership=None
        if welcome_transition!='candidate-to-stock':
            assert exe.samefile(generation()/'app/opencpn.exe')
            ownership=json.loads((generation()/'ownership.json').read_text(encoding='utf-8-sig'))
        else:
            assert exe.samefile(original) and not (INSTALL/'state.json').exists()
        accepted=json.loads((ROOT/'tools/accepted-beta1.lock.json').read_text())
        candidate=json.loads((ROOT/'build/developer-preview/SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json').read_text())
        proof=welcome.version_transition(welcome_transition,sha(exe),ownership,accepted['commit'],candidate['commit'])
    before=startup_baseline(profile)
    p=subprocess.Popen([str(exe),'--no_opengl',*mode]);owned.add(p.pid)
    if proof:
        # Pinned MyApp::OnInit shows this caution whenever ConfigVersionString
        # changes, including the genuine accepted Beta1/candidate transition.
        # Preserve and acknowledge that visible notice; never edit its INI flags.
        dialog,_=ui.wait_window('Welcome to OpenCPN',p.pid,timeout=45)
        assert p.poll() is None and sha(exe)==proof['executableSha256']
        query=ui.declare(ui.kernel,'QueryFullProcessImageNameW',ctypes.c_int,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong))
        buffer=ctypes.create_unicode_buffer(32768);length=ctypes.c_ulong(len(buffer))
        assert query(int(p._handle),0,buffer,ctypes.byref(length)) and Path(buffer.value).samefile(exe),'Welcome belongs to another executable'
        def verify_notice():
            assert p.poll() is None and sha(exe)==proof['executableSha256']
            buttons=[]
            for child,_ in ui.children(dialog):
                kind=ctypes.create_unicode_buffer(128);ui.GetClassNameW(child,kind,len(kind))
                if kind.value=='Button':
                    assert ui.IsWindowEnabled(child)
                    buttons.append(ui.control_text(child))
            welcome.validate_notice(p.pid,dialog,ui.windows(p.pid),buttons)
        verify_notice()
        image=EVIDENCE/(name+'-welcome.png')
        ui.capture(dialog,image,resize=False,screen_pixels=True)
        report['screenshots'].append(image.name)
        verify_notice()
        ui.dismiss_native_dialog(dialog,'Agree')
        report.setdefault('versionNotices',[]).append(dict(proof,pid=p.pid,screenshot=image.name,transition=welcome_transition))
        check('Expected '+welcome_transition+' safety notice captured and acknowledged through its visible Agree button')
    h,pid=ui.wait_window(title,p.pid,timeout=45);wait_ready(profile,before)
    assert ui.IsWindowEnabled(h),'Application startup is still blocked by a modal dialog'
    image=EVIDENCE/(name+'.png');rgb=ui.capture(h,image)
    report['screenshots'].append(image.name)
    return p,h,rgb
def close(p,h):
    ui.close(h);assert p.wait(timeout=30)==0;owned.discard(p.pid)
def wizard(stock,install=False):
    p=subprocess.Popen([str(SETUP)]);owned.add(p.pid)
    title='SKAGER Beta 2 Setup'
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
        for expected_page, image_name in [('Your recovery backup','backup'),('Start menu shortcuts','options'),('Ready to install','ready')]:
            press(1)
            deadline=time.monotonic()+45
            while time.monotonic()<deadline:
                if any(expected_page in label for _,label in ui.children(h)): break
                time.sleep(.1)
            else: raise RuntimeError('Beta 2 wizard page missing: '+expected_page)
            image=EVIDENCE/('installer-wizard-'+image_name+'.png')
            ui.capture(h,image,resize=False,screen_pixels=True); report['screenshots'].append(image.name)
        press(1)
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            assert p.poll() is None,'Installer exited before its completion page'
            for notice,_,_ in ui.windows(p.pid):
                captions=[ui.control_text(child) for child,_ in ui.children(notice)]
                if any('SKAGER setup did not complete' in caption for caption in captions):
                    raise RuntimeError('Beta 2 wizard reported installation failure; retained engine logs contain the cause')
            if ui.control_text(get_item(h,1)).replace('&','')=='Finish' and ui.IsWindowEnabled(get_item(h,1)):
                break
            time.sleep(.2)
        else:raise RuntimeError('Beta 2 wizard did not reach Finish')
        for child,caption in ui.children(h):
            if caption.replace('&','')=='Launch SKAGER Beta 2':
                ui.SendMessageW(child,0x00F1,0,0)
        time.sleep(.5)
        ui.capture(h,EVIDENCE/'installer-wizard-installed.png',resize=False,screen_pixels=True)
        report['screenshots'].append('installer-wizard-installed.png')
        press(1)
        assert p.wait(timeout=30)==0
        check('Actual Beta 2 wizard detection, recovery, shortcuts, ready, validation and Finish pass without command-line options')
    else:
        press(2)
        p.wait(timeout=30)
        assert not INSTALL.exists()
        check('Conventional wizard opens without command-line options; default Install and path input work; Cancel changes no installation')
    owned.discard(p.pid)
if args.prepare_only:
    prepared=prepare_retained(args.retained_package,ROOT,args.expected_commit)
    prepared.update(status='prepared',scope='Inputs verified and restored only; no native lifecycle acceptance',
                    harness_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
                    test_source_sha256=sha(Path(__file__)))
    (EVIDENCE/'installer-retained-inputs.json').write_text(json.dumps(prepared,indent=2)+'\n')
    raise SystemExit(0)

try:
    report['harness_commit']=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip()
    report['test_source_sha256']=sha(Path(__file__))
    if args.retained_package:
        report.update(prepare_retained(args.retained_package,ROOT,args.expected_commit))
    product=json.loads((ROOT/'build/developer-preview/SKAGER-Beta2-Portable-Recovery/docs/PRODUCT_BUILD.json').read_text())
    assert product.get('xnav_hardware_output_policy')=='status-only'
    report['product_commit']=product['commit']
    assert not INSTALL.exists(),'Runner must not contain a previous/user Alpha installation'
    report['display']=ui.ensure_desktop()
    try:
        if args.mode=='production':
            subprocess.run([sys.executable,str(ROOT/'tools/build-installer-prior-fixture.py')],check=True)
            # Fetch both immutable historical packages before deleting the Actions
            # credential. No tested installer/application may inherit that token.
            subprocess.run([sys.executable,str(ROOT/'tools/build-installer-prior-fixture.py'),
                            '--early-beta2-layout'],check=True)
    finally:
        os.environ.pop('OPENNAV_ARTIFACT_TOKEN',None)  # Do not pass it to any tested application.
    # On failure a live executable can still lock the disposable stock tree.
    # Preserve the original test exception; owned processes are stopped below.
    with installer_fixture() as temp:
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
        assert not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
        SHORTCUTS.mkdir();foreign_link=SHORTCUTS/'Skager.lnk'
        foreign_link.write_bytes(b'foreign shortcut content must not be claimed')
        foreign_before=inventory(SHORTCUTS)
        try:
            failure=setup('Install',original,expected=1)
            assert 'no verified SKAGER owner' in failure['error'],failure
            assert inventory(SHORTCUTS)==foreign_before and not INSTALL.exists()
            assert inventory(stock)==stock_before
        finally:
            foreign_link.unlink();SHORTCUTS.rmdir()
        check('A foreign SKAGER Start-menu group is refused before root creation; its same-name shortcut and stock remain unchanged')
        INSTALL.mkdir();(INSTALL/'owner.json').write_text('{"owner":"foreign fixture"}')
        (INSTALL/'keep.txt').write_text('Do not claim or change this directory')
        unowned=inventory(INSTALL)
        setup('Install',original,expected=1)
        assert inventory(INSTALL)==unowned and inventory(stock)==stock_before
        shutil.rmtree(INSTALL) # Only the explicitly created disposable fixture.
        check('Unknown installation ownership refused without adding logs or changing files')
        # Obtain wx standard profile path without initializing it; never guess it.
        loader=temporary/'locations.json'
        r=subprocess.run([str(ROOT/'build/production-install/opencpn.exe'),'--opennav-self-test',str(loader)],timeout=30)
        assert r.returncode==0
        profile=Path(json.loads(loader.read_text())['normal_config_directory'])
        assert str(profile).lower().startswith(os.environ['PROGRAMDATA'].lower()),profile
        if profile.exists():shutil.move(profile,temporary/'stock-created-profile')
        subprocess.run([sys.executable,str(ROOT/'tools/prepare-test-profile.py'),'--build',str(ROOT/'build/production-windows'),'--profile',str(profile)],check=True)
        fixtures.seed(profile)
        with (profile/'opencpn.conf').open('a') as f:f.write('\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n')
        shutil.copy2(profile/'opencpn.conf',profile/'opencpn.ini')
        expected=fixtures.snapshot(profile);before=inventory(profile)
        # Reject against an existing seeded profile, and inspect the actual
        # unsupported tree passed to Setup rather than only its supported sibling.
        bad=temporary/'unknown';bad.mkdir();shutil.copy2(original,bad/'opencpn.exe')
        with (bad/'opencpn.exe').open('ab') as f:f.write(b'unsupported build')
        (bad/'user-owned').mkdir();(bad/'user-owned/keep.txt').write_bytes(b'Preserve unsupported installation contents.\n')
        rejected_before=inventory(bad)
        assert before and rejected_before and sha(bad/'opencpn.exe')!=STOCK_HASH
        assert not INSTALL.exists() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
        failure=setup('Install',bad/'opencpn.exe',expected=1)
        assert failure['status']=='failed' and 'Unsupported OpenCPN executable.' in failure['error'],failure
        rejected_after=inventory(bad);profile_after=inventory(profile)
        assert rejected_after==rejected_before,'Unsupported installation tree changed during rejection'
        assert profile_after==before,'Existing shared profile changed during unsupported-build rejection'
        assert inventory(stock)==stock_before
        assert not INSTALL.exists() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
        report['unsupported_build_preservation']={
            'status':'passed','setup_sha256':sha(SETUP),'test_source_sha256':sha(Path(__file__)),
            'package_manifest_sha256':sha(PACKAGE/'package.json'),
            'rejected_tree_files':len(rejected_before),'profile_files':len(before),
            'rejected_tree_before_sha256':hashlib.sha256(json.dumps(rejected_before,sort_keys=True).encode()).hexdigest(),
            'rejected_tree_after_sha256':hashlib.sha256(json.dumps(rejected_after,sort_keys=True).encode()).hexdigest(),
            'profile_before_sha256':hashlib.sha256(json.dumps(before,sort_keys=True).encode()).hexdigest(),
            'profile_after_sha256':hashlib.sha256(json.dumps(profile_after,sort_keys=True).encode()).hexdigest(),
            'install_root_absent':True,'all_shortcut_groups_absent':True}
        check('Unknown executable hash refused; rejected tree and existing profile unchanged; install root and all shortcut groups absent')
        # Stock plugins are user-owned inputs. A legacy TLS DLL copied from
        # there must refuse a fresh candidate without deleting the source.
        stock_plugins=stock/'plugins';created_stock_plugins=not stock_plugins.exists()
        stock_plugins.mkdir(exist_ok=True)
        for name in ('libeay32.dll','ssleay32.dll'):
            legacy=stock_plugins/name
            assert not legacy.exists()
            legacy.write_bytes(('unmanaged stock plugin '+name).encode())
            stock_with_legacy=inventory(stock)
            refused_cleanly=False
            try:
                failure=setup('Install',original,expected=1)
                assert 'Unsupported legacy TLS runtime dependency in candidate' in failure['error'],failure
                assert inventory(stock)==stock_with_legacy and legacy.is_file()
                assert inventory(profile)==before and not (INSTALL/'state.json').exists()
                assert not (INSTALL/'transaction.json').exists()
                assert not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
                refused_cleanly=True
            finally:
                legacy.unlink()
                # A rejected first install may leave only its disposable,
                # unpublished owner/recovery/staging files behind.
                if refused_cleanly and INSTALL.exists():
                    shutil.rmtree(INSTALL)
        if created_stock_plugins:stock_plugins.rmdir()
        assert inventory(stock)==stock_before and not INSTALL.exists()
        check('Fresh install refuses each unmanaged stock plugin legacy TLS DLL and preserves stock/profile without publication')
        wizard(original,install=True)
        assert inventory(profile)==before and inventory(stock)==stock_before
        assert not state()['previous']
        assert (SHORTCUTS/'Skager.lnk').is_file() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
        assert sha(generation()/'app/opencpn.exe')==sha(ROOT/'build/production-install/opencpn.exe')
        assert sha(generation()/'Lifecycle.ps1')==sha(PRODUCT_SOURCE/'installer/windows/Lifecycle.ps1')
        p,h,rgb=launch(generation()/'app/opencpn.exe',['--xnav'],'SKAGER / OpenCPN',profile,'installer-00-clean-candidate')
        chart_check(rgb,'XNav','Clean installed XNav');close(p,h);assert fixture_snapshot(profile)==expected
        stable_resources(profile,stock)
        before=inventory(profile)
        engine('Rollback')
        assert not (INSTALL/'state.json').exists()
        assert not list((INSTALL/'generations').glob('*/app/opencpn.exe'))
        assert not (INSTALL/'transaction.json').exists()
        assert inventory(profile)==before and inventory(stock)==stock_before
        check('Exact candidate clean install and first-install rollback preserve stock/profile; real coastline and candidate hash verified')
        stable_resources(profile,stock)
        assert not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
        if args.mode=='production':
            # Early Beta 2 had the same 0.4 version but its immutable maintainer
            # only knew the historical Alpha 1 folder. Test the exact published
            # bytes; neither relabel a current package nor patch retained engines.
            early_lock=json.loads((ROOT/'tools/early-beta2-layout.lock.json').read_text())
            early_setup=ROOT/'build/prior-beta2-layout-fixture/setup'/early_lock['setupName']
            before=inventory(profile)
            setup('Install',original,executable=early_setup)
            early_generation=state()['current'];early_owned=inventory(generation())
            early_record=json.loads((generation()/'ownership.json').read_text())
            assert early_record['version']=='0.4.0-beta2' and early_record['commit']==early_lock['commit']
            assert 'shellLayout' not in early_record
            assert (OLD_SHORTCUTS/'Maintain OpenNav.lnk').is_file() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            setup('Update',original)
            assert state()['previous']==early_generation
            assert json.loads((generation()/'ownership.json').read_text())['shellLayout']=='OpenNavX.SkagerStartMenu.1'
            assert (SHORTCUTS/'Maintain Skager.lnk').is_file() and not OLD_SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            assert inventory(profile)==before and inventory(stock)==stock_before
            engine('Rollback',expected=1,failure='after-commit')
            assert state()['current']==early_generation and inventory(generation())==early_owned
            assert (INSTALL/'transaction.json').is_file()
            assert not OLD_SHORTCUTS.exists() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            early_diagnostics=maintenance('Diagnostics')
            assert (INSTALL/'transaction.json').is_file() and not OLD_SHORTCUTS.exists()
            assert early_diagnostics['stockVerified'] and early_diagnostics['state']['current']==early_generation
            assert all(f['expected']==f['actual'] for f in early_diagnostics['files'])
            maintenance('Uninstall')
            assert not (INSTALL/'transaction.json').exists()
            assert not (INSTALL/'state.json').exists() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
            assert inventory(profile)==before and inventory(stock)==stock_before
            check('Exact historical-layout Beta 2 updates and rolls back with byte-identical old engine; original Maintain diagnostics and uninstall leave neither group')
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
            prior=ROOT/'build/prior-alpha-fixture/setup/OpenNavX-Beta1-Setup.exe'
            setup('Install',original,executable=prior)
            assert inventory(profile)==before and inventory(stock)==stock_before
            assert json.loads((generation()/'ownership.json').read_text())['version']=='0.3.0-beta1'
            assert (OLD_SHORTCUTS/'Maintain OpenNav.lnk').is_file() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            old_exe=generation()/'app/opencpn.exe'
            assert sha(old_exe)!=sha(ROOT/'build/production-install/opencpn.exe')
            p,h,rgb=launch(old_exe,['--xnav'],'OpenNav X / OpenCPN',profile,'installer-00-prior-test-version',welcome_transition='candidate-to-beta1')
            charts.reference(rgb);close(p,h);assert fixture_snapshot(profile)==expected
            stable_resources(profile,stock,[custom_tide])
            prior_generation=state()['current'];prior_owned=inventory(generation());before=inventory(profile)
            check('Accepted Beta 1 release installs and opens real coastline with shared fixtures')
            setup('Update',original)
            assert state()['previous']==prior_generation
            assert json.loads((generation()/'ownership.json').read_text())['version']=='0.4.0-beta2'
            assert json.loads((generation()/'ownership.json').read_text())['shellLayout']=='OpenNavX.SkagerStartMenu.1'
            assert sha(generation()/'app/opencpn.exe')==sha(ROOT/'build/production-install/opencpn.exe')
            assert inventory(profile)==before and inventory(stock)==stock_before
            assert (SHORTCUTS/'Skager.lnk').is_file() and not OLD_SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            check('Accepted Beta 1 updates to the exact Beta 2 candidate executable; stock/profile unchanged')
            recoveries=list((INSTALL/'recovery').glob('*.json'))
            assert recoveries,'Missing durable before-state recovery set'
            latest=max(recoveries,key=lambda p:p.stat().st_mtime_ns)
            recovery=json.loads(latest.read_text(encoding='utf-8-sig'))
            assert recovery['before']['current']==prior_generation
            assert recovery['stock']['sha256']==STOCK_HASH and recovery['nextVersion']=='0.4.0-beta2'
            check('Versioned recovery record identifies exact Beta 1 generation, stock hash and Beta 2 target before update')
            # Exercise the genuine older immutable engine after rollback, not a
            # same-version mock. Its original group must remain usable and its
            # files byte-identical; then migrate forward again using current Setup.
            engine('Rollback')
            assert state()['current']==prior_generation and inventory(generation())==prior_owned
            assert (OLD_SHORTCUTS/'Maintain OpenNav.lnk').is_file() and not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            prior_diagnostics=maintenance('Diagnostics')
            assert prior_diagnostics['stockVerified'] and prior_diagnostics['state']['current']==prior_generation
            assert all(f['expected']==f['actual'] for f in prior_diagnostics['files'])
            assert inventory(profile)==before and inventory(stock)==stock_before
            check('Genuine Beta 1 rollback restores its historical group and unchanged original maintainer; native diagnostics work')
            # Interrupt after publishing usable neutral links, before old cleanup.
            setup('Update',original,expected=1,failure='after-shortcuts')
            committed_migration=state()['current']
            assert committed_migration!=prior_generation and (INSTALL/'transaction.json').exists()
            assert (SHORTCUTS/'Skager.lnk').is_file() and (OLD_SHORTCUTS/'OpenNav X.lnk').is_file()
            setup('Repair',original)
            assert state()['previous']==committed_migration and not (INSTALL/'transaction.json').exists()
            assert (SHORTCUTS/'Maintain Skager.lnk').is_file() and not OLD_SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            assert inventory(profile)==before and inventory(stock)==stock_before
            check('Genuine Beta 1 to Beta 2 interrupted group migration recovers from committed state and removes only old owned links')
            maintenance_wizard()
            assert inventory(profile)==before and inventory(stock)==stock_before
            first=state()['current'];exe=generation()/'app/opencpn.exe'
            assert not (exe.parent/'OPENNAV_PORTABLE_PREVIEW').exists()
            p,h,rgb=launch(exe,['--xnav'],'SKAGER / OpenCPN',profile,'installer-01-xnav',welcome_transition='beta1-to-candidate')
            chart_check(rgb,'XNav','Installed XNav');close(p,h);assert fixture_snapshot(profile)==expected
            check('Installed XNav starts against normal wx profile; coastline and navigation fixtures preserved')
            for mode,title,name in [('--legacy','SKAGER Legacy / OpenCPN','legacy'),('--safe-mode','SKAGER Safe Mode / OpenCPN','safe')]:
                p,h,rgb=launch(exe,[mode],title,profile,'installer-02-'+name)
                chart_check(rgb,'Standard','Installed '+name);close(p,h);assert fixture_snapshot(profile)==expected
            check('Installed Legacy and Safe start with charts and shared navigation/config/plugin preferences')
            # Controlled return through both product interfaces, not only separate launches.
            p,h,rgb=launch(exe,['--xnav'],'SKAGER / OpenCPN',profile,'installer-03-before-switch')
            chart_check(rgb,'XNav','Installed XNav before mode switch')
            before=startup_baseline(profile);ui.open_system(p.pid);ui.click_text(p.pid,'Open Legacy OpenCPN')
            assert p.wait(timeout=35)==0;owned.discard(p.pid)
            h,pid=ui.wait_window('SKAGER Legacy / OpenCPN');owned.add(pid);wait_ready(profile,before)
            rgb=ui.capture(h,EVIDENCE/'installer-03-switched-legacy.png');report['screenshots'].append('installer-03-switched-legacy.png')
            chart_check(rgb,'Standard','Installed XNav to Legacy')
            before=startup_baseline(profile);monitor=ui.monitor_process(pid);ui.click_menu(h,'Switch to SKAGER');ui.wait_clean_exit(monitor);owned.discard(pid)
            h,pid=ui.wait_window('SKAGER / OpenCPN');owned.add(pid);wait_ready(profile,before)
            rgb=ui.capture(h,EVIDENCE/'installer-04-returned-xnav.png');report['screenshots'].append('installer-04-returned-xnav.png')
            chart_check(rgb,'XNav','Installed XNav Legacy XNav')
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
            engine('Repair',shortcut_modes='xnav')
            shortcut_root=SHORTCUTS
            assert not OLD_SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists()
            assert (shortcut_root/'Skager.lnk').exists() and (shortcut_root/'Maintain Skager.lnk').exists()
            assert not (shortcut_root/'OpenCPN Legacy.lnk').exists() and not (shortcut_root/'Skager Safe Mode.lnk').exists()
            assert state()['shortcutModes']==['xnav']
            engine('Repair')
            assert state()['shortcutModes']==['xnav'] and not (shortcut_root/'OpenCPN Legacy.lnk').exists()
            check('Optional shortcuts respect explicit choices; retained-package repair preserves preferences')
            engine('Repair',shortcut_modes='xnav,legacy,safe')
            assert (shortcut_root/'OpenCPN Legacy.lnk').exists() and (shortcut_root/'Skager Safe Mode.lnk').exists()
            assert inventory(profile)==before
            check('Legacy and Safe shortcuts can be restored without altering shared navigation data')
            repaired=state()['current'];setup('Update',original)
            assert state()['previous']==repaired
            assert (generation()/'app/plugins/alpha-user-preserved.txt').exists()
            check('Update preserves shared profile and user plugin additions; prior generation backed up')
            previous=INSTALL/'generations'/repaired
            previous_ownership=previous/'ownership.json'
            saved_ownership=previous_ownership.read_bytes()
            rollback_state=sha(INSTALL/'state.json');shortcut_before=inventory(SHORTCUTS)
            malformed=json.loads(saved_ownership);malformed['shellLayout']=None
            previous_ownership.write_text(json.dumps(malformed),encoding='utf-8')
            previous_files=inventory(previous);current_files=inventory(generation())
            try:
                failure=engine('Rollback',expected=1)
                assert 'Unknown generation Start-menu layout' in failure['error'],failure
                assert sha(INSTALL/'state.json')==rollback_state and inventory(SHORTCUTS)==shortcut_before
                assert inventory(previous)==previous_files and inventory(generation())==current_files
                assert inventory(profile)==before and inventory(stock)==stock_before
                assert not (INSTALL/'transaction.json').exists()
            finally:
                previous_ownership.write_bytes(saved_ownership)
            check('Unknown explicit rollback layout refuses before state/journal publication; owned generations, shortcuts and user data remain unchanged')
            rollback_marker=previous/'app/OPENNAV_PORTABLE_PREVIEW'
            assert not rollback_marker.exists()
            rollback_marker.write_text('unowned inherited rollback sentinel')
            previous_files=inventory(previous); current_files=inventory(generation())
            rollback_state=sha(INSTALL/'state.json')
            try:
                failure=engine('Rollback',expected=1)
                assert 'portable profile marker' in failure['error'],failure
                assert sha(INSTALL/'state.json')==rollback_state
                assert inventory(previous)==previous_files and inventory(generation())==current_files
                assert inventory(profile)==before and inventory(stock)==stock_before
                assert not (INSTALL/'transaction.json').exists()
            finally:
                rollback_marker.unlink()
            check('Rollback refuses an unowned portable marker in the previous generation without altering either generation, state or user data')
            # Rollback is recovery of an existing generation, not acceptance of a
            # newly staged candidate. An unowned DLL there must not block recovery.
            rollback_legacy=previous/'app/plugins/ssleay32.dll'
            assert not rollback_legacy.exists()
            shutil.copy2(previous/'app/zlib1.dll',rollback_legacy)
            rollback_legacy_hash=sha(rollback_legacy)
            previous_files=inventory(previous)
            try:
                engine('Rollback')
                assert state()['current']==repaired and inventory(generation())==previous_files
                assert sha(rollback_legacy)==rollback_legacy_hash
                assert inventory(profile)==before and inventory(stock)==stock_before
            finally:
                rollback_legacy.unlink()
            check('Rollback restores exact prior generation with an unmanaged legacy TLS addition and leaves navigation data unchanged')
            active_hash=sha(generation()/'app/opencpn.exe'); state_hash=sha(INSTALL/'state.json')
            shortcut_before=inventory(SHORTCUTS)
            def unchanged():
                assert state()['current']==repaired and sha(INSTALL/'state.json')==state_hash
                assert sha(generation()/'app/opencpn.exe')==active_hash
                assert inventory(profile)==before and inventory(stock)==stock_before
            for source in ('stock','active'):
                for name in ('libeay32.dll','ssleay32.dll'):
                    legacy=(stock/'plugins'/name) if source=='stock' else (generation()/'app/plugins'/name)
                    assert not legacy.exists()
                    legacy.parent.mkdir(parents=True,exist_ok=True)
                    legacy.write_bytes(('unmanaged '+source+' '+name).encode())
                    stock_with_legacy=inventory(stock);active_with_legacy=inventory(generation())
                    try:
                        for action in ('Update','Repair'):
                            failure=setup(action,original,expected=1)
                            assert 'Unsupported legacy TLS runtime dependency in candidate' in failure['error'],failure
                            assert name in failure['error'],failure
                            assert inventory(profile)==before
                            assert inventory(stock)==stock_with_legacy
                            assert inventory(generation())==active_with_legacy
                            assert legacy.is_file(), 'Refusal silently removed unmanaged TLS file'
                            assert sha(INSTALL/'state.json')==state_hash
                            assert inventory(SHORTCUTS)==shortcut_before
                            assert not (INSTALL/'transaction.json').exists()
                    finally:
                        legacy.unlink()
            assert inventory(stock)==stock_before
            unchanged()
            check('Update/repair refuse each stock or active unmanaged legacy TLS DLL; current installation and source bytes survive')
            # Unknown user additions are normally preserved, but cannot turn an
            # installed generation back into a portable or developer distribution.
            for relative, action, error in (
                    ('app/OPENNAV_PORTABLE_PREVIEW','Update','portable profile marker'),
                    ('Run-XNav-Demo.cmd','Repair','Developer/demo content refused'),
                    ('app/OPENNAV_ROUTE_FIXTURE','Update','Developer/demo content refused')):
                addition=generation()/relative
                assert not addition.exists()
                addition.write_text('unowned inherited sentinel; must never reach a committed product')
                previous_files=inventory(generation())
                try:
                    failure=setup(action,original,expected=1)
                    assert error in failure['error'],failure
                    unchanged()
                    assert inventory(generation())==previous_files,'Rejected inheritance changed old generation'
                    assert not (INSTALL/'transaction.json').exists(),'Rejected content reached publication journal'
                finally:
                    # Remove only the disposable sentinel inserted by this case.
                    addition.unlink()
            unchanged();check('Update/repair reject inherited portable marker, Demo launcher and route fixture before publication; prior generation/state/stock/profile unchanged')
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
            # Valid ZIP/manifest hashes are insufficient: dependency closure must
            # reject this unpublished stage before invoking the application loader.
            manifest=json.loads((PACKAGE/'package.json').read_text())
            # Use the base library, not whichever optional wx library sorts first,
            # so the separate real-loader check below also has a required import.
            dependencies=[f['path'] for f in manifest['files'] if f['path'].lower()=='app/wxbase32u_vc14x.dll']
            assert len(dependencies)==1,dependencies
            dependency=dependencies[0]
            with zipfile.ZipFile(PACKAGE/'payload.zip') as source, zipfile.ZipFile(damaged_package/'payload.zip','w',zipfile.ZIP_DEFLATED) as target:
                for entry in source.infolist():
                    if entry.filename!=dependency:target.writestr(entry,source.read(entry.filename))
            manifest['files']=[f for f in manifest['files'] if f['path']!=dependency]
            manifest['payloadSha256']=sha(damaged_package/'payload.zip')
            (damaged_package/'package.json').write_text(json.dumps(manifest))
            stages_before=set((INSTALL/'generations').iterdir())
            failure=package_engine(damaged_package,original)
            stages_after=set((INSTALL/'generations').iterdir())-stages_before
            assert len(stages_after)==1,stages_after
            stage=stages_after.pop().resolve()
            assert stage.is_dir() and len(stage.name)==32 and all(c in '0123456789abcdef' for c in stage.name)
            assert not (stage/'ownership.json').exists() and not (INSTALL/'transaction.json').exists()
            prefix='Missing app-local PE import '+Path(dependency).name.lower()+' in '
            assert failure['status']=='failed' and failure['action']=='Update' and failure['error'].startswith(prefix),failure
            importer=Path(failure['error'][len(prefix):]).resolve()
            assert importer.is_file() and importer.is_relative_to(stage/'app'),failure
            assert not (stage/dependency).exists()
            for _ in range(10):
                assert not any(title=='opencpn.exe - System Error' for _,_,title in ui.windows()),'Loader failure left an operating-system modal dialog'
                time.sleep(.1)
            unchanged();check('Exact missing wx base import rejected before loader or commit, without OS modal residue')
            # Exercise the unchanged production SelfTest separately on this private
            # failed stage. Never bypass dependency closure in the installer path.
            loader_report=EVIDENCE/'installer-missing-dll-selftest.json'
            helper=PRODUCT_SOURCE/'tools/test-installer-missing-dll-selftest.ps1'
            with (EVIDENCE/'installer-missing-dll-selftest.log').open('wb') as output:
                child=subprocess.Popen([str(PS),'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
                    '-File',str(helper),'-Stage',str(stage),'-ExpectedExecutableSha256',sha(ROOT/'build/production-install/opencpn.exe'),
                    '-MissingDependency',Path(dependency).name,'-Commit',manifest['commit'],'-Version',manifest['version'],
                    '-Report',str(loader_report)],stdout=output,stderr=subprocess.STDOUT)
                try:code=child.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True,timeout=15)
                    child.wait(timeout=10)
                    raise
            assert code==0,'Missing-DLL SelfTest proof failed; inspect retained log/receipt'
            loader=json.loads(loader_report.read_text(encoding='utf-8-sig'))
            assert loader['status']=='passed' and loader['error']=='Staged executable self-test failed: -1073741515',loader
            assert loader['sourceSha256']==sha(PRODUCT_SOURCE/'installer/windows/Lifecycle.ps1') and loader['executableSha256']==sha(ROOT/'build/production-install/opencpn.exe'),loader
            assert loader['initialErrorMode']==0 and loader['restoredErrorMode']==0,loader
            for _ in range(10):
                assert not any(title=='opencpn.exe - System Error' for _,_,title in ui.windows()),'SelfTest left an operating-system modal dialog'
                time.sleep(.1)
            unchanged();assert not (stage/'ownership.json').exists() and not (INSTALL/'transaction.json').exists()
            check('Actual production SelfTest suppresses missing-DLL OS dialog and restores error mode in separate failed-stage process')
            # A correctly hashed fixture-enabled executable is still forbidden in
            # the installed product. Exercise the actual native loader identity,
            # not merely a package label or cache option.
            fixture_exe=ROOT/'build/xnav-install/opencpn.exe'
            assert fixture_exe.is_file() and sha(fixture_exe)!=sha(ROOT/'build/production-install/opencpn.exe')
            fixture_bytes=fixture_exe.read_bytes()
            manifest=json.loads((PACKAGE/'package.json').read_text())
            with zipfile.ZipFile(PACKAGE/'payload.zip') as source, zipfile.ZipFile(damaged_package/'payload.zip','w',zipfile.ZIP_DEFLATED) as target:
                for entry in source.infolist():
                    target.writestr(entry,fixture_bytes if entry.filename=='app/opencpn.exe' else source.read(entry.filename))
            for entry in manifest['files']:
                if entry['path']=='app/opencpn.exe':entry['sha256']=sha(fixture_exe)
            manifest['payloadSha256']=sha(damaged_package/'payload.zip')
            (damaged_package/'package.json').write_text(json.dumps(manifest))
            failure=package_engine(damaged_package,original)
            assert 'Developer/test-fixture executable refused' in failure['error'],failure
            unchanged();check('Correctly hashed native fixture-enabled executable is refused before product publication')
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
            maintenance('Uninstall')
            assert not (INSTALL/'state.json').exists()
            assert inventory(profile)==before and inventory(stock)==stock_before
            for record in (INSTALL/'generations').glob('*/ownership.json'):
                assert not (record.parent/'app/opencpn.exe').exists(), 'Unmodified committed OpenNav binary remains'
            # Failed, unpublished stages deliberately remain diagnostic evidence;
            # the engine never recursively deletes a tree without ownership.json.
            report['unpublished_stages_retained']=sum(1 for d in (INSTALL/'generations').iterdir() if d.is_dir() and not (d/'ownership.json').exists())
            assert list((INSTALL/'generations').glob('*/app/plugins/alpha-user-preserved.txt')), 'Custom additions were removed'
            check('Conventional uninstaller removes verified owned app files; exact stock/profile unchanged; custom additions retained')
            p,h,rgb=launch(original,[],'OpenCPN 5.12.4-0',profile,'installer-05-restored-stock',welcome_transition='candidate-to-stock')
            chart_check(rgb,'Standard','Untouched stock after uninstall');close(p,h)
            assert fixture_snapshot(profile)==expected
            stable_resources(profile,stock,[custom_tide])
            check('Stock resource defaults survive every generation and uninstall; explicit custom tide selection is preserved')
            check('Original official OpenCPN still loads charts and shared navigation data after uninstall')
            before=inventory(profile)
            setup('Install',original)
            assert json.loads((generation()/'ownership.json').read_text())['version']=='0.4.0-beta2'
            assert json.loads((generation()/'ownership.json').read_text())['shellLayout']=='OpenNavX.SkagerStartMenu.1'
            assert inventory(profile)==before and inventory(stock)==stock_before
            check('Beta 2 reinstall after uninstall preserves original stock, shared profile and retained custom additions')
            setup('Update',original)
            assert state()['previous'] and inventory(profile)==before
            check('Beta 2 same-version rebuild/update creates a rollback generation without changing user data')
            engine('Uninstall')
            assert inventory(profile)==before and inventory(stock)==stock_before
            assert not SHORTCUTS.exists() and not NEUTRAL_SHORTCUTS.exists() and not OLD_SHORTCUTS.exists()
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
    (EVIDENCE/('installer-staging.json' if args.mode=='staging' else 'installer-lifecycle.json')).write_text(json.dumps(report,indent=2)+'\n')
