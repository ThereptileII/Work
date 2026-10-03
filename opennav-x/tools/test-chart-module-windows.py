#!/usr/bin/env python3
"""Exercise the explicit early check in the actual fixture-free installed host."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]

def identity(path):
    data=path.read_bytes()
    return {'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install',type=Path,required=True)
    parser.add_argument('--vendor-archive',type=Path,help='Optional exact locked archive; otherwise fetch the fixed lock into build/ocharts-identity-cache')
    parser.add_argument('--evidence',type=Path,required=True)
    args=parser.parse_args()
    if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true':
        raise SystemExit('Disposable native Windows Actions runner required; never the boat')
    install=args.install.resolve();evidence=args.evidence.resolve()
    if evidence==install or install in evidence.parents:
        raise ValueError('Identity input/evidence must be outside application discovery directories')
    evidence.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('loader_probe',ROOT/'tools/test-ocharts-loader-windows.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    lock=json.loads((ROOT/'tests/windows_ocharts_loader/inputs.lock.json').read_text())['vendorReadOnly']
    identity_cache=(ROOT/'build/ocharts-identity-cache').resolve()
    if any(identity_cache==root or root in identity_cache.parents for root in (install,evidence)):
        raise ValueError('Identity cache must be outside evidence and application discovery directories')
    archive=args.vendor_archive
    if archive is None:
        spec=importlib.util.spec_from_file_location('changed_units',ROOT/'tools/test-windows-changed-units.py')
        api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)
        archive=identity_cache/lock['file']
        api.fetch(lock,archive)
    original=identity_cache/'extracted/o-charts_pi.dll'
    original_identity=helper.extract_read_only_vendor(archive,original,lock)
    sources={}
    for name in (
        'tools/test-chart-module-windows.py','tools/test-ocharts-loader-windows.py',
        'tools/test-windows-changed-units.py','tests/windows_ocharts_loader/inputs.lock.json',
        'src/integration/InstallerSelfTest.cpp','src/integration/InstallerSelfTest.h',
        'src/integration/ChartModuleCheck.cpp','src/integration/ChartModuleCheck.h',
        'src/integration/ChartModulePe.h','src/integration/OChartsModuleLoader.cpp',
        'src/integration/OChartsModuleLoader.h','src/integration/PluginPresentationFallback.h',
        'src/integration/SkagerOChartsPackage.h.in','src/integration/OpenCPN.cmake',
        'src/plugin-adapters/ChartPresentationBindingV1.h',
        'src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp',
        'src/plugin-adapters/ocharts/BindingState.h','patches/opencpn-5.12.4-xnav.patch'):
        sources[name]=identity(ROOT/name)
    repository=Path(subprocess.check_output(['git','rev-parse','--show-toplevel'],cwd=ROOT,text=True).strip()).resolve()
    if repository!=ROOT and ROOT!=repository/'opennav-x':
        raise ValueError('Unexpected repository layout')
    workflow_name=os.environ.get('GITHUB_WORKFLOW_REF','').partition('/.github/workflows/')[2].split('@',1)[0]
    if not workflow_name or Path(workflow_name).name!=workflow_name or not workflow_name.endswith(('.yml','.yaml')):
        raise ValueError('Missing or invalid actual workflow reference')
    workflow=repository/'.github/workflows'/workflow_name
    sources[os.path.relpath(workflow,ROOT).replace('\\','/')]=identity(workflow)
    (evidence/'source-identities.json').write_text(json.dumps(sources,indent=2)+'\n')
    exe=install/'opencpn.exe';adapter=install/'skager-ocharts-adapter.dll'
    before={name:identity(install/name) for name in ('opencpn.exe','skager-ocharts-adapter.dll')}
    profiles=[Path(os.environ[key])/'opencpn' for key in ('APPDATA','LOCALAPPDATA','PROGRAMDATA')]
    def snapshot():
        values={}
        for root in profiles:
            values[str(root)]=None if not root.exists() else {
                str(p.relative_to(root)):identity(p) for p in root.rglob('*') if p.is_file()}
        return values
    profile_before=snapshot()
    summary={'status':'failed','scope':'real installed host imports/binding/unload; no chart rendering',
             'host':before,'original':original_identity,'candidate':os.environ.get('GITHUB_SHA'),
             'nativeProductAcceptance':False,'cases':[],
             'vendorArchive':identity(archive),'sources':sources}
    def run(name,args,expected):
        cmd=[str(exe),*map(str,args)]
        (evidence/(name+'.command.json')).write_text(json.dumps(cmd,indent=2)+'\n')
        try:
            done=subprocess.run(cmd,cwd=evidence,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=30)
        except subprocess.TimeoutExpired as failure:
            (evidence/(name+'.stdout.txt')).write_bytes(failure.stdout or b'')
            (evidence/(name+'.stderr.txt')).write_bytes(failure.stderr or b'')
            summary['cases'].append({'name':name,'timeoutSeconds':30,'expected':expected})
            raise
        (evidence/(name+'.stdout.txt')).write_bytes(done.stdout)
        (evidence/(name+'.stderr.txt')).write_bytes(done.stderr)
        summary['cases'].append({'name':name,'exit':done.returncode,'expected':expected})
        if done.returncode!=expected:raise RuntimeError(name+' returned unexpected exit')
    try:
        run('option-without-selftest',['--skager-chart-module-check',original],2)
        default=evidence/'default.json'
        run('default',['--opennav-self-test',default],0)
        result=json.loads(default.read_text())
        if result['profile_initialized'] or result['plugins_loaded'] or result['test_fixtures'] or 'chart_module' in result:
            raise ValueError('Default selftest changed or host is not fixture-free')
        if result['commit']!=os.environ['GITHUB_SHA']:
            raise ValueError('Installed host commit differs')
        positive=evidence/'module.json'
        run('module',['--opennav-self-test',positive,'--skager-chart-module-check',original],0)
        result=json.loads(positive.read_text());module=result['chart_module']
        if (not result['passed'] or result['profile_initialized'] or result['plugins_loaded'] or
            result['test_fixtures'] or not module['passed'] or not module['module_loaded'] or
            not module['unload_succeeded'] or not module['host_imports_resolved'] or
            not module['child_process_creation_blocked'] or not module['resources_verified'] or
            not module['loaded_modules_observed'] or
            module['factory_called'] or module['plugin_initialized'] or module['original_dll_executed'] or
            module['binding_state']!=1 or module['binding_reason']!=0 or
            module['original_sha256']!=lock['dllSha256'] or
            module['adapter_sha256']!=before['skager-ocharts-adapter.dll']['sha256'] or
            int(module['adapter_bytes'])!=before['skager-ocharts-adapter.dll']['bytes'] or
            Path(module['module_path']).resolve()!=adapter or 'opencpn.exe' not in module['imports']):
            raise ValueError('Real module proof facts differ')
        loaded={}
        for name in module['loaded_modules']:
            path=Path(name)
            if not path.is_absolute() or not path.is_file():
                raise ValueError('Observed runtime path is not an existing absolute file')
            loaded[str(path)]=identity(path)
        if {exe,adapter}-set(map(Path,loaded)):
            raise ValueError('Actual loaded-module inventory omitted host or private module')
        summary['loadedRuntime']=loaded
        (evidence/'loaded-runtime-identities.json').write_text(json.dumps(loaded,indent=2)+'\n')
        # Controlled identity-only mutation. It must fail before private or
        # original execution; restore the exact original bytes in finally.
        raw=original.read_bytes()
        try:
            original.write_bytes(raw[:-1]+bytes([raw[-1]^1]))
            rejected=evidence/'wrong-original.json'
            run('wrong-original',['--opennav-self-test',rejected,'--skager-chart-module-check',original],1)
            failure=json.loads(rejected.read_text())
            if failure['passed'] or failure['chart_module']['module_loaded']:
                raise ValueError('Changed original identity was accepted')
        finally:original.write_bytes(raw)
        if snapshot()!=profile_before:raise ValueError('Normal profile bytes changed during early checks')
        if any(identity(install/name)!=value for name,value in before.items()):
            raise ValueError('Installed host/module bytes changed')
        if identity(original)!=original_identity:raise ValueError('Original identity input changed')
        summary['profilesUnchanged']=True
        summary['status']='passed'
    finally:
        summary['profilesUnchanged']=snapshot()==profile_before
        (evidence/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':main()
