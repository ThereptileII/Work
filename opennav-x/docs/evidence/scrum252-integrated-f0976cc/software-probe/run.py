#!/usr/bin/env python3
"""Explicit final-identity gate. Only launched after root supplies final hashes."""
import argparse,hashlib,json,os,re,subprocess,sys
from pathlib import Path
here=Path(__file__).resolve().parent
cache=Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity')
p=argparse.ArgumentParser();p.add_argument('--expected-commit',required=True);p.add_argument('--expected-exe-sha256',required=True)
p.add_argument('--renderer',required=True,choices=('software','opengl'));p.add_argument('--name',required=True)
a=p.parse_args()
assert re.fullmatch('[0-9a-f]{40}',a.expected_commit)
assert re.fullmatch('[0-9a-f]{64}',a.expected_exe_sha256)
assert re.fullmatch('[a-zA-Z0-9_-]+',a.name)
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
app=cache/'app';build=cache/'build';exe=cache/'install/bin/opencpn'
prep=json.loads((here/'preparation.json').read_text())
def identity():
    staged=json.loads((cache/'staged-inputs.json').read_text());frozen=json.loads((cache/'frozen-inputs.json').read_text())
    assert staged['commit']==frozen['commit']==a.expected_commit,'Final source is not the supplied staged identity'
    assert staged['binary_sha256']==sha(exe)==a.expected_exe_sha256,'Final installed executable differs'
    assert sha(app/'tools/smoke-navigation.py')==prep['sourceSha256'],'Prepared original collector changed; re-inspect before preparing again'
    assert sha(app/'tests/RouteProgressScenario.cpp')==prep['routeScenarioSha256'],'Original 26-check scenario changed'
    env=dict(os.environ,GIT_OPTIONAL_LOCKS='0')
    git=['git','--git-dir='+str(app/'.git'),'--work-tree='+str(app)]
    assert subprocess.check_output(git+['rev-parse','HEAD'],env=env,text=True).strip()==a.expected_commit
    assert not subprocess.check_output(git+['status','--porcelain','--untracked-files=no'],env=env).strip(),'Staged source is dirty'
    resources=cache/'install/share/opencpn/opennav/chart-style/v1'
    assert sha(resources/'manifest.json')==staged['manifest_sha256']
    for name,value in staged['resource_files'].items():
        path=resources/name
        assert path.stat().st_size==value['bytes'] and sha(path)==value['sha256'],name
    for name,value in frozen['patches'].items():assert sha(app/'patches'/name)==value,name
    assert sha(here/'smoke-navigation-route252.py')==prep['collectorSha256']
    return {'staged':staged,'frozen':frozen,'collectorPreparation':prep,
            'scope':'Isolated copied actual route scenario; no external input/output, model-pointer injection, product or frozen cache edits',
            'rendererRequested':a.renderer,'upstreamPatchValidation':'Root frozen source; patch identities independently retained here',
            'collectorHelperSha256':sha(here/'route252_capture.py')}
record=identity()
out=here/('capture-'+a.name);out.mkdir(exist_ok=False)
(out/'input-identities.json').write_text(json.dumps(record,indent=2)+'\n')
env=dict(os.environ,PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/bin:/usr/local/bin:/usr/bin:/bin',
         LD_LIBRARY_PATH='/home/standard/Projects/X-nav/.local/sysroot/usr/lib',
         PYTHONPATH=str(app/'tools'),SKAGER_ROUTE252_APP=str(app),SKAGER_ROUTE252_BUILD=str(build),
         SKAGER_ROUTE252_EXE=str(exe),SKAGER_ROUTE252_OUTPUT=str(out),SKAGER_ROUTE252_COMMIT=a.expected_commit,
         SKAGER_ROUTE252_DISPLAY='221' if a.renderer=='software' else '222')
python='/home/standard/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3'
command=['bwrap','--die-with-parent','--unshare-user','--unshare-pid','--unshare-net',
 '--ro-bind','/','/','--proc','/proc','--dev','/dev','--tmpfs','/tmp',
 '--bind',str(here),str(here),
 '--ro-bind',str(app),'/home/standard/Projects/X-nav-worktrees/waypoint-touch-regression',
 '--ro-bind',str(build),'/home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux',
 '--ro-bind',str(cache/'upstream'),'/home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source',
 '--ro-bind',str(cache/'install'),'/home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/build/xnav-install',
 '--chdir',str(here),python,str(here/'smoke-navigation-route252.py'),'--route-fixture','--renderer',a.renderer]
with (out/'collector.log').open('w') as log:completed=subprocess.run(command,env=env,stdout=log,stderr=log,timeout=180)
record['exitCode']=completed.returncode
record['identitiesUnchangedAfter']=identity()=={k:v for k,v in record.items() if k not in ('exitCode','identitiesUnchangedAfter')}
assert record['identitiesUnchangedAfter'],'Source or staged executable/resources changed during capture'
prefix='route'+('-opengl' if a.renderer=='opengl' else '')
log=out/f'{prefix}-input-profile/opencpn.log'
record['glEvidence']=[line for line in log.read_text(errors='replace').splitlines() if 'OpenGL->' in line] if log.exists() else []
(out/'run-result.json').write_text(json.dumps(record,indent=2)+'\n')
assert completed.returncode==0,'Copied route collector failed; preserve its evidence'
result=json.loads((out/f'{prefix}-input-results.json').read_text())
assert result['route_contract']['result']=='passed' and len(result['route_contract']['checks'])==26
assert len(result['route_label_cycle'])==4 and result['route_label_stale']
if a.renderer=='opengl':assert any('Renderer' in line for line in record['glEvidence']),'Actual GL renderer evidence missing'
print('Focused route-label and original 26-check route scenario passed:',out)
