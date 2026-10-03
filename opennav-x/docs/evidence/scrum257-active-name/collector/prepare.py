from pathlib import Path
import ast,collections,hashlib,json
here=Path(__file__).resolve().parent
source=Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/tools/smoke-navigation.py')
original=source.read_text();s=original
s=s.replace('from diagnostic_snapshot import read_json_snapshot','from diagnostic_snapshot import read_json_snapshot\nfrom route252_capture import capture_cycle, capture_stale')
s=s.replace('root = Path(__file__).resolve().parents[1]',"root = Path(os.environ['SKAGER_ROUTE252_APP'])")
s=s.replace("evidence = root / 'evidence/local'","evidence = Path(os.environ['SKAGER_ROUTE252_OUTPUT'])")
s=s.replace("prefix='opennav input '","prefix='sr252-'")
s=s.replace("str(root / 'build' / variant)","os.environ['SKAGER_ROUTE252_BUILD']")
s=s.replace('        display = 101',"        display = int(os.environ['SKAGER_ROUTE252_DISPLAY'])")
s=s.replace("        exe = root / 'build/xnav-install/bin/opencpn'","        exe = Path(os.environ['SKAGER_ROUTE252_EXE'])")
needle="                if result.get('phase') == 'stop-input':\n                    phase[0] = 'none'"
assert s.count(needle)==1
s=s.replace(needle,"                if result.get('phase') == 'stop-input':\n                    if 'route_label_cycle' not in report:\n                        capture_cycle(profile,evidence,prefix,env,args,report,capture,result,app,phase)\n                    phase[0] = 'none'")
needle="                        capture('02-stale-position')\n                        seen_stale = True"
assert s.count(needle)==1
s=s.replace(needle,"                        capture('02-stale-position')\n                        capture_stale(profile,evidence,prefix,report)\n                        seen_stale = True")
s=s.replace("                    report['route_contract'] = result","                    assert len(result['checks']) == 26, 'All original 26 route assertions are required'\n                    report['route_contract'] = result")
a=lambda text:collections.Counter(ast.dump(n.test) for n in ast.walk(ast.parse(text)) if isinstance(n,ast.Assert))
assert not (a(original)-a(s)), 'An original smoke assertion was removed or changed'
(here/'smoke-navigation-original.py').write_text(original)
(here/'smoke-navigation-route252.py').write_text(s)
manifest={'source':'tools/smoke-navigation.py','sourceSha256':hashlib.sha256(original.encode()).hexdigest(),
 'collectorSha256':hashlib.sha256(s.encode()).hexdigest(),'preservedOriginalAssertions':sum(a(original).values()),
 'collectorAssertions':sum(a(s).values()),'routeScenarioSha256':hashlib.sha256((source.parents[1]/'tests/RouteProgressScenario.cpp').read_bytes()).hexdigest(),
 'preparedOnly':True,'launchRequires':'Explicit parent final source commit and installed executable SHA256; no launch performed'}
(here/'preparation.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
