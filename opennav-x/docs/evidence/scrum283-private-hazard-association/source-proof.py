from pathlib import Path
import hashlib,json,importlib.util,shutil,subprocess
r=Path.cwd();original=Path('/home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source');derived=r/'.local/source';fresh=r/'.local/reproduced';shutil.copytree(original,fresh)
def module(name,file):
 s=importlib.util.spec_from_file_location(name,file);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
prep=module('prep',r/'tools/prepare-ocharts-adapter.py');prep.apply_patches(fresh,r);f=module('loader',r/'tools/verify-anchor-loader.py').function
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a={str(p.relative_to(fresh)):sha(p) for p in fresh.rglob('*') if p.is_file()};b={str(p.relative_to(derived)):sha(p) for p in derived.rglob('*') if p.is_file()};assert a==b
old=(original/'src/eSENCChart.cpp').read_text();new=(derived/'src/eSENCChart.cpp').read_text()
unchanged={}
for sig in ('ListOfS57Obj *eSENCChart::GetAssociatedObjects(', 'ListOfPI_S57Obj *eSENCChart::GetObjRuleListAtLatLon(', 'eSENCChart::~eSENCChart()'):
 x,y=f(old,sig),f(new,sig);assert x==y;unchanged[sig]=hashlib.sha256(x.encode()).hexdigest()
active=f(new,'int eSENCChart::BuildRAZFromSENCFile(');assert active.count('calloc( sizeof(chart_context), 1)')==2 # one commented historical line, one active.
assert active.count('\n        m_this_chart_context = (chart_context *)calloc')==1
assert active.count('get_associated_objects = &SkagerAssociatedObjects')==1
assert active.index('get_associated_objects = &SkagerAssociatedObjects')<active.index('obj->m_chart_context = m_this_chart_context;',active.index('get_associated_objects = &SkagerAssociatedObjects'))
assert f(new,'eSENCChart::~eSENCChart()').index('FreeObjectsAndRules();')<f(new,'eSENCChart::~eSENCChart()').index('free( m_this_chart_context )')
# Disabled old PI-context branch remains byte-for-byte equal.
for text in (old,new):assert '#if 0\nint eSENCChart::BuildRAZFromSENCFile' in text
old_disabled=f(old[old.index('#if 0\nint eSENCChart::BuildRAZFromSENCFile'):],'int eSENCChart::BuildRAZFromSENCFile(')
new_disabled=f(new[new.index('#if 0\nint eSENCChart::BuildRAZFromSENCFile'):],'int eSENCChart::BuildRAZFromSENCFile(');assert old_disabled==new_disabled
api='opencpn-libs/api-17/ocpn_plugin.h';assert sha(original/api)==sha(derived/api)
report={'patches':{x:sha(r/x) for x in prep.PATCHES},'reproducedFiles':len(a),'derivedSourceFiles':a,'originalQuerySelectionDestructorUnchanged':unchanged,'api17Sha256':sha(original/api),'activeContextInitializers':1,'callbackAssignments':1,'disabledPiContextUnchanged':True,'destructionOrder':'FreeObjectsAndRules before context free','scope':'Source/ownership closure, not concurrent render/destruction or native DLL runtime qualification'}
(r/'.local/source-proof.json').write_text(json.dumps(report,indent=2)+'\n');print(len(a),'exact composed source files; unchanged association algorithm/selection/destructor/API17; one active initializer bound')
