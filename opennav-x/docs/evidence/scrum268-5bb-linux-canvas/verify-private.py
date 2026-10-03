from pathlib import Path
import importlib.util,hashlib,json,shutil
root=Path(__file__).resolve().parents[3];prep=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('adapter',root/'tools/prepare-ocharts-adapter.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
original=Path('/home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source')
lock=json.loads((root/m.LOCK).read_text());expected={}
for key in ('source','gitlink'):
 for name,record in lock[key]['files'].items():expected[('opencpn-libs/' if key=='gitlink' else '')+name]=record
actual={str(p.relative_to(original)) for p in original.rglob('*') if p.is_file()}
assert actual<=set(expected)
source=prep/'private-source';source.mkdir();origins={}
for name,record in expected.items():
 origin=original/name if name in actual else prep.parent/'ca-light-326daf7/private-source'/name
 raw=origin.read_bytes();assert m.blob_ok(raw,record),name
 target=source/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw);origins[name]=str(origin)
m.apply_patches(source,root);m.copy_local_inputs(prep/'private-prepared')
for name in m.LOCAL:assert (prep/'private-prepared/local'/name).read_bytes()==(root/name).read_bytes().replace(b'\r\n',b'\n')
receipt={'source':'5bb7e0584029c72ce21f9230e246dad5771dc321','scope':'Exact219 locked source/gitlink bytes copied read-only; current two ordered patches and21 owned inputs; no private DLL build','original':str(original),'originalInventoryLimitation':'Supplied original directory lacks COPYING.gplv2 and debian/copyright; these two were read from retained previous private-source and each independently matched current lock bytes/Gitblob before use','origins':origins,'lockedOriginalPaths':len(expected),'patches':{name:m.record(root/name) for name in m.PATCHES},'localInputs':{name:m.record(root/name) for name in m.LOCAL},'outputs':{name:m.record(source/name) for name in ('libs/s52plib/src/s52plib.cpp','libs/s52plib/src/s52plib.h','src/eSENCChart.cpp')}}
(prep/'private-closure.json').write_text(json.dumps(receipt,indent=2)+'\n');print('Verified',len(expected),'locked original paths,',len(m.PATCHES),'ordered patches,',len(m.LOCAL),'exact owned inputs')
