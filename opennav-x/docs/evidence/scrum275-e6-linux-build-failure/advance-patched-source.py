"""Advance only a proven previous disposable patch tree to exact authorized inputs."""
import hashlib,importlib.util,json,os,pathlib,subprocess,tempfile
prep=pathlib.Path(__file__).resolve().parent
os.environ['SKAGER_CAPTURE_SCRATCH']=str(prep/'output')
spec=importlib.util.spec_from_file_location('checked',prep/'iho/cache-inputs-readonly.py');i=importlib.util.module_from_spec(spec);spec.loader.exec_module(i)
old='e1d01367692f7b8c866be369417e68c836c5d563';new='e6ef3843dc604368d32bc823c770bec76d890b69'
i.require(i.git(i.APP,'rev-parse','HEAD').decode().strip()==new,'Different authorized app source')
i.require(not i.git(i.APP,'status','--porcelain','--untracked-files=no').strip(),'App source dirty')
with tempfile.TemporaryDirectory(dir=prep/'output',prefix='old-patches-') as temporary:
 root=pathlib.Path(temporary);(root/'tools').mkdir();(root/'patches').mkdir()
 for rel in ['tools/prepare-integration.py']+[str(x.relative_to(i.APP)) for x in i.patches()]:
  (root/rel).write_bytes(i.git(i.APP,'show',old+':'+rel))
 with i.expected(root) as (env,old_records,inputs):i.verify_expected(env)
 with i.expected() as (env,new_records,inputs):
  changes=[]
  for name,(mode,oid) in new_records.items():
   if old_records.get(name)==(mode,oid):continue
   path=i.UPSTREAM/name;raw=i.git(i.UPSTREAM,'cat-file','blob',oid,env=env)
   i.require(mode in ('100644','100755'),'Unexpected nonregular patch input')
   before=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
   if not path.exists() or path.read_bytes()!=raw:
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
   path.chmod(0o755 if mode=='100755' else 0o644)
   changes.append({'path':name,'oldSha256':before,'sha256':hashlib.sha256(raw).hexdigest(),'mode':mode})
  for name in old_records.keys()-new_records.keys():
   (i.UPSTREAM/name).unlink();changes.append({'path':name,'deleted':True})
  i.verify_expected(env)
 (prep/'source-advance.json').write_text(json.dumps({'from':old,'to':new,'oldTreeMatchedAllNinePatches':True,'newTreeMatchedAllNinePatches':True,'changes':changes},indent=2)+'\n')
 print('Verified old and new nine-patch trees;',len(changes),'changed source paths; unchanged source mtimes preserved')
