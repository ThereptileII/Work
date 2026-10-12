import base64,hashlib,json,subprocess,time,zipfile
from pathlib import Path
root=Path('/home/standard/Projects/X-nav-worktrees/source-batch-20261011')
out=Path('/home/standard/Projects/.tmp/generation-executor-native-20261012')
remote=r'C:\XNav\ComponentChecks\generation-executor-605b0bea'
files={
'brokerstore.test.exe':Path('/home/standard/Projects/.tmp/history-commissioning-joined-20261012/executor-brokerstore.test.exe'),
'runner.ps1':root/'opennav-x/tools/boat/test-native-client-registration.ps1'}
hashes={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in files.items()}
expected={'brokerstore.test.exe': '523ad80f99e0a18b58ce02d169ca880131f6b1f41871cfbb140076e096f03829'}
assert all(hashes[k]==v for k,v in expected.items())
(out/'native-transfer-manifest.json').write_text(json.dumps({'sourceCommit':'605b0bea6374dc6a59f426d225639fe4d69e1efc','testRevision':'605b0bea6374dc6a59f426d225639fe4d69e1efc','files':hashes,'remote':remote},indent=2)+'\n')
archive=out/'inert-executor-fixtures.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for name,path in files.items():z.write(path,name)
archive_sha=hashlib.sha256(archive.read_bytes()).hexdigest()
def run(name,args,timeout=120):
 started=time.monotonic()
 try:
  r=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
  record={'exitCode':r.returncode,'durationSeconds':round(time.monotonic()-started,3),'stdout':r.stdout,'stderr':r.stderr}
 except subprocess.TimeoutExpired as e:
  record={'exitCode':None,'durationSeconds':round(time.monotonic()-started,3),'timeout':True,'stdout':str(e.stdout),'stderr':str(e.stderr)}
 (out/(name+'.json')).write_text(json.dumps(record,indent=2)+'\n')
 print(name,record['exitCode'],flush=True)
 return record

def ps(name,script,timeout=120):
 encoded=base64.b64encode(("$ErrorActionPreference='Stop'; "+script).encode('utf-16le')).decode()
 return run(name,['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','boat','powershell -NoProfile -ExecutionPolicy Bypass -EncodedCommand '+encoded],timeout)

r=ps('native-stage',f"$p=Get-Item -LiteralPath 'C:\\XNav\\ComponentChecks'; if(-not $p.PSIsContainer -or ($p.Attributes -band [IO.FileAttributes]::ReparsePoint)){{throw 'plain existing component parent required'}}; if(Test-Path -LiteralPath '{remote}'){{throw 'fresh test directory required'}}; $null=New-Item -ItemType Directory -Path '{remote}'; Get-Process opencpn -ErrorAction SilentlyContinue | Select-Object Id,Responding | ConvertTo-Json -Compress")
if r['exitCode']!=0:raise SystemExit(1)
r=run('native-transfer',['scp','-o','BatchMode=yes','-o','ConnectTimeout=10',str(archive),'boat:C:/XNav/ComponentChecks/generation-executor-605b0bea/fixtures.zip'],120)
if r['exitCode']!=0:raise SystemExit(1)
checks='; '.join("if((Get-FileHash -LiteralPath '"+remote+'\\'+name+"' -Algorithm SHA256).Hash.ToLowerInvariant() -cne '"+sha+"'){throw 'member hash differs'}" for name,sha in hashes.items())
r=ps('native-preflight',f"if((Get-FileHash -LiteralPath '{remote}\\fixtures.zip' -Algorithm SHA256).Hash.ToLowerInvariant() -cne '{archive_sha}'){{throw 'archive hash differs'}}; Expand-Archive -LiteralPath '{remote}\\fixtures.zip' -DestinationPath '{remote}'; {checks}; $t=$null; $e=$null; $null=[Management.Automation.Language.Parser]::ParseFile('{remote}\\runner.ps1',[ref]$t,[ref]$e); if(@($e).Count -ne 0){{throw 'runner parse failure'}}; 'EXACT HASHES AND NATIVE PARSE PASS'")
if r['exitCode']!=0:raise SystemExit(1)
failed=False
for group,binary in [('GenerationExecutor','brokerstore.test.exe')]:
 r=ps('native-'+group,f"& '{remote}\\runner.ps1' -TestExecutable '{remote}\\{binary}' -ExpectedSha256 '{hashes[binary]}' -EvidenceDirectory '{remote}\\{group}' -Group '{group}'",140)
 failed=failed or r['exitCode']!=0
r=ps('native-after',"[ordered]@{navigation=@(Get-Process opencpn -ErrorAction SilentlyContinue | Select-Object Id,Responding); ownedTestProcesses=@(Get-Process -ErrorAction SilentlyContinue | Where-Object {$_.ProcessName -in @('brokerstore.test','brokerenroll.test')} | Select-Object Id,ProcessName)} | ConvertTo-Json -Depth 4")
failed=failed or r['exitCode']!=0
r=ps('native-results-pack',f"Compress-Archive -LiteralPath '{remote}\\GenerationExecutor' -DestinationPath '{remote}\\results.zip'; (Get-FileHash -LiteralPath '{remote}\\results.zip' -Algorithm SHA256).Hash.ToLowerInvariant()")
if r['exitCode']==0:
 download=out/'native-results.zip'
 transfer=run('native-retrieval',['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','boat:C:/XNav/ComponentChecks/generation-executor-605b0bea/results.zip',str(download)],120)
 if transfer['exitCode']!=0:failed=True
 else:
  assert hashlib.sha256(download.read_bytes()).hexdigest()==r['stdout'].strip()
  target=out/'native-windows';target.mkdir(exist_ok=False)
  with zipfile.ZipFile(download) as z:
   for member in z.infolist():
    p=Path(member.filename.replace('\\','/'))
    assert not p.is_absolute() and '..' not in p.parts
    if member.is_dir() or member.filename.endswith(('/', '\\')):continue
    assert p.suffix in ('.json','.log') and member.file_size<4*1024*1024
    destination=target/p;destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(z.read(member))
else:failed=True
raise SystemExit(1 if failed else 0)
