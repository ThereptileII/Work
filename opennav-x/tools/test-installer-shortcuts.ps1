# Actual WScript.Shell links and lifecycle publication/recovery, confined to a
# unique disposable directory and HKCU test key. Never run the engine entrypoint.
param([switch]$PolicyOnly)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if (-not $PolicyOnly -and ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT')) { throw 'Disposable Windows CI only.' }
$parseErrors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'),[ref]$null,[ref]$parseErrors)
if ($parseErrors) { throw ($parseErrors | Out-String) }
foreach ($name in @('Log','Hash','PlainPath','RelativePath','ReadJson','AtomicJson','Generation','ReadState','ReadGeneration','ShellGroups','ShortcutGroup','ShortcutNames','ShortcutSpec','AssertShortcut','AssertShellOwnership','RemoveShortcutGroup','PublishShell','RemoveShell','Failure','Recover')) {
  $nodes=@($ast.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq $name},$true))
  if ($nodes.Count -ne 1) { throw "Expected one actual engine function: $name" }
  . ([scriptblock]::Create($nodes[0].Extent.Text))
}
$Fixture=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav shortcuts '+[guid]::NewGuid().ToString('N'))
$Root=Join-Path $Fixture 'owned integration'
$Programs=Join-Path $Fixture 'test Programs'
$Registry='HKCU:\Software\OpenNavXShortcutTests\'+[guid]::NewGuid().ToString('N')
$Owner='OpenNavX.Alpha1.SideBySide.1'
$Utf8=New-Object Text.UTF8Encoding($false)
$SessionLog=New-Object 'Collections.Generic.List[string]'
$FailurePoint='';$Checks=0;$junction=$null
function Check([bool]$ok,[string]$name) { if (-not $ok) { throw "FAILED: $name" }; $script:Checks++; Write-Host "PASS: $name" }
function Snapshot {
  $files=@(Get-ChildItem -LiteralPath $Fixture -Recurse -Force -File | Sort-Object FullName | ForEach-Object { $_.FullName+' '+(Hash $_.FullName) })
  $reg='';if(Test-Path -LiteralPath $Registry){$reg=(Get-ItemProperty -LiteralPath $Registry | Select-Object DisplayName,DisplayVersion,ModifyPath,UninstallString,OpenNavOwner | ConvertTo-Json -Compress)}
  return (($files -join "`n")+"`n"+$reg)
}
function RefusesUnchanged([scriptblock]$operation,[string]$name) {
  $before=Snapshot;$failed=$false
  try { & $operation | Out-Null } catch { $failed=$true }
  Check $failed $name
  Check ((Snapshot) -ceq $before) ($name+' preserves files, links and registration')
}
foreach($version in @('0.2.0-alpha1','0.3.0-beta1','0.4.0-beta2')) {
  Check ((ShortcutGroup ([pscustomobject]@{version=$version})) -ceq (Join-Path $Programs 'OpenNav X Alpha 1')) ('Absent marker preserves historical layout independently of '+$version)
}
Check ((ShortcutGroup ([pscustomobject]@{version='0.4.0-beta2';shellLayout='OpenNavX.NeutralStartMenu.1'})) -ceq (Join-Path $Programs 'OpenNav X')) 'Exact layout marker selects neutral group'
Check ((ShortcutGroup ([pscustomobject]@{version='0.4.0-beta2';shellLayout='OpenNavX.SkagerStartMenu.1'})) -ceq (Join-Path $Programs 'SKAGER')) 'New marker selects SKAGER group'
Check (@(ShellGroups).Count -eq 3) 'Only the three versioned shortcut groups are recognized'
Check ((@(ShortcutNames (Join-Path $Programs 'SKAGER')) -join ',') -ceq 'Skager.lnk,OpenCPN Legacy.lnk,Skager Safe Mode.lnk,Maintain Skager.lnk') 'SKAGER shortcut labels are exact'
foreach($badLayout in @('', 'OpenNavX.NeutralStartMenu.2', 'opennavx.neutralstartmenu.1', 'opennavx.skagerstartmenu.1', 1, $true, $null)) {
  $refused=$false
  try { $null=ShortcutGroup ([pscustomobject]@{version='0.4.0-beta2';shellLayout=$badLayout}) } catch { $refused=$true }
  Check $refused 'Present invalid layout marker never falls back to a guessed group'
}
if($PolicyOnly){Write-Host "$Checks shell-layout policy checks passed; no COM, registry or filesystem mutation.";return}
function FixtureGeneration([string]$id,[string]$version,[switch]$Neutral,[switch]$Skager,[int]$StartupHealth=-1) {
  $d=Generation $id;$null=[IO.Directory]::CreateDirectory((Join-Path $d 'app'))
  [IO.File]::WriteAllText((Join-Path $d 'app/opencpn.exe'),'inert target '+$id,$Utf8)
  [IO.File]::WriteAllText((Join-Path $d 'Maintain.exe'),'inert maintainer '+$id,$Utf8)
  $record=@{owner=$Owner;version=$version;managedFiles=@(
    @{path='app/opencpn.exe';sha256=(Hash (Join-Path $d 'app/opencpn.exe'))},
    @{path='Maintain.exe';sha256=(Hash (Join-Path $d 'Maintain.exe'))})}
  if($Neutral){$record.shellLayout='OpenNavX.NeutralStartMenu.1'}
  if($Skager){$record.shellLayout='OpenNavX.SkagerStartMenu.1'}
  if($StartupHealth -ge 0){$record.updateStartupHealth=$StartupHealth}
  if($StartupHealth -eq 1){
    [IO.File]::WriteAllText((Join-Path $d 'app/skager-start.exe'),'inert launcher '+$id,$Utf8)
    $record.managedFiles+=@{path='app/skager-start.exe';sha256=(Hash (Join-Path $d 'app/skager-start.exe'))}
  }
  AtomicJson (Join-Path $d 'ownership.json') $record
  return [pscustomobject]@{owner=$Owner;schema=1;current=$id;previous='';stock=@{path=(Join-Path $Fixture 'stock/opencpn.exe')};shortcutModes=@('xnav','legacy','safe')}
}
function CheckGroup($state,[string]$version) {
  $group=ShortcutGroup (ReadGeneration $state.current);$others=@(ShellGroups | Where-Object {$_ -cne $group})
  Check ((Test-Path -LiteralPath $group) -and @($others | Where-Object {Test-Path -LiteralPath $_}).Count -eq 0) ('Only expected group for '+$version)
  $shell=New-Object -ComObject WScript.Shell
  $files=@(Get-ChildItem -LiteralPath $group -File)
  Check ($files.Count -eq (@($state.shortcutModes).Count+1)) 'Only requested shortcuts plus maintenance'
  foreach($f in $files){
    AssertShortcut $f.FullName $shell
    $s=ShortcutSpec $f.Name (ReadGeneration $state.current);$link=$shell.CreateShortcut($f.FullName)
    Check ([string]::Equals($link.TargetPath,(RelativePath (Generation $state.current) $s.target),[StringComparison]::OrdinalIgnoreCase)) ('Current generation '+$f.Name)
  }
  Check ((Get-ItemProperty -LiteralPath $Registry).ModifyPath -ceq ('"'+(Join-Path (Generation $state.current) 'Maintain.exe')+'"')) 'Registry maintenance points to the exact selected generation'
}
try {
  $null=[IO.Directory]::CreateDirectory($Root)
  $null=[IO.Directory]::CreateDirectory($Programs)
  AtomicJson (Join-Path $Root 'owner.json') @{owner=$Owner}
  $old=FixtureGeneration ('a'*32) '0.3.0-beta1'
  $next=FixtureGeneration ('b'*32) '0.4.0-beta2' -Neutral
  $early=FixtureGeneration ('d'*32) '0.4.0-beta2'
  $skager=FixtureGeneration ('e'*32) '0.4.0-beta2' -Skager
  $health=FixtureGeneration ('f'*32) '0.4.0-beta2' -Skager -StartupHealth 1
  $health0=FixtureGeneration ('1'*32) '0.4.0-beta2' -Skager -StartupHealth 0
  $shell=New-Object -ComObject WScript.Shell
  foreach($selected in @($skager,$health0,$health)) {
    PublishShell $selected;CheckGroup $selected 'startup-health publication'
    $group=ShortcutGroup (ReadGeneration $selected.current)
    $expected=if($selected.current -ceq $health.current){'app/skager-start.exe'}else{'app/opencpn.exe'}
    $link=$shell.CreateShortcut((Join-Path $group 'Skager.lnk'))
    Check ($link.TargetPath -ieq (Join-Path (Generation $selected.current) $expected)) 'Only startup-health 1 selects the update launcher'
    foreach($name in @('OpenCPN Legacy.lnk','Skager Safe Mode.lnk')) {
      Check ($shell.CreateShortcut((Join-Path $group $name)).TargetPath -ieq (Join-Path (Generation $selected.current) 'app/opencpn.exe')) 'Legacy and Safe remain direct app launches'
    }
    Remove-Item -LiteralPath (Join-Path $group 'Skager.lnk')
    PublishShell $selected;CheckGroup $selected 'repair restores selected startup target'
  }
  foreach($prior in @($health0,$skager)) {
    PublishShell $health
    AtomicJson (Join-Path $Root 'state.json') $prior
    AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$health;after=$prior}
    Recover;CheckGroup $prior 'historical SKAGER rollback recovery'
    Check ($shell.CreateShortcut((Join-Path (Join-Path $Programs 'SKAGER') 'Skager.lnk')).TargetPath -ieq (Join-Path (Generation $prior.current) 'app/opencpn.exe')) 'Health0 or absent health marker recovery restores direct launch'
  }
  RemoveShell
  PublishShell $next;CheckGroup $next '0.4.0-beta2'
  Check ((Get-ItemProperty -LiteralPath $Registry).DisplayName -ceq 'OpenNav X Beta 2') 'Beta 2 display name has no Alpha label'
  RemoveShell
  Check (@(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'Clean uninstall removes only its empty group'
  PublishShell $old;CheckGroup $old '0.3.0-beta1'
  AtomicJson (Join-Path $Root 'state.json') $next
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Update';before=$old;after=$next}
  $FailurePoint='after-shortcuts';$failed=$false
  try { PublishShell $next } catch { $failed=$_.Exception.Message -eq 'Injected interruption at after-shortcuts' }
  $FailurePoint=''
  Check ($failed -and (Test-Path (ShortcutGroup (ReadGeneration $old.current))) ) 'Interrupted migration retains the prior usable group'
  Check (Test-Path (ShortcutGroup (ReadGeneration $next.current))) 'Complete new group exists before old links are removed'
  Recover;CheckGroup $next '0.4.0-beta2'
  Check (-not (Test-Path (Join-Path $Root 'transaction.json'))) 'Recovery clears journal only after complete publication'
  $next.shortcutModes=@('xnav');PublishShell $next;CheckGroup $next '0.4.0-beta2'
  # Recover a committed rollback in the opposite direction, for the immutable
  # old engine's group and unchanged maintainer target.
  AtomicJson (Join-Path $Root 'state.json') $old
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$next;after=$old}
  $FailurePoint='after-shortcuts';try { PublishShell $old } catch { if($_.Exception.Message -ne 'Injected interruption at after-shortcuts'){throw} }
  $FailurePoint='';Recover;CheckGroup $old '0.3.0-beta1'
  PublishShell $next;CheckGroup $next '0.4.0-beta2'
  $neutral=ShortcutGroup (ReadGeneration $next.current);$legacy=ShortcutGroup (ReadGeneration $old.current)
  Check ((ShortcutGroup (ReadGeneration $early.current)) -ceq $legacy) 'Early Beta 2 without a layout marker uses its immutable maintainer group'
  $earlyBefore=Get-ChildItem -LiteralPath (Generation $early.current) -Recurse -File | ForEach-Object { $_.FullName+' '+(Hash $_.FullName) }
  PublishShell $early;CheckGroup $early '0.4.0-beta2 historical layout'
  Check (-not (Test-Path -LiteralPath $neutral)) 'Early Beta 2 rollback removes neutral group'
  PublishShell $next;CheckGroup $next '0.4.0-beta2 neutral layout'
  AtomicJson (Join-Path $Root 'state.json') $early
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$next;after=$early}
  $FailurePoint='after-shortcuts';$failed=$false
  try { PublishShell $early } catch { $failed=$_.Exception.Message -eq 'Injected interruption at after-shortcuts' }
  $FailurePoint=''
  Check ($failed -and (Test-Path -LiteralPath $legacy) -and (Test-Path -LiteralPath $neutral)) 'Interrupted early Beta 2 rollback leaves both complete groups'
  Recover;CheckGroup $early '0.4.0-beta2 historical recovery'
  Check (-not (Test-Path -LiteralPath (Join-Path $Root 'transaction.json'))) 'Early Beta 2 recovery completes before journal removal'
  $earlyAfter=Get-ChildItem -LiteralPath (Generation $early.current) -Recurse -File | ForEach-Object { $_.FullName+' '+(Hash $_.FullName) }
  Check (($earlyBefore -join "`n") -ceq ($earlyAfter -join "`n")) 'Early Beta 2 owned targets and marker-free ownership remain byte-identical'
  RemoveShell;Check (@(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'Historical early Beta 2 group removes cleanly'
  PublishShell $next;CheckGroup $next '0.4.0-beta2 neutral layout'
  $earlyRecordPath=Join-Path (Generation $early.current) 'ownership.json';$earlyRecordBytes=[IO.File]::ReadAllBytes($earlyRecordPath)
  foreach($badLayout in @('', 'OpenNavX.NeutralStartMenu.2', 'opennavx.neutralstartmenu.1', 1, $null)) {
    $record=ReadJson $earlyRecordPath;$record | Add-Member -NotePropertyName shellLayout -NotePropertyValue $badLayout -Force
    AtomicJson $earlyRecordPath $record
    RefusesUnchanged {ReadGeneration $early.current} 'Invalid target layout refuses at generation read before state publication'
    RefusesUnchanged {PublishShell $early} 'Invalid explicit layout refuses before changing shell'
    [IO.File]::WriteAllBytes($earlyRecordPath,$earlyRecordBytes)
  }
  $linkPath=Join-Path $neutral 'OpenNav X.lnk';$saved=[IO.File]::ReadAllBytes($linkPath)
  $shell=New-Object -ComObject WScript.Shell
  foreach($mutation in @('foreign-target','extra-arguments','foreign-directory','unknown-generation')){
    $link=$shell.CreateShortcut($linkPath)
    switch($mutation){
      'foreign-target' {$link.TargetPath=Join-Path $Fixture 'foreign.exe'}
      'extra-arguments' {$link.Arguments='--xnav --portable'}
      'foreign-directory' {$link.WorkingDirectory=$Fixture}
      'unknown-generation' {$link.TargetPath=Join-Path (Generation ('c'*32)) 'app/opencpn.exe'}
    };$link.Save()
    RefusesUnchanged {PublishShell $old} ('Publication refuses '+$mutation)
    RefusesUnchanged {RemoveShell} ('Uninstall refuses '+$mutation)
    [IO.File]::WriteAllBytes($linkPath,$saved)
  }
  $null=[IO.Directory]::CreateDirectory($legacy)
  $foreign=Join-Path $legacy 'unrelated.txt';[IO.File]::WriteAllText($foreign,'preserve me',$Utf8)
  RefusesUnchanged {PublishShell $next} 'Unknown old-group entry refused before touching new group'
  RefusesUnchanged {RemoveShell} 'Unknown old-group entry prevents partial uninstall'
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Update';before=$old;after=$next}
  RefusesUnchanged {Recover} 'Recovery preserves unknown entries and its journal for inspection'
  Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
  Remove-Item -LiteralPath $foreign;Remove-Item -LiteralPath $legacy
  $ownerPath=Join-Path $Root 'owner.json';$savedOwner=[IO.File]::ReadAllBytes($ownerPath)
  AtomicJson $ownerPath @{owner='foreign'}
  RefusesUnchanged {PublishShell $next} 'Root ownership required even when filenames match'
  [IO.File]::WriteAllBytes($ownerPath,$savedOwner)
  $recordPath=Join-Path (Generation $next.current) 'ownership.json';$savedRecord=[IO.File]::ReadAllBytes($recordPath)
  $record=ReadJson $recordPath;$record.managedFiles+=@($record.managedFiles[0]);AtomicJson $recordPath $record
  RefusesUnchanged {PublishShell $old} 'Ambiguous target ownership refused'
  [IO.File]::WriteAllBytes($recordPath,$savedRecord)
  $junction=$legacy;$null=New-Item -ItemType Junction -Path $junction -Value $neutral
  $failed=$false;try{AssertShellOwnership}catch{$failed=$true}
  Check $failed 'Redirected old shortcut group refused'
  [IO.Directory]::Delete($junction);$junction=$null
  # Broken managed app files are intentionally repairable without weakening
  # shortcut target/argument/ownership checks.
  Remove-Item -LiteralPath (Join-Path (Generation $next.current) 'app/opencpn.exe')
  AssertShellOwnership;Check $true 'Missing owned binary does not prevent repair of its exact shortcut'
  RemoveShell
  Check (-not (Test-Path -LiteralPath $Registry) -and @(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'Both historical groups and test registration are removed'
  [IO.File]::WriteAllText((Join-Path (Generation $next.current) 'app/opencpn.exe'), 'inert target '+$next.current, $Utf8)
  $nextAppRecord=@((ReadGeneration $next.current).managedFiles | Where-Object {$_.path -ceq 'app/opencpn.exe'})
  Check ($nextAppRecord.Count -eq 1 -and
    (Hash (Join-Path (Generation $next.current) 'app/opencpn.exe')) -ceq $nextAppRecord[0].sha256) 'Neutral predecessor fixture is restored before SKAGER migration checks'
  # New installs, updates and repairs publish only the SKAGER group. The
  # historical generations are unmodified and rollback restores their own
  # group before any SKAGER links disappear.
  PublishShell $skager;CheckGroup $skager 'SKAGER clean install'
  Check ((Get-ItemProperty -LiteralPath $Registry).DisplayName -ceq 'SKAGER') 'New installed-app label is SKAGER'
  RemoveShell;Check (@(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'SKAGER clean uninstall removes its verified group'
  PublishShell $next;CheckGroup $next 'neutral predecessor'
  AtomicJson (Join-Path $Root 'state.json') $skager
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Update';before=$next;after=$skager}
  $FailurePoint='after-shortcuts';$failed=$false
  try { PublishShell $skager } catch { $failed=$_.Exception.Message -eq 'Injected interruption at after-shortcuts' }
  $FailurePoint=''
  Check ($failed -and (Test-Path (ShortcutGroup (ReadGeneration $next.current))) -and
    (Test-Path (ShortcutGroup (ReadGeneration $skager.current)))) 'Interrupted neutral to SKAGER migration keeps both complete groups'
  Recover;CheckGroup $skager 'SKAGER recovered update'
  Check (-not (Test-Path (Join-Path $Root 'transaction.json'))) 'SKAGER recovery clears journal after old group removal'
  $skager.shortcutModes=@('xnav');PublishShell $skager;CheckGroup $skager 'SKAGER repair with optional links omitted'
  Check (-not (Test-Path (Join-Path (ShortcutGroup (ReadGeneration $skager.current)) 'OpenCPN Legacy.lnk'))) 'SKAGER repair preserves shortcut selection'
  $skager.shortcutModes=@('xnav','legacy','safe');PublishShell $skager;CheckGroup $skager 'SKAGER repair restores optional links'
  AtomicJson (Join-Path $Root 'state.json') $early
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$skager;after=$early}
  $FailurePoint='after-shortcuts';$failed=$false
  try { PublishShell $early } catch { $failed=$_.Exception.Message -eq 'Injected interruption at after-shortcuts' }
  $FailurePoint=''
  Check ($failed -and -not (Test-Path (ShortcutGroup (ReadGeneration $skager.current))) -and
    (Test-Path (ShortcutGroup (ReadGeneration $early.current)))) 'Interrupted rollback removes SKAGER before exposing immutable historical maintenance'
  Recover;CheckGroup $early 'historical rollback from SKAGER'
  PublishShell $skager;CheckGroup $skager 'SKAGER after historical update'
  AtomicJson (Join-Path $Root 'state.json') $next
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$skager;after=$next}
  $FailurePoint='after-shortcuts';$failed=$false
  try { PublishShell $next } catch { $failed=$_.Exception.Message -eq 'Injected interruption at after-shortcuts' }
  $FailurePoint=''
  Check ($failed -and -not (Test-Path (ShortcutGroup (ReadGeneration $skager.current))) -and
    (Test-Path (ShortcutGroup (ReadGeneration $next.current)))) 'Interrupted neutral rollback removes SKAGER before exposing neutral maintenance'
  Recover;CheckGroup $next 'neutral rollback from SKAGER'
  PublishShell $skager;CheckGroup $skager 'SKAGER after neutral update'
  $skagerGroup=ShortcutGroup (ReadGeneration $skager.current)
  $skagerLink=Join-Path $skagerGroup 'Skager.lnk';$savedSkager=[IO.File]::ReadAllBytes($skagerLink)
  $shell=New-Object -ComObject WScript.Shell
  foreach($mutation in @('foreign-target','extra-arguments','foreign-directory','unknown-generation')) {
    $link=$shell.CreateShortcut($skagerLink)
    switch($mutation) {
      'foreign-target' {$link.TargetPath=Join-Path $Fixture 'foreign.exe'}
      'extra-arguments' {$link.Arguments='--xnav --portable'}
      'foreign-directory' {$link.WorkingDirectory=$Fixture}
      'unknown-generation' {$link.TargetPath=Join-Path (Generation ('c'*32)) 'app/opencpn.exe'}
    };$link.Save()
    RefusesUnchanged {PublishShell $old} ('SKAGER rollback refuses '+$mutation)
    RefusesUnchanged {RemoveShell} ('SKAGER uninstall refuses '+$mutation)
    [IO.File]::WriteAllBytes($skagerLink,$savedSkager)
  }
  $wrongName=Join-Path $skagerGroup 'OpenNav X.lnk';Copy-Item -LiteralPath $skagerLink -Destination $wrongName
  RefusesUnchanged {PublishShell $next} 'Old label in SKAGER group refused before rollback changes'
  Remove-Item -LiteralPath $wrongName
  $foreign=Join-Path $skagerGroup 'unrelated.txt';[IO.File]::WriteAllText($foreign,'preserve me',$Utf8)
  RefusesUnchanged {PublishShell $old} 'Unknown SKAGER group entry refuses rollback without partial loss'
  RefusesUnchanged {RemoveShell} 'Unknown SKAGER group entry refuses uninstall without partial loss'
  Remove-Item -LiteralPath $foreign
  $legacyGroup=ShortcutGroup (ReadGeneration $old.current)
  $null=[IO.Directory]::CreateDirectory($legacyGroup)
  $wrongName=Join-Path $legacyGroup 'Skager.lnk';Copy-Item -LiteralPath $skagerLink -Destination $wrongName
  RefusesUnchanged {PublishShell $skager} 'SKAGER label in historical group refused before repair changes'
  Remove-Item -LiteralPath $wrongName;Remove-Item -LiteralPath $legacyGroup
  $junction=Join-Path $Programs 'OpenNav X';$null=New-Item -ItemType Junction -Path $junction -Value $skagerGroup
  RefusesUnchanged {PublishShell $skager} 'Redirected neutral group refuses SKAGER publication'
  [IO.Directory]::Delete($junction);$junction=$null
  AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Rollback';before=$skager;after=$early}
  RemoveShortcutGroup $skagerGroup
  AtomicJson (Join-Path $Root 'state.json') $early
  Check (-not (Test-Path -LiteralPath $skagerGroup) -and -not (Test-Path -LiteralPath $legacyGroup)) 'Crash after old-state commit exposes neither SKAGER nor old maintenance before recovery'
  # Execute the actual published early Beta 2 engine functions, not a
  # reconstructed model. This fixture is the unmodified Lifecycle.ps1 from
  # 8e780edc34f68abd693a5d5f6aecdb3ba05a75c4, pinned byte-for-byte.
  $oldSource=Join-Path $PSScriptRoot 'fixtures/early-beta2-Lifecycle.ps1'
  Check ((Get-FileHash -LiteralPath $oldSource -Algorithm SHA256).Hash.ToLowerInvariant() -ceq
    'e22c17802a3d201c28c1007ecc12cb9bb980ae1aa49396e99b641d6157ae3b9f') 'Published historical lifecycle source is byte-identical to pinned commit'
  $oldErrors=$null
  $oldAst=[Management.Automation.Language.Parser]::ParseFile($oldSource,[ref]$null,[ref]$oldErrors)
  if ($oldErrors) { throw ($oldErrors | Out-String) }
  $Shortcuts=$legacyGroup
  foreach ($name in @('PublishShell','RemoveShell','Recover')) {
    $node=@($oldAst.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq $name},$true))
    if ($node.Count -ne 1) { throw "Expected one exact old engine function: $name" }
    . ([scriptblock]::Create($node[0].Extent.Text))
  }
  Recover;CheckGroup $early 'exact old engine recovers committed historical rollback'
  Check (-not (Test-Path (Join-Path $Root 'transaction.json'))) 'Old engine clears recovered transaction after publishing its original group'
  RemoveShell
  Check (-not (Test-Path -LiteralPath $Registry) -and @(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'Exact old engine uninstall cannot orphan SKAGER shortcuts'
  Write-Host "$Checks native shortcut migration checks passed in PowerShell $($PSVersionTable.PSVersion), $([IntPtr]::Size*8)-bit host."
} finally {
  if($junction -and (Test-Path -LiteralPath $junction)){[IO.Directory]::Delete($junction)}
  if(Test-Path -LiteralPath $Registry){Remove-Item -LiteralPath $Registry -Recurse -Force}
  if(Test-Path -LiteralPath $Fixture){Remove-Item -LiteralPath $Fixture -Recurse -Force}
}
