# Actual WScript.Shell links and lifecycle publication/recovery, confined to a
# unique disposable directory and HKCU test key. Never run the engine entrypoint.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT') { throw 'Disposable Windows CI only.' }
$parseErrors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'),[ref]$null,[ref]$parseErrors)
if ($parseErrors) { throw ($parseErrors | Out-String) }
foreach ($name in @('Log','Hash','PlainPath','RelativePath','ReadJson','AtomicJson','Generation','ReadState','ReadGeneration','ShellGroups','ShortcutGroup','ShortcutSpec','AssertShortcut','AssertShellOwnership','RemoveShortcutGroup','PublishShell','RemoveShell','Failure','Recover')) {
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
function FixtureGeneration([string]$id,[string]$version) {
  $d=Generation $id;$null=[IO.Directory]::CreateDirectory((Join-Path $d 'app'))
  [IO.File]::WriteAllText((Join-Path $d 'app/opencpn.exe'),'inert target '+$id,$Utf8)
  [IO.File]::WriteAllText((Join-Path $d 'Maintain.exe'),'inert maintainer '+$id,$Utf8)
  AtomicJson (Join-Path $d 'ownership.json') @{owner=$Owner;version=$version;managedFiles=@(
    @{path='app/opencpn.exe';sha256=(Hash (Join-Path $d 'app/opencpn.exe'))},
    @{path='Maintain.exe';sha256=(Hash (Join-Path $d 'Maintain.exe'))})}
  return [pscustomobject]@{owner=$Owner;schema=1;current=$id;previous='';stock=@{path=(Join-Path $Fixture 'stock/opencpn.exe')};shortcutModes=@('xnav','legacy','safe')}
}
function CheckGroup($state,[string]$version) {
  $group=ShortcutGroup $version;$other=@(ShellGroups | Where-Object {$_ -cne $group})[0]
  Check ((Test-Path -LiteralPath $group) -and -not (Test-Path -LiteralPath $other)) ('Only expected group for '+$version)
  $shell=New-Object -ComObject WScript.Shell
  $files=@(Get-ChildItem -LiteralPath $group -File)
  Check ($files.Count -eq (@($state.shortcutModes).Count+1)) 'Only requested shortcuts plus maintenance'
  foreach($f in $files){
    AssertShortcut $f.FullName $shell
    $s=ShortcutSpec $f.Name;$link=$shell.CreateShortcut($f.FullName)
    Check ([string]::Equals($link.TargetPath,(RelativePath (Generation $state.current) $s.target),[StringComparison]::OrdinalIgnoreCase)) ('Current generation '+$f.Name)
  }
  Check ((Get-ItemProperty -LiteralPath $Registry).ModifyPath -ceq ('"'+(Join-Path (Generation $state.current) 'Maintain.exe')+'"')) 'Registry maintenance points to the exact selected generation'
}
try {
  $null=[IO.Directory]::CreateDirectory($Root)
  $null=[IO.Directory]::CreateDirectory($Programs)
  AtomicJson (Join-Path $Root 'owner.json') @{owner=$Owner}
  $old=FixtureGeneration ('a'*32) '0.3.0-beta1'
  $next=FixtureGeneration ('b'*32) '0.4.0-beta2'
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
  Check ($failed -and (Test-Path (ShortcutGroup '0.3.0-beta1')) ) 'Interrupted migration retains the prior usable group'
  Check (Test-Path (ShortcutGroup '0.4.0-beta2')) 'Complete new group exists before old links are removed'
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
  $neutral=ShortcutGroup '0.4.0-beta2';$legacy=ShortcutGroup '0.3.0-beta1'
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
  Check (-not (Test-Path -LiteralPath $Registry) -and @(Get-ChildItem -LiteralPath $Programs -Force).Count -eq 0) 'Both owned groups and test registration are removed'
  Write-Host "$Checks native shortcut migration checks passed in PowerShell $($PSVersionTable.PSVersion), $([IntPtr]::Size*8)-bit host."
} finally {
  if($junction -and (Test-Path -LiteralPath $junction)){[IO.Directory]::Delete($junction)}
  if(Test-Path -LiteralPath $Registry){Remove-Item -LiteralPath $Registry -Recurse -Force}
  if(Test-Path -LiteralPath $Fixture){Remove-Item -LiteralPath $Fixture -Recurse -Force}
}
