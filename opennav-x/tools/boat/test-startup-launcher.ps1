# Inert identities/temporary files. Native observation launches only an inert
# PowerShell child; no app, installed profile, certificate, network or boat input.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
if(-not $native -and -not $PortableContracts){throw 'Use explicit portable contracts off Windows.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true'){throw 'Disposable native CI or explicit isolated local tests required.'}
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'StartupLauncher.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Passed,[string]$Message){if(-not $Passed){throw $Message};$script:checks.Add($Message)}
function Reject([scriptblock]$Call,[string]$Message){$failed=$false;try{$null=& $Call}catch{$failed=$true};Check $failed $Message}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 10|ConvertFrom-Json)}
$root=Join-Path ([IO.Path]::GetTempPath()) ('skager-startup-launch-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$sourceFiles=@('StartupLauncher.ps1','test-startup-launcher.ps1')|ForEach-Object {[pscustomobject]@{path=('tools/boat/'+$_);sha256=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $_) -Algorithm SHA256).Hash.ToLowerInvariant()}}
if(-not $native){
 function Assert-LocalPath([string]$Path){
  $full=[IO.Path]::GetFullPath($Path)
  if($full -ne $root -and -not $full.StartsWith($root+'/',[StringComparison]::Ordinal)){throw 'Outside disposable fixture.'}
  $walk=$full
  while($walk -and $walk -ne $root){if((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Redirected fixture path.'};$walk=[IO.Path]::GetDirectoryName($walk)}
  return $full
 }
}
try {
 foreach($name in @('StartupLauncher.ps1','Common.ps1','InteractiveJob.ps1','run-mode.ps1','run-xnav.ps1','smoke-test.ps1')){
  $errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$null,[ref]$errors)
  Check (-not $errors) ('Parse '+$name)
 }
 $id='a'*32;$directory=Join-Path (Join-Path $root 'generations') $id
 $null=New-Item -ItemType Directory -Path (Join-Path $directory 'app') -Force
 [IO.File]::WriteAllText((Join-Path $root 'state.json'),'inert state')
 [IO.File]::WriteAllText((Join-Path $directory 'ownership.json'),'inert ownership')
 $files=@('app/opencpn.exe','app/skager-start.exe','UpdateTransaction.ps1','UpdateSupervisor.ps1','Lifecycle.ps1')|ForEach-Object {
  $path=Join-Path $directory $_;[IO.File]::WriteAllText($path,('INERT; NEVER EXECUTE: '+$_));[pscustomobject]@{path=$_;sha256=(Get-Digest $path)}
 }
 $installed=[pscustomobject]@{root=$root;generation=$directory;executable=(Join-Path $directory 'app/opencpn.exe');state=[pscustomobject]@{current=$id};ownership=[pscustomobject]@{updateStartupHealth=1;commit=('a'*40);packageSha256=('b'*64);managedFiles=@($files)}}
 $before=@(Get-ChildItem -LiteralPath $root -Recurse -File | ForEach-Object {[pscustomobject]@{path=$_.FullName;sha256=(Get-Digest $_.FullName)}})
 $context=Get-StartupLauncherContext $installed
 Check ($context.identity.generation -ceq $id -and $context.files.Count -eq 5) 'Exact health1 generation and five owned launch inputs resolve'
 foreach($health in @($false,0,2,'1')){$bad=Clone $installed;$bad.ownership.updateStartupHealth=$health;Reject {Get-StartupLauncherContext $bad} 'Unsupported/noninteger health contract refuses'}
 foreach($name in @('update-pending.json',('generations/'+$id+'/app/update-trust.json'))){
  $path=Join-Path $root $name;[IO.File]::WriteAllText($path,'even invalid records must refuse')
  try{Reject {Get-StartupLauncherContext $installed} ('Existing '+$name+' refuses without parsing or mutation')}finally{Remove-Item -LiteralPath $path}
 }
 $bad=Clone $installed;$bad.ownership.managedFiles+=@($bad.ownership.managedFiles[0]);Reject {Get-StartupLauncherContext $bad} 'Duplicate executable ownership refuses'
 $bad=Clone $installed;$bad.ownership.commit='bad';Reject {Get-StartupLauncherContext $bad} 'Unbound commit refuses'
 $path=$context.files['UpdateSupervisor.ps1'].path;$original=[IO.File]::ReadAllBytes($path)
 try{[IO.File]::AppendAllText($path,'changed');Reject {Get-StartupLauncherContext $installed} 'Changed supervisor refuses'}finally{[IO.File]::WriteAllBytes($path,$original)}
 foreach($entry in $before){Check ((Get-Digest $entry.path) -ceq $entry.sha256) 'Refused context checks preserve exact fixture bytes'}
 $launch=[pscustomobject]@{pid=10;startedTicks=100L;exitTicks=180L}
 $bridge=[pscustomobject]@{pid=11;parentPid=10;startedTicks=110L;exitTicks=190L}
 $app=[pscustomobject]@{pid=12;parentPid=11;startedTicks=120L;sid='S-1-5-21-1';session=2;image='C:\owned\opencpn.exe';sha256=('a'*64)}
 Assert-StartupObservedIdentity $app $app.image $app.sha256 $app.sid 2 100
 Assert-StartupParentChain $app $launch $bridge $false
 Check $true 'Exact retained bootstrap parent chain and process identity accepts'
 Reject {Assert-StartupParentChain $app $launch $null $false} 'Missed intermediate ancestry refuses'
 foreach($field in @('parentPid','startedTicks')){$bad=Clone $app;if($field -eq 'parentPid'){$bad.parentPid=999}else{$bad.startedTicks=200L};Reject {Assert-StartupParentChain $bad $launch $bridge $false} ('Wrong child '+$field+' refuses')}
 $bad=Clone $bridge;$bad.parentPid=999;Reject {Assert-StartupParentChain $app $launch $bad $false} 'Unrelated supervisor refuses'
 $direct=Clone $app;$direct.parentPid=10;Assert-StartupParentChain $direct $launch $null $true
 Check $true 'Known-good no-config child binds directly to retained launcher'
 Reject {Assert-StartupParentChain $direct $launch $bridge $true} 'Unexpected intermediate refuses on direct path'
 foreach($field in @('pid','startedTicks','sid','session','image','sha256')){
  $bad=Clone $app
  $bad.$field=switch($field){'pid'{0};'startedTicks'{99L};'sid'{'S-1-5-21-2'};'session'{3};'image'{'C:\other\opencpn.exe'};'sha256'{('b'*64)}}
  Reject {Assert-StartupObservedIdentity $bad $app.image $app.sha256 $app.sid 2 100} ('Wrong actual process '+$field+' refuses')
 }
 Assert-StartupCommandArguments @('C:\owned\opencpn.exe','--xnav') 'C:\owned\opencpn.exe' '' '' $false
 Reject {Assert-StartupCommandArguments @('C:\owned\opencpn.exe','--xnav','--portable') 'C:\owned\opencpn.exe' '' '' $false} 'Profile/mode additions refuse'
 $system='C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe';$wow='C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe'
 $arguments=@($system,'-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File','C:/owned/UpdateSupervisor.ps1','-InstallationRoot','C:/owned','-Action','QualifyCurrent','-BootstrapExpectation',($id+':absent'))
 Assert-StartupCommandArguments $arguments $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')
 Check $true 'WOW64 command spelling accepts only independently verified system engine paths'
 $changed=@($arguments);$changed[10]='LaunchPending';Reject {Assert-StartupCommandArguments $changed $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')} 'Update/recovery action refuses in bootstrap-only helper'
 $changed=@($arguments);$changed[6]='C:\unowned.ps1';Reject {Assert-StartupCommandArguments $changed $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')} 'Unowned supervisor command refuses'
 $changed=@($arguments);$changed[0]='C:\other\powershell.exe';Reject {Assert-StartupCommandArguments $changed $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')} 'Arbitrary interpreter path refuses'
 $changed=@($arguments);$changed[12]=('b'*32)+':absent';Reject {Assert-StartupCommandArguments $changed $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')} 'Another bootstrap generation refuses'
 $changed=@($arguments);$changed[12]='';Reject {Assert-StartupCommandArguments $changed $wow 'C:\owned' 'C:\owned\UpdateSupervisor.ps1' $true @($system,$wow) ($id+':absent')} 'Missing private route expectation refuses'
 Initialize-StartupArguments
 Check $true 'Exact Windows argument-parser interop compiles'
 if($native){
  $path=Join-Path $root 'inert.ps1';[IO.File]::WriteAllText($path,'[Threading.Thread]::Sleep(15000)')
  $engine=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe'
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$engine;$start.Arguments='-NoProfile -NonInteractive -File "'+$path+'"';$start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $child=[Diagnostics.Process]::Start($start);$null=$child.get_Handle();$observed=$null;$current=$null
  try {
   $nativeChild=@(Get-CimInstance Win32_Process -Filter ('ProcessId='+$child.Id) -OperationTimeoutSec 3)
   Check ($nativeChild.Count -eq 1) 'Inert native process has unique OS record'
   $current=[Diagnostics.Process]::GetCurrentProcess()
   $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
   $observed=Get-StartupObservedProcess $nativeChild[0] $sid $current.SessionId
   Check ($observed.parentPid -eq $PID -and $observed.pid -eq $child.Id -and $observed.startedTicks -eq $child.StartTime.ToUniversalTime().Ticks -and $observed.sha256 -ceq (Get-Digest $observed.image)) 'Native owner/session/image/hash/creation and parent PID bind retained child handle'
   Check ($observed.arguments.Count -eq 5 -and $observed.arguments[4] -ceq $path) 'Native command parser retains quoted fixture path'
  } finally {
   if($current){$current.Dispose()}
   # Termination is confined to this exact inert test child; production helper
   # never calls Kill, and no OpenCPN/application process is created here.
   if(-not $child.HasExited){$child.Kill();$null=$child.WaitForExit(5000)}
   if($observed){$observed.process.Dispose()};$child.Dispose()
  }
 }
 [pscustomobject]@{schema=1;status='passed';scope='bootstrap/no-config helper contracts only; no boat or installed application';environment=$(if($native){'native-windows-inert-process'}else{'linux-portable-contracts'});nativeObservation=$(if($native){'passed'}else{'pending'});installedBootstrap='pending';signedOffersAndRollback='pending';checks=@($checks);count=$checks.Count;sourceFiles=@($sourceFiles)}|ConvertTo-Json -Depth 5
} finally {if(Test-Path -LiteralPath $root){Remove-Item -LiteralPath $root -Recurse -Force}}
