# Only temporary directory/INI fixtures and inert PowerShell children. No boat.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if (-not $native -and -not $PortableContracts) {throw 'Native Windows or explicit portable compile/refusal check required.'}
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true') {throw 'Use disposable CI or explicit isolated temporary fixtures.'}
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
if (-not ('OpenNavX.CommissioningProfileLock' -as [type])) {Add-Type -Path (Join-Path $PSScriptRoot 'CommissioningProfileLock.cs')}
$checks=New-Object 'Collections.Generic.List[string]'
function Require($Value,[string]$Label) {if (-not $Value) {throw $Label};$checks.Add($Label)}
function Refuse([scriptblock]$Action,[string]$Label) {
  $denied=$false;try {$null=& $Action} catch {$denied=$true}
  Require $denied $Label
}
if (-not $native) {
  Refuse {Open-CommissioningProfileLock 'C:\inert\profile'} 'Native operation refuses on non-Windows without touching a profile'
  [pscustomobject]@{status='passed';checks=$checks.ToArray();native=$false;scope='Exact C# compiled; native mutex/path fixtures not executed.'}|ConvertTo-Json
  return
}
$root=Join-Path ([IO.Path]::GetTempPath()) ('SKAGER profile exclusion '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$profile=Join-Path $root 'profile';$other=Join-Path $root 'other'
$subst=Join-Path ([Environment]::GetFolderPath('System')) 'subst.exe'
$null=New-Item -ItemType Directory -Path $profile,$other
$ini=Join-Path $profile 'opencpn.ini';[IO.File]::WriteAllText($ini,'inert fixture; no connections')
$bytes=[IO.File]::ReadAllBytes($ini);$acl=(Get-Acl -LiteralPath $profile).Sddl
$childScript=Join-Path $root 'child.ps1'
@'
param([string]$Source,[string]$Profile,[string]$Result,[string]$Ready,[string]$ExitGate)
$ErrorActionPreference='Stop'
Add-Type -Path $Source
try {
  $owner=[OpenNavX.CommissioningProfileLock]::Acquire($Profile)
  if ($Ready) {
    [IO.File]::WriteAllText($Ready,$owner.Identity)
    $until=[DateTime]::UtcNow.AddSeconds(20)
    while(-not [IO.File]::Exists($ExitGate)) {
      if ([DateTime]::UtcNow -ge $until) {throw 'Inert fixture release gate expired.'}
      Start-Sleep -Milliseconds 25
    }
    # Intentional owner loss fixture; only a temporary fixture mutex is held.
    exit 0
  }
  $owner.Dispose();[IO.File]::WriteAllText($Result,'acquired');exit 0
} catch {
  [IO.File]::WriteAllText($Result,$_.Exception.ToString());exit 17
}
'@|Set-Content -LiteralPath $childScript -Encoding UTF8
function Start-InertChild([string]$Path,[string]$Label,[switch]$Abandon) {
  $record=Join-Path $root ($Label+'.txt');$ready=Join-Path $root ($Label+'-ready.txt');$exitGate=Join-Path $root ($Label+'-exit.txt')
  $info=New-Object Diagnostics.ProcessStartInfo
  $info.FileName=(Get-Process -Id $PID).Path
  $source=Join-Path $PSScriptRoot 'CommissioningProfileLock.cs'
  $info.Arguments='-NoProfile -NonInteractive -File "'+$childScript+'" -Source "'+$source+'" -Profile "'+$Path+'" -Result "'+$record+'"'
  if($Abandon){$info.Arguments+=' -Ready "'+$ready+'" -ExitGate "'+$exitGate+'"'}
  $info.UseShellExecute=$false;$info.WorkingDirectory=$root
  $process=[Diagnostics.Process]::Start($info)
  $null=$process.get_Handle() # Retain the original inert child's native handle.
  return [pscustomobject]@{process=$process;result=$record;ready=$ready;exitGate=$exitGate}
}
function Close-InertChild($Child) {
  # Test-only cleanup of the original child we created; never a product process.
  try {
    if(-not $Child.process.HasExited) {
      $Child.process.Kill()
      if(-not $Child.process.WaitForExit(5000)) {throw 'Original inert fixture child did not exit; retain fixture files.'}
    }
  } finally {$Child.process.Dispose()}
}
function Denied-InertChild([string]$Path,[string]$Label) {
  $child=Start-InertChild $Path $Label
  try {
    Require ($child.process.WaitForExit(25000)) ($Label+': child terminates within bound')
    Require ($child.process.ExitCode -eq 17 -and [IO.File]::ReadAllText($child.result).Contains('Another commissioning operation owns this physical profile')) ($Label+': independent process denied by physical profile exclusion')
  } finally {Close-InertChild $child}
}
$owner=$null;$second=$null;$observer=$null;$abandoned=$null;$drive=$null;$junction=$null
try {
  $owner=Open-CommissioningProfileLock $profile;$identity=$owner.Identity
  Refuse {Open-CommissioningProfileLock $profile} 'Same-process recursion cannot bypass exclusion'
  Denied-InertChild $profile 'same-profile-other-process'
  $second=Open-CommissioningProfileLock $other
  Require ($second.Identity -cne $identity) 'Different physical profile can be owned independently'
  $second.Dispose();$second=$null
  Refuse {[IO.Directory]::Move($profile,$profile+'-moved')} 'Held physical profile directory cannot be replaced'
  $candidate=Join-Path $profile 'candidate.ini';[IO.File]::WriteAllBytes($candidate,$bytes)
  $backup=Join-Path $profile 'opencpn-before-replace.ini'
  [IO.File]::Replace($candidate,$ini,$backup)
  Require ([Convert]::ToBase64String([IO.File]::ReadAllBytes($backup)) -ceq [Convert]::ToBase64String($bytes)) 'Held replacement retains exact original backup bytes'
  Denied-InertChild $profile 'replaced-ini'
  Require ((Get-Acl -LiteralPath $profile).Sddl -ceq $acl) 'Exclusion changes no profile directory permissions'
  $drive=@(90..68|ForEach-Object {[string][char]$_+':'}|Where-Object {-not [IO.Directory]::Exists($_+'\')}|Select-Object -First 1)[0]
  if (-not $drive) {throw 'Native SUBST fixture requires a free drive letter.'}
  & $subst $drive $root
  if($LASTEXITCODE -ne 0){throw 'Native fixture SUBST failed.'}
  Denied-InertChild ($drive+'\profile') 'alias-other-process'
  $owner.Dispose();$owner=$null
  $owner=Open-CommissioningProfileLock ($drive+'\profile')
  Require ($owner.Identity -ceq $identity) 'DOS alias and physical spelling select the same mutex identity'
  $owner.Dispose();$owner=$null
  & $subst $drive /d
  if($LASTEXITCODE -ne 0){throw 'Native fixture SUBST cleanup failed.'};$drive=$null
  $junction=Join-Path $root 'redirected'
  $null=New-Item -ItemType Junction -Path $junction -Target $profile
  Refuse {Open-CommissioningProfileLock $junction} 'Reparse profile ancestry is refused'
  [IO.Directory]::Delete($junction)
  Require ((-not [IO.Directory]::Exists($junction)) -and
           [IO.Directory]::Exists($profile) -and
           [Convert]::ToBase64String([IO.File]::ReadAllBytes($ini)) -ceq [Convert]::ToBase64String($bytes)) 'Removing fixture junction leaves original profile intact'
  $junction=$null
  foreach($path in @('relative',('\\server\share\profile'),($profile+'\..\profile'),($profile+':stream'),($profile+'.'),(Join-Path $root 'missing'))) {
    Refuse {Open-CommissioningProfileLock $path} 'Missing, remote or ambiguous profile directory is refused'
  }
  $abandoned=Start-InertChild $profile 'abandoned' -Abandon
  $deadline=[DateTime]::UtcNow.AddSeconds(20)
  while(-not [IO.File]::Exists($abandoned.ready)) {
    if($abandoned.process.HasExited -or [DateTime]::UtcNow -ge $deadline){throw 'Abandoned fixture failed before ready.'};Start-Sleep -Milliseconds 25
  }
  $observer=[Threading.Mutex]::OpenExisting([IO.File]::ReadAllText($abandoned.ready))
  [IO.File]::WriteAllText($abandoned.exitGate,'exit without Dispose')
  Require ($abandoned.process.WaitForExit(10000) -and $abandoned.process.ExitCode -eq 0) 'Inert owner-loss child exits normally'
  $denied=$false
  try {$owner=Open-CommissioningProfileLock $profile} catch {$denied=$_.Exception.ToString().Contains('Abandoned profile commissioning')}
  Require $denied 'Abandoned native ownership is refused rather than accepted as success'
  Require ([Convert]::ToBase64String([IO.File]::ReadAllBytes($ini)) -ceq [Convert]::ToBase64String($bytes)) 'Original fixture profile bytes remain exact'
  [pscustomobject]@{status='passed';checks=$checks.ToArray();native=$true;architecture=$env:PROCESSOR_ARCHITECTURE;scope='Temporary directory/INI and inert child fixtures; no application/boat/hardware.'}|ConvertTo-Json -Depth 5
} finally {
  if($owner){$owner.Dispose()};if($second){$second.Dispose()};if($observer){$observer.Dispose()}
  if($abandoned){
    if(-not $abandoned.process.HasExited){[IO.File]::WriteAllText($abandoned.exitGate,'cleanup');$null=$abandoned.process.WaitForExit(25000)}
    Close-InertChild $abandoned
  }
  if($drive){& $subst $drive /d}
  if($junction -and [IO.Directory]::Exists($junction)){[IO.Directory]::Delete($junction)}
  Remove-Item -LiteralPath $root -Recurse -Force
}
